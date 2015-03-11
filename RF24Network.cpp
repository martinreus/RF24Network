/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "RF24Network_config.h"
#include "RF24.h"
#include "RF24Network.h"
#include "RF24NetworkStructs.h"

#if defined (ENABLE_SLEEP_MODE)  && !defined (__ARDUINO_X86__)
	#include <avr/sleep.h>
	#include <avr/power.h>
	volatile byte sleep_cycles_remaining;
#endif

uint16_t RF24NetworkHeader::next_id = 1;

uint64_t pipe_address( uint16_t node, uint8_t pipe );
bool is_valid_address( uint16_t node );

/******************************************************************/
#if !defined (DUAL_HEAD_RADIO)
RF24Network::RF24Network( RF24& _radio ): radio(_radio), next_frame(frame_queue), messageBufferUsedPositions(0), nextHeartbeatSend(0)
{
}
#else
RF24Network::RF24Network( RF24& _radio, RF24& _radio1 ): radio(_radio), radio1(_radio1), next_frame(frame_queue)
{
}
#endif
/******************************************************************/

void RF24Network::begin(uint8_t _channel, uint16_t _node_address )
{
  if (! is_valid_address(_node_address) )
    return;

  node_address = _node_address;

  if ( ! radio.isValid() ){
    return;
  }

  // Set up the radio the way we want it to look
  radio.setChannel(_channel);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  // Use different retry periods to reduce data collisions

  uint8_t retryVar = (node_address % 7) + 5;
  radio.setRetries(retryVar, 5); // decreased from 15 to 5
  txTimeout = retryVar * 17;

#if defined (DUAL_HEAD_RADIO)
  radio1.setChannel(_channel);
  radio1.setDataRate(RF24_1MBPS);
  radio1.setCRCLength(RF24_CRC_16);

#endif

  // Setup our address helper cache
  setup_address();

  // Open up all listening pipes
  int i = 6;
  while (i--)
    radio.openReadingPipe(i,pipe_address(_node_address,i));
  radio.startListening();

}

/******************************************************************/

void RF24Network::update(void) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: update\n\r"),millis()));
    readMessages();
    sendConnectionAttempts();
    sendHeartbeatRequests();
    sendBufferizedMessages();
}

/******************************************************************/

void RF24Network::readMessages(void) {
     // if there is data ready
  uint8_t pipe_num;
  while ( radio.isValid() && radio.available(&pipe_num) )
  {
    // Dump the payloads until we've gotten everything

    while (radio.available())
    {
      // Fetch the payload, and see if this was the last one.
      radio.read( frame_buffer, sizeof(frame_buffer) );

      // Read the beginning of the frame as the header
      const RF24NetworkHeader& header = * reinterpret_cast<RF24NetworkHeader*>(frame_buffer);

      IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Received on %u %s\n\r"),millis(),pipe_num,header.toString()));
      IF_SERIAL_DEBUG(const uint16_t* i = reinterpret_cast<const uint16_t*>(frame_buffer + sizeof(RF24NetworkHeader));printf_P(PSTR("%lu: NET message %04x\n\r"),millis(),*i));

      // Throw it away if it's not a valid address
      if ( !is_valid_address(header.to_node) ){
		continue;
	  }

        // Is this for us?
        if ( header.to_node == node_address ){
            // Add it to the buffer of frames for us if it is a user message.
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Received packet of type %d\n\r"),millis(),header.type));
            if (header.type < 128) {
                enqueue();
            // if not, treat it as a system message.
            } else {
                treatSystemMessages();
            }
        }else{
            // Relay it
            write(header.to_node);
        }
      // NOT NEEDED anymore.  Now all reading pipes are open to start.
#if 0
      // If this was for us, from one of our children, but on our listening
      // pipe, it could mean that we are not listening to them.  If so, open up
      // and listen to their talking pipe

      if ( header.to_node == node_address && pipe_num == 0 && is_descendant(header.from_node) )
      {
	uint8_t pipe = pipe_to_descendant(header.from_node);
	radio.openReadingPipe(pipe,pipe_address(node_address,pipe));

	// Also need to open pipe 1 so the system can get the full 5-byte address of the pipe.
	radio.openReadingPipe(1,pipe_address(node_address,1));
      }
#endif
    }
  }
}

/******************************************************************/

void RF24Network::sendBufferizedMessages(void) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendBufM; \n\r"),millis()));
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendBufMsgSize: %d; \n\r"),millis(), sizeof(sendMessageBuffer)));

    Message * message = &sendMessageBuffer[0];
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: pointer addresses: array: %d; first: %d \n\r"),millis(), sendMessageBuffer, message));

    for (int bufferPos = 0 ; bufferPos < messageBufferUsedPositions ; bufferPos++ ) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Buff pos %d\n\r"),millis(), bufferPos));
        Message * message = &sendMessageBuffer[bufferPos];

        // address information
        RF24NetworkHeader * header = reinterpret_cast<RF24NetworkHeader *>(message->payload);
        Connection * connection = getConnection(header->to_node);

        if (message->retries < MSG_SENDING_MAX_RETRIES) {
            if (connection == NULL) {
                if (message->messageCallback != NULL) {
                    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: No Conn, invoking handler\n\r"),millis()));
                    MessageStatus status(MSG_STATUS_DISCONNECTED, header);
                    message->messageCallback(&status);
                }
                continue;
            }
            //send!
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Sending message!\n\r"),millis()));
            write(*header, message->payload + sizeof(RF24NetworkHeader), FRAME_SIZE - sizeof(RF24NetworkHeader));
            message->retries++;
            message->retryTimeout = millis() + connection->delayBetweenMsg;
        } else {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: maxRetries for message %d\n\r"),millis(),header->id));
            purgeBufferedMessage(bufferPos);

            //we go back one position if the last one was excluded from the array due to retries overflow
            bufferPos--;
            //let who tried to send this message now that it was not delivered.
            if (message->messageCallback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: invoking handler\n\r"),millis()));
                MessageStatus status(MSG_STATUS_TIMEOUT, header);
                message->messageCallback(&status);
            }
        }

    }
}


/******************************************************************/

void RF24Network::prepareMessage(Message * message) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: prepareMsg\n\r"),millis()));
    message->retryTimeout = 0;
    message->retries = 0;
    message->messageCallback = NULL; 
    // clean only payload of the message, which also contains the header that will be sent OTA.
    memset(message->payload + sizeof(RF24NetworkHeader), 0, sizeof(message->payload) - sizeof(RF24NetworkHeader));

    IF_SERIAL_DEBUG(printMessage(message));
}

/******************************************************************/

Message * RF24Network::getNextFreeMessageBufferPosition() {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: nextFreeMsg\n\r"),millis()));

    if (messageBufferUsedPositions >= BUFFER_SIZE) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: fullBuffer\n\r"),millis()));
        return NULL;
    }
 
    Message * bufferPos = &sendMessageBuffer[messageBufferUsedPositions];
    prepareMessage(bufferPos);
    IF_SERIAL_DEBUG(printMessage(bufferPos));
    messageBufferUsedPositions++;

    return bufferPos;
}

/******************************************************************/

void RF24Network::purgeBufferedMessage(int bufferPos) {
    // only purge valid buffer positions
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: purgeBufPos %d; \n\r"),millis(), bufferPos));
    if (bufferPos >= messageBufferUsedPositions || bufferPos < 0 || messageBufferUsedPositions == 0)
        return;

    Message * message = &sendMessageBuffer[bufferPos];
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: purging\n\r"),millis()));
    IF_SERIAL_DEBUG(printMessage(message));

    // if it is the last position, just update the retries to max.
    if (bufferPos + 1 == messageBufferUsedPositions) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: last position...\n\r"),millis()));
        // ensure that retries are toped.
        message->retries = MSG_SENDING_MAX_RETRIES;
    // if not, we need to shift all messages one position to the left
    } else {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: moving\n\r"),millis()));
        memmove(message, message + sizeof(Message), sizeof(Message) * (bufferPos - messageBufferUsedPositions));

    }

    messageBufferUsedPositions--;
}


/******************************************************************/

void RF24Network::sendConnectionAttempts(void) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SendConnAtt;\n\r"),millis()));
    uint16_t currentTimestamp = millis();
    Connection * connectionPos;

    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        IF_SERIAL_DEBUG(printConnectionProperties(connectionPos));

        if (!connectionPos->connected
                && !connectionPos->dirty
                && connectionPos->connectionRetries < CONN_RETRIES
                && connectionPos->connectionRetryTimestamp < currentTimestamp) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; tN %o\n\r"),millis(),connectionPos->nodeAddress));
            connectionPos->connectionRetries++;
            connectionPos->connectionRetryTimestamp = currentTimestamp + connectionPos->delayBetweenMsg;
            increaseConnectionDelay(connectionPos);
            RF24NetworkHeader header(connectionPos->nodeAddress, MSG_SYN);
            write(header, NULL, 0);
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; tN: %o; dBM: %lu; cRT: %lu; cR: %d;\n\r"),millis(),
                    connectionPos->nodeAddress, connectionPos->delayBetweenMsg, connectionPos->connectionRetryTimestamp, connectionPos->connectionRetries));
        } else if (!connectionPos->connected
                && !connectionPos->dirty
                && connectionPos->connectionRetries >= CONN_RETRIES) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; timeout reached for node: %o; Setting connection dirty.\n\r"),millis(),connectionPos->nodeAddress));
            connectionPos->dirty = true;
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SendConnAtt; timeout reached for node: %o; Setting connection dirty.\n\r"),millis(),connectionPos->nodeAddress));
            if (connectionPos->connCallback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SendConnAtt; Going to invoke connection handler with CONN_TIMEOUT.\n\r"),millis()));
                ConnectionStatus status(CONN_TIMEOUT, connectionPos->nodeAddress);
                connectionPos->connCallback(&status);
            }
        }

    }
}



void RF24Network::increaseConnectionDelay(Connection * connection) {
    if (connection->delayBetweenMsg < (uint16_t) CONN_MAX_MSG_DELAY) {
        connection->delayBetweenMsg = (connection->delayBetweenMsg) * 2;
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: increaseConnDelay; node %o with %lu millis between messages\n\r"),millis(), connection->nodeAddress, connection->delayBetweenMsg));
    }
}

/******************************************************************/

void RF24Network::decreaseConnectionDelay(Connection * connection) {
    if (connection->delayBetweenMsg > (uint16_t) CONN_MIN_MSG_DELAY) {
        connection->delayBetweenMsg = (connection->delayBetweenMsg) / 2;
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: decreaseConnDelay; node %o with %lu millis between messages\n\r"),millis(), connection->nodeAddress, connection->delayBetweenMsg));
    }
}
/******************************************************************/

void RF24Network::treatSystemMessages() {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes \n\r"),millis()));
    // Read the beginning of the frame as the header
    const RF24NetworkHeader & header = * reinterpret_cast<RF24NetworkHeader*>(frame_buffer);
    Connection* connection;

    switch (header.type) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, header type: %i\n\r"),millis(), header.type));
        case MSG_SYN:
            // Try getting an already open connection for this node or try opening a new one.
            connection = getExistingOrAllocateNewConnection(header.from_node);
            if (connection == NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, send MSG_CONN_REFUSED to node %o\n\r"),millis(),header.from_node));
                // No connection is possible.
                RF24NetworkHeader destinationHeader(header.from_node, MSG_CONN_REFUSED);
                write(destinationHeader, NULL, 0);
            } else {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, send MSG_SYN_ACK to node %o\n\r"),millis(),header.from_node));
                // send a SYN_ACK for new connection of already existing connection.
                RF24NetworkHeader destinationHeader(header.from_node, MSG_SYN_ACK); 
                write(destinationHeader, NULL, 0);
            }
            break;
        case MSG_SYN_ACK:
            // Get the connection that originated this SYN_ACK.
            connection = getDanglingConnection(header.from_node);
            if (connection != NULL) { //if there is no connection for this node, simply ignore this message.
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, set connected=true for node %o\n\r"),millis(),header.from_node));
                connection->connected = true;
                if (connection->connCallback != NULL) {
                    ConnectionStatus status(CONN_SUCCESSFUL, connection->nodeAddress);
                    connection->connCallback(&status);
                }
            }
            break;
        case MSG_CONN_REFUSED:
            // Get the connection regarding this MSG_CONN_REFUSED.
            connection = getDanglingConnection(header.from_node);
            if (connection != NULL) { //if there is no connection for this node, simply ignore this message.
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, conn refused for node %o\n\r"),millis(),header.from_node));
                connection->dirty = true;
                if (connection->connCallback != NULL) {
                    ConnectionStatus status(CONN_REFUSED, connection->nodeAddress);
                    connection->connCallback(&status);
                }
            }
            break;
        case MSG_HEARTBEAT_REQ:
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, heartbeat request from node %o\n\r"),millis(),header.from_node));
            // Get the connection that the other node wants to test.
            connection = getConnection(header.from_node);
            if (connection != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, sending heartbeat response to node %o\n\r"),millis(),header.from_node));
                connection->heartbeatRetries = 0;
                RF24NetworkHeader destinationHeader(header.from_node, MSG_HEARTBEAT_ACK); 
                write(destinationHeader, NULL, 0);
            }
            //TODO when not connected, send not connected message...
            break;
        case MSG_HEARTBEAT_ACK:
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, heartbeat ack from node %o\n\r"),millis(),header.from_node));
            // Get the connection for which we sent a heartbeat request.
            connection = getConnection(header.from_node);
            if (connection != NULL) {
                connection->heartbeatRetries = 0;
            }
            break;
    }
}

/******************************************************************/

void RF24Network:: sendHeartbeatRequests(void) {
    long currentMillis = millis();
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; \n\r"),millis()));

    //if it is not time to send heartbeat requests, just return.
    if (currentMillis < nextHeartbeatSend) {
        return;       
    }

    Connection * connectionPos;

    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (!connectionPos->dirty && connectionPos->connected && connectionPos->heartbeatRetries < CONN_MAX_HEARTBEAT_RETRIES) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; sending heartbeat to node %o\n\r"),millis(), connectionPos->nodeAddress));
            RF24NetworkHeader header(connectionPos->nodeAddress, MSG_HEARTBEAT_ACK);
            write(header, NULL, 0);
            connectionPos->heartbeatRetries++;
        } else if (!connectionPos->dirty && connectionPos->connected && connectionPos->heartbeatRetries >= CONN_MAX_HEARTBEAT_RETRIES) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; heartbeat timed out; closing connection to node %o\n\r"),millis(), connectionPos->nodeAddress));
            // We did not receive a heartbeat response, so the other node is either dead or connection was closed on the other side.
            // We just close our side of the connection.
            connectionPos->dirty = true;
            //if there is a connection handler, invoke it with connection timed out.
            if (connectionPos->connCallback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; invoking connection callback with CONN_TIMEOUT\n\r"),millis()));
                ConnectionStatus status(CONN_TIMEOUT, connectionPos->nodeAddress);
                connectionPos->connCallback(&status);
            }
        }
    }

    nextHeartbeatSend = currentMillis + (1000 * CONN_HEARTBEAT_INTERVAL);
}

/******************************************************************/
//TODO send ACK messages....
bool RF24Network::enqueue(void)
{
  bool result = false;

  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Enqueue @%x "),millis(),next_frame-frame_queue));

  // Copy the current frame into the frame queue
  if ( next_frame < frame_queue + sizeof(frame_queue) )
  {
    memcpy(next_frame,frame_buffer, FRAME_SIZE );
    next_frame += FRAME_SIZE;

    result = true;
    IF_SERIAL_DEBUG(printf_P(PSTR("ok\n\r")));
  }
  else
  {
    IF_SERIAL_DEBUG(printf_P(PSTR("failed\n\r")));
  }

  return result;
}

/******************************************************************/

bool RF24Network::available(void)
{
  // Are there frames on the queue for us?
  return (next_frame > frame_queue);
}

/******************************************************************/

uint16_t RF24Network::parent() const
{
  if ( node_address == 0 )
    return -1;
  else
    return parent_node;
}

/******************************************************************/

void RF24Network::peek(RF24NetworkHeader& header)
{
  if ( available() )
  {
    // Copy the next available frame from the queue into the provided buffer
    memcpy(&header,next_frame-FRAME_SIZE,sizeof(RF24NetworkHeader));
  }
}

/******************************************************************/

size_t RF24Network::read(RF24NetworkHeader& header,void* message, size_t maxlen)
{
  size_t bufsize = 0;

  if ( available() )
  {
    // Move the pointer back one in the queue
    next_frame -= FRAME_SIZE;
    uint8_t* frame = next_frame;

    memcpy(&header,frame,sizeof(RF24NetworkHeader));

    if (maxlen > 0)
    {
      // How much buffer size should we actually copy?
      bufsize = min(maxlen,FRAME_SIZE-sizeof(RF24NetworkHeader));

      // Copy the next available frame from the queue into the provided buffer
      memcpy(message,frame+sizeof(RF24NetworkHeader),bufsize);
    }

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Received %s\n\r"),millis(),header.toString()));
  }

  return bufsize;
}

/******************************************************************/

void RF24Network::connect(uint16_t nodeAddress, void (* callback)(ConnectionStatus *)) {
    Connection * connectionPos;

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect;\n\r"),millis()));
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; inside loop; connections: %d; connectionPos: %d;\n\r"),millis(), connections, connectionPos));
        if (connectionPos->connected && !connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            if (callback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; already conn to node %o\n\r"),millis(),nodeAddress));
                ConnectionStatus status(CONN_ALREADY_CONNECTED, nodeAddress);
                callback(&status);
            }
            return;
        }
        if (connectionPos->dirty) {
            IF_SERIAL_DEBUG(printConnectionProperties(connectionPos));
            resetConnection(connectionPos, callback, false, nodeAddress);
            IF_SERIAL_DEBUG(printConnectionProperties(connectionPos));
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; allocated for node %o\n\r"),millis(),nodeAddress));
            return;
        }

    }

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; exiting loop, connPosAddr: %lu \n\r"),millis(), connectionPos));
    if (callback != NULL) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; ERR, buff full for node %o. Invoking callback..\n\r"),millis(),nodeAddress));
        ConnectionStatus status(CONN_NO_SPACE_AVAILABLE, nodeAddress);
        callback(&status);
    }
}

/******************************************************************/
void RF24Network::sendReliable(RF24NetworkHeader * header, const void * message, size_t len, void (*callback)(MessageStatus *)) {
    Connection * connection = getConnection(header->to_node);
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendRel;\n\r"),millis()));
    if (connection == NULL) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: connection is null;\n\r"),millis()));
        if (callback != NULL) {
            MessageStatus status(MSG_NOT_CONNECTED, header);
            callback(&status);
        }
        return;
    } else {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: get buff pos;\n\r"),millis()));
        Message * bufferPosition = getNextFreeMessageBufferPosition();
        if (bufferPosition == NULL) {
            if (callback != NULL) {
                MessageStatus status(MSG_BUFFER_FULL, header);
                callback(&status);
            }
            return;
        }

        //enforce this nodes address.
        header->from_node = node_address;
        memcpy(bufferPosition->payload,header,sizeof(RF24NetworkHeader));
        if (len) {
          memcpy(bufferPosition->payload + sizeof(RF24NetworkHeader),message,min(FRAME_SIZE-sizeof(RF24NetworkHeader),len));
        }
        bufferPosition->retries = 0;
        bufferPosition->messageCallback = callback;
        bufferPosition->retryTimeout = connection->delayBetweenMsg + millis();

        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: stored new message;\n\r"),millis()));
        IF_SERIAL_DEBUG(printMessage(bufferPosition));
    }
}

/******************************************************************/
// This method is used only internally, when receiving a SYN message.
Connection * RF24Network::getExistingOrAllocateNewConnection(uint16_t nodeAddress) {
    Connection * connectionPos;

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getExOrAllNew; \n\r"),millis()));
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (!connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getExOrAllNew; setting conn true for existing conn of node %o\n\r"),millis(), nodeAddress));   
            connectionPos->connected = true;
            return connectionPos;
        } else if (connectionPos->dirty == true) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getExOrAllNew; allocated new conn for node %o\n\r"),millis(), nodeAddress));  
            resetConnection(connectionPos, NULL, true, nodeAddress);
            return connectionPos;
        }

    }

    return NULL;
}

/******************************************************************/

void RF24Network::resetConnection(Connection * connection, void (* callback)(ConnectionStatus *), bool connected, uint8_t nodeAddress) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: resetConn; node: %o; conn: %d\n\r"),millis(),nodeAddress, connected)); 
    connection->nodeAddress = nodeAddress;
    connection->lastMsgRcvdId = 0;
    connection->ackSent = true;
    connection->connected = connected;
    connection->dirty = false;
    connection->connectionRetryTimestamp = millis(); //when to send next connection attempt.
    connection->delayBetweenMsg = CONN_MIN_MSG_DELAY;
    connection->connectionRetries = 0;
    connection->connCallback = callback;
    connection->heartbeatRetries = 0;
}

/******************************************************************/
// Get a connection which isn't dirty but has not yet received a SYN_ACK confirmation for the nodeAddress passed as parameter.
Connection * RF24Network::getDanglingConnection(uint16_t nodeAddress) {
    Connection * connectionPos;
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getDanglingConnection;\n\r"),millis()));
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (!connectionPos->connected && !connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getDanglingConnection; found dangling conn for node %o\n\r"),millis(), nodeAddress));
            return connectionPos;
        }

    }

    return NULL;
}

/******************************************************************/

Connection * RF24Network::getConnection(uint16_t nodeAddress) {
    Connection * connectionPos;
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getConnection;\n\r"),millis())); 
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (connectionPos->connected && !connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getConnection; found conn for node: %o\n\r"),millis(), nodeAddress));
            return connectionPos;
        }

    }

    return NULL;
}

/******************************************************************/

bool RF24Network::write(RF24NetworkHeader& header,const void* message, size_t len)
{
  // Fill out the header
  header.from_node = node_address;

  // Build the full frame to send
  memcpy(frame_buffer,&header,sizeof(RF24NetworkHeader));
  if (len)
    memcpy(frame_buffer + sizeof(RF24NetworkHeader),message,min(FRAME_SIZE-sizeof(RF24NetworkHeader),len));

  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n\r"),millis(),header.toString()));
  if (len)
  {
    IF_SERIAL_DEBUG(const uint16_t* i = reinterpret_cast<const uint16_t*>(message);printf_P(PSTR("%lu: NET message %04x\n\r"),millis(),*i));
  }


  // If the user is trying to send it to himself
  if ( header.to_node == node_address )
    // Just queue it in the received queue
    return enqueue();
  else
    // Otherwise send it out over the air, non-reliably, reliability is ensured on peer nodes
    return write(header.to_node);
}

/******************************************************************/

bool RF24Network::write(uint16_t to_node)
{
  bool ok = false;

  // Throw it away if it's not a valid address
  if ( !is_valid_address(to_node) )
    return false;

  // Where do we send this?  By default, to our parent
  uint16_t send_node = parent_node;
  // On which pipe
  uint8_t send_pipe = parent_pipe;

  // If the node is a direct child,
  if ( is_direct_child(to_node) )
  {
    // Send directly
    send_node = to_node;

    // To its listening pipe
    send_pipe = 0;
  }
  // If the node is a child of a child
  // talk on our child's listening pipe,
  // and let the direct child relay it.
  else if ( is_descendant(to_node) )
  {
    send_node = direct_child_route_to(to_node);
    send_pipe = 0;
  }

  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r"),millis(),to_node,send_node,send_pipe));

#if !defined (DUAL_HEAD_RADIO)
  // First, stop listening so we can talk
  radio.stopListening();
#endif

  ok = write_to_pipe( send_node, send_pipe );

      // NOT NEEDED anymore.  Now all reading pipes are open to start.
#if 0
  // If we are talking on our talking pipe, it's possible that no one is listening.
  // If this fails, try sending it on our parent's listening pipe.  That will wake
  // it up, and next time it will listen to us.

  if ( !ok && send_node == parent_node )
    ok = write_to_pipe( parent_node, 0 );
#endif

#if !defined (DUAL_HEAD_RADIO)
  // Now, continue listening
  radio.startListening();
#endif

  return ok;
}

/******************************************************************/

bool RF24Network::write_to_pipe( uint16_t node, uint8_t pipe )
{
  bool ok = false;

  uint64_t out_pipe = pipe_address( node, pipe );

#if !defined (DUAL_HEAD_RADIO)
 // Open the correct pipe for writing.
  radio.openWritingPipe(out_pipe);

  radio.writeFast(frame_buffer, FRAME_SIZE);
  ok = radio.txStandBy(txTimeout);
#else
  radio1.openWritingPipe(out_pipe);
  radio1.writeFast(frame_buffer, FRAME_SIZE);
  ok = radio1.txStandBy(txTimeout);

#endif

  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sent on %lx %S\n\r"),millis(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));

  return ok;
}

/******************************************************************/

const char* RF24NetworkHeader::toString(void) const
{
  static char buffer[45];
  //snprintf_P(buffer,sizeof(buffer),PSTR("id %04x from 0%o to 0%o type %c"),id,from_node,to_node,type);
  sprintf_P(buffer,PSTR("id %04x from 0%o to 0%o type %d"),id,from_node,to_node,(int)type);
  return buffer;
}

/******************************************************************/

bool RF24Network::is_direct_child( uint16_t node )
{
  bool result = false;

  // A direct child of ours has the same low numbers as us, and only
  // one higher number.
  //
  // e.g. node 0234 is a direct child of 034, and node 01234 is a
  // descendant but not a direct child

  // First, is it even a descendant?
  if ( is_descendant(node) )
  {
    // Does it only have ONE more level than us?
    uint16_t child_node_mask = ( ~ node_mask ) << 3;
    result = ( node & child_node_mask ) == 0 ;
  }

  return result;
}

/******************************************************************/

bool RF24Network::is_descendant( uint16_t node )
{
  return ( node & node_mask ) == node_address;
}

/******************************************************************/

void RF24Network::setup_address(void)
{
  // First, establish the node_mask
  uint16_t node_mask_check = 0xFFFF;
  while ( node_address & node_mask_check )
    node_mask_check <<= 3;

  node_mask = ~ node_mask_check;

  // parent mask is the next level down
  uint16_t parent_mask = node_mask >> 3;

  // parent node is the part IN the mask
  parent_node = node_address & parent_mask;

  // parent pipe is the part OUT of the mask
  uint16_t i = node_address;
  uint16_t m = parent_mask;
  while (m)
  {
    i >>= 3;
    m >>= 3;
  }
  parent_pipe = i;

#ifdef SERIAL_DEBUG
  printf_P(PSTR("setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r"),node_address,node_mask,parent_node,parent_pipe);
#endif
}

/******************************************************************/

uint16_t RF24Network::direct_child_route_to( uint16_t node )
{
  // Presumes that this is in fact a child!!

  uint16_t child_mask = ( node_mask << 3 ) | 0B111;
  return node & child_mask ;
}

/******************************************************************/

uint8_t RF24Network::pipe_to_descendant( uint16_t node )
{
  uint16_t i = node;
  uint16_t m = node_mask;

  while (m)
  {
    i >>= 3;
    m >>= 3;
  }

  return i & 0B111;
}

/******************************************************************/

bool is_valid_address( uint16_t node )
{
  bool result = true;

  while(node)
  {
    uint8_t digit = node & 0B111;
    if (digit < 1 || digit > 5)
    {
      result = false;
      printf_P(PSTR("*** WARNING *** Invalid address 0%o\n\r"),node);
      break;
    }
    node >>= 3;
  }

  return result;
}

/******************************************************************/

uint64_t pipe_address( uint16_t node, uint8_t pipe )
{
  static uint8_t pipe_segment[] = { 0x3c, 0x5a, 0x69, 0x96, 0xa5, 0xc3 };

  uint64_t result;
  uint8_t* out = reinterpret_cast<uint8_t*>(&result);

  out[0] = pipe_segment[pipe];

  uint8_t w;
  short i = 4;
  short shift = 12;
  while(i--)
  {
    w = ( node >> shift ) & 0xF ;
    w |= ~w << 4;
    out[i+1] = w;

    shift -= 4;
  }

  IF_SERIAL_DEBUG(uint32_t* top = reinterpret_cast<uint32_t*>(out+1);printf_P(PSTR("%lu: NET Pipe %i on node 0%o has address %lx%x\n\r"),millis(),pipe,node,*top,*out));

  return result;
}


/********************** debugging helper functions ********************************************/
#ifdef SERIAL_DEBUG
void RF24Network::printConnectionProperties(Connection * conn) {
    
    printf_P(PSTR("%lu: ConnProp; drt: %s; tN: %o; cn: %s; lMRI: %d; aS: %s; cR: %d; cRT: %lu; dBM: %lu; clb: %d; FR: %d \n\r"),
            millis(),
            conn->dirty ? "t" : "f",
            conn->nodeAddress,
            conn->connected ? "t" : "f",
            conn->lastMsgRcvdId,
            conn->ackSent ? "t" : "f",
            conn->connectionRetries,
            conn->connectionRetryTimestamp,
            conn->delayBetweenMsg,
            conn->connCallback,
            freeRam()
    );
    
}

int RF24Network::freeRam (void) {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void RF24Network::printMessage(Message * message) {
    const RF24NetworkHeader& header = * reinterpret_cast<RF24NetworkHeader*>(message->payload);
    unsigned char payload[30];
    memcpy(payload, message->payload + sizeof(RF24NetworkHeader), 30);
    
    printf_P(PSTR("%lu: payloadAddress; %d; payloadWithOffset: %d;\n\r"), millis(), message->payload, message->payload + sizeof(RF24NetworkHeader));
    
    
    printf_P(PSTR("headers; from: %o; to: %o, id: %d; type: %d;\n\r"), header.from_node, header.to_node, header.id, header.type);
    

    printf_P(PSTR("first chars; %c%c%c%c%c%c;\n\r"), payload[0], payload[1], payload[2], payload[3], payload[4], payload[5]);
    
    printf_P(PSTR("%lu: MsgProp; retT: %lu; retries: %d; head: %s; FR: %i \n\r"),
            millis(),
            message->retryTimeout,
            message->retries,
            header.toString(),
            freeRam()
    );
}

#endif


/************************ Sleep Mode ******************************************/


#if defined ENABLE_SLEEP_MODE

#if !defined(__arm__) && !defined (__ARDUINO_X86__)

void wakeUp(){
  sleep_disable();
  sleep_cycles_remaining = 0;
}

ISR(WDT_vect){
  --sleep_cycles_remaining;
}

void RF24Network::sleepNode( unsigned int cycles, int interruptPin ){

  sleep_cycles_remaining = cycles;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  if(interruptPin != 255){
  	attachInterrupt(interruptPin,wakeUp, LOW);
  }
  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  WDTCR |= _BV(WDIE);
  #else
  WDTCSR |= _BV(WDIE);
  #endif
  
  while(sleep_cycles_remaining){
    sleep_mode();                        // System sleeps here
  }                                     // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out
  detachInterrupt(interruptPin);
  
  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	WDTCR &= ~_BV(WDIE);
  #else
	WDTCSR &= ~_BV(WDIE);
  #endif
}

void RF24Network::setup_watchdog(uint8_t prescalar){

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag
  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  WDTCR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
  #else
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
  #endif
}


#endif // not ATTiny
#endif // Enable sleep mode
