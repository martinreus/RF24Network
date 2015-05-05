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
  radio.setRetries(retryVar, 5); // decreased from 15 to 2
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
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: update\n"),millis()));
    readMessages();
    sendAcks();
    sendConnectionAttempts();
    sendHeartbeatRequests();
    sendBufferizedMessages();
}

/******************************************************************/

void RF24Network::sendAcks(){
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendAcks\n"),millis()));
    for (Connection * connection = connections; connection < &connections[NUMBER_OF_CONNECTIONS]; connection++) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: connection iter: %i; Conn start: %i;\n"),millis(), connection, connections));
        if (connection->connected && !connection->ackSent) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connection found with acks to send. ConnNumber: %i; ackId: %i;\n"),millis(),connection->nodeAddress, connection->lastMsgRcvdId));
            RF24NetworkHeader header(connection->nodeAddress, MSG_ACK);
            write(header, &(connection->lastMsgRcvdId), sizeof(connection->lastMsgRcvdId));
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Ack sent!\n"),millis()));
            connection->ackSent = true;
        }
    }
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

      IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Received on %u %s\n"),millis(),pipe_num,header.toString()));
      IF_SERIAL_DEBUG(const uint16_t* i = reinterpret_cast<const uint16_t*>(frame_buffer + sizeof(RF24NetworkHeader));printf_P(PSTR("%lu: NET message %04x\n"),millis(),*i));

      // Throw it away if it's not a valid address
      if ( !is_valid_address(header.to_node) ){
		continue;
	  }

        // Is this for us?
        if ( header.to_node == node_address ){
            // Add it to the buffer of frames for us if it is a user message.
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Received packet of type %d\n"),millis(),header.type));
            if (header.type < SYSTEM_MESSAGES) {
                enqueue();
            // if not, treat it as a system message.
            } else {
                treatSystemMessages();
            }
            // If there is a connection for the sender of this message, 
            // decrease the connection delay for every message received.
            Connection * conn = getConnection(header.from_node);
            if (conn != NULL) {
                decreaseConnectionDelay(conn);
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
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendBufM; \n"),millis()));
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendBufMsgSize: %d; \n"),millis(), sizeof(sendMessageBuffer)));

    for (int bufferPos = 0 ; bufferPos < messageBufferUsedPositions ; bufferPos++ ) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Buff pos %d\n"),millis(), bufferPos));
        Message * message = &sendMessageBuffer[bufferPos];
        IF_SERIAL_DEBUG(printMessage(message)); 

        // address information
        RF24NetworkHeader * header = reinterpret_cast<RF24NetworkHeader *>(message->payload);

        // if the connection is not valid anymore, do not send the buffered message.
        Connection * connection = getConnection(header->to_node);
        if (connection == NULL) {
            if (message->messageCallback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: No Conn, invoking handler\n"),millis()));
                MessageStatus status(MSG_STATUS_DISCONNECTED, header->to_node, header->id);
                message->messageCallback(&status);
            }
            continue;
        }

        // if the message has not been sent MSG_SENDING_MAX_RETRIES times, has reached its next time to be sent and has a valid id, then send it!
        if (message->retries < MSG_SENDING_MAX_RETRIES && message->retryTimeout < millis() && connection->lastMessageIdSent >= header->id) {
            // if this message was already tried to be sent but no ACK was received, we increment the connection delay, to prevent
            // burst sending messages from a single node.
            if (message->retries > 0 && connection->delayBetweenMsg < CONN_MAX_MSG_DELAY) {
                increaseConnectionDelay(connection);
            }
            
            //send!
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Sending message!\n"),millis()));
            write(*header, message->payload + sizeof(RF24NetworkHeader), FRAME_SIZE - sizeof(RF24NetworkHeader));
            message->retries++;
            message->retryTimeout = millis() + connection->delayBetweenMsg;
        // if the message was already tried to be sent MSG_SENDING_MAX_RETRIES times or it has an invalid id, then purge it from the buffer.
        } else if (message->retries >= MSG_SENDING_MAX_RETRIES || smaller(connection->lastMessageIdSent,header->id)) {
            // if the message has timed out, but the message id is still valid, we invalidate all messages with id's greater than this message->id,
            // so that the next messages after this one are purged too. That is, we are purging all messages with id greater than this one which
            // has failed MSG_SENDING_MAX_RETRIES times for a single target node.
            if (connection->lastMessageIdSent >= header->id) {
                connection->lastMessageIdSent = header->id - 1;
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: lastMessageSentId set to %i\n"),millis(),connection->lastMessageIdSent));
            }

            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: max retries or id not valid for message %d; purging..\n"),millis(),header->id));
            purgeBufferedMessage(bufferPos);

            //we go back one position if the last one was excluded from the array due to retries overflow
            bufferPos--;
            //let who tried to send this message know that it was not delivered.
            if (message->messageCallback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: invoking handler\n"),millis()));
                MessageStatus status(MSG_STATUS_TIMEOUT, header->to_node, header->id);
                message->messageCallback(&status);
            }
        }

    }
}


/******************************************************************/

void RF24Network::prepareMessage(Message * message) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: prepareMsg\n"),millis()));
    message->retryTimeout = 0;
    message->retries = 0;
    message->messageCallback = NULL; 
    // clean only payload of the message, which also contains the header that will be sent OTA.
    memset(message->payload + sizeof(RF24NetworkHeader), 0, sizeof(message->payload) - sizeof(RF24NetworkHeader));

    IF_SERIAL_DEBUG(printMessage(message));
}

/******************************************************************/

Message * RF24Network::getNextFreeMessageBufferPosition(uint16_t targetNodeAddress) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: nextFreeMsg\n"),millis()));

    if (messageBufferUsedPositions >= BUFFER_SIZE) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: fullBuffer\n"),millis()));
        return NULL;
    }

    Message * bufferPos;
    // insert a message in the buffer right after the last message with id immediately lower than the new message for
    // the same node. We start searching a message for the same recipient from the last array position, so that
    // we guarantee the order of message sending. This also eases the cleanup of messages that failed to be delivered,
    // while sending new ones.
    for (uint8_t i = messageBufferUsedPositions; i >= 0; i--) {
        // If the iteration is on the start of the buffer, just use this position
        if (i == 0) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: freeing pos for msg at index 0.\n"),millis()));
            // if there are already messages on the buffer, we first have to shift all buffered messages one position ahead,
            // so that we may use the zero-th position.
            if (messageBufferUsedPositions > 0){
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: moving %i msgs to the r. [moving %i bytes from %i to %i][0thBufpos: %i][sizeof Mes: %i]\n"),millis(), messageBufferUsedPositions,messageBufferUsedPositions * sizeof(Message),sendMessageBuffer,&sendMessageBuffer[1],sendMessageBuffer, sizeof(Message)));
                memmove(&sendMessageBuffer[1] , sendMessageBuffer, messageBufferUsedPositions * sizeof(Message));
            }
            // then we may use this position.            
            bufferPos = &sendMessageBuffer[i];
            prepareMessage(bufferPos);
            IF_SERIAL_DEBUG(printMessage(bufferPos));
            break; // Do NOT forget this! we are finished allocating a new buffer position, so we just leave.
        } else {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: entering else in get new bP, prev msg:\n"),millis()));
            // if the previous message on the buffer is for the same recipient, we allocate a new message right after this one.
            Message * previousMessage = &sendMessageBuffer[i - 1];
            IF_SERIAL_DEBUG(printMessage(previousMessage));
            RF24NetworkHeader * header = reinterpret_cast<RF24NetworkHeader *>(previousMessage->payload);
            
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Id Pr. Mes.: %o; Target Node: %o\n"),millis(), header->to_node, targetNodeAddress));
            if(header->to_node == targetNodeAddress) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Prev. addr. is same. Using index %i \n"),millis(), i));
                // first, move messageBufferUsedPositions of messages on the array 
                if (messageBufferUsedPositions - i > 0) {
                    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: moving %i msgs to the r. [moving %i bytes from %i to %i]\n"),millis(), messageBufferUsedPositions - i,(messageBufferUsedPositions - i) * sizeof(Message),&sendMessageBuffer[i],&sendMessageBuffer[i + 1]));
                    memmove(&sendMessageBuffer[i + 1] , &sendMessageBuffer[i], (messageBufferUsedPositions - i) * sizeof(Message));
                }
                //then, assign the new freed up position as the position we want to use, and prepare it.
                bufferPos = &sendMessageBuffer[i];
                prepareMessage(bufferPos);
                IF_SERIAL_DEBUG(printMessage(bufferPos));   
                break; // Do NOT forget this! we are finished allocating a new buffer position, so we just leave.
            }
        }
    }
    
    messageBufferUsedPositions++;
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Increased used buf. pos. to %i \n"),millis(), messageBufferUsedPositions));
    return bufferPos;
}

/******************************************************************/

void RF24Network::purgeBufferedMessage(uint8_t bufferPos) {
    // only purge valid buffer positions
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: purgeBufPos %i; \n"),millis(), bufferPos));
    if (bufferPos >= messageBufferUsedPositions || bufferPos < 0 || messageBufferUsedPositions == 0)
        return;

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: purging\n"),millis()));
    IF_SERIAL_DEBUG(printMessage(&sendMessageBuffer[bufferPos]));

    messageBufferUsedPositions--;

    // if it is the last position, just update the retries to max.
    if (bufferPos == messageBufferUsedPositions) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: last position, doing nothing...\n"),millis()));
    // if not, we need to shift all messages one position to the left
    } else {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: moving %i positions [moving %i bytes from %i to %i]\n"),millis(),messageBufferUsedPositions - bufferPos,sizeof(Message) * (messageBufferUsedPositions - bufferPos),&sendMessageBuffer[bufferPos + 1],&sendMessageBuffer[bufferPos]));
        memmove(&sendMessageBuffer[bufferPos], &sendMessageBuffer[bufferPos + 1], sizeof(Message) * (messageBufferUsedPositions - bufferPos));
    }

}


/******************************************************************/

void RF24Network::sendConnectionAttempts(void) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SendConnAtt(SCA);\n"),millis()));
    uint16_t currentTimestamp = millis();
    Connection * connectionPos;

    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        IF_SERIAL_DEBUG(printConnectionProperties(connectionPos));

        if (!connectionPos->connected
                && !connectionPos->dirty
                && connectionPos->connectionRetries < CONN_RETRIES
                && connectionPos->connectionRetryTimestamp < currentTimestamp) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; tN %o\n"),millis(),connectionPos->nodeAddress));
            connectionPos->connectionRetries++;
            connectionPos->connectionRetryTimestamp = currentTimestamp + connectionPos->delayBetweenMsg;
            increaseConnectionDelay(connectionPos);
            RF24NetworkHeader header(connectionPos->nodeAddress, MSG_SYN);
            write(header, NULL, 0);
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; tN: %o; dBM: %lu; cRT: %lu; cR: %d;\n"),millis(),
                    connectionPos->nodeAddress, connectionPos->delayBetweenMsg, connectionPos->connectionRetryTimestamp, connectionPos->connectionRetries));
        } else if (!connectionPos->connected
                && !connectionPos->dirty
                && connectionPos->connectionRetries >= CONN_RETRIES) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; timeout reached node: %o; Set conn dirty.\n"),millis(),connectionPos->nodeAddress));
            connectionPos->dirty = true;
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; timeout reached node: %o; Set conn dirty.\n"),millis(),connectionPos->nodeAddress));
            if (connectionPos->connCallback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: SCA; Invoke conn handler CONN_TIMEOUT.\n"),millis()));
                ConnectionStatus status(CONN_TIMEOUT, connectionPos->nodeAddress);
                connectionPos->connCallback(&status);
            }
        }

    }
}

/******************************************************************/

void RF24Network::increaseConnectionDelay(Connection * connection) {
    if (connection->delayBetweenMsg < (uint16_t) CONN_MAX_MSG_DELAY) {
        connection->delayBetweenMsg = (connection->delayBetweenMsg) * 2;
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: increaseConnDelay; node %o with %lu millis between messages\n"),millis(), connection->nodeAddress, connection->delayBetweenMsg));
    }
}

/******************************************************************/

void RF24Network::decreaseConnectionDelay(Connection * connection) {
    if (connection->delayBetweenMsg > (uint16_t) CONN_MIN_MSG_DELAY) {
        connection->delayBetweenMsg = (connection->delayBetweenMsg) / 2;
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: decreaseConnDelay; node %o with %lu millis between messages\n"),millis(), connection->nodeAddress, connection->delayBetweenMsg));
    }
}
/******************************************************************/

void RF24Network::treatSystemMessages() {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes \n"),millis()));
    // Read the beginning of the frame as the header
    const RF24NetworkHeader & header = * reinterpret_cast<RF24NetworkHeader*>(frame_buffer);
    Connection* connection;

    switch (header.type) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, header type: %i\n"),millis(), header.type));
        case MSG_SYN:
            // Try getting an already open connection for this node or try opening a new one.
            connection = getExistingOrAllocateNewConnection(header.from_node);
            if (connection == NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, send MSG_CONN_REFUSED to node %o\n"),millis(),header.from_node));
                // No connection is possible.
                RF24NetworkHeader destinationHeader(header.from_node, MSG_CONN_REFUSED);
                write(destinationHeader, NULL, 0);
            } else {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, send MSG_SYN_ACK to node %o\n"),millis(),header.from_node));
                // send a SYN_ACK for new connection of already existing connection.
                RF24NetworkHeader destinationHeader(header.from_node, MSG_SYN_ACK); 
                write(destinationHeader, NULL, 0);
            }
            break;
        case MSG_SYN_ACK:
            // Get the connection that originated this SYN_ACK.
            connection = getDanglingConnection(header.from_node);
            if (connection != NULL) { //if there is no connection for this node, simply ignore this message.
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, set connected=true for node %o\n"),millis(),header.from_node));
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
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, conn refused for node %o\n"),millis(),header.from_node));
                connection->dirty = true;
                if (connection->connCallback != NULL) {
                    ConnectionStatus status(CONN_REFUSED, connection->nodeAddress);
                    connection->connCallback(&status);
                }
            }
            break;
        case MSG_HEARTBEAT_REQ:
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, heartbeat request from node %o\n"),millis(),header.from_node));
            // Get the connection that the other node wants to test.
            connection = getConnection(header.from_node);
            if (connection != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, sending heartbeat response to node %o\n"),millis(),header.from_node));
                connection->heartbeatRetries = 0;
                RF24NetworkHeader destinationHeader(header.from_node, MSG_HEARTBEAT_ACK); 
                write(destinationHeader, NULL, 0);
            }
            //TODO when not connected, send not connected message...
            break;
        case MSG_HEARTBEAT_ACK:
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, heartbeat ack from node %o\n"),millis(),header.from_node));
            // Get the connection for which we sent a heartbeat request.
            connection = getConnection(header.from_node);
            if (connection != NULL) {
                connection->heartbeatRetries = 0;
            }
            break;
        case MSG_ACK:
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: TreatSysMes, ack from node %o\n"),millis(),header.from_node));
            // Get the connection for which we sent a heartbeat request.
            connection = getConnection(header.from_node);
            if (connection != NULL) {
                //assuming we have a correct id in the payload,
                const uint8_t ackedId = * reinterpret_cast<uint8_t*>(frame_buffer + sizeof(RF24NetworkHeader));
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Node %o received our messages until id %i.\n"),millis(),header.from_node,ackedId));
                purgeSentMessagesFromSendBufferAndFireMessageCallBack(connection, ackedId);
            }
            break;
    }
}

/******************************************************************/

void RF24Network:: sendHeartbeatRequests(void) {
    long currentMillis = millis();
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; \n"),millis()));

    //if it is not time to send heartbeat requests, just return.
    if (currentMillis < nextHeartbeatSend) {
        return;       
    }

    Connection * connectionPos;

    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (!connectionPos->dirty && connectionPos->connected && connectionPos->heartbeatRetries < CONN_MAX_HEARTBEAT_RETRIES) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; sending heartbeat to node %o\n"),millis(), connectionPos->nodeAddress));
            RF24NetworkHeader header(connectionPos->nodeAddress, MSG_HEARTBEAT_ACK);
            write(header, NULL, 0);
            connectionPos->heartbeatRetries++;
        } else if (!connectionPos->dirty && connectionPos->connected && connectionPos->heartbeatRetries >= CONN_MAX_HEARTBEAT_RETRIES) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; heartbeat timed out; closing connection to node %o\n"),millis(), connectionPos->nodeAddress));
            // We did not receive a heartbeat response, so the other node is either dead or connection was closed on the other side.
            // We just close our side of the connection.
            connectionPos->dirty = true;
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; conn timed out. Purging messages from buffer..\n"),millis()));
            purgeMessagesForTimedOutConn(connectionPos);
            //if there is a connection handler, invoke it with connection timed out after purging all messages that are still in the buffer.
            if (connectionPos->connCallback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendHeartbeatRequests; invoking connection callback with CONN_TIMEOUT\n"),millis()));
                ConnectionStatus status(CONN_TIMEOUT, connectionPos->nodeAddress);
                connectionPos->connCallback(&status);
            }
        }
    }

    nextHeartbeatSend = currentMillis + (1000 * CONN_HEARTBEAT_INTERVAL);
}

/******************************************************************/
void RF24Network::purgeSentMessagesFromSendBufferAndFireMessageCallBack(Connection * connection, uint8_t ackedId){
    RF24NetworkHeader * messageHeader;
    bool higherIdMessage = true;
    for (int8_t i = messageBufferUsedPositions - 1; i >= 0; i-- ) {
        messageHeader = reinterpret_cast<RF24NetworkHeader *>(sendMessageBuffer[i].payload);
        
        if (messageHeader->to_node == connection->nodeAddress && smallerOrEqual(messageHeader->id, ackedId)) {
            // fire callback only for the message with higher id.
            if (higherIdMessage) {
                higherIdMessage = false;
                if (sendMessageBuffer[i].messageCallback != NULL) {
                    MessageStatus status(MSG_STATUS_OK, messageHeader->to_node, messageHeader->id);
                    sendMessageBuffer[i].messageCallback(&status);
                }
            }
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: purging message with id %i for node %o; found message.\n"),millis(), messageHeader->id, messageHeader->to_node));
            purgeBufferedMessage(i);
        }
    }
}

/******************************************************************/
void RF24Network::purgeMessagesForTimedOutConn(Connection * conn){
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: purgeMessagesForTimedOutConn;\n"),millis()));
    Message * initialPositionToMove = NULL;
    RF24NetworkHeader * currentHeader;
    uint8_t foundMessages = 0;

    for (uint8_t i = 0 ; i < messageBufferUsedPositions; i++ ) {
        currentHeader = reinterpret_cast<RF24NetworkHeader *>(sendMessageBuffer[i].payload);
        
        if (currentHeader->to_node == conn->nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: purgeMessagesForTimedOutConn; found message.\n"),millis()));
            IF_SERIAL_DEBUG(printMessage(&sendMessageBuffer[i]));
            if (initialPositionToMove == NULL) {
                initialPositionToMove = &sendMessageBuffer[i];
            }
            foundMessages++;
        }
    }
    
    if (foundMessages > 0) {
        //remove messages from buffer by moving 
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: moving %i positions [moving %i bytes from %i to %i]\n"),millis(),foundMessages,sizeof(Message) * (messageBufferUsedPositions - foundMessages),initialPositionToMove + foundMessages,initialPositionToMove));
        memmove(initialPositionToMove, initialPositionToMove + foundMessages, sizeof(Message) * (messageBufferUsedPositions - foundMessages));
        messageBufferUsedPositions -= foundMessages;
    }
}


/******************************************************************/
bool RF24Network::enqueue(void)
{
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Enqueue "),millis()));
    uint8_t * insertAt = NULL;
    bool foundForSameTarget = false;
    RF24NetworkHeader * newMessageHeader = reinterpret_cast<RF24NetworkHeader *>(frame_buffer);;
    RF24NetworkHeader * previousMessage;
  
    // if the connection is not valid anymore, do not store the buffered message. (and possibly send a NOT_CONN)
    Connection * conn = getConnection(newMessageHeader->from_node);
    if (conn == NULL) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Message dropped. Connection closed."),millis()));
        return false;
    }
  
    //we are expecting a message with id incremented by 1 over the last received. 
    //For implementation simplicity, all others should be discarded
    if (smallerOrEqual(newMessageHeader->id, conn->lastMsgRcvdId)) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu; Already received message with id %i from node %o, setting to ack.\n"), millis(),newMessageHeader->id, newMessageHeader->from_node));
        conn->ackSent = false; //force ACK to be sent for messages we have already received.
        return true;
    } else if (conn->lastMsgRcvdId + 1 != newMessageHeader->id) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu; Ignored message from node %o with id %i, we are expecting message with id %i.\n"), millis(), newMessageHeader->from_node, newMessageHeader->id, (uint8_t) (conn->lastMsgRcvdId + 1)));
        conn->ackSent = false; //force ACK to be sent so that sending node knows what to send.
        return false;
    }
  
    //we are receiving a new message.. lets process it.
    if ( next_frame < frame_queue + sizeof(frame_queue) )
    {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu; Received message with id %i from node %o, processing it..\n"), millis(), newMessageHeader->id, conn->nodeAddress));
        //search for a message with id immediately lower than the one in frame_buffer.
        //point to the last position in the buffer and decrease until first position is reached
        for (uint8_t * frame_queue_pointer = next_frame; frame_queue_pointer >= frame_queue; frame_queue_pointer -= FRAME_SIZE) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu; For loop, frame_queue_pointer: %i; next_frame = %i; frame_queue: %i;..\n"), millis(), frame_queue_pointer, next_frame, frame_queue));
            //reached first array position
            if (frame_queue_pointer == frame_queue) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu; Inserting at the beginning of the queue..\n"), millis()));
                //if there wasn't any other message for the same node and the id is immediately higher than the last ACKed message, 
                //put it in the received messages buffer.
                insertAt = frame_queue_pointer;
            } else {
                previousMessage = reinterpret_cast<RF24NetworkHeader *>(frame_queue_pointer - FRAME_SIZE);
                if (previousMessage->to_node == newMessageHeader->to_node && previousMessage->id + 1 == newMessageHeader->id) {
                    insertAt = frame_queue_pointer;
                    IF_SERIAL_DEBUG(printf_P(PSTR("%lu; Inserting at %i, queue starts at %i..\n"), millis(), insertAt, frame_queue));
                    break; //DO NOT FORGET this one
                }
            }
        }

        if (insertAt != NULL) {
            conn->lastMsgRcvdId = newMessageHeader->id;
            conn->ackSent = false;
            //shift messages to the right so that received messages are in order.
            if (insertAt < next_frame) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Shifting, initialArrayPos: %i [moving %i bytes from %i to %i]\n"),millis(),frame_queue,next_frame - insertAt, insertAt, insertAt + FRAME_SIZE));
                memmove(insertAt + FRAME_SIZE, insertAt, next_frame - insertAt);
            }
            //copy into to the frame queue
            memcpy(insertAt,frame_buffer, FRAME_SIZE);
            //increment next_frame;
            next_frame += FRAME_SIZE;
            IF_SERIAL_DEBUG(printf_P(PSTR("Stored new incoming message.\n")));
            return true;
        }
    } else {
        IF_SERIAL_DEBUG(printf_P(PSTR("Message dropped, buffer full!\n")));
    }
    return false;
}

/******************************************************************/

bool RF24Network::available(uint16_t nodeAddress) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: available "),millis()));
    // Are there frames on the queue for us?
    if (next_frame == frame_queue) {
        // fail fast if no message exists in buffer.
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: No new message in buffer "),millis()));
        return false;
    }

    RF24NetworkHeader * messageHeaders;
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Messages exist in buffer. Checking if there is one from (%o). Iterating through buffer..."),millis(), nodeAddress));
    for (uint8_t * frame_queue_pointer = frame_queue; frame_queue_pointer < next_frame; frame_queue_pointer += FRAME_SIZE) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Iteration position: %i; Buffer start: %i; next_frame: %i "),millis(), this->node_address, frame_queue_pointer, next_frame));
        messageHeaders = reinterpret_cast<RF24NetworkHeader *>(frame_queue_pointer);
        if (messageHeaders->from_node == nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Found message from (%o) "),millis(), messageHeaders->from_node));
            return true;
        }
    }
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: No message found from (%o) "),millis(), messageHeaders->from_node));
    return false;
}

/******************************************************************/

bool RF24Network::available() {
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

size_t RF24Network::read(RF24NetworkHeader& header,void* message, size_t maxlen) {
    size_t bufsize = 0;

    if ( available() ) {
        // Move the pointer back one in the queue
        next_frame -= FRAME_SIZE;
        //read the first message, since we should read messages in order.
        uint8_t* frame = frame_queue;

        memcpy(&header,frame,sizeof(RF24NetworkHeader));

        if (maxlen > 0) {
            // How much buffer size should we actually copy?
            bufsize = min(maxlen,FRAME_SIZE-sizeof(RF24NetworkHeader));

            // Copy the next available frame from the queue into the provided buffer
            memcpy(message,frame+sizeof(RF24NetworkHeader),bufsize);
        }

        // Move all queue one message back
        memmove(frame_queue, frame_queue + FRAME_SIZE, next_frame - frame_queue);
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Received %s\n"),millis(),header.toString())); 

    }

    return bufsize;
}

/******************************************************************/

void RF24Network::connect(uint16_t nodeAddress, void (* callback)(ConnectionStatus *)) {
    Connection * connectionPos;

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect;\n"),millis()));
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; inside loop; connections: %d; connectionPos: %d;\n"),millis(), connections, connectionPos));
        if (connectionPos->connected && !connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            if (callback != NULL) {
                IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; already conn to node %o\n"),millis(),nodeAddress));
                ConnectionStatus status(CONN_ALREADY_CONNECTED, nodeAddress);
                callback(&status);
            }
            return;
        }
        if (connectionPos->dirty) {
            IF_SERIAL_DEBUG(printConnectionProperties(connectionPos));
            resetConnection(connectionPos, callback, false, nodeAddress);
            IF_SERIAL_DEBUG(printConnectionProperties(connectionPos));
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; allocated for node %o\n"),millis(),nodeAddress));
            return;
        }

    }

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; exiting loop, connPosAddr: %lu \n"),millis(), connectionPos));
    if (callback != NULL) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: Connect; ERR, buff full for node %o. Invoking callback..\n"),millis(),nodeAddress));
        ConnectionStatus status(CONN_NO_SPACE_AVAILABLE, nodeAddress);
        callback(&status);
    }
}

/******************************************************************/
void RF24Network::sendReliable(RF24NetworkHeader * header, const void * message, size_t len, void (*callback)(MessageStatus *)) {
    Connection * connection = getConnection(header->to_node);
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: sendRel;\n"),millis()));
    if (connection == NULL) {
        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: connection is null;\n"),millis()));
        if (callback != NULL) {
            MessageStatus status(MSG_NOT_CONNECTED, header->to_node, header->id);
            callback(&status);
        }
        return;
    } else {
        //enforce this node's address.
        header->from_node = node_address;
        header->id = connection->lastMessageIdSent + 1;
        connection->lastMessageIdSent = header->id;

        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: send reliable; lMSI: %i;\n"),millis(), connection->lastMessageIdSent));

        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: get buff pos;\n"),millis()));
        Message * bufferPosition = getNextFreeMessageBufferPosition(header->to_node);
        if (bufferPosition == NULL) {
            if (callback != NULL) {
                MessageStatus status(MSG_BUFFER_FULL, header->to_node, header->id);
                callback(&status);
            }
            return;
        }

        memcpy(bufferPosition->payload,header,sizeof(RF24NetworkHeader));
        if (len) {
          memcpy(bufferPosition->payload + sizeof(RF24NetworkHeader),message,min(FRAME_SIZE-sizeof(RF24NetworkHeader),len));
        }
        bufferPosition->retries = 0;
        bufferPosition->messageCallback = callback;
        bufferPosition->retryTimeout = connection->delayBetweenMsg + millis();

        IF_SERIAL_DEBUG(printf_P(PSTR("%lu: stored new message;\n"),millis()));
        IF_SERIAL_DEBUG(printMessage(bufferPosition));
    }
}

/******************************************************************/
// This method is used only internally, when receiving a SYN message.
Connection * RF24Network::getExistingOrAllocateNewConnection(uint16_t nodeAddress) {
    Connection * connectionPos;

    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getExOrAllNew; \n"),millis()));
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (!connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getExOrAllNew; setting conn true for existing conn of node %o\n"),millis(), nodeAddress));   
            connectionPos->connected = true;
            return connectionPos;
        } else if (connectionPos->dirty == true) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getExOrAllNew; allocated new conn for node %o\n"),millis(), nodeAddress));  
            resetConnection(connectionPos, NULL, true, nodeAddress);
            return connectionPos;
        }

    }

    return NULL;
}

/******************************************************************/

void RF24Network::resetConnection(Connection * connection, void (* callback)(ConnectionStatus *), bool connected, uint8_t nodeAddress) {
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: resetConn; node: %o; conn: %d\n"),millis(),nodeAddress, connected)); 
    connection->nodeAddress = nodeAddress;
    connection->lastMsgRcvdId = 0;
    connection->lastMessageIdSent = 0;
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
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getDanglingConnection;\n"),millis()));
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (!connectionPos->connected && !connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getDanglingConnection; found dangling conn for node %o\n"),millis(), nodeAddress));
            return connectionPos;
        }

    }

    return NULL;
}

/******************************************************************/

Connection * RF24Network::getConnection(uint16_t nodeAddress) {
    Connection * connectionPos;
    IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getConnection;\n"),millis())); 
    for (int i = 0; i < NUMBER_OF_CONNECTIONS ; i++) {
        connectionPos = &connections[i];
        if (connectionPos->connected && !connectionPos->dirty && connectionPos->nodeAddress == nodeAddress) {
            IF_SERIAL_DEBUG(printf_P(PSTR("%lu: getConnection; found conn for node: %o\n"),millis(), nodeAddress));
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

  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n"),millis(),header.toString()));
  if (len)
  {
    IF_SERIAL_DEBUG(const uint16_t* i = reinterpret_cast<const uint16_t*>(message);printf_P(PSTR("%lu: NET message %04x\n"),millis(),*i));
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

  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sending to 0%o via 0%o on pipe %x\n"),millis(),to_node,send_node,send_pipe));

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

  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sent on %lx %S\n"),millis(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));

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
  printf_P(PSTR("setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n"),node_address,node_mask,parent_node,parent_pipe);
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
      printf_P(PSTR("*** WARNING *** Invalid address 0%o\n"),node);
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

  IF_SERIAL_DEBUG(uint32_t* top = reinterpret_cast<uint32_t*>(out+1);printf_P(PSTR("%lu: NET Pipe %i on node 0%o has address %lx%x\n"),millis(),pipe,node,*top,*out));

  return result;
}

/******************************************************************/

bool RF24Network::smaller(uint8_t a, uint8_t b) {
    return (char) a - (char) b < 0;
}

/******************************************************************/

bool RF24Network::smallerOrEqual(uint8_t a, uint8_t b) {
    return (char) a - (char) b <= 0;
}

/********************** debugging helper functions ********************************************/
#ifdef SERIAL_DEBUG
void RF24Network::printConnectionProperties(Connection * conn) {
    
    printf_P(PSTR("%lu: ConnProp; drt: %s; tN: %o; cn: %s; lMRI: %d; aS: %s; cR: %d; cRT: %lu; dBM: %lu; clb: %d; FR: %d \n"),
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
    
    printf_P(PSTR("%lu: payloadAddress; %d; payloadWithOffset: %d;\n"), millis(), message->payload, message->payload + sizeof(RF24NetworkHeader));
    
    
    printf_P(PSTR("headers; from: %o; to: %o, id: %d; type: %d;\n"), header.from_node, header.to_node, header.id, header.type);
    

    printf_P(PSTR("first chars; %c%c%c%c%c%c;\n"), payload[0], payload[1], payload[2], payload[3], payload[4], payload[5]);
    
    printf_P(PSTR("%lu: MsgProp; retT: %lu; retries: %d; head: %s; FR: %i \n"),
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
