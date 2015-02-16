/*
 * File:   RF24NetworkStructs.h
 *
 * Created on February 15, 2015, 4:27 PM
 */

#ifndef RF24NETWORKSTRUCTS_H
#define	RF24NETWORKSTRUCTS_H

#define CONN_RETRIES 20
#define CONN_RETRY_TIME_MULTIPLIER 2
#define CONN_MIN_MSG_DELAY 1
#define CONN_MAX_MSG_DELAY 255

#define FRAME_SIZE 32 /**< How large is each frame over the air */

#include "RF24NetworkStatus.h"

/**
 * Structure that stores information about a connection to a node.
 */
struct Connection {

    // Node address of the connection.
    uint16_t nodeAddress;
    // Id of the last received message. This is used for sending ACK messages.
    uint16_t lastMsgRcvdId;
    // Whether an ACK was sent for the last received message.
    bool ackSent;
    // If this node is connected to the nodeAddress declared in this structure.
    bool connected;

    // If the position of the buffer that stores this Connection structure can be reused.
    bool dirty;
    // Retry interval (in milliseconds * 4) for messages that failed to deliver.
    // In a perfect case scenario, this is always the lowest threshold, or zero.
    // If this node is failing to receive ACK's for the messages it is sending, this is
    // doubled (or increased) automatically on each call to update(), and added to each message's
    // retryTimeout parameter. This way we try to prevent message flooding and mid-air
    // packet collision.
    uint8_t delayBetweenMsg;
    // Number of retries while trying to establish a connection to a node.
    uint8_t connectionRetries;
    // Handler function pointer for feedback on trying to establish a connection.
    void (* connCallback) (ConnectionStatus *);

    Connection() : nodeAddress(0), lastMsgRcvdId(0),
        ackSent(true), dirty(true), delayBetweenMsg(CONN_MIN_MSG_DELAY), connectionRetries(0), connCallback(NULL) {};
};

/**
 * Structure used to store a message in a send buffer.
 */
struct Message {
    // Destination address
    uint16_t destinationNode;
    // How many much time to wait until message is sent again. 
    // When updating, this is typically always current microcontroller time + Connection.delayBetweenMsg
    uint16_t retryTimeout;
    // How many retries were already made
    uint8_t retries;

    // If the position of the buffer containing this message can be reused.
    // That is, if this message is already old and can be discarded or isn't valid anymore, set dirty = true;
    bool dirty;
    // The payload of the message to be sent
    uint8_t message[FRAME_SIZE];
    // A function pointer for a callback that will be called either on message delivery failure or success
    void (* messageCallback) (MessageStatus *);

    Message() : destinationNode(0), retryTimeout(0), retries(0), dirty(true), messageCallback(NULL){};

};

#endif	/* RF24NETWORKSTRUCTS_H */


