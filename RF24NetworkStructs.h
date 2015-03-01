/*
 * File:   RF24NetworkStructs.h
 *
 * Created on February 15, 2015, 4:27 PM
 */

#ifndef RF24NETWORKSTRUCTS_H
#define	RF24NETWORKSTRUCTS_H

// Reserved message types sent between nodes. Message types ranging from 0-127 are user defined messages.
#define MSG_ACK 130
#define MSG_SYN 131
#define MSG_SYN_ACK 132
#define MSG_BUFFER_FULL 133
#define MSG_CONN_REFUSED 134
#define MSG_HEARTBEAT_REQ 135
#define MSG_HEARTBEAT_ACK 136
#define MSG_NOT_CONNECTED 137

// Constants for connection control
#define CONN_RETRIES 10
#define CONN_RETRY_TIME_MULTIPLIER 2
#define CONN_MIN_MSG_DELAY 1
#define CONN_MAX_MSG_DELAY 512
#define CONN_MAX_HEARTBEAT_RETRIES 5
//heart beat interval in seconds.
#define CONN_HEARTBEAT_INTERVAL 6

#define FRAME_SIZE 32 /**< How large is each frame over the air */

#include "RF24NetworkStatus.h"
#include "RF24Network.h"

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

    // For connection keep alive control. If no response is given for a given amount of heartbeat
    // retries, the connection is automatically closed.
    uint8_t heartbeatRetries;

    // Retry interval (in milliseconds * 4) for messages that failed to deliver.
    // In a perfect case scenario, this is always the lowest threshold, or zero.
    // If this node is failing to receive ACK's for the messages it is sending, this is
    // doubled (or increased) automatically on each call to update(), and added to each message's
    // retryTimeout parameter. This way we try to prevent message flooding and mid-air
    // packet collision.
    unsigned long delayBetweenMsg;
    // next timestamp to send a connection attempt message (SYN)
    unsigned long connectionRetryTimestamp;
    // Number of retries while trying to establish a connection to a node.
    unsigned int connectionRetries;
    // Handler function pointer for feedback on trying to establish a connection.
    void (* connCallback) (ConnectionStatus *);

    Connection() : nodeAddress(0), connected(false), lastMsgRcvdId(0), connectionRetryTimestamp(0),
        ackSent(true), dirty(true), delayBetweenMsg(CONN_MIN_MSG_DELAY), connectionRetries(0), connCallback(NULL), heartbeatRetries(0) {};
};

/**
 * Structure used to store a message in a send buffer.
 */
struct Message {
    // Destination address
    uint16_t destinationNode;
    // How many much time to wait until message is sent again. 
    // When updating, this is typically always current microcontroller time + Connection.delayBetweenMsg
    unsigned long retryTimeout;
    // How many retries were already made
    int retries;
    

    // If the position of the buffer containing this message can be reused.
    // That is, if this message is already old and can be discarded or isn't valid anymore, set dirty = true;
    bool dirty;
    // The payload of the message to be sent
    unsigned char message[FRAME_SIZE];
    // A function pointer for a callback that will be called either on message delivery failure or success
    void (* messageCallback) (MessageStatus *);

    Message() : destinationNode(0), retryTimeout(0), retries(0), dirty(true), messageCallback(NULL){};

};

#endif	/* RF24NETWORKSTRUCTS_H */


