/*
 * File:   RF24NetworkStructs.h
 *
 * Created on February 15, 2015, 4:27 PM
 */

#ifndef RF24NETWORKSTRUCTS_H
#define	RF24NETWORKSTRUCTS_H


// Feedback statuses for when establishing a connection to another node.
#define CONN_NO_SPACE_AVAILABLE -3 //connection control structure array on this node is full
#define CONN_REFUSED -2 // connection refused on target node
#define CONN_TIMEOUT -1 // connection timeout (for an unknown reason)
#define CONN_SUCCESSFUL 1 // connection attempt was successful
#define CONN_ALREADY_CONNECTED 0 //already connected

// Constants for connection control
#define CONN_RETRIES 10
#define CONN_RETRY_TIME_MULTIPLIER 2
#define CONN_MIN_MSG_DELAY 1
#define CONN_MAX_MSG_DELAY 512
#define CONN_MAX_HEARTBEAT_RETRIES 5
//heart beat interval in seconds.
#define CONN_HEARTBEAT_INTERVAL 6

// Feedback 
#define MSG_STATUS_TIMEOUT -2
#define MSG_STATUS_DISCONNECTED -1
#define MSG_STATUS_OK 1

//class RF24;
// Reserved message types sent between nodes. Message types ranging from 0-127 are user defined messages.
#define MSG_ACK 130
#define MSG_SYN 131
#define MSG_SYN_ACK 132
#define MSG_BUFFER_FULL 133
#define MSG_CONN_REFUSED 134
#define MSG_HEARTBEAT_REQ 135
#define MSG_HEARTBEAT_ACK 136
#define MSG_NOT_CONNECTED 137


// Constants for message sending
#define MSG_SENDING_MAX_RETRIES 10

#define FRAME_SIZE 32 /**< How large is each frame over the air */

#include "RF24Network.h"

struct RF24NetworkHeader;

/**
 * A message status is given as argument when invoking the callback function passed as parameter of the
 * sendReliable method used for sending reliable messages to a node. The status code represents a status for whether
 * the message was successfully sent or not, and the msgId is the internally generated id of the message
 * that succeeded or failed to be sent.
 */
struct MessageStatus {

    int8_t statusCode;
    RF24NetworkHeader * header; 

    MessageStatus(int8_t status, RF24NetworkHeader * messageHeader): statusCode(status), header(messageHeader) {};

};

/**
 * A connection status is given as argument when invoking the callback function passed as parameter of the
 * connect() method used for establishing a reliable connection between two nodes.
 * The status code represents a status for whether a connection was successfull.
 */
struct ConnectionStatus {

    int8_t status;
    uint16_t node;

    ConnectionStatus(int8_t statusCode, uint16_t nodeAddress): status(statusCode), node(nodeAddress) {};

};

/**
 * Structure that stores information about a connection to a node.
 */
struct Connection {

    // Node address of the connection.
    uint16_t nodeAddress;
    // If this node is connected to the nodeAddress declared in this structure.
    bool connected;

    // Id of the last message id sent for what we received an ACK from the remote node.
    uint8_t lastMessageIdReceivedByRemote;
    // Id of the last message id sent. Should be incremented for each message sent. There is one
    // caveat though: if there is a message in the buffer for which there was a send timeout, this variable should be
    // set to the value of lastMessageIdReceivedByRemote and all the buffered messages purged.
    uint8_t lastMessageIdSent;

    // Whether an ACK was sent for the last received message.
    bool ackSent;
    // Id of the last received message. This is used for sending ACK messages.
    uint8_t lastMsgRcvdId;

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
        ackSent(true), dirty(true), delayBetweenMsg(CONN_MIN_MSG_DELAY), connectionRetries(0), 
        connCallback(NULL), heartbeatRetries(0), lastMessageIdSent(0), lastMessageIdReceivedByRemote(0) {};
};

/**
 * Structure used to store a message in a send buffer.
 */
struct Message {
    
    // How much time to wait until message is sent again. 
    // When updating, this is typically current microcontroller time + Connection.delayBetweenMsg
    unsigned long retryTimeout;
    // How many retries were already made. The message can be considered dirty if the maximum threshold is reached. Therefore
    // the buffer position can be reused.
    uint8_t retries;

    // The payload of the message to be sent
    unsigned char payload[FRAME_SIZE];
    // A function pointer for a callback that will be called either on message delivery failure or success
    void (* messageCallback) (MessageStatus *);

    Message() : retryTimeout(0), retries(0), messageCallback(NULL){};

};

#endif	/* RF24NETWORKSTRUCTS_H */


