/*
 * File:   RF24NetworkStatus.h
 *
 * Created on February 15, 2015, 4:27 PM
 */

#ifndef RF24NETWORKSTATUS_H
#define	RF24NETWORKSTATUS_H

// Reserved message types sent between nodes. Message types ranging from 0-127 are user defined messages.
#define MSG_ACK 130
#define MSG_SYN 131
#define MSG_SYN_ACK 132
#define MSG_BUFFER_FULL 133
#define MSG_CONN_REFUSED 134
#define MSG_DISCONNECT 135
#define MSG_DISCONNECT_ACK 136
#define MSG_NOT_CONNECTED 137

// Feedback statuses for when establishing a connection to another node.
#define CONN_REFUSED -2
#define CONN_TIMEOUT -1
#define CONN_SUCCESS 1

/**
 * A message status is given as argument when invoking the callback function passed as parameter of the
 * sendReliable method used for sending reliable messages to a node. The status code represents a status for whether
 * the message was successfully sent or not, and the msgId is the internally generated id of the message
 * that succeeded or failed to be sent.
 */
struct MessageStatus {
    uint8_t statusCode;
    uint16_t msgId;

    MessageStatus(uint8_t status, uint16_t messageId): statusCode(status), msgId(messageId) {};
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

#endif	/* RF24NETWORKSTATUS_H */


