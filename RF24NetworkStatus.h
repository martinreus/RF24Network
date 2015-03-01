/*
 * File:   RF24NetworkStatus.h
 *
 * Created on February 15, 2015, 4:27 PM
 */

#ifndef RF24NETWORKSTATUS_H
#define	RF24NETWORKSTATUS_H

// Feedback statuses for when establishing a connection to another node.
#define CONN_NO_SPACE_AVAILABLE -3 //connection control structure array on this node is full
#define CONN_REFUSED -2 // connection refused on target node
#define CONN_TIMEOUT -1 // connection timeout (for an unknown reason)
#define CONN_SUCCESSFUL 1 // connection attempt was successful
#define CONN_ALREADY_CONNECTED 0 //already connected

/**
 * A message status is given as argument when invoking the callback function passed as parameter of the
 * sendReliable method used for sending reliable messages to a node. The status code represents a status for whether
 * the message was successfully sent or not, and the msgId is the internally generated id of the message
 * that succeeded or failed to be sent.
 */
struct MessageStatus {
    int8_t statusCode;
    uint16_t msgId;

    MessageStatus(int8_t status, uint16_t messageId): statusCode(status), msgId(messageId) {};
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


