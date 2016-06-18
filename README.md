# martinreus 2015 - This is a work in progress!

Currently implementing a simplified way to connect and RELIABLY send packets (in a TCP-ish manner) over the original implementation of the excelent F24Network library. This library is optimized for usage with arduino, since its footprint is pretty low but still provides mesh networking capabilities between several nodes.
This library depends on the RF24 library (https://github.com/TMRh20/RF24.git), commit hash a46779fad0f52ffffd1e43a4fd1e3214aed17011

## What has been developed so far:

- Establishing a connection between nodes, with heartbeat message sending to detect connection loss or failure of one node.
- Sending bufferized messages. Handlers are used to check whether a message was received or not. Messages are automatically resent if an ACK was not received.
- Receiving and acking messages throughout the network.
- Read available messages from app layer.
- Test when message id returns to 0 after reaching maximum value.

## What remains to be done:

Main tasks involve testing of this library.
- Test when some of the timers reach maximum threshold, check if the network continues to work.
- Do a full stress test, sending and receiving messages from multiple nodes to ensure everything is working.
- Optimize library. Some of the operations that rely on iteration through the connection and message buffer array can be heavily optimized, increasing total throughput.

Please see the full documentation at http://tmrh20.github.io/RF24Network/

## Pin layout

The table below shows how to connect the the pins of the NRF24L01(+) to different boards.
CE and CSN are configurable.

| PIN | NRF24L01 | Arduino UNO | ATtiny25/45/85 [0] | ATtiny44/84 [1] |
|-----|----------|-------------|--------------------|-----------------|
|  1  |   GND    |   GND       |     pin 4          |    pin 14       |
|  2  |   VCC    |   3.3V      |     pin 8          |    pin  1       |
|  3  |   CE     |   digIO 7   |     pin 2          |    pin 12       |
|  4  |   CSN    |   digIO 8   |     pin 3          |    pin 11       |
|  5  |   SCK    |   digIO 13  |     pin 7          |    pin  9       |
|  6  |   MOSI   |   digIO 11  |     pin 6          |    pin  7       |
|  7  |   MISO   |   digIO 12  |     pin 5          |    pin  8       |
|  8  |   IRQ    |      -      |        -           |         -       |

[0] https://learn.sparkfun.com/tutorials/tiny-avr-programmer-hookup-guide/attiny85-use-hints
[1] http://highlowtech.org/?p=1695
