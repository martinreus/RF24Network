# martinreus 2015 - This is a work in progress! Lots of bugs and broken functionality!
( Shouldn't be commiting on master, by the way =P )

Currently implementing a simplified way to connect and reliably send packets over the original implementation of this excelent library.

## What has been developed so far:

- Establishing a connection between nodes, with heartbeat sending to detect connection loss or failure of one node.
- Sending bufferized messages. Handlers are used to check whether this message was received or not. Messages are resent if an ACK was not received.

## What is being developed / has to be done:

- Receiving and acking messages through the network.

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
