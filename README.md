Introduction
============
This project aims to provide hardware and software to build a versatile and generic device that can be used to connect all sorts of sensors and actuators to a CANopen network. My main intention is to use these devices in home automation applications.

The hardware is based on the ubiquitous ATmega 328 microcontroller found e.g. on the Arduino Uno. The mc is preflashed with the Arduino bootloader and can be programmed using an FTDI USB to serial adapter.

Connection to the CAN bus is established via an MCP2515 CAN controller.

The free input and output pins of the ATmega are not routed to the available connectors. Instead, the board features a prototyping area that can be used to customize the device to connect to any sensor or actuator. This allows to mass-produce the board and use it for many different applications.

The software is currently based on the www.canfesitval.org CANopen stack adapted to run on the ATmega 328. For sending and receiving of CAN messages a modified version of the https://github.com/Seeed-Studio/CAN_BUS_Shield library is used. This example code is configured to let the user read 6 digital inputs and write 4 digital outputs. The software has to be compiled using the avr-gcc via a makefile.

The CANopen stack is a protocol that runs ontop of the CAN communication layer and facilitates the acces to device parameters and input/ouput data in a standardized way. Check out the following links to get a quick start on the basic concept:

* http://en.wikipedia.org/wiki/CANopen
* http://www.canopensolutions.com/english/about_canopen/about_canopen.shtml
* http://www.can-cia.org/index.php?id=503

In the future I plan to rewrite the software as an Arduino library so that the Arduino IDE can be used to configure the device behaviour, e.g. read special sensor protocols or to control pwm outputs.

Getting Started
===============
The software is not restricted to run on the supplied controller board. Though I haye not tested it, the software can be run on most Arduino boards with a CAN bus shield. But beware that in its current incarnation the SPI chip selection of the MCP2515 is not connected to PB2 (Arduino pin 10). If you want to use the software with CAN bus shields you will probably have to change the following line in mcp_can.c to match your chip select configuration:
```c
#define DDR_CS DDRD
#define PORT_CS PORTD
#define P_CS PD7
```

For the standard SS pin you will have to change these to
```c
#define DDR_CS DDRB
#define PORT_CS PORTB
#define P_CS PB2
```
