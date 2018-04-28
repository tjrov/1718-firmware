#SPECIFY THESE TO COMPILE
#where Arduino IDE data is and where arduino-mk is installed
ARDUINO_DIR  = /usr/share/arduino

#port to which programmer box is attached. With 2 XBOX controllers and the RS-485 transceiver already plugged in, this should work
ARDUINO_PORT = /dev/ttyUSB3

#use an UNO board to represent the ROV computer
BOARD_TAG    = uno

include /usr/share/arduino/Arduino.mk

#custom installation target for programmer box
rov:
	avrdude -p m328p -c avrisp -b 19200 -P $(ARDUINO_PORT) -v -U flash:w:$(dir $(realpath $(firstword $(MAKEFILE_LIST))))$(TARGET_HEX):i
