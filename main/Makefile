#SPECIFY THESE TO COMPILE
#where Arduino IDE data is and where arduino-mk is installed
ARDUINO_DIR  = /usr/share/arduino

#list of libraries this code depends on, separated by spaces
ARDUINO_LIBS = Wire SparkFun_MS5803_I2C I2Cdev MPU6050_6Axis_MotionApps20 Arduino_I2C_ESC ModbusRtu

#find non-standard libraries in 1718-firmware/libraries
USER_LIB_DIR = /home/pi/1718-firmware/libraries

#port to which RS-485 transceiver is attached
ARDUINO_PORT = /dev/ttyUSB0

#directory of Uploader.java code (comes with tjrov/bootloader github repo)
UPLOADER_JAVA_DIR = /home/pi/bootloader/Uploader
#END VARIABLES

#use an UNO board to represent the ROV computer
BOARD_TAG    = uno

include /usr/share/arduino/Arduino.mk

#custom installation target for RS-485
rov:
	cd $(UPLOADER_JAVA_DIR); echo "Running Uploader.java to load "$(TARGET_HEX) "onto ROV by port "$(ARDUINO_PORT); \
	java -cp ".:./jars/jSerialComm-1.3.11.jar" Uploader $(dir $(realpath $(firstword $(MAKEFILE_LIST))))$(TARGET_HEX) $(ARDUINO_PORT)
