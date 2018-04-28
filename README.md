# 1718-firmware
Firmware for 2017-2018 season.

Intended to run on an ATMega328P. 5V, 16MHz clock.

Please see considerations in tjrov/bootloader when adding changes to ensure the bootloader will continue working with any new code.

# Compiling
To compile, you need some stuff other than the Makefile here. The package arduino is the Arduino IDE and the software tools for compiling that come with it. Arduino-mk is a dependency to the current project's makefile that knows how to compile arduino C.

<code>sudo apt-get install arduino arduino-mk</code>

In the Makefile, configure the options variables at the top to compile properly.

Run <code>make</code> to compile the code into binaries, which go into a subfolder of the current directory.

# Adding libraries
Put the library folder in the Arduino IDE installation directory/libraries (usually /usr/share/arduino/libraries). In the Makefile, append the complete name of this library to the ARDUINO_LIBS variable, with a space separating the new name from the previous ones.

# Uploading to commercial Arduino UNO (for during development)
Compile the source. To install on a regular Arduino UNO board, just run <code>make upload</code>. This uploads over USB, the usual way.

# Uploading to ROV computer
To install on the ROV, bring the ROV to the surface and attach the ROV flasher box to the ROV's firmware flashing port. Run make rov to install the code.

# About the ROV flasher box
It's just an Arduino UNO running the ArduinoISP sketch. The Arduino inside the box converts USB serial data at 19200 baud into ISP data that can be used to program another Arduino chip inside the ROV.

The LED codes go like this:

Green - Slow fade in and out indicates ready state
Yellow - Data is being transferred through the box (either received or transmitted)
Red - An error has occured. Some errors will occur without lighting this lamp and still stop the flash from completing properly.
Blue (on ROV computer) - when data is reaching the ROV over the ISP lines, this LED will flash (because it is connected to the ISP clock line as well as a digital output of the ROV computer)
