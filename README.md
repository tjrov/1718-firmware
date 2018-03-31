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

# Uploading to ROV computer
To install on the ROV, make sure you first get the ROV out of its current firmware and into the bootloader (the status LED should blink 5x on bootloader start and then stay off).

Now just run <code>make rov</code>. The status LED will blink 2x when the code finishes installing, the ROV will reboot, and the new firmware will run.

# Uploading to commercial Arduino UNO (for during development)
Compile the source. To install on a regular Arduino UNO board, just run <code>make upload</code>. This uploads over USB, the usual way.
