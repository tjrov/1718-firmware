//This is the main code for the arduino I guess. 
//Including things we need to include
//Look at example code included with libraries for how to use APIs

#include "pindefs.h" //use quotes since file is in same directory
#include <Wire.h> //Arduino I2C Library used by all libs below
//for depth sensor over i2c bus
//Pressure measured = density * gravity constant * depth + air pressure
#include <SparkFun_MS5803_I2C.h> //Henry's Pin Libraries??
//for orientation sensor using MPU-6050 chip over i2c bus
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
//for ESC (BlueRobotics Basic ESC) control over the i2c bus
#include <Arduino_I2C_ESC.h>
//For ModBus RTU
//Read about that here:
//https://en.wikipedia.org/wiki/Modbus
//Github: Modbus-Master-Slave-for-Arduino
#include <ModbusRtu.h>

//Defining of Variables
//Timing-related
unsigned long lastLoopMicros;
byte count = 0;
//Communication-related
byte numRegisters = 10; //some number
uint16_t modbusRegisters[numRegisters] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/*Modbus Register contents (add more as needed, these are the bare minimum to control robot functions
The library requires an array of UNsigned 16-bit integers, but we can use them as needed
The purpose of each element of the array is listed by index here:
//=============Fast loop data (High priority) =================
0. Thruster Speed #1 (signed 16-bit integers. Negative is reverse, positive is forward)
1. Thruster Speed #2 (Can be sent directly to ESCs over i2c bus)
2. Thruster Speed #3
3. Thruster Speed #4
4. Thruster Speed #5
5. Thruster Speed #6
6. Manipulator Speeds #1, #2 (8-bit signed integer. High byte controls one motor, 
                              low byte controls other motor)
7. Manipulator Speeds #3, #4 (Negative is reverse, positive is forward, abs value is speed
                             see pindefs.h for guidelines controlling manipulators)
8. Voltage @ ROV (see pindefs.h for analog pin to use)
9. Yaw (these are floating points ranging -180 <-> 180 degrees. We don't need more than one decimal
        place precision)
10. Pitch
11. Roll
12. Depth Reading (Use conversion methods in sensor library to get mbars and send that up for conversion to depth based on weather conditions)
//==============Slow loop data (Low priority; you don't have to check these unless you want to)================
13. Booleans for the two relay control pins' states, headlights state, buzzer (for OBS activation) state (use each bit as a boolean)
14. Thruster RPM #1
15.Thruster RPM #2
16.Thruster RPM #3
17.Thruster RPM #4
18.Thruster RPM #5
19.Thruster RPM #6
20. Thruster Temp #1
21.Thruster Temp #2
22.Thruster Temp #3
23.Thruster Temp #4
24.Thruster Temp #5
25.Thruster Temp #6
26.Water temp (From depth sensor, which is in contact w/ water)

//more exist. see slack post with listing in #programming dated Feb 25th


//Object instantiations
//slave address 1, use Arduino serial port, TX_EN pin is defined in pindefs.h file
Modbus rs485(1, 0, TX_EN);

//Required setup and loop functions
//Runs at power on
void setup()
{
  rs485.begin(250000); //250kbit/s RS-485
  Wire.begin(400000); //400 kHz i2c
}
//Runs after setup() in an infinite loop
void loop()
{
  if(count == 0) {
    //10 times a second
    slowLoop();
  }
  if(micros() - lastLoopMicros > 10000) {
    //every 10000 microseconds, or every 10 milliseconds, or 100 times a second
    lastLoopMicros = micros();
    fastLoop();
    count++;
    if(count >= 10) {
      count = 0;
    }
  }
  //as often as possible, update Modbus registers with serial port data
  rs485.poll(modbusRegisters, numRegisters);
}
//Function Declarations go here 
void fastLoop() {
  //Msg state OK?
  if()//!msgState)
  {
    //report error, set debug LED state to error, exit loop
  }
  //Set debug LED state to connected

  //Write to thrusters the 6 16-bit #s 

  // Set 4 manipulators motor speeds and direction
 
  if()//oddIteration)
  {
    //Start depth reading
    //oddIteration = false;
  }
  else
  {
    //oddIteration = true;
  }

  //Read out data from sensor

  //Get IMU DMP readings

  //Convert

  //Map to 16-bit int

  //Put above^ in register array

  //AnalogRead voltage sensor
  if()//current voltage < some number
  {
    //Stop all motors
  }
  //Place things in array

  //slowTimer++;
  //Report Errors
  //runs 100 times / second
}
void slowLoop() {
  //runs 10 times / second
}
//function takes a signed 8-bit integer for the speed (range -128<->127)
//negative values go reverse, positive forwards
//other two params are the pins to use (use pindefs.h constants please)
void setManipulator(int8_t val, byte dir1, byte dir2) {
  if(val > 0) {
    analogWrite(dir1, (val*2));
    digitalWrite(dir2, LOW);
  }
  else if(val < 0) {
    analogWrite(dir1, (257+(val*2)));
    digitalWrite(dir2, HIGH);
  } else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
  }
}
