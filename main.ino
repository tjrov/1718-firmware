//This is the main code for the arduino I guess. 
//Including things we need to include
//Look at example code included with libraries for how to use APIs

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
  //runs 100 times / second
}
void slowLoop() {
  //runs 10 times / second
}
