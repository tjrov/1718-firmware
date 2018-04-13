///////////////////////////////////////////////////////////////////////////////config & constants
#define GRAVITY_ACCELERATION 9.81415 //For Fairfax. Used vertical component from here:
//http://www.wolframalpha.com/widgets/view.jsp?id=e856809e0d522d3153e2e7e8ec263bf2
#define BAROMETRIC_PRESSURE 100000 //Varies greatly based on weather. 100kPa is a ballpark.
#define WATER_DENSITY 0.99272 //chlorine reduces density of pool water
#define DEPTH_CALC_CONSTANT (1/WATER_DENSITY/GRAVITY_ACCELERATION)

//when this is NOT commented out, debug Serial.print statements will work but Modbus will not work
#define DEBUG


//This is the main code for the arduino I guess.
//Look at example code included with libraries for how to use APIs
///////////////////////////////////////////////////////////////////////////////references
#include "pindefs.h" //use quotes since file is in same directory
#include <Wire.h> //Arduino I2C Library used by all libs below
//for depth sensor over i2c bus
//Pressure measured = density * gravity constant * depth + air pressure
#include "SparkFun_MS5803_I2C.h"
//for orientation sensor using MPU-6050 chip over i2c bus
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//for ESC (BlueRobotics Basic ESC) control over the i2c bus
#include "Arduino_I2C_ESC.h"
//For ModBus RTU
//Read about that here:
//https://en.wikipedia.org/wiki/Modbus
//Github: Modbus-Master-Slave-for-Arduino
#include "ModbusRtu.h"
///////////////////////////////////////////////////////////////////////////////variables
//ROV states for Status & Control Register; binary values specify LED flash patterns over time
#define STATE_DISCONNECTED 0b00000001 //no communication to surface or a communication error (pulse status LED once in a while)
#define STATE_CONNECTED_DISARMED 0b00001111 //communication works, but the user has disarmed the ROV so it won't drive around (slow flash status LED)
#define STATE_CONNECTED_ARMED 0b10101010 //communication works, and the user has armed the ROV so it can be driven around (fast flash status LED)
uint8_t rovState = STATE_DISCONNECTED;
//ROV errors for Status & Control Register
#define ERROR_NONE 0
//others as needed
uint8_t rovError = ERROR_NONE;

//Timing-related
unsigned long lastLoopMicros;
byte count = 0; //count for slow loop
byte blinkCount = 0; //count LED blink state
bool oddIteration; //is the iteration odd
//Communication-related
const byte numRegisters = 29; //some number
uint16_t modbusRegisters[numRegisters] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t manipRegisters[4] = {0, 0, 0, 0}; //registers in uint8_t for manipulators
uint8_t msgState = 0; //is the msgState ok?
//Hardware-related
double myVoltage = 0.0;
float myPressure = 0;
uint16_t myDepth = 0;
float myTemperature = 0;
//MPU variables
uint16_t mpuPacketSize;
uint16_t mpuFifoCount;
uint8_t mpuFifoBuffer[64];
uint8_t mpuIntStatus;
Quaternion q;
VectorFloat gravity;
float ypr[3];
///////////////////////////////////////////////////////////////////////////////modbusRegister explain
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
  27.ROV Status & Control Register. High byte holds error codes (can't communicate with an ESC, that sort of thing)
				Low byte can be changed to arm/disarm ROV, reset board, enter bootloader, other stuff
  28.Modbus Status Register. High byte holds getErrCount(), Low byte holds getLastError().
*/
///////////////////////////////////////////////////////////////////////////////Object instantiations
//slave address 1, use Arduino serial port, TX_EN pin is defined in pindefs.h file
#define MS5803_ADDR 0x76
#define MPU6050_ADDR 0x68
#ifndef DEBUG
Modbus rs485(1, 0, TX_EN);
#endif
Arduino_I2C_ESC thruster1(uint8_t(0x2A), uint8_t(6));
Arduino_I2C_ESC thruster2(uint8_t(0x2B), uint8_t(6));
Arduino_I2C_ESC thruster3(uint8_t(0x2C), uint8_t(6));
Arduino_I2C_ESC thruster4(uint8_t(0x2D), uint8_t(6));
Arduino_I2C_ESC thruster5(uint8_t(0x2E), uint8_t(6));
Arduino_I2C_ESC thruster6(uint8_t(0x2F), uint8_t(6));
MS5803 ms5803(ms5803_addr(MS5803_ADDR)); //0x76
MPU6050 mpu(MPU6050_ADDR); //the IMU @ 0x68
///////////////////////////////////////////////////////////////////////////////Function Declarations
/*******************************************************************************/
//function takes a signed 8-bit integer for the speed (range -128<->127)
//negative values go reverse, positive forwards
//other two params are the pins to use (use pindefs.h constants please)
/*******************************************************************************/
void setManipulator(int8_t val, byte dir1, byte dir2) {
  if (val > 0) {
    analogWrite(dir1, (val * 2));
    digitalWrite(dir2, LOW);
  }
  else if (val < 0) {
    analogWrite(dir1, (257 + (val * 2)));
    digitalWrite(dir2, HIGH);
  } else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
  }
}
void readIMU() {
  //wait for IMU to have readings ready
  while (mpuFifoCount < mpuPacketSize) mpuFifoCount = mpu.getFIFOCount();
  mpuIntStatus = mpu.getIntStatus();
  mpuFifoCount = mpu.getFIFOCount();
  //check for overflow of IMU data buffer
  if ((mpuIntStatus & 0x10) || mpuFifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    //collect MPU data
    mpu.getFIFOBytes(mpuFifoBuffer, mpuPacketSize);
    mpuFifoCount -= mpuPacketSize;
  }
  //Get IMU DMP readings aka MPU6050 readings, convert, map to 16-bit int, put in register array
  mpu.dmpGetQuaternion(&q, mpuFifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}
void fastLoop() { //runs 100 times a second
  //Check if msg state is ok
#ifndef DEBUG
  if (msgState <= 4)
  {
    rovState = STATE_DISCONNECTED;
    return; //break
  }
#endif
  //Write to thrusters the 6 16-bit #s
  thruster1.set(modbusRegisters[0]);//yo i don't know if this is right???
  thruster2.set(modbusRegisters[1]);
  thruster3.set(modbusRegisters[2]);
  thruster4.set(modbusRegisters[3]);
  thruster5.set(modbusRegisters[4]);
  thruster6.set(modbusRegisters[5]);
  //converting to uint8_ts
  manipRegisters[0] = (uint8_t)((modbusRegisters[6] & 0xFF00) >> 8);
  manipRegisters[1] = (uint8_t)(modbusRegisters[6] & 0x00FF);
  manipRegisters[2] = (uint8_t)((modbusRegisters[7] & 0xFF00) >> 8);
  manipRegisters[3] = (uint8_t)(modbusRegisters[7] & 0x00FF);
  // Set 4 manipulators motor speeds and direction
  setManipulator(manipRegisters[0], MOT1_DIR1, MOT1_DIR2);
  setManipulator(manipRegisters[1], MOT2_DIR1, MOT2_DIR2);
  setManipulator(manipRegisters[2], MOT3_DIR1, MOT3_DIR2);
  setManipulator(manipRegisters[3], MOT4_DIR1, MOT4_DIR2);

  //AnalogRead voltage sensor
  modbusRegisters[8] = analogRead(VOLT_MONITOR);
  /*if (myVoltage < 2.5) //If voltage is lower than 2.5V
  {
    setManipulator(0, MOT1_DIR1, MOT1_DIR2);
    setManipulator(0, MOT2_DIR1, MOT2_DIR2);
    setManipulator(0, MOT3_DIR1, MOT3_DIR2);
    setManipulator(0, MOT4_DIR1, MOT4_DIR2);//Stop all motors
  }*/

  readIMU();
  //ypr is now a float array with RADIANS yaw, pitch, and roll in indices 0, 1, 2 respectively
  //each value ranges between positive and negative pi
  modbusRegisters[9] = map(ypr[0], -PI, PI, 0, 65535);
  modbusRegisters[10] = map(ypr[1], -PI, PI, 0, 65535);
  modbusRegisters[11] = map(ypr[2], -PI, PI, 0, 65535);

  /*Serial.print(ypr[0]);
    Serial.print('\t');
    Serial.print(ypr[1]);
    Serial.print('\t');
    Serial.println(ypr[2]);*/

  myPressure = ms5803.getPressure(ADC_1024); //returns Pascals (N/m^2), precise to 0.4mbar or about 4mm depth change
  myPressure -= BAROMETRIC_PRESSURE; //ignore the atmosphere, we only care about the pool water pressure here
  myDepth = myPressure * DEPTH_CALC_CONSTANT; //Assumption that water density is 1 g/cm^3
  //myDepth is in meters
  modbusRegisters[12] = map(myDepth, 0, 20, 0, 65535);

  //Place things in array

  //Report Errors
}
void slowLoop() { //runs 10 times / second
  //do things with the booleans

  //get thruster rpm

  //get thruster temperatures

  //get water temperatures
  myTemperature = ms5803.getTemperature(CELSIUS, ADC_1024);

  //update status LED
  digitalWrite(STATUS_LED, (rovState & (1 << blinkCount))); //set LED state to the nth bit of the ROV's state. The LED thus blinks differently in different states
  blinkCount++;
  if (blinkCount >= 8) blinkCount = 0;
}

void initializePins() {
  //default to LOW state, don't need to digitalWrite LOW
  pinMode(STATUS_LED, OUTPUT);
  pinMode(TX_EN, OUTPUT);
  pinMode(MOT1_DIR1, OUTPUT);
  pinMode(MOT1_DIR2, OUTPUT);
  pinMode(MOT2_DIR1, OUTPUT);
  pinMode(MOT2_DIR2, OUTPUT);
  pinMode(MOT3_DIR1, OUTPUT);
  pinMode(MOT3_DIR2, OUTPUT);
  pinMode(MOT4_DIR1, OUTPUT);
  pinMode(MOT4_DIR2, OUTPUT);
  pinMode(RLY1_CTRL, OUTPUT);
  pinMode(RLY2_CTRL, OUTPUT);
  pinMode(LED_CTRL, OUTPUT);
  pinMode(SPK, OUTPUT);
}
//Required setup and loop functions
//Runs at power on
///////////////////////////////////////////////////////////////////////////////setup
void setup()
{
  initializePins();
#ifdef DEBUG
  digitalWrite(TX_EN, HIGH);
  Serial.begin(250000);
#else
  rs485.begin(250000); //250kbit/s RS-485
#endif
  Wire.begin((int)400000); //400 kHz i2c
  ms5803.begin();
  mpu.initialize();
  //mpu.testConnection() and report any errors
  uint8_t mpuStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (mpuStatus == 0) {
    //initialized MPU alright
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    mpuPacketSize = mpu.dmpGetFIFOPacketSize();
  } else {
    //MPU initializing error
    //mpuStatus is the error code
  }
}
///////////////////////////////////////////////////////////////////////////////loop
void loop()
{
  if (count == 0) {
    //10 times a second
    slowLoop();
  }
  if (micros() - lastLoopMicros > 10000) {
    //every 10000 microseconds, or every 10 milliseconds, or 100 times a second
    lastLoopMicros = micros();
    fastLoop();
    count++;
    if (count >= 10) {
      count = 0;
    }
  }
  //as often as possible, update Modbus registers with serial port data
#ifndef DEBUG
  msgState = rs485.poll(modbusRegisters, numRegisters);
#endif
}
