/******************************
   THRUSTER DEFINITIONS
               FORE
        [1]             [4]

   PORT [2]             [5] STARBOARD

        [3]             [6]
                AFT
 ******************************/

#define SERIAL_DEBUG 0 //change to 0 to stop Serial debug and allow Modbus operation
#define FAST_LOOP_INTERVAL 10000 //when disconnected from computer, system follows repeats loop at this interval. Otherwise, updates occur just after received messages
#define SERIAL_BAUD 250000
#define RS485_TIMEOUT 500

//This is the main code for the arduino I guess.
//Look at example code included with libraries for how to use APIs
///////////////////////////////////////////////////////////////////////////////references
#include "pindefs.h" //use quotes since file is in same directory
#include "registers.h"
#include <Wire.h> //Arduino I2C Library used by all libs below
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
#define STATE_CONNECTED 0b10101010 //communication works, and the user has armed the ROV so it can be driven around (fast flash status LED)
uint8_t rovState = STATE_DISCONNECTED;

//Timing variables
unsigned long lastReceivedCount = 0, lastLoopMicros = 0;
uint8_t count = 0; //count for slow loop
uint8_t blinkCount = 0; //count LED blink state
//Modbus variables
const uint8_t numRegisters = 29; //some number
uint16_t modbusRegisters[numRegisters] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t manipRegisters[4] = {0, 0, 0, 0}; //registers in uint8_t for manipulators
//MPU variables
uint16_t mpuPacketSize;
uint16_t mpuFifoCount;
uint8_t mpuFifoBuffer[64];
uint8_t mpuIntStatus;
Quaternion q;
VectorFloat gravity;
float ypr[3];

///////////////////////////////////////////////////////////////////////////////Object instantiations
//slave address 1, use Arduino serial port, TX_EN pin is defined in pindefs.h file
#define MPU6050_ADDR 0x68

Modbus rs485(1, 0, TX_EN);


Arduino_I2C_ESC *thrusters[6];

/*  
 *   Arduino_I2C_ESC(uint8_t(0x31), uint8_t(6)), 
  Arduino_I2C_ESC(uint8_t(0x31), uint8_t(6)) 
};
/*Arduino_I2C_ESC thruster2(uint8_t(0x2B), uint8_t(6));
Arduino_I2C_ESC thruster3(uint8_t(0x2C), uint8_t(6));
Arduino_I2C_ESC thruster4(uint8_t(0x2D), uint8_t(6));
Arduino_I2C_ESC thruster5(uint8_t(0x2E), uint8_t(6));
Arduino_I2C_ESC thruster6(uint8_t(0x2F), uint8_t(6));*/

MPU6050 mpu(MPU6050_ADDR); //the IMU @ 0x68
///////////////////////////////////////////////////////////////////////////////Function Declarations
/*******************************************************************************/
//function takes a signed 8-bit integer for the speed (range -128<->127)
//negative values go reverse, positive forwards
//other two params are the pins to use (use pindefs.h constants please)
/*******************************************************************************/
void setManipulator(int val, byte dir1, byte dir2) {
  if (val > 0) {
    analogWrite(dir1, val);
    digitalWrite(dir2, LOW);
  }
  else if (val < 0) {
    analogWrite(dir1, 256 + val);
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
  //digitalWrite(12, HIGH);
  //Write to thrusters the 6 16-bit #s
  for(int i = 0; i < 6; i++) {
    thrusters[i]->set((int16_t)modbusRegisters[THRUSTER_SPEED_REGISTERS]);
  }

  readIMU();
  //ypr is now a float array with RADIANS yaw, pitch, and roll in indices 0, 1, 2 respectively
  //each value ranges between positive and negative pi
  for(int i = 0; i < 3; i++) {
    modbusRegisters[AVIONICS_REGISTERS + i] = map(ypr[0], -PI, PI, 0, 65535);
  }

  /*Serial.print(ypr[0]);
    Serial.print('\t');
    Serial.print(ypr[1]);
    Serial.print('\t');
    Serial.println(ypr[2]);*/

  //Place things in array

  //Report Errors
  //digitalWrite(12, LOW);
}
void slowLoop() { //runs 10 times / second
  setManipulator(map(modbusRegisters[MANIPULATOR_REGISTERS], 0, 65535, -256, 255), MOT1_DIR1, MOT1_DIR2);
  setManipulator(map(modbusRegisters[MANIPULATOR_REGISTERS + 1], 0, 65535, -256, 255), MOT2_DIR1, MOT2_DIR2);
  setManipulator(map(modbusRegisters[MANIPULATOR_REGISTERS + 2], 0, 65535, -256, 255), MOT3_DIR1, MOT3_DIR2);
  setManipulator(map(modbusRegisters[MANIPULATOR_REGISTERS + 3], 0, 65535, -256, 255), MOT4_DIR1, MOT4_DIR2);
  
  //set relays, headlights
  digitalWrite(RLY1_CTRL, modbusRegisters[RELAY_REGISTER] & (1 << 0));
  digitalWrite(RLY2_CTRL, modbusRegisters[RELAY_REGISTER] & (1 << 1));
  digitalWrite(LED_CTRL, modbusRegisters[13] & (1 << 2));

  //AnalogRead voltage sensor
  modbusRegisters[VOLTMETER_REGISTER] = analogRead(VOLT_MONITOR);
  
  //get thruster rpms and temperatures. .update() puts these values in the objects where they can be accessed by methods
  for(int i = 0; i < 6; i++) {
    thrusters[i]->update();
    //range is signed 16bit
    modbusRegisters[THRUSTER_RPM_REGISTERS + i] = map(thrusters[i]->rpm(), -32768, 32767, 0, 65535);
    //range is decimal 0 to 100
    modbusRegisters[THRUSTER_TEMP_REGISTERS + i] = (int)map(thrusters[i]->temperature(), 0.0, 100.0, 0.0, 65535.0);
    //set sensors to max value when ESC fails to notify control station
    if(!thrusters[i]->isAlive()) {
      modbusRegisters[THRUSTER_RPM_REGISTERS + i] = 65535;
      modbusRegisters[THRUSTER_TEMP_REGISTERS + i] = 65535;
    }
  }

  //update status LED
  digitalWrite(STATUS_LED, (rovState & (1 << blinkCount))); //set LED state to the nth bit of the ROV's state. The LED thus blinks differently in different states
  blinkCount++;
  if (blinkCount >= 8) blinkCount = 0;

  //update modbus status/error registers
  modbusRegisters[COMMUNICATION_REGISTER] = (rs485.getLastError() << 8) | (rs485.getErrCnt() & 0x00FF);
  /*From ModbusRtu.h, getLastError() returns the following values:
    ERR_NOT_MASTER = -1,
    ERR_POLLING = -2,
    ERR_BUFF_OVERFLOW = -3,
    ERR_BAD_CRC = -4,
    ERR_EXCEPTION = -5*/
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
}
//Required setup and loop functions
//Runs at power on
///////////////////////////////////////////////////////////////////////////////setup
void setup()
{
  initializePins();
  thrusters[0] = new Arduino_I2C_ESC((uint8_t)0x31, (uint8_t)6);
  thrusters[1] = new Arduino_I2C_ESC((uint8_t)0x2B, (uint8_t)6);
  thrusters[2] = new Arduino_I2C_ESC((uint8_t)0x2C, (uint8_t)6);
  thrusters[3] = new Arduino_I2C_ESC((uint8_t)0x2D, (uint8_t)6);
  thrusters[4] = new Arduino_I2C_ESC((uint8_t)0x2E, (uint8_t)6);
  thrusters[5] = new Arduino_I2C_ESC((uint8_t)0x2F, (uint8_t)6);
  //Transmit-only Serial debugging
#if SERIAL_DEBUG
  digitalWrite(TX_EN, LOW);
  Serial.begin(SERIAL_BAUD);
  Serial.println("initialized");
#else
  rs485.begin(SERIAL_BAUD); //250kbit/s RS-485
  rs485.setTimeOut(RS485_TIMEOUT); //if no comms occur for 500ms, ROV goes into disconnected state
#endif
  Wire.begin((int)400000); //400 kHz i2c
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
#if SERIAL_DEBUG
    Serial.println("IMU init");
#endif
  } else {
    //MPU initializing error
    //mpuStatus is the error code
#if SERIAL_DEBUG
    Serial.println("IMU failed");
#endif
  }
  //pinMode(12, OUTPUT);
  //digitalWrite(12, LOW);
}
///////////////////////////////////////////////////////////////////////////////loop
void loop()
{
  //state to disconnected if no comms have occurred during timeout period
  if (rs485.getTimeOutState()) {
    rovState = STATE_DISCONNECTED;
    //if motors are in a running state, stop them on lost connection
    modbusRegisters[0] = 0;
    modbusRegisters[1] = 0;
    modbusRegisters[2] = 0;
    modbusRegisters[3] = 0;
    modbusRegisters[4] = 0;
    modbusRegisters[5] = 0;
    modbusRegisters[6] = 0;
    modbusRegisters[7] = 0;
    modbusRegisters[13] = 0;
    modbusRegisters[14] = 0;
    lastReceivedCount = 0;
  } else {
    rovState = STATE_CONNECTED;
  }
  
  if ((rovState == STATE_DISCONNECTED && micros() - lastLoopMicros > FAST_LOOP_INTERVAL) || (rovState == STATE_CONNECTED && rs485.getInCnt() > lastReceivedCount)) {
    lastReceivedCount = rs485.getInCnt();
    lastLoopMicros = micros();
    fastLoop();
    count++;
    if (count >= 10) {
      slowLoop();
#if SERIAL_DEBUG
      Serial.println("slow");
#endif
      count = 0;
    }
#if SERIAL_DEBUG
    Serial.println(count);
#endif
  }
  //as often as possible, update Modbus registers with serial port data
#if SERIAL_DEBUG
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case 'w':
        modbusRegisters[0] = 1000;
        modbusRegisters[1] = 0;
        modbusRegisters[2] = 1000;
        modbusRegisters[3] = 1000;
        modbusRegisters[4] = 0;
        modbusRegisters[5] = 1000;
        break;
      case 's':
        modbusRegisters[0] = -1000;
        modbusRegisters[1] = 0;
        modbusRegisters[2] = -1000;
        modbusRegisters[3] = -1000;
        modbusRegisters[4] = 0;
        modbusRegisters[5] = -1000;
        break;
      case 'a':
        modbusRegisters[0] = -1000;
        modbusRegisters[1] = 0;
        modbusRegisters[2] = 0;
        modbusRegisters[3] = 1000;
        modbusRegisters[4] = 0;
        modbusRegisters[5] = 0;
        break;
      case 'd':
        modbusRegisters[0] = 1000;
        modbusRegisters[1] = 0;
        modbusRegisters[2] = 0;
        modbusRegisters[3] = -1000;
        modbusRegisters[4] = 0;
        modbusRegisters[5] = 0;
        break;
      case 'k':
        modbusRegisters[0] = 0;
        modbusRegisters[1] = 1000;
        modbusRegisters[2] = 0;
        modbusRegisters[3] = 0;
        modbusRegisters[4] = 1000;
        modbusRegisters[5] = 0;
        break;
      case 'm':
        modbusRegisters[0] = 0;
        modbusRegisters[1] = -1000;
        modbusRegisters[2] = 0;
        modbusRegisters[3] = 0;
        modbusRegisters[4] = -1000;
        modbusRegisters[5] = 0;
        break;
      case 'l':
        modbusRegisters[0] = 0;
        modbusRegisters[1] = 0;
        modbusRegisters[2] = 0;
        modbusRegisters[3] = 0;
        modbusRegisters[4] = 0;
        modbusRegisters[5] = 0;
        break;

    }
  }
#else
  rs485.poll(modbusRegisters, numRegisters);
#endif
}
