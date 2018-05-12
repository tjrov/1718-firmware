/******************************
 * THRUSTER DEFINITIONS
 *             FORE
 *      [1]             [4]
 *  
 * PORT [2]             [5] STARBOARD
 * 
 *      [3]             [6]
 *              AFT
 ******************************/

#define SERIAL_DEBUG 1 //change to 0 to stop Serial debug, which blocks Modbus operation

//This is the main code for the arduino I guess.
//Look at example code included with libraries for how to use APIs
///////////////////////////////////////////////////////////////////////////////references
#include "pindefs.h" //use quotes since file is in same directory
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
//ROV errors for Status & Control Register
#define ERROR_NONE 0
//others as needed
uint8_t rovError = ERROR_NONE;

//Timing variables
unsigned long lastLoopMicros;
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
  13. Booleans for the two relay control pins' states (RLY1 is buzzer), headlights state (use each bit as a boolean)
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
  27.ROV Status: holds error codes
  28.Modbus Status Register. High byte holds getLastError(), Low byte holds getErrCnt().
*/
///////////////////////////////////////////////////////////////////////////////Object instantiations
//slave address 1, use Arduino serial port, TX_EN pin is defined in pindefs.h file
#define MPU6050_ADDR 0x68

Modbus rs485(1, 0, TX_EN);

Arduino_I2C_ESC thruster1(uint8_t(0x31), uint8_t(6));
Arduino_I2C_ESC thruster2(uint8_t(0x2B), uint8_t(6));
Arduino_I2C_ESC thruster3(uint8_t(0x2C), uint8_t(6));
Arduino_I2C_ESC thruster4(uint8_t(0x2D), uint8_t(6));
Arduino_I2C_ESC thruster5(uint8_t(0x2E), uint8_t(6));
Arduino_I2C_ESC thruster6(uint8_t(0x2F), uint8_t(6));

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
  //Write to thrusters the 6 16-bit #s
  thruster1.set(modbusRegisters[0]);
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

  //Place things in array

  //Report Errors
}
void slowLoop() { //runs 10 times / second
  //set relays, headlights
	digitalWrite(RLY1_CTRL, modbusRegisters[13] & (1<<0));
	digitalWrite(RLY2_CTRL, modbusRegisters[13] & (1<<1));
	digitalWrite(LED_CTRL, modbusRegisters[13] & (1<<2));

  //get thruster rpms and temperatures. .update() puts these values in the objects where they can be accessed by methods
	thruster1.update();
	modbusRegister[14] = (int)(thruster1.temperature()*10);
	modbusRegister[15] = (int)(thruster2.temperature()*10);
	modbusRegister[16] = (int)(thruster3.temperature()*10);
	modbusRegister[17] = (int)(thruster4.temperature()*10);
	modbusRegister[18] = (int)(thruster5.temperature()*10);
	modbusRegister[19] = (int)(thruster6.temperature()*10);
	modbusRegister[20] = thruster1.rpm();
	modbusRegister[21] = thruster2.rpm();
	modbusRegister[22] = thruster3.rpm();
	modbusRegister[23] = thruster4.rpm();
	modbusRegister[24] = thruster5.rpm();
	modbusRegister[25] = thruster6.rpm();
	
	
  //update status LED
  digitalWrite(STATUS_LED, (rovState & (1 << blinkCount))); //set LED state to the nth bit of the ROV's state. The LED thus blinks differently in different states
  blinkCount++;
  if (blinkCount >= 8) blinkCount = 0;
	
  //update modbus status/error registers
	modbusRegisters[28] = (rs485.getLastError() << 8) | (rs485.getErrCnt() & 0x00FF)
	/*From ModbusRtu.h, getLastError() returns the following values:
	ERR_NOT_MASTER = -1,
	ERR_POLLING = -2,
	ERR_BUFF_OVERFLOW = -3,
	ERR_BAD_CRC = -4,
	ERR_EXCEPTION = -5*/
		
		//update ROV status/error registers
		modbusRegisters[27] = rovError;
		
//state to disconnected if no comms have occurred during timeout period
		if(rs485.getTimeOutState()) {
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
		} else {
			rovState = STATE_CONNECTED;
		}
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
  //Transmit-only Serial debugging
  #if SERIAL_DEBUG
  digitalWrite(TX_EN, HIGH);
  Serial.begin(250000);
  #else
  rs485.begin(250000); //250kbit/s RS-485
  rs485.setTimeout(500); //if no comms occur for 500ms, ROV goes into disconnected state
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
  } else {
    //MPU initializing error
    //mpuStatus is the error code
  }
}
///////////////////////////////////////////////////////////////////////////////loop
void loop()
{
  //spin();
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
#if SERIAL_DEBUG
#else
  rs485.poll(modbusRegisters, numRegisters);
#endif
}
