//This is the main code for the arduino I guess. 
//Including things we need to include
#include <Wire.h> //I2C Library
#include <SparkFun_MS5803_I2C.h> //Henry's Pin Libraries??
//Defining of Variables

//Setup and loop functions
void setup()
{
  Serial.begin(9600);
  Wire.begin();
}
void loop()
{
  Serial.println("kill me now");// wau
}
//Function Declarations go here 
