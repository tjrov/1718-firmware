//Simple i2c scanner for ROV
//Gives addresses of connected devices
//If it hangs on "scanning", could be:
//-Short of SCL or SDA high or low single-ended wiring anywhere on ROV
//-Broken connection of differential or single-ended wiring on ROV

#include <Wire.h>
 
 
void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(100);
  Serial.begin(115200);
  Wire.begin();
  //Sometimes, clock speed may be an issue. The ROV uses 400kHz i2c bus, but try 100kHz if there are problems
  Wire.setClock(400000);
  //while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(1000);           // wait 5 seconds for next scan
}
