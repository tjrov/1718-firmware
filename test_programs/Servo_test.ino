#include "pindefs.h"

void setup() {
  pinMode(MOT1_DIR1, OUTPUT);
  pinMode(MOT1_DIR2, OUTPUT);
  pinMode(TX_EN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(TX_EN, LOW);
  Serial.begin(115200);
}

void loop() {
  while(!Serial.available());
  int spd = Serial.parseInt();
  digitalWrite(STATUS_LED, HIGH);
  delay(50);
  digitalWrite(STATUS_LED, LOW);
  setMotor(spd, MOT1_DIR1, MOT1_DIR2);
  delay(2000);
}

//function takes a signed 8-bit integer for the speed (range -128<->127)
//negative values go reverse, positive forwards
//other two params are the pins to use (use pindefs.h constants please)
void setMotor(int8_t val, byte dir1, byte dir2) {
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
