/*************************************************** 
  This is an example for Servo driver
  
   ****************************************************/

#include <Wire.h>
#include <ZPCA9685.h>

// called this way, it uses the default address 0x40, see begin to setup an address
ZPCA9685 servos = ZPCA9685();
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  servos.begin(&Wire,0x43);  
  for (servonum = 0; servonum < 16; servonum++)
  servos.attach(servonum);
   delay(10);
}

void loop() {
  // Drive each servo one at a time
  
  for (uint16_t angle = 0; angle < 180; angle++) {
    Serial.println(angle);
     for (servonum = 0; servonum < 16; servonum++)
    servos.write(servonum,  angle);
    delay(20);
  }

  delay(500);
  for (uint16_t angle = 180; angle > 0; angle--) {
    Serial.println(angle);
     for (servonum = 0; servonum < 16; servonum++)
    servos.write(servonum, angle);
   delay(20);
  }
  delay(500);


}
