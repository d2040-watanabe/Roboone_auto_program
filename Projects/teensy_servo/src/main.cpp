#include <Arduino.h>
#include <IcsHardSerialClass.h>

const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 1000;

IcsHardSerialClass krs(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);
// put function declarations here:
// int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  krs.begin();
  delay(20);
  krs.setSpd(0, 27);
  krs.setSpd(1, 27);
  krs.setStrc(0, 60);
  krs.setStrc(1, 60);
}

void loop() {
  // put your main code here, to run repeatedly:
  krs.setPos(0,9500);
  // delay(500);
  // krs.setPos(0,4500);
  // delay(500);
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }