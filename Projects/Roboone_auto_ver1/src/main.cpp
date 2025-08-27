#include <IcsHardSerialClass.h>
#include <Wire.h>
#include <math.h>
#include "gyro.hpp"
#include "standard.hpp"
#include "control.hpp"
#include "motion.hpp"
#include "psd.hpp"
#include <climits>
#include <iostream>

void setup()
{
  delay(500); //電源投入後500ms待機
  // serial monitor
  Serial.begin(115200);

  // krs set up
  krs.begin();
  delay(20);
  krs.setSpd(0, 90);
  krs.setSpd(1, 90);
  krs.setStrc(0, 60);
  krs.setStrc(1, 60);

  // // for Gyro
  // Wire.begin();
  // Wire.setClock(400000);
  // Wire.setSDA(18);
  // Wire.setSCL(19);
  // // start up
  // Wire.beginTransmission(0x68);
  // Wire.write(0x6B);
  // Wire.write(0x00);
  // Wire.endTransmission();
  // // acc initial setup
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1C);
  // Wire.write(0x10);
  // Wire.endTransmission();
  // // gyro initial setup
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1B);
  // Wire.write(0x08);
  // Wire.endTransmission();
  // //LPF setup
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1A);
  // Wire.write(0x05);
  // Wire.endTransmission();
  // delay(10);
  
  
  //Wire.onReceive();
//   MsTimer2::set(10, ReadGyro);
//   MsTimer2::start();

}

void loop()
{
    //ReadGyro();
    //degdeg();
    //delay(100);
    //degdeg();
    
    //Set_Pos_All(4500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    // krs.setPos(0,4500);
    // delay(1000);
    // // Set_Pos_All(9500, 9500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    // krs.setPos(0,9500);
    // delay(1000);

    psd(21);

    if (psd(21) > 10){
      krs.setPos(0,4500);
      krs.setPos(1,4500);
      // Set_Pos_All(4500, 4500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }else{
      krs.setPos(0,7500);
      krs.setPos(1,7500);
      // Set_Pos_All(7500, 7500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

}