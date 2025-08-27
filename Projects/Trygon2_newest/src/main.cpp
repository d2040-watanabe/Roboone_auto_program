// #include <Wire.h>
// #include "kalman.hpp" // Source: https://github.com/TKJElectronics/KalmanFilter

// #define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// Kalman kalmanX; // Create the Kalman instances
// Kalman kalmanY;

// /* IMU Data */
// double accX, accY, accZ;
// double gyroX, gyroY, gyroZ;
// int16_t tempRaw;

// double gyroXangle, gyroYangle; // Angle calculate using the gyro only
// double compAngleX, compAngleY; // Calculated angle using a complementary filter
// double kalAngleX, kalAngleY;   // Calculated angle using a Kalman filter

// uint32_t timer;
// uint8_t i2cData[14]; // Buffer for I2C data

// // TODO: Make calibration routine

// const uint8_t IMUAddress = 0x68;   // AD0 is logic low on the PCB
// const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

// // uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop)
// // {
// //   return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
// // }

// uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop)
// {
//   Wire.beginTransmission(IMUAddress);
//   Wire.write(registerAddress);
//   Wire.write(data, length);
//   uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
//   if (rcode)
//   {
//     Serial.print(F("i2cWrite failed: "));
//     Serial.println(rcode);
//   }
//   return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
// }

// uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes)
// {
//   uint32_t timeOutTimer;
//   Wire.beginTransmission(IMUAddress);
//   Wire.write(registerAddress);
//   uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
//   if (rcode)
//   {
//     Serial.print(F("i2cRead failed: "));
//     Serial.println(rcode);
//     return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
//   }
//   Wire.requestFrom(IMUAddress, nbytes, (uint8_t) true); // Send a repeated start and then release the bus after reading
//   for (uint8_t i = 0; i < nbytes; i++)
//   {
//     if (Wire.available())
//       data[i] = Wire.read();
//     else
//     {
//       timeOutTimer = micros();
//       while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available())
//         ;
//       if (Wire.available())
//         data[i] = Wire.read();
//       else
//       {
//         Serial.println(F("i2cRead timeout"));
//         return 5; // This error value is not already taken by endTransmission
//       }
//     }
//   }
//   return 0; // Success
// }

// void setup()
// {
//   Serial.begin(115200);
//   Wire.begin();
// #if ARDUINO >= 157
//   Wire.setClock(400000UL); // Set I2C frequency to 400kHz
// #else
//   TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
// #endif

//   i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
//   i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
//   i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
//   i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
//   while (i2cWrite(0x19, i2cData, 4, false))
//     ; // Write to all four registers at once
//   while (i2cWrite(0x6B, 0x01, 4, true))
//     ; // PLL with X axis gyroscope reference and disable sleep mode

//   while (i2cRead(0x75, i2cData, 1))
//     ;
//   if (i2cData[0] != 0x68)
//   { // Read "WHO_AM_I" register
//     Serial.print(F("Error reading sensor"));
//     while (1)
//       ;
//   }

//   delay(100); // Wait for sensor to stabilize

//   /* Set kalman and gyro starting angle */
//   while (i2cRead(0x3B, i2cData, 6))
//     ;
//   accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
//   accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
//   accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

//   // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
//   // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
//   // It is then converted from radians to degrees
// #ifdef RESTRICT_PITCH // Eq. 25 and 26
//   double roll = atan2(accY, accZ) * RAD_TO_DEG;
//   double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
// #else // Eq. 28 and 29
//   double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
//   double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
// #endif

//   kalmanX.setAngle(roll); // Set starting angle
//   kalmanY.setAngle(pitch);
//   gyroXangle = roll;
//   gyroYangle = pitch;
//   compAngleX = roll;
//   compAngleY = pitch;

//   timer = micros();
// }

// void loop()
// {
//   /* Update all the values */
//   while (i2cRead(0x3B, i2cData, 14))
//     ;
//   accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
//   accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
//   accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
//   tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
//   gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
//   gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
//   gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
//   ;

//   double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
//   timer = micros();

//   // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
//   // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
//   // It is then converted from radians to degrees
// #ifdef RESTRICT_PITCH // Eq. 25 and 26
//   double roll = atan2(accY, accZ) * RAD_TO_DEG;
//   double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
// #else // Eq. 28 and 29
//   double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
//   double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
// #endif

//   double gyroXrate = gyroX / 131.0; // Convert to deg/s
//   double gyroYrate = gyroY / 131.0; // Convert to deg/s

// #ifdef RESTRICT_PITCH
//   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
//   if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
//   {
//     kalmanX.setAngle(roll);
//     compAngleX = roll;
//     kalAngleX = roll;
//     gyroXangle = roll;
//   }
//   else
//     kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

//   if (abs(kalAngleX) > 90)
//     gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
//   kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
// #else
//   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
//   if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
//   {
//     kalmanY.setAngle(pitch);
//     compAngleY = pitch;
//     kalAngleY = pitch;
//     gyroYangle = pitch;
//   }
//   else
//     kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

//   if (abs(kalAngleY) > 90)
//     gyroXrate = -gyroXrate;                          // Invert rate, so it fits the restriced accelerometer reading
//   kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
// #endif

//   gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
//   gyroYangle += gyroYrate * dt;
//   // gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
//   // gyroYangle += kalmanY.getRate() * dt;

//   compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
//   compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

//   // Reset the gyro angle when it has drifted too much
//   if (gyroXangle < -180 || gyroXangle > 180)
//     gyroXangle = kalAngleX;
//   if (gyroYangle < -180 || gyroYangle > 180)
//     gyroYangle = kalAngleY;

//     /* Print Data */
// #if 0 // Set to 1 to activate
//   Serial.print(accX); Serial.print("\t");
//   Serial.print(accY); Serial.print("\t");
//   Serial.print(accZ); Serial.print("\t");

//   Serial.print(gyroX); Serial.print("\t");
//   Serial.print(gyroY); Serial.print("\t");
//   Serial.print(gyroZ); Serial.print("\t");

//   Serial.print("\t");
// #endif

//   Serial.print(roll);
//   Serial.print("\t");
//   Serial.print(gyroXangle);
//   Serial.print("\t");
//   Serial.print(compAngleX);
//   Serial.print("\t");
//   Serial.print(kalAngleX);
//   Serial.print("\t");

//   Serial.print("\t");

//   Serial.print(pitch);
//   Serial.print("\t");
//   Serial.print(gyroYangle);
//   Serial.print("\t");
//   Serial.print(compAngleY);
//   Serial.print("\t");
//   Serial.print(kalAngleY);
//   Serial.print("\t");

// #if 0 // Set to 1 to print the temperature
//   Serial.print("\t");

//   double temperature = (double)tempRaw / 340.0 + 36.53;
//   Serial.print(temperature); Serial.print("\t");
// #endif

//   Serial.print("\r\n");
//   delay(2);
// }

#include <IcsHardSerialClass.h>
#include <TimerOne.h>
#include <MsTimer2.h>
#include <SPI.h>
#include <math.h>
#include <wire.h>
#include "standard.hpp"
#include "motion.hpp"
#include "control.hpp"
#include "gyro.hpp"
#include "Invkinema.hpp"
bool flag = false;

// bool flag_1 = false;
void setup()
{
  delay(500); //電源投入後500ms待機
  // serial monitor
  Serial.begin(1250000);

  // SPI setup
  //  SPI.setMOSI(11);                                     // MOSI pin11
  //  SPI.setMISO(12);                                     // MISO pin12
  //  SPI.setSCK(13);                                      // SCK pin27
  //  SPISettings settingsA(100000, LSBFIRST, SPI_MODE3);  // default 100000ps
  //  pinMode(MISO, INPUT);
  //  pinMode(10, OUTPUT);
  //  SPI.begin();
  //  SPI.beginTransaction(settingsA);

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
  // LPF setup
  // Wire.beginTransmission(0x68);
  // Wire.write(0x1A);
  // Wire.write(0x05);
  // Wire.endTransmission();
  delay(10);
  // Wire.onReceive();

  // krs set up
  krs.begin();
  delay(20);
  krs.setSpd(0, 127);
  krs.setSpd(1, 127);
  krs.setSpd(2, 127);
  krs.setSpd(3, 127);
  krs.setSpd(4, 127);
  krs.setSpd(5, 127);
  krs.setSpd(6, 127);
  krs.setSpd(7, 127);
  krs.setSpd(8, 127);
  krs.setSpd(9, 127);
  krs.setSpd(10, 127);
  krs.setSpd(11, 127);
  krs.setSpd(12, 127);
  krs.setStrc(0, 10);
  krs.setStrc(1, 10);
  krs.setStrc(2, 10);
  krs.setStrc(3, 10);
  krs.setStrc(4, 10);
  krs.setStrc(5, 10);
  krs.setStrc(6, 10);
  krs.setStrc(7, 10);
  krs.setStrc(8, 10);
  krs.setStrc(9, 10);
  krs.setStrc(10, 10);
  krs.setStrc(11, 10);
  krs.setStrc(12, 10);
  delay(10);
  // MsTimer2::set(10, ReadGyro);
  // MsTimer2::start();
  Timer1.initialize(10000); // 10ms
  Timer1.attachInterrupt(ReadVS2);
}

void loop()
{
  // ReadVS2();
  // ReadGyro();
  // FilterGyro();
  //
  if (BIT_select)
  {
    /*Set_Pos_All(0, 0, 0, 0, 0, 0, 0, 0, 0, -4500, -2000, 4500, 2000);
    delay(10);*/
    while (1)
    {
      Set_Pos_Free();
      // ReadVS2();
      if (BIT_start)
        break; //トルクオン
    }
    delay(100);
  }
  else if (BIT_down)
    Walk_Backward(); //後歩行
  else if (BIT_up)
    //Walk_Forward(); //前歩行
    Walk_5m();//5m走
  else if (BIT_right)
    Walk_Right(); //右歩行
  else if (BIT_left)
    Walk_Left(); //左歩行
  else if (BIT_sikaku)
    Attack_Left();//左攻撃
  else if (BIT_R1)
    Turn_Right(); //右旋回
  else if (BIT_L1)
    Turn_Left(); //左旋回
  else if (BIT_batu)
    Squat(); //しゃがみ（起上がり分岐含む）
  else if (BIT_sankaku)
    //Walk_5m();//5m走
    Attack_both(); //両手攻撃
  else if (BIT_R3)
    return_neutral_pos();
  else if (BIT_maru)
    Attack_Right();//右攻撃
  else if (BIT_R2)
    attack();
  else if (BIT_L2)
    stand_erect(); // 起き上がり分岐 
  else if (BIT_sankaku && BIT_R1)
    ;
  else if (BIT_L3)
    ;
  // else if (theta < -0.01)
  //   slip_F();
  // else if (theta > 0.05)
  //   slip_B();
  // else if (flag_1 == false)
  // {
  //   flag_1 = true;
  //   // Timer1.start();
  //   setPosInvLeg(0, 0, 0, 0, 20, 0, 0, 20);
  // }
  else if (flag == false)
  {
    flag = true;
    Neutral_Pos();
  }
  else
    Neutral_Pos2();
  // delay(20);
  // serial monitor
  // ReadVS2();
  //  if (BIT_select) Serial.println("select");
  //  else if (BIT_start) Serial.println("start");
  //  else if (BIT_down) Serial.println("down");
  //  else if (BIT_up) Serial.println("up");
  //  else if (BIT_right) Serial.println("right");
  //  else if (BIT_left) Serial.println("left");
  //  else if (BIT_R2) Serial.println("R2");
  //  else if (BIT_L2) Serial.println("L2");
  //  else if (BIT_R1) Serial.println("R1");
  //  else if (BIT_L1) Serial.println("L1");
  //  else if (BIT_batu) Serial.println("batu");
  //  else if (BIT_sankaku) Serial.println("sankaku");
  //  else if (BIT_sikaku) Serial.println("shikaku");
  //  else if (BIT_maru) Serial.println("maru");
  //  else Neutral_Pos();
  //  delay(10);
}
// void loop()
// {
//     degdeg();
//   // delay(20);
//   // serial monitor
//   // ReadVS2();
//   // if (BIT_select) Serial.println("select");
//   //  else if (BIT_start) Serial.println("start");
//   //  else if (BIT_down) Serial.println("down");
//   //  else if (BIT_up) Serial.println("up");
//   //  else if (BIT_right) Serial.println("right");
//   //  else if (BIT_left) Serial.println("left");
//   //  else if (BIT_R2) Serial.println("R2");
//   //  else if (BIT_L2) Serial.println("L2");
//   //  else if (BIT_R1) Serial.println("R1");
//   //  else if (BIT_L1) Serial.println("L1");
//   //  else if (BIT_batu) Serial.println("batu");
//   //  else if (BIT_sankaku) Serial.println("sankaku");
//   //  else if (BIT_sikaku) Serial.println("shikaku");
//   //  else if (BIT_maru) Serial.println("maru");
//   //  else Neutral_Pos();
//   //  delay(10);
// }