#include <Wire.h>
#include <math.h>
#include "gyro.hpp"
#include "standard.hpp"
#include "control.hpp"
#include <climits>
#include <iostream>

#define RESTRICT_PITCH

short int AccX, AccY, AccZ;
short int Temp;
short int GyroX, GyroY, GyroZ;

float AccX_f = 0.0f, AccY_f = 0.0f, AccZ_f = 0.0f;
float GyroX_f = 0.0f, GyroY_f = 0.0f, GyroZ_f = 0.0f;
float ACC_X_OFS = -2340;
float ACC_Y_OFS = 601;
float ACC_Z_OFS = 2851;
float GYRO_X_OFS = 21;
float GYRO_Y_OFS = -35;
float GYRO_Z_OFS = -7;
float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
float theta;
float theta_deg;
float alpha;

int D = 2147483647;

bool flag1;

// neutral pos offset (rad)
float npo_theta = -0.4798;
void ReadGyro()
{
    //  send start address
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_AX);
    Wire.endTransmission();
    //  request 14bytes (int16 x 7)
    Wire.requestFrom(MPU6050_ADDR, 14);
    //  get 14bytes
    AccX = Wire.read() << 8 | Wire.read();
    AccY = Wire.read() << 8 | Wire.read();
    AccZ = Wire.read() << 8 | Wire.read();
    Temp = Wire.read() << 8 | Wire.read();
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // low pass fiter
    AccX_f = 0.9 * AccX_f + 0.1 * (AccX - ACC_X_OFS);
    AccY_f = 0.9 * AccY_f + 0.1 * (AccY - ACC_Y_OFS);
    AccZ_f = 0.9 * AccZ_f + 0.1 * (AccZ - ACC_Z_OFS);
    GyroX_f = 0.9 * GyroX_f + 0.1 * (GyroX - GYRO_X_OFS);
    GyroY_f = 0.9 * GyroY_f + 0.1 * (GyroY - GYRO_Y_OFS);
    GyroZ_f = 0.9 * GyroZ_f + 0.1 * (GyroZ - GYRO_Z_OFS);

    acc_x = AccX_f / 16384.0; // FS_SEL_0 16,384 LSB / g
    acc_y = AccY_f / 16384.0;
    acc_z = AccZ_f / 16384.0;

    gyro_x = GyroX_f / 131.0; // FS_SEL_0 131 LSB / (°/s)
    gyro_y = GyroY_f / 131.0;
    gyro_z = GyroZ_f / 131.0;

    // pitch inclination
    theta = atan2(acc_z, acc_x) - npo_theta;
    theta_deg = atan2(acc_z,acc_x) * 180/PI;
    // roll inclination
    // alpha = atan2(acc_y, acc_x);

    

    // z:roll axis direction
    // y:pitch axis direction
    // x:yaw axis direction

    //Serial.println(atan2(acc_z,acc_x),5);
    Serial.println(theta, 5);
    Serial.println(theta_deg,5);
    // Serial.println("");
    // Serial.print("  ");
    //Serial.println(gyro_x, 5);
    // Serial.println("");
    // Serial.print("  ");
    //Serial.println(gyro_y, 5);
    // Serial.println("");
    // Serial.print("  ");
    //Serial.print(gyro_z, 5);
    //Serial.println("");
    //Serial.print("  ");
    //Serial.println(acc_x, 3);
    // Serial.println("");
    // Serial.print("  ");
    //Serial.println(acc_z, 3);
    // Serial.println("");
    // Serial.print("  ");
    //Serial.println(acc_y, 5);
    Serial.println("");
    //Serial.print("  ");
}

double deg[2];

bool flg = false;

void degdeg()
{
    if (flg == false)
    {
        deg[0] = 0;
        flg = true;
    }
    else if (flag1 == false)
    {
        deg[1] = deg[0] + gyro_x * 0.001;
        flag1 = true;
        Serial.println(deg[1], 10);
    }
    else if (flag1 == true)
    {
        deg[0] = deg[1] + gyro_x * 0.001;
        flag1 = false;
        Serial.println(deg[0], 10);
    }
}