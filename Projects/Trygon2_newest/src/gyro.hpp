#ifndef _GYRO_HPP
#define _GYRO_HPP

#include <Wire.h>

// MPU-6050のアドレス、レジスタ設定値
// #define MPU6050_WHO_AM_I 0x75   // Read Only
// #define MPU6050_PWR_MGMT_1 0x6B // Read and Write
// #define MPU_ADDRESS 0x68
#define MPU6050_ADDR 0x68
#define MPU6050_AX 0x3B
#define MPU6050_AY 0x3D
#define MPU6050_AZ 0x3F
#define MPU6050_TP 0x41 //  data not used
#define MPU6050_GX 0x43
#define MPU6050_GY 0x45
#define MPU6050_GZ 0x47

extern float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, theta;

void ReadGyro();

void degdeg();

#endif