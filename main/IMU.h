#pragma once
#include <Arduino.h> 
#include <MPU6050.h> 

// REGISTER/I2C ADDRESSES -- based off the MPU6050 REGISTER MAP & DESCRIPTIONS DOC
#define GYRO_CONFIG_REG 
#define OUTPUT_MPU6050_READINGS             // comment this to disable Serial outputs  
#define ACCEL_FS_RANGE 2                    // +- 2g sensitivity 
#define GYRO_FS_RANGE 250                   // 250 degree/sec 
#define ACCEL_SCALE 16384               // in LSB/g, for accelerometer 2g FS # Section 4.17
#define GYRO_SCALE 131              // for 250 deg/s # Section 4.19 

struct IMUData{
    float yaw; 
    float pitch;
    float roll; 
};

MPU6050 mpu;
// tbh i forgot if this is correct declaration method 
extern float current_roll, current_pitch, current_yaw;






