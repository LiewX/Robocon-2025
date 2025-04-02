#pragma once
#include <Arduino.h> 
#include <MPU6050.h> 

// REGISTER/I2C ADDRESSES -- based off the MPU6050 REGISTER MAP & DESCRIPTIONS DOC
#define GYRO_CONFIG_REG 
#define OUTPUT_MPU6050_READINGS             // comment this to disable Serial outputs  
#define ACCEL_FS_RANGE 2                    // +- 2g sensitivity 
#define GYRO_FS_RANGE 250                   // 250 degree/sec 
#define ACCEL_SCALE 16384               // in LSB/g, for accelerometer 2g FS # Section 4.17
#define GYRO_SCALE 131                  // for 250 deg/s # Section 4.19 

// Struct to send IMU data to
struct IMUData{
    float yaw; 
    float pitch;
    float roll; 
};

class IMU{ 
    private: 
        MPU6050 mpu;            // the imu instance itself 
        int16_t ax, ay, az;     // raw Linear acceleration  
        int16_t gx, gy, gz;     // raw Angular velocity 
        float AccX, AccY, AccZ;     // normalised 
        float GyroX, GyroY, GyroZ;  // normalised
        float angle_roll, angle_pitch, angle_yaw; // roll - x, pitch - y, yaw - z  
        float aErrorX, aErrorY, gErrorX, gErrorY, gErrorZ;
        float acc_roll, acc_pitch; 
        float current_roll, current_pitch, current_yaw; 
        const float alpha = 0.98; // Complementary filter constant..? 

    public: 
        // constructor
        IMU(); 
        void read_raw_gyro_data();      // reads raw data via I2C 
        void calculate_orientation();   // calculates current orientation using fusion for accelerometer and gyroscope  
        void calculate_roll_pitch();    // calculates roll and pitch based on accelerometer readings 


    
}; 






