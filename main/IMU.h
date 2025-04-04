#pragma once
#include <Arduino.h> 
#include <MPU6050.h> 

// REGISTER/I2C ADDRESSES -- based off the MPU6050 REGISTER MAP & DESCRIPTIONS DOC
#define OUTPUT_MPU6050_READINGS             // comment this to disable Serial outputs  
#define ACCEL_FS_RANGE 2                    // +- 2g sensitivity 
#define GYRO_FS_RANGE 250                   // 250 degree/sec 
#define ACCEL_SCALE 16384               // in LSB/g, for accelerometer 2g FS # Section 4.17
#define GYRO_SCALE 131                  // for 250 deg/s # Section 4.19 
#define ACCEL_SCALE_RECIPROCAL  (1/ACCEL_SCALE)
#define GYRO_SCALE_RECIPROCAL   (1/GYRO_SCALE)

class IMU_Class{ 
    private: 
        MPU6050 mpu;            // the imu instance itself 
        int16_t ax, ay, az;     // raw Linear acceleration  
        int16_t gx, gy, gz;     // raw Angular velocity 
        double AccX, AccY, AccZ;     // normalised 
        double GyroX, GyroY, GyroZ;  // normalised
        double angle_roll, angle_pitch, angle_yaw; // roll - x, pitch - y, yaw - z  
        double aErrorX, aErrorY, gErrorX, gErrorY, gErrorZ;
        double acc_roll, acc_pitch; 
        double current_roll, current_pitch, current_yaw; 
        const double alpha = 0.98; // Complementary filter constant..? 

    public: 
        // constructor
        IMU_Class(); 
        void read_raw_gyro_data();      // reads raw data via I2C 
        void calculate_orientation();   // calculates current yaw angle using gyroscope only
        void calculate_roll_pitch();    // calculates roll and pitch based on accelerometer readings
        inline void reset_gyro();       // reset yaw angle to 0
        inline double get_gyro_yaw();   // returns current yaw angle of robot since last reset
}; 






