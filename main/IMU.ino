#include <Arduino.h>
#include "Globals.h"
#include "IMU.h"

/*
Accelerometer: gravitational acceleration
Gyroscope: earth's gravity to determine orientation
*/

// Variables for orientation tracking
unsigned long last_update_time = 0;

// Constructor to initialise the IMU 
IMU::IMU() {
    // initialise to 2G fullscale for accelerometer, 250dps for gyroscope
    this->mpu.initialize(ACCEL_FS::A2G, GYRO_FS::G250DPS); 

    /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
    // Serial.println("Updating internal sensor offsets...\n");
    this->mpu.setXAccelOffset(0); //Set your accelerometer offset for axis X
    this->mpu.setYAccelOffset(0); //Set your accelerometer offset for axis Y
    this->mpu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
    this->mpu.setXGyroOffset(0);  //Set your gyro offset for axis X
    this->mpu.setYGyroOffset(0);  //Set your gyro offset for axis Y
    this->mpu.setZGyroOffset(0);  //Set your gyro offset for axis Z

    /*Print the defined offsets*/
    #ifdef OUTPUT_MPU6050_READINGS
        Serial.print("\t");
        Serial.print(this->mpu.getXAccelOffset());
        Serial.print("\t");
        Serial.print(this->mpu.getYAccelOffset()); 
        Serial.print("\t");
        Serial.print(this->mpu.getZAccelOffset());
        Serial.print("\t");
        Serial.print(this->mpu.getXGyroOffset()); 
        Serial.print("\t");
        Serial.print(this->mpu.getYGyroOffset());
        Serial.print("\t");
        Serial.print(this->mpu.getZGyroOffset());
        Serial.print("\n");
    #endif 
    
    // will have to use the rtos version of this..
    last_update_time = millis(); 
}

// read raw data 
void IMU::read_raw_gyro_data() {

    // Read raw accel/gyro data from the module. 
    this->mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       // 16bit signed! 
    // Scale the values by LSB Sensitivity (Datasheet) 
    this->AccX = (float) ax / ACCEL_SCALE; 
    this->AccY = (float) ay / ACCEL_SCALE; 
    this->AccZ = (float) az / ACCEL_SCALE; 
    this->GyroX = (float) gx / GYRO_SCALE;
    this->GyroY = (float) gy / GYRO_SCALE; 
    this->GyroZ = (float) gz / GYRO_SCALE;  

    #ifdef OUTPUT_MPU6050_READINGS
        Serial.print("a/g:\t");
        Serial.print(this->AccX); Serial.print("\t");
        Serial.print(this->AccY); Serial.print("\t");
        Serial.print(this->AccZ); Serial.print("\t");
        Serial.print(this->GyroX); Serial.print("\t");
        Serial.print(this->GyroY); Serial.print("\t");
        Serial.println(this->GyroZ);
    #endif
}


// calculate orientation corresponding to accelerometer readings
void IMU::calculate_roll_pitch() {
    // Calculate roll & pitch ## formula src: https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
    this->acc_roll = (atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180 / PI);  // roll = atan2(ay, az) * 180.0 / PI; // i think this also works
    this->acc_pitch = (atan2(-1 * AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180 / PI);

    // Print values 
    #ifdef OUTPUT_MPU6050_READINGS
        Serial.print("Acc Pitch: "); Serial.print(acc_pitch); Serial.print("°  ");
        Serial.print("Acc Roll: "); Serial.print(acc_roll); Serial.println("°  ");
    #endif
}


// this is for the overall orientation, does fusion for accelerometer and gyroscope data with a 
// complementary filter -- integration over time 
void IMU::calculate_orientation() {
    // NOTE: use of millis here, adjust for RTOS dkfgjkfklgld
    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0; // Convert to seconds
    last_update_time = current_time;

    float gyro_yaw_rate = GyroZ; // Gyro Z-axis gives yaw rate in deg/sec
    this->current_yaw += gyro_yaw_rate * dt; // Integrate over time

    // update raw roll and pitch as read from accelerometer
    calculate_roll_pitch();   

    this->current_roll = alpha * (current_roll + GyroX * dt) + (1 - alpha) * acc_roll;
    this->current_pitch = alpha * (current_pitch + GyroY * dt) + (1 - alpha) * acc_pitch;

    #ifdef OUTPUT_MPU6050_READINGS
        Serial.print("Yaw: "); Serial.print(current_yaw); Serial.print("°  ");
        Serial.print("Pitch: "); Serial.print(current_pitch); Serial.print("°  ");
        Serial.print("Roll: "); Serial.print(current_roll); Serial.println("°");
    #endif
}
