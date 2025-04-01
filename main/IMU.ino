#include <Arduino.h>
#include "Globals.h"
#include "IMU.h"

// calling this gyro with a grain of salt. technically will do both gyro and accelerometers
/*
Accelerometer: gravitational acceleration
Gyroscope: earth's gravity to determine orientation
*/

// Variables for orientation tracking
float current_roll = 0.0, current_pitch = 0.0, current_yaw = 0.0;
unsigned long last_update_time = 0;
const float alpha = 0.98; // Complementary filter constant..? 

// SETUP FUNCTION FOR MPU
void mpu_setup() {

    Serial.println("Initializing MPU6050...");
    mpu.initialize(ACCEL_FS::A2G, GYRO_FS::G250DPS); 

    /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
    Serial.println("Updating internal sensor offsets...\n");
    mpu.setXAccelOffset(0); //Set your accelerometer offset for axis X
    mpu.setYAccelOffset(0); //Set your accelerometer offset for axis Y
    mpu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
    mpu.setXGyroOffset(0);  //Set your gyro offset for axis X
    mpu.setYGyroOffset(0);  //Set your gyro offset for axis Y
    mpu.setZGyroOffset(0);  //Set your gyro offset for axis Z
    /*Print the defined offsets*/
    Serial.print("\t");
    Serial.print(mpu.getXAccelOffset());
    Serial.print("\t");
    Serial.print(mpu.getYAccelOffset()); 
    Serial.print("\t");
    Serial.print(mpu.getZAccelOffset());
    Serial.print("\t");
    Serial.print(mpu.getXGyroOffset()); 
    Serial.print("\t");
    Serial.print(mpu.getYGyroOffset());
    Serial.print("\t");
    Serial.print(mpu.getZGyroOffset());
    Serial.print("\n");
    
    // will have to use the rtos version of this..
    last_update_time = millis(); 
}


// obtain raw readings 
int16_t ax, ay, az;     // raw Linear acceleration  
int16_t gx, gy, gz;     // raw Angular velocity 
float AccX, AccY, AccZ;     // normalised 
float GyroX, GyroY, GyroZ;  // normalised
float angle_roll, angle_pitch, angle_yaw; // roll - x, pitch - y, yaw - z  
float aErrorX, aErrorY, gErrorX, gErrorY, gErrorZ;
float acc_roll, acc_pitch; 


// read raw data 
void read_raw_gyro_data() {

    // Read raw accel/gyro data from the module. 
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       // 16bit signed! 
    // Scale the values by LSB Sensitivity (Datasheet) 
    AccX = (float) ax / ACCEL_SCALE; 
    AccY = (float) ay / ACCEL_SCALE; 
    AccZ = (float) az / ACCEL_SCALE; 
    GyroX = (float) gx / GYRO_SCALE;
    GyroY = (float) gy / GYRO_SCALE; 
    GyroZ = (float) gz / GYRO_SCALE;  

    #ifdef OUTPUT_MPU6050_READINGS
        Serial.print("a/g:\t");
        Serial.print(AccX); Serial.print("\t");
        Serial.print(AccY); Serial.print("\t");
        Serial.print(AccZ); Serial.print("\t");
        Serial.print(GyroX); Serial.print("\t");
        Serial.print(GyroY); Serial.print("\t");
        Serial.println(GyroZ);
    #endif
}

// calculate orientation corresponding to accelerometer readings
void calculate_roll_pitch(float acc_roll, float acc_pitch) {
    // Calculate roll & pitch ## formula src: https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
    acc_roll = (atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180 / PI);  // roll = atan2(ay, az) * 180.0 / PI; // i think this also works
    acc_pitch = (atan2(-1 * AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180 / PI);

    // Print values 
    #ifdef OUTPUT_MPU6050_READINGS
        Serial.print("Acc Pitch: "); Serial.print(acc_pitch); Serial.print("°  ");
        Serial.print("Acc Roll: "); Serial.print(acc_roll); Serial.println("°  ");
    #endif
}


// this is for the overall orientation, does fusion for accelerometer and gyroscope data with a 
// complementary filter -- integration over time 
void calculate_orientation() {
    // NOTE: use of millis here, adjust for RTOS dkfgjkfklgld
    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0; // Convert to seconds
    last_update_time = current_time;

    float gyro_yaw_rate = GyroZ; // Gyro Z-axis gives yaw rate in deg/sec
    current_yaw += gyro_yaw_rate * dt; // Integrate over time

    calculate_roll_pitch(&acc_roll, &acc_pitch);   

    current_roll = alpha * (current_roll + GyroX * dt) + (1 - alpha) * acc_roll;
    current_pitch = alpha * (current_pitch + GyroY * dt) + (1 - alpha) * acc_pitch;

    #ifdef OUTPUT_MPU6050_READINGS
        Serial.print("Yaw: "); Serial.print(current_yaw); Serial.print("°  ");
        Serial.print("Pitch: "); Serial.print(current_pitch); Serial.print("°  ");
        Serial.print("Roll: "); Serial.print(current_roll); Serial.println("°");
    #endif
}
