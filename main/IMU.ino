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
IMU_Class::IMU_Class() 
    : ax(0), ay(0), az(0),
      gx(0), gy(0), gz(0),
      AccX(0.0), AccY(0.0), AccZ(0.0),
      GyroX(0.0), GyroY(0.0), GyroZ(0.0),
      angle_roll(0.0), angle_pitch(0.0), angle_yaw(0.0),
      aErrorX(0.0), aErrorY(0.0),
      gErrorX(0.0), gErrorY(0.0), gErrorZ(0.0),
      acc_roll(0.0), acc_pitch(0.0),
      current_roll(0.0), current_pitch(0.0), current_yaw(0.0) {

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
    #if OUTPUT_MPU6050_READINGS
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
void IMU_Class::read_raw_gyro_data() {
    // Read raw accel/gyro data from the module.
    this->mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       // 16bit signed! 
    // Scale the values by LSB Sensitivity (Datasheet)
    this->AccX = (double) ax * ACCEL_SCALE_RECIPROCAL;
    this->AccY = (double) ay * ACCEL_SCALE_RECIPROCAL;
    this->AccZ = (double) az * ACCEL_SCALE_RECIPROCAL;
    this->GyroX = (double) gx * GYRO_SCALE_RECIPROCAL;
    this->GyroY = (double) gy * GYRO_SCALE_RECIPROCAL;
    this->GyroZ = (double) gz * GYRO_SCALE_RECIPROCAL;

    #if OUTPUT_MPU6050_READINGS
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
void IMU_Class::calculate_roll_pitch() {
    // Calculate roll & pitch ## formula src: https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
    this->acc_roll = (atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180 / PI);  // roll = atan2(ay, az) * 180.0 / PI; // i think this also works
    this->acc_pitch = (atan2(-1 * AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180 / PI);

    // Print values
    #if OUTPUT_MPU6050_READINGS
        Serial.printf("Acc Pitch: %.3f° Acc Roll: %.3f°\n", acc_pitch, acc_roll);
    #endif
}


// this is for the overall orientation, does fusion for accelerometer and gyroscope data with a 
// complementary filter -- integration over time 
// void IMU_Class::calculate_orientation() {
//     // NOTE: use of millis here, adjust for RTOS dkfgjkfklgld
//     unsigned long current_time = millis();
//     double dt = (current_time - last_update_time) / 1000.0; // Convert to seconds
//     last_update_time = current_time;

//     double gyro_yaw_rate = GyroZ; // Gyro Z-axis gives yaw rate in deg/sec
//     this->current_yaw += gyro_yaw_rate * dt; // Integrate over time

//     // update raw roll and pitch as read from accelerometer
//     calculate_roll_pitch();

//     this->current_roll = alpha * (current_roll + GyroX * dt) + (1 - alpha) * acc_roll;
//     this->current_pitch = alpha * (current_pitch + GyroY * dt) + (1 - alpha) * acc_pitch;

//     #if OUTPUT_MPU6050_READINGS
//         Serial.print("Yaw: "); Serial.print(current_yaw); Serial.print("°  ");
//         Serial.print("Pitch: "); Serial.print(current_pitch); Serial.print("°  ");
//         Serial.print("Roll: "); Serial.print(current_roll); Serial.println("°");
//     #endif
// }

void IMU_Class::calculate_orientation() {
    // NOTE: use of millis here, adjust for RTOS dkfgjkfklgld
    unsigned long current_time = millis();
    double dt = (current_time - last_update_time) * 0.001; // Convert to seconds
    last_update_time = current_time;

    double gyro_yaw_rate = GyroZ; // Gyro Z-axis gives yaw rate in deg/sec

    if (xSemaphoreTake(xMutex_imuYaw, portMAX_DELAY)) {
        this->current_yaw += gyro_yaw_rate * dt;    // Integrate over time
        xSemaphoreGive(xMutex_wheelMotorPwm);       // Release the mutex after modifying the variable
    }

    // // update raw roll and pitch as read from accelerometer
    // calculate_roll_pitch();

    // this->current_roll = alpha * (current_roll + GyroX * dt) + (1 - alpha) * acc_roll;
    // this->current_pitch = alpha * (current_pitch + GyroY * dt) + (1 - alpha) * acc_pitch;

    // #if OUTPUT_MPU6050_READINGS
    //     Serial.print("Yaw: "); Serial.print(current_yaw); Serial.print("°  ");
    //     Serial.print("Pitch: "); Serial.print(current_pitch); Serial.print("°  ");
    //     Serial.print("Roll: "); Serial.print(current_roll); Serial.println("°");
    // #endif
}

inline void IMU_Class::reset_gyro() {
    this->current_yaw = 0;
}

inline double IMU_Class::get_gyro_yaw() {
    double yaw;
    if (xSemaphoreTake(xMutex_imuYaw, portMAX_DELAY)) {
        yaw = this->current_yaw;
        xSemaphoreGive(xMutex_wheelMotorPwm);  // Release the mutex after modifying the variable
        return yaw;
    }
}