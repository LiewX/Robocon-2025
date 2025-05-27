#pragma once
#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include <Bluepad32.h>
#include <ArduinoWebsockets.h>
#include "RuntimePrints.h"
#include "IMU.h"
#include "PS4.h"

// I2C Definitions
#define ESP2_I2C_SLAVE_ADDRESS 0x20
#define ESP3_I2C_SLAVE_ADDRESS 0x30
#define ESP4_I2C_SLAVE_ADDRESS 0x40

// PS4 Definitions
#define MAX_ANALOG_STICK_VALUE 512
#define PS4_DEADZONE 4

// Motor Definitions
#define PWM_RES 12
#define PWM_MAX_BIT ((1 << PWM_RES) - 1)    // equivalent to 2^PWM_RES - 1
#define PWM_FREQ 10000                      // test 1-20kHz range

// Software compensation for intertia imbalance on wheels
#define PWM_FACTOR_CORRECTION_UL 1.0
#define PWM_FACTOR_CORRECTION_UR 1.000459355
#define PWM_FACTOR_CORRECTION_BL 0.9406509621
#define PWM_FACTOR_CORRECTION_BR 1.020506243
#define PWM_OFFSET_UL  0.0
#define PWM_OFFSET_UR -0.07340768895
#define PWM_OFFSET_BL  1.82371705
#define PWM_OFFSET_BR -2.758928898


/*========================================================================================
=                            WHEEL MOTOR GLOBAL VARIABLES                                =
========================================================================================*/

extern MotorWithEncoder UL_Motor; // Upper Left Wheel Motor
extern MotorWithEncoder UR_Motor; // Upper Right Wheel Motor
extern MotorWithEncoder BL_Motor; // Bottom Left Wheel Motor
extern MotorWithEncoder BR_Motor; // Bottom Right Wheel Motor

// An array of wheel motor of class MotorWithEncoder
extern MotorWithEncoder wheelMotors [4];
extern double motorWheelsPwm [4];          // PWM input calculated to actuate motor

// Flag to send wheel encoder values to WiFi
extern bool sendWheelEncoderToWifi;

/*========================================================================================
=                                IMU GLOBAL VARIABLE                                     =
========================================================================================*/
extern IMU_Class IMU;

/*========================================================================================
=                                PS4 GLOBAL VARIABLES                                    =
========================================================================================*/
// I2C Class
extern Ps4ToI2cBridge I2C_ESP2;
extern Ps4ToI2cBridge I2C_ESP3;
extern Ps4ToI2cBridge I2C_ESP4;

extern int ps4StickOutputs [4];
extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*========================================================================================
=                      WiFi DATA TRANSMISSION GLOBAL VARIABLES                           =
========================================================================================*/
using namespace websockets;

extern const char* ssid;        // Replace with your Wi-Fi SSID
extern const char* password;    // Replace with your Wi-Fi password

extern WebsocketsServer server; // Create a WebSocket server
extern WebsocketsClient client; // Store the connected client
extern bool clientConnected;    // Track client connection status

/*========================================================================================
=                       I2C DATA TRANSMISSION GLOBAL VARIABLES                           =
========================================================================================*/
// Define a struct for the I2C data packet with const char* for data
struct I2cDataPacket {
    uint8_t slaveAddress;
    uint8_t message;
    uint8_t lastMessage;
};

/*========================================================================================
=                                         RTOS                                           =
========================================================================================*/
// Task Handles
extern TaskHandle_t xTask_Ps4Sampling;
extern TaskHandle_t xTask_UpdateEncoders;
extern TaskHandle_t xTask_ActuateMotors;
extern TaskHandle_t xTask_WebsocketHandler;
extern TaskHandle_t xTask_SendToWiFi;
extern TaskHandle_t xTask_SendToI2C;
extern TaskHandle_t xTask_CalibrateWheelMotor;
extern TaskHandle_t xTask_UpdateIMU;
extern TaskHandle_t xTask_OrientationControl;
extern TaskHandle_t xTask_SendButtonStatesThroughI2c;

// Semaphores (Note: Initialize these semaphores in main.ino )
extern SemaphoreHandle_t xMutex_motorWheelsPwm;              // Mutex (Mutual Exclusion Semaphore) for ps4StickOutputs global var
extern SemaphoreHandle_t xMutex_sendWheelEncoderToWifi; // Mutex for global var sendWheelEncoderToWifi
extern SemaphoreHandle_t xMutex_imuYaw;                 // Mutex for accessing yaw_angle in IMU instance
extern SemaphoreHandle_t xMutex_I2C_ESP2;               // Mutex for accessing I2C Class for ESP2
extern SemaphoreHandle_t xMutex_I2C_ESP3;               // Mutex for accessing I2C Class for ESP3
extern SemaphoreHandle_t xMutex_I2C_ESP4;               // Mutex for accessing I2C Class for ESP4
extern SemaphoreHandle_t bsem_callibrateWheelMotor;     // Binary semaphore to indicate that new data is acquired in ps4StickOutputs global var
// Queue Handles
extern QueueHandle_t xQueue_wifi;
extern QueueHandle_t xQueue_i2c;