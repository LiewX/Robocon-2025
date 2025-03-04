#pragma once
#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include <Bluepad32.h>
//#include <ArduinoWebsockets.h>
#include "RuntimePrints.h"
#include <WebSocketsClient.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/queue.h>

// // Queue Handles
// extern QueueHandle_t xQueue_wifi;
// extern QueueHandle_t xQueue_i2c;

#define SLAVE_ADDR_ESP1 0x10
#define SLAVE_ADDR_ESP2 0x20

/*========================================================================================
=                            WHEEL MOTOR GLOBAL VARIABLES                                =
========================================================================================*/

extern MotorWithEncoder UL_Motor; // Upper Left Wheel Motor
extern MotorWithEncoder UR_Motor; // Upper Right Wheel Motor
extern MotorWithEncoder BL_Motor; // Bottom Left Wheel Motor
extern MotorWithEncoder BR_Motor; // Bottom Right Wheel Motor

// An array of wheel motor of class MotorWithEncoder
extern MotorWithEncoder wheelMotors [4];
extern double wheelMotorPs4Inputs [4];          // Raw velocity calculated from PS4 analog stick



/*========================================================================================
=                                PS4 GLOBAL VARIABLES                                    =
========================================================================================*/
extern int ps4StickOutputs [4];
extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*========================================================================================
=                      WiFi DATA TRANSMISSION GLOBAL VARIABLES                           =
========================================================================================*/
// using namespace websockets;

extern const char* ssid;        // Replace with your Wi-Fi SSID
extern const char* password;    // Replace with your Wi-Fi password

extern WebSocketsClient webSocket;  // Declare it as an external variable

// extern WebsocketsServer server; // Create a WebSocket server
// extern WebsocketsClient client; // Store the connected client
// extern bool clientConnected;    // Track client connection status

/*========================================================================================
=                       I2C DATA TRANSMISSION GLOBAL VARIABLES                           =
========================================================================================*/
// Define a struct for the I2C data packet with const char* for data
struct I2cDataPacket {
    uint8_t slaveAddress;
    char message[BUFFER_SIZE];
};

/*========================================================================================
=                                         RTOS                                           =
========================================================================================*/
// Semaphores (Note: Initialize these semaphores in main.ino )
extern SemaphoreHandle_t xMutex_wheelMotorPs4Inputs;  // Mutex (Mutual Exclusion Semaphore) for ps4StickOutputs global var
extern SemaphoreHandle_t bsem_;  // Binary semaphore to indicate that new data is acquired in ps4StickOutputs global var

// Queue Handles
extern QueueHandle_t xQueue_wifi;
extern QueueHandle_t xQueue_i2c;