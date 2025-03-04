#pragma once
#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include <Bluepad32.h>
//#include <ArduinoWebsockets.h>
//#include <WiFi.h>
#include <WebSocketsClient.h>

/*========================================================================================
=                            WHEEL MOTOR GLOBAL VARIABLES                                =
========================================================================================*/
// Upper Left Wheel Motor
MotorWithEncoder UL_Motor(
    MOTOR_UL_DIR_1,     // Motor Dir Pin 1
    MOTOR_UL_PWM,       // Motor Enable Pin
    MOTOR_UL_ENCODER_A, // Encoder Pin A
    MOTOR_UL_ENCODER_B, // Encoder Pin B
    2.5,                // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0,                  // Kd
    -PWM_MAX_BIT,       // Output Min Value
    PWM_MAX_BIT         // Output Max Value
);

// Upper Right Wheel Motor
MotorWithEncoder UR_Motor(
    MOTOR_UR_DIR_1,     // Motor Dir Pin 1
    MOTOR_UR_PWM,       // Motor Enable Pin
    MOTOR_UR_ENCODER_A, // Encoder Pin A
    MOTOR_UR_ENCODER_B, // Encoder Pin B
    2.5,                // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0,                  // Kd
    -PWM_MAX_BIT,       // Output Min Value
    PWM_MAX_BIT         // Output Max Value
);

// Bottom Left Wheel Motor
MotorWithEncoder BL_Motor(
    MOTOR_BL_DIR_1,     // Motor Dir Pin 1
    MOTOR_BL_PWM,       // Motor Enable Pin
    MOTOR_BL_ENCODER_A, // Encoder Pin A
    MOTOR_BL_ENCODER_B, // Encoder Pin B
    2.5,                // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0,                  // Kd
    -PWM_MAX_BIT,       // Output Min Value
    PWM_MAX_BIT         // Output Max Value
);

// Bottom Right Wheel Motor
MotorWithEncoder BR_Motor(
    MOTOR_BR_DIR_1,     // Motor Dir Pin 1
    MOTOR_BR_PWM,       // Motor Enable Pin
    MOTOR_BR_ENCODER_A, // Encoder Pin A
    MOTOR_BR_ENCODER_B, // Encoder Pin B
    2.5,                // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    10,                 // Kp
    0,                  // Ki
    0,                  // Kd
    -PWM_MAX_BIT,       // Output Min Value
    PWM_MAX_BIT         // Output Max Value
);

// An array of wheel motor of class MotorWithEncoder
MotorWithEncoder wheelMotors [4] = {UL_Motor, UR_Motor, BL_Motor, BR_Motor};
// Inputs for all 4 wheel motors computed from PD velocity controller.
double wheelMotorPs4Inputs [4] = {0, 0, 0, 0};          // Raw velocity calculated from PS4 analog stick

/*========================================================================================
=                                PS4 GLOBAL VARIABLES                                    =
========================================================================================*/
int ps4StickOutputs [4] = {0, 0, 0, 0};
ControllerPtr myControllers[BP32_MAX_GAMEPADS];



/*========================================================================================
=                      WiFi DATA TRANSMISSION GLOBAL VARIABLES                           =
========================================================================================*/
//using namespace websockets;

const char* ssid = "Ram";          // Replace with your Wi-Fi SSID
const char* password = "roey4363";  // Replace with your Wi-Fi password

// Define the WebSocket client
WebSocketsClient webSocket;

// const char* serverAddress = "192.168.231.75"; // Replace with your server's IP
// const uint16_t serverPort = 81; // Ensure this matches your server's port

//WebSocketsClient webSocket;

// WebsocketsServer server; // Create a WebSocket server
// WebsocketsClient client; // Store the connected client
// bool clientConnected = false;        // Track client connection status