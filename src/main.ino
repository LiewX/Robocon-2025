#include <Arduino.h>
#include "PID.h"
#include "Encoder.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "Wire.h"
#include "Utils.h"
#include "motor.h"
#include "globals.h"
#include "Hood.h"

// To be implemented: task hood actuation
// To be implemented: pwm deadzone wrapper
// To be implemented: use internal pull up resistors for encoders
// Fixed flywheel function: changed to ledcWrite


#define PWM_PIN 27
#define FLYWHEEL_ENCODER_PIN 26
#define FLYWHEEL_MOTOR_ACTUATION_PERIOD 100
#define HOOD_MOTOR_ACTUATION_PERIOD 100
#define SEND_TO_I2C_PERIOD 150
#define BUFFER_SIZE 128
#define FLYWHEEL_PWM_RES 8
#define FLYWHEEL_PWM_FREQ 10000
#define FLYWHEEL_PWM_MAX_BIT ((1 << FLYWHEEL_PWM_RES) - 1)
#define POT_PIN 25
#define I2C_SLAVE_ADDR 0x08
#define RPI_SIGNAL_PIN 13  // Pin to signal RPi for angle calculation
#define SHOOTING_STATUS_PIN 12 // Pin to indicate shooting status (HIGH = shooting)

Encoder flywheelEncoder(FLYWHEEL_ENCODER_PIN, 6, 100, 2300UL, 7000UL); 
PID_Controller PID_stuffs(1, 0, 0, 100, 0, 3500);

// #define HALL_PIN 13
// #define ANTICLOCKWISE_PIN 12
// #define CLOCKWISE_PIN 25
// #define HOOD_LIM_SW 32
// Hood hoodstuffs(HALL_PIN,CLOCKWISE_PIN,ANTICLOCKWISE_PIN,HOOD_LIM_SW);

// Define a struct for the I2C data packet with const char* for data
struct I2cDataPacket {
    uint8_t slaveAddress;
    char message[BUFFER_SIZE];
    size_t length;
};

// Function prototypes for tasks
void task_actuate_flywheel_motor(void *pvParameters);
void task_actuate_hood_motor(void* pvParameters);

// Task Handles
TaskHandle_t xTask_ActuateFlywheelMotors;
TaskHandle_t xTask_ActuateHoodMotor;

// Queue Handles
QueueHandle_t xQueue_i2c;

uint8_t flywheelPwmChannel;

void setup() {
    Serial.begin(115200);
    
    // Flywheel motor pin setup
    flywheelPwmChannel = Motor::pwmChannelsUsed;
    ledcSetup(flywheelPwmChannel, FLYWHEEL_PWM_FREQ, FLYWHEEL_PWM_RES);

    // Start I2C bus as master
    Wire.begin();

    // Set up GPIO pins
    pinMode(PWM_PIN, OUTPUT);
    pinMode(RPI_SIGNAL_PIN, OUTPUT);
    pinMode(SHOOTING_STATUS_PIN, OUTPUT);
    pinMode(FLYWHEEL_ENCODER_PIN, INPUT_PULLUP); // Assuming encoder uses pullup
    digitalWrite(RPI_SIGNAL_PIN, LOW); // Ensure signal pin is initially low

    ledcAttachPin(PWM_PIN, flywheelPwmChannel);
    Motor::pwmChannelsUsed++;
    flywheelEncoder.begin();

    // Create queues
    // Note: These queues are declared in Globals.h so that they can be accessed in any file.
    xQueue_i2c = xQueueCreate(10, sizeof(uint8_t));  // Create a queue for I2C messages to be sent
    xQueue_angle = xQueueCreate(1, sizeof(float));  // Queue to receive angle from RPi

    bool creationStatus = 1; // Creation status flag for all FreeRTOS kernel objects

    // Check creation status for each queue
    check_queue_creation(creationStatus, xQueue_i2c, "Queue - Send to I2C");
    check_queue_creation(creationStatus, xQueue_angle, "Queue - Angle from RPi");
    
    // Create tasks
    // Arguments: Task function, Task name, Stack size (bytes), Parameters, Priority (higher numerical value means a more critical priority), Task handle
    BaseType_t taskCreation_ActuateFlywheelMotors = xTaskCreate(task_actuate_flywheel_motor, "Task - Actuate Flywheel Motors", 4096, NULL, 6, &xTask_ActuateFlywheelMotors);
    BaseType_t taskCreation_ActuateHoodMotor = xTaskCreate(task_actuate_flywheel_motor, "Task - Actuate Flywheel Motors", 4096, NULL, 6, &xTask_ActuateFlywheelMotors);
    BaseType_t taskCreation_I2CReceive = xTaskCreate(task_handle_i2c_receive, "Task - I2C Receive", 2048, NULL, 7, NULL);
    BaseType_t taskCreation_SendToI2C = xTaskCreate(task_send_to_i2c, "Task - Send to I2C", 2048, NULL, 5, NULL);
    
    // Check creation status for each task
    check_task_creation(creationStatus, taskCreation_ActuateFlywheelMotors, "Task - Actuate Flywheel Motors");
    check_task_creation(creationStatus, taskCreation_ActuateHoodMotor, "Task - Actuate Hood Motor");
    check_task_creation(creationStatus, taskCreation_I2CReceive, "Task - I2C Receive");
    check_task_creation(creationStatus, taskCreation_SendToI2C, "Task - Send to I2C");

    // If any of the queues/tasks failed to create, exit
    if (creationStatus == 0) {
        Serial.printf("Exiting program.\n");
        stop_program();
    }

    // Other initialization if needed, e.g., I2C setup
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(onReceiveI2C); // Register the callback

}
`
void loop() {
    vTaskDelay(10000);
}

void onReceiveI2C(int byteCount) {
  if (byteCount > 0) {
    String request = "";
    while (Wire.available()) {
      request += (char)Wire.read();
    }
    request.trim(); // remove whitespace

    if (request == "request angle") {
      Serial.println("Received angle request. Signaling RPi.");
      digitalWrite(RPI_SIGNAL_PIN, HIGH); // Signal RPi
      vTaskDelay(pdMS_TO_TICKS(10));  // Short delay for signal
      digitalWrite(RPI_SIGNAL_PIN, LOW);

      // Wait for angle from RPi (simulated here, replace with actual RPi communication)
      //  -->  Needs implementation for serial communication with RPi <--
      float receivedAngle; 
      if (xQueueReceive(xQueue_angle, &receivedAngle, pdMS_TO_TICKS(1000)) == pdPASS) { // Wait up to 1 second
        Serial.printf("Received angle from RPi: %.2f\n", receivedAngle);

        // Send angle to other ESP32 slave
        I2cDataPacket packet;
        packet.slaveAddress = I2C_SLAVE_ADDR; // Assuming same slave address, adjust if needed
        snprintf(packet.message, BUFFER_SIZE, "angle:%.2f", receivedAngle);
        packet.length = strnlen(packet.message, BUFFER_SIZE);
        if (xQueueSend(xQueue_i2c, &packet, portMAX_DELAY) != pdPASS) {
          Serial.println("Failed to send angle to slave.");
        }

        // Motor actions and shooting status handling  moved to task_actuate_flywheel_motor
        // ...  (lifting, ramping, shooting status)  ...

      } else {
        Serial.println("Timeout waiting for angle from RPi.");
      }
    } else if (request == "abort") {
      Serial.println("Received abort request from slave. Sending abort signal.");
      // Send abort signal back to slave
      I2cDataPacket packet;
      packet.slaveAddress = I2C_SLAVE_ADDR; // To the same slave that sent the abort request
      strncpy(packet.message, "abort", BUFFER_SIZE - 1); // -1 for null terminator
      packet.message[BUFFER_SIZE - 1] = '\0'; // Ensure null termination
      packet.length = strlen(packet.message);
      if (xQueueSend(xQueue_i2c, &packet, portMAX_DELAY) != pdPASS) {
        Serial.println("Failed to send abort signal to slave.");
      }
    }
  }
}

void task_handle_i2c_receive(void *pvParameters) {
  for (;;) {
    // The actual I2C receiving logic is now in the callback `onReceiveI2C`
    // This task can have a small delay, or be repurposed if needed for other checks.
    vTaskDelay(pdMS_TO_TICKS(50)); // Check for new messages regularly, but not constantly
  }
}

void simulate_rpi_angle(float angle) {
  // In a real scenario, this function would involve communication with the RPi
  // For this example, we simply queue a simulated angle after a delay
  Serial.printf("Simulating RPi angle calculation... %.2f\n", angle);
  vTaskDelay(pdMS_TO_TICKS(500)); // Simulate calculation time
  if (xQueueSend(xQueue_angle, &angle, 0) != pdPASS) {
    Serial.println("Failed to send simulated angle to queue.");
  }
}

void task_send_to_i2c(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_I2C_PERIOD);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    I2cDataPacket packet;

    for (;;) {
        if (xQueueReceive(xQueue_i2c, &packet, portMAX_DELAY) == pdPASS) {
            Wire.beginTransmission(packet.slaveAddress);
            Wire.write((uint8_t*)packet.message, packet.length);
            if (Wire.endTransmission() == 0) {
                Serial.printf("Sent: %s to %d\n", packet.message, packet.slaveAddress);
            } else {
                Serial.printf("Failed to send to %d\n", packet.slaveAddress);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);    }
}


void task_actuate_flywheel_motor(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(FLYWHEEL_MOTOR_ACTUATION_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time

    double cur_rpm=0;
    int pwm_set_val=0;
    double PID_out=0;
    double setpoint_val=3250;
    bool shooting = false; // Flag for shooting status

    for (;;) {
        // Check if angle has been received and motor actions should be performed
        if (ulTaskNotifyTake(pdTRUE, 0)) { // Check notification without blocking
            // Angle received, perform motor lift and flywheel ramp-up
            // (Replace placeholders with your actual motor control code)
            Serial.println("Lifting motor and ramping up flywheel...");
            // Example actions, replace with your control logic:
            // - Lift motor:  control Hood motor to target position for example 
            // - Ramp flywheel: increase setpoint gradually, for example
            setpoint_val = 2500; // Example: set a specific RPM for shooting
            vTaskDelay(pdMS_TO_TICKS(500)); // Simulate lift and ramp time
            shooting = true;
            digitalWrite(SHOOTING_STATUS_PIN, HIGH);
            Serial.println("Shooting status: ON");
        }

        // If shooting, allow for ramp down after a period
        if (shooting) {
            // If shooting for longer than 5 seconds (adjust as needed)
            if (xTaskGetTickCount() - xLastWakeTime > pdMS_TO_TICKS(5000)) {
                shooting = false;
                digitalWrite(SHOOTING_STATUS_PIN, LOW);
                Serial.println("Shooting status: OFF");
                setpoint_val = 0;
                Serial.println("Flywheel Ramping down...");

                // Notify other ESP32 slave about shooting status off
                I2cDataPacket packet;
                packet.slaveAddress = I2C_SLAVE_ADDR;  // Assuming same slave address
                strncpy(packet.message, "shooting:off", BUFFER_SIZE - 1);
                packet.message[BUFFER_SIZE - 1] = '\0';
                packet.length = strlen(packet.message);
                if (xQueueSend(xQueue_i2c, &packet, portMAX_DELAY) != pdPASS) {
                    Serial.println("Failed to send shooting status to slave.");
                }
            }
        }

        // PID Control
        PID_stuffs.setSetpoint(setpoint_val);
        cur_rpm=flywheelEncoder.getRPM();
        PID_out=PID_stuffs.compute(setpoint_val,cur_rpm);
        pwm_set_val=(PID_out+70.232)/13.041; // y=13.041*x-70.232 as the transfer function

        pwm_set_val = constrain(pwm_set_val, 0, FLYWHEEL_PWM_MAX_BIT);      // limits value between maximum and minimum
        ledcWrite(flywheelPwmChannel, abs(pwm_set_val));

        //Serial.printf("Setpoint: %.3f | Current RPM: %.3f | PID Output: %d\n", setpoint_val, cur_rpm, pwm_set_val);
        
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Position Control for hood
void task_actuate_hood_motor(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(HOOD_MOTOR_ACTUATION_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time

    for (;;) {
        double controlOutput = HoodMotor.PID.compute(0, HoodMotor.get_tick_position());
        HoodMotor.set_motor_PWM(controlOutput);
        if (HoodMotor.PID.is_within_tolerance(5)) {

        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/*

    // Check free stack of each tasks
    vTaskDelay(pdMS_TO_TICKS(4000)); // delay to let tasks run before printing free stack on each tasks
    print_free_stack(xTask_ActuateFlywheelMotors, "Task - Actuate Flywheel Motors");
    Serial.printf("Free heap size: %d bytes\n", esp_get_free_heap_size());  
    Serial.printf("Minimum free heap ever: %d bytes\n", esp_get_minimum_free_heap_size()); 
}
*/