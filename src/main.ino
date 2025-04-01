#include <Arduino.h>
#include "PID.h"
#include "Encoder.h"
#include "math.h"
#include "Wire.h"
#include "Utils.h"
#include "motor.h"
#include "globals.h"

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

Encoder flywheelEncoder(FLYWHEEL_ENCODER_PIN, 6, 100, 2300UL, 7000UL); 
PID_Controller PID_stuffs(1, 0, 0, 100, 0, 3500);

// Define a struct for the I2C data packet with const char* for data
struct I2cDataPacket {
    uint8_t slaveAddress;
    char message[BUFFER_SIZE];
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
    ledcAttachPin(PWM_PIN, flywheelPwmChannel);
    Motor::pwmChannelsUsed++;
    flywheelEncoder.begin();

    // Create queues
    // Note: These queues are declared in Globals.h so that they can be accessed in any file.
    xQueue_i2c = xQueueCreate(10, sizeof(uint8_t));  // Create a queue for I2C messages to be sent

    bool creationStatus = 1; // Creation status flag for all FreeRTOS kernel objects

    // Check creation status for each queue
    check_queue_creation(creationStatus, xQueue_i2c, "Queue - Send to I2C");
    
    // Create tasks
    // Arguments: Task function, Task name, Stack size (bytes), Parameters, Priority (higher numerical value means a more critical priority), Task handle
    BaseType_t taskCreation_ActuateFlywheelMotors = xTaskCreate(task_actuate_flywheel_motor, "Task - Actuate Flywheel Motors", 4096, NULL, 6, &xTask_ActuateFlywheelMotors);
    BaseType_t taskCreation_ActuateHoodMotor = xTaskCreate(task_actuate_flywheel_motor, "Task - Actuate Flywheel Motors", 4096, NULL, 6, &xTask_ActuateFlywheelMotors);

    // Check creation status for each task
    check_task_creation(creationStatus, taskCreation_ActuateFlywheelMotors, "Task - Actuate Flywheel Motors");
    check_task_creation(creationStatus, taskCreation_ActuateHoodMotor, "Task - Actuate Hood Motor");

    // If any of the semaphore/mutex and queue has failed to create, exit
    if (creationStatus == 0) {
        Serial.printf("Exiting program.\n");
        stop_program();
    }

    // Check free stack of each tasks
    vTaskDelay(pdMS_TO_TICKS(4000)); // delay to let tasks run before printing free stack on each tasks
    print_free_stack(xTask_ActuateFlywheelMotors, "Task - Actuate Flywheel Motors");
    Serial.printf("Free heap size: %d bytes\n", esp_get_free_heap_size());  
    Serial.printf("Minimum free heap ever: %d bytes\n", esp_get_minimum_free_heap_size()); 
}


void loop() {
    vTaskDelay(10000);
}


void task_actuate_flywheel_motor(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(FLYWHEEL_MOTOR_ACTUATION_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time

    double cur_rpm=0;
    int pwm_set_val=0;
    double PID_out=0;
    double setpoint_val=3250;

    for (;;) {
        int rawValue = analogRead(POT_PIN);      // Read ADC value (0 - 4095)
        setpoint_val = map(rawValue, 0, 4095, 0, 3250);  // Map to 0 - 3250


        PID_stuffs.setSetpoint(setpoint_val);
        cur_rpm=flywheelEncoder.getRPM();
        PID_out=PID_stuffs.compute(setpoint_val,cur_rpm);
        pwm_set_val=(PID_out+70.232)/13.041;

        pwm_set_val = constrain(pwm_set_val, 0, FLYWHEEL_PWM_MAX_BIT);      // limits value between maximum and minimum
        ledcWrite(flywheelPwmChannel, abs(pwm_set_val));

        Serial.printf("Setpoint: %.3f | Current RPM: %.3f | PID Output: %d\n", setpoint_val, cur_rpm, pwm_set_val);
        
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

// Task to send data to other ESP32 through I2C
void task_send_to_i2c(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_I2C_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    I2cDataPacket packet;
    for (;;) {
        // Wait until there is data in the I2C queue
        if (xQueueReceive(xQueue_i2c, &packet, portMAX_DELAY) == pdPASS) {
            Wire.beginTransmission(packet.slaveAddress);    // Set to send to specified slave
            Wire.write( (uint8_t*) packet.message, strlen(packet.message) );    // Send data
            if (Wire.endTransmission() == 0) {
                Serial.printf("Data sent successfully to slave.\n");
            } 
            else {
                Serial.printf("Failed to send data.\n");
            }
        }

        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}