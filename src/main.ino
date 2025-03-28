#include <Arduino.h>
#include "PID.h"
#include "Encoder.h"
#include "math.h"
#include "Wire.h"
#include "Utils.h"
#include "Hood.h"

#define PWM_PIN 26
#define ENCODER_PIN 27
#define SETPOINT_TEST 4
#define HALL_PIN 13
#define ANTICLOCKWISE_PIN 12
#define CLOCKWISE_PIN 25
#define HOOD_LIM_SW 32
#define FLYWHEEL_MOTOR_ACTUATION_PERIOD 100
#define HOOD_ACTUATION_PERIOD 100

#define SEND_TO_I2C_PERIOD 150
#define BUFFER_SIZE 128

Encoder encoder(ENCODER_PIN,6,100,2300UL,7000UL); 
PID_Controller PID_stuffs(1,0,0, 100, 0,3500);
Hood hoodstuffs(HALL_PIN,CLOCKWISE_PIN,ANTICLOCKWISE_PIN,HOOD_LIM_SW);

// Define a struct for the I2C data packet with const char* for data
struct I2cDataPacket {
    uint8_t slaveAddress;
    char message[BUFFER_SIZE];
};

// Function prototypes for tasks
void task_actuate_flywheel_motor(void *pvParameters);
void task_actuate_Hood(void *pvParameters);
// Task Handles
TaskHandle_t xTask_ActuateFlywheelMotors;
TaskHandle_t xTask_ActuateHood;

// Queue Handles
QueueHandle_t xQueue_i2c;

void setup() {
    // Todo: configure pull up resistors if needed by encoders

    pinMode(PWM_PIN, OUTPUT);
    pinMode(SETPOINT_TEST, INPUT);
    Serial.begin(9600);
    encoder.begin();

    // Create queues
    // Note: These queues are declared in Globals.h so that they can be accessed in any file.
    xQueue_i2c = xQueueCreate(10, sizeof(uint8_t));  // Create a queue for I2C messages to be sent

    bool creationStatus = 1; // Creation status flag for all FreeRTOS kernel objects

    // Check creation status for each queue
    check_queue_creation(creationStatus, xQueue_i2c, "Queue - Send to I2C");
    
    // Create tasks
    // Arguments: Task function, Task name, Stack size (bytes), Parameters, Priority (higher numerical value means a more critical priority), Task handle
    BaseType_t taskCreation_ActuateFlywheelMotors = xTaskCreate(task_actuate_flywheel_motor, "Task - Actuate Flywheel Motors", 4096, NULL, 6, &xTask_ActuateFlywheelMotors);
    // Check creation status for each task
    check_task_creation(creationStatus, taskCreation_ActuateFlywheelMotors, "Task - Actuate Flywheel Motors");

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
    delay(100);
}


void task_actuate_flywheel_motor(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(FLYWHEEL_MOTOR_ACTUATION_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time

    float cur_rpm=0;
    int pwm_set_val=0;
    double PID_out=0;
    int setpoint_val=3250;

    for (;;) {

        PID_stuffs.setSetpoint(setpoint_val);
        cur_rpm=encoder.getRPM();
        PID_out=PID_stuffs.compute(setpoint_val,cur_rpm);
        // y=13.041*x-70.232 as the transfer function of 
        pwm_set_val=(PID_out+70.232)/13.041;
        analogWrite(PWM_PIN,floorf(pwm_set_val));

        Serial.print("Setpoint:");
        Serial.print(setpoint_val);
        Serial.print("|Current RPM:");
        Serial.print(cur_rpm);
        Serial.print("|PID RPM:");
        Serial.print(PID_out);
        Serial.print("|Current PWM:");
        Serial.println(pwm_set_val);

        // Delay until the next execution time
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