#include <Arduino.h>
#include "Globals.h"      // Initialize global variables (Note: Variables in here can be accessed anywhere in any file)
#include "RuntimePrints.h"
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include "PS4.h"
#include <ESP32Encoder.h> //https://github.com/madhephaestus/ESP32Encoder
#include "Wire.h"
#include "CpuUtilization.h"

/* 
 * To be implemented:
 * 
 * 
 * Last changed:
 * - In the input shaping function for the wheel motor actuation, used another constant to allow for a bigger decrease in input so that the robot to stop faster.
 * - If PS4 disconnects, deactivate motor wheels
 * 
 * To be tested:
 * - Communication of PS4 button presses through I2C
 * - Testing new task "Task - Calibrate Wheel Motors"
*/

// Global tasks names
const char* task1Name = "Task - PS4 Sampling";                // PS4 Sampling
const char* task2Name = "Task - Update Encoders";             // Update Wheel Encoders
const char* task3Name = "Task - Actuate Motors";              // Actuate Wheel Motors
const char* task4Name = "Task - WebSocket Handler";           // WebSocket Handler
const char* task5Name = "Task - Send WiFi Data";              // Send Data to WiFi
const char* task6Name = "Task - Send I2C Data";               // Send Data to I2C
const char* task7Name = "Task - Calibrate Wheel Motors";     // Send Data to I2C

// Global class variable for calculating CPU Utilization for each task
TaskCpuUtilization UtilPs4Sampling      (PS4_SAMPLING_PERIOD,           task1Name);
TaskCpuUtilization UtilUpdateEncoders   (MOTOR_WHEEL_ENCODER_PERIOD,    task2Name);
TaskCpuUtilization UtilActuateMotors    (MOTOR_WHEEL_ACTUATION_PERIOD,  task3Name);
TaskCpuUtilization UtilWebSocketHandler (WEBSOCKET_HANDLING_PERIOD,     task4Name);
TaskCpuUtilization UtilSendToWifi       (SEND_TO_WIFI_PERIOD,           task5Name);
TaskCpuUtilization UtilSendToI2c        (SEND_TO_I2C_PERIOD,            task6Name);

// Function prototypes for setup functions
void websocket_setup();
void ps4_setup();

// Function prototypes for tasks
void task_ps4_sampling          (void *pvParameters);
void task_update_encoders       (void *pvParameters);
void task_actuate_motors        (void *pvParameters);
void task_websocket_handler     (void *pvParameters);
void task_send_to_wifi          (void *pvParameters);
void task_send_to_i2c           (void *pvParameters);
void task_calibrate_wheel_motor (void *pvParameters);

// Task Handles
TaskHandle_t xTask_Ps4Sampling;
TaskHandle_t xTask_UpdateEncoders;
TaskHandle_t xTask_ActuateMotors;
TaskHandle_t xTask_WebsocketHandler;
TaskHandle_t xTask_SendToWiFi;
TaskHandle_t xTask_SendToI2C;
TaskHandle_t xTask_CalibrateWheelMotor;

// Semaphore Handles
SemaphoreHandle_t xMutex_wheelMotorPs4Inputs;
SemaphoreHandle_t xMutex_sendWheelEncoderToWifi;
SemaphoreHandle_t bsem_calibrateWheelMotor;

// Queue Handles
QueueHandle_t xQueue_wifi;
QueueHandle_t xQueue_i2c;

// WebSocket Server Setup
void websocket_setup() {
    WiFi.begin(ssid, password);
    // Wait for the ESP32 to connect to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        static int retryCount = 0;
        Serial.printf("Connecting to WiFi (%s) ...\n", ssid);
        retryCount++;
        if (retryCount >= 10) {
            Serial.println("Can't connect to WiFi. Restarting.");
            ESP.restart(); // Restart ESP32 if can't connect to WiFi after 10 tries
        }
        delay(500);
    }
    Serial.printf("Connected to WiFi!\nESP32 IP Address: ");
    Serial.print(WiFi.localIP());
    Serial.printf(":81\n");

    // Start the WebSocket server
    server.listen(81); // Listen on port 81
    Serial.println("WebSocket server started!");
}

// PS4 Controller Connection Setup
void ps4_setup() {
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Wait for ps4 Connection
    // bool dataUpdated = BP32.update();
    // while (!dataUpdated) {
    //     dataUpdated = BP32.update();
    //     delay(250);
    // }
}

void setup(){
    Serial.begin(115200);
    Serial.printf("Initializing...\n");
    // Encoder setup
	ESP32Encoder::useInternalWeakPullResistors = puType::up;    // Enable the weak pull up resistors

    // Setup
    websocket_setup();  // WebSocket Server Setup
    ps4_setup();        // PS4 Controller Setup
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C

    bool creationStatus = 1; // Creation status flag for all FreeRTOS kernel objects
    
    // Create Mutex (Mutual Exclusion Semaphore) for global variables and binary semaphores
    // Note: These semaphores are declared in Globals.h so that they can be accessed in any file.
    xMutex_wheelMotorPs4Inputs = xSemaphoreCreateMutex();       // Mutex for global var ps4StickOutputs
    xMutex_sendWheelEncoderToWifi = xSemaphoreCreateBinary();   // Mutex for global var sendWheelEncoderToWifi
    bsem_calibrateWheelMotor = xSemaphoreCreateBinary();       // Binary semaphore to indicate that wheel callibration needs to be commenced 
    // Check creation status for each semaphore/mutex
    check_sem_creation(creationStatus, xMutex_wheelMotorPs4Inputs, "Mutex - PS4 Stick Outputs");
    check_sem_creation(creationStatus, xMutex_sendWheelEncoderToWifi, "Mutex - Send Wheel Encoders' Values to WiFi");
    check_sem_creation(creationStatus, bsem_calibrateWheelMotor, "Binary Semaphore - Placeholder");

    // Create queues
    // Note: These queues are declared in Globals.h so that they can be accessed in any file.
    xQueue_wifi = xQueueCreate(10, BUFFER_SIZE);  // Create a queue for WiFi messages to be sent
    xQueue_i2c = xQueueCreate(10, sizeof(uint8_t));  // Create a queue for I2C messages to be sent
    // Check creation status for each queue
    check_queue_creation(creationStatus, xQueue_wifi, "Queue - Send to WiFi");
    check_queue_creation(creationStatus, xQueue_i2c, "Queue - Send to I2C");

    // Create tasks
    // Arguments: Task function, Task name, Stack size (bytes), Parameters, Priority (higher numerical value means a more critical priority), Task handle
    BaseType_t taskCreation_ps4Sampling             = xTaskCreate(task_ps4_sampling,            "Task - PS4 Sampling",              4096, NULL, 4, &xTask_Ps4Sampling);
    BaseType_t taskCreation_UpdateEncoders          = xTaskCreate(task_update_encoders,         "Task - Update Encoders",           2048, NULL, 5, &xTask_UpdateEncoders);
    BaseType_t taskCreation_ActuateMotors           = xTaskCreate(task_actuate_motors,          "Task - Actuate Motors",            4096, NULL, 6, &xTask_ActuateMotors);
    BaseType_t taskCreation_WebsocketHandler        = xTaskCreate(task_websocket_handler,       "Task - WebSocket Handler",         3072, NULL, 2, &xTask_WebsocketHandler);
    BaseType_t taskCreation_SendToWiFi              = xTaskCreate(task_send_to_wifi,            "Task - Send Data",                 2048, NULL, 3, &xTask_SendToWiFi);
    BaseType_t taskCreation_SendToI2C               = xTaskCreate(task_send_to_i2c,             "Task - Send I2C Data",             2048, NULL, 2, &xTask_SendToI2C);
    BaseType_t taskCreation_CalibrateWheelMotors    = xTaskCreate(task_calibrate_wheel_motor,   "Task - Calibrate Wheel Motors",    3072, NULL, 7, &xTask_CalibrateWheelMotor);
    // Check creation status for each task
    check_task_creation(creationStatus, taskCreation_ps4Sampling,           task1Name);
    check_task_creation(creationStatus, taskCreation_UpdateEncoders,        task2Name);
    check_task_creation(creationStatus, taskCreation_ActuateMotors,         task3Name);
    check_task_creation(creationStatus, taskCreation_WebsocketHandler,      task4Name);
    check_task_creation(creationStatus, taskCreation_SendToWiFi,            task5Name);
    check_task_creation(creationStatus, taskCreation_SendToI2C,             task6Name);
    check_task_creation(creationStatus, taskCreation_CalibrateWheelMotors,  task7Name);

    // If any of the semaphore/mutex and queue has failed to create, exit
    if (creationStatus == 0) {
        Serial.printf("Exiting program.\n");
        stop_program();
    }

    // Display blinking LED to indicate start of program.
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Starting program."); 
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);

    // Check free stack of each tasks
    #if PRINT_FREE_STACK_ON_EACH_TASKS
    vTaskDelay(pdMS_TO_TICKS(4000)); // delay to let tasks run before printing free stack on each tasks
    print_free_stack(xTask_Ps4Sampling, task1Name);
    print_free_stack(xTask_UpdateEncoders, task2Name);
    print_free_stack(xTask_ActuateMotors, task3Name);
    print_free_stack(xTask_WebsocketHandler, task4Name);
    print_free_stack(xTask_SendToWiFi, task5Name);
    print_free_stack(xTask_SendToI2C, task6Name);
    print_free_stack(xTask_CalibrateWheelMotor, task7Name);
    Serial.printf("Free heap size: %d bytes\n", esp_get_free_heap_size());  
    Serial.printf("Minimum free heap ever: %d bytes\n", esp_get_minimum_free_heap_size()); 
    #endif
}

// Loop is also treated as a task. Use it to get CPU utilization.
void loop() {
    #if PRINT_CPU_UTILIZATION
    UtilPs4Sampling.send_util_to_wifi();
    UtilUpdateEncoders.send_util_to_wifi();
    UtilActuateMotors.send_util_to_wifi();
    UtilWebSocketHandler.send_util_to_wifi();
    UtilSendToWifi.send_util_to_wifi();
    UtilSendToI2c.send_util_to_wifi();
    #endif
    vTaskDelay(pdMS_TO_TICKS(CPU_UTIL_CALCULATION_PERIOD));
}

// Task - Get input from PS4 //
void task_ps4_sampling(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(PS4_SAMPLING_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    bool dataUpdated;
    uint8_t noDataCount = 0;
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilPs4Sampling.set_start_time();

        // Get new PS4 data
        dataUpdated = BP32.update();
        if (dataUpdated) {
            processControllers();
            noDataCount = 0;
        }
        else { 
            noDataCount++;
            if (noDataCount == 5) {
                ps4StickOutputs[0] = 0;
                ps4StickOutputs[1] = 0;
                ps4StickOutputs[2] = 0;
                ps4StickOutputs[3] = 0;
            }
        }
        // Calculate motor input based on ps4 analog stick and modifies wheelMotorps4Inputs. Does not include ramp function
        ps4_input_to_wheel_velocity();

        // Set task end time (to calculate for CPU Utilization)
        UtilPs4Sampling.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task - Get encoder count from all motors and send through WiFi //
void task_update_encoders(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_WHEEL_ENCODER_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    char formattedMessage[BUFFER_SIZE];  // Buffer to store the formatted message
    int32_t encoderUL, encoderUR, encoderBL, encoderBR;

    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilUpdateEncoders.set_start_time();

        // Update tick velocity for each wheel motors
        encoderUL = UL_Motor.update_tick_velocity();
        encoderUR = UR_Motor.update_tick_velocity();
        encoderBL = BL_Motor.update_tick_velocity();
        encoderBR = BR_Motor.update_tick_velocity();

        // Create and send the message to the queue
        if (xSemaphoreTake(xMutex_sendWheelEncoderToWifi, 0)) {
            if (sendWheelEncoderToWifi) {
                xSemaphoreGive(xMutex_sendWheelEncoderToWifi);
                sprintf(formattedMessage, "1:%d,2:%d,3:%d,4:%d\n", encoderUL, encoderUR, encoderBL, encoderBR);
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            }
        }
        // Set task end time (to calculate for CPU Utilization)
        UtilUpdateEncoders.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task - Actuate all motors //
void task_actuate_motors(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_WHEEL_ACTUATION_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilActuateMotors.set_start_time();

        // Actuate Wheel Motor
        actuate_motor_wheels();

        // Set task end time (to calculate for CPU Utilization)
        UtilActuateMotors.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// WebSocket Handling //
void task_websocket_handler(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(WEBSOCKET_HANDLING_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilWebSocketHandler.set_start_time();

        // Accept new WebSocket client connections
        if (!clientConnected) {
            auto newClient = server.accept();
            if (newClient.available()) {
                Serial.println("New WebSocket client connected!");
                client = newClient;
                clientConnected = true;
            }
        }
        // Handle client disconnection
        if (clientConnected && !client.available()) {
            Serial.println("Client disconnected!");
            client.close();
            clientConnected = false;
        }

        // Set task end time (to calculate for CPU Utilization)
        UtilWebSocketHandler.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task to print messages in serial monitor (to be changed to WiFi sending)
void task_send_to_wifi(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_WIFI_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    char message [128];
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilSendToWifi.set_start_time();

        // Wait until there is data in the WiFi queue
        if (xQueueReceive(xQueue_wifi, &message, portMAX_DELAY)) {
            // If client is connected to WebSocket server
            if (clientConnected && client.available())
                client.send(message);  // Send the received message to WebSocket server (Important: make sure 'message' is null-terminated)
        }

        // Set task end time (to calculate for CPU Utilization)
        UtilSendToWifi.set_end_time();
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
        // Set task start time (to calculate for CPU Utilization)
        UtilSendToI2c.set_start_time();

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

        // Set task end time (to calculate for CPU Utilization)
        UtilSendToI2c.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task to calibrate motor wheels due to different inertia of the wheels
void task_calibrate_wheel_motor(void *pvParemeters) {
    // Callibration parameters
    double initialPwm = 0;
    double targetPwm = 60;
    double rampUpTimeMs = 9000;
    double maxSpeedTime = 5000;
    double rampDownTimeMs = 9000;

    int currentPwm = initialPwm;
    TickType_t speedUpDelayAmount = rampUpTimeMs / (targetPwm - currentPwm) + 0.5;
    TickType_t speedDownDelayAmount = rampUpTimeMs / (targetPwm - currentPwm) + 0.5;

    // Setting up alias
    MotorWithEncoder& UL_Motor = wheelMotors[0];
    MotorWithEncoder& UR_Motor = wheelMotors[1];
    MotorWithEncoder& BL_Motor = wheelMotors[2];
    MotorWithEncoder& BR_Motor = wheelMotors[3];

    for (;;) {
        // When task is first created or has finished 1 iteration, suspend itself
        vTaskSuspend(NULL);

        // Ramp up
        if (currentPwm < targetPwm) {
            currentPwm += 2.55;     // increment equals to 1 pwm bit
            UL_Motor.set_motor_PWM(currentPwm); // Top Right (UR)
            UR_Motor.set_motor_PWM(currentPwm); // Bottom Right (BR)
            BL_Motor.set_motor_PWM(currentPwm); // Bottom left (BL)
            BR_Motor.set_motor_PWM(currentPwm); // Top Left (UL)
            vTaskDelay(speedUpDelayAmount);
        }

        // Maintain max speed
        vTaskDelay(maxSpeedTime);

        // Ramp down
        if (currentPwm > targetPwm) {
            currentPwm -= 2.55;     // increment equals to 1 pwm bit
            UL_Motor.set_motor_PWM(currentPwm); // Top Right (UR)
            UR_Motor.set_motor_PWM(currentPwm); // Bottom Right (BR)
            BL_Motor.set_motor_PWM(currentPwm); // Bottom left (BL)
            BR_Motor.set_motor_PWM(currentPwm); // Top Left (UL)
            vTaskDelay(speedDownDelayAmount);
        }

        currentPwm = initialPwm;

        // Stop
        UL_Motor.stop_motor();
        UR_Motor.stop_motor();
        BL_Motor.stop_motor();
        BR_Motor.stop_motor();
    }
}