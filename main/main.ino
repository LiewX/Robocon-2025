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
//#include <WiFi.h>
#include <WebSocketsClient.h>

/* 
 * To be implemented:
 * - Collecting data and applying values to software compensation code for intertia imbalance of the wheels
 * - Split up PID code for better debugging. (include prints etc.)
 * 
 * Last changed:
 * - Implementation of software compensation code for intertia imbalance of the wheels (no callibration values added yet)
 * - Reverted current priority assignments for each tasks (I was under the wrong assumption that a lower numerical value means a more critical priority)
 * 
 * To be tested:
 * - Pin assignment and open loop wheel motion on robot
 * - Communication of PS4 button presses through I2C
 * 
*/

// Global tasks names
const char* task1Name = "Task - PS4 Sampling";      // PS4 Sampling
const char* task2Name = "Task - Update Encoders";   // Update Wheel Encoders
const char* task3Name = "Task - Actuate Motors";    // Actuate Wheel Motors
const char* task4Name = "Task - WebSocket Handler"; // WebSocket Handler
const char* task5Name = "Task - Send WiFi Data";    // Send Data to WiFi
const char* task6Name = "Task - Send I2C Data";     // Send Data to I2C

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
void task_ps4_sampling      (void *pvParameters);
void task_update_encoders   (void *pvParameters);
void task_actuate_motors    (void *pvParameters);
void task_websocket_handler (void *pvParameters);
void task_send_to_wifi      (void *pvParameters);
void task_send_to_i2c       (void *pvParameters);

// Task Handles
TaskHandle_t xTask_Ps4Sampling;
TaskHandle_t xTask_UpdateEncoders;
TaskHandle_t xTask_ActuateMotors;
TaskHandle_t xTask_WebsocketHandler;
TaskHandle_t xTask_SendToWiFi;
TaskHandle_t xTask_SendToI2C;

// Semaphore Handles
SemaphoreHandle_t xMutex_wheelMotorPs4Inputs;
SemaphoreHandle_t bsem_;

// Initilaise Queues
// QueueHandle_t xQueue_wifi = NULL;
// QueueHandle_t xQueue_i2c = NULL;

// WebSocket Server Setup
const char* serverAddress = "192.168.231.75"; // Replace with your server's IP
const uint16_t serverPort = 81; //  server's port

void websocket_setup() {


    WiFi.disconnect(true, true);  // Forget all saved WiFi networks
    delay(1000);

    WiFi.begin(ssid, password);
    // Wait for the ESP32 to connect to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        static int retryCount = 0;
        Serial.printf("Connecting to WiFi (%s) ...\n", ssid);
        Serial.printf("WiFi Status: %d\n", WiFi.status());

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

    // // Start the WebSocket server
    // server.listen(81); // Listen on port 81
    // Serial.println("WebSocket server started!");
    // Connect to server
    webSocket.begin(serverAddress, serverPort, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);  // Auto-reconnect every 5s
    
}

// PS4 Controller Connection Setup
void ps4_setup() {
    // Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    // const uint8_t* addr = BP32.localBdAddress();
    // Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Wait for ps4 Connection
   // bool dataUpdated = BP32.update();
    // while (!dataUpdated) {
    //     dataUpdated = BP32.update();
    //     delay(250);
    // }
}

// void setup(){
//     Serial.begin(115200);
//     Serial.printf("Initializing...\n");
    
//     // Encoder setup
// 	ESP32Encoder::useInternalWeakPullResistors = puType::up;    // Enable the weak pull up resistors

//     // Setup
//     websocket_setup();  // WebSocket Server Setup
//     ps4_setup();        // PS4 Controller Setup
//     Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C

//     // run forward_hard_coded_with_encoder once to collect data
//     forward_hard_coded_with_encoder(0, PWM_MAX_BIT, 15000, 5000, 15000, wheelMotors);
    
//     // Task creation result variables
//     BaseType_t taskCreation_ps4Sampling;
//     BaseType_t taskCreation_UpdateEncoders;
//     BaseType_t taskCreation_ActuateMotors;
//     BaseType_t taskCreation_WebsocketHandler;
//     BaseType_t taskCreation_SendToWiFi;
//     BaseType_t taskCreation_SendToI2C;

//     bool creationStatus = 1; // Creation status flag
//     // Create Mutex (Mutual Exclusion Semaphore) for global variables
//     // Note: These semaphores are declared in Globals.h so that they can be accessed in any file.
//     xMutex_wheelMotorPs4Inputs = xSemaphoreCreateMutex();        // Mutex for global var ps4StickOutputs
//     bsem_ = xSemaphoreCreateBinary(); // Binary semaphore to indicate that new data is acquired in 
//     // Check creation status for each semaphore/mutex
//     check_sem_creation(creationStatus, xMutex_wheelMotorPs4Inputs, "Mutex - PS4 Stick Outputs");
//     check_sem_creation(creationStatus, bsem_, "Binary Semaphore - Placeholder");

    // Create queues
    // Define the queue handles

    // Note: These queues are declared in Globals.h so that they can be accessed in any file.
    // xQueue_wifi = xQueueCreate(10, BUFFER_SIZE);  // Create a queue for WiFi messages to be sent
    // xQueue_i2c = xQueueCreate(10, sizeof(uint8_t));  // Create a queue for I2C messages to be sent
    // // Check creation status for each queue
    // check_queue_creation(creationStatus, xQueue_wifi, "Queue - Send to WiFi");
    // check_queue_creation(creationStatus, xQueue_i2c, "Queue - Send to I2C");

//     // Create tasks
//     // Arguments: Task function, Task name, Stack size (bytes), Parameters, Priority (higher numerical value means a more critical priority), Task handle
//     taskCreation_ps4Sampling        = xTaskCreate(task_ps4_sampling,        "Task - PS4 Sampling",      4096, NULL, 4, &xTask_Ps4Sampling);
//     taskCreation_UpdateEncoders     = xTaskCreate(task_update_encoders,     "Task - Update Encoders",   2048, NULL, 5, &xTask_UpdateEncoders);
//     taskCreation_ActuateMotors      = xTaskCreate(task_actuate_motors,      "Task - Actuate Motors",    4096, NULL, 6, &xTask_ActuateMotors);
//     taskCreation_WebsocketHandler   = xTaskCreate(task_websocket_handler,   "Task - WebSocket Handler", 3072, NULL, 2, &xTask_WebsocketHandler);
//     taskCreation_SendToWiFi         = xTaskCreate(task_send_to_wifi,        "Task - Send Data",         2048, NULL, 3, &xTask_SendToWiFi);
//     taskCreation_SendToI2C          = xTaskCreate(task_send_to_i2c,         "Task - Send I2C Data",     2048, NULL, 2, &xTask_SendToI2C);

//     // Check creation status for each task
//     check_task_creation(creationStatus, taskCreation_ps4Sampling,       task1Name);
//     check_task_creation(creationStatus, taskCreation_UpdateEncoders,    task2Name);
//     check_task_creation(creationStatus, taskCreation_ActuateMotors,     task3Name);
//     check_task_creation(creationStatus, taskCreation_WebsocketHandler,  task4Name);
//     check_task_creation(creationStatus, taskCreation_SendToWiFi,        task5Name);
//     check_task_creation(creationStatus, taskCreation_SendToI2C,         task6Name);

//     // If any of the semaphore/mutex and queue has failed to create, exit
//     if (creationStatus == 0) {
//         Serial.printf("Exiting program.\n");
//         stop_program();
//     }

//     // Display blinking LED to indicate start of program.
//     pinMode(LED_PIN, OUTPUT);
//     digitalWrite(LED_PIN, HIGH);
//     Serial.println("Starting program.");
//     vTaskDelay(pdMS_TO_TICKS(500));
//     digitalWrite(LED_PIN, LOW);

//     // Check free stack of each tasks
//     #if PRINT_FREE_STACK_ON_EACH_TASKS
//     vTaskDelay(pdMS_TO_TICKS(4000)); // delay to let tasks run before checking free stack on each tasks
//     print_free_stack(xTask_Ps4Sampling, task1Name);
//     print_free_stack(xTask_UpdateEncoders, task2Name);
//     print_free_stack(xTask_ActuateMotors, task3Name);
//     print_free_stack(xTask_WebsocketHandler, task4Name);
//     print_free_stack(xTask_SendToWiFi, task5Name);
//     print_free_stack(xTask_SendToI2C, task6Name);
//     Serial.printf("Free heap size: %d bytes\n", esp_get_free_heap_size());  
//     Serial.printf("Minimum free heap ever: %d bytes\n", esp_get_minimum_free_heap_size()); 
//     #endif
// }

// // Loop is also treated as a task. Use it to get CPU utilization.
// void loop() {
//     #if PRINT_CPU_UTILIZATION
//     UtilPs4Sampling.send_util_to_wifi();
//     UtilUpdateEncoders.send_util_to_wifi();
//     UtilActuateMotors.send_util_to_wifi();
//     UtilWebSocketHandler.send_util_to_wifi();
//     UtilSendToWifi.send_util_to_wifi();
//     UtilSendToI2c.send_util_to_wifi();
//     #endif
//     vTaskDelay(pdMS_TO_TICKS(CPU_UTIL_CALCULATION_PERIOD));
// }
//this is for software compensation data collection purposes




void setup(){
    Serial.begin(115200);
    Serial.printf("Initializing...\n");
    // Encoder setup
	ESP32Encoder::useInternalWeakPullResistors = puType::up;    // Enable the weak pull up resistors

    // Setup
    websocket_setup();  // WebSocket Server Setup
    // Display blinking LED to indicate start of program.
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Starting program.");
    digitalWrite(LED_PIN, LOW);

    // Create queues
    // xQueue_wifi = xQueueCreate(10, BUFFER_SIZE);  // Create a queue for WiFi messages to be sent
    // xQueue_i2c = xQueueCreate(10, sizeof(uint8_t));  // Create a queue for I2C messages to be sent

    // Check creation status for each queue
    // bool creationStatus = true;
    // check_queue_creation(creationStatus, xQueue_wifi, "Queue - Send to WiFi");
    // check_queue_creation(creationStatus, xQueue_i2c, "Queue - Send to I2C");

    // run forward_hard_coded_with_encoder once to collect data
    //forward_hard_coded_with_encoder(0, 100, 15000, 5000, 15000,wheelMotors);


    //currentIter = 0;
    //lastUpdateTime = millis();
    
    
}
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    Serial.printf("WebSocket Event Type: %d\n", type); // Debugging print
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket: Disconnected from server");
            webSocket.begin(serverAddress, serverPort, "/");  // Try reconnecting
            break;
        case WStype_CONNECTED:
            Serial.println("WebSocket: Connected to server");
            webSocket.sendTXT("Hello Server!");
            // Call your function here
            forward_hard_coded_with_encoder(0, 100, 15000, 5000, 15000, wheelMotors);
            break;
        case WStype_TEXT:
            Serial.printf("WebSocket: Received text: %s\n", payload);
            break;
        default:
            Serial.println("WebSocket: Unknown event");
            break;
    }
}


// bool forwardRun = false;
void loop() {
    webSocket.loop(); // Process WebSocket events
    delay(10);        // Optional: Small delay to avoid CPU overuse
}

// Task - Get input from PS4
void task_ps4_sampling(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(PS4_SAMPLING_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    bool dataUpdated;
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilPs4Sampling.set_start_time();

        // Get new PS4 data
        dataUpdated = BP32.update();
        if (dataUpdated) processControllers();
        // Calculate motor input based on ps4 analog stick and modifies wheelMotorps4Inputs. Does not include ramp function
        ps4_input_to_wheel_velocity();

        // Set task end time (to calculate for CPU Utilization)
        UtilPs4Sampling.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task - Get encoder count from all motors//
void task_update_encoders(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_WHEEL_ENCODER_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilUpdateEncoders.set_start_time();

        // Update tick velocity for each wheel motors
        UL_Motor.update_tick_velocity();
        UR_Motor.update_tick_velocity();
        BL_Motor.update_tick_velocity();
        BR_Motor.update_tick_velocity();

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

        // TODO: Need PWM to speed mapping
        // Calculate PD PWM output of each wheel's motor
        // wheelMotorInputs[0] = UL_Motor.PID.compute(wheelMotorInputs[0], UL_Motor.ticksPerSample);
        // wheelMotorInputs[1] = UR_Motor.PID.compute(wheelMotorInputs[1], UR_Motor.ticksPerSample);
        // wheelMotorInputs[2] = BL_Motor.PID.compute(wheelMotorInputs[2], BL_Motor.ticksPerSample);
        // wheelMotorInputs[3] = BR_Motor.PID.compute(wheelMotorInputs[3], BR_Motor.ticksPerSample);
        // Actuate Wheel Motor
        actuate_motor_wheels();

        // Set task end time (to calculate for CPU Utilization)
        UtilActuateMotors.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// WebSocket Handling //
// void task_websocket_handler(void *pvParameters) {
//     const TickType_t xFrequency = pdMS_TO_TICKS(WEBSOCKET_HANDLING_PERIOD); // Set task running frequency
//     TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
//     for (;;) {
//         // Set task start time (to calculate for CPU Utilization)
//         UtilWebSocketHandler.set_start_time();

//         // Accept new WebSocket client connections
//         if (!clientConnected) {
//             auto newClient = server.accept();
//             if (newClient.available()) {
//                 Serial.println("New WebSocket client connected!");
//                 client = newClient;
//                 clientConnected = true;
//             }
//         }
//         // Handle client disconnection
//         if (clientConnected && !client.available()) {
//             Serial.println("Client disconnected!");
//             client.close();
//             clientConnected = false;
//         }

//         // Set task end time (to calculate for CPU Utilization)
//         UtilWebSocketHandler.set_end_time();
//         // Delay until the next execution time
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// Task to print messages in serial monitor (to be changed to WiFi sending)
// void task_send_to_wifi(void *pvParameters) {
//     const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_WIFI_PERIOD); // Set task running frequency
//     TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
//     char message [128];
//     for (;;) {
//         // Set task start time (to calculate for CPU Utilization)
//         UtilSendToWifi.set_start_time();

//         // Wait until there is data in the WiFi queue
//         if (xQueueReceive(xQueue_wifi, &message, portMAX_DELAY)) {
//             // If client is connected to WebSocket server
//             if (clientConnected && client.available())
//                 client.send(message);  // Send the received message to WebSocket server (Important: make sure 'message' is null-terminated)
//         }

//         // Set task end time (to calculate for CPU Utilization)
//         UtilSendToWifi.set_end_time();
//         // Delay until the next execution time
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// Task to send data to other ESP32 through I2C
// void task_send_to_i2c(void *pvParameters) {
//     const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_I2C_PERIOD); // Set task running frequency
//     TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
//     I2cDataPacket packet;
//     for (;;) {
//         // Set task start time (to calculate for CPU Utilization)
//         UtilSendToI2c.set_start_time();

//         // Wait until there is data in the I2C queue
//         if (xQueueReceive(xQueue_i2c, &packet, portMAX_DELAY) == pdPASS) {
//             Wire.beginTransmission(packet.slaveAddress);    // Set to send to specified slave
//             Wire.write(packet.message);    // Send data
//             if (Wire.endTransmission() == 0) {
//                 Serial.printf("Data sent successfully to slave.\n");
//             } 
//             else {
//                 Serial.printf("Failed to send data.\n");
//             }
//         }

//         // Set task end time (to calculate for CPU Utilization)
//         UtilSendToI2c.set_end_time();
//         // Delay until the next execution time
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }