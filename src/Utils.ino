#include <Arduino.h>
#include "Utils.h"

void stop_program(){
    for(;;);
}

// Modifies creation status to 0 if the task has not been created successfully.
inline void check_task_creation(bool creationStatus, BaseType_t status, const char *taskName) {
    if (status == pdPASS) {
        Serial.printf("'%s' created successfully.\n", taskName);
    }
    else {
        Serial.printf("Failed to create task '%s'.\n", taskName);
        creationStatus = 0;
    }
}

// Modifies creation status to 0 if the semaphore has not been created successfully.
inline void check_sem_creation(bool creationStatus, SemaphoreHandle_t sem, const char *semName){
    if (sem != NULL) {
        Serial.printf("'%s' created successfully.\n", semName);
    }
    else {
        Serial.printf("Failed to create semaphore '%s'.\n", semName);
        creationStatus = 0;
    }
}

// Modifies creation status to 0 if the semaphore has not been created successfully.
inline void check_queue_creation(bool creationStatus, QueueHandle_t queue, const char *queueName){
    if (queue != NULL) {
        Serial.printf("'%s' created successfully.\n", queueName);
    }
    else {
        Serial.printf("Failed to create queue '%s'.\n", queueName);
        creationStatus = 0;
    }
}

// Checking and printing free stack (available from allocated amount) of each task
inline void print_free_stack(TaskHandle_t taskHandle, const char* taskName) {
    if (taskHandle != NULL) {
        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
        Serial.print("Stack High Watermark for ");
        Serial.print(taskName);
        Serial.print(": ");
        Serial.println(highWaterMark);
    }
}