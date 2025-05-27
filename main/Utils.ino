#include <Arduino.h>
#include "Utils.h"

void stop_program(){
    for(;;);
}

// Semaphore creation and checking function
bool create_and_check_sem(SemaphoreHandle_t &sem, const char* semName) {
    sem = xSemaphoreCreateMutex();
    if (sem == NULL) {
        Serial.printf("Failed to create semaphore '%s'.\n", semName);
        return false;
    }
    Serial.printf("'%s' created successfully.\n", semName);
    return true;
}

// Queue creation and checking function
bool create_and_check_queue(QueueHandle_t &queue, const char* queueName, size_t size, size_t itemSize) {
    queue = xQueueCreate(size, itemSize);
    if (queue == NULL) {
        Serial.printf("Failed to create queue '%s'.\n", queueName);
        return false;
    }
    Serial.printf("'%s' created successfully.\n", queueName);
    return true;
}

// Task creation and checking function
bool create_and_check_task(void (*taskFunc)(void*), const char* taskName, uint32_t stackSize, UBaseType_t priority, TaskHandle_t* taskHandle) {
    BaseType_t status = xTaskCreate(taskFunc, taskName, stackSize, NULL, priority, taskHandle);
    if (status != pdPASS) {
        Serial.printf("Failed to create task '%s'.\n", taskName);
        return false;
    }
    Serial.printf("'%s' created successfully.\n", taskName);
    return true;
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