#pragma once
#include <Arduino.h>

enum Direction : unsigned char {
    FORWARD, BACKWARD, LEFT, RIGHT
};

void stop_program();

bool create_and_check_sem(SemaphoreHandle_t &sem, const char* semName);
bool create_and_check_queue(QueueHandle_t &queue, const char* queueName, size_t size, size_t itemSize);
bool create_and_check_task(void (*taskFunc)(void*), const char* taskName, uint32_t stackSize, UBaseType_t priority, TaskHandle_t* taskHandle);
inline void print_free_stack(TaskHandle_t taskHandle, const char* taskName);
