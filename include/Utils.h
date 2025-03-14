#pragma once
#include <Arduino.h>

void stop_program();

inline void check_task_creation(bool creationStatus, BaseType_t status, const char *taskName);
inline void check_sem_creation(bool creationStatus, SemaphoreHandle_t sem, const char *semName);
inline void check_queue_creation(bool creationStatus, QueueHandle_t queue, const char *queueName);
inline void print_free_stack(TaskHandle_t taskHandle, const char* taskName);
