#pragma once
#include <Arduino.h>
#include "Globals.h"

class TaskCpuUtilization {
private:
    uint32_t startTime, endTime;    // Task start time, end time
    double taskDuration;           // Task duration (can sometimes be negative if too less delay between start and end)
    uint32_t taskExecutionCount;    // Amount of times task has run within CPU_UTIL_CALCULATION_PERIOD
    double taskUtilization;         // Percentage time spent in task during CPU_UTIL_CALCULATION_PERIOD
    const char* taskToMonitor;
    TaskHandle_t* pTaskHandle;
    char formattedMessage[BUFFER_SIZE];     // Buffer to store the message to send to WiFi
    SemaphoreHandle_t xSemaphore_ExecutedTask;

public:
    // Constructor
    TaskCpuUtilization();

    void init(uint32_t taskPeriod, const char* taskName, TaskHandle_t* pHandle);
    inline void set_start_time();
    inline void set_end_time();
    inline void send_util_to_wifi();
};