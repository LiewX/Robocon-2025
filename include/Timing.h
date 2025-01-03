#pragma once
#include "Utils.h"

#define MAIN_LOOP_PERIOD 10
#define MOTOR_WHEEL_ACTUATION_PERIOD 100
#define MOTOR_WHEEL_ENCODER_PERIOD 50

// Set encoder sampling to be 10ms earlier
#define MOTOR_WHEEL_ENCODER_TIME_OFFSET -10
#define MOTOR_WHEEL_ENCODER_LOOP_OFFSET (MOTOR_WHEEL_ENCODER_TIME_OFFSET / MAIN_LOOP_PERIOD)

// Define period (run once every x loopCount) for each tasks
#define MOTOR_WHEEL_ACTUATION_MOD (MOTOR_WHEEL_ACTUATION_PERIOD / MAIN_LOOP_PERIOD)
#define MOTOR_WHEEL_ENCODER_MOD (MOTOR_WHEEL_ENCODER_PERIOD / MAIN_LOOP_PERIOD)

int timing_setup_check();