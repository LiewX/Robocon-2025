#pragma once
#include "motor.h"

#define HOOD_MOTOR_DIR       0  // Motor Dir Pin 1
#define HOOD_MOTOR_PWM       0  // Motor Enable Pin
#define HOOD_MOTOR_ENCODER_A 0  // Encoder Pin A
#define HOOD_MOTOR_ENCODER_B 0  // Encoder Pin B

/*========================================================================================
=                             HOOD MOTOR GLOBAL VARIABLES                                =
========================================================================================*/
extern MotorWithEncoder HoodMotor;


/*========================================================================================
=                                         RTOS                                           =
========================================================================================*/