#include "globals.h"

MotorWithEncoder HoodMotor(
    HOOD_MOTOR_DIR, // Motor Dir Pin 1
    HOOD_MOTOR_PWM, // Motor Enable Pin
    HOOD_MOTOR_ENCODER_A, // Encoder Pin A
    HOOD_MOTOR_ENCODER_B, // Encoder Pin B
    10, // Max Pwm Increment Per Acutation Period (units: duty cycle; range: 0~100)
    20, // Max Pwm Decrement Per Acutation Period (units: duty cycle; range: 0~100)
    1 , // Kp
    0 , // Ki
    0 , // Kd
    -PWM_MAX_BIT, // Output Min Value
    PWM_MAX_BIT // Output Max Value
);