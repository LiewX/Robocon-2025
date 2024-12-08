#include <Arduino.h>
#include "Motor.h"
#include "PinAssignment.h"
#include "Utils.h"

#define MAX_ACCELERATION 1000 // mm/s^2
#define MAX_VELOCITY     1000 // mm/s
#define PWMResolutionMaxValue 255

// Constructor
MotorControl::MotorControl(byte pin1, byte pin2, byte pwmPin)
    : motorPin1(pin1), motorPin2(pin2), motorPWM(pwmPin){
        // Pin Initialisation
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        pinMode(pwmPin, OUTPUT);

        // TODO: Find max acceleration and then find the max the rate of change of PWM
        // Note: Requires PWM to speed mapping
    }

// Method to set motor speed and direction
void MotorControl::set_motor_PWM(double dutyCycle) {
    int pwmValue = constrain(dutyCycle * driveDir, -PWMResolutionMaxValue, PWMResolutionMaxValue);
    
    if (pwmValue > 0) {            // Clockwise
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(motorPWM, abs(pwmValue));
    } else if (pwmValue < 0) {     // CCW
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(motorPWM, abs(pwmValue));
    } else {
        stop_motor();
    }
}

// Method to stop the motor
void MotorControl::stop_motor() {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorPWM, 0);
}

// (Blocking) Method to test motor
void MotorControl::test_motor(int direction) {
    // Array of PWM values to test
    int pwmValues[] = {20, 40, 60, 80, 100};

    for (int i = 0; i < 5; i++) {
        this->set_motor_PWM(pwmValues[i]); // Set motor PWM
        Serial.print("Testing motor with PWM: ");
        Serial.println(pwmValues[i]);     // Print current PWM value to monitor
        delay(500);                       // Wait 500ms
    }

    // TODO: Implement max PWM rate of change (roc) to get smooth 0 to 100
}

// Method to find max acceleration and then find the max the rate of change of PWM
/*
void MotorControl::find_max_pwm_roc() {


}
*/

// Converts unit of speed from duty cycle to PWM value
inline int duty_cycle_to_PWM(double dutyCycle) {
    return (int) ((dutyCycle * PWMResolutionMaxValue / 100.0) + 0.5);
}

// Test sequentially of all wheel motors
void test_all_wheel_motors(MotorControl* UL_Motor, MotorControl* UR_Motor, MotorControl* BL_Motor, MotorControl* BR_Motor) {
    // Note: Each function is blocking and takes 2.5s
    UL_Motor->test_motor(FORWARD);
    UL_Motor->test_motor(BACKWARD);
    UR_Motor->test_motor(FORWARD);
    UR_Motor->test_motor(BACKWARD);
    BL_Motor->test_motor(FORWARD);
    BL_Motor->test_motor(BACKWARD);
    BR_Motor->test_motor(FORWARD);
    BR_Motor->test_motor(BACKWARD);
}

void forward_hard_coded(double maxPWM, double rampTime, double duration, MotorControl* UL_Motor, MotorControl* UR_Motor, MotorControl* BL_Motor, MotorControl* BR_Motor) {
    if (2*rampTime < duration) {
        double duration;
        double samplingPeriod;
        double rampTime;
        int maxIter = rampTime/samplingPeriod;
        double pwmIncrement = maxPWM / maxIter;
        double currentPWM = 0;
        // Increasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM += pwmIncrement;
            UL_Motor->set_motor_PWM(currentPWM);
            UR_Motor->set_motor_PWM(-currentPWM);
            BL_Motor->set_motor_PWM(-currentPWM);
            BR_Motor->set_motor_PWM(currentPWM);
            delay(samplingPeriod);
        }

        // Maintain Max Velocity
        delay(duration - 2*rampTime);

        // Decreasing Velocity
        for (int i = 0; i < maxIter; i++) {
            currentPWM -= pwmIncrement;
            UL_Motor->set_motor_PWM(currentPWM);
            UR_Motor->set_motor_PWM(-currentPWM);
            BL_Motor->set_motor_PWM(-currentPWM);
            BR_Motor->set_motor_PWM(currentPWM);
            delay(samplingPeriod);
        }
    }
    else {
        Serial.println("Ramp time should be 2x lesser than duration.");
    }
}