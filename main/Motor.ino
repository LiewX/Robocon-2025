#include <Arduino.h>
#include "PinAssignment.h"
#include "Motor.h"
#include "Utils.h"
#include "Timing.h"
#include "Globals.h"
#include "RuntimePrints.h"

// Constructor for class MotorControl
Motor::Motor(uint8_t pin1, uint8_t pwmPin, double maxPwmIncrement)
    : motorDirPin(pin1), motorPwmPin(pwmPin), maxPwmIncrement(maxPwmIncrement), previousDutyCycle(0.0) {
        // Pin Initialisation
        pinMode(pin1, OUTPUT);
        
        // Check for available PWM channels
        if (Motor::pwmChannelsUsed == 16) {
            Serial.printf("Max PWM channels limit reached. Motor class cannot be initialized.\nStopping program.\n");
            stop_program();
        }
        pwmChannel = Motor::pwmChannelsUsed;
        ledcSetup(pwmChannel, PWM_FREQ, PWM_RES);
        ledcAttachPin(pwmPin, pwmChannel);
        Motor::pwmChannelsUsed++;

        // TODO: Find max acceleration and then find the max the rate of change of PWM
        // Note: Requires PWM to speed mapping
    }

// Method to set motor speed and direction
void Motor::set_motor_PWM(double dutyCycle) {
    int pwmValue = (int) ((dutyCycle * PWM_MAX_BIT + 0.5) / 100);   // converts duty cycle to units of bits while rounds to closest integer
    pwmValue = constrain(pwmValue, -PWM_MAX_BIT, PWM_MAX_BIT);      // limits value between maximum and minimum
       
    if (pwmValue >= 0) {            // CW
        digitalWrite(motorDirPin, LOW);
        ledcWrite(pwmChannel, abs(pwmValue));
    } 
    else if (pwmValue < 0) {     // CCW
        digitalWrite(motorDirPin, HIGH);
        ledcWrite(pwmChannel, abs(pwmValue));
    }

    // Update previousDutyCycle variable for next cycle
    this->previousDutyCycle = dutyCycle;
}

void Motor::stop_motor() {
    this->set_motor_PWM(0); // Set motor PWM
    this->previousDutyCycle = 0;
}

// This function applies input shaping (ramping function) to the raw input of the motor and does not actuate motor. 
// Use set_motor_pwm() afterwards.
// Do not use function by itself. It should be called multiple times.
/* Example:
 * double targetPWM = 100;
 * for (;;) {
 *     double shapedInput = Motor.ramp_PWM(targetPWM);
 *     Motor.set_motor_pwm(shapedInput);
 *     delay(MOTOR_ACTUATION_PERIOD);
 * }
*/
double Motor::input_shape_ramp(double rawInput) {
    // Find current unclamped increment from controller output
    double unclampedIncrement = rawInput - this->previousDutyCycle;
    
    // Limit Increment and return accordingly
    if (unclampedIncrement > this->maxPwmIncrement) {
        return this->previousDutyCycle + maxPwmIncrement;
    }
    else if (unclampedIncrement < - this->maxPwmIncrement) {
        return this->previousDutyCycle - maxPwmIncrement;
    }
    else {
        return this->previousDutyCycle + unclampedIncrement;
    }
}

// Currently open loop
/**
 * Accepts raw PWM input from PS4 controller. Applies a ramping function and then actuate the wheel motors
 * @param wheelMotors An array of wheel motor classes passed by reference.
 * @param wheelMotorPs4Inputs Raw duty cycle motor inputs derived from PS4 inputs.
 * @return none
 * @warning Do not use this function for other motors other than wheel motors.
 * @note Example use case - ramp_wheel_PWM(wheelMotors, wheelMotorPWMs);
 */
void actuate_motor_wheels() {
    double shapedInputs[4] = {0, 0, 0, 0};

    // Wait for mutex before getting value from ps4StickInputs
    if (xSemaphoreTake(xMutex_wheelMotorPs4Inputs, portMAX_DELAY)) {
        // Apply input shaping (ramp function) to raw duty cycle inputs derived from PS4 inputs
        for (int i = 0; i < 4; ++i) {
            shapedInputs[i] = wheelMotors[i].input_shape_ramp(wheelMotorPs4Inputs[i]);
        }
        xSemaphoreGive(xMutex_wheelMotorPs4Inputs);  // Release the mutex after using the variable
    }
    
    // Apply PD to get closed loop input to motor
    double controlOutput[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        controlOutput[i] = shapedInputs[i] + wheelMotors[i].PID.compute(wheelMotors[i].measuredPwmSpeed);
    }

    // Actuate each motors using shaped feedforward inputs and PID output (summed)
    for (int i = 0; i < 4; ++i) {
        wheelMotors[i].set_motor_PWM(controlOutput[i]);
    }

    // Printing in WiFi WebSocket //
    #if (PRINT_WHEEL_INPUT_CLAMPED_VELOCITY || PRINT_PID_OUTPUT_PLUS_FEEDFORWARD)
        char formattedMessage[BUFFER_SIZE];  // Buffer to store the formatted message
    #endif
    // Print clamped wheel inputs (unit: duty cycle)
    #if PRINT_WHEEL_INPUT_CLAMPED_VELOCITY
        // Create formatted message
        snprintf(
            formattedMessage, 
            sizeof(formattedMessage), 
            "Wheels' Clamped Duty Input \t(1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f)", shapedInputs[0], shapedInputs[1], shapedInputs[2], shapedInputs[3]
        );
        // Send the formatted message to the queue
        xQueueSend(xQueue_wifi, &formattedMessage, 0);
    #endif

    // Print PID output and feedforward input (unit: duty cycle) 
    #if PRINT_PID_OUTPUT_PLUS_FEEDFORWARD
        // Create formatted message
        snprintf(
            formattedMessage,
            sizeof(formattedMessage),
            "Wheels' PID and FF Duty Sum\t(1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f)", controlOutput[0], controlOutput[1], controlOutput[2], controlOutput[3]
        );
        // Serial.print(formattedMessage2);
        // Send the formatted message to the queue
        xQueueSend(xQueue_wifi, &formattedMessage, 0);
    #endif
}