#include "motor.h"
#include "Utils.h"

// Constructor for class MotorControl
Motor::Motor(uint8_t pin1, uint8_t pwmPin, double maxPwmIncrement, double maxPwmDecrement)
    : motorDirPin(pin1), motorPwmPin(pwmPin), maxPwmIncrement(maxPwmIncrement), maxPwmDecrement(maxPwmDecrement), previousDutyCycle(0.0) {
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

// Constructor for child class of MotorControl
MotorWithEncoder::MotorWithEncoder(uint8_t pin1, uint8_t pwmPin, uint8_t encoderA, uint8_t encoderB, double maxPwmIncrement, double maxPwmDecrement, double kp, double ki, double kd, double outputMin, double outputMax)
    : Motor(pin1, pwmPin, maxPwmIncrement, maxPwmDecrement),           // Motor setup (constructor from parent class)
    currentEncoderCount(0), previousEncoderCount(0), // Encoder variables initialization
    PID(kp, ki, kd, outputMin, outputMax)   // Create PID controller class
    {
    // Encoder setup
    Encoder.attachFullQuad(encoderA, encoderB);
}

inline int MotorWithEncoder::get_tick_position() {
    return (int32_t) Encoder.getCount();
}

inline void MotorWithEncoder::reset_encoder_tick() {
    Encoder.clearCount();
}