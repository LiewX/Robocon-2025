#pragma once


class MotorControl {
private:
    // Motor pin variables
    byte motorDirPin;
    byte motorPwmPin;
    double speed;

public:
    // Constructor
    MotorControl(byte pin1, byte pwmPin);

    // Methods
    void set_motor_PWM(double dutyCycle);
    void stop_motor();
    void test_motor(double dutyCycle);
};



inline int duty_cycle_to_PWM(double dutyCycle);
void test_all_wheel_motors(MotorControl* UL_motor, MotorControl* UR_motor, MotorControl* BL_motor, MotorControl* BR_motor);
void forward_hard_coded(double initialPWM, double maxPWM, double rampTime, double duration, MotorControl* UL_Motor, MotorControl* UR_Motor, MotorControl* BL_Motor, MotorControl* BR_Motor);