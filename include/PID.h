#pragma once

class PID_Controller {
private:
    // PID coefficients
    double kp;
    double ki;
    double kd;

    // PID variables
    double error;
    double previousError;
    double integral;
    double setpoint;
    double deltaTime;

    // Output limits
    double outputMin;
    double outputMax;

public:
    // Constructor
    PID_Controller(double kp, double ki, double kd, double period, double outputMin = -1e6, double outputMax = 1e6);

    // Set target value (setpoint)
    inline void setSetpoint(double target);

    // Compute the PID output
    double compute(double feedforwardInput, double currentValue);

    // Reset the PID controller
    inline void reset();

    // Set PID coefficients
    inline void set_coefficients(double kp, double ki, double kd);

    // Check if process variable is close to target
    inline bool is_within_tolerance(double tolerance);
};
