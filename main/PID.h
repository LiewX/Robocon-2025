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

    // Debug flag to print PID variables
    bool debug = false;

public:
    // Constructor
    PID_Controller(double kp, double ki, double kd, double period, double outputMin = -1e6, double outputMax = 1e6);

    // Set target value (setpoint)
    inline void setSetpoint(double target);

    // Compute the PID output
    double compute(double currentValue);

    // Clamp value to outputMin and outputMax
    double clamp_output(double unclampedValue);

    // Reset the PID controller
    inline void reset();

    // Set PID coefficients
    inline void setCoefficients(double kp, double ki, double kd);

    // Checks if the process variable error is within the acceptable threshold from the target
    inline bool is_within_tolerance(double tolerance);

    // Debug mode setter
    inline void set_debug(bool enable);

    // Print PID variables for tuning purposes
    void print_debug_info(double currentValue, double proportional, double integralTerm, double derivativeTerm, double output);
};
