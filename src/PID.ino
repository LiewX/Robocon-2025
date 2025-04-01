#include "PID.h"
#include <math.h>

// Constructor
PID_Controller::PID_Controller(double kp, double ki, double kd, double period, double outputMin, double outputMax)
    : kp(kp), ki(ki), kd(kd), deltaTime(period), outputMin(outputMin), outputMax(outputMax),
      previousError(0.0), integral(0.0), setpoint(0.0) {}

// Set the desired target value (setpoint)
inline void PID_Controller::setSetpoint(double target) {
    this->setpoint = target;
}

// Compute the PID output
double PID_Controller::compute(double feedforwardInput, double currentValue) {
    // Calculate error
    error = setpoint - currentValue;

    // Proportional term
    double proportional = kp * error;

    // Integral term
    double integralTerm = 0.0;
    if (ki != 0) {
        integral += error * deltaTime;
        integralTerm = ki * integral;
    }

    // Derivative term
    double derivative = (error - previousError) / deltaTime;
    double derivativeTerm = kd * derivative;

    // Calculate total output
    double output = proportional + integralTerm + derivativeTerm;

    // Save the current error for the next derivative calculation
    previousError = error;

    // Clamp output to min and max limits
    if (feedforwardInput + output > outputMax) return outputMax;
    if (feedforwardInput + output < outputMin) return outputMin;

    return feedforwardInput + output;
}

// Reset the PID controller
inline void PID_Controller::reset() {
    this->previousError = 0.0;
    this->integral = 0.0;
}

// Set PID coefficients
inline void PID_Controller::set_coefficients(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// Checks if the process variable error is within the acceptable threshold from the target
inline bool PID_Controller::is_within_tolerance(double tolerance) {
    if (abs(error) < tolerance)
        return true;
    else return false;
}