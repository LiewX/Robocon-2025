#include "PID.h"
#include "Globals.h"
#include <math.h>

// Constructor
PID_Controller::PID_Controller(double kp, double ki, double kd, double period, double outputMin, double outputMax)
    : kp(kp), ki(ki), kd(kd),
      error(0.0), previousError(0.0), integral(0.0), setpoint(0.0), deltaTime(period), 
      outputMin(outputMin), outputMax(outputMax),
      debug(false) {}

// Set the desired target value (setpoint)
inline void PID_Controller::setSetpoint(double target) {
    this->setpoint = target;
}

// Compute the PID output
double PID_Controller::compute(double currentValue) {
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

    // Print debug info
    print_debug_info(currentValue, proportional, integralTerm, derivativeTerm, output);

    return output;
}

// Clamp value to outputMin and outputMax
double PID_Controller::clamp_output(double unclampedValue) {
    if (unclampedValue > this->outputMax)
        return this->outputMax;
    else if (unclampedValue < this->outputMin)
        return this->outputMin;
    else
        return unclampedValue;
}


// Reset the PID controller
inline void PID_Controller::reset() {
    this->previousError = 0.0;
    this->integral = 0.0;
}

// Set PID coefficients
inline void PID_Controller::setCoefficients(double kp, double ki, double kd) {
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

// Toggle debug mode to print PID variables for tuning purposes
inline void PID_Controller::set_debug(bool enable) {
    this->debug = enable;
}

// Print PID variables for tuning purposes
void PID_Controller::print_debug_info(double currentValue, double proportional, double integralTerm, double derivativeTerm, double output) {
    if (!debug) return;
    char formattedMessage[BUFFER_SIZE];
    // Create formatted message
    snprintf(
        formattedMessage, 
        sizeof(formattedMessage), 
        "[PID] SP: %.2f | PV: %.2f | Err: %.2f | P: %.2f I: %.2f D: %.2f | OUT: %.2f",
        setpoint, currentValue, error,
        proportional, integralTerm, derivativeTerm,
        output
    );
    // Send the formatted message to the queue
    xQueueSend(xQueue_wifi, &formattedMessage, 0);
}