#pragma once
#include <Bluepad32.h>
#include <Arduino.h>
#include "Globals.h"


enum Ps4ButtonId : uint8_t {
    X, SQUARE, TRIANGLE, CIRCLE, L1, L2, R1, R2 // Warning! Pls update SLAVE_PS4_BUTTON_COUNTS after making changes
};

class Ps4ToI2cBridge {
    private:
        uint8_t slaveAddress;
        uint8_t previousState;
        uint8_t currentState;
    
    public:
        // Constructor
        Ps4ToI2cBridge(uint8_t address);
    
        // Update the state of a button (true = pressed, false = released)
        void update_button_state(uint8_t index, bool pressed);
    
        void send_to_i2c_transmission_queue();
        void clear_button_states();
    };

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void dumpGamepad(ControllerPtr ctl);
void processGamepad(ControllerPtr ctl);
void processStick(ControllerPtr ctl);
void processControllers();
void ps4_input_to_wheel_velocity ();
inline int check_deadzone(int value);