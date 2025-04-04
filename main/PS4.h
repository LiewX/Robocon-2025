#pragma once
#include <Bluepad32.h>
#include <Arduino.h>
#include "Globals.h"


#define SLAVE_PS4_BUTTON_COUNTS 8
enum Ps4ButtonId : uint8_t {
    X, SQUARE, TRIANGLE, CIRCLE, L1, L2, R1, R2 // Warning! Pls update SLAVE_PS4_BUTTON_COUNTS after making changes
};

class Ps4ToI2cBridge {
private:
    static const char* buttonNames [SLAVE_PS4_BUTTON_COUNTS];
    uint8_t previousButtonStates[SLAVE_PS4_BUTTON_COUNTS];
    uint8_t currentButtonStates[SLAVE_PS4_BUTTON_COUNTS];
public:
    // Constructor
    Ps4ToI2cBridge();
    // Update current state of button, checks last state of button
    inline void update_button_state(uint8_t &value, Ps4ButtonId button);
    // Notify other ESP32 if button state is changed
    inline void send_to_i2c();
};

const char* Ps4ToI2cBridge::buttonNames[SLAVE_PS4_BUTTON_COUNTS] = {
    "X",
    "Square",
    "Triangle",
    "Circle",
    "L1",
    "L2",
    "R1",
    "R2",
};

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void dumpGamepad(ControllerPtr ctl);
void dumpMouse(ControllerPtr ctl);
void dumpKeyboard(ControllerPtr ctl);
void dumpBalanceBoard(ControllerPtr ctl);
void processGamepad(ControllerPtr ctl);
void processMouse(ControllerPtr ctl);
void processKeyboard(ControllerPtr ctl);
void processBalanceBoard(ControllerPtr ctl);
void processStick(ControllerPtr ctl);
void processControllers();
void ps4_input_to_wheel_velocity ();
inline int check_deadzone(int value);