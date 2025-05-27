#include <Arduino.h>
#include <Bluepad32.h>
#include "PS4.h"
#include "math.h"
#include "Globals.h"
#include "RuntimePrints.h"
#include "Motor.h"

Ps4ToI2cBridge::Ps4ToI2cBridge(uint8_t address)
: slaveAddress(address), previousState(0x00), currentState(0x00) {}

void Ps4ToI2cBridge::update_button_state(uint8_t index, bool pressed) {
    if (index > 7) return;
    if (pressed)
        currentState |= (1 << index);  // Set bit
    else
        currentState &= ~(1 << index); // Clear bit
}

void Ps4ToI2cBridge::clear_button_states() {
    currentState = 0;
}

uint8_t get_button_state(Ps4ToI2cBridge& esp, SemaphoreHandle_t xMutex_I2cButtonStates) {
    uint8_t message;
    if (xSemaphoreTake(xMutex_I2cButtonStates, 50)) {
        message = esp.currentState;
        return message;
    }
    xSemaphoreGive(xMutex_I2cButtonStates); // Release the mutex after accessing variable
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    uint16_t rawButtons = ctl->buttons();

    // Update I2C message that is to be transmitted to ESP2 (Shooting) with desired button states
    if (xSemaphoreTake(xMutex_I2C_ESP2, 0)) {
        I2C_ESP2.update_button_state(0, (rawButtons & 0x0008) != 0); // Triangle button (bit 0)
        I2C_ESP2.update_button_state(1, (rawButtons & 0x0010) != 0); // L1 button (bit 1)
        I2C_ESP2.update_button_state(2, (rawButtons & 0x0020) != 0); // R1 button (bit 2)
    }
    xSemaphoreGive(xMutex_I2C_ESP2);
    
    // Update I2C message that is to be transmitted to ESP3 (Catching) with desired button states
    if (xSemaphoreTake(xMutex_I2C_ESP3, 0)) {
        
    }
    xSemaphoreGive(xMutex_I2C_ESP3);

    // Update I2C message that is to be transmitted to ESP4 (Dribbling) with desired button states
    if (xSemaphoreTake(xMutex_I2C_ESP4, 0)) {
        
    }
    xSemaphoreGive(xMutex_I2C_ESP4);

    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // dumpGamepad(ctl);
}

// Function to get input to global variable
void processStick(ControllerPtr ctl) {
    ps4StickOutputs[0] = ctl->axisX();        // (-511 - 512) left X Axis
    ps4StickOutputs[1] = -1*(ctl->axisY());   // (-511 - 512) left Y axis
    ps4StickOutputs[2] = ctl->axisRX();       // (-511 - 512) right X axis
    ps4StickOutputs[3] = -1*(ctl->axisRY());  // (-511 - 512) right Y axis
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            processStick(myController);
            if (myController->isGamepad()) {
                processGamepad(myController);
            } 
            else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Function to convert left and right analog stick of ps4 to velocity for each wheel motors
// Implementation method is based on this website: https://seamonsters-2605.github.io/archive/mecanum/
void ps4_input_to_wheel_velocity () {
    // If value input is low and within deadzone, ignore it
    double stickLx = (double) check_deadzone(ps4StickOutputs[0]);
    double stickLy = (double) check_deadzone(ps4StickOutputs[1]);
    double stickRx = (double) check_deadzone(ps4StickOutputs[2]);
    double stickRy = (double) check_deadzone(ps4StickOutputs[3]);

    // Get actuation effort and angle from left stick input
    double leftStickActuation = std::sqrt(stickLx*stickLx + stickLy*stickLy);
    double leftStickAngle = atan2(stickLy, stickLx);
    // Get actuation effort from right stick input
    double rightStickActuation;
    if (stickRx > 0) {
        rightStickActuation = hypot(stickRx, stickRy);
    } else {
        rightStickActuation = -hypot(stickRx, stickRy);
    }

    // Calculate and update PWM needed for each wheels
    update_wheel_pwm(leftStickActuation, leftStickAngle, rightStickActuation);
}

inline int check_deadzone(int value) {
    if (value > PS4_DEADZONE) {
        return value;
    }
    else if (value < -PS4_DEADZONE){
        return value;
    }
    else return 0;
}