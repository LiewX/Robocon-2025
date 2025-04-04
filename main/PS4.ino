#include <Arduino.h>
#include <Bluepad32.h>
#include "PS4.h"
#include "math.h"
#include "Globals.h"
#include "RuntimePrints.h"
#include "Motor.h"

// Global variable array. For each index corresponding to each button press, it specifies which ESP32 the I2C message should be sent to
uint8_t I2cButtonSendingAddress [SLAVE_PS4_BUTTON_COUNTS] = 
{
    SLAVE_ADDR_ESP1,    // Button X
    SLAVE_ADDR_ESP2,    // Button Square
    SLAVE_ADDR_ESP1,    // Button Triangle
    SLAVE_ADDR_ESP2,    // Button Circle
    SLAVE_ADDR_ESP1,    // Button L1
    SLAVE_ADDR_ESP2,    // Button L2
    SLAVE_ADDR_ESP1,    // Button R1
    SLAVE_ADDR_ESP2,    // Button R2
};

// Constructor for class Ps4ToI2cBridge
Ps4ToI2cBridge::Ps4ToI2cBridge() {
    // Initialize arrays to 0
    memset(previousButtonStates, 0, sizeof(previousButtonStates));
    memset(currentButtonStates, 0, sizeof(currentButtonStates));
}

// Updates a specified PS4 button state
inline void Ps4ToI2cBridge::update_button_state(uint8_t &value, Ps4ButtonId buttonIndex) {
    previousButtonStates[buttonIndex] = currentButtonStates[buttonIndex];
    currentButtonStates[buttonIndex] = value;
}

// Notify any changed button states to specified ESP32 through I2C
inline void Ps4ToI2cBridge::send_to_i2c() {
    // Iterate through each button the arrays to check if current state differs from the previous state
    for (int i = 0; i < SLAVE_PS4_BUTTON_COUNTS; ++i) {
        if (currentButtonStates[i] != previousButtonStates[i]) {
            // I2C struct to send to RTOS queue
            I2cDataPacket packet;
            // Set ESP32 address of packet for the I2C message to send to
            packet.slaveAddress = I2cButtonSendingAddress[i];
            // Create formatted message
            snprintf(
                packet.message,
                BUFFER_SIZE,
                "%s:%d\n", buttonNames[i], currentButtonStates[i]
            );
            // Send the packet to the queue
            BaseType_t result = xQueueSend(xQueue_i2c, &packet, 0);
            
            // Check if the item was failed to be sent
            if (result != pdPASS) {
                // Create formatted error message
                snprintf(
                    packet.message,
                    BUFFER_SIZE,
                    "Fail to send data '%s:%d' to I2C queue", buttonNames[i], currentButtonStates[i]
                );
                // Send the error message to the WiFi queue
                xQueueSend(xQueue_wifi, &packet.message, 0);
            }
        }
    }
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

void dumpMouse(ControllerPtr ctl) {
    Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    static const char* key_names[] = {
        // clang-format off
        // To avoid having too much noise in this file, only a few keys are mapped to strings.
        // Starts with "A", which is offset 4.
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        // Special keys
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        // Function keys
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        // Cursors and others
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
        // clang-format on
    };
    static const char* modifier_names[] = {
        // clang-format off
        // From 0xe0 to 0xe7
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
        // clang-format on
    };
    Serial.printf("idx=%d, Pressed keys: ", ctl->index());
    for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = key_names[key-4];
            Serial.printf("%s,", keyName);
       }
    }
    for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = modifier_names[key-0xe0];
            Serial.printf("%s,", keyName);
        }
    }
    Console.printf("\n");
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
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

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    // dumpGamepad(ctl);
}

void processMouse(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }

    // See "dumpMouse" for possible things to query.
    // dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {
    if (!ctl->isAnyKeyPressed())
        return;

    // This is just an example.
    if (ctl->isKeyPressed(Keyboard_A)) {
        // Do Something
        Serial.println("Key 'A' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        // Do something else
        Serial.println("Key 'LEFT SHIFT' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        // Do something else
        Serial.println("Key 'Left Arrow' pressed");
    }

    // See "dumpKeyboard" for possible things to query.
    // dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    // dumpBalanceBoard(ctl);
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
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
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