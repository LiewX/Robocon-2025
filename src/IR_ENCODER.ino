#include "IR_Encoder.h"

// Constructor
IREncoder::IREncoder(uint8_t pin, unsigned long calcInterval) {
    encoderPin = pin;
    interval = calcInterval;
    pulseCount = 0;
    lastTime = 0;
    rpm = 0;
    instanceIndex = (instances[0] == nullptr) ? 0 : 1;
    instances[instanceIndex] = this;
}

// Initialize the encoder
void IREncoder::begin() {
    pinMode(encoderPin, INPUT_PULLUP);
    if (instanceIndex == 0) {
        attachInterrupt(digitalPinToInterrupt(encoderPin), handleInterrupt0, RISING);
    } else if (instanceIndex == 1) {
        attachInterrupt(digitalPinToInterrupt(encoderPin), handleInterrupt1, RISING);
    }
    lastTime = millis();
}

// Get RPM
float IREncoder::getRPM() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
        noInterrupts();
        unsigned int count = pulseCount;
        pulseCount = 0;
        interrupts();

        rpm = (count * 60.0 / 5.60727) / (interval / 1000.0);
        lastTime = currentTime;
    }
    return rpm;
}

// Reset pulse count
void IREncoder::reset() {
    noInterrupts();
    pulseCount = 0;
    interrupts();
}
