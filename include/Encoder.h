#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

class Encoder {
public:
    Encoder(uint8_t encoderPin, int slotsPerRevolution, int interval,
           unsigned long minDebounce = 8000UL, unsigned long maxDebounce = 20000UL);
    void begin();
    float getRPM();


private:
    static void IRAM_ATTR encoderISR();
    
    static volatile unsigned long count;
    static volatile unsigned long lastEdgeTime;
    static volatile unsigned long debounceTime;
    static portMUX_TYPE mux;
    static unsigned long MIN_DEBOUNCE;
    static unsigned long MAX_DEBOUNCE;
    int interval;
    uint8_t encoderPin;
    int slotsPerRev;
    unsigned long Lasttime;
    
};

#endif