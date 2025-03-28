#ifndef Hood_h
#define Hood_h
#include "Arduino.h"
class Hood {
public:
    Hood(
        int hallPin,
        int clockwisePin,
        int counterClockwisePin,
        int limitSwitchPin
    );
    void begin();
    void Recalibrate();
    void turn_angle(float angle);





private:
    static void IRAM_ATTR HoodencoderISR();
    static volatile unsigned long Hoodcount;
    int hallPin;
    int clockwisePin;
    int counterClockwisePin;
    int limitSwitchPin; 
    const float degreesPerPulse = 0.0243425246;  
    float targetDegree = 0.0;  
    bool calibrated = false;


};

#endif
