#include "Hood.h"
// class creation 
Hood::Hood(
    int hallPin,int clockwisePin,int counterClockwisePin,int limitSwitchPin) 
    : hallPin(hallPin),clockwisePin(clockwisePin),counterClockwisePin(counterClockwisePin),limitSwitchPin(limitSwitchPin)
    {

    };

// initialises the hood class and preps to get counts from ISR
void Hood::begin(){
    pinMode(hallPin, INPUT_PULLUP);
    pinMode(clockwisePin, OUTPUT);
    pinMode(counterClockwisePin, OUTPUT);
    pinMode(limitSwitchPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(hallPin), HoodencoderISR, RISING);
}
// returns the hood position to 0 degrees when this is called (theoretically)
void Hood::Recalibrate()
{
    bool calibrated=false;
    Serial.println("Calibration mode: Searching for limit switch (0° position). ");
    while (!calibrated) {
        digitalWrite(clockwisePin, LOW);
        digitalWrite(counterClockwisePin, 255);

        if (digitalRead(limitSwitchPin) == LOW) {  // Stop when limit switch is triggered
            digitalWrite(counterClockwisePin, 0);
            noInterrupts();
            Hoodcount = 0;
            interrupts();
            calibrated = true;
            Serial.println("Calibration complete. 0° position established.");
            
        }
    }
}

// turn to a specific angle 
void Hood::turn_angle(float angle)
{
    // check hood count for where the angle is at rn
    if (Hoodcount*degreesPerPulse<targetDegree)
    {
        // set clockwise pin high (extending)
        digitalWrite(clockwisePin,HIGH);
        digitalWrite(counterClockwisePin,LOW);
    }
    else
    {
        // set all to stop when it reaches the desired angle
        digitalWrite(clockwisePin,LOW);
        digitalWrite(counterClockwisePin,LOW);
    }
}




// encoder count up
void IRAM_ATTR Hood::HoodencoderISR()
{
    Hoodcount++;
}