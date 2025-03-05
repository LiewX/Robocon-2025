#include <Arduino.h>
#include "PID.h"
#include "Encoder.h"
#include "math.h"

#define PWM_PIN 26
#define ENCODER_PIN 27
#define SETPOINT_TEST 4 
Encoder encoder(ENCODER_PIN,6,100,2300UL,7000UL); 
PID_Controller PID_stuffs(1,0,0, 100, 0,3500);
float cur_rpm=0;
int pwm_set_val=0;
double PID_out=0;
int setpoint_val=3250;
void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_PIN, OUTPUT);
  pinMode(SETPOINT_TEST, INPUT);
  Serial.begin(9600);
  encoder.begin();
  
}


void loop() {
  // put your main code here, to run repeatedly:
  //setpoint_val=analogRead(SETPOINT_TEST)/2;
  PID_stuffs.setSetpoint(setpoint_val);
  cur_rpm=encoder.getRPM();
  PID_out=PID_stuffs.compute(setpoint_val,cur_rpm);
  pwm_set_val=(PID_out+70.232)/13.041;

  if (pwm_set_val>255)
  {
    pwm_set_val=255;
  }
  if (pwm_set_val<0)
  {
    pwm_set_val=0;
  }
  Serial.print("Setpoint:");
  Serial.print(setpoint_val);
  Serial.print("|Current RPM:");
  Serial.print(cur_rpm);
  Serial.print("|PID RPM:");
  Serial.print(PID_out);
  Serial.print("|Current PWM:");
  Serial.println(pwm_set_val);

  analogWrite(PWM_PIN,floorf(pwm_set_val));
  delay(100);



}
