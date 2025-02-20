#include <Bluepad32.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// I2C Configuration
#define I2C_SDA_PIN 21 // Replace with your SDA pin
#define I2C_SCL_PIN 22 // Replace with your SCL pin
#define I2C_SLAVE_ADDRESS 0x40 // Replace with your I2C slave address

/********** Pin Definitions **********/
// ----- Net Motors -----
// Motor 1
#define MOTOR1_DIR 32        // Direction pin for Motor 1
#define MOTOR1_PWM 33        // PWM pin for Motor 1

// Motor 2
#define MOTOR2_DIR 25        // Direction pin for Motor 2
#define MOTOR2_PWM 26        // PWM pin for Motor 2

// ----- Tilting Mechanism -----
#define DIR_PIN 27           // Direction pin for Motor 3 (MD10A)
#define PWM_PIN 14           // PWM pin for Motor 3

// ----- Limit Switches -----
#define LIMIT_SWITCH_1 19    // For Motor 1 stop functionality at bottom
#define LIMIT_SWITCH_2 18    // For Motor 2 stop functionality at bottom 
#define LIMIT_SWITCH_3 17    // For Motor 1 stop functionality on top 
#define LIMIT_SWITCH_4 16    // For Motor 2 stop functionality on top 

/********** PWM Configuration **********/
// For net motors (Motor 1 & Motor 2), using Code 2’s parameters:
#define NET_PWM_FREQ 1000    // 1 kHz PWM frequency
#define NET_PWM_RES  8       // 8-bit resolution (0-255)
#define PWM_MAX      255     // Maximum PWM value

// For tilting mechanism (Motor 3), keep original Code 1 PWM settings:
#define M3_PWM_FREQ 8000     // 8 kHz PWM frequency for Motor 3
#define M3_PWM_RES  8        // 8-bit resolution

// PWM Channels
#define PWM_CHANNEL_M1 0     // Motor 1 net motor
#define PWM_CHANNEL_M2 1     // Motor 2 net motor
#define PWM_CHANNEL_M3 5     // Motor 3 tilting mechanism

/********** PS4 CONTROLLER **********/
// Define button masks
#define R1_BUTTON 0x0020
#define L1_BUTTON 0x0010
#define R2_BUTTON 0x0080
#define L2_BUTTON 0x0040

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/********** Motor 3 (Tilting) Control Functions **********/
void controlTiltingMotors(bool forward, int speed) {
  speed = constrain(speed, 0, PWM_MAX); // Important: Keep the constrain!
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  ledcWrite(PWM_CHANNEL_M3, speed);
}

void stopTiltingMotor()
{
  ledcWrite(PWM_CHANNEL_M3, 0);
}

/********** Net Motors (Motor 1 & Motor 2) Control Functions **********/
void controlMotor1(bool forward, int speed) {
  // the ternary operator inside digitalRead()
  // if forward, check the status of LIMIT_SWITCH_3, as it will not go backwards
  if (digitalRead(forward ? LIMIT_SWITCH_3 : LIMIT_SWITCH_1) == LOW) {
    ledcWrite(PWM_CHANNEL_M1, 0);
  } else {
    digitalWrite(MOTOR1_DIR, forward ? HIGH : LOW);
    ledcWrite(PWM_CHANNEL_M1, speed);
  }
}

void controlMotor2(bool forward, int speed) {
  if (digitalRead(forward ? LIMIT_SWITCH_4 : LIMIT_SWITCH_2) == LOW) {
    ledcWrite(PWM_CHANNEL_M2, 0);
  } else {
    digitalWrite(MOTOR2_DIR, forward ? HIGH : LOW);
    ledcWrite(PWM_CHANNEL_M2, speed);
  }
}

void controlNetMotors(bool forward, int speed) {
  controlMotor1(forward, speed);
  controlMotor2(forward, speed);
}

void stopNetMotors() {
  ledcWrite(PWM_CHANNEL_M1, 0);
  ledcWrite(PWM_CHANNEL_M2, 0);
}

/********** Bluepad32 Callbacks **********/
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Controller %d connected\n", i);
      return;
    }
  }
  Serial.println("Controller connected but no empty slot");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Controller %d disconnected\n", i);
      return;
    }
  }
}

/********** Controller Processing **********/
void processGamepad(ControllerPtr ctl) {

  stopTiltingMotor();
  stopNetMotors();

  // Tilting Motors
  if (ctl->buttons() & R1_BUTTON) {
    controlTiltingMotors(false, PWM_MAX);
  }

  else if (ctl->buttons() & L1_BUTTON) {
    controlTiltingMotors(true, PWM_MAX);
  }

  // Net Motors
  if (ctl->buttons() & R2_BUTTON) {
    controlNetMotors(true, PWM_MAX);
  }
  
  else if (ctl->buttons() & L2_BUTTON) {
    controlNetMotors(false, PWM_MAX);
  }
}

void processControllers() {
  for (auto controller : myControllers) {
    if (controller && controller->isConnected()) {
      processGamepad(controller);
    }
  }
}

/********** Setup & Loop **********/
void setup() {
  Serial.begin(115200);
  
  // --- Configure Net Motors (Motor 1 & Motor 2)
  // Motor 1 configuration:
  pinMode(MOTOR1_DIR, OUTPUT);
  ledcSetup(PWM_CHANNEL_M1, NET_PWM_FREQ, NET_PWM_RES);
  ledcAttachPin(MOTOR1_PWM, PWM_CHANNEL_M1);
  
  // Motor 2 configuration:
  pinMode(MOTOR2_DIR, OUTPUT);
  ledcSetup(PWM_CHANNEL_M2, NET_PWM_FREQ, NET_PWM_RES);
  ledcAttachPin(MOTOR2_PWM, PWM_CHANNEL_M2);
  
  // --- Configure Motor 3 (Tilting mechanism)
  pinMode(DIR_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL_M3, M3_PWM_FREQ, M3_PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL_M3);

  // --- Configure Limit Switches
  pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_3, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_4, INPUT_PULLUP);

  // --- Initialize Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
  vTaskDelay(1);
}