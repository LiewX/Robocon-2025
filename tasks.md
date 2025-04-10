PID Closed loop rotation and IMU interfacing

IMU: 
- create task
- acquire data through i2c by sending 
- get gyroscope readings 

RECEIVE TASK FOR i2c
- receive data, calculate orientation
- take mutex, update orientation, give mutex

PID:
- send error through wifi 
- take mutex, take orientation variable, give mutex, update pid 
- actuate motors 

## Todo: 
1. Calibrate gyroscope of IMU to further prevent drift.
    - May need to use gyro offset for this.

## Changelog
(2/4/2025)
1. created task Gyro Sampling under main.ino
    - sampling period set to 100 
    - put at priority 4 (not sure where to put it at...)
    - requires library installation of MPU6050 by ElectronicCats

2. Called to mpu_setup in main.ino setup function 
    - initialises the mpu, sets accel and gyro offsets
    - all of these are printed out to serial monitor

3. made the imu into a class rather than a standalone import file, reducing globals

4. next patch! rtos integration and tidied up pid calculations
- also integration with motors and encoders. hopefully i dont crash the car. 

## Misc. Changelog
(5/4/2025)
1. In the input shaping function for the wheel motor actuation, used another constant to allow for a bigger decrease in input so that the robot to stop faster.

2. If PS4 disconnects, deactivate motor wheels

## Changelog
(10/4/2025)
1. Got rid of useless PS4 functions
2. Changed I2C sending protocol. It's now optimized to send only 1 byte.

# Todo
1. Create I2C message from button presses.
2. Create 1 task to transmit I2C message to ESP32 slaves at regular intervals.
3. Clear button state in I2C class if PS4 gets disconnected.

## To Test:
    (~)
    - Testing of new task "Task - Calibrate Wheel Motors"

    (5/4/2025)
    - Closed loop orientation control of robot using IMU

    (10/4/2025)
    - Button press. Should be notified through WiFi if PRINT_BUTTON_I2C_PRESS=1 in RuntimePrints.h

Note

// I2C struct to send to RTOS queue
I2cDataPacket packet;
// Set ESP32 address of packet for the I2C message to send to
packet.slaveAddress = I2cButtonSendingAddress[i];
// Create formatted message
packet.message = 0x00
// Send the packet to the queue
BaseType_t result = xQueueSend(xQueue_i2c, &packet, 0);