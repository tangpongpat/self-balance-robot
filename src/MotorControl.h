// #ifndef MOTORCONTROL_H
// #define MOTORCONTROL_H

// class MotorControl {
// public:
//     MotorControl(int dirPinA, int pwmPinA, int dirPinB, int pwmPinB);

//     void setSpeed(float speedA, float speedB);

// private:
//     int dirPinA, pwmPinA;
//     int dirPinB, pwmPinB;
// };

// #endif

// #ifndef MOTORCONTROL_H
// #define MOTORCONTROL_H

// class MotorControl {
// public:
//     MotorControl(int dirPinA, int pwmPinA, int dirPinB, int pwmPinB);
//     void setSpeed(float speedA, float speedB);  // Speed ranges from 0 to 255

// private:
//     int dirPinA, pwmPinA;
//     int dirPinB, pwmPinB;
// };

// #endif


// MotorControl.cpp
// MotorControl.h
// MotorControl.h

// MotorControl.h
// MotorControl.h
// MotorControl.h

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <MPU6050_light.h>
#include <Wire.h>

// Define motor control pins (change the pin numbers as per your setup)
#define DIR1 4    // Direction pin for Motor A
#define PWM1 2    // PWM pin for Motor A
#define DIR2 13   // Direction pin for Motor B
#define PWM2 12   // PWM pin for Motor B

// Declare the MPU6050 object as extern
extern MPU6050 mpu;

// Declare PID control variables as extern
extern double setpoint;
extern double input;
extern double output;
extern double Kp;
extern double Ki;
extern double Kd;

// Function to control motor speeds
void setMotorSpeeds(double speed);

#endif // MOTOR_CONTROL_H
