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

#include "MotorControl.h"

// Define the MPU6050 object
MPU6050 mpu(Wire);

// Define the PID control variables
double setpoint = 0;
double input = 0;
double output = 0;
double Kp = 1.0, Ki = 0.5, Kd = 0.1;

// Define the motor control function
void setMotorSpeeds(double speed) {
  if (speed > 0) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
  } else {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
  }

  analogWrite(PWM1, abs(speed));  // Motor A
  analogWrite(PWM2, abs(speed));  // Motor B
}
