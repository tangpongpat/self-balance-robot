// #include "MotorControl.h"
// #include <Arduino.h>

// MotorControl::MotorControl(int dirPinA, int pwmPinA, int dirPinB, int pwmPinB) {
//     this->dirPinA = dirPinA;
//     this->pwmPinA = pwmPinA;
//     this->dirPinB = dirPinB;
//     this->pwmPinB = pwmPinB;

//     pinMode(dirPinA, OUTPUT);
//     pinMode(pwmPinA, OUTPUT);
//     pinMode(dirPinB, OUTPUT);
//     pinMode(pwmPinB, OUTPUT);
// }

// void MotorControl::setSpeed(float speedA, float speedB) {
//     if (speedA > 0) {
//         digitalWrite(dirPinA, HIGH); // Forward direction
//     } else {
//         digitalWrite(dirPinA, LOW); // Reverse direction
//     }
//     analogWrite(pwmPinA, abs(speedA));

//     if (speedB > 0) {
//         digitalWrite(dirPinB, HIGH); // Forward direction
//     } else {
//         digitalWrite(dirPinB, LOW); // Reverse direction
//     }
//     analogWrite(pwmPinB, abs(speedB));
// }

// #include "MotorControl.h"
// #include <Arduino.h>

// MotorControl::MotorControl(int dirPinA, int pwmPinA, int dirPinB, int pwmPinB) {
//     this->dirPinA = dirPinA;
//     this->pwmPinA = pwmPinA;
//     this->dirPinB = dirPinB;
//     this->pwmPinB = pwmPinB;

//     // Set the direction and PWM pins as outputs
//     pinMode(dirPinA, OUTPUT);
//     pinMode(pwmPinA, OUTPUT);
//     pinMode(dirPinB, OUTPUT);
//     pinMode(pwmPinB, OUTPUT);
// }

// void MotorControl::setSpeed(float speedA, float speedB) {
//     // Ensure motors always move forward
//     digitalWrite(dirPinA, HIGH);  // Forward for Motor A
//     digitalWrite(dirPinB, HIGH);  // Forward for Motor B

//     // Constrain speed to be between 0 and 255
//     speedA = constrain(abs(speedA), 0, 255);
//     speedB = constrain(abs(speedB), 0, 255);

//     // Set PWM values for both motors
//     analogWrite(pwmPinA, speedA);
//     analogWrite(pwmPinB, speedB);
// }


// #include "MotorControl.h"
// #include <Arduino.h>

// MotorControl::MotorControl(int dirPinA, int pwmPinA, int dirPinB, int pwmPinB) {
//     this->dirPinA = dirPinA;
//     this->pwmPinA = pwmPinA;
//     this->dirPinB = dirPinB;
//     this->pwmPinB = pwmPinB;

//     // Set the direction and PWM pins as outputs
//     pinMode(dirPinA, OUTPUT);
//     pinMode(pwmPinA, OUTPUT);
//     pinMode(dirPinB, OUTPUT);
//     pinMode(pwmPinB, OUTPUT);
// }

// void MotorControl::setSpeed(float speedA, float speedB) {
//     // Ensure motors always move forward
//     digitalWrite(dirPinA, HIGH);  // Forward for Motor A
//     digitalWrite(dirPinB, HIGH);  // Forward for Motor B

//     // Constrain speed to be between 0 and 255
//     speedA = constrain(abs(speedA), 0, 255);
//     speedB = constrain(abs(speedB), 0, 255);

//     // Set PWM values for both motors
//     analogWrite(pwmPinA, speedA);
//     analogWrite(pwmPinB, speedB);
// }
// MotorControl.cpp

// MotorControl.cpp
// MotorControl.cpp

// MotorControl.cpp
// MotorControl.cpp

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
