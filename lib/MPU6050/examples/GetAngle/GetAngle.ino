/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

// Pin definitions
#define DIR1 4    // Direction pin for Motor A
#define PWM1 2    // PWM pin for Motor A
#define DIR2 13   // Direction pin for Motor B
#define PWM2 12   // PWM pin for Motor B

// MPU6050 pins
#define SDA_PIN 21   // SDA pin for MPU6050
#define SCL_PIN 22   // SCL pin for MPU6050

MPU6050 mpu(Wire);

// PID control variables
double setpoint = 0;      // Desired angle (upright)
double input = 0;         // Current angle (from MPU6050)
double output = 0;        // PID output
double Kp = 1.0, Ki = 0.5, Kd = 0.1;  // PID tuning parameters

// PID object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Set motor pins as outputs
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Initialize I2C for MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.begin();
  
  // Calibrate MPU6050
  mpu.calcOffsets();  // This will take some time; hold the robot still
  
  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Set PID output limits to motor PWM range

  Serial.begin(115200);  // Start Serial for debugging
}

void loop() {
  // Update MPU6050 data
  mpu.update();

  // Get the tilt angle from the MPU6050
  input = mpu.getAngleX();  // Angle on the X-axis (pitch)

  // Compute PID output based on the tilt angle
  myPID.Compute();

  // Use the PID output to control the motors
  setMotorSpeeds(output);

  delay(10); // Adjust delay as needed
}

// Control the motor speeds based on PID output
void setMotorSpeeds(double speed) {
  if (speed > 0) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
  } else {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
  }

  analogWrite(PWM1, abs(speed));
  analogWrite(PWM2, abs(speed));
}

