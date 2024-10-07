#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

#define DIR1 4
#define PWM1 2
#define DIR2 13
#define PWM2 12

const float WHEEL_RADIUS = 0.0345;
const float WHEEL_DISTANCE = 0.170;

MPU6050 mpu(Wire);

// State Variables
double theta = 0;           // Angle of tilt (degrees)
double omega = 0;           // Angular velocity (degrees/sec)
double pidOutput = 0;      // PID output

// Control Gain Constants (State Feedback Gains)
double Kp_theta = 20.0;     // Gain for tilt angle
double Kd_omega = 1.5;      // Gain for angular velocity

// PID Controller Parameters
double setpoint = 0;       // Desired angle (setpoint)
double Kp = 5.5, Ki = 4.3, Kd = 0.1;  // PID tuning values

unsigned long previousMillis = 0;
const long interval = 100;
unsigned long previousSerialMillis = 0;

double filterConstant = 0.1;  // Low-pass filter constant

// Create PID controller object
PID myPID(&theta, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);

// PID Velocity Control Parameters
double Kp_speed = 30.0, Ki_speed = 100.0, Kd_speed = 0.0;
double input_speed1 = 0, input_speed2 = 0;
double output_speed1 = 0, output_speed2 = 0;
double setpoint_speed1 = 0, setpoint_speed2 = 0;
PID pid_speed1(&input_speed1, &output_speed1, &setpoint_speed1, Kp_speed, Ki_speed, Kd_speed, DIRECT);
PID pid_speed2(&input_speed2, &output_speed2, &setpoint_speed2, Kp_speed, Ki_speed, Kd_speed, DIRECT);

void setMotorSpeeds(double speed1, double speed2) {
  // Clamp the motor speeds to a safe range
  speed1 = constrain(speed1, -255, 255);
  speed2 = constrain(speed2, -255, 255);

  Serial.print("Setting motor speed1: ");
  Serial.print(speed1);
  Serial.print(" | speed2: ");
  Serial.println(speed2);

  // Set direction and speed for Motor A (Adjusted direction)
  if (speed1 > 0) {
    digitalWrite(DIR1, HIGH);  // Adjusted direction
  } else {
    digitalWrite(DIR1, LOW); // Adjusted direction
  }
  ledcWrite(0, abs(speed1)); // PWM for Motor A

  // Set direction and speed for Motor B (Adjusted direction)
  if (speed2 > 0) {
    digitalWrite(DIR2, LOW); // Adjusted direction
  } else {
    digitalWrite(DIR2, HIGH);  // Adjusted direction
  }
  ledcWrite(1, abs(speed2)); // PWM for Motor B
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");
  Wire.begin(21, 22);
  
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  mpu.calcOffsets();

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);

  ledcAttachPin(PWM1, 0);
  ledcAttachPin(PWM2, 1);
  
  Serial.println("MPU6050 Initialized and Calibrated");

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Limit the output to motor PWM range
  pid_speed1.SetMode(AUTOMATIC);
  pid_speed1.SetOutputLimits(-255, 255);
  pid_speed2.SetMode(AUTOMATIC);
  pid_speed2.SetOutputLimits(-255, 255);
}

void newCalibration() {
  mpu.calcOffsets();
  theta = 0;  // Reset angle
  Serial.println("MPU6050 re-calibrated and angle reset");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Update sensor data
    Serial.println("Updating MPU data");
    mpu.update();
    double rawTheta = -mpu.getAngleX();    // Get tilt angle from MPU6050
    double rawOmega = mpu.getGyroX();      // Get angular velocity from MPU6050

    // Calculate time step
    double dt = (currentMillis - previousMillis) / 1000.0;

    // Apply low-pass filter to get filtered angle
    theta = (filterConstant * rawTheta) + ((1 - filterConstant) * theta);

    // Display variables for Serial Monitor with 1-second delay
    if (currentMillis - previousSerialMillis >= 1000) {
      previousSerialMillis = currentMillis;
      Serial.print("Filtered Angle: ");
      Serial.print(theta);
      Serial.print(" | Omega: ");
      Serial.print(rawOmega);
      Serial.print(" | Kp: ");
      Serial.print(Kp);
      Serial.print(" | Ki: ");
      Serial.print(Ki);
      Serial.print(" | Kd: ");
      Serial.print(Kd);
      Serial.print(" | Kp_theta: ");
      Serial.print(Kp_theta);
      Serial.print(" | Kd_omega: ");
      Serial.print(Kd_omega);
      Serial.println();
    }

    // Update velocity PID inputs
    input_speed1 = rawOmega;
    input_speed2 = rawOmega;

    pid_speed1.Compute();
    pid_speed2.Compute();

    // Check if angle is within deadband to stop motors
    if (abs(theta - setpoint) < 2.0) { // Increased Deadband to 2 degrees
      setMotorSpeeds(0, 0);
      Serial.println("Angle within deadband, stopping motors");
    } else {
      // State Feedback Control
      double stateControlSignal1 = -Kp_theta * theta - Kd_omega * omega;
      setMotorSpeeds(stateControlSignal1, stateControlSignal1);
    }
  }

  // Handle Serial commands to tune parameters
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("Kp_theta")) {
      Kp_theta = command.substring(9).toFloat();
      Serial.print("Kp_theta updated to: ");
      Serial.println(Kp_theta);
    } else if (command.startsWith("Kd_omega")) {
      Kd_omega = command.substring(9).toFloat();
      Serial.print("Kd_omega updated to: ");
      Serial.println(Kd_omega);
    } else if (command.startsWith("Kp")) {
      Kp = command.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kp updated to: ");
      Serial.println(Kp);
    } else if (command.startsWith("Ki")) {
      Ki = command.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Ki updated to: ");
      Serial.println(Ki);
    } else if (command.startsWith("Kd")) {
      Kd = command.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kd updated to: ");
      Serial.println(Kd);
    } else if (command.startsWith("FILTER")) {
      filterConstant = command.substring(7).toFloat();
      Serial.print("Filter Constant updated to: ");
      Serial.println(filterConstant);
    } else if (command == "NEW_CALIBRATE") {
      newCalibration();
    }
  }
}