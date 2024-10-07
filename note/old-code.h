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

// Kalman Filter Variables
double kalmanAngle = 0;    // Filtered angle
double bias = 0;           // Bias estimation
double rate = 0;           // Rate estimation
double P[2][2] = {{1, 0}, {0, 1}};  // Error covariance matrix
double Q_angle = 0.001;    // Process noise variance for the angle
double Q_bias = 0.003;     // Process noise variance for the bias
double R_measure = 0.03;   // Measurement noise variance

// Control Gain Constants (State Feedback Gains)
double Kp_theta = 20.0;     // Gain for tilt angle
double Kd_omega = 1.5;      // Gain for angular velocity

// PID Controller Parameters
double setpoint = 0;       // Desired angle (setpoint)
double Kp = 5.5, Ki = 4.3, Kd = 0.1;  // PID tuning values

unsigned long previousMillis = 0;
const long interval = 100;

double filterConstant = 0.1;  // Low-pass filter constant

// Create PID controller object
PID myPID(&kalmanAngle, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);

void setMotorSpeeds(double speed) {
  // Set direction and speed for Motor A (Adjusted direction)
  if (speed > 0) {
    digitalWrite(DIR1, LOW);  // Adjusted direction
  } else {
    digitalWrite(DIR1, HIGH); // Adjusted direction
  }
  ledcWrite(0, abs(speed)); // PWM for Motor A

  // Add a small delay to ensure both motors are synchronized (optional)
  delay(1); // Delay 1 ms, you can adjust this as needed

  // Set direction and speed for Motor B (Adjusted direction)
  if (speed > 0) {
    digitalWrite(DIR2, HIGH); // Adjusted direction
  } else {
    digitalWrite(DIR2, LOW);  // Adjusted direction
  }
  ledcWrite(1, abs(speed)); // PWM for Motor B
}

void setup() {
  Serial.begin(115200);
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
}

double kalmanFilter(double newAngle, double newRate, double dt) {
  // Predict
  rate = newRate - bias;
  kalmanAngle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update
  double y = newAngle - kalmanAngle;
  double S = P[0][0] + R_measure;
  double K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  kalmanAngle += K[0] * y;
  bias += K[1] * y;

  double P00_temp = P[0][0];
  double P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return kalmanAngle;
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Update sensor data
    mpu.update();
    double rawTheta = -mpu.getAngleX();    // Get tilt angle from MPU6050
    double rawOmega = mpu.getGyroX();      // Get angular velocity from MPU6050

    // Calculate time step
    double dt = (currentMillis - previousMillis) / 1000.0;

    // Apply Kalman filter to get filtered angle
    kalmanAngle = kalmanFilter(rawTheta, rawOmega, dt);

    // Display variables for Serial Monitor
    Serial.print("Kalman Angle: ");
    Serial.print(kalmanAngle);
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
    Serial.print(" | Bias: ");
    Serial.println(bias);
  }

  // State Feedback Control
  double stateControlSignal = -Kp_theta * kalmanAngle - Kd_omega * omega;

  // Compute PID output
  myPID.Compute();

  // Combine State Feedback and PID Control
  double controlSignal = stateControlSignal + pidOutput;

  // Set motor speed using combined control signal
  setMotorSpeeds(controlSignal);

  delay(10);
}

// Function to handle Serial commands to tune parameters
void serialCommandHandler() {
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
    } else if (command.startsWith("Q_angle")) {
      Q_angle = command.substring(8).toFloat();
      Serial.print("Q_angle updated to: ");
      Serial.println(Q_angle);
    } else if (command.startsWith("Q_bias")) {
      Q_bias = command.substring(7).toFloat();
      Serial.print("Q_bias updated to: ");
      Serial.println(Q_bias);
    } else if (command.startsWith("R_measure")) {
      R_measure = command.substring(10).toFloat();
      Serial.print("R_measure updated to: ");
      Serial.println(R_measure);
    }
  }
}
