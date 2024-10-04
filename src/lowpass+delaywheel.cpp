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

double setpoint = 0;
double input = 0;
double output = 0;
double Kp = 5.5, Ki = 4.3, Kd = 0.1;
double deadband = 0.13;
unsigned long previousMillis = 0;
const long interval = 100;

float filterConstant = 0.1;  // Low-pass filter constant

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setMotorSpeeds(double speed) {
  // Set direction and speed for Motor A
  if (speed > 0) {
    digitalWrite(DIR1, HIGH);
  } else {
    digitalWrite(DIR1, LOW);
  }
  ledcWrite(0, abs(speed)); // PWM for Motor A

  // Add a small delay to ensure both motors are synchronized (optional)
  delay(1); // Delay 1 ms, you can adjust this as needed

  // Set direction and speed for Motor B
  if (speed > 0) {
    digitalWrite(DIR2, LOW);
  } else {
    digitalWrite(DIR2, HIGH);
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
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-250, 250);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);

  ledcAttachPin(PWM1, 0);
  ledcAttachPin(PWM2, 1);

  Serial.println("MPU6050 Initialized and Calibrated");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    mpu.update();
    float rawInput = -mpu.getAngleX(); // Get raw sensor reading
    input = (filterConstant * rawInput) + ((1 - filterConstant) * input); // Low-pass filter

    Serial.println(input);

    // Display current tuning parameters and filter constant
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Kp: ");
    Serial.print(Kp);
    Serial.print(" | Ki: ");
    Serial.print(Ki);
    Serial.print(" | Kd: ");
    Serial.print(Kd);
    Serial.print(" | Deadband: ");
    Serial.print(deadband);
    Serial.print(" | Filter Constant: ");
    Serial.println(filterConstant);
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command == "NEW_CALIBRATE") {
      mpu.calcOffsets();
      Serial.println("NEW CALIBRATION DONE");
    } else if (command.startsWith("SETPOINT")) {
      setpoint = command.substring(9).toFloat();
      Serial.print("Setpoint updated to: ");
      Serial.println(setpoint);
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
    } else if (command.startsWith("DEADBAND")) {
      deadband = command.substring(9).toFloat();
      Serial.print("Deadband updated to: ");
      Serial.println(deadband);
    } else if (command.startsWith("FILTER")) {
      filterConstant = command.substring(7).toFloat();
      Serial.print("Filter Constant updated to: ");
      Serial.println(filterConstant);
    }
  }

  if (abs(input) > deadband) {
    myPID.Compute();
    setMotorSpeeds(output);
  }

  delay(100);
}
