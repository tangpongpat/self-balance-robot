#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

// Define pin numbers for motors and MPU6050 sensor
#define DIR1 4    // Direction pin for Motor A
#define PWM1 2    // PWM pin for Motor A (using PWM channel 0)
#define DIR2 13   // Direction pin for Motor B
#define PWM2 12   // PWM pin for Motor B (using PWM channel 1)

// Define robot parameters
const float WHEEL_RADIUS = 0.0345;  // Wheel radius 34.5 mm
const float WHEEL_DISTANCE = 0.170; // Distance between wheels 170 mm

// Create an object for MPU6050
MPU6050 mpu(Wire);

// Variables for PID control
double setpoint = 0;    // Balance point (0 degrees)
double input = 0;       // Angle from MPU6050 (tilt angle in X-axis)
double output = 0;      // PID output for motor control
double Kp = 5.5, Ki = 4.3, Kd = 0.1;  // PID tuning values
double deadband = 0.13; // Deadband value for angle adjustments
unsigned long previousMillis = 0; // Stores the last time the angle was updated
const long interval = 100; // Interval for updating angle (100 ms)

// Create a PID object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Function to control motor speeds
void setMotorSpeeds(double speed) {
  if (speed > 0) {
    digitalWrite(DIR1, HIGH);  // Motor A rotates forward
  } else {
    digitalWrite(DIR1, LOW);   // Motor A rotates backward
  }
  ledcWrite(0, abs(speed)); // Use PWM channel 0 for Motor A

  if (speed > 0) {
    digitalWrite(DIR2, LOW);   // Motor B rotates forward
  } else {
    digitalWrite(DIR2, HIGH);  // Motor B rotates backward
  }
  ledcWrite(1, abs(speed));  // Use PWM channel 1 for Motor B
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  // Start MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  mpu.calcOffsets();  // Calibrate MPU6050
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-250, 250);

  // Set pins as output for motors
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  // Configure PWM channels for ESP32
  ledcSetup(0, 5000, 8); // PWM channel 0 at 5kHz frequency and 8-bit resolution
  ledcSetup(1, 5000, 8); // PWM channel 1 at 5kHz frequency and 8-bit resolution

  // Attach PWM channels to motor pins
  ledcAttachPin(PWM1, 0); // Attach PWM channel 0 to PWM1 pin
  ledcAttachPin(PWM2, 1); // Attach PWM channel 1 to PWM2 pin
  
  Serial.println("MPU6050 Initialized and Calibrated");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Update MPU6050 data and send only the angle to Serial Plotter
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Update data from MPU6050
    mpu.update();
    input = -mpu.getAngleX(); // Read X-axis angle from MPU6050
    
    // Send only the angle to the Serial Plotter (graph will show angle from MPU6050)
    Serial.println(input); // For Serial Plotter (shows only the angle)
    
    // Continuously display updated values on Serial Monitor
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Kp: ");
    Serial.print(Kp);
    Serial.print(" | Ki: ");
    Serial.print(Ki);
    Serial.print(" | Kd: ");
    Serial.print(Kd);
    Serial.print(" | Deadband: ");
    Serial.println(deadband);
  }

  // Check if any command is sent through Serial Monitor
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    
    // If the received command is "NEW_CALIBRATE"
    if (command == "NEW_CALIBRATE") {
      mpu.calcOffsets();
      Serial.println("NEW CALIBRATION DONE");
    } 
    // If the command starts with "SETPOINT"
    else if (command.startsWith("SETPOINT")) {
      setpoint = command.substring(9).toFloat();
      Serial.print("Setpoint updated to: ");
      Serial.println(setpoint);
    } 
    // If the command starts with "Kp"
    else if (command.startsWith("Kp")) {
      Kp = command.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kp updated to: ");
      Serial.println(Kp);
    } 
    // If the command starts with "Ki"
    else if (command.startsWith("Ki")) {
      Ki = command.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Ki updated to: ");
      Serial.println(Ki);
    } 
    // If the command starts with "Kd"
    else if (command.startsWith("Kd")) {
      Kd = command.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kd updated to: ");
      Serial.println(Kd);
    } 
    // If the command starts with "DEADBAND"
    else if (command.startsWith("DEADBAND")) {
      deadband = command.substring(9).toFloat(); // Receive deadband value via Serial
      Serial.print("Deadband updated to: ");
      Serial.println(deadband);
    }
  }

  // Use Deadband to check if the angle change is significant
  if (abs(input) > deadband) {
    myPID.Compute(); // Compute PID value
    setMotorSpeeds(output); // Set motor speed based on PID output
  }

  delay(100);  // Delay for 100 milliseconds (adjust as needed)
}
