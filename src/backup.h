#include <dummy.h>

//Activate the motor to check the linear speed and angular speed values of the encoder.
//Conversion of the encoder into linear velocity and angular velocity values.
//Closed loop speed control (rad/s)
// Enter linear and angular velocity values and control the closed loop of omega.
// Balance Robot with IMU PID
//Enter the angular speed and linear speed values,
//then the system will convert that speed to the angular speed of each wheel and control the wheels with pid.
//Caution: This closed loop system method cannot control the robot's angular velocity and linear velocity. !!!!!!!!
#include <PID_v1.h>
#include <Wire.h>
#include <TB6612_ESP32.h>

//-------------------------------- MPU6050 --------------------------------//

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

float LoopTimer;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float KalmanAngleYaw = 0, KalmanUncertaintyAngleYaw = 2 * 2;

float Kalman1DOutput[] = { 0, 0 };

void kalman_1d(float KalmanState,
               float KalmanUncertainty, float KalmanInput,
               float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);

  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4069 - 0.05;
  AccY = (float)AccYLSB / 4069 + 0.01;
  AccZ = (float)AccZLSB / 4069 - 0.01;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

//-------------------------------- MPU6050 --------------------------------//



//-------------------------------- Drive and Encoder --------------------------------//
// Define encoder pins
#define ENCODER_A1 32  // Pin for Encoder A1
#define ENCODER_B1 33  // Pin for Encoder B1
#define ENCODER_A2 34  // Pin for Encoder A2
#define ENCODER_B2 35  // Pin for Encoder B2

// Global variables for storing the encoder positions
volatile int encoder_m1_value = 0;
volatile int encoder_m2_value = 0;
volatile long last_encoder_m1_value = 0;
volatile long last_encoder_m2_value = 0;
unsigned long last_time = 0;
unsigned long last_display_time = 0;
unsigned long current_time = 0;
double delta_time = 0;
double speed_m1, speed_m2 = 0;
double robot_linear_velocity, robot_angular_velocity = 0;

// Define wheel characteristics
#define WHEEL_RADIUS 0.034       // Radius of the wheel
#define WHEEL_DISTANCE 0.227     // Distance between wheels
#define REVOLUTION_STEPS 1980.0  // Steps per wheel revolution


// Motor controller pins and settings
#define AIN1 13  // Pin for TB6612 AIN1
#define BIN1 12  // Pin for TB6612 BIN1
#define AIN2 14  // Pin for TB6612 AIN2
#define BIN2 27  // Pin for TB6612 BIN2
#define PWMA 26  // Pin for TB6612 PWMA
#define PWMB 25  // Pin for TB6612 PWMB
#define STBY 23  // Pin for TB6612 STBY

// Motor direction constants
const int offsetA = 1;  // Forward direction for motor A, -1 Backward
const int offsetB = 1;  // Forward direction for motor B, -1 Backward

// Motor objects
// Motor(pin_direction, pin_direction, pin_pwm, offset, pin_STBY, Hz, Bit, number_motor);
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);

//-------------------------------- Drive and Encoder --------------------------------//

//-------------------------------- PID Velocity --------------------------------//
// PID parameters Speed
double Kp_speed = 20.0;   // Proportional gain
double Ki_speed = 500.0;  // Integral gain
double Kd_speed = 0.0;    // Derivative gain

// PID Input and Output
double input_speed1, input_speed2 = 0;
double output_speed1, output_speed2 = 0;
// PID Setpoint
double setpoint_speed1, setpoint_speed2 = 0;
// PID objects
PID pid_speed1(&input_speed1, &output_speed1, &setpoint_speed1, Kp_speed, Ki_speed, Kd_speed, DIRECT);
PID pid_speed2(&input_speed2, &output_speed2, &setpoint_speed2, Kp_speed, Ki_speed, Kd_speed, DIRECT);
//---------------------------------------------------------------------------------------------------------//

double velocity1, velocity2 = 0;
double linear_velocity, angular_velocity = 0;
//-------------------------------- PID Velocity --------------------------------//
//-------------------------------- PID Balance Robot --------------------------------//
// PID parameters Speed
double Kp_balance = 1.5;   // Proportional gain
double Ki_balance = 50.0;  // Integral gain
double Kd_balance = 0.05;  // Derivative gain

// PID Input and Output
double input_balance = 0;
double output_balance = 0;
// PID Setpoint
double setpoint_balance = -1.5;
// PID objects
PID pid_balance(&input_balance, &output_balance, &setpoint_balance, Kp_balance, Ki_balance, Kd_balance, DIRECT);

bool check1, check2 = false;
//-------------------------------- PID Balance Robot --------------------------------//

// Interrupt Service Routine (ISR) for Encoder 1
void encoder_m1_isr() {
  // Read current state of encoder A1 and B1
  int EA1 = digitalRead(ENCODER_A1);
  int EB1 = digitalRead(ENCODER_B1);

  // Determine rotation direction and update count
  if ((EA1 == HIGH) != (EB1 == LOW)) {
    encoder_m1_value--;
  } else {
    encoder_m1_value++;
  }
}
// Interrupt Service Routine (ISR) for Encoder 2
void encoder_m2_isr() {
  // Read current state of encoder A2 and B2
  int EA2 = digitalRead(ENCODER_A2);
  int EB2 = digitalRead(ENCODER_B2);

  // Determine rotation direction and update count
  if ((EA2 == HIGH) != (EB2 == LOW)) {
    encoder_m2_value--;
  } else {
    encoder_m2_value++;
  }
}

// Function to control the motors
void move_robot(int speedM1, int speedM2) {
  // Apply speed constraints and control motors
  //speedM < 0 is backward or ccw
  //speedM > 0 is forward or cw
  //other Stop Motor

  if (speedM1 < 0) {
    speedM1 = constrain(speedM1, -255, 255);

  } else if (speedM1 > 0) {
    speedM1 = constrain(speedM1, -255, 255);
  } else {
    motor1.brake();
  }

  if (speedM2 < 0) {
    speedM2 = constrain(speedM2, -255, 255);

  } else if (speedM2 > 0) {
    speedM2 = constrain(speedM2, -255, 255);

  } else {
    motor2.brake();
  }
  // Drive the motors with the given speeds
  motor1.drive(speedM1, 0);
  motor2.drive(speedM2, 0);
}

// Function to calculate and display speed and angular velocity
void calculateSpeedAndAngularVelocity() {

  current_time = millis();
  delta_time = (current_time - last_time) / 1000.0;  // change time from milliseconds to seconds

  if (delta_time >= 0.01) {  // update 0.01 sec or 10 milliseconds

    //Calculate linear velocity for robots omega rad/s .
    speed_m1 = 2 * PI * (encoder_m1_value - last_encoder_m1_value);
    speed_m1 = speed_m1 / (REVOLUTION_STEPS * delta_time);

    speed_m2 = 2 * PI * (encoder_m2_value - last_encoder_m2_value);
    speed_m2 = speed_m2 / (REVOLUTION_STEPS * delta_time);

    //Calculate linear velocity for robots.
    robot_linear_velocity = WHEEL_RADIUS / 2;  // R / 2 * (omegaR + omegaL)
    robot_linear_velocity = (robot_linear_velocity * (speed_m1 + speed_m2));

    //Calculate angular velocity for robots.
    robot_angular_velocity = WHEEL_RADIUS / WHEEL_DISTANCE;  // R / L * (omegaR - omegaL)
    robot_angular_velocity = (robot_angular_velocity * (speed_m1 - speed_m2));

    // Speed input
    input_speed1 = speed_m1;  // อัปเดตค่า input สำหรับ Speed 1
    input_speed2 = speed_m2;  // อัปเดตค่า input สำหรับ Speed 2

    // Velocity input
    velocity1 = (2 * linear_velocity) + (angular_velocity * WHEEL_DISTANCE);
    velocity1 = (velocity1 / (2 * WHEEL_RADIUS));  // (2*V + omega*L)/2*R

    velocity2 = (2 * linear_velocity) - (angular_velocity * WHEEL_DISTANCE);
    velocity2 = (velocity2 / (2 * WHEEL_RADIUS));  // (2*V - omega*L)/2*R


    // Update global variables for the next calculation
    last_encoder_m1_value = encoder_m1_value;
    last_encoder_m2_value = encoder_m2_value;
    last_time = current_time;
  }

  // Check if 0.1 seconds have passed since last display
  if (current_time - last_display_time >= 100) {

    Serial.print("  ");
    Serial.print(setpoint_balance, 3);
    Serial.print("  ");
    Serial.print(setpoint_speed1, 3);
    Serial.print("  ");
    Serial.print(setpoint_speed2, 3);
    Serial.print("  ");
    Serial.print(velocity1, 3);
    Serial.print("  ");
    Serial.println(velocity2, 3);

    // Display speed and angular velocity
    // Serial.print(" Speed M1: ");
    // Serial.print(speed_m1, 3);
    // Serial.print(" rad/s, Speed M2: ");
    // Serial.print(speed_m2, 3);

    Serial.print("output_balance ");
    Serial.print(output_balance);

    Serial.print(" Roll Angle [°] ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" linear_velocity: ");
    Serial.print(robot_linear_velocity, 3);
    Serial.print(" m/s, Angular Velocity: ");
    Serial.print(robot_angular_velocity, 3);
    Serial.println(" rad/s");

    last_display_time = current_time;  // Update the last display time
  }
}


void readSerialAndSetParameters() {

  // if (Serial.available() > 0) {
  //   String inputString = Serial.readStringUntil('\n');

  //   int firstCommaIndex = inputString.indexOf(',');
  //   int secondCommaIndex = inputString.indexOf(',', firstCommaIndex + 1);
  //   int thirdCommaIndex = inputString.indexOf(',', secondCommaIndex + 1);
  //   int fourthCommaIndex = inputString.indexOf(',', thirdCommaIndex + 1);
  //   int fifthCommaIndex = inputString.indexOf(',', fourthCommaIndex + 1);

  //   // อ่านค่า speed_m1 และ speed_m2
  //   setpoint_speed1 = inputString.substring(0, firstCommaIndex).toFloat();
  //   setpoint_speed2 = inputString.substring(firstCommaIndex + 1).toFloat();

  //   // อ่านค่า PID
  //   Kp_speed = inputString.substring(secondCommaIndex + 1, thirdCommaIndex).toFloat();
  //   Ki_speed = inputString.substring(thirdCommaIndex + 1, fourthCommaIndex).toFloat();
  //   Kd_speed = inputString.substring(fourthCommaIndex + 1, fifthCommaIndex).toFloat();

  // if (setpoint_speed1 == 0 && setpoint_speed2 == 0) {
  //   output_speed1 = 0;
  //   output_speed2 = 0;
  //   Kp_speed = 0;
  //   Ki_speed = 0;
  //   Kd_speed = 0;
  //   pid_speed1.SetMode(MANUAL);
  //   pid_speed1.SetTunings(Kp_speed, Ki_speed, Kd_speed);
  //   pid_speed1.SetMode(AUTOMATIC);
  //   pid_speed2.SetMode(MANUAL);
  //   pid_speed2.SetTunings(Kp_speed, Ki_speed, Kd_speed);
  //   pid_speed2.SetMode(AUTOMATIC);
  //   Serial.println(" STOP ");
  // }
  //   else {
  //     Kp_speed = 20;
  //     Ki_speed = 500;
  //     Kd_speed = 0.0;
  //   }
  //   Serial.print("Motor 1 Speed: ");
  //   Serial.print(setpoint_speed1);
  //   Serial.print(", Motor 2 Speed: ");
  //   Serial.print(setpoint_speed2);

  //   Serial.print(", PID: P=");
  //   Serial.print(Kp_speed);
  //   Serial.print(", I=");
  //   Serial.print(Ki_speed);
  //   Serial.print(", D=");
  //   Serial.println(Kd_speed);
  // }

  // pid_speed1.SetTunings(Kp_speed, Ki_speed, Kd_speed);
  // pid_speed2.SetTunings(Kp_speed, Ki_speed, Kd_speed);


  // if (Serial.available() > 0) {
  //   String inputString = Serial.readStringUntil('\n');

  //   int firstCommaIndex = inputString.indexOf(',');
  //   int secondCommaIndex = inputString.indexOf(',', firstCommaIndex + 1);
  //   int thirdCommaIndex = inputString.indexOf(',', secondCommaIndex + 1);
  //   int fourthCommaIndex = inputString.indexOf(',', thirdCommaIndex + 1);
  //   int fifthCommaIndex = inputString.indexOf(',', fourthCommaIndex + 1);

  //   //linear_velocity max = 0.392 angular_velocity max = 3.454
  //   linear_velocity = inputString.substring(0, firstCommaIndex).toFloat();
  //   angular_velocity = inputString.substring(firstCommaIndex + 1).toFloat();

  //   Serial.print("SP linear_velocity: ");
  //   Serial.print(linear_velocity);
  //   Serial.print(", SP angular_velocity: ");
  //   Serial.println(angular_velocity);
  // }

  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');

    int firstCommaIndex = inputString.indexOf(',');
    int secondCommaIndex = inputString.indexOf(',', firstCommaIndex + 1);
    int thirdCommaIndex = inputString.indexOf(',', secondCommaIndex + 1);
    int fourthCommaIndex = inputString.indexOf(',', thirdCommaIndex + 1);

    setpoint_balance = inputString.substring(0, firstCommaIndex).toFloat();
    // อ่านค่า PID
    Kp_balance = inputString.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
    Ki_balance = inputString.substring(secondCommaIndex + 1, thirdCommaIndex).toFloat();
    Kd_balance = inputString.substring(thirdCommaIndex + 1, fourthCommaIndex).toFloat();

    // if (KalmanAngleRoll > 25 || KalmanAngleRoll < -25) {
    //   output_balance = 0;
    //   Kp_balance = 0;
    //   Ki_balance = 0;
    //   Kd_balance = 0;
    //   pid_balance.SetMode(MANUAL);
    //   pid_balance.SetTunings(Kp_balance, Ki_balance, Kd_balance);
    //   pid_balance.SetMode(AUTOMATIC);
    //   Serial.println(" STOP ");
    // }
    // else {
    //   Kp_speed = 20;
    //   Ki_speed = 500;
    //   Kd_speed = 0.0;
    // }
    Serial.print("Setpoint Balance: ");
    Serial.print(setpoint_balance);

    Serial.print(", PID: P=");
    Serial.print(Kp_balance);
    Serial.print(", I=");
    Serial.print(Ki_balance);
    Serial.print(", D=");
    Serial.println(Kd_balance);

    pid_balance.SetTunings(Kp_balance, Ki_balance, Kd_balance);
  }


  // if (Serial.available() > 0) {
  //   String inputString = Serial.readStringUntil('\n');

  //   int firstCommaIndex = inputString.indexOf(',');
  //   int secondCommaIndex = inputString.indexOf(',', firstCommaIndex + 1);
  //   int thirdCommaIndex = inputString.indexOf(',', secondCommaIndex + 1);
  //   int fourthCommaIndex = inputString.indexOf(',', thirdCommaIndex + 1);

  //   linear_velocity = inputString.substring(0, firstCommaIndex).toFloat();
  //   angular_velocity = inputString.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();

  //   Serial.print(" linear_velocity: ");
  //   Serial.print(linear_velocity);
  //   Serial.print(" angular_velocity: ");
  //   Serial.println(angular_velocity);
  // }
}


void setup() {
  delay(1000);

  Serial.begin(115200);  //Initialize serial communication

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0;
       RateCalibrationNumber < 2000;
       RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  LoopTimer = micros();
  digitalWrite(2, LOW);


  // Setup encoder pins as input pull-ups
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder_m1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder_m2_isr, CHANGE);

  // Initialize PID
  // Setting time to 10 milliseconds
  pid_speed1.SetSampleTime(10);
  pid_speed1.SetTunings(Kp_speed, Ki_speed, Kd_speed);
  pid_speed1.SetOutputLimits(-255, 255);  // Setting the output limits from -255 to 255
  pid_speed1.SetMode(AUTOMATIC);

  pid_speed2.SetSampleTime(10);
  pid_speed2.SetTunings(Kp_speed, Ki_speed, Kd_speed);
  pid_speed2.SetOutputLimits(-255, 255);  // Setting the output limits from -255 to 255
  pid_speed2.SetMode(AUTOMATIC);


  pid_balance.SetSampleTime(5);
  pid_balance.SetTunings(Kp_balance, Ki_balance, Kd_balance);
  pid_balance.SetOutputLimits(-11.424, 11.424);  // Setting the output limits from -255 to 255
  pid_balance.SetMode(AUTOMATIC);
}


void loop() {

  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll,
            KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch,
            KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Yaw calculation
  kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, RateYaw, 0);  // Assuming no external yaw measurement
  KalmanAngleYaw = Kalman1DOutput[0];
  KalmanUncertaintyAngleYaw = Kalman1DOutput[1];

  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();

  if (KalmanAngleRoll > 25 && check1 == false) {
    setpoint_speed1 = 0;
    output_speed1 = 0;
    pid_speed1.SetMode(MANUAL);
    pid_speed1.SetTunings(0.0, 0.0, 0.0);
    pid_speed1.SetMode(AUTOMATIC);
    output_speed2 = 0;
    pid_speed2.SetMode(MANUAL);
    pid_speed2.SetTunings(0.0, 0.0, 0.0);
    pid_speed2.SetMode(AUTOMATIC);
    setpoint_speed2 = 0;
    output_balance = 0;
    pid_balance.SetMode(MANUAL);
    pid_balance.SetTunings(0.0, 0.0, 0.0);
    pid_balance.SetMode(AUTOMATIC);

    check1 = true;
    Serial.println("------------------STOP Forward------------------");
  } else if (KalmanAngleRoll < 25 && check1 == true) {
    pid_speed1.SetMode(MANUAL);
    pid_speed1.SetTunings(20.0, 500.0, 0.0);
    pid_speed1.SetMode(AUTOMATIC);
    pid_speed2.SetMode(MANUAL);
    pid_speed2.SetTunings(20.0, 500.0, 0.0);
    pid_speed2.SetMode(AUTOMATIC);

    setpoint_balance = -1.5;
    pid_balance.SetMode(MANUAL);
    pid_balance.SetTunings(1.5, 50.0, 0.05);
    pid_balance.SetMode(AUTOMATIC);

    check1 = false;
    Serial.println("------------------START Forward------------------");
  }

  if (KalmanAngleRoll < -25 && check2 == false) {
    setpoint_speed1 = 0;
    output_speed1 = 0;
    pid_speed1.SetMode(MANUAL);
    pid_speed1.SetTunings(0.0, 0.0, 0.0);
    pid_speed1.SetMode(AUTOMATIC);
    output_speed2 = 0;
    pid_speed2.SetMode(MANUAL);
    pid_speed2.SetTunings(0.0, 0.0, 0.0);
    pid_speed2.SetMode(AUTOMATIC);
    setpoint_speed2 = 0;
    output_balance = 0;
    pid_balance.SetMode(MANUAL);
    pid_balance.SetTunings(0.0, 0.0, 0.0);
    pid_balance.SetMode(AUTOMATIC);

    check2 = true;
    Serial.println("------------------STOP Backward------------------");
  } else if (KalmanAngleRoll > -25 && check2 == true) {
    pid_speed1.SetMode(MANUAL);
    pid_speed1.SetTunings(20.0, 500.0, 0.0);
    pid_speed1.SetMode(AUTOMATIC);
    pid_speed2.SetMode(MANUAL);
    pid_speed2.SetTunings(20.0, 500.0, 0.0);
    pid_speed2.SetMode(AUTOMATIC);

    setpoint_balance = -1.5;
    pid_balance.SetMode(MANUAL);
    pid_balance.SetTunings(1.5, 50.0, 0.05);
    pid_balance.SetMode(AUTOMATIC);

    check2 = false;
    Serial.println("------------------START Backward------------------");
  }

  if (linear_velocity != 0.0) {
    setpoint_balance = (linear_velocity * 10) - 1.5;  // -2 offset 0
  } else if (linear_velocity && angular_velocity == 0.0) {
    setpoint_balance = -1.5;

    setpoint_speed1 = 0;
    output_speed1 = 0;
    pid_speed1.SetMode(MANUAL);
    pid_speed1.SetTunings(0.0, 0.0, 0.0);
    pid_speed1.SetMode(AUTOMATIC);
    output_speed2 = 0;
    pid_speed2.SetMode(MANUAL);
    pid_speed2.SetTunings(0.0, 0.0, 0.0);
    pid_speed2.SetMode(AUTOMATIC);
    setpoint_speed2 = 0;
    output_balance = 0;
    pid_balance.SetMode(MANUAL);
    pid_balance.SetTunings(0.0, 0.0, 0.0);
    pid_balance.SetMode(AUTOMATIC);
  }

  input_balance = KalmanAngleRoll;

  pid_balance.Compute();

  readSerialAndSetParameters();
  // Function Calculate and display the speeds
  calculateSpeedAndAngularVelocity();

  setpoint_speed1 = -output_balance;
  setpoint_speed2 = -output_balance;


  // setpoint_speed1 = (output_balance * -1) + (velocity1);
  // setpoint_speed2 = (output_balance * -1) + (velocity2);



  // เรียก Compute เพื่ออัปเดต PID
  pid_speed1.Compute();
  pid_speed2.Compute();


  // ใช้ pidOutput1 และ pidOutput2 เพื่อควบคุมมอเตอร์
  move_robot(output_speed1, output_speed1);
}