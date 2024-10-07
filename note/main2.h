#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

// กำหนดขาพินสำหรับมอเตอร์และเซ็นเซอร์ MPU6050
#define DIR1 4    // Direction pin for Motor A
#define PWM1 2    // PWM pin for Motor A
#define DIR2 13   // Direction pin for Motor B
#define PWM2 12   // PWM pin for Motor B

// กำหนดพารามิเตอร์ของหุ่นยนต์
const float WHEEL_RADIUS = 0.0345;
const float WHEEL_DISTANCE = 0.170;

// สร้างออบเจ็กต์ของ MPU6050
MPU6050 mpu(Wire);

// Kalman Filter Class
class KalmanFilter {
  public:
    KalmanFilter(float processNoise, float sensorNoise, float estimatedError, float initialValue) {
      this->q = processNoise;
      this->r = sensorNoise;
      this->p = estimatedError;
      this->x = initialValue;
      this->k = 0;
    }

    float update(float measurement) {
      this->p = this->p + this->q;
      this->k = this->p / (this->p + this->r);
      this->x = this->x + this->k * (measurement - this->x);
      this->p = (1 - this->k) * this->p;
      return this->x;
    }

  public:  
    float q;  // Process noise covariance
    float r;  // Measurement noise covariance
    float p;  // Estimation error covariance
    float k;  // Kalman gain
    float x;  // Estimated value
};

// Prototype สำหรับฟังก์ชัน handleCommand()
void handleCommand(String command);

// Create Kalman filter object for MPU6050 data
KalmanFilter kalmanX(0.1, 0.1, 0.1, 0);

// ตัวแปรสำหรับ PID control
double setpoint = -1;
double input = 0;
double output = 1;
double Kp = 5.65, Ki = 7, Kd = 1;
unsigned long previousMillis = 0;
const long interval = 10;

// สร้าง PID object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ฟังก์ชันควบคุมความเร็วของมอเตอร์
void setMotorSpeeds(double speed) {
  if (speed > 0) {
    digitalWrite(DIR1, HIGH);
  } else {
    digitalWrite(DIR1, LOW);
  }
  analogWrite(PWM1, abs(speed));
  

  if (speed > 0) {
    digitalWrite(DIR2, LOW);
  } else {
    digitalWrite(DIR2, HIGH);
  }
  analogWrite(PWM2, abs(speed));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }

  mpu.calcOffsets(); // Initial calibration
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-200, 200);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  Serial.println("MPU6050 Initialized and Calibrated");
  Serial.println("Ready for PID tuning through Serial Monitor");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      handleCommand(command);  // ฟังก์ชันจัดการคำสั่ง Serial
    }

    mpu.update();
    float rawAngleX = -mpu.getAngleX();
    input = kalmanX.update(rawAngleX);

    if (abs(input) > 0.01) {
      myPID.Compute();
      setMotorSpeeds(output);
      
      // แสดงค่ามุมและความเร็วของมอเตอร์ออกทาง Serial เพื่อดูกราฟ
      Serial.print("AngleX: ");
      Serial.print(input);  // แสดงค่ามุมที่ผ่าน Kalman Filter
      Serial.print(" | Motor Speed: ");
      Serial.println(output);
    } else {
      Serial.println("Robot is stable.");
    }
    
    // ส่งค่ามุมไปยัง Serial Plotter เพื่อสร้างกราฟ
    Serial.print("AngleX: ");
    Serial.println(input);  // ส่งค่ามุมไปยัง Plotter
  }
}
// ฟังก์ชันจัดการคำสั่งจาก Serial Monitor
void handleCommand(String command) {
  command.trim();  // ลบช่องว่างด้านหน้าและหลัง

  if (command.startsWith("KP ")) {
    double newKp = command.substring(3).toDouble();
    Kp = newKp;
    myPID.SetTunings(Kp, Ki, Kd);
    Serial.print("New Kp: ");
    Serial.println(Kp);
  }  else if (command.startsWith("out ")) {
    double newKd = command.substring(3).toDouble();
    Kd = newKd;
    myPID.SetTunings(Kp, Ki, Kd);
    Serial.print("out: ");
    Serial.println(output);
  }else if (command.startsWith("in ")) {
    double newKd = command.substring(3).toDouble();
    Kd = newKd;
    myPID.SetTunings(Kp, Ki, Kd);
    Serial.print("in: ");
    Serial.println(input);
  }else if (command.startsWith("KI ")) {
    double newKi = command.substring(3).toDouble();
    Ki = newKi;
    myPID.SetTunings(Kp, Ki, Kd);
    Serial.print("New Ki: ");
    Serial.println(Ki);
  }
  else if (command.startsWith("KD ")) {
    double newKd = command.substring(3).toDouble();
    Kd = newKd;
    myPID.SetTunings(Kp, Ki, Kd);
    Serial.print("New Kd: ");
    Serial.println(Kd);
  }
  else if (command.startsWith("Q ")) {
    float newQ = command.substring(2).toFloat();
    kalmanX.q = newQ;
    Serial.print("New Q: ");
    Serial.println(kalmanX.q);
  }
  else if (command.startsWith("R ")) {
    float newR = command.substring(2).toFloat();
    kalmanX.r = newR;
    Serial.print("New R: ");
    Serial.println(kalmanX.r);
  }
  else if (command.startsWith("P ")) {
    float newP = command.substring(2).toFloat();
    kalmanX.p = newP;
    Serial.print("New P: ");
    Serial.println(kalmanX.p);
  }else if (command.startsWith("setp ")) {
    float newP = command.substring(2).toFloat();
    kalmanX.p = newP;
    Serial.print("setp P: ");
    Serial.println(setpoint);
  }
  else if (command.startsWith("CAL")) {  // Add calibration command
    Serial.println("Recalibrating MPU6050...");
    mpu.calcOffsets();  // Recalibrate MPU6050
    Serial.println("Calibration Complete!");
  }
  else {
    Serial.println("Invalid command. Use 'KP', 'KI', 'KD', 'Q', 'R', 'P', or 'CAL'.");
  }
}
