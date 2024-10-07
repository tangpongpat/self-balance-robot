#include <Wire.h>
#include <MPU6050_light.h>
#include <PID_v1.h>

// กำหนดขาพินสำหรับมอเตอร์และเซ็นเซอร์ MPU6050
#define DIR1 4    // Direction pin for Motor A
#define PWM1 2    // PWM pin for Motor A
#define DIR2 13   // Direction pin for Motor B
#define PWM2 12   // PWM pin for Motor B

// PWM configuration for ESP32
const int pwmFrequency = 10000;  // PWM frequency set to 10kHz
const int pwmResolution = 12;     // PWM resolution set to 12-bit
const int pwmChannel1 = 0;       // PWM channel for Motor A
const int pwmChannel2 = 1;       // PWM channel for Motor B

// กำหนดพารามิเตอร์ของหุ่นยนต์
const float WHEEL_RADIUS = 0.0345;
const float WHEEL_DISTANCE = 0.170;
int deadband = 10;
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
double setpoint = 0;
double input = 0;
double output = 0;
double Kp = 9.6, Ki = 0, Kd = 0;
unsigned long previousMillis = 0;
const long interval = 5;

int minPWMA = 1400;  // Default minimum PWM for Motor A
int minPWMB = 1400;   // Default minimum PWM for Motor B
// สร้าง PID object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ฟังก์ชันควบคุมความเร็วของมอเตอร์
// ฟังก์ชันควบคุมความเร็วของมอเตอร์
void setMotorSpeeds(double speed) {
    // เช็คว่า speed อยู่ในขอบเขต deadband หรือไม่
    if (abs(speed) < deadband) {
        ledcWrite(pwmChannel1, 0);  // หยุดมอเตอร์ A
        ledcWrite(pwmChannel2, 0);  // หยุดมอเตอร์ B
        return;  // ออกจากฟังก์ชันโดยไม่ทำงานต่อ
    }

    int maxPWM = pow(2, pwmResolution) - 1;  // Max PWM value for the resolution
    
    // กำหนดค่า PWM แยกสำหรับมอเตอร์แต่ละข้าง
    int pwmValueA = map(abs(speed), 0, 4095, minPWMA, maxPWM);  // Adjust PWM for Motor A
    int pwmValueB = map(abs(speed), 0, 4095, minPWMB, maxPWM);  // Adjust PWM for Motor B// PWM สำหรับ Motor B

    // Set Motor A direction and PWM value
    if (speed > 0) {
        digitalWrite(DIR1, LOW);
    } else {
        digitalWrite(DIR1, HIGH);
    }
    ledcWrite(pwmChannel1, pwmValueA);  // Set PWM for Motor A
    
    // Set Motor B direction and PWM value
    if (speed > 0) {
        digitalWrite(DIR2, HIGH);
    } else {
        digitalWrite(DIR2, LOW);
    }
    ledcWrite(pwmChannel2, pwmValueB);  // Set PWM for Motor B
}



void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }

  // Initialize PWM for Motor A
  ledcSetup(pwmChannel1, pwmFrequency, pwmResolution);  // Set frequency and resolution for Motor A
  ledcAttachPin(PWM1, pwmChannel1);  // Attach PWM pin for Motor A

  // Initialize PWM for Motor B
  ledcSetup(pwmChannel2, pwmFrequency, pwmResolution);  // Set frequency and resolution for Motor B
  ledcAttachPin(PWM2, pwmChannel2);  // Attach PWM pin for Motor B

  // Set up MPU6050
  mpu.calcOffsets();  // Initial calibration

  // Set PID mode
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-4095, 4095);  // Limit the output to fit the motor range

  // Set pin modes for motor direction
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

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
    float rawAngleX = -mpu.getAngleX();  // รับค่ามุม X จาก MPU6050 (ค่าดิบ)
    input = kalmanX.update(rawAngleX);   // ใช้ Kalman Filter กรองข้อมูลมุม X

    if (abs(input) > 0.01) {
      myPID.Compute();
      setMotorSpeeds(output);

      // แสดงค่ามุมที่กรองแล้วออกทาง Serial
      Serial.print("Filtered AngleX: ");
      Serial.println(input);  // แสดงค่ามุมที่ผ่าน Kalman Filter
    } else {
      Serial.println("Robot is stable.");
    }

    // ส่งค่ามุมดิบ (rawAngleX) และค่ามุมที่กรองแล้ว (input) ไปยัง Serial Plotter เพื่อดูกราฟ
    Serial.print(rawAngleX);  // ส่งค่ามุมดิบไปยัง Plotter
    Serial.print(", ");
    Serial.println(input);    // ส่งค่ามุมที่ผ่าน Kalman Filter ไปยัง Plotter
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
    float newSetpoint = command.substring(5).toFloat();  // รับค่า setpoint จาก Serial
    setpoint = newSetpoint;  // อัพเดตค่า setpoint สำหรับ PID control
    Serial.print("New Setpoint: ");
    Serial.println(setpoint);
  } else if (command.startsWith("db ")) {
    int newDeadband = command.substring(3).toInt();  // รับค่า deadband จาก Serial
    deadband = newDeadband;
    Serial.print("New Deadband: ");
    Serial.println(deadband);
  }else if (command.startsWith("CAL")) {  // Add calibration command
    Serial.println("Recalibrating MPU6050...");
    mpu.calcOffsets();  // Recalibrate MPU6050
    Serial.println("Calibration Complete!");
  }else if(command.startsWith("minA ")) {
    minPWMA = command.substring(5).toInt();
    Serial.print("New minPWM for Motor A: ");
    Serial.println(minPWMA);
  } else if (command.startsWith("minB ")) {
    minPWMB = command.substring(5).toInt();
    Serial.print("New minPWM for Motor B: ");
    Serial.println(minPWMB);}
  else {
    Serial.println("Invalid command. Use 'KP', 'KI', 'KD', 'Q', 'R', 'P', or 'CAL'.");
  }
}
