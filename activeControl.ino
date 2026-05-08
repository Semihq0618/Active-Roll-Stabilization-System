#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP32Servo.h>

#pragma region MPU6050

#define MPU_SDA    21   // MPU6050 SDA → IO4
#define MPU_SCL    22   // MPU6050 SCL → IO5
#define INTERRUPT_PIN 2

float ax;
float ay;
float az;

float yaw;
float pitch;
float roll;

float yawDeg;
float pitchDeg;
float rollDeg;

MPU6050 mpu;

// MPU6050 DMP için
 
volatile bool MPUInterrupt = false;
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];  // Yaw, Pitch, Roll

// MPU6050 DMP Interrupt Handler
void DMPDataReady() {
  MPUInterrupt = true;
}
#pragma endregion

#pragma region PID

#define SERVO_PIN1 13
#define SERVO_PIN2 12
#define SERVO_PIN3 14
#define SERVO_PIN4 27

// --- PID PARAMETRELERİ (Tuning Gerektirir) ---
float Kp = 1;   // Oransal çarpan (Hızlı tepki)
float Kd = 0;   // Türev çarpan (Sönümleme)
float Ki = 0;  // Integral çarpan (Hata birikimi)

float rollError, lastRollError, rollIntegral, rollDerivative;
float setPoint = 0; // Hedef: 0 derece roll
float pidOutput = 0;

// --- ZAMANLAMA ---
unsigned long lastTime;
const int sampleTime = 100; // 10ms = 100Hz döngü hızı

Servo rollServo1;
Servo rollServo2;
Servo rollServo3;
Servo rollServo4;

#pragma endregion

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);
  
  #pragma region MPU6050 setup
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
mpu.setXAccelOffset(4381);
    mpu.setYAccelOffset(5439);
    mpu.setZAccelOffset(9029);
    mpu.setXGyroOffset(89);
    mpu.setYGyroOffset(-147);
    mpu.setZGyroOffset(-1);
    // mpu.CalibrateAccel(15);
    // mpu.CalibrateGyro(15);
    mpu.PrintActiveOffsets();
    
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    Serial.println("DMP ready!");
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
  #pragma endregion

  #pragma region PID setup

  // Servo kurulumu
  rollServo1.attach(SERVO_PIN1);
  rollServo2.attach(SERVO_PIN2);
  rollServo3.attach(SERVO_PIN3);
  rollServo4.attach(SERVO_PIN4);

  rollServo1.write(90);
  rollServo2.write(90);
  rollServo3.write(90);
  rollServo4.write(90);

  #pragma endregion

}

void loop() {
  // MPU6050'dan verileri oku (sadece vertical ivme için)
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
    mpu.dmpGetQuaternion(&q, FIFOBuffer);

    #pragma region IMU Rotation
    // --- YAZILIMSAL SENSÖR DÖNDÜRME İŞLEMİ BAŞLANGICI ---
    // Quaternion değerlerini geçici değişkenlere alıyoruz
    float qw = q.w;
    float qx = q.x;
    float qy = q.y;
    float qz = q.z;
    float sq2 = 0.70710678f; // Kök(2)/2 değeri (Matematiksel 90 derece dönüş sabiti)

    // KARTINIZIN NASIL TAKILDIĞINA GÖRE AŞAĞIDAKİLERDEN SADECE BİRİNİ SEÇİN:

    /* DURUM 1: Sensör Y ekseni etrafında 90 derece dönükse 
      (Örn: Kart dik takılmış, Z ekseni yukarı değil ileri bakıyor) */
    // q.w = sq2 * (qw - qy);
    // q.x = sq2 * (qx - qz);
    // q.y = sq2 * (qw + qy);
    // q.z = sq2 * (qx + qz);

    /* DURUM 2: Sensör X ekseni etrafında 90 derece dönükse 
      (Örn: Kart diğer yönde dik takılmış) */
    // q.w = sq2 * (qw - qx);
    // q.x = sq2 * (qw + qx);
    // q.y = sq2 * (qy + qz);
    // q.z = sq2 * (qz - qy);

    /* DURUM 3: Sensör Z ekseni etrafında 90 derece dönükse 
      (Örn: Kart yatay ama ileri değil, sağa/sola bakıyor) */
    q.w = sq2 * (qw - qz);
    q.x = sq2 * (qx + qy);
    q.y = sq2 * (qy - qx);
    q.z = sq2 * (qw + qz);

    // Not: Eğer yönler tam tersi çıkarsa (90 yerine -90 gerekiyorsa), 
    // formüllerdeki eksi (-) ve artı (+) işaretlerinin yerlerini değiştirin.
    // --- YAZILIMSAL DÖNDÜRME İŞLEMİ BİTİŞİ ---
    #pragma endregion

    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Bu satır eklendi, böylece ypr güncellenir.
  }
  ax = gravity.x;
  ay = gravity.y;
  az = gravity.z;

  yaw = ypr[0];
  pitch = ypr[1];
  roll = ypr[2];

  yawDeg = ypr[0] * 180.0 / PI;
  pitchDeg = ypr[1] * 180.0 / PI;
  rollDeg = ypr[2] * 180.0 / PI;

  // Serial.print("yaw: "); Serial.print(yawDeg);
  // Serial.print("\t pitch: "); Serial.print(pitchDeg);
  // Serial.print("\t roll: "); Serial.println(rollDeg);

  // Serial.print("ax: "); Serial.print(ax);
  // Serial.print("\t ay: "); Serial.print(ay);
  // Serial.print("\t az: "); Serial.print(az);

  // --- PID HESAPLAMA DÖNGÜSÜ ---
  unsigned long now = millis();
  float dt = (float)(now - lastTime) / 1000.0;

  if (dt >= (float)sampleTime / 1000.0) {
    // 1. Hata hesabı
    rollError = setPoint - rollDeg;

    // 2. Integral hesabı (Rüzgar vb. etkiler için)
    rollIntegral += rollError * dt;
    // Integral Windup koruması (Limit koymak iyidir)
    rollIntegral = constrain(rollIntegral, -20, 20);

    // 3. Türev hesabı (Hız değişimi)
    rollDerivative = (rollError - lastRollError) / dt;

    // 4. PID Çıkışı
    pidOutput = (Kp * rollError) + (Ki * rollIntegral) + (Kd * rollDerivative);

    // 5. Servoya Uygula
    // Servonun orta noktası 90'dır. PID çıkışını üzerine ekliyoruz.
    int servoTarget = 90 + pidOutput;
    servoTarget = constrain(servoTarget, 45, 135); // def 45,135 // Servonun mekanik limiti
    
    rollServo1.write(servoTarget);
    rollServo2.write(servoTarget);
    rollServo3.write(90 - pidOutput);
    rollServo4.write(servoTarget);

    // Değerleri güncelle
    lastRollError = rollError;
    lastTime = now;

    // Debug
    Serial.print("yaw: "); Serial.print(yawDeg);
    Serial.print("\t pitch: "); Serial.print(pitchDeg);
    Serial.print("\t roll: "); Serial.println(rollDeg);

    Serial.print("Roll: "); Serial.print(rollDeg);
    Serial.print(" | PID Out: "); Serial.print(pidOutput);
    Serial.print(" | Servo: "); Serial.println(servoTarget);
  }
}