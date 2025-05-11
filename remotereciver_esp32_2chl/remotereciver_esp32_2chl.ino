#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <printf.h>
#include <Wire.h>

// MPU6500 I2C address
#define MPU6500_ADDR 0x68

// MPU6500 Register Map
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

// PID control parameters
#define KP 2.0    // Proportional gain
#define KI 0.1    // Integral gain
#define KD 1.0    // Derivative gain

// 自定义 SPI 引脚（MOSI, MISO, SCK）
#define MOSI_PIN 6
#define MISO_PIN 5
#define SCK_PIN  4
// CE 和 CSN 引脚
#define CE_PIN   20
#define CSN_PIN  10

//无线模块
RF24 radio(CE_PIN, CSN_PIN); 
//舵机&电调
Servo esc, aileron, elevator, rudder;

unsigned long lastSignalTime = 0;

//管脚定义
#define PIN_LED 7             // 信号灯
#define PIN_ESC 3             // 电调
#define PIN_aileron 2         // 副翼
#define PIN_elevator 1        // 升降舵
#define PIN_rudder 0          // 方向舵

struct ControlData {
  uint16_t throttle;
  int16_t left_flap;
  int16_t right_flap;
  uint8_t checksum;
};

const byte address[6] = "FLY01";

// MPU6500 data structure
struct MPUData {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float pitch, roll;
};

MPUData mpuData;
float pitchError = 0, rollError = 0;
float pitchIntegral = 0, rollIntegral = 0;
float lastPitchError = 0, lastRollError = 0;

void initMPU6500() {
  Wire.begin();
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Wake up MPU6500
  Wire.endTransmission(true);
  
  // Configure gyroscope range (±2000°/s)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission(true);
  
  // Configure accelerometer range (±16g)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission(true);
  
  // Configure digital low pass filter
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void readMPU6500() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 14, true);
  
  // Read accelerometer data
  mpuData.accX = (Wire.read() << 8 | Wire.read()) / 2048.0;
  mpuData.accY = (Wire.read() << 8 | Wire.read()) / 2048.0;
  mpuData.accZ = (Wire.read() << 8 | Wire.read()) / 2048.0;
  
  // Read temperature (unused)
  Wire.read(); Wire.read();
  
  // Read gyroscope data
  mpuData.gyroX = (Wire.read() << 8 | Wire.read()) / 16.4;
  mpuData.gyroY = (Wire.read() << 8 | Wire.read()) / 16.4;
  mpuData.gyroZ = (Wire.read() << 8 | Wire.read()) / 16.4;
  
  // Calculate angles
  mpuData.pitch = atan2(mpuData.accY, sqrt(mpuData.accX * mpuData.accX + mpuData.accZ * mpuData.accZ)) * 180 / PI;
  mpuData.roll = atan2(-mpuData.accX, mpuData.accZ) * 180 / PI;
}

void stabilizeFlight() {
  // Calculate errors
  pitchError = -mpuData.pitch;  // Negative because we want to correct in opposite direction
  rollError = -mpuData.roll;
  
  // Calculate integral terms
  pitchIntegral += pitchError;
  rollIntegral += rollError;
  
  // Limit integral terms to prevent windup
  pitchIntegral = constrain(pitchIntegral, -100, 100);
  rollIntegral = constrain(rollIntegral, -100, 100);
  
  // Calculate PID outputs
  float pitchOutput = KP * pitchError + KI * pitchIntegral + KD * (pitchError - lastPitchError);
  float rollOutput = KP * rollError + KI * rollIntegral + KD * (rollError - lastRollError);
  
  // Update last errors
  lastPitchError = pitchError;
  lastRollError = rollError;
  
  // Apply corrections to servos
  int currentElevator = elevator.read();
  int currentAileron = aileron.read();
  
  elevator.write(constrain(currentElevator + pitchOutput, 45, 135));
  aileron.write(constrain(currentAileron + rollOutput, 45, 135));
}

void setup() {
   // 手动初始化 SPI 并指定引脚
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN); 

  Serial.begin(9600);
  printf_begin();

  pinMode(PIN_LED, OUTPUT);

  initController();
  initRF();
  digitalWrite(PIN_LED, HIGH);
  showLight();

  // 上电安全检测
  esc.writeMicroseconds(2000); // 电调解锁前满油门
  Serial.println("满油门...");
  delay(2000);

  esc.writeMicroseconds(1000); // 电调解锁前零油门
  Serial.println("零油门...");
  delay(2000);

  Serial.println("电调校准完成...");
  
  selfCheck();
  initMPU6500();
}


void initController(){
  esc.attach(PIN_ESC, 1000, 2000);   // 电调初始化
  aileron.attach(PIN_aileron);           
  elevator.attach(PIN_elevator);         
  rudder.attach(PIN_rudder);             
  aileron.write(90);
  elevator.write(90);
  rudder.write(90);
}

void showLight(){
  digitalWrite(PIN_LED, HIGH);
  delay(500);
  digitalWrite(PIN_LED, LOW);
  delay(500);
  digitalWrite(PIN_LED, HIGH);
  delay(500);
  digitalWrite(PIN_LED, LOW);
}

void initRF(){
  if (!radio.begin()){
    radio.printPrettyDetails();
    while(1);
  }  

  radio.setChannel(108); 
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true); // 默认启用
  radio.setRetries(5, 15); // 延迟=250μs*5, 重试15次
  radio.setCRCLength(RF24_CRC_16);
  //radio.printDetails();
  radio.startListening();
}

void selfCheck(){
  //回正
  aileron.write(90);
  elevator.write(90);
  rudder.write(90);
  delay(500);

  //低极值
  aileron.write(45);
  elevator.write(45);
  rudder.write(45);
  delay(500);

  //高极值
  aileron.write(135);
  elevator.write(135);
  rudder.write(135);
  delay(500);

  //回正
  aileron.write(90);
  elevator.write(90);
  rudder.write(90);

  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  if(radio.available()){
    ControlData data;
    radio.read(&data, sizeof(data));
    uint8_t sum = (data.throttle + data.left_flap + data.right_flap) % 256;
    if(sum == data.checksum) {
      digitalWrite(PIN_LED, HIGH);
      lastSignalTime = millis();
      
      // 油门通道处理
      int throttle = map(data.throttle, 0, 1023, 1000, 2000);
      esc.writeMicroseconds(throttle);
      
      // 舵机通道处理（±30度）
      aileron.write(map(data.left_flap, -512, 511, 45, 135));
      //elevator.write(map(data.roll, -512, 511, 45, 135));
      rudder.write(map(data.right_flap, -512, 511, 135, 45));
      
      // 如果油门为0，启用自稳
      if (data.throttle == 0) {
        readMPU6500();
        stabilizeFlight();
      }
    } 
  } else {  
    digitalWrite(PIN_LED, LOW);  
  }  
  
  // 失控保护（2秒无信号切断油门）
  if(millis() - lastSignalTime > 2000) {
    esc.writeMicroseconds(1000);      // 紧急停机
    aileron.write(90);                // 回中
    elevator.write(90);
    rudder.write(90);
  }
}