#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <printf.h>
#include <Wire.h>

// MPU6500 I2C地址
#define MPU6500_ADDR 0x68

// MPU6500寄存器映射
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

// 机型定义
#define AIRCRAFT_PAPER_PLANE    0  // 纸飞机
#define AIRCRAFT_CAMEL         1  // 骆驼战斗机
#define AIRCRAFT_P51           2  // P51野马战斗机

// 飞行模式定义
#define FLIGHT_MODE_MANUAL     0  // 手动模式
#define FLIGHT_MODE_STABILIZE  1  // 自稳模式
#define FLIGHT_MODE_HOLD       2  // 姿态保持模式

// 死区控制参数
#define DEADBAND_PITCH         2.0  // 俯仰死区（度）
#define DEADBAND_ROLL          2.0  // 横滚死区（度）

// 各机型PID参数
struct PIDParams {
  float kp;    // 比例增益
  float ki;    // 积分增益
  float kd;    // 微分增益
  float max_i; // 积分限幅
};

// 纸飞机PID参数
const PIDParams PAPER_PLANE_PID = {
  .kp = 2.0,
  .ki = 0.1,
  .kd = 1.0,
  .max_i = 100
};

// 骆驼战斗机PID参数
const PIDParams CAMEL_PID = {
  .kp = 1.5,
  .ki = 0.05,
  .kd = 1.2,
  .max_i = 80
};

// P51 PID参数
const PIDParams P51_PID = {
  .kp = 1.8,
  .ki = 0.08,
  .kd = 1.1,
  .max_i = 90
};

// 舵机混合控制参数
struct ServoMix {
  float pitch;  // 俯仰混合比例
  float roll;   // 横滚混合比例
};

// 纸飞机舵机混合参数
const ServoMix PAPER_PLANE_MIX = {
  .pitch = 1.0,  // 俯仰控制比例
  .roll = 0.8    // 横滚控制比例
};

// 骆驼战斗机舵机混合参数
const ServoMix CAMEL_MIX = {
  .pitch = 1.2,  // 俯仰控制比例
  .roll = 1.0    // 横滚控制比例
};

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
Servo esc, ch1, ch2, ch3;

unsigned long lastSignalTime = 0;

//管脚定义
#define PIN_LED 7             // 信号灯
#define PIN_ESC 3             // 电调
#define PIN_ch1 2             // 通道1
#define PIN_ch2 1             // 通道2
#define PIN_ch3 0             // 通道3

struct ControlData {
  uint8_t aircraft_type;      // 机型
  uint8_t flight_mode;        // 飞行模式
  uint16_t throttle;          // 油门
  int16_t ch1;               // 通道1
  int16_t ch2;               // 通道2
  int16_t ch3;               // 通道3
  uint8_t checksum;          // 校验和
};

const byte address[6] = "FLY01";

// MPU6500数据结构
struct MPUData {
  float accX, accY, accZ;    // 加速度计数据
  float gyroX, gyroY, gyroZ; // 陀螺仪数据
  float pitch, roll;         // 俯仰角和横滚角
};

MPUData mpuData;
float pitchError = 0, rollError = 0;           // 俯仰和横滚误差
float pitchIntegral = 0, rollIntegral = 0;     // 积分项
float lastPitchError = 0, lastRollError = 0;   // 上一次误差

// 姿态保持目标值
float targetPitch = 0;
float targetRoll = 0;

void initMPU6500() {
  Wire.begin();
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // 唤醒MPU6500
  Wire.endTransmission(true);
  
  // 配置陀螺仪量程（±2000°/s）
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission(true);
  
  // 配置加速度计量程（±16g）
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission(true);
  
  // 配置数字低通滤波器
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
  
  // 读取加速度计数据
  mpuData.accX = (Wire.read() << 8 | Wire.read()) / 2048.0;
  mpuData.accY = (Wire.read() << 8 | Wire.read()) / 2048.0;
  mpuData.accZ = (Wire.read() << 8 | Wire.read()) / 2048.0;
  
  // 读取温度（未使用）
  Wire.read(); Wire.read();
  
  // 读取陀螺仪数据
  mpuData.gyroX = (Wire.read() << 8 | Wire.read()) / 16.4;
  mpuData.gyroY = (Wire.read() << 8 | Wire.read()) / 16.4;
  mpuData.gyroZ = (Wire.read() << 8 | Wire.read()) / 16.4;
  
  // 计算角度
  mpuData.pitch = atan2(mpuData.accY, sqrt(mpuData.accX * mpuData.accX + mpuData.accZ * mpuData.accZ)) * 180 / PI;
  mpuData.roll = atan2(-mpuData.accX, mpuData.accZ) * 180 / PI;
}

void stabilizeFlight(ControlData data) {
  // 计算误差
  pitchError = -mpuData.pitch;  // 取负值是因为需要向相反方向修正
  rollError = -mpuData.roll;
  
  // 死区控制
  if(abs(pitchError) < DEADBAND_PITCH) pitchError = 0;
  if(abs(rollError) < DEADBAND_ROLL) rollError = 0;
  
  // 获取当前机型的PID参数
  PIDParams pid;
  ServoMix mix;
  switch(data.aircraft_type) {
    case AIRCRAFT_PAPER_PLANE:
      pid = PAPER_PLANE_PID;
      mix = PAPER_PLANE_MIX;
      break;
    case AIRCRAFT_CAMEL:
      pid = CAMEL_PID;
      mix = CAMEL_MIX;
      break;
    case AIRCRAFT_P51:
      pid = P51_PID;
      mix = {1.0, 1.0};  // P51不需要混合
      break;
  }
  
  // 计算积分项
  pitchIntegral += pitchError;
  rollIntegral += rollError;
  
  // 限制积分项以防止积分饱和
  pitchIntegral = constrain(pitchIntegral, -pid.max_i, pid.max_i);
  rollIntegral = constrain(rollIntegral, -pid.max_i, pid.max_i);
  
  // 计算PID输出
  float pitchOutput = pid.kp * pitchError + pid.ki * pitchIntegral + pid.kd * (pitchError - lastPitchError);
  float rollOutput = pid.kp * rollError + pid.ki * rollIntegral + pid.kd * (rollError - lastRollError);
  
  // 更新上一次误差
  lastPitchError = pitchError;
  lastRollError = rollError;
  
  // 将修正值应用到舵机
  int currentCh2 = ch2.read();
  int currentCh1 = ch1.read();
  
  // 声明所有可能用到的变量
  float leftWing = 0;
  float rightWing = 0;
  
  // 根据不同机型应用舵机混合控制
  switch(data.aircraft_type) {
    case AIRCRAFT_PAPER_PLANE:
      // 纸飞机：混合控制
      leftWing = currentCh1 - (rollOutput * mix.roll) + (pitchOutput * mix.pitch);
      rightWing = currentCh2 + (rollOutput * mix.roll) + (pitchOutput * mix.pitch);
      ch1.write(constrain(leftWing, 45, 135));
      ch2.write(constrain(rightWing, 45, 135));
      break;
      
    case AIRCRAFT_CAMEL:
      // 骆驼战斗机：独立控制
      ch1.write(constrain(currentCh1 + (pitchOutput * mix.pitch), 45, 135));
      ch2.write(constrain(currentCh2 + (rollOutput * mix.roll), 45, 135));
      break;
      
    case AIRCRAFT_P51:
      // P51：标准控制
      ch2.write(constrain(currentCh2 + pitchOutput, 45, 135));
      ch1.write(constrain(currentCh1 + rollOutput, 45, 135));
      break;
  }
}

void holdAttitude() {
  // 更新目标姿态
  targetPitch = mpuData.pitch;
  targetRoll = mpuData.roll;
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
  ch1.attach(PIN_ch1);           
  ch2.attach(PIN_ch2);         
  ch3.attach(PIN_ch3);             
  ch1.write(90);
  ch2.write(90);
  ch3.write(90);
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
  ch1.write(90);
  ch2.write(90);
  ch3.write(90);
  delay(500);

  //低极值
  ch1.write(45);
  ch2.write(45);
  ch3.write(45);
  delay(500);

  //高极值
  ch1.write(135);
  ch2.write(135);
  ch3.write(135);
  delay(500);

  //回正
  ch1.write(90);
  ch2.write(90);
  ch3.write(90);

  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  if(radio.available()){
    ControlData data;
    radio.read(&data, sizeof(data));
    uint8_t sum = (data.aircraft_type + data.flight_mode + data.throttle + data.ch1 + data.ch2 + data.ch3) % 256;
    if(sum == data.checksum) {
      digitalWrite(PIN_LED, HIGH);
      lastSignalTime = millis();
      
      // 油门通道处理
      int throttle = map(data.throttle, 0, 1023, 1000, 2000);
      esc.writeMicroseconds(throttle);
      
      // 根据不同机型处理舵机控制
      switch(data.aircraft_type) {
        case AIRCRAFT_PAPER_PLANE:
          // 纸飞机：只使用ch1和ch2
          ch1.write(map(data.ch1, -512, 511, 45, 135));  // 左翼
          ch2.write(map(data.ch2, -512, 511, 45, 135));  // 右翼
          ch3.write(90);  // 保持中立
          break;
          
        case AIRCRAFT_CAMEL:
          // 骆驼战斗机：只使用ch1和ch2
          ch1.write(map(data.ch1, -512, 511, 45, 135));  // 水平尾翼
          ch2.write(map(data.ch2, -512, 511, 45, 135));  // 垂直尾翼
          ch3.write(90);  // 保持中立
          break;
          
        case AIRCRAFT_P51:
          // P51使用全部三个通道
          ch1.write(map(data.ch1, -512, 511, 45, 135));
          ch2.write(map(data.ch2, -512, 511, 45, 135));
          ch3.write(map(data.ch3, -512, 511, 135, 45));
          break;
      }
      
      // 飞行模式处理
      switch(data.flight_mode) {
        case FLIGHT_MODE_STABILIZE:
          readMPU6500();
          stabilizeFlight(data);
          break;
        case FLIGHT_MODE_HOLD:
          readMPU6500();
          holdAttitude();
          stabilizeFlight(data);
          break;
      }
    } 
  } else {  
    digitalWrite(PIN_LED, LOW);  
  }  
  
  // 失控保护（2秒无信号切断油门）
  if(millis() - lastSignalTime > 2000) {
    esc.writeMicroseconds(1000);      // 紧急停机
    ch1.write(90);                // 回中
    ch2.write(90);
    ch3.write(90);
  }
}