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

// 纸飞机PID参数 - 降低PID参数以减少抖动
const PIDParams PAPER_PLANE_PID = {
  .kp = 0.8,    // 降低比例增益
  .ki = 0.02,   // 降低积分增益
  .kd = 0.5,    // 降低微分增益
  .max_i = 50   // 降低积分限幅
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
  float gyroX_offset = 0;    // 陀螺仪X轴零偏
  float gyroY_offset = 0;    // 陀螺仪Y轴零偏
  float gyroZ_offset = 0;    // 陀螺仪Z轴零偏
  float pitch, roll;         // 俯仰角和横滚角
};

MPUData mpuData;
float pitchError = 0, rollError = 0;           // 俯仰和横滚误差
float pitchIntegral = 0, rollIntegral = 0;     // 积分项
float lastPitchError = 0, lastRollError = 0;   // 上一次误差

// 姿态保持目标值
float targetPitch = 0;
float targetRoll = 0;

// 全局变量
static float lastPitch = 0.0;
static float lastRoll = 0.0;
static uint32_t lastUpdateTime = 0;

static float filteredGyroX = 0, filteredGyroY = 0;

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
  // 读取原始传感器数据
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 14, true);
  
  // 读取加速度计数据 (单位: g)
  mpuData.accX = (Wire.read() << 8 | Wire.read()) / 2048.0;
  mpuData.accY = (Wire.read() << 8 | Wire.read()) / 2048.0;
  mpuData.accZ = (Wire.read() << 8 | Wire.read()) / 2048.0;
  
  // 跳过温度数据
  Wire.read(); Wire.read();
  
  // 读取陀螺仪数据 (单位: °/s)
  mpuData.gyroX = (Wire.read() << 8 | Wire.read()) / 16.4;
  mpuData.gyroY = (Wire.read() << 8 | Wire.read()) / 16.4;
  mpuData.gyroZ = (Wire.read() << 8 | Wire.read()) / 16.4;
  
  // 应用校准偏移
  mpuData.gyroX -= mpuData.gyroX_offset;
  mpuData.gyroY -= mpuData.gyroY_offset;
  mpuData.gyroZ -= mpuData.gyroZ_offset;
  
  // 计算时间增量 (单位: 秒)
  uint32_t currentTime = micros();
  float dt = (currentTime - lastUpdateTime) / 1000000.0;
  lastUpdateTime = currentTime;
  
  // 设置最小时间阈值防止除零
  if (dt <= 0) dt = 0.001;
  
  // === 1. 加速度计姿态计算 ===
  // 俯仰角 (绕Y轴旋转) - 使用改进公式避免万向锁问题
  float pitchAcc = atan2(-mpuData.accX, 
                      copysignf(sqrtf(mpuData.accY*mpuData.accY + 
                                  mpuData.accZ*mpuData.accZ), 
                              mpuData.accZ)) * 180 / PI;
  
  // 横滚角 (绕X轴旋转) - 注意Y轴方向处理
  // 修正：添加负号以符合航空标准（右滚为正）
  float rollAcc = atan2(-mpuData.accY,  // 关键修正：添加负号
                     copysignf(sqrtf(mpuData.accX*mpuData.accX + 
                                 mpuData.accZ*mpuData.accZ), 
                             mpuData.accZ)) * 180 / PI;
  
  
  // === 2. 陀螺仪数据处理 ===
  // 根据实际测试结果修正方向：
  // - 抬头运动：+gyroY → 俯仰角速度应为正
  // - 右滚运动：-gyroX → 横滚角速度应为正
  const float gyroFilterFactor = 0.2; // 滤波系数
  filteredGyroX = gyroFilterFactor * mpuData.gyroX + (1 - gyroFilterFactor) * filteredGyroX;
  filteredGyroY = gyroFilterFactor * mpuData.gyroY + (1 - gyroFilterFactor) * filteredGyroY;

  // 使用滤波后的数据
  float gyroPitchRate = -filteredGyroY;
  float gyroRollRate = filteredGyroX;
  
  // 陀螺仪积分计算
  float pitchGyro = lastPitch + gyroPitchRate * dt;
  float rollGyro = lastRoll + gyroRollRate * dt;
  
  // === 3. 互补滤波融合 ===
  const float alpha = 0.96;  // 陀螺仪权重
  
  mpuData.pitch = alpha * pitchGyro + (1 - alpha) * pitchAcc;
  mpuData.roll = alpha * rollGyro + (1 - alpha) * rollAcc;

  if (isnan(mpuData.pitch) || abs(mpuData.pitch) > 180.0f) {
    mpuData.pitch = lastPitch; // 使用上次有效值
  }
  if (isnan(mpuData.roll) || abs(mpuData.roll) > 180.0f) {
    mpuData.roll = lastRoll;
  } 
  // 在互补滤波后添加角度约束
  mpuData.pitch = constrain(mpuData.pitch, -89.9, 89.9);
  mpuData.roll = constrain(mpuData.roll, -89.9, 89.9);

  // 保存当前角度用于下一次计算
  lastPitch = mpuData.pitch;
  lastRoll = mpuData.roll;
  
  // 调试输出
  Serial.print("Pitch: ");
  Serial.print(mpuData.pitch);
  Serial.print("°, Roll: ");
  Serial.print(mpuData.roll);
  Serial.print("° | Gyro: PitchRate=");
  Serial.print(gyroPitchRate);
  Serial.print("°/s, RollRate=");
  Serial.print(gyroRollRate);
  Serial.println("°/s");
}

void stabilizeFlight(ControlData data) {
  return;
  // 计算误差
  pitchError = -mpuData.pitch;  // 取负值是因为需要向相反方向修正
  rollError = -mpuData.roll;
  
  // 调试模式：直接映射传感器数据到舵机
  if(data.aircraft_type == AIRCRAFT_PAPER_PLANE) {
    // 将pitch和roll数据映射到舵机角度范围
    // pitch范围：-90到90度，映射到45-135度
    // roll范围：-90到90度，映射到45-135度
    
    // 计算舵机位置
    // 左舵机：pitch控制，向上为正
    int leftServo = 90 + (mpuData.pitch * 0.5);  // 0.5是缩放因子，限制舵机运动范围
    leftServo = constrain(leftServo, 45, 135);   // 限制在有效范围内
    
    // 右舵机：roll控制，向右为正
    int rightServo = 90 + (mpuData.roll * 0.5);  // 0.5是缩放因子，限制舵机运动范围
    rightServo = constrain(rightServo, 45, 135); // 限制在有效范围内
    
    // 应用舵机位置
    ch1.write(leftServo);
    ch2.write(rightServo);
    
    return;  // 调试模式下不执行PID控制
  }
  
  // 以下是正常的PID控制逻辑
  // 死区控制
  if(abs(pitchError) < DEADBAND_PITCH * 2) pitchError = 0;
  if(abs(rollError) < DEADBAND_ROLL * 2) rollError = 0;
  
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
  
  // 计算积分项 - 添加积分限幅
  pitchIntegral += pitchError;
  rollIntegral += rollError;
  
  // 限制积分项以防止积分饱和
  pitchIntegral = constrain(pitchIntegral, -pid.max_i, pid.max_i);
  rollIntegral = constrain(rollIntegral, -pid.max_i, pid.max_i);
  
  // 计算PID输出 - 添加输出限幅
  float pitchOutput = pid.kp * pitchError + pid.ki * pitchIntegral + pid.kd * (pitchError - lastPitchError);
  float rollOutput = pid.kp * rollError + pid.ki * rollIntegral + pid.kd * (rollError - lastRollError);
  
  // 限制PID输出范围
  pitchOutput = constrain(pitchOutput, -20.0, 20.0);
  rollOutput = constrain(rollOutput, -20.0, 20.0);
  
  // 更新上一次误差
  lastPitchError = pitchError;
  lastRollError = rollError;
  
  // 获取当前舵机位置
  int currentCh1 = ch1.read();
  int currentCh2 = ch2.read();
  
  // 根据不同机型应用舵机控制
  switch(data.aircraft_type) {
    case AIRCRAFT_PAPER_PLANE: {
      // 纸飞机：对称控制俯仰，差动控制横滚
      // 计算目标舵机位置
      float leftTarget = 90.0 + (pitchOutput * mix.pitch) - (rollOutput * mix.roll);
      float rightTarget = 90.0 + (pitchOutput * mix.pitch) + (rollOutput * mix.roll);
      
      // 限制输出范围
      leftTarget = constrain(leftTarget, 45.0, 135.0);
      rightTarget = constrain(rightTarget, 45.0, 135.0);
      
      // 平滑过渡到目标位置 - 减小步进比例
      float leftStep = (leftTarget - currentCh1) * 0.15;  // 降低到15%的步进
      float rightStep = (rightTarget - currentCh2) * 0.15;
      
      // 应用新的舵机位置
      ch1.write(currentCh1 + leftStep);
      ch2.write(currentCh2 + rightStep);
      break;
    }
      
    case AIRCRAFT_CAMEL: {
      // 骆驼战斗机：独立控制
      ch1.write(constrain(currentCh1 + (pitchOutput * mix.pitch), 45, 135));
      ch2.write(constrain(currentCh2 + (rollOutput * mix.roll), 45, 135));
      break;
    }
      
    case AIRCRAFT_P51: {
      // P51：标准控制
      ch2.write(constrain(currentCh2 + pitchOutput, 45, 135));
      ch1.write(constrain(currentCh1 + rollOutput, 45, 135));
      break;
    }
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

  // 安全解锁流程
  Serial.println(F("等待解锁确认..."));
  Serial.println(F("请将油门摇杆推到最低位置"));
  
  // 等待接收机收到信号并确认油门在最低位置
  bool throttleConfirmed = false;
  unsigned long startTime = millis();
  
  while (!throttleConfirmed && (millis() - startTime < 60000)) { // 10秒超时
    if(radio.available()) {
      ControlData data;
      radio.read(&data, sizeof(data));
      uint8_t sum = (data.aircraft_type + data.flight_mode + data.throttle + data.ch1 + data.ch2 + data.ch3) % 256;
      if(sum == data.checksum) {
        // 检查油门是否在最低位置（小于200）
        if(data.throttle < 200) {
          throttleConfirmed = true;
          Serial.println(F("油门位置确认，开始解锁流程"));
          
          // 执行解锁序列
          esc.writeMicroseconds(1000); // 确保零油门
          delay(2000);
          
          // 快速闪烁LED表示正在解锁
          for(int i = 0; i < 5; i++) {
            digitalWrite(PIN_LED, HIGH);
            delay(100);
            digitalWrite(PIN_LED, LOW);
            delay(100);
          }
          
          Serial.println(F("电调解锁完成"));
        } else {
          Serial.println(F("请将油门摇杆推到最低位置"));
        }
      }
    }
    delay(100);
  }
  
  if (!throttleConfirmed) {
    Serial.println(F("解锁超时，请检查遥控器连接"));
    // 进入安全模式
    esc.writeMicroseconds(1000);
    while(1) {
      digitalWrite(PIN_LED, HIGH);
      delay(100);
      digitalWrite(PIN_LED, LOW);
      delay(100);
    }
  }

  Serial.println(F("开始自检"));
  selfCheck();
  
  Serial.println(F("开始初始化MPU6500"));
  initMPU6500();
  
  calibrateGyro();
  
  lastUpdateTime = micros();

  // 初始化角度为0
  lastPitch = 0;
  lastRoll = 0;
  
  // 读取一次传感器数据
  readMPU6500();
}

// 改进的陀螺仪校准
void calibrateGyro() {
  float gx = 0, gy = 0, gz = 0;
  float ax = 0, ay = 0, az = 0;
  const int samples = 500;
  
  Serial.println(F("开始陀螺仪校准..."));
  Serial.println(F("请保持设备完全静止"));
  
  for(int i=0; i<samples; i++) {
    // 读取完整传感器数据
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6500_ADDR, 14, true);
    
    // 读取加速度计数据
    ax += (Wire.read() << 8 | Wire.read()) / 2048.0;
    ay += (Wire.read() << 8 | Wire.read()) / 2048.0;
    az += (Wire.read() << 8 | Wire.read()) / 2048.0;
    
    // 跳过温度数据
    Wire.read(); Wire.read();
    
    // 读取陀螺仪数据
    int16_t gx_raw = Wire.read() << 8 | Wire.read();
    int16_t gy_raw = Wire.read() << 8 | Wire.read();
    int16_t gz_raw = Wire.read() << 8 | Wire.read();
    
    gx += gx_raw / 16.4;
    gy += gy_raw / 16.4;
    gz += gz_raw / 16.4;
    
    // 进度指示
    if (i % 50 == 0) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
    delay(10);
  }
  
  // 验证加速度计数据（应接近重力加速度）
  float accMagnitude = sqrt(ax*ax + ay*ay + az*az) / samples;
  if (abs(accMagnitude - 1.0) > 0.2) {
    Serial.println(F("警告：校准过程中设备移动！"));
  }
  
  mpuData.gyroX_offset = gx/samples;
  mpuData.gyroY_offset = gy/samples;
  mpuData.gyroZ_offset = gz/samples;
  
  Serial.print("陀螺仪校准完成: ");
  Serial.print(mpuData.gyroX_offset, 4); 
  Serial.print(", ");
  Serial.print(mpuData.gyroY_offset, 4);
  Serial.print(", ");
  Serial.println(mpuData.gyroZ_offset, 4);
  digitalWrite(PIN_LED, LOW);
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
          ch1.write(map(data.ch1, -512, 511, 45, 135));  // 左翼，舵机安装方向，所以方向映射
          ch2.write(map(data.ch2, -512, 511, 135, 45));  // 右翼
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