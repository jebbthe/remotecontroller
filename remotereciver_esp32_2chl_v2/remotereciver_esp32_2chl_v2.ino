#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <printf.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

// MPU6500 I2C地址
#define MPU6500_ADDR 0x68
#define ENABLE_SELFCTL 0
//#define TEST_MPU6500 1

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
#define DEADBAND_PITCH         4.0  // 俯仰死区（度）- 增加死区范围
#define DEADBAND_ROLL          4.0  // 横滚死区（度）- 增加死区范围

// 各机型PID参数
struct PIDParams {
  float kp;    // 比例增益
  float ki;    // 积分增益
  float kd;    // 微分增益
  float max_i; // 积分限幅
};

// 纸飞机PID参数 - 进一步降低参数值以减少抖动
const PIDParams PAPER_PLANE_PID = {
  .kp = 1.0,    // 进一步降低比例增益
  .ki = 0.01,   // 进一步降低积分增益
  .kd = 0.3,    // 大幅降低微分增益以减少噪声放大
  .max_i = 20   // 进一步降低积分限幅
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

// 纸飞机舵机混合参数 - 调整混合比例以适应面对安装的舵机
const ServoMix PAPER_PLANE_MIX = {
  .pitch = -1.5,  // 保持俯仰控制比例
  .roll = 1.0     // 降低横滚控制比例，从1.8改为1.0
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
#define CE_PIN   18
#define CSN_PIN  10

// I2C 引脚定义（用于MPU6500）
#define I2C_SDA_PIN 19
#define I2C_SCL_PIN 8

//无线模块
RF24 radio(CE_PIN, CSN_PIN); 
//舵机&电调
Servo esc, ch1, ch2, ch3;

unsigned long lastSignalTime = 0;

// 重启诊断相关
#define RESTART_INFO_NAMESPACE "restart_info"
#define RESTART_INFO_KEY "last_crash"

// 重启信息结构体
struct RestartInfo {
  uint8_t resetReason;           // 重启原因
  uint32_t resetTime;            // 重启时间戳（从启动开始计算的毫秒数）
  uint32_t freeHeap;             // 可用堆内存
  uint32_t minFreeHeap;          // 最小可用堆内存
  uint32_t maxAllocHeap;         // 最大可分配堆内存
  uint32_t uptimeBeforeReset;    // 重启前运行时间（秒）
  uint8_t taskCount;             // 任务数量
  char resetReasonStr[32];       // 重启原因字符串
  uint32_t magic;                // 魔数，用于验证数据有效性
};

#define RESTART_INFO_MAGIC 0xDEADBEEF

Preferences preferences;

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
  float accX_offset = 0;     // 加速度计X轴零偏
  float accY_offset = 0;     // 加速度计Y轴零偏
  float accZ_offset = 0;     // 加速度计Z轴零偏
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

// 添加全局变量用于存储上一次有效的加速度计数据
float lastAccX = 0, lastAccY = 0, lastAccZ = 0;

// 获取重启原因字符串
const char* getResetReasonString(esp_reset_reason_t reason) {
  switch(reason) {
    case ESP_RST_UNKNOWN:   return "未知原因";
    case ESP_RST_POWERON:   return "上电复位(正常断电重启)";
    case ESP_RST_EXT:       return "外部复位";
    case ESP_RST_SW:        return "软件复位";
    case ESP_RST_PANIC:     return "异常/Panic";
    case ESP_RST_INT_WDT:   return "中断看门狗复位";
    case ESP_RST_TASK_WDT:  return "任务看门狗复位";
    case ESP_RST_WDT:       return "其他看门狗复位";
    case ESP_RST_DEEPSLEEP: return "深度睡眠唤醒";
    case ESP_RST_BROWNOUT:  return "欠压复位";
    case ESP_RST_SDIO:      return "SDIO复位";
    default:                return "未定义";
  }
}

// 检查是否为正常重启
bool isNormalRestart(esp_reset_reason_t reason) {
  // 只有上电复位被认为是正常重启
  return (reason == ESP_RST_POWERON);
}

// 保存重启信息到Flash
void saveRestartInfo(esp_reset_reason_t reason, uint32_t uptime) {
  RestartInfo info;
  info.resetReason = (uint8_t)reason;
  info.resetTime = millis();
  // 记录重启后的堆内存状态（虽然这不是重启前的状态，但仍有助于诊断）
  info.freeHeap = ESP.getFreeHeap();
  info.minFreeHeap = ESP.getMinFreeHeap();
  info.maxAllocHeap = ESP.getMaxAllocHeap();
  info.uptimeBeforeReset = uptime;
  info.taskCount = 0; // ESP32 Arduino框架不直接提供任务计数
  strncpy(info.resetReasonStr, getResetReasonString(reason), sizeof(info.resetReasonStr) - 1);
  info.resetReasonStr[sizeof(info.resetReasonStr) - 1] = '\0';
  info.magic = RESTART_INFO_MAGIC;
  
  preferences.begin(RESTART_INFO_NAMESPACE, false);
  bool success = preferences.putBytes(RESTART_INFO_KEY, &info, sizeof(info)) == sizeof(info);
  preferences.end();
  
  if (success) {
    Serial.print(F("重启信息已成功保存到Flash: "));
    Serial.println(info.resetReasonStr);
  } else {
    Serial.println(F("警告: 保存重启信息到Flash失败！"));
  }
}

// 从Flash读取重启信息
bool readRestartInfo(RestartInfo* info) {
  preferences.begin(RESTART_INFO_NAMESPACE, true);
  size_t len = preferences.getBytes(RESTART_INFO_KEY, info, sizeof(RestartInfo));
  preferences.end();
  
  if (len == sizeof(RestartInfo) && info->magic == RESTART_INFO_MAGIC) {
    return true;
  }
  return false;
}

// 清除Flash中的重启信息
void clearRestartInfo() {
  preferences.begin(RESTART_INFO_NAMESPACE, false);
  preferences.remove(RESTART_INFO_KEY);
  preferences.end();
}

// 打印重启信息到串口
void printRestartInfo(const RestartInfo* info) {
  Serial.println(F("\n========== Flash中异常重启信息 =========="));
  Serial.print(F("重启原因: "));
  Serial.println(info->resetReasonStr);
  Serial.print(F("重启原因代码: 0x"));
  Serial.println(info->resetReason, HEX);
  Serial.print(F("重启时间戳: "));
  Serial.print(info->resetTime);
  Serial.println(F(" ms"));
  Serial.print(F("重启前运行时间: "));
  Serial.print(info->uptimeBeforeReset);
  Serial.println(F(" 秒"));
  Serial.print(F("可用堆内存: "));
  Serial.print(info->freeHeap);
  Serial.println(F(" 字节"));
  Serial.print(F("最小可用堆内存: "));
  Serial.print(info->minFreeHeap);
  Serial.println(F(" 字节"));
  Serial.print(F("最大可分配堆内存: "));
  Serial.print(info->maxAllocHeap);
  Serial.println(F(" 字节"));
  Serial.println(F("=====================================\n"));
}

// 初始化看门狗定时器
void initWatchdog() {
  esp_err_t err;
  
  // ESP32 Arduino框架可能已经自动初始化了看门狗
  // 先尝试直接添加当前任务到看门狗
  err = esp_task_wdt_add(NULL);
  if (err == ESP_OK) {
    // 成功添加，说明看门狗已经初始化
    Serial.println(F("看门狗已存在，已添加当前任务到看门狗"));
    return;
  }
  
  // 如果添加失败（可能是看门狗未初始化），尝试初始化
  // 配置看门狗超时时间为30秒（增加超时时间以适应setup()中的长时间操作）
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,       // 30秒超时（足够setup()完成）
    .idle_core_mask = 0,       // 不监控空闲任务
    .trigger_panic = true      // panic模式，超时后重启
  };
  
  err = esp_task_wdt_init(&wdt_config);
  if (err == ESP_OK) {
    // 初始化成功，添加任务
    err = esp_task_wdt_add(NULL);
    if (err == ESP_OK) {
      Serial.println(F("看门狗定时器已启动（30秒超时）"));
    } else {
      Serial.println(F("警告: 添加任务到看门狗失败"));
    }
  } else if (err == ESP_ERR_INVALID_STATE) {
    // 看门狗已经初始化（可能在其他地方），再次尝试添加任务
    err = esp_task_wdt_add(NULL);
    if (err == ESP_OK) {
      Serial.println(F("看门狗已存在，已添加当前任务到看门狗"));
    } else {
      Serial.print(F("警告: 添加任务到看门狗失败，错误代码: 0x"));
      Serial.println(err, HEX);
    }
  } else {
    Serial.print(F("警告: 初始化看门狗失败，错误代码: 0x"));
    Serial.println(err, HEX);
  }
}

// 喂狗（在loop中定期调用）
void feedWatchdog() {
  esp_task_wdt_reset();
}

void initRestartDiagnostics() {
  esp_reset_reason_t lastReason = esp_reset_reason();
  const char* reasonStr = getResetReasonString(lastReason);
  
  Serial.println(F("\n========== 系统启动诊断 =========="));
  Serial.print(F("上次重启原因: "));
  Serial.println(reasonStr);
  Serial.print(F("重启原因代码: 0x"));
  Serial.println(lastReason, HEX);
  Serial.print(F("当前可用堆内存: "));
  Serial.print(ESP.getFreeHeap());
  Serial.println(F(" 字节"));
  Serial.print(F("当前最小可用堆内存: "));
  Serial.print(ESP.getMinFreeHeap());c:\Users\jebbt\Documents\gitroot\remotecontroller\fpv_video\VTXFirmware\VTXFirmware.ino
  Serial.println(F(" 字节"));
  Serial.println(F("==================================\n"));
  
  RestartInfo savedInfo;
  if (readRestartInfo(&savedInfo)) {
    Serial.println(F("发现之前保存的异常重启记录:"));
    printRestartInfo(&savedInfo);
  } else {
    Serial.println(F("未发现之前保存的异常重启记录\n"));
  }
  
  if (!isNormalRestart(lastReason)) {
    // 获取上次保存的运行时间（如果存在）
    uint32_t lastUptime = 0;
    preferences.begin(RESTART_INFO_NAMESPACE, true);
    if (preferences.isKey("uptime")) {
      lastUptime = preferences.getUInt("uptime", 0);
    }
    preferences.end();
    // 保存异常重启信息到Flash
    Serial.println(F("检测到异常重启，正在保存重启信息..."));
    clearRestartInfo();
    saveRestartInfo(lastReason, lastUptime);
  } else {
    Serial.println(F("正常上电重启，无需保存重启信息\n"));
  }
  
  preferences.begin(RESTART_INFO_NAMESPACE, false);
  preferences.putUInt("uptime", 0);
  preferences.end();
}

// 更新运行时间（在loop中定期调用）
void updateUptime() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  
  // 每10秒更新一次运行时间
  if (now - lastUpdate >= 10000) {
    lastUpdate = now;
    uint32_t uptimeSeconds = now / 1000;
    preferences.begin(RESTART_INFO_NAMESPACE, false);
    preferences.putUInt("uptime", uptimeSeconds);
    preferences.end();
  }
}

void initMPU6500() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);        // 唤醒MPU6500
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
  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();
  
  // 跳过温度数据
  Wire.read(); Wire.read();
  
  // 读取陀螺仪数据 (单位: °/s)
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
  
  // Serial.print("rawAccX: ");
  // Serial.print(rawAccX);
  // Serial.print("      rawAccY: ");
  // Serial.print(rawAccY);
  // Serial.print("      rawGyroX: ");
  // Serial.print(rawGyroX);
  // Serial.print("      rawGyroY: ");
  // Serial.print(rawGyroY);
  // Serial.print("      rawGyroZ: ");
  // Serial.println(rawGyroZ);


  // 检查陀螺仪数据是否在合理范围内
  const float maxGyroRate = 2000.0; // 最大角速度限制（°/s）
  const float maxGyroRaw = maxGyroRate * 16.4; // 对应的原始数据值
  
  if (abs(rawGyroX) > maxGyroRaw || abs(rawGyroY) > maxGyroRaw || abs(rawGyroZ) > maxGyroRaw) {
    // 数据异常，使用上一次的有效值
    mpuData.gyroX = filteredGyroX;
    mpuData.gyroY = filteredGyroY;
    mpuData.gyroZ = 0; // Z轴不使用
  } else {
    // 转换陀螺仪数据并应用校准偏移
    mpuData.gyroX = (rawGyroX / 16.4) - mpuData.gyroX_offset;
    mpuData.gyroY = (rawGyroY / 16.4) - mpuData.gyroY_offset;
    mpuData.gyroZ = (rawGyroZ / 16.4) - mpuData.gyroZ_offset;
  }
  
  // 计算时间增量 (单位: 秒)
  uint32_t currentTime = micros();
  float dt = (currentTime - lastUpdateTime) / 1000000.0;
  lastUpdateTime = currentTime;
  
  // 设置最小时间阈值防止除零
  if (dt <= 0) dt = 0.001;
  
  // === 1. 加速度计数据处理 ===
  // 检查加速度计数据是否在合理范围内
  if (abs(rawAccX) > 32768 || abs(rawAccY) > 32768 || abs(rawAccZ) > 32768) {
    // 数据异常，使用上一次的有效值
    mpuData.accX = lastAccX;
    mpuData.accY = lastAccY;
    mpuData.accZ = lastAccZ;
  } else {
    // 转换加速度计数据为g单位并应用零偏校准
    mpuData.accX = (rawAccX / 2048.0) - mpuData.accX_offset;
    mpuData.accY = (rawAccY / 2048.0) - mpuData.accY_offset;
    mpuData.accZ = (rawAccZ / 2048.0) - mpuData.accZ_offset;
    
    // 保存当前值用于下次比较
    lastAccX = mpuData.accX;
    lastAccY = mpuData.accY;
    lastAccZ = mpuData.accZ;
  }
  
  // 根据安装方向调整加速度计数据
  // X轴朝机头，Y轴朝左翼，Z轴朝上
  float accX = mpuData.accX;  // 机头方向
  float accY = mpuData.accY;  // 左翼方向
  float accZ = mpuData.accZ;  // 垂直向上
  
  // 计算俯仰角 (绕Y轴旋转)
  // 机头向上为正(0~90°)，向下为负(0~-90°)
  float pitchAcc = atan2(accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
  
  // 计算横滚角 (绕X轴旋转)
  // 右倾为正(0~90°)，左倾为负(0~-90°)
  float rollAcc = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
  
  // === 2. 陀螺仪数据处理 ===
  // 根据安装方向调整陀螺仪数据
  // 注意：陀螺仪数据方向需要与角度定义一致
  const float gyroFilterFactor = 0.3; // 降低滤波系数，减少噪声
  
  // 陀螺仪数据滤波
  filteredGyroX = gyroFilterFactor * mpuData.gyroX + (1 - gyroFilterFactor) * filteredGyroX;
  filteredGyroY = gyroFilterFactor * mpuData.gyroY + (1 - gyroFilterFactor) * filteredGyroY;
  
  // 陀螺仪角速度积分
  // 注意：陀螺仪数据方向需要与角度定义一致
  float gyroPitchRate = filteredGyroY;  // Y轴角速度对应俯仰
  float gyroRollRate = filteredGyroX;   // X轴角速度对应横滚
  
  // 限制角速度范围
  gyroPitchRate = constrain(gyroPitchRate, -500.0, 500.0);
  gyroRollRate = constrain(gyroRollRate, -500.0, 500.0);
  
  // 陀螺仪积分计算
  float pitchGyro = lastPitch + gyroPitchRate * dt;
  float rollGyro = lastRoll + gyroRollRate * dt;
  
  // === 3. 改进的互补滤波融合 ===
  // 计算加速度计数据质量
  float accMagnitude = sqrt(accX * accX + accY * accY + accZ * accZ);
  float accQuality = 1.0 - abs(accMagnitude - 1.0); // 1.0表示质量最好，0.0表示质量最差
  
  // 动态调整互补滤波系数
  float alpha = 0.98; // 基础陀螺仪权重
  
  // 当加速度计数据质量好时，增加其权重
  if (accQuality > 0.8) {
    alpha = 0.85; // 降低陀螺仪权重
  }
  
  // 当设备静止时，进一步增加加速度计权重
  if (abs(gyroPitchRate) < 0.5 && abs(gyroRollRate) < 0.5) {
    alpha = 0.7; // 显著降低陀螺仪权重
  }
  
  // 应用互补滤波
  mpuData.pitch = alpha * pitchGyro + (1 - alpha) * pitchAcc;
  mpuData.roll = alpha * rollGyro + (1 - alpha) * rollAcc;
  
  // 角度限幅和异常值处理
  if (isnan(mpuData.pitch) || abs(mpuData.pitch) > 90.0f) {
    mpuData.pitch = lastPitch;
  }
  if (isnan(mpuData.roll) || abs(mpuData.roll) > 90.0f) {
    mpuData.roll = lastRoll;
  }
  
  // 在互补滤波后添加角度约束
  mpuData.pitch = constrain(mpuData.pitch, -90.0, 90.0);
  mpuData.roll = constrain(mpuData.roll, -90.0, 90.0);
  
  // 保存当前角度用于下一次计算
  lastPitch = mpuData.pitch;
  lastRoll = mpuData.roll;
  // Serial.print("Pitch: ");
  // Serial.print(lastPitch);
  // Serial.print(",      Roll: ");
  // Serial.println(lastRoll);
}

void stabilizeFlight(ControlData data) {
  // 计算时间间隔
  static uint32_t lastPIDTime = 0;
  uint32_t currentTime = micros();
  float dt = (currentTime - lastPIDTime) / 1000000.0;
  lastPIDTime = currentTime;
  
  // 设置最小时间阈值防止除零
  if (dt <= 0) dt = 0.01; // 使用10ms作为默认值
  
  // 计算误差
  pitchError = -mpuData.pitch;  // 取负值是因为需要向相反方向修正
  rollError = -mpuData.roll;
  
  // 死区控制 - 增加死区范围以减少小角度抖动
  bool pitchInDeadband = abs(pitchError) < DEADBAND_PITCH * 0.5;
  bool rollInDeadband = abs(rollError) < DEADBAND_ROLL * 0.5;
  
  if(pitchInDeadband) pitchError = 0;
  if(rollInDeadband) rollError = 0;
  
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
  pitchIntegral += pitchError * dt;
  rollIntegral += rollError * dt;
  
  // 当误差进入死区时，逐渐衰减积分项
  if (pitchInDeadband) {
    pitchIntegral *= 0.95; // 每次衰减5%
  }
  if (rollInDeadband) {
    rollIntegral *= 0.95; // 每次衰减5%
  }
  
  // 限制积分项以防止积分饱和
  pitchIntegral = constrain(pitchIntegral, -pid.max_i, pid.max_i);
  rollIntegral = constrain(rollIntegral, -pid.max_i, pid.max_i);
  
  // 计算微分项 - 考虑时间间隔
  float pitchDerivative = (pitchError - lastPitchError) / dt;
  float rollDerivative = (rollError - lastRollError) / dt;
  
  // 微分项滤波 - 减少噪声放大
  static float filteredPitchDerivative = 0;
  static float filteredRollDerivative = 0;
  const float derivativeFilterFactor = 0.3; // 微分项滤波系数
  
  filteredPitchDerivative = derivativeFilterFactor * pitchDerivative + (1 - derivativeFilterFactor) * filteredPitchDerivative;
  filteredRollDerivative = derivativeFilterFactor * rollDerivative + (1 - derivativeFilterFactor) * filteredRollDerivative;
  
  // 计算PID输出
  float pitchOutput = pid.kp * pitchError + pid.ki * pitchIntegral + pid.kd * filteredPitchDerivative;
  float rollOutput = pid.kp * rollError + pid.ki * rollIntegral + pid.kd * filteredRollDerivative;
  
  // 限制PID输出范围 - 使用固定限制而不是动态限制
  float pitchLimit = 60.0;  // 固定俯仰输出限制
  float rollLimit = 50.0;   // 固定横滚输出限制
  
  pitchOutput = constrain(pitchOutput, -pitchLimit, pitchLimit);
  rollOutput = constrain(rollOutput, -rollLimit, rollLimit);
  
  // 输出平滑 - 减少舵面抖动
  static float smoothedPitchOutput = 0;
  static float smoothedRollOutput = 0;
  const float outputSmoothFactor = 0.4; // 输出平滑系数
  
  smoothedPitchOutput = outputSmoothFactor * pitchOutput + (1 - outputSmoothFactor) * smoothedPitchOutput;
  smoothedRollOutput = outputSmoothFactor * rollOutput + (1 - outputSmoothFactor) * smoothedRollOutput;
  
  // 最小输出阈值 - 避免微小输出导致的抖动
  const float minOutputThreshold = 2.0; // 最小输出阈值
  
  if (abs(smoothedPitchOutput) < minOutputThreshold) {
    smoothedPitchOutput = 0;
  }
  if (abs(smoothedRollOutput) < minOutputThreshold) {
    smoothedRollOutput = 0;
  }
  
  // 更新上一次误差
  lastPitchError = pitchError;
  lastRollError = rollError;
  
  // 获取当前舵机位置
  int currentCh1 = ch1.read();  // 右舵
  int currentCh2 = ch2.read();  // 左舵
  
  // 根据不同机型应用舵机控制
  switch(data.aircraft_type) {
    case AIRCRAFT_PAPER_PLANE: {
      // 纸飞机：对称控制俯仰，差动控制横滚
      // 计算目标舵机位置 - 考虑舵机面对安装和机头向上的情况
      float rightTarget = 90.0 + (smoothedPitchOutput * mix.pitch) - (smoothedRollOutput * mix.roll);   // 右舵机 (ch1)，横滚方向相反
      float leftTarget = 90.0 - (smoothedPitchOutput * mix.pitch) - (smoothedRollOutput * mix.roll);    // 左舵机 (ch2)
      
      // 限制输出范围
      rightTarget = constrain(rightTarget, 45.0, 135.0);
      leftTarget = constrain(leftTarget, 45.0, 135.0);
      
      // 直接设置舵机位置
      ch1.write(rightTarget);  // 右舵机
      ch2.write(leftTarget);   // 左舵机
      
      // 调试输出 - 添加时间戳
      Serial.print("[");
      Serial.print(millis());
      Serial.print("ms] Pitch Error: ");
      Serial.print(pitchError);
      Serial.print(" Roll Error: ");
      Serial.print(rollError);
      Serial.print(" Current Pitch: ");
      Serial.print(mpuData.pitch);
      Serial.print(" Current Roll: ");
      Serial.print(mpuData.roll);
      Serial.print(" Right Target: ");
      Serial.print(rightTarget);
      Serial.print(" Left Target: ");
      Serial.println(leftTarget);
      break;
    }
      
    case AIRCRAFT_CAMEL: {
      // 骆驼战斗机：独立控制
      ch1.write(constrain(currentCh1 + (smoothedPitchOutput * mix.pitch), 45, 135));
      ch2.write(constrain(currentCh2 + (smoothedRollOutput * mix.roll), 45, 135));
      break;
    }
      
    case AIRCRAFT_P51: {
      // P51：标准控制
      ch2.write(constrain(currentCh2 + smoothedPitchOutput, 45, 135));
      ch1.write(constrain(currentCh1 + smoothedRollOutput, 45, 135));
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
  Serial.println(F("Starting..."));
  delay(100); // 等待串口稳定
  printf_begin();
  
  // 首先初始化重启诊断系统（在Serial可用后立即执行）
  initRestartDiagnostics();
  
  // 初始化看门狗定时器
  initWatchdog();
  
  pinMode(PIN_LED, OUTPUT);
  
  //提示主电源接通，芯片开始工作
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  feedWatchdog();  // 喂狗
  digitalWrite(PIN_LED, LOW);

  initController();

  initRF();

  //提示核心部件启动成功
  showLight();
  feedWatchdog();  // 喂狗

#ifndef TEST_MPU6500
  // 安全解锁流程
  Serial.println(F("等待解锁确认..."));
  Serial.println(F("请将油门摇杆推到最低位置"));
  
  // 等待接收机收到信号并确认油门在最低位置
  bool throttleConfirmed = false;
  unsigned long startTime = millis();
  
  while (!throttleConfirmed && (millis() - startTime < 60000)) { // 60秒超时
    feedWatchdog();  // 在循环中定期喂狗
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
          feedWatchdog();  // 喂狗
          
          // 快速闪烁LED表示正在解锁
          for(int i = 0; i < 5; i++) {
            digitalWrite(PIN_LED, HIGH);
            delay(100);
            feedWatchdog();  // 喂狗
            digitalWrite(PIN_LED, LOW);
            delay(100);
            feedWatchdog();  // 喂狗
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
      feedWatchdog();  // 喂狗
      digitalWrite(PIN_LED, LOW);
      delay(100);
      feedWatchdog();  // 喂狗
    }
  }
#endif

  Serial.println(F("开始自检"));
  selfCheck();
  feedWatchdog();  // 喂狗

#ifdef ENABLE_SELFCTL  
  Serial.println(F("开始初始化MPU6500"));
  initMPU6500();
  feedWatchdog();  // 喂狗
  calibrateGyro();
  feedWatchdog();  // 喂狗
  lastUpdateTime = micros();
  // 初始化角度为0
  lastPitch = 0;
  lastRoll = 0;
  
  // 读取一次传感器数据
  readMPU6500();
  feedWatchdog();  // 喂狗
#endif
  
  Serial.println(F("系统初始化完成"));
  feedWatchdog();  // 喂狗
}

// 改进的陀螺仪校准
void calibrateGyro() {
  float gx = 0, gy = 0, gz = 0;
  float ax = 0, ay = 0, az = 0;
  const int samples = 500;
  
  Serial.println(F("开始传感器校准..."));
  Serial.println(F("请保持设备完全静止"));
  
  for(int i=0; i<samples; i++) {
    // 读取完整传感器数据
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6500_ADDR, 14, true);
    
    // 读取加速度计数据
    int16_t ax_raw = Wire.read() << 8 | Wire.read();
    int16_t ay_raw = Wire.read() << 8 | Wire.read();
    int16_t az_raw = Wire.read() << 8 | Wire.read();
    
    ax += ax_raw / 2048.0;
    ay += ay_raw / 2048.0;
    az += az_raw / 2048.0;
    
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
      feedWatchdog();  // 每50次循环喂一次狗
    }
    delay(10);
  }
  
  // 计算平均值
  mpuData.gyroX_offset = gx/samples;
  mpuData.gyroY_offset = gy/samples;
  mpuData.gyroZ_offset = gz/samples;
  
  // 计算加速度计零偏
  // 注意：Z轴应该接近1g（重力加速度）
  mpuData.accX_offset = ax/samples;
  mpuData.accY_offset = ay/samples;
  mpuData.accZ_offset = (az/samples) - 1.0;  // 减去1g的重力加速度
  
  Serial.println(F("传感器校准完成:"));
  Serial.print(F("陀螺仪零偏: "));
  Serial.print(mpuData.gyroX_offset, 4);
  Serial.print(F(", "));
  Serial.print(mpuData.gyroY_offset, 4);
  Serial.print(F(", "));
  Serial.println(mpuData.gyroZ_offset, 4);
  
  Serial.print(F("加速度计零偏: "));
  Serial.print(mpuData.accX_offset, 4);
  Serial.print(F(", "));
  Serial.print(mpuData.accY_offset, 4);
  Serial.print(F(", "));
  Serial.println(mpuData.accZ_offset, 4);
  
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
  Serial.println("initController OK");
}

void showLight(){
  digitalWrite(PIN_LED, HIGH);
  delay(500);
  feedWatchdog();  // 喂狗
  digitalWrite(PIN_LED, LOW);
  delay(500);
  feedWatchdog();  // 喂狗
  digitalWrite(PIN_LED, HIGH);
  delay(500);
  feedWatchdog();  // 喂狗
  digitalWrite(PIN_LED, LOW);
}

void initRF(){
  if (!radio.begin()){
    Serial.println("radio.begin failed!");
    radio.printPrettyDetails();
    while(1) {
      feedWatchdog();  // 喂狗，避免重启
      delay(100);
    }
  }  

  radio.setChannel(108); 
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.setRetries(0, 0);  
  radio.setCRCLength(RF24_CRC_16);  
  radio.startListening();
  Serial.println("initRF OK");
}

void selfCheck(){
  //回正
  ch1.write(90);
  ch2.write(90);
  ch3.write(90);
  delay(500);
  feedWatchdog();  // 喂狗

  //低极值
  ch1.write(45);
  ch2.write(45);
  ch3.write(45);
  delay(500);
  feedWatchdog();  // 喂狗

  //高极值
  ch1.write(135);
  ch2.write(135);
  ch3.write(135);
  delay(500);
  feedWatchdog();  // 喂狗

  //回正
  ch1.write(90);
  ch2.write(90);
  ch3.write(90);

  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  feedWatchdog();  // 喂狗
  digitalWrite(PIN_LED, LOW);
}

void testMPU(){
  readMPU6500();
  ControlData data;
  data.aircraft_type = AIRCRAFT_PAPER_PLANE;
  stabilizeFlight(data);
  return;
}

void loop() {
  // 更新运行时间（用于重启诊断）
  updateUptime();
  
  // 喂看门狗
  feedWatchdog();
  
#ifdef TEST_MPU6500  
  delay(100);
  return testMPU();
#endif

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
          ch1.write(map(data.ch1, -512, 511, 45, 135));  // 右舵机
          ch2.write(map(data.ch2, -512, 511, 135, 45));  // 左舵机
          ch3.write(90);  // 保持中立
          break;
          
        case AIRCRAFT_CAMEL:
          // 骆驼战斗机：只使用ch1和ch2
          ch1.write(map(data.ch1, -512, 511, 45, 135));  // 右舵机
          ch2.write(map(data.ch2, -512, 511, 45, 135));  // 左舵机
          ch3.write(90);  // 保持中立
          break;
          
        case AIRCRAFT_P51:
          // P51使用全部三个通道
          ch1.write(map(data.ch1, -512, 511, 45, 135));  // 右舵机
          ch2.write(map(data.ch2, -512, 511, 45, 135));  // 左舵机
          ch3.write(map(data.ch3, -512, 511, 135, 45));
          break;
      } 

      // 飞行模式处理
#ifdef ENABLE_SELFCTL      
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
#endif      
    } else {
        Serial.println(F("checksum failed..."));      
    }   
  } else {  
    digitalWrite(PIN_LED, LOW);  
  }  
  
  // 失控保护（1.5秒无信号切断油门）- 减少保护时间
  if(millis() - lastSignalTime > 1500) {
    esc.writeMicroseconds(1000);      // 紧急停机
    ch1.write(90);                // 回中
    ch2.write(90);
    ch3.write(90);
  }
}