#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <printf.h>
#include <Wire.h>

// MPU6500 I2C地址
#define MPU6500_ADDR 0x68

#include "driver/spi_common.h"  // 用于释放SPI总线
#include "driver/spi_master.h"

#define PIN_LED 7             // 信号灯
// 1. 定义I2C引脚：SPIQ(GPIO17)=SCL，GPIO19=SDA
#define I2C_SDA_PIN 19    // I2C数据引脚（GPIO19）
#define I2C_SCc:\Users\jebbt\Documents\gitroot\i2c\CH423S_7Segment.inoL_PIN 8    // I2C时钟引脚（原SPIQ，GPIO17）
#define I2C_MASTER_NUM I2C_NUM_0  // 选择I2C控制器（I2C0，也可选I2C1）
#define I2C_MASTER_FREQ_HZ 400000  // I2C通信速率（400kHz快速模式，兼容多数设备；100kHz为标准模式）


void setup() {
   // 手动初始化 SPI 并指定引脚
  Serial.begin(115200);
  printf_begin();
  
  pinMode(PIN_LED, OUTPUT);
  //提示主电源接通，芯片开始工作
  digitalWrite(PIN_LED, LOW);

  Serial.println(F("开始初始化MPU6500"));
  delay(1000);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println(F("初始化MPU6500成功"));
}

void loop() {
  return;
}