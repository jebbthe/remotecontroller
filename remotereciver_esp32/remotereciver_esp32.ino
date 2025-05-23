#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <printf.h>

// 自定义 SPI 引脚（MOSI, MISO, SCK）
#define MOSI_PIN 5
#define MISO_PIN 6
#define SCK_PIN  4
// CE 和 CSN 引脚
#define CE_PIN   9
#define CSN_PIN  8

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
  int16_t roll, pitch, yaw;
  uint8_t checksum;
};

const byte address[6] = "FLY01";

void setup() {
   // 手动初始化 SPI 并指定引脚
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN); 

  Serial.begin(9600);
  printf_begin();

  pinMode(PIN_LED, OUTPUT);

  initController();
  initRF();
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
}


void initController(){
  esc.attach(PIN_ESC, 1000, 2000);   // 电调初始化
  aileron.attach(PIN_aileron);           
  elevator.attach(PIN_elevator);         
  rudder.attach(PIN_rudder);             
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
    while(1);
  }  
  
  radio.setChannel(108); 
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true); // 默认启用
  radio.setRetries(3, 5); // 延迟=250μs*5, 重试15次
  radio.setCRCLength(RF24_CRC_16);
  radio.printDetails();
  radio.startListening();
}

void selfCheck(){
  //回正
  aileron.write(90);
  elevator.write(90);
  rudder.write(90);
  delay(1000);

  //低极值
  aileron.write(45);
  elevator.write(45);
  rudder.write(45);
  delay(1000);

  //高极值
  aileron.write(135);
  elevator.write(135);
  rudder.write(135);
  delay(1000);

  //回正
  aileron.write(90);
  elevator.write(90);
  rudder.write(90);
  delay(1000);

  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
}


void loop() {
  if(radio.available()){
    ControlData data;
    radio.read(&data, sizeof(data));
    uint8_t sum = (data.throttle + data.roll + data.pitch + data.yaw) % 256;
    if(sum == data.checksum) {
      digitalWrite(PIN_LED, HIGH);
      lastSignalTime = millis();
      // 油门通道处理
      int throttle = map(data.throttle, 0, 1023, 1000, 2000);
      esc.writeMicroseconds(throttle);
      // 舵机通道处理（±30度）
      aileron.write(map(data.pitch, -512, 511, 45, 135));
      elevator.write(map(data.roll, -512, 511, 45, 135));
      rudder.write(map(data.yaw, -512, 511, 45, 135));
    } 
  } else {  
      digitalWrite(PIN_LED, LOW);  
  }  
  
  // 失控保护（1秒无信号切断油门）
  if(millis() - lastSignalTime > 1000) {
    esc.writeMicroseconds(1000);      // 紧急停机
    aileron.write(90);                // 回中
    elevator.write(90);
    rudder.write(90);
  }
}