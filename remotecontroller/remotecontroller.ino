#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <printf.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RF24 radio(7, 8); // CE,CSN
const byte address[6] = "FLY01";
const int deadZone = 20;
const bool debug = false;

unsigned long lastAlertTime = 0;
unsigned long failedCount = 0;
const unsigned long ALERT_LIMIT = 250;


#define R1 10000    // 10kΩ
#define R2 12000    // 12kΩ
#define VOLTAGE_DIVIDER_RATIO ((R1 + R2) / (float)R2)

// 硬件引脚定义
#define VIN_ANALOG_PIN   A6    // 电压检测引脚
#define REV_SW_PITCH     4     // 俯仰反转开关
#define REV_SW_ROLL      5     // 横滚反转开关
#define REV_SW_YAW       6     // 偏航反转开关
#define POWER_LED        2     // 电源LED
#define SIG_LED          9     // 信号LED
#define BEEP             3     // 喇叭

struct ControlData {
  uint16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint8_t checksum;
};

// 函数声明
void initOLED();
void displayStatus(float voltage, int rssi, int throttle, int pitch, int roll, int yaw);
bool checkThrottleSafety();
int readChannel(int pin, bool reverse);
float readVoltage();
int calculateRSSI();

void setup() {
  delay(1000);
  
  analogReference(DEFAULT);
  Serial.begin(9600);
  printf_begin();
  
  //设置电源灯
  pinMode(POWER_LED, OUTPUT);
  digitalWrite(POWER_LED, HIGH);

  //设置喇叭
  pinMode(BEEP, OUTPUT);
  digitalWrite(BEEP, LOW);

  // 初始化OLED
  initOLED();

  //无线模块自检
  //初始化无线模块
  if (!radio.begin()){
    showText("RFModule not exist!");
    while(1);
  }

  radio.setChannel(108); 
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.setRetries(3, 5);       // 延迟750μs，重试5次
  radio.setAutoAck(true);
  radio.setCRCLength(RF24_CRC_16);
  radio.printDetails();
  radio.stopListening();

  // 油门安全检查
  while(!checkThrottleSafety()) {
    digitalWrite(BEEP, HIGH);
    showText("Shutoff throttle!");
    delay(500);
    digitalWrite(BEEP, LOW);
    delay(1000);
  }

  // 初始化反转开关
  pinMode(REV_SW_PITCH, INPUT_PULLUP);
  pinMode(REV_SW_ROLL, INPUT_PULLUP);
  pinMode(REV_SW_YAW, INPUT_PULLUP);
  
  //设置信号灯
  pinMode(SIG_LED, OUTPUT);
  digitalWrite(SIG_LED, LOW);

  showReady();
}

void showText(const char* str){
    oled.clearDisplay();
    oled.setCursor(5, 28);
    oled.print(str);
    oled.display();
}

void showReady(){
  digitalWrite(BEEP, HIGH);
  delay(200);
  digitalWrite(BEEP, LOW);
  delay(200);
  digitalWrite(BEEP, HIGH);
  delay(200);
  digitalWrite(BEEP, LOW);
}

void loop() {

  // 数据采集
  int throttlePercentage = map(analogRead(A0), 0, 1023, 0, 100);
  int pitchPercentage = readChannel(A1, digitalRead(REV_SW_PITCH));
  int rollPercentage = readChannel(A2, digitalRead(REV_SW_ROLL));
  int yawPercentage = readChannel(A3, digitalRead(REV_SW_YAW));

  // 状态检测
  float voltage = readVoltage();
  int rssi = calculateRSSI();

  // OLED显示
  displayStatus(voltage, rssi, throttlePercentage, pitchPercentage, rollPercentage, yawPercentage);

  // 无线发送（保留原有逻辑）
  ControlData data;
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);
  int a3 = analogRead(A3);

  data.throttle = constrain(a0, 0, 1023);
  data.pitch = map(a1, 0, 1023, -512, 511);
  data.roll = map(a2, 0, 1023, -512, 511);
  data.yaw = map(a3, 0, 1023, -512, 511);

  if (digitalRead(REV_SW_PITCH)){
      data.pitch = -1 * data.pitch;
      data.pitch = constrain(data.pitch, -512, 511);
  }

  if (digitalRead(REV_SW_ROLL)){
      data.roll = -1 * data.roll;          
      data.roll = constrain(data.roll, -512, 511);
  }

  if (digitalRead(REV_SW_YAW)){
      data.yaw = -1 * data.yaw;          
      data.yaw = constrain(data.yaw, -512, 511);
  }

  if (debug){
    // 合并输出（推荐格式：CSV逗号分隔）
    Serial.print(a0);               // 油门值
    Serial.print(",");              // 分隔符
    Serial.print(a1);               // 横滚
    Serial.print(",");              // 分隔符
    Serial.print(a2);               // 俯仰
    Serial.print(",");              // 分隔符
    Serial.print(a3);              // 偏航（最后用println换行）
    Serial.print(" ----- ");       // 偏航（最后用println换行）
    
    Serial.print(data.throttle);   // 油门值
    Serial.print(",");             // 分隔符
    Serial.print(data.roll);       // 横滚
    Serial.print(",");             // 分隔符
    Serial.print(data.pitch);      // 俯仰
    Serial.print(",");             // 分隔符
    Serial.println(data.yaw);      // 偏航（最后用println换行）
  }
  
  // 校验和计算
  data.checksum = (data.throttle + data.roll + data.pitch + data.yaw) % 256;
  if (radio.write(&data, sizeof(data))){
    failedCount = 0;
    Serial.println("发送数据成功");
    digitalWrite(SIG_LED, HIGH);
    delayForNextSend();
    digitalWrite(SIG_LED, LOW);
  } else {
    failedCount ++;
    Serial.println("发送数据失败");
    if (!beepForSendFailed()){
      delayForNextSend();
    }
  }
}

void delayForNextSend(){
  if (!debug){
    delay(20);  //50Hz刷新率
  } else {
    delay(5000);
  }
}

boolean beepForSendFailed(){
  if (failedCount < ALERT_LIMIT){
    return false;
  }
  unsigned long now = millis();
  if (now - lastAlertTime < 3000){
    return false;
  }
  lastAlertTime = now;
  digitalWrite(BEEP, HIGH);
  delay(150);
  digitalWrite(BEEP, LOW);
  delay(150);
  digitalWrite(BEEP, HIGH);
  delay(150);
  digitalWrite(BEEP, LOW);
  delay(150);
  digitalWrite(BEEP, HIGH);
  delay(150);
  digitalWrite(BEEP, LOW);
  return true;
}

// OLED初始化
void initOLED() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
      Serial.println("oled failed");
      while(1);
  }
  Serial.println("oled inited");
  oled.clearDisplay();
  oled.setTextSize(1.8);
  oled.setTextColor(SSD1306_WHITE);
}

// 状态显示函数
void displayStatus(float voltage, int rssi, int t, int p, int r, int y) {
  oled.clearDisplay();
  
  // 顶部状态栏
  oled.setCursor(0, 0);
  oled.print("V:");
  oled.print(voltage, 1);
  oled.print("V  ");
  oled.print("S:");
  oled.print(rssi);
  oled.print("%");

  // 通道数据显示
  oled.setCursor(0, 16);
  oled.print("THR:"); oled.print(t); oled.println("%");
  oled.print("ROL:"); oled.print(p); oled.println("%");
  oled.print("PIT:"); oled.print(r); oled.println("%");
  oled.print("YAW:"); oled.print(y); oled.println("%");
  
  oled.display();
}

// 油门安全检查
bool checkThrottleSafety() {
  return (analogRead(A0) < 200); // 死区阈值
}

// 带反转的通道读取
int readChannel(int pin, bool reverse) {
  int val = analogRead(pin);
  val = map(val, 0, 1023, -100, 100);
  return reverse ? -val : val;
}

// 电压检测函数
float readVoltage() {
  int raw = analogRead(VIN_ANALOG_PIN);
  float measured_voltage = raw * 5.0 / 1023.0;  // 假设参考电压5V
  return measured_voltage * VOLTAGE_DIVIDER_RATIO;
}

int calculateRSSI() {
  uint8_t arc = radio.getARC();  // 获取自动重发次数（0-15）
  return constrain(100 - (arc * 6), 0, 100); // 限制在0-100%
}

// 带死区的映射函数
int16_t mapWithDeadzone(int val, int center, int zone) {
  if(abs(val - center) < zone) return 0;
  return map(val, 0, 1023, -512, 511);
}
