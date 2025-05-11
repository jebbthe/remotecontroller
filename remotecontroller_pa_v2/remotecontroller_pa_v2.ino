#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <printf.h>
#include <EEPROM.h>

//#define DEBUG

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define CE_P             7
#define CSN_P            8
RF24 radio(CE_P, CSN_P); 

// 硬件引脚定义
#define PIN_SIG_LED          9     // 信号LED
#define PIN_BEEP             3     // 喇叭
#define MAX_STILL_LOOP       1500
#define FLAP_MAX  511
#define FLAP_MIN -512

const byte address[6] = "FLY01";
const int deadZone = 20;

unsigned long lastAlertTime = 0;
unsigned long failedCount = 0;
const unsigned long ALERT_LIMIT = 250;

// Add flight mode enum after ModelType enum
enum FlightMode { FLIGHT_MODE_MANUAL, FLIGHT_MODE_STABILIZE, FLIGHT_MODE_HOLD };

struct ControlData {
  uint8_t aircraft_type;      // 机型
  uint8_t flight_mode;        // 飞行模式
  uint16_t throttle;          // 油门
  int16_t ch1;                // 通道1
  int16_t ch2;                // 通道2
  int16_t ch3;                // 通道3
  uint8_t checksum;           // 校验和
};

ControlData lastData;           //近一次控制数据
unsigned long stillCount = 0;   //持续静止周期数

unsigned long lastPitchTime = 0;
int lastPitchValue = 512;
unsigned long lastRollTime = 0;
int lastRollValue = 512;

// Menu system variables
enum ModelType { PAPER_PLANE, CAMEL_FIGHTER, P51_FIGHTER };
enum MixMode { MIXED, DIRECT };
struct Config {
  ModelType modelType;
  MixMode mixMode;
  bool ch1Reversed;
  bool ch2Reversed;
} config;

// Menu state variables
enum MenuState { MAIN_MENU, MODEL_SELECT, MIX_SELECT, CH1_REVERSE, CH2_REVERSE, CONFIG_DONE };
MenuState currentMenuState = MAIN_MENU;
int menuCursor = 0;
const int MENU_DEADZONE = 20;
unsigned long lastMenuInput = 0;
const unsigned long MENU_INPUT_DELAY = 200; // ms 

// Joystick threshold values (0-1023)
const int JOYSTICK_THRESHOLD_HIGH = 800;  // 70% of max value
const int JOYSTICK_THRESHOLD_LOW = 200;   // 30% of max value

// Configuration storage
#define CONFIG_VERSION 1  // 用于配置版本控制
#define CONFIG_START_ADDR 0  // EEPROM起始地址

// 配置存储结构
struct StoredConfig {
  uint8_t version;
  ModelType modelType;
  MixMode mixMode;
  bool ch1Reversed;
  bool ch2Reversed;
};

// Menu items count for each state
const int MAIN_MENU_ITEMS = 5;  // Load Config, Model Type, Mix Mode, CH1 Reverse, CH2 Reverse
const int MODEL_SELECT_ITEMS = 3;  // Paper Plane, Camel Fighter, P51 Fighter
const int MIX_SELECT_ITEMS = 2;    // Mixed, Direct
const int REVERSE_ITEMS = 2;       // Normal, Reversed

// Add these variables at the top with other globals
const int RSSI_HISTORY_SIZE = 10;
int rssiHistory[RSSI_HISTORY_SIZE];
int rssiHistoryIndex = 0;
unsigned long lastRssiUpdate = 0;
const unsigned long RSSI_UPDATE_INTERVAL = 100; // Update RSSI every 100ms

// Menu system functions
void handleMenuInput() {
  int ch1 = analogRead(A1);
  int ch2 = analogRead(A2);
  
  if (millis() - lastMenuInput < MENU_INPUT_DELAY) return;
  
  // Channel 1 for navigation
  if (ch1 > JOYSTICK_THRESHOLD_HIGH || ch1 < JOYSTICK_THRESHOLD_LOW) {
    int maxItems;
    switch (currentMenuState) {
      case MAIN_MENU:
        maxItems = MAIN_MENU_ITEMS;
        break;
      case MODEL_SELECT:
        maxItems = MODEL_SELECT_ITEMS;
        break;
      case MIX_SELECT:
        maxItems = MIX_SELECT_ITEMS;
        break;
      case CH1_REVERSE:
      case CH2_REVERSE:
        maxItems = REVERSE_ITEMS;
        break;
      default:
        maxItems = 2;
    }
    
    if (ch1 < JOYSTICK_THRESHOLD_LOW) {
      menuCursor = (menuCursor + 1) % maxItems;
    } else {
      menuCursor = (menuCursor - 1 + maxItems) % maxItems;
    }
    lastMenuInput = millis();
  }
  
  // Channel 2 for selection/back
  if (ch2 > JOYSTICK_THRESHOLD_HIGH || ch2 < JOYSTICK_THRESHOLD_LOW) {
    if (ch2 > JOYSTICK_THRESHOLD_HIGH) { // Select
      switch (currentMenuState) {
        case MAIN_MENU:
          if (menuCursor == 0) {
            // Load Config option
            if (loadConfig()) {
              currentMenuState = CONFIG_DONE;
            } else {
              // 如果加载失败，显示错误信息
              showText("No Config");
              delay(1000);
            }
          } else {
            currentMenuState = (MenuState)(menuCursor);  // 注意这里不再+1，因为第一个选项是Load Config
            menuCursor = 0;
          }
          break;
        case MODEL_SELECT:
          config.modelType = (ModelType)menuCursor;
          currentMenuState = MIX_SELECT;
          menuCursor = 0;
          break;
        case MIX_SELECT:
          config.mixMode = (MixMode)menuCursor;
          currentMenuState = CH1_REVERSE;
          menuCursor = 0;
          break;
        case CH1_REVERSE:
          config.ch1Reversed = menuCursor == 1;
          currentMenuState = CH2_REVERSE;
          menuCursor = 0;
          break;
        case CH2_REVERSE:
          config.ch2Reversed = menuCursor == 1;
          currentMenuState = CONFIG_DONE;
          break;
      }
    } else { // Back
      if (currentMenuState > MAIN_MENU) {
        currentMenuState = MAIN_MENU;
        menuCursor = 0;
      }
    }
    lastMenuInput = millis();
  }
}

void displayMenu() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  
  switch (currentMenuState) {
    case MAIN_MENU:
      oled.println(F("Select Option:"));
      oled.println();  // Add blank line
      for (int i = 0; i < MAIN_MENU_ITEMS; i++) {
        oled.print(menuCursor == i ? F(">") : F(" "));
        switch(i) {
          case 0: oled.println(F("Load Config")); break;
          case 1: oled.println(F("Model Type")); break;
          case 2: oled.println(F("Mix Mode")); break;
          case 3: oled.println(F("CH1 Reverse")); break;
          case 4: oled.println(F("CH2 Reverse")); break;
        }
      }
      break;
      
    case MODEL_SELECT:
      oled.println(F("Select Model:"));
      oled.println();  // Add blank line
      for (int i = 0; i < MODEL_SELECT_ITEMS; i++) {
        oled.print(menuCursor == i ? F(">") : F(" "));
        switch(i) {
          case 0: oled.println(F("Paper Plane")); break;
          case 1: oled.println(F("Camel Fighter")); break;
          case 2: oled.println(F("P51 Fighter")); break;
        }
      }
      break;
      
    case MIX_SELECT:
      oled.println(F("Select Mix Mode:"));
      oled.println();  // Add blank line
      for (int i = 0; i < MIX_SELECT_ITEMS; i++) {
        oled.print(menuCursor == i ? F(">") : F(" "));
        switch(i) {
          case 0: oled.println(F("Mixed")); break;
          case 1: oled.println(F("Direct")); break;
        }
      }
      break;
      
    case CH1_REVERSE:
      oled.println(F("CH1 Reverse:"));
      oled.println();  // Add blank line
      for (int i = 0; i < REVERSE_ITEMS; i++) {
        oled.print(menuCursor == i ? F(">") : F(" "));
        switch(i) {
          case 0: oled.println(F("Normal")); break;
          case 1: oled.println(F("Reversed")); break;
        }
      }
      break;
      
    case CH2_REVERSE:
      oled.println(F("CH2 Reverse:"));
      oled.println();  // Add blank line
      for (int i = 0; i < REVERSE_ITEMS; i++) {
        oled.print(menuCursor == i ? F(">") : F(" "));
        switch(i) {
          case 0: oled.println(F("Normal")); break;
          case 1: oled.println(F("Reversed")); break;
        }
      }
      break;
      
    case CONFIG_DONE:
      oled.println(F("Config Complete!"));
      oled.println();  // Add blank line
      oled.println(F("Press to start"));
      break;
  }
  oled.display();
}

void runMenuSystem() {
  while (currentMenuState != CONFIG_DONE) {
    handleMenuInput();
    displayMenu();
    delay(50);
  }
  
  // 保存配置
  saveConfig();
  
  // 显示完整配置内容并停留2秒
  showConfigSummary();
  delay(2000);
  
  // 等待用户确认开始
  while (abs(analogRead(A2) - 512) <= MENU_DEADZONE) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println(F("Config Complete!"));
    oled.println();
    oled.println(F("Press to start"));
    oled.display();
    delay(50);
  }
}

void setup() {
  delay(1000);
  
  analogReference(DEFAULT);
  Serial.begin(9600);
  printf_begin();


  Serial.println(F("开始初始化系统..."));
  //初始化OLED
  Serial.println(F("开始初始化OLED..."));
  initOLED();
  Serial.println(F("OLED初始化完成!"));

  Serial.println(F("开始初始化nano管脚..."));
  initPin();
  Serial.println(F("管脚设置完成!"));

  //无线模块自检
  Serial.println(F("开始初始化RF..."));
  initRF();
  initRSSI();  // Initialize RSSI history
  Serial.println(F("RF初始化完成!"));

  // Run configuration menu
  runMenuSystem();

  // 油门安全检查
  while(!checkThrottleSafety()) {
    showText("Zero T...");
    shortBeep();
    delay(1000);
  }
  Serial.println(F("油门安全正常"));

  showReady();
}

void shortBeep(){
  digitalWrite(PIN_BEEP, HIGH);
  delay(500);
  digitalWrite(PIN_BEEP, LOW);
}


// OLED初始化
void initOLED() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED初始化失败!"));
    while(1);
  }
  
  // 设置OLED显示参数
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  
  // 设置显示方向
  oled.setRotation(0);
  
  // 禁用滚动
  oled.stopscroll();
  
  // 显示初始画面
  showText("Loading...");
}

void initPin(){
  //设置管脚模式
  pinMode(PIN_SIG_LED, OUTPUT);
  pinMode(PIN_BEEP, OUTPUT);
  Serial.println("管脚模式设置完成");

  digitalWrite(PIN_BEEP, LOW);
  digitalWrite(PIN_SIG_LED, LOW);
  Serial.println("LED设置完成");
}

void initRF(){
  //初始化无线模块
  if (!radio.begin()){
    showText("RF fail");
    beepForRFFail();
    while(1);
  }

  Serial.println("radio.begin");
  Serial.println("无线模块正常");

  radio.setChannel(108); 
  radio.setDataRate(RF24_250KBPS);  
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.setRetries(3, 5);        // 减少重试次数和延迟
  radio.setAutoAck(true);
  radio.setCRCLength(RF24_CRC_16);
  radio.stopListening();
#ifdef DEBUG
  radio.printDetails();
#endif
}

void beepForRFFail(){
  for (int i=0;i<3;i++){
    digitalWrite(PIN_BEEP, HIGH);
    delay(100);
    digitalWrite(PIN_BEEP, LOW);
    delay(100);
  }
}

// 油门安全检查
bool checkThrottleSafety() {
  return (analogRead(A0) < 200); // 死区阈值
}

void showText(const char* str){
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(5, 28);
    oled.print(str);
    oled.display();
}

void showReady(){
  showText("Hello Baby");
  digitalWrite(PIN_SIG_LED, HIGH);
  delay(500);
  digitalWrite(PIN_SIG_LED, LOW);

  Serial.println(F("系统就绪提示音"));
  digitalWrite(PIN_BEEP, HIGH);
  delay(1000);
  digitalWrite(PIN_BEEP, LOW);
}

void showConfigSummary() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("Config Summary:"));
  oled.println();
  oled.print(F("Model: "));
  oled.println(config.modelType == PAPER_PLANE ? F("Paper Plane") : config.modelType == CAMEL_FIGHTER ? F("Camel Fighter") : F("P51 Fighter"));
  oled.print(F("Mix: "));
  oled.println(config.mixMode == MIXED ? F("Mixed") : F("Direct"));
  oled.print(F("CH1: "));
  oled.println(config.ch1Reversed ? F("Reversed") : F("Normal"));
  oled.print(F("CH2: "));
  oled.println(config.ch2Reversed ? F("Reversed") : F("Normal"));
  oled.display();
}

void loop() {

  // 数据采集
  int throttle = analogRead(A0);
  int throttlePercentage = map(throttle, 0, 1023, 0, 100); 
  //支持死区+反转+映射正负区间
  int16_t pitchValue = processFlapInput(A1,  config.ch1Reversed, lastPitchValue, lastPitchTime);
  int16_t rollValue = processFlapInput(A2,  config.ch2Reversed, lastRollValue, lastRollTime);

#ifdef DEBUG
  Serial.print("flagInput:");Serial.print(pitchValue);Serial.print(",");Serial.println(rollValue);
#endif  

  int16_t left_flap = 0;
  int16_t right_flap = 0;
  
  if (config.mixMode == MIXED){
    // 冲突解决逻辑    
    // 俯仰控制（对称操作）
    left_flap += pitchValue;
    right_flap += pitchValue;
    
    // 翻滚控制（差动操作）
    left_flap -= rollValue;
    right_flap += rollValue;

    // 限制舵机范围并应用优先级
    bool pitchNewer = (lastPitchTime > lastRollTime);
    constrainFlaps(left_flap, right_flap, pitchNewer);
  } else {
    left_flap = pitchValue;
    right_flap = rollValue;
  }

  // 状态检测
  int rssi = calculateRSSI();

  ControlData data;
  // Set aircraft_type and flight_mode based on config/menu (example:)
  data.aircraft_type = config.modelType; // Now maps directly to 0, 1, or 2
  if (throttle == 0 && abs(pitchValue) < deadZone && abs(rollValue) < deadZone) {
    data.flight_mode = FLIGHT_MODE_STABILIZE; // Auto stabilize when throttle is zero and no joystick input
  } else {
    data.flight_mode = FLIGHT_MODE_MANUAL;    // Manual mode otherwise
  }
  data.throttle = throttle;
  // Map left_flap/right_flap to ch1/ch2, ch3=0 for now
  data.ch1 = left_flap;
  data.ch2 = right_flap;
  data.ch3 = 0; // If you have a third channel, set it here
  data.checksum = (data.aircraft_type + data.flight_mode + data.throttle + data.ch1 + data.ch2 + data.ch3) % 256;

  if (!checkAction(data)){
    stillCount++;
  } else {
    stillCount = 0;
  }

  //Serial.println(stillCount);
  if (stillCount >= MAX_STILL_LOOP){
    Serial.println(F("进入睡眠模式"));
    if (stillCount == MAX_STILL_LOOP){
      shortBeep();
    }
    showText("Sleeping");
    return;
  } else {
    // OLED显示
    displayStatus(rssi, throttlePercentage, 
               map(left_flap, FLAP_MIN, FLAP_MAX, -100, 100),
               map(right_flap, FLAP_MIN, FLAP_MAX, -100, 100));

    if (radio.write(&data, sizeof(data))){
      failedCount = 0;
      Serial.println(F("发送数据成功"));
      digitalWrite(PIN_SIG_LED, HIGH);
      delayForNextSend();
      digitalWrite(PIN_SIG_LED, LOW);
    } else {
      failedCount ++;
      Serial.println(F("发送数据失败"));
#ifdef DEBUG
      radio.printDetails();
#endif    
      if (!beepForSendFailed()){
        delayForNextSend();
      }
    } 
  }
}

boolean checkAction(ControlData &data){
  int throttleDiff = abs(lastData.throttle - data.throttle);
  int ch1Diff = abs(lastData.ch1 - data.ch1);
  int ch2Diff = abs(lastData.ch2 - data.ch2);
  int ch3Diff = abs(lastData.ch3 - data.ch3);
  if (throttleDiff < 3 && ch1Diff < 3 && ch2Diff < 3 && ch3Diff < 3) {
    return false;
  } else {
    lastData.throttle = data.throttle;
    lastData.ch1 = data.ch1;
    lastData.ch2 = data.ch2;
    lastData.ch3 = data.ch3;
    return true;
  }
}

//冲突解决
void constrainFlaps(int16_t &left, int16_t &right, bool pitchPriority) {
  left = constrain(left, FLAP_MIN, FLAP_MAX);
  right = constrain(right, FLAP_MIN, FLAP_MAX);
}

int16_t processFlapInput(int pin, bool reverse, int &lastActiveValue, unsigned long &lastActiveTime) {
  int val = analogRead(pin);
#ifdef DEBUG
  Serial.print("analogRead:");Serial.println(val);
#endif  
  if(abs(val - 512) < deadZone){
    val = 512;
  }
  if(val != lastActiveValue) { 
    lastActiveValue = val;
    lastActiveTime = millis();
  }
  val = map(val, 0, 1023, FLAP_MIN, FLAP_MAX);
  val = reverse ? -val : val;
  return constrain(val, FLAP_MIN, FLAP_MAX);
}


void delayForNextSend(){
#ifdef DEBUG
    delay(1000);
#else    
    delay(10);  //100Hz刷新率
#endif    
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
  for (int i=0;i<3;i++){
    digitalWrite(PIN_BEEP, HIGH);
    delay(150);
    digitalWrite(PIN_BEEP, LOW);
    delay(150);
  }
  return true;
}

// 修改显示函数，优化显示更新逻辑
void displayStatus(int rssi, int t, int left, int right) {
  static unsigned long lastDisplayUpdate = 0;
  static int lastRssi = 0;
  static int lastT = 0;
  static int lastLeft = 0;
  static int lastRight = 0;
  
  // 检查是否需要更新显示
  if (millis() - lastDisplayUpdate < 200) {  // 降低更新频率到200ms
    return;
  }
  
  // 检查数据是否发生变化
  if (rssi == lastRssi && t == lastT && left == lastLeft && right == lastRight) {
    return;
  }
  
  // 更新上次显示的数据
  lastRssi = rssi;
  lastT = t;
  lastLeft = left;
  lastRight = right;
  lastDisplayUpdate = millis();

  // 使用局部缓冲区
  oled.clearDisplay();
  
  // 顶部状态栏 - 使用固定宽度格式化
  oled.setCursor(0, 0);
  oled.print(F("SIG:"));
  oled.print(rssi);
  oled.print(F("%"));

  // 通道数据显示 - 使用固定宽度格式化
  oled.setCursor(0, 16);
  oled.print(F("THR:"));
  oled.print(t);
  oled.print(F("%"));
  
  oled.setCursor(0, 32);
  oled.print(F("CH1:"));
  oled.print(left);
  oled.print(F("%"));
  
  oled.setCursor(0, 48);
  oled.print(F("CH2:"));
  oled.print(right);
  oled.print(F("%"));
  
  // 确保显示更新完成
  oled.display();
}

int calculateRSSI() {
  static int lastRssi = 50;  // 默认中间值
  static unsigned long totalPackets = 0;  // 总数据包数
  static unsigned long failedPackets = 0;  // 失败数据包数
  
  // 更新数据包统计
  totalPackets++;
  if (failedCount > 0) {
    failedPackets++;
  }
  
  // 计算丢包率(0-100%)
  int packetLossRate = (totalPackets > 0) ? (failedPackets * 100 / totalPackets) : 0;
  
  // 获取自动重发计数(0-15)
  uint8_t arc = radio.getARC();
  
  // 基于ARC计算基础RSSI(0-100%)
  int arcRssi = constrain(100 - (arc * 6), 0, 100);
  
  // 使用加权平均计算最终RSSI
  // ARC基础RSSI权重60%
  // 丢包率权重40%
  int currentRssi = (arcRssi * 60 + (100 - packetLossRate) * 40) / 100;
  
  // 使用移动平均进行平滑处理
  rssiHistory[rssiHistoryIndex] = currentRssi;
  rssiHistoryIndex = (rssiHistoryIndex + 1) % RSSI_HISTORY_SIZE;
  
  int smoothedRssi = 0;
  for (int i = 0; i < RSSI_HISTORY_SIZE; i++) {
    smoothedRssi += rssiHistory[i];
  }
  smoothedRssi /= RSSI_HISTORY_SIZE;
  
  // 定期重置统计数据
  if (millis() - lastRssiUpdate > 5000) {  // 每5秒重置一次
    totalPackets = 0;
    failedPackets = 0;
    lastRssiUpdate = millis();
  }
  
  return smoothedRssi;
}

// 在setup()中添加RSSI历史记录初始化
void initRSSI() {
  for (int i = 0; i < RSSI_HISTORY_SIZE; i++) {
    rssiHistory[i] = 50;  // 使用中间值初始化
  }
}

// Configuration functions
void saveConfig() {
  StoredConfig storedConfig;
  storedConfig.version = CONFIG_VERSION;
  storedConfig.modelType = config.modelType;
  storedConfig.mixMode = config.mixMode;
  storedConfig.ch1Reversed = config.ch1Reversed;
  storedConfig.ch2Reversed = config.ch2Reversed;
  
  EEPROM.put(CONFIG_START_ADDR, storedConfig);
}

bool loadConfig() {
  StoredConfig storedConfig;
  EEPROM.get(CONFIG_START_ADDR, storedConfig);
  
  if (storedConfig.version != CONFIG_VERSION) {
    return false;  // 版本不匹配，配置无效
  }
  
  config.modelType = storedConfig.modelType;
  config.mixMode = storedConfig.mixMode;
  config.ch1Reversed = storedConfig.ch1Reversed;
  config.ch2Reversed = storedConfig.ch2Reversed;
  
  return true;
}