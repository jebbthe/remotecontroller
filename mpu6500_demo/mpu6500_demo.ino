#include <Wire.h>
#include <Servo.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

Servo myServoX;  // 控制X轴的舵机
Servo myServoY;  // 控制Y轴的舵机

// 互补滤波参数
const float alpha = 0.98;
float filteredAngleX = 0;
float filteredAngleY = 0;
float gyroRateX = 0;
float gyroRateY = 0;
unsigned long lastTime = 0;
float gyroZeroOffsetX = 0;
float gyroZeroOffsetY = 0;

void setup() {
    Serial.begin(9600);
    myServoX.attach(9);
    myServoY.attach(10);
    myServoX.write(0);
    myServoY.write(0);

    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission(true);

    // 校准陀螺仪零点偏移
    calibrateGyro();

    lastTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // 读取加速度计数据
    int16_t accelX = readWordFromReg(ACCEL_XOUT_H);
    int16_t accelY = readWordFromReg(ACCEL_XOUT_H + 2);
    int16_t accelZ = readWordFromReg(ACCEL_XOUT_H + 4);

    // 读取陀螺仪数据
    int16_t gyroX = readWordFromReg(GYRO_XOUT_H);
    int16_t gyroY = readWordFromReg(GYRO_XOUT_H + 2);

    // 计算X轴倾斜角度（加速度计）
    float accelAngleX = atan2((float)accelY, (float)accelZ) * 180 / PI;
    // 计算Y轴倾斜角度（加速度计）
    float accelAngleY = atan2(-(float)accelX, (float)accelZ) * 180 / PI;

    // 计算X轴角速度（减去零点偏移）
    gyroRateX = (float)(gyroX - gyroZeroOffsetX) / 131.0;
    // 计算Y轴角速度（减去零点偏移）
    gyroRateY = (float)(gyroY - gyroZeroOffsetY) / 131.0;

    // 互补滤波融合加速度计和陀螺仪数据
    filteredAngleX = alpha * (filteredAngleX + gyroRateX * dt) + (1 - alpha) * accelAngleX;
    filteredAngleY = alpha * (filteredAngleY + gyroRateY * dt) + (1 - alpha) * accelAngleY;

    // 映射角度到舵机范围（假设舵机为0 - 180°）
    int servoAngleX = map(filteredAngleX, -90, 90, 0, 180);
    servoAngleX = constrain(servoAngleX, 0, 180);
    int servoAngleY = map(filteredAngleY, -90, 90, 0, 180);
    servoAngleY = constrain(servoAngleY, 0, 180);

    myServoX.write(servoAngleX);
    myServoY.write(servoAngleY);

    // 打印计算得到的倾斜角度和舵机控制角度
    Serial.print("Filtered Angle X: ");
    Serial.print(filteredAngleX);
    Serial.print("°, Servo Angle X: ");
    Serial.print(servoAngleX);
    Serial.print("°, Filtered Angle Y: ");
    Serial.print(filteredAngleY);
    Serial.print("°, Servo Angle Y: ");
    Serial.println(servoAngleY);

    delay(50);
}

// 从指定寄存器读取一个16位的数据
int16_t readWordFromReg(byte reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    return (Wire.read() << 8) | Wire.read();
}

// 陀螺仪零点校准函数
void calibrateGyro() {
    Serial.println("校准中...保持MPU静止！");
    float sumX = 0;
    float sumY = 0;
    for (int i = 0; i < 200; i++) {
        int16_t gyroX = readWordFromReg(GYRO_XOUT_H);
        int16_t gyroY = readWordFromReg(GYRO_XOUT_H + 2);
        sumX += (float)gyroX;
        sumY += (float)gyroY;
        delay(10);
    }
    gyroZeroOffsetX = sumX / 200;
    gyroZeroOffsetY = sumY / 200;
    Serial.print("X轴零点偏移校准值：");
    Serial.println(gyroZeroOffsetX);
    Serial.print("Y轴零点偏移校准值：");
    Serial.println(gyroZeroOffsetY);
}    