# 航模飞行姿态模拟器 V2.0

这是一个Python桌面程序，用于模拟显示航模的飞行姿态以及自平衡动作，支持串口数据实时获取和回放。

## 新功能特点

### 1. **串口数据获取**
- 支持实时串口连接，获取Arduino/ESP32的PID输出数据
- 自动解析stabilizeFlight函数的Serial.println输出格式
- 每2秒回放一次串口数据，模拟真实飞行过程
- 支持串口配置：端口选择、波特率设置

### 2. **优化的纸飞机显示**
- 重新设计的纸飞机外形，更接近真实纸飞机
- 只有左右两个舵面，基于混合控制模式
- 舵面角度实时显示和控制
- 尾部视角显示，清晰展示舵面变化

### 3. **修复的中文显示**
- 自动检测和设置中文字体
- 解决matplotlib中文显示为方框的问题
- 支持多种中文字体：SimHei、Microsoft YaHei等

### 4. **增强的PID数据显示**
- 实时显示PID误差、积分、输出值
- 显示左右舵机的目标角度
- 支持串口数据的实时更新

## 安装和运行

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 运行程序
```bash
python flight_simulator_v2.py
```

## 使用说明

### 串口连接
1. **选择串口**: 在串口配置面板中选择正确的COM端口
2. **设置波特率**: 通常为9600（与Arduino/ESP32匹配）
3. **连接**: 点击"Connect"按钮建立连接
4. **数据回放**: 连接后程序会自动解析和回放串口数据

### 数据格式
程序支持解析以下格式的串口数据：
```
Pitch Error: 2.50 Roll Error: -1.20 Pitch Integral: 5.30 Roll Integral: -2.10 Pitch Output: 8.20 Roll Output: -4.50 Right Target: 95.20 Left Target: 84.80
```

### 控制面板
- **姿态控制**: 手动调节俯仰角、横滚角、偏航角
- **自平衡模式**: 启用/禁用PID自平衡控制
- **目标姿态**: 设置自平衡的目标角度
- **PID数据显示**: 实时显示PID参数和舵机角度

### 显示区域
- **3D姿态仪**: 显示飞机的三维姿态
- **纸飞机视图**: 从尾部视角显示纸飞机和舵面变化

## 技术特点

1. **串口通信**: 使用pyserial库实现稳定的串口通信
2. **多线程**: 串口读取在独立线程中运行，不阻塞UI
3. **数据解析**: 正则表达式解析PID输出数据
4. **混合控制**: 实现纸飞机的对称俯仰控制和差动横滚控制
5. **字体管理**: 自动检测和配置中文字体

## 系统要求

- Python 3.7+
- Windows/macOS/Linux
- 支持串口通信的硬件
- 至少4GB内存
- 支持OpenGL的显卡

## 故障排除

### 串口连接问题
1. 确保Arduino/ESP32已正确连接
2. 检查串口号是否正确
3. 确认波特率设置匹配
4. 检查设备驱动是否正常

### 显示问题
1. 确保安装了所有依赖包
2. 检查显卡驱动是否支持OpenGL
3. 如果中文仍显示为方框，手动安装中文字体

### 数据解析问题
1. 确认Arduino代码输出格式正确
2. 检查串口数据是否完整
3. 查看控制台输出的解析错误信息

## 与Arduino代码集成

确保Arduino/ESP32代码中的stabilizeFlight函数输出格式如下：
```cpp
Serial.print("Pitch Error: ");
Serial.print(pitchError);
Serial.print(" Roll Error: ");
Serial.print(rollError);
Serial.print(" Pitch Integral: ");
Serial.print(pitchIntegral);
Serial.print(" Roll Integral: ");
Serial.print(rollIntegral);
Serial.print(" Pitch Output: ");
Serial.print(smoothedPitchOutput);
Serial.print(" Roll Output: ");
Serial.print(smoothedRollOutput);
Serial.print(" Right Target: ");
Serial.print(rightTarget);
Serial.print(" Left Target: ");
Serial.println(leftTarget);
``` 