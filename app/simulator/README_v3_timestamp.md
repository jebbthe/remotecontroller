# 航模飞行姿态模拟器 V3.0 (带时间戳)

这是一个纯串口数据驱动的Python桌面程序，用于实时显示航模的飞行姿态和控制数据，支持时间戳功能。

## 主要特点

### 1. **时间戳支持**
- ESP32代码中的stabilizeFlight函数输出包含时间戳
- 格式：`[12345ms] Pitch Error: 2.50 Roll Error: -1.20 ...`
- 模拟器实时解析和显示时间戳信息
- 支持数据时序分析和调试

### 2. **纯串口数据驱动**
- 完全移除内置的PID自平衡仿真功能
- 所有飞行数据都通过串口实时获取
- 实时解析Arduino/ESP32的stabilizeFlight函数输出
- 支持数据统计和更新频率显示

### 3. **优化的纸飞机显示**
- 重新设计的纸飞机外形，更接近真实纸飞机
- 使用更自然的颜色和材质效果
- 添加了垂直尾翼和机头装饰
- 舵面显示更加真实，包含厚度和边缘效果
- 使用不同颜色区分左右舵面

### 4. **实时数据监控**
- 实时显示PID误差、积分、输出值
- 显示左右舵机的目标角度
- 数据统计功能（数据计数、更新频率、时间戳）
- 姿态数据实时更新

## 安装和运行

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 运行程序
```bash
python flight_simulator_v3.py
```

## 使用说明

### 串口连接
1. **选择串口**: 在串口配置面板中选择正确的COM端口
2. **设置波特率**: 通常为9600（与Arduino/ESP32匹配）
3. **连接**: 点击"Connect"按钮建立连接
4. **数据监控**: 连接后程序会实时显示串口数据

### 数据格式
程序支持解析以下格式的串口数据（带时间戳）：
```
[12345ms] Pitch Error: 2.50 Roll Error: -1.20 Pitch Integral: 5.30 Roll Integral: -2.10 Pitch Output: 8.20 Roll Output: -4.50 Right Target: 95.20 Left Target: 84.80
```

### 显示面板
- **Flight Data**: 显示高度、速度等基本飞行数据
- **Attitude Data**: 显示俯仰角、横滚角、偏航角
- **PID Data**: 实时显示PID控制参数
- **Control Surfaces**: 显示舵面角度和舵机位置
- **Data Statistics**: 显示数据统计信息（包括时间戳）

### 可视化区域
- **3D姿态仪**: 显示飞机的三维姿态
- **纸飞机视图**: 从尾部视角显示纸飞机和舵面变化

## ESP32代码优化

### stabilizeFlight函数更新
在ESP32代码的stabilizeFlight函数中，Serial.print输出已添加时间戳：

```cpp
// 调试输出 - 添加时间戳
Serial.print("[");
Serial.print(millis());
Serial.print("ms] Pitch Error: ");
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

### 时间戳功能
- **millis()**: 使用Arduino的millis()函数获取系统运行时间
- **格式**: `[时间戳ms]` 格式，便于解析和显示
- **用途**: 用于数据时序分析、性能监控和调试

## 技术特点

1. **时间戳解析**: 使用正则表达式解析时间戳信息
2. **纯数据驱动**: 不包含任何仿真逻辑，完全依赖串口数据
3. **实时处理**: 串口数据立即处理并显示，无延迟
4. **数据统计**: 实时计算数据更新频率和统计信息
5. **优化的渲染**: 使用更真实的颜色和材质效果
6. **多线程设计**: 串口读取在独立线程中运行

## 与Arduino代码集成

确保Arduino/ESP32代码中的stabilizeFlight函数输出格式如下：
```cpp
Serial.print("[");
Serial.print(millis());
Serial.print("ms] Pitch Error: ");
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

## 纸飞机设计特点

### 外形设计
- **机身**: 细长的三角形，使用淡紫色填充
- **主翼**: 自然的翼型设计，使用米色填充
- **尾翼**: 水平尾翼和垂直尾翼，使用淡蓝色填充
- **机头**: 金色装饰，增加视觉效果

### 舵面显示
- **右舵面**: 红色填充，清晰显示角度变化
- **左舵面**: 青色填充，与右舵面区分
- **舵面厚度**: 显示舵面的立体效果
- **角度指示**: 实时显示舵面偏转角度

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
1. 确认Arduino代码输出格式正确（包含时间戳）
2. 检查串口数据是否完整
3. 查看控制台输出的解析错误信息

## 版本更新说明

### V3.0 主要改进
- 移除所有内置仿真功能
- 优化纸飞机外形设计
- 添加数据统计功能
- 改进舵面显示效果
- 增强实时数据处理能力
- **新增时间戳支持**

### 时间戳功能优势
- **时序分析**: 可以分析数据的时间分布和频率
- **性能监控**: 监控系统响应时间和数据更新频率
- **调试支持**: 便于定位和解决时序相关问题
- **数据记录**: 支持数据记录和回放功能 