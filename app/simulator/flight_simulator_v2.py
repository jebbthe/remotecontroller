import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import time
import threading
import serial
import serial.tools.list_ports
import re
from matplotlib import font_manager

class FlightSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Flight Attitude Simulator")
        self.root.geometry("1400x900")
        
        # 设置中文字体
        self.setup_chinese_font()
        
        # 串口相关
        self.serial_port = None
        self.serial_thread = None
        self.serial_running = False
        self.serial_data_queue = []
        self.last_serial_time = 0
        
        # 飞行数据
        self.pitch = 0.0  # 俯仰角 (度)
        self.roll = 0.0   # 横滚角 (度)
        self.yaw = 0.0    # 偏航角 (度)
        self.altitude = 100.0  # 高度 (米)
        self.speed = 15.0      # 速度 (m/s)
        
        # 舵面角度
        self.elevator_angle = 0.0  # 升降舵角度
        self.aileron_angle = 0.0   # 副翼角度
        self.rudder_angle = 0.0    # 方向舵角度
        
        # PID数据
        self.pitch_error = 0.0
        self.roll_error = 0.0
        self.pitch_integral = 0.0
        self.roll_integral = 0.0
        self.pitch_output = 0.0
        self.roll_output = 0.0
        self.right_target = 90.0
        self.left_target = 90.0
        
        # 自平衡模式
        self.stabilize_mode = True
        self.target_pitch = 0.0
        self.target_roll = 0.0
        
        # PID参数 (模拟纸飞机的参数)
        self.pid_gains = {
            'pitch': {'Kp': 1.0, 'Ki': 0.01, 'Kd': 0.3},
            'roll': {'Kp': 1.0, 'Ki': 0.01, 'Kd': 0.3}
        }
        
        self.pid_integral = {'pitch': 0.0, 'roll': 0.0}
        self.pid_prev_error = {'pitch': 0.0, 'roll': 0.0}
        self.last_time = time.time()
        
        self.setup_ui()
        self.setup_animation()
        
    def setup_chinese_font(self):
        """设置中文字体"""
        # 尝试设置中文字体
        chinese_fonts = ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei', 'DejaVu Sans']
        font_found = False
        
        for font_name in chinese_fonts:
            try:
                font_manager.findfont(font_name)
                plt.rcParams['font.sans-serif'] = [font_name]
                font_found = True
                break
            except:
                continue
        
        if not font_found:
            # 如果没有找到中文字体，使用默认字体
            plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
        
        plt.rcParams['axes.unicode_minus'] = False
        
    def setup_ui(self):
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 串口配置面板
        self.setup_serial_panel(main_frame)
        
        # 内容框架
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        
        # 左侧控制面板
        left_frame = ttk.Frame(content_frame, width=300)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # 控制面板标题
        ttk.Label(left_frame, text="Control Panel", font=("Arial", 14, "bold")).pack(pady=10)
        
        # 姿态控制
        attitude_frame = ttk.LabelFrame(left_frame, text="Attitude Control", padding=10)
        attitude_frame.pack(fill=tk.X, pady=5)
        
        # 俯仰角控制
        ttk.Label(attitude_frame, text="Pitch (deg):").pack(anchor=tk.W)
        self.pitch_var = tk.DoubleVar(value=self.pitch)
        pitch_scale = ttk.Scale(attitude_frame, from_=-45, to=45, variable=self.pitch_var, 
                               orient=tk.HORIZONTAL, command=self.update_pitch)
        pitch_scale.pack(fill=tk.X, pady=2)
        self.pitch_label = ttk.Label(attitude_frame, text=f"{self.pitch:.1f}°")
        self.pitch_label.pack(anchor=tk.W)
        
        # 横滚角控制
        ttk.Label(attitude_frame, text="Roll (deg):").pack(anchor=tk.W, pady=(10, 0))
        self.roll_var = tk.DoubleVar(value=self.roll)
        roll_scale = ttk.Scale(attitude_frame, from_=-45, to=45, variable=self.roll_var, 
                              orient=tk.HORIZONTAL, command=self.update_roll)
        roll_scale.pack(fill=tk.X, pady=2)
        self.roll_label = ttk.Label(attitude_frame, text=f"{self.roll:.1f}°")
        self.roll_label.pack(anchor=tk.W)
        
        # 偏航角控制
        ttk.Label(attitude_frame, text="Yaw (deg):").pack(anchor=tk.W, pady=(10, 0))
        self.yaw_var = tk.DoubleVar(value=self.yaw)
        yaw_scale = ttk.Scale(attitude_frame, from_=-180, to=180, variable=self.yaw_var, 
                             orient=tk.HORIZONTAL, command=self.update_yaw)
        yaw_scale.pack(fill=tk.X, pady=2)
        self.yaw_label = ttk.Label(attitude_frame, text=f"{self.yaw:.1f}°")
        self.yaw_label.pack(anchor=tk.W)
        
        # 自平衡模式控制
        balance_frame = ttk.LabelFrame(left_frame, text="Stabilize Mode", padding=10)
        balance_frame.pack(fill=tk.X, pady=5)
        
        self.stabilize_var = tk.BooleanVar(value=self.stabilize_mode)
        stabilize_check = ttk.Checkbutton(balance_frame, text="Enable Stabilize", 
                                         variable=self.stabilize_var, command=self.toggle_stabilize)
        stabilize_check.pack(anchor=tk.W)
        
        # 目标姿态设置
        ttk.Label(balance_frame, text="Target Pitch (deg):").pack(anchor=tk.W, pady=(10, 0))
        self.target_pitch_var = tk.DoubleVar(value=self.target_pitch)
        target_pitch_scale = ttk.Scale(balance_frame, from_=-30, to=30, variable=self.target_pitch_var, 
                                      orient=tk.HORIZONTAL, command=self.update_target_pitch)
        target_pitch_scale.pack(fill=tk.X, pady=2)
        
        ttk.Label(balance_frame, text="Target Roll (deg):").pack(anchor=tk.W, pady=(10, 0))
        self.target_roll_var = tk.DoubleVar(value=self.target_roll)
        target_roll_scale = ttk.Scale(balance_frame, from_=-30, to=30, variable=self.target_roll_var, 
                                     orient=tk.HORIZONTAL, command=self.update_target_roll)
        target_roll_scale.pack(fill=tk.X, pady=2)
        
        # 飞行数据显示
        data_frame = ttk.LabelFrame(left_frame, text="Flight Data", padding=10)
        data_frame.pack(fill=tk.X, pady=5)
        
        self.altitude_label = ttk.Label(data_frame, text=f"Altitude: {self.altitude:.1f} m")
        self.altitude_label.pack(anchor=tk.W)
        
        self.speed_label = ttk.Label(data_frame, text=f"Speed: {self.speed:.1f} m/s")
        self.speed_label.pack(anchor=tk.W)
        
        # PID数据显示
        pid_frame = ttk.LabelFrame(left_frame, text="PID Data", padding=10)
        pid_frame.pack(fill=tk.X, pady=5)
        
        self.pitch_error_label = ttk.Label(pid_frame, text=f"Pitch Error: {self.pitch_error:.2f}")
        self.pitch_error_label.pack(anchor=tk.W)
        
        self.roll_error_label = ttk.Label(pid_frame, text=f"Roll Error: {self.roll_error:.2f}")
        self.roll_error_label.pack(anchor=tk.W)
        
        self.pitch_integral_label = ttk.Label(pid_frame, text=f"Pitch Integral: {self.pitch_integral:.2f}")
        self.pitch_integral_label.pack(anchor=tk.W)
        
        self.roll_integral_label = ttk.Label(pid_frame, text=f"Roll Integral: {self.roll_integral:.2f}")
        self.roll_integral_label.pack(anchor=tk.W)
        
        self.pitch_output_label = ttk.Label(pid_frame, text=f"Pitch Output: {self.pitch_output:.2f}")
        self.pitch_output_label.pack(anchor=tk.W)
        
        self.roll_output_label = ttk.Label(pid_frame, text=f"Roll Output: {self.roll_output:.2f}")
        self.roll_output_label.pack(anchor=tk.W)
        
        # 舵面角度显示
        control_frame = ttk.LabelFrame(left_frame, text="Control Surfaces", padding=10)
        control_frame.pack(fill=tk.X, pady=5)
        
        self.elevator_label = ttk.Label(control_frame, text=f"Elevator: {self.elevator_angle:.1f}°")
        self.elevator_label.pack(anchor=tk.W)
        
        self.aileron_label = ttk.Label(control_frame, text=f"Aileron: {self.aileron_angle:.1f}°")
        self.aileron_label.pack(anchor=tk.W)
        
        self.right_target_label = ttk.Label(control_frame, text=f"Right Servo: {self.right_target:.1f}°")
        self.right_target_label.pack(anchor=tk.W)
        
        self.left_target_label = ttk.Label(control_frame, text=f"Left Servo: {self.left_target:.1f}°")
        self.left_target_label.pack(anchor=tk.W)
        
        # 右侧显示区域
        right_frame = ttk.Frame(content_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # 创建matplotlib图形
        self.fig = plt.Figure(figsize=(12, 8))
        
        # 左侧姿态仪
        self.attitude_ax = self.fig.add_subplot(121, projection='3d')
        self.attitude_ax.set_title("Flight Attitude Indicator", fontsize=12)
        self.attitude_ax.set_xlabel('X')
        self.attitude_ax.set_ylabel('Y')
        self.attitude_ax.set_zlabel('Z')
        self.attitude_ax.set_xlim(-1, 1)
        self.attitude_ax.set_ylim(-1, 1)
        self.attitude_ax.set_zlim(-1, 1)
        
        # 右侧纸飞机
        self.plane_ax = self.fig.add_subplot(122)
        self.plane_ax.set_title("Paper Plane View (Tail View)", fontsize=12)
        self.plane_ax.set_xlim(-2, 2)
        self.plane_ax.set_ylim(-2, 2)
        self.plane_ax.set_aspect('equal')
        self.plane_ax.grid(True)
        
        # 嵌入到tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def setup_serial_panel(self, parent):
        """设置串口配置面板"""
        serial_frame = ttk.LabelFrame(parent, text="Serial Port Configuration", padding=10)
        serial_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 串口选择
        port_frame = ttk.Frame(serial_frame)
        port_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side=tk.LEFT, padx=(5, 10))
        
        # 波特率选择
        ttk.Label(port_frame, text="Baudrate:").pack(side=tk.LEFT)
        self.baudrate_var = tk.StringVar(value="9600")
        baudrate_combo = ttk.Combobox(port_frame, textvariable=self.baudrate_var, 
                                     values=["9600", "19200", "38400", "57600", "115200"], width=10)
        baudrate_combo.pack(side=tk.LEFT, padx=(5, 10))
        
        # 刷新按钮
        refresh_btn = ttk.Button(port_frame, text="Refresh Ports", command=self.refresh_ports)
        refresh_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # 连接/断开按钮
        self.connect_btn = ttk.Button(port_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT)
        
        # 状态显示
        self.status_label = ttk.Label(serial_frame, text="Status: Disconnected", foreground="red")
        self.status_label.pack(anchor=tk.W)
        
        # 初始化串口列表
        self.refresh_ports()
        
    def refresh_ports(self):
        """刷新可用串口列表"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
            
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.serial_port is None:
            self.connect_serial()
        else:
            self.disconnect_serial()
            
    def connect_serial(self):
        """连接串口"""
        try:
            port = self.port_var.get()
            baudrate = int(self.baudrate_var.get())
            
            if not port:
                messagebox.showerror("Error", "Please select a port")
                return
                
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.serial_running = True
            self.serial_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
            self.serial_thread.start()
            
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="Status: Connected", foreground="green")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {str(e)}")
            
    def disconnect_serial(self):
        """断开串口连接"""
        if self.serial_port:
            self.serial_running = False
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Status: Disconnected", foreground="red")
            
    def serial_read_thread(self):
        """串口读取线程"""
        while self.serial_running and self.serial_port:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.parse_serial_data(line)
            except Exception as e:
                print(f"Serial read error: {e}")
                break
                
    def parse_serial_data(self, line):
        """解析串口数据"""
        try:
            # 解析PID输出数据格式
            # 示例: "Pitch Error: 2.50 Roll Error: -1.20 Pitch Integral: 5.30 Roll Integral: -2.10 Pitch Output: 8.20 Roll Output: -4.50 Right Target: 95.20 Left Target: 84.80"
            
            # 提取数值
            patterns = {
                'pitch_error': r'Pitch Error: ([-\d.]+)',
                'roll_error': r'Roll Error: ([-\d.]+)',
                'pitch_integral': r'Pitch Integral: ([-\d.]+)',
                'roll_integral': r'Roll Integral: ([-\d.]+)',
                'pitch_output': r'Pitch Output: ([-\d.]+)',
                'roll_output': r'Roll Output: ([-\d.]+)',
                'right_target': r'Right Target: ([-\d.]+)',
                'left_target': r'Left Target: ([-\d.]+)'
            }
            
            data = {}
            for key, pattern in patterns.items():
                match = re.search(pattern, line)
                if match:
                    data[key] = float(match.group(1))
            
            # 将数据添加到队列
            if data:
                self.serial_data_queue.append(data)
                
        except Exception as e:
            print(f"Parse error: {e}")
            
    def update_pitch(self, value):
        self.pitch = float(value)
        self.pitch_label.config(text=f"{self.pitch:.1f}°")
        
    def update_roll(self, value):
        self.roll = float(value)
        self.roll_label.config(text=f"{self.roll:.1f}°")
        
    def update_yaw(self, value):
        self.yaw = float(value)
        self.yaw_label.config(text=f"{self.yaw:.1f}°")
        
    def update_target_pitch(self, value):
        self.target_pitch = float(value)
        
    def update_target_roll(self, value):
        self.target_roll = float(value)
        
    def toggle_stabilize(self):
        self.stabilize_mode = self.stabilize_var.get()
        if self.stabilize_mode:
            # 重置PID积分项
            self.pid_integral = {'pitch': 0.0, 'roll': 0.0}
            self.pid_prev_error = {'pitch': 0.0, 'roll': 0.0}
            self.last_time = time.time()
    
    def calculate_pid_output(self, current, target, axis):
        """计算PID输出"""
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
            
        error = target - current
        
        # 死区
        deadband = 1.0
        if abs(error) < deadband:
            error = 0.0
            self.pid_integral[axis] *= 0.95  # 积分衰减
        
        # PID计算
        Kp = self.pid_gains[axis]['Kp']
        Ki = self.pid_gains[axis]['Ki']
        Kd = self.pid_gains[axis]['Kd']
        
        # 比例项
        proportional = Kp * error
        
        # 积分项
        self.pid_integral[axis] += error * dt
        integral = Ki * self.pid_integral[axis]
        
        # 微分项
        derivative = Kd * (error - self.pid_prev_error[axis]) / dt
        self.pid_prev_error[axis] = error
        
        # 输出限制
        output = proportional + integral + derivative
        output = max(-30, min(30, output))  # 限制舵面角度
        
        self.last_time = current_time
        return output
    
    def draw_attitude_indicator(self):
        """绘制3D姿态仪"""
        self.attitude_ax.clear()
        self.attitude_ax.set_title("Flight Attitude Indicator", fontsize=12)
        self.attitude_ax.set_xlabel('X')
        self.attitude_ax.set_ylabel('Y')
        self.attitude_ax.set_zlabel('Z')
        self.attitude_ax.set_xlim(-1, 1)
        self.attitude_ax.set_ylim(-1, 1)
        self.attitude_ax.set_zlim(-1, 1)
        
        # 转换角度为弧度
        pitch_rad = math.radians(self.pitch)
        roll_rad = math.radians(self.roll)
        yaw_rad = math.radians(self.yaw)
        
        # 创建飞机坐标系
        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])
        z = np.array([0, 0, 1])
        
        # 应用旋转矩阵
        # 绕X轴旋转 (横滚)
        Rx = np.array([[1, 0, 0],
                      [0, math.cos(roll_rad), -math.sin(roll_rad)],
                      [0, math.sin(roll_rad), math.cos(roll_rad)]])
        
        # 绕Y轴旋转 (俯仰)
        Ry = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)],
                      [0, 1, 0],
                      [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])
        
        # 绕Z轴旋转 (偏航)
        Rz = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0],
                      [math.sin(yaw_rad), math.cos(yaw_rad), 0],
                      [0, 0, 1]])
        
        # 组合旋转
        R = Rz @ Ry @ Rx
        x_rot = R @ x
        y_rot = R @ y
        z_rot = R @ z
        
        # 绘制坐标轴
        self.attitude_ax.quiver(0, 0, 0, x_rot[0], x_rot[1], x_rot[2], 
                               color='red', arrow_length_ratio=0.2, label='X (Nose)')
        self.attitude_ax.quiver(0, 0, 0, y_rot[0], y_rot[1], y_rot[2], 
                               color='green', arrow_length_ratio=0.2, label='Y (Right Wing)')
        self.attitude_ax.quiver(0, 0, 0, z_rot[0], z_rot[1], z_rot[2], 
                               color='blue', arrow_length_ratio=0.2, label='Z (Up)')
        
        # 绘制水平面参考
        xx, yy = np.meshgrid(np.linspace(-1, 1, 10), np.linspace(-1, 1, 10))
        zz = np.zeros_like(xx)
        self.attitude_ax.plot_surface(xx, yy, zz, alpha=0.3, color='gray')
        
        # 绘制飞机轮廓
        plane_x = np.array([0.8, 0, -0.2, -0.2, 0])
        plane_y = np.array([0, 0.3, 0.2, -0.2, -0.3])
        plane_z = np.array([0, 0, 0, 0, 0])
        
        # 应用旋转到飞机轮廓
        plane_points = np.column_stack([plane_x, plane_y, plane_z])
        plane_rotated = (R @ plane_points.T).T
        
        self.attitude_ax.plot(plane_rotated[:, 0], plane_rotated[:, 1], plane_rotated[:, 2], 
                             'k-', linewidth=2, label='Aircraft Outline')
        
        self.attitude_ax.legend()
        self.attitude_ax.view_init(elev=20, azim=45)
        
    def draw_paper_plane(self):
        """绘制纸飞机 (尾部视角) - 只有左右两个舵面"""
        self.plane_ax.clear()
        self.plane_ax.set_title("Paper Plane View (Tail View)", fontsize=12)
        self.plane_ax.set_xlim(-2, 2)
        self.plane_ax.set_ylim(-2, 2)
        self.plane_ax.set_aspect('equal')
        self.plane_ax.grid(True)
        
        # 纸飞机主体 (从尾部看)
        # 机身 - 三角形
        body_points = np.array([
            [1.5, 0],    # 机头
            [0, 0.1],    # 右后角
            [0, -0.1],   # 左后角
        ])
        self.plane_ax.fill(body_points[:, 0], body_points[:, 1], color='lightblue', alpha=0.8)
        
        # 主翼 - 三角形
        wing_points = np.array([
            [0.2, 0.8],   # 右翼尖
            [0, 0.1],     # 右翼根
            [0, -0.1],    # 左翼根
            [0.2, -0.8],  # 左翼尖
        ])
        self.plane_ax.fill(wing_points[:, 0], wing_points[:, 1], color='white', alpha=0.9)
        
        # 尾翼 - 小三角形
        tail_points = np.array([
            [-0.1, 0.3],   # 右尾翼
            [-0.3, 0.05],  # 右尾翼根
            [-0.3, -0.05], # 左尾翼根
            [-0.1, -0.3],  # 左尾翼
        ])
        self.plane_ax.fill(tail_points[:, 0], tail_points[:, 1], color='lightgray', alpha=0.8)
        
        # 左右舵面 (混合控制)
        # 右舵面
        right_servo_angle_rad = math.radians(self.right_target - 90)
        right_servo_length = 0.4
        right_servo_x = [-0.3, -0.3 - right_servo_length * math.cos(right_servo_angle_rad)]
        right_servo_y = [0.05, 0.05 + right_servo_length * math.sin(right_servo_angle_rad)]
        self.plane_ax.plot(right_servo_x, right_servo_y, 'r-', linewidth=3, label='Right Servo')
        
        # 左舵面
        left_servo_angle_rad = math.radians(self.left_target - 90)
        left_servo_length = 0.4
        left_servo_x = [-0.3, -0.3 - left_servo_length * math.cos(left_servo_angle_rad)]
        left_servo_y = [-0.05, -0.05 + left_servo_length * math.sin(left_servo_angle_rad)]
        self.plane_ax.plot(left_servo_x, left_servo_y, 'g-', linewidth=3, label='Left Servo')
        
        # 添加标注
        self.plane_ax.text(0, -1.5, f"Right Servo: {self.right_target:.1f}°", ha='center', fontsize=10, color='red')
        self.plane_ax.text(0, -1.7, f"Left Servo: {self.left_target:.1f}°", ha='center', fontsize=10, color='green')
        self.plane_ax.text(0, -1.9, f"Pitch: {self.pitch:.1f}° Roll: {self.roll:.1f}°", ha='center', fontsize=10)
        
        # 添加坐标轴指示
        self.plane_ax.arrow(0, 0, 0.5, 0, head_width=0.05, head_length=0.1, fc='red', ec='red')
        self.plane_ax.text(0.6, 0, 'Nose', fontsize=10, color='red')
        
        self.plane_ax.legend()
        
    def update_flight_data(self):
        """更新飞行数据"""
        # 处理串口数据 (每2秒回放一次)
        current_time = time.time()
        if self.serial_data_queue and (current_time - self.last_serial_time) > 2.0:
            data = self.serial_data_queue.pop(0)
            self.last_serial_time = current_time
            
            # 更新PID数据
            if 'pitch_error' in data:
                self.pitch_error = data['pitch_error']
            if 'roll_error' in data:
                self.roll_error = data['roll_error']
            if 'pitch_integral' in data:
                self.pitch_integral = data['pitch_integral']
            if 'roll_integral' in data:
                self.roll_integral = data['roll_integral']
            if 'pitch_output' in data:
                self.pitch_output = data['pitch_output']
            if 'roll_output' in data:
                self.roll_output = data['roll_output']
            if 'right_target' in data:
                self.right_target = data['right_target']
            if 'left_target' in data:
                self.left_target = data['left_target']
                
            # 根据舵机位置反推姿态
            # 简化的反推模型
            pitch_diff = (self.right_target + self.left_target - 180) / 2
            roll_diff = (self.right_target - self.left_target) / 2
            
            self.pitch = pitch_diff * 0.5  # 缩放因子
            self.roll = roll_diff * 0.5
            
            # 更新滑块
            self.pitch_var.set(self.pitch)
            self.roll_var.set(self.roll)
            self.pitch_label.config(text=f"{self.pitch:.1f}°")
            self.roll_label.config(text=f"{self.roll:.1f}°")
        
        if self.stabilize_mode:
            # 自平衡模式 - 使用PID控制
            self.elevator_angle = self.calculate_pid_output(self.pitch, self.target_pitch, 'pitch')
            self.aileron_angle = self.calculate_pid_output(self.roll, self.target_roll, 'roll')
            self.rudder_angle = 0  # 简化，方向舵保持中立
            
            # 模拟飞机响应
            pitch_response = self.elevator_angle * 0.1  # 简化响应模型
            roll_response = self.aileron_angle * 0.1
            
            # 更新实际姿态 (模拟飞机运动)
            self.pitch += pitch_response
            self.roll += roll_response
            
            # 限制姿态角度
            self.pitch = max(-45, min(45, self.pitch))
            self.roll = max(-45, min(45, self.roll))
            
            # 更新滑块和标签
            self.pitch_var.set(self.pitch)
            self.roll_var.set(self.roll)
            self.pitch_label.config(text=f"{self.pitch:.1f}°")
            self.roll_label.config(text=f"{self.roll:.1f}°")
            
            # 计算舵机目标位置 (混合控制)
            # 纸飞机：对称控制俯仰，差动控制横滚
            self.right_target = 90.0 + self.elevator_angle - self.aileron_angle
            self.left_target = 90.0 + self.elevator_angle + self.aileron_angle
            
            # 限制舵机范围
            self.right_target = max(45.0, min(135.0, self.right_target))
            self.left_target = max(45.0, min(135.0, self.left_target))
        
        # 更新显示标签
        self.altitude_label.config(text=f"Altitude: {self.altitude:.1f} m")
        self.speed_label.config(text=f"Speed: {self.speed:.1f} m/s")
        self.elevator_label.config(text=f"Elevator: {self.elevator_angle:.1f}°")
        self.aileron_label.config(text=f"Aileron: {self.aileron_angle:.1f}°")
        self.right_target_label.config(text=f"Right Servo: {self.right_target:.1f}°")
        self.left_target_label.config(text=f"Left Servo: {self.left_target:.1f}°")
        
        # 更新PID数据显示
        self.pitch_error_label.config(text=f"Pitch Error: {self.pitch_error:.2f}")
        self.roll_error_label.config(text=f"Roll Error: {self.roll_error:.2f}")
        self.pitch_integral_label.config(text=f"Pitch Integral: {self.pitch_integral:.2f}")
        self.roll_integral_label.config(text=f"Roll Integral: {self.roll_integral:.2f}")
        self.pitch_output_label.config(text=f"Pitch Output: {self.pitch_output:.2f}")
        self.roll_output_label.config(text=f"Roll Output: {self.roll_output:.2f}")
        
    def animate(self):
        """动画循环"""
        self.update_flight_data()
        self.draw_attitude_indicator()
        self.draw_paper_plane()
        self.canvas.draw()
        self.root.after(50, self.animate)  # 20 FPS
        
    def setup_animation(self):
        """设置动画"""
        self.animate()
        
    def on_closing(self):
        """程序关闭时的清理"""
        self.disconnect_serial()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = FlightSimulator(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main() 