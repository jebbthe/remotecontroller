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
        self.root.title("Flight Attitude Simulator V3.0")
        self.root.geometry("1400x900")
        
        # 设置中文字体
        self.setup_chinese_font()
        
        # 串口相关
        self.serial_port = None
        self.serial_thread = None
        self.serial_running = False
        self.serial_data_queue = []
        self.last_serial_time = 0
        
        # 飞行数据 - 只从串口获取
        self.pitch = 0.0  # 俯仰角 (度)
        self.roll = 0.0   # 横滚角 (度)
        self.yaw = 0.0    # 偏航角 (度)
        self.altitude = 100.0  # 高度 (米)
        self.speed = 15.0      # 速度 (m/s)
        
        # 舵面角度 - 从串口获取
        self.elevator_angle = 0.0  # 升降舵角度
        self.aileron_angle = 0.0   # 副翼角度
        self.rudder_angle = 0.0    # 方向舵角度
        
        # 控制误差数据 - 从串口获取
        self.pitch_error = 0.0
        self.roll_error = 0.0
        self.right_target = 90.0
        self.left_target = 90.0
        
        # 数据统计
        self.data_count = 0
        self.last_update_time = time.time()
        self.last_timestamp = 0
        
        # 串口统计
        self.bytes_received = 0
        self.lines_received = 0
        self.parse_success = 0
        self.total_bytes_received = 0  # 累计字节数
        self.total_lines_received = 0  # 累计行数
        self.total_parse_success = 0   # 累计解析成功数
        self.last_serial_stats_time = time.time()
        self.refresh_interval = 50  # 默认刷新间隔50ms
        self.last_timestamp_interval = 0  # 初始化时间戳间隔
        self.last_raw_data = "No data received"  # 最近接收的原始数据
        self.needs_redraw = False  # 是否需要重绘图形
        
        # 数据队列管理 - 限制队列大小，丢弃旧数据
        self.max_queue_size = 10  # 最大队列大小
        
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
        ttk.Label(left_frame, text="Serial Data Monitor", font=("Arial", 14, "bold")).pack(pady=10)
        

        
        # 姿态数据显示
        attitude_frame = ttk.LabelFrame(left_frame, text="Attitude Data", padding=10)
        attitude_frame.pack(fill=tk.X, pady=5)
        
        self.pitch_label = ttk.Label(attitude_frame, text=f"Pitch: {self.pitch:.1f}°")
        self.pitch_label.pack(anchor=tk.W)
        
        self.roll_label = ttk.Label(attitude_frame, text=f"Roll: {self.roll:.1f}°")
        self.roll_label.pack(anchor=tk.W)
        
        self.yaw_label = ttk.Label(attitude_frame, text=f"Yaw: {self.yaw:.1f}°")
        self.yaw_label.pack(anchor=tk.W)
        
        # 控制误差显示
        error_frame = ttk.LabelFrame(left_frame, text="Control Errors", padding=10)
        error_frame.pack(fill=tk.X, pady=5)
        
        self.pitch_error_label = ttk.Label(error_frame, text=f"Pitch Error: {self.pitch_error:.2f}")
        self.pitch_error_label.pack(anchor=tk.W)
        
        self.roll_error_label = ttk.Label(error_frame, text=f"Roll Error: {self.roll_error:.2f}")
        self.roll_error_label.pack(anchor=tk.W)
        
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
        
        # 数据统计
        stats_frame = ttk.LabelFrame(left_frame, text="Data Statistics", padding=10)
        stats_frame.pack(fill=tk.X, pady=5)
        
        self.data_count_label = ttk.Label(stats_frame, text=f"Data Count: {self.data_count}")
        self.data_count_label.pack(anchor=tk.W)
        
        self.update_rate_label = ttk.Label(stats_frame, text="Update Rate: 0 Hz")
        self.update_rate_label.pack(anchor=tk.W)
        
        self.timestamp_label = ttk.Label(stats_frame, text="Last Timestamp: 0ms")
        self.timestamp_label.pack(anchor=tk.W)
        
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
        
        # 串口数据统计
        stats_frame = ttk.Frame(serial_frame)
        stats_frame.pack(fill=tk.X, pady=(5, 0))
        
        # 左侧统计
        left_stats = ttk.Frame(stats_frame)
        left_stats.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.bytes_received_label = ttk.Label(left_stats, text="Bytes Received: 0")
        self.bytes_received_label.pack(anchor=tk.W)
        
        self.lines_received_label = ttk.Label(left_stats, text="Lines Received: 0")
        self.lines_received_label.pack(anchor=tk.W)
        
        self.parse_success_label = ttk.Label(left_stats, text="Parse Success: 0")
        self.parse_success_label.pack(anchor=tk.W)
        
        # 右侧统计
        right_stats = ttk.Frame(stats_frame)
        right_stats.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        
        self.data_rate_label = ttk.Label(right_stats, text="Data Rate: 0 B/s")
        self.data_rate_label.pack(anchor=tk.E)
        
        self.refresh_interval_label = ttk.Label(right_stats, text="Refresh Interval: 50ms")
        self.refresh_interval_label.pack(anchor=tk.E)
        
        self.last_timestamp_label = ttk.Label(right_stats, text="Last Timestamp: 0ms")
        self.last_timestamp_label.pack(anchor=tk.E)
        
        # 最近接收的原始数据
        raw_data_frame = ttk.Frame(serial_frame)
        raw_data_frame.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(raw_data_frame, text="Last Raw Data:").pack(anchor=tk.W)
        
        # 使用Text组件显示原始数据，支持换行和滚动
        self.raw_data_text = tk.Text(raw_data_frame, height=3, width=80, font=('Consolas', 8))
        self.raw_data_text.pack(fill=tk.X, pady=(2, 0))
        
        # 添加滚动条
        raw_data_scrollbar = ttk.Scrollbar(raw_data_frame, orient=tk.VERTICAL, command=self.raw_data_text.yview)
        raw_data_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.raw_data_text.configure(yscrollcommand=raw_data_scrollbar.set)
        
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
                        self.lines_received += 1
                        self.total_lines_received += 1
                        line_bytes = len(line.encode('utf-8'))
                        self.bytes_received += line_bytes
                        self.total_bytes_received += line_bytes
                        self.last_raw_data = line  # 保存原始数据
                        
                        # 限制队列大小，丢弃旧数据
                        if len(self.serial_data_queue) >= self.max_queue_size:
                            self.serial_data_queue.pop(0)  # 丢弃最旧的数据
                        
                        self.parse_serial_data(line)
            except Exception as e:
                print(f"Serial read error: {e}")
                break
                
    def parse_serial_data(self, line):
        """解析串口数据 - 适配新的输出格式"""
        try:
            # 解析新的数据格式
            # 示例: "[12345ms] Pitch Error: 2.50 Roll Error: -1.20 Current Pitch: 1.30 Current Roll: -0.80 Right Target: 95.20 Left Target: 84.80"
            
            # 提取时间戳
            timestamp_match = re.search(r'\[(\d+)ms\]', line)
            timestamp = int(timestamp_match.group(1)) if timestamp_match else 0
            
            # 提取数值 - 新的字段格式
            patterns = {
                'timestamp': r'\[(\d+)ms\]',
                'pitch_error': r'Pitch Error: ([-\d.]+)',
                'roll_error': r'Roll Error: ([-\d.]+)',
                'current_pitch': r'Current Pitch: ([-\d.]+)',
                'current_roll': r'Current Roll: ([-\d.]+)',
                'right_target': r'Right Target: ([-\d.]+)',
                'left_target': r'Left Target: ([-\d.]+)'
            }
            
            data = {}
            for key, pattern in patterns.items():
                match = re.search(pattern, line)
                if match:
                    data[key] = float(match.group(1))
            
            # 将数据添加到队列并立即更新显示
            if data:
                self.serial_data_queue.append(data)
                self.parse_success += 1
                self.total_parse_success += 1
                
                # 立即更新飞行数据（不重绘图形）
                self.update_flight_data_from_parsed(data)
                # 标记需要重绘
                self.needs_redraw = True
                
        except Exception as e:
            print(f"Parse error: {e}")
    
    def update_flight_data_from_parsed(self, data):
        """从解析的数据立即更新飞行数据 - 适配新格式"""
        # 更新时间戳并调整刷新间隔
        if 'timestamp' in data:
            self.last_timestamp = data['timestamp']
            # 基于时间戳动态调整刷新间隔
            self.adjust_refresh_interval(data['timestamp'])
        
        # 更新错误数据
        if 'pitch_error' in data:
            self.pitch_error = data['pitch_error']
        if 'roll_error' in data:
            self.roll_error = data['roll_error']
        
        # 更新当前姿态数据（直接使用传感器数据）
        if 'current_pitch' in data:
            self.pitch = data['current_pitch']
        if 'current_roll' in data:
            self.roll = data['current_roll']
        
        # 更新舵机目标位置
        if 'right_target' in data:
            self.right_target = data['right_target']
        if 'left_target' in data:
            self.left_target = data['left_target']
        
        # 计算舵面角度
        self.elevator_angle = (self.right_target + self.left_target - 180) / 2
        self.aileron_angle = (self.right_target - self.left_target) / 2
        
        # 立即更新显示标签
        self.update_display_labels()
    
    def update_display_labels(self):
        """更新所有显示标签"""
        # 更新姿态数据显示
        self.pitch_label.config(text=f"Pitch: {self.pitch:.1f}°")
        self.roll_label.config(text=f"Roll: {self.roll:.1f}°")
        self.yaw_label.config(text=f"Yaw: {self.yaw:.1f}°")
        
        # 更新控制面数据显示
        self.elevator_label.config(text=f"Elevator: {self.elevator_angle:.1f}°")
        self.aileron_label.config(text=f"Aileron: {self.aileron_angle:.1f}°")
        self.right_target_label.config(text=f"Right Servo: {self.right_target:.1f}°")
        self.left_target_label.config(text=f"Left Servo: {self.left_target:.1f}°")
        
        # 更新控制误差显示
        self.pitch_error_label.config(text=f"Pitch Error: {self.pitch_error:.2f}")
        self.roll_error_label.config(text=f"Roll Error: {self.roll_error:.2f}")
        
        # 更新数据计数和时间戳
        self.data_count_label.config(text=f"Data Count: {self.data_count}")
        self.timestamp_label.config(text=f"Last Timestamp: {self.last_timestamp}ms")
    
    def draw_attitude_indicator(self):
        """绘制3D姿态仪（修正Y轴方向与实际一致，roll为正时左翼上扬）"""
        self.attitude_ax.clear()
        self.attitude_ax.set_title("Flight Attitude Indicator", fontsize=12)
        self.attitude_ax.set_xlabel('X (Nose)')
        self.attitude_ax.set_ylabel('Y (Right Wing)')
        self.attitude_ax.set_zlabel('Z (Up)')
        self.attitude_ax.set_xlim(-1, 1)
        self.attitude_ax.set_ylim(-1, 1)
        self.attitude_ax.set_zlim(-1, 1)

        # 修正方向：roll为正时左翼上扬，pitch为正时机头上仰
        pitch_rad = -math.radians(self.pitch)
        roll_rad = math.radians(self.roll)  # 不取负号
        yaw_rad = math.radians(self.yaw)

        Rx = np.array([[1, 0, 0],
                      [0, math.cos(roll_rad), -math.sin(roll_rad)],
                      [0, math.sin(roll_rad), math.cos(roll_rad)]])
        Ry = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)],
                      [0, 1, 0],
                      [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])
        Rz = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0],
                      [math.sin(yaw_rad), math.cos(yaw_rad), 0],
                      [0, 0, 1]])
        R = Rz @ Ry @ Rx
        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])  # Y轴为右侧
        z = np.array([0, 0, 1])
        x_rot = R @ x
        y_rot = R @ y
        z_rot = R @ z
        self.attitude_ax.quiver(0, 0, 0, x_rot[0], x_rot[1], x_rot[2], color='red', arrow_length_ratio=0.2, label='X (Nose)')
        self.attitude_ax.quiver(0, 0, 0, y_rot[0], y_rot[1], y_rot[2], color='green', arrow_length_ratio=0.2, label='Y (Right Wing)')
        self.attitude_ax.quiver(0, 0, 0, z_rot[0], z_rot[1], z_rot[2], color='blue', arrow_length_ratio=0.2, label='Z (Up)')

        xx, yy = np.meshgrid(np.linspace(-1, 1, 10), np.linspace(-1, 1, 10))
        zz = np.zeros_like(xx)
        self.attitude_ax.plot_surface(xx, yy, zz, alpha=0.3, color='gray')

        outline = np.array([
            [ 0.9,  0.0, 0.0],   # 机头
            [-0.9,  0.6, 0.0],   # 左翼
            [-0.7,  0.0, 0.0],   # 机尾中
            [-0.9, -0.6, 0.0],   # 右翼
            [ 0.9,  0.0, 0.0],   # 回到机头
        ])
        outline_rot = (R @ outline.T).T
        self.attitude_ax.plot(outline_rot[:, 0], outline_rot[:, 1], outline_rot[:, 2], color='#8B0000', linewidth=2.5, label='Aircraft Outline')
        body_line = np.array([
            [0.9, 0, 0],
            [-0.7, 0, 0],
        ])
        body_rot = (R @ body_line.T).T
        self.attitude_ax.plot(body_rot[:, 0], body_rot[:, 1], body_rot[:, 2], color='#4B0082', linewidth=2)
        nose_rot = (R @ np.array([0.9, 0, 0]))
        self.attitude_ax.scatter(nose_rot[0], nose_rot[1], nose_rot[2], color='red', s=40, zorder=10)

        self.attitude_ax.legend()
        self.attitude_ax.view_init(elev=20, azim=45)

    def draw_paper_plane(self):
        """优化后的纸飞机水平视图（舵面以靠近机头侧为轴心旋转）"""
        self.plane_ax.clear()
        self.plane_ax.set_title("Paper Plane View (Horizontal)", fontsize=12)
        self.plane_ax.set_xlim(-1.2, 1.2)
        self.plane_ax.set_ylim(-0.8, 0.8)
        self.plane_ax.set_aspect('equal')
        self.plane_ax.grid(True, alpha=0.3)

        # 主翼三角形
        wing = np.array([
            [-1.0,  0.7],
            [ 1.0,   0.0],
            [-1.0, -0.7],
        ])
        self.plane_ax.fill(wing[:, 0], wing[:, 1], color='#F5F5DC', alpha=0.95, edgecolor='#8B4513', linewidth=2)

        # 机身
        body = np.array([
            [-1.0,  0.0],
            [ 1.0,  0.0],
        ])
        self.plane_ax.plot(body[:, 0], body[:, 1], color='#4B0082', linewidth=4, zorder=3)

        # 舵面参数
        elevon_length = 0.25
        elevon_width = 0.12
        elevon_x0 = -1.0 + elevon_length  # 舵面靠近机头侧的x

        # 右舵面（上方）
        right_angle = self.right_target - 90
        right_theta = np.radians(right_angle)
        right_base = np.array([elevon_x0, 0.35])
        # 以靠近机头侧为轴心，构造矩形并旋转
        right_rect = np.array([
            [0, -elevon_width/2],
            [-elevon_length, -elevon_width/2],
            [-elevon_length, elevon_width/2],
            [0, elevon_width/2],
        ])
        R = np.array([[np.cos(right_theta), -np.sin(right_theta)], [np.sin(right_theta), np.cos(right_theta)]])
        right_rect_rot = (R @ right_rect.T).T + right_base
        self.plane_ax.fill(right_rect_rot[:, 0], right_rect_rot[:, 1], color='#FF6B6B', alpha=0.8, edgecolor='#CC0000', linewidth=2, zorder=4)

        # 左舵面（下方，反向安装）
        left_angle = 90 - self.left_target
        left_theta = np.radians(left_angle)
        left_base = np.array([elevon_x0, -0.35])
        left_rect = np.array([
            [0, -elevon_width/2],
            [-elevon_length, -elevon_width/2],
            [-elevon_length, elevon_width/2],
            [0, elevon_width/2],
        ])
        Rl = np.array([[np.cos(left_theta), -np.sin(left_theta)], [np.sin(left_theta), np.cos(left_theta)]])
        left_rect_rot = (Rl @ left_rect.T).T + left_base
        self.plane_ax.fill(left_rect_rot[:, 0], left_rect_rot[:, 1], color='#4ECDC4', alpha=0.8, edgecolor='#008080', linewidth=2, zorder=4)

        # 机头标记
        self.plane_ax.plot([1.0, 1.08], [0, 0], color='red', linewidth=3, zorder=5)
        self.plane_ax.text(1.1, 0, 'Nose', fontsize=10, color='red', weight='bold', va='center')

        # 标注（数值显示区域下移，避免与机身重叠）
        self.plane_ax.text(0, 0.75, f"Right Servo: {self.right_target:.1f}°", ha='center', fontsize=9, color='#CC0000', weight='bold')
        self.plane_ax.text(0, -0.75, f"Left Servo: {self.left_target:.1f}°", ha='center', fontsize=9, color='#008080', weight='bold')
        self.plane_ax.text(0, -0.62, f"Pitch: {self.pitch:.1f}°  Roll: {self.roll:.1f}°", ha='center', fontsize=10, weight='bold', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2'))

        # 图例
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='#FF6B6B', alpha=0.8, label='Right Control Surface'),
            Patch(facecolor='#4ECDC4', alpha=0.8, label='Left Control Surface (Reversed)')
        ]
        self.plane_ax.legend(handles=legend_elements, loc='upper right', fontsize=8)
        
    def update_flight_data(self):
        """更新飞行数据 - 简化版本，主要用于统计更新"""
        # 处理队列中的剩余数据（如果有的话）
        current_time = time.time()
        if self.serial_data_queue:
            # 清空队列，因为数据已经在解析时处理了
            self.serial_data_queue.clear()
            self.last_serial_time = current_time
            self.data_count += 1
        
        # 计算更新频率
        if current_time - self.last_update_time > 1.0:  # 每秒更新一次
            update_rate = self.data_count / (current_time - self.last_update_time)
            self.update_rate_label.config(text=f"Update Rate: {update_rate:.1f} Hz")
            self.data_count = 0
            self.last_update_time = current_time
        
        # 更新串口统计
        self.update_serial_stats()
        
    def adjust_refresh_interval(self, timestamp):
        """基于时间戳动态调整刷新间隔"""
        # 计算时间戳间隔
        if hasattr(self, 'last_timestamp_interval'):
            interval = timestamp - self.last_timestamp_interval
            # 根据数据更新频率调整刷新间隔
            if interval > 0:
                # 如果数据更新频率高，减少刷新间隔
                if interval < 50:  # 小于50ms
                    self.refresh_interval = max(20, int(interval))  # 最小20ms，确保是整数
                elif interval < 100:  # 50-100ms
                    self.refresh_interval = int(interval)
                else:  # 大于100ms
                    self.refresh_interval = min(100, int(interval))  # 最大100ms，确保是整数
        self.last_timestamp_interval = timestamp
        
    def update_serial_stats(self):
        """更新串口统计信息"""
        current_time = time.time()
        
        # 每秒更新一次统计信息
        if current_time - self.last_serial_stats_time > 1.0:
            # 计算数据速率
            data_rate = self.bytes_received / (current_time - self.last_serial_stats_time)
            
            # 更新显示 - 显示累计数据
            self.bytes_received_label.config(text=f"Bytes Received: {self.total_bytes_received}")
            self.lines_received_label.config(text=f"Lines Received: {self.total_lines_received}")
            self.parse_success_label.config(text=f"Parse Success: {self.total_parse_success}")
            self.data_rate_label.config(text=f"Data Rate: {data_rate:.1f} B/s")
            self.refresh_interval_label.config(text=f"Refresh Interval: {self.refresh_interval}ms")
            self.last_timestamp_label.config(text=f"Last Timestamp: {self.last_timestamp}ms")
            
            # 更新原始数据显示
            self.raw_data_text.delete(1.0, tk.END)
            self.raw_data_text.insert(tk.END, self.last_raw_data)
            
            # 重置周期统计（保留累计统计）
            self.bytes_received = 0
            self.lines_received = 0
            self.parse_success = 0
            self.last_serial_stats_time = current_time
        
    def animate(self):
        """动画循环 - 智能重绘版本"""
        self.update_flight_data()
        
        # 智能重绘：只在需要时重绘图形
        if self.needs_redraw:
            self.draw_attitude_indicator()
            self.draw_paper_plane()
            self.canvas.draw()
            self.needs_redraw = False
        
        self.root.after(self.refresh_interval, self.animate)  # 动态刷新间隔
        
    def setup_animation(self):
        """设置动画"""
        # 初始化显示
        self.update_display_labels()
        self.draw_attitude_indicator()
        self.draw_paper_plane()
        self.canvas.draw()
        # 启动动画循环
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