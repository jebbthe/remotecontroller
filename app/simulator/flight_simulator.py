import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle, Rectangle, Polygon
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import time
import threading

class FlightSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("航模飞行姿态模拟器")
        self.root.geometry("1400x800")
        
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
        
        # 自平衡模式
        self.stabilize_mode = True
        self.target_pitch = 0.0
        self.target_roll = 0.0
        
        # PID参数 (模拟纸飞机的参数)
        self.pid_gains = {
            'pitch': {'Kp': 2.0, 'Ki': 0.1, 'Kd': 0.5},
            'roll': {'Kp': 2.0, 'Ki': 0.1, 'Kd': 0.5}
        }
        
        self.pid_integral = {'pitch': 0.0, 'roll': 0.0}
        self.pid_prev_error = {'pitch': 0.0, 'roll': 0.0}
        self.last_time = time.time()
        
        self.setup_ui()
        self.setup_animation()
        
    def setup_ui(self):
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 左侧控制面板
        left_frame = ttk.Frame(main_frame, width=300)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # 控制面板标题
        ttk.Label(left_frame, text="控制面板", font=("Arial", 14, "bold")).pack(pady=10)
        
        # 姿态控制
        attitude_frame = ttk.LabelFrame(left_frame, text="姿态控制", padding=10)
        attitude_frame.pack(fill=tk.X, pady=5)
        
        # 俯仰角控制
        ttk.Label(attitude_frame, text="俯仰角 (度):").pack(anchor=tk.W)
        self.pitch_var = tk.DoubleVar(value=self.pitch)
        pitch_scale = ttk.Scale(attitude_frame, from_=-45, to=45, variable=self.pitch_var, 
                               orient=tk.HORIZONTAL, command=self.update_pitch)
        pitch_scale.pack(fill=tk.X, pady=2)
        self.pitch_label = ttk.Label(attitude_frame, text=f"{self.pitch:.1f}°")
        self.pitch_label.pack(anchor=tk.W)
        
        # 横滚角控制
        ttk.Label(attitude_frame, text="横滚角 (度):").pack(anchor=tk.W, pady=(10, 0))
        self.roll_var = tk.DoubleVar(value=self.roll)
        roll_scale = ttk.Scale(attitude_frame, from_=-45, to=45, variable=self.roll_var, 
                              orient=tk.HORIZONTAL, command=self.update_roll)
        roll_scale.pack(fill=tk.X, pady=2)
        self.roll_label = ttk.Label(attitude_frame, text=f"{self.roll:.1f}°")
        self.roll_label.pack(anchor=tk.W)
        
        # 偏航角控制
        ttk.Label(attitude_frame, text="偏航角 (度):").pack(anchor=tk.W, pady=(10, 0))
        self.yaw_var = tk.DoubleVar(value=self.yaw)
        yaw_scale = ttk.Scale(attitude_frame, from_=-180, to=180, variable=self.yaw_var, 
                             orient=tk.HORIZONTAL, command=self.update_yaw)
        yaw_scale.pack(fill=tk.X, pady=2)
        self.yaw_label = ttk.Label(attitude_frame, text=f"{self.yaw:.1f}°")
        self.yaw_label.pack(anchor=tk.W)
        
        # 自平衡模式控制
        balance_frame = ttk.LabelFrame(left_frame, text="自平衡模式", padding=10)
        balance_frame.pack(fill=tk.X, pady=5)
        
        self.stabilize_var = tk.BooleanVar(value=self.stabilize_mode)
        stabilize_check = ttk.Checkbutton(balance_frame, text="启用自平衡", 
                                         variable=self.stabilize_var, command=self.toggle_stabilize)
        stabilize_check.pack(anchor=tk.W)
        
        # 目标姿态设置
        ttk.Label(balance_frame, text="目标俯仰角 (度):").pack(anchor=tk.W, pady=(10, 0))
        self.target_pitch_var = tk.DoubleVar(value=self.target_pitch)
        target_pitch_scale = ttk.Scale(balance_frame, from_=-30, to=30, variable=self.target_pitch_var, 
                                      orient=tk.HORIZONTAL, command=self.update_target_pitch)
        target_pitch_scale.pack(fill=tk.X, pady=2)
        
        ttk.Label(balance_frame, text="目标横滚角 (度):").pack(anchor=tk.W, pady=(10, 0))
        self.target_roll_var = tk.DoubleVar(value=self.target_roll)
        target_roll_scale = ttk.Scale(balance_frame, from_=-30, to=30, variable=self.target_roll_var, 
                                     orient=tk.HORIZONTAL, command=self.update_target_roll)
        target_roll_scale.pack(fill=tk.X, pady=2)
        
        # 飞行数据显示
        data_frame = ttk.LabelFrame(left_frame, text="飞行数据", padding=10)
        data_frame.pack(fill=tk.X, pady=5)
        
        self.altitude_label = ttk.Label(data_frame, text=f"高度: {self.altitude:.1f} m")
        self.altitude_label.pack(anchor=tk.W)
        
        self.speed_label = ttk.Label(data_frame, text=f"速度: {self.speed:.1f} m/s")
        self.speed_label.pack(anchor=tk.W)
        
        # 舵面角度显示
        control_frame = ttk.LabelFrame(left_frame, text="舵面角度", padding=10)
        control_frame.pack(fill=tk.X, pady=5)
        
        self.elevator_label = ttk.Label(control_frame, text=f"升降舵: {self.elevator_angle:.1f}°")
        self.elevator_label.pack(anchor=tk.W)
        
        self.aileron_label = ttk.Label(control_frame, text=f"副翼: {self.aileron_angle:.1f}°")
        self.aileron_label.pack(anchor=tk.W)
        
        self.rudder_label = ttk.Label(control_frame, text=f"方向舵: {self.rudder_angle:.1f}°")
        self.rudder_label.pack(anchor=tk.W)
        
        # 右侧显示区域
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # 创建matplotlib图形
        self.fig = plt.Figure(figsize=(12, 8))
        
        # 左侧姿态仪
        self.attitude_ax = self.fig.add_subplot(121, projection='3d')
        self.attitude_ax.set_title("飞行姿态仪", fontsize=12)
        self.attitude_ax.set_xlabel('X')
        self.attitude_ax.set_ylabel('Y')
        self.attitude_ax.set_zlabel('Z')
        self.attitude_ax.set_xlim(-1, 1)
        self.attitude_ax.set_ylim(-1, 1)
        self.attitude_ax.set_zlim(-1, 1)
        
        # 右侧纸飞机
        self.plane_ax = self.fig.add_subplot(122)
        self.plane_ax.set_title("纸飞机视图 (尾部视角)", fontsize=12)
        self.plane_ax.set_xlim(-2, 2)
        self.plane_ax.set_ylim(-2, 2)
        self.plane_ax.set_aspect('equal')
        self.plane_ax.grid(True)
        
        # 嵌入到tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
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
        self.attitude_ax.set_title("飞行姿态仪", fontsize=12)
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
                               color='red', arrow_length_ratio=0.2, label='X (机头)')
        self.attitude_ax.quiver(0, 0, 0, y_rot[0], y_rot[1], y_rot[2], 
                               color='green', arrow_length_ratio=0.2, label='Y (右翼)')
        self.attitude_ax.quiver(0, 0, 0, z_rot[0], z_rot[1], z_rot[2], 
                               color='blue', arrow_length_ratio=0.2, label='Z (上)')
        
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
                             'k-', linewidth=2, label='飞机轮廓')
        
        self.attitude_ax.legend()
        self.attitude_ax.view_init(elev=20, azim=45)
        
    def draw_paper_plane(self):
        """绘制纸飞机 (尾部视角)"""
        self.plane_ax.clear()
        self.plane_ax.set_title("纸飞机视图 (尾部视角)", fontsize=12)
        self.plane_ax.set_xlim(-2, 2)
        self.plane_ax.set_ylim(-2, 2)
        self.plane_ax.set_aspect('equal')
        self.plane_ax.grid(True)
        
        # 飞机主体 (从尾部看)
        body_length = 1.5
        body_width = 0.1
        
        # 机身
        body_x = [-body_length/2, body_length/2, body_length/2, -body_length/2]
        body_y = [-body_width/2, -body_width/2, body_width/2, body_width/2]
        self.plane_ax.fill(body_x, body_y, color='lightblue', alpha=0.8)
        
        # 机翼
        wing_span = 1.8
        wing_chord = 0.4
        wing_x = [-wing_chord/2, wing_chord/2, wing_chord/2, -wing_chord/2]
        wing_y = [-wing_span/2, -wing_span/2, wing_span/2, wing_span/2]
        self.plane_ax.fill(wing_x, wing_y, color='white', alpha=0.9)
        
        # 尾翼
        tail_span = 0.6
        tail_chord = 0.2
        tail_x = [-body_length/2 - tail_chord, -body_length/2, -body_length/2, -body_length/2 - tail_chord]
        tail_y = [-tail_span/2, -tail_span/2, tail_span/2, tail_span/2]
        self.plane_ax.fill(tail_x, tail_y, color='lightgray', alpha=0.8)
        
        # 升降舵 (水平尾翼)
        elevator_span = 0.4
        elevator_chord = 0.15
        elevator_angle_rad = math.radians(self.elevator_angle)
        
        # 升降舵位置 (在尾翼后面)
        elevator_x_base = -body_length/2 - tail_chord
        elevator_y_base = 0
        
        # 升降舵轮廓
        elevator_x = [elevator_x_base, elevator_x_base - elevator_chord * math.cos(elevator_angle_rad),
                     elevator_x_base - elevator_chord * math.cos(elevator_angle_rad), elevator_x_base]
        elevator_y = [-elevator_span/2, -elevator_span/2 + elevator_chord * math.sin(elevator_angle_rad),
                     elevator_span/2 + elevator_chord * math.sin(elevator_angle_rad), elevator_span/2]
        
        self.plane_ax.fill(elevator_x, elevator_y, color='red', alpha=0.7)
        
        # 副翼 (在机翼上)
        aileron_span = 0.3
        aileron_chord = 0.1
        aileron_angle_rad = math.radians(self.aileron_angle)
        
        # 左副翼
        left_aileron_x = [-aileron_chord/2, aileron_chord/2, aileron_chord/2, -aileron_chord/2]
        left_aileron_y = [-wing_span/2, -wing_span/2, -wing_span/2 + aileron_span, -wing_span/2 + aileron_span]
        # 应用副翼角度
        left_aileron_y = [y + aileron_chord/2 * math.sin(aileron_angle_rad) for y in left_aileron_y]
        self.plane_ax.fill(left_aileron_x, left_aileron_y, color='orange', alpha=0.7)
        
        # 右副翼 (相反方向)
        right_aileron_x = [-aileron_chord/2, aileron_chord/2, aileron_chord/2, -aileron_chord/2]
        right_aileron_y = [wing_span/2 - aileron_span, wing_span/2 - aileron_span, wing_span/2, wing_span/2]
        # 应用副翼角度 (相反方向)
        right_aileron_y = [y - aileron_chord/2 * math.sin(aileron_angle_rad) for y in right_aileron_y]
        self.plane_ax.fill(right_aileron_x, right_aileron_y, color='orange', alpha=0.7)
        
        # 方向舵 (垂直尾翼)
        rudder_height = 0.4
        rudder_chord = 0.15
        rudder_angle_rad = math.radians(self.rudder_angle)
        
        # 方向舵位置
        rudder_x_base = -body_length/2 - tail_chord
        rudder_y_base = 0
        
        # 方向舵轮廓
        rudder_x = [rudder_x_base, rudder_x_base - rudder_chord * math.cos(rudder_angle_rad),
                   rudder_x_base - rudder_chord * math.cos(rudder_angle_rad), rudder_x_base]
        rudder_y = [0, rudder_chord * math.sin(rudder_angle_rad),
                   rudder_height + rudder_chord * math.sin(rudder_angle_rad), rudder_height]
        
        self.plane_ax.fill(rudder_x, rudder_y, color='green', alpha=0.7)
        
        # 添加标注
        self.plane_ax.text(0, -1.5, f"升降舵: {self.elevator_angle:.1f}°", ha='center', fontsize=10)
        self.plane_ax.text(0, -1.7, f"副翼: {self.aileron_angle:.1f}°", ha='center', fontsize=10)
        self.plane_ax.text(0, -1.9, f"方向舵: {self.rudder_angle:.1f}°", ha='center', fontsize=10)
        
        # 添加坐标轴指示
        self.plane_ax.arrow(0, 0, 0.5, 0, head_width=0.05, head_length=0.1, fc='red', ec='red', label='机头方向')
        self.plane_ax.text(0.6, 0, '机头', fontsize=10, color='red')
        
    def update_flight_data(self):
        """更新飞行数据"""
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
        
        # 更新显示标签
        self.altitude_label.config(text=f"高度: {self.altitude:.1f} m")
        self.speed_label.config(text=f"速度: {self.speed:.1f} m/s")
        self.elevator_label.config(text=f"升降舵: {self.elevator_angle:.1f}°")
        self.aileron_label.config(text=f"副翼: {self.aileron_angle:.1f}°")
        self.rudder_label.config(text=f"方向舵: {self.rudder_angle:.1f}°")
        
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

def main():
    root = tk.Tk()
    app = FlightSimulator(root)
    root.mainloop()

if __name__ == "__main__":
    main() 