#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂实时坐标系查看器
统一交互式脚本，支持文本模式和3D可视化

基于 URDF: SO100描述文件/so100.urdf
"""

import time
import sys
import os
import numpy as np
import threading

# Windows 启用 ANSI 转义序列支持
if os.name == 'nt':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 2077)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from SDK import PortHandler, PacketHandler

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d import Axes3D
    LIBS_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] 可视化库未安装: {e}")
    print("请运行: pip install roboticstoolbox-python spatialmath-python matplotlib")
    LIBS_AVAILABLE = False
    sys.exit(1)


# ───────── ANSI 颜色代码 ─────────
class Colors:
    """终端颜色代码"""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'

    # 前景色
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

    # 亮色
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'

    # 背景色
    BG_BLACK = '\033[40m'
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'
    BG_MAGENTA = '\033[45m'
    BG_CYAN = '\033[46m'
    BG_WHITE = '\033[47m'

    @staticmethod
    def ok(text: str) -> str:
        return f"{Colors.GREEN}{text}{Colors.RESET}"

    @staticmethod
    def warn(text: str) -> str:
        return f"{Colors.YELLOW}{text}{Colors.RESET}"

    @staticmethod
    def error(text: str) -> str:
        return f"{Colors.RED}{text}{Colors.RESET}"

    @staticmethod
    def info(text: str) -> str:
        return f"{Colors.CYAN}{text}{Colors.RESET}"

    @staticmethod
    def title(text: str) -> str:
        return f"{Colors.BOLD}{Colors.BRIGHT_CYAN}{text}{Colors.RESET}"

    @staticmethod
    def header(text: str) -> str:
        return f"{Colors.BOLD}{Colors.BRIGHT_WHITE}{text}{Colors.RESET}"


# ───────── 常量 (基于 SO100 URDF) ─────────
SERVO_CENTER = 2047
SERVO_RANGE = 270.0
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

# 关节名称 (匹配 URDF 标准)
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

# 关节安全限制 (基于 URDF, 单位: 度)
JOINT_LIMITS = {
    'shoulder_pan': (-114.6, 114.6),
    'shoulder_lift': (0.0, 200.5),
    'elbow_flex': (-180.0, 0.0),
    'wrist_flex': (-143.2, 68.8),
    'wrist_roll': (-180.0, 180.0),
    'gripper': (-11.5, 114.6),
}

# DH 参数
DH_PARAMS = [
    (0.0,    np.pi/2,  0.0165,  np.pi/2),
    (0.0,    -np.pi/2, 0.1478,  0.0),
    (0.0,    0.0,      0.14057, 0.0),
    (0.0,    0.0,      0.1349,  0.0),
    (0.0,    np.pi/2,  0.0,     0.0),
]

# 关节显示名称
JOINT_DISPLAY_NAMES = {
    'shoulder_pan': 'J1 底部旋转',
    'shoulder_lift': 'J2 大臂升降',
    'elbow_flex': 'J3 肘部弯曲',
    'wrist_flex': 'J4 手腕俯仰',
    'wrist_roll': 'J5 手腕旋转',
    'gripper': 'J6 夹爪',
}

# 符号
SYMBOLS = {
    'ok': '[OK]',
    'fail': '[FAIL]',
    'warn': '[WARN]',
    'info': '[INFO]',
    'torque_on': '[LOCK]',
    'torque_off': '[FREE]',
    'motor': '●',
    'check': '✓',
    'cross': '✗',
    'arrow': '→',
}


def position_to_angle(pos: int, center: int = 2047, range_deg: float = 270) -> float:
    """舵机位置转角度"""
    return (pos - center) * (range_deg / 4095)


# 全局配置
PORT_CONFIG = {
    'port': 'COM7',
    'simulation': False,
    'auto_freedrag': True,
}


class RobotMonitor:
    """机械臂监控器"""

    def __init__(self, port: str = "COM7", baudrate: int = 1000000, simulation: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.simulation = simulation

        self.robot = rtb.DHRobot(
            [rtb.RevoluteDH(a=row[0], alpha=row[1], d=row[2], offset=row[3])
             for row in DH_PARAMS],
            name="SO100_DH",
            tool=SE3(0, 0, 0.0601)
        )

        self.current_angles = [0.0, 10.0, -20.0, 15.0, 0.0]
        self.end_effector_pose = SE3()

    def connect(self):
        if self.simulation:
            print(f"{Colors.info('[SIMULATION]')} 模拟模式，无需硬件连接")
            return

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)

        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"无法设置波特率 {self.baudrate}")
        print(f"{Colors.ok(SYMBOLS['ok'])} 已连接到 {Colors.header(self.port)}")

    def enable_torque(self, motor_id: int):
        if self.simulation: return
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)

    def disable_torque(self, motor_id: int):
        if self.simulation: return
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 0)

    def get_torque_state(self, motor_id: int) -> bool:
        if self.simulation: return False
        data, result, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, 40)
        return data == 1 if result == 0 else False

    def enable_all_torque(self):
        for motor_id in MOTOR_IDS:
            self.enable_torque(motor_id)
        print(f"{Colors.warn(SYMBOLS['torque_on'])} 所有电机已锁定")

    def disable_all_torque(self):
        for motor_id in MOTOR_IDS:
            self.disable_torque(motor_id)
        print(f"{Colors.ok(SYMBOLS['torque_off'])} 所有电机已释放 - 可以手动拖动")

    def read_joint_positions(self) -> list:
        if self.simulation:
            import time as t
            tt = t.time()
            return [
                2047 + int(500 * np.sin(tt)),
                2047 + int(300 * np.sin(tt * 0.7)),
                2047 + int(400 * np.sin(tt * 0.5)),
                2047 + int(200 * np.sin(tt * 1.2)),
                2047 + int(150 * np.sin(tt * 0.8)),
            ]

        positions = []
        for motor_id in MOTOR_IDS[:5]:
            try:
                data, result, _ = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, 56)
                positions.append(data if result == 0 else 2047)
            except:
                positions.append(2047)
        while len(positions) < 5:
            positions.append(2047)
        return positions

    def update(self):
        try:
            positions = self.read_joint_positions()
            self.current_angles = [position_to_angle(p) for p in positions]

            if len(self.current_angles) != 5:
                self.current_angles = [0.0] * 5

            q = np.array(self.current_angles)
            q_rad = np.deg2rad(q)
            self.end_effector_pose = self.robot.fkine(q_rad)

            return list(self.current_angles), self.end_effector_pose
        except Exception as e:
            return [0.0] * 5, SE3()

    def disconnect(self):
        if self.port_handler:
            for motor_id in MOTOR_IDS:
                self.enable_torque(motor_id)
            time.sleep(0.1)
            self.port_handler.closePort()
            print(f"{Colors.ok(SYMBOLS['ok'])} 已断开连接 (扭矩已恢复)")


def format_angle_bar(value: float, vmin: float, vmax: float, width: int = 15) -> str:
    """生成角度可视化条"""
    ratio = (value - vmin) / (vmax - vmin) if vmax != vmin else 0.5
    ratio = max(0, min(1, ratio))
    filled = int(ratio * width)

    # 颜色根据位置决定
    if ratio < 0.3:
        color = Colors.BRIGHT_BLUE
    elif ratio < 0.7:
        color = Colors.BRIGHT_GREEN
    else:
        color = Colors.BRIGHT_YELLOW

    bar = color + '█' * filled + Colors.DIM + '░' * (width - filled) + Colors.RESET
    return bar


def text_mode(monitor: RobotMonitor, interval: float = 0.05):
    """文本模式 - 增强版显示"""
    print(Colors.RESET)
    print("\n" + Colors.title("════════════════════════════════════════════════════════════"))
    print(Colors.title("           文本模式 - 实时坐标系监控"))
    print(Colors.title("════════════════════════════════════════════════════════════"))
    print(f"\n{Colors.info('按 Ctrl+C 退出')}\n")

    print(f"{Colors.DIM}{'─' * 70}{Colors.RESET}")
    print(f"  {'关节':<15} {'角度':<8} {'范围':<20} {'状态'}")
    print(f"{Colors.DIM}{'─' * 70}{Colors.RESET}")

    # 显示关节限制
    for name in JOINT_NAMES[:5]:
        limits = JOINT_LIMITS[name]
        print(f"  {JOINT_DISPLAY_NAMES[name]:<15} {Colors.DIM}{'┄'}{Colors.RESET:<8} {limits[0]:>6.0f}° ~ {limits[1]:<6.0f}°")
    print(f"{Colors.DIM}{'─' * 70}{Colors.RESET}\n")

    print(f"{Colors.CYAN}实时更新中...{Colors.RESET}\n")

    # 用于记录上一行，避免重复输出
    last_line = ""

    try:
        while True:
            angles, pose = monitor.update()
            rpy = np.rad2deg(pose.rpy())

            # 清除上一行并输出新状态
            print(f"\r{' ' * 100}\r", end="")

            # 主状态行
            status = (
                f"{Colors.BRIGHT_CYAN}Pos{Colors.RESET}:({pose.t[0]:.3f},{pose.t[1]:.3f},{pose.t[2]:.3f})m  "
                f"{Colors.BRIGHT_CYAN}RPY{Colors.RESET}:({rpy[0]:.0f},{rpy[1]:.0f},{rpy[2]:.0f})°"
            )
            print(f"\r{status}", end="", flush=True)

            time.sleep(interval)
    except KeyboardInterrupt:
        print(f"\r{' ' * 100}\r", end="")
        print(f"\n{Colors.info('[退出]')}")


def detailed_text_mode(monitor: RobotMonitor, interval: float = 0.1):
    """详细文本模式 - 显示所有关节状态"""
    print(Colors.RESET)
    print("\n" + Colors.title("════════════════════════════════════════════════════════════"))
    print(Colors.title("           详细模式 - 实时关节状态"))
    print(Colors.title("════════════════════════════════════════════════════════════"))
    print(f"\n{Colors.info('按 Ctrl+C 退出')}\n")

    try:
        while True:
            angles, pose = monitor.update()
            rpy = np.rad2deg(pose.rpy())

            # 清屏并重绘
            os.system('cls' if os.name == 'nt' else 'clear')

            print("\n" + Colors.title("════════════════════════════════════════════════════════════"))
            print(Colors.title("           SO100 机械臂实时状态"))
            print(Colors.title("════════════════════════════════════════════════════════════"))

            # 末端执行器位置
            print(f"\n  {Colors.BRIGHT_CYAN}┌─ 末端执行器 ─────────────────────────┐")
            print(f"  │ 位置:  X={pose.t[0]:>7.3f}m  Y={pose.t[1]:>7.3f}m  Z={pose.t[2]:>7.3f}m")
            print(f"  │ 姿态:  R={rpy[0]:>6.0f}°  P={rpy[1]:>6.0f}°  Y={rpy[2]:>6.0f}°")
            print(f"  └─────────────────────────────────────────┘{Colors.RESET}")

            # 关节状态
            print(f"\n  {Colors.BRIGHT_CYAN}┌─ 关节角度 ────────────────────────────┐{Colors.RESET}")

            for i, (name, angle) in enumerate(zip(JOINT_NAMES[:5], angles)):
                limits = JOINT_LIMITS[name]
                vmin, vmax = limits

                # 判断是否超出范围
                is_out_of_range = angle < vmin or angle > vmax
                angle_color = Colors.BRIGHT_RED if is_out_of_range else Colors.BRIGHT_WHITE

                # 角度值
                print(f"  │ {JOINT_DISPLAY_NAMES[name]:<12} {angle_color}{angle:>7.1f}°{Colors.RESET} ", end="")

                # 可视化条
                bar = format_angle_bar(angle, vmin, vmax)
                print(f"{bar} {Colors.DIM}{vmin:>5.0f}°~{vmax:<5.0f}°{Colors.RESET}")

            print(f"  {Colors.BRIGHT_CYAN}└──────────────────────────────────────────┘{Colors.RESET}")
            print(f"\n  {Colors.DIM}更新中... (Ctrl+C 退出){Colors.RESET}")

            time.sleep(interval)
    except KeyboardInterrupt:
        print(f"\n{Colors.info('[退出]')}")


class Visualizer:
    """3D 可视化器"""

    def __init__(self, monitor: RobotMonitor):
        self.monitor = monitor
        self.frame_count = 0

        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 9), facecolor='#1a1a2e')
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#1a1a2e')

        self.ax.xaxis.label.set_color('#e0e0e0')
        self.ax.yaxis.label.set_color('#e0e0e0')
        self.ax.zaxis.label.set_color('#e0e0e0')
        self.ax.tick_params(axis='x', colors='#888888')
        self.ax.tick_params(axis='y', colors='#888888')
        self.ax.tick_params(axis='z', colors='#888888')

        self.ax.grid(False)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.xaxis.pane.set_edgecolor('#333333')
        self.ax.yaxis.pane.set_edgecolor('#333333')
        self.ax.zaxis.pane.set_edgecolor('#333333')

    def update_frame(self, frame):
        try:
            self.ax.cla()
            self.ax.set_facecolor('#1a1a2e')
            self.ax.grid(False)
            self.ax.xaxis.pane.fill = False
            self.ax.yaxis.pane.fill = False
            self.ax.zaxis.pane.fill = False

            limit = 0.4
            self.ax.set_xlim(-limit, limit)
            self.ax.set_ylim(-limit, limit)
            self.ax.set_zlim(0, 0.5)
            self.ax.set_xlabel('X (m)', color='#e0e0e0', fontsize=10)
            self.ax.set_ylabel('Y (m)', color='#e0e0e0', fontsize=10)
            self.ax.set_zlabel('Z (m)', color='#e0e0e0', fontsize=10)
            self.ax.tick_params(colors='#666666', labelsize=8)

            # 网格地板
            grid_x = np.linspace(-limit, limit, 9)
            grid_y = np.linspace(-limit, limit, 9)
            for i in range(len(grid_x)):
                self.ax.plot(grid_x[i] * np.ones(9), grid_y, np.zeros(9),
                            color='#2a2a4a', linewidth=0.5, alpha=0.5)
                self.ax.plot(grid_x, grid_y[i] * np.ones(9), np.zeros(9),
                            color='#2a2a4a', linewidth=0.5, alpha=0.5)

            # 原点坐标轴
            axis_len = 0.08
            self.ax.plot([0, axis_len], [0, 0], [0, 0], '#ff4757', linewidth=2, alpha=0.8)
            self.ax.plot([0, 0], [0, axis_len], [0, 0], '#2ed573', linewidth=2, alpha=0.8)
            self.ax.plot([0, 0], [0, 0], [0, axis_len], '#1e90ff', linewidth=2, alpha=0.8)

            angles, pose = self.monitor.update()
            if not isinstance(angles, (list, tuple)):
                angles = [angles]
            angles = list(angles)[:5]
            while len(angles) < 5:
                angles.append(0.0)

            # 计算关节位置
            points = [[0, 0, 0]]
            q = np.deg2rad(angles)
            for i in range(5):
                q_work = np.zeros(5)
                q_work[:i+1] = q[:i+1]
                pt = self.monitor.robot.fkine(q_work).t
                points.append([pt[0], pt[1], pt[2]])
            points = np.array(points)

            # 渐变色连杆
            colors = ['#ff4757', '#ffa502', '#ffdd59', '#2ed573', '#1e90ff']
            for i in range(len(points) - 1):
                c = colors[i % len(colors)]
                self.ax.plot(points[i:i+2, 0], points[i:i+2, 1], points[i:i+2, 2],
                            '-', linewidth=5, color=c, solid_capstyle='round', alpha=0.9)

            # 关节点
            self.ax.scatter(points[1:, 0], points[1:, 1], points[1:, 2],
                           s=120, c='#ffffff', edgecolors='#00d2d3',
                           linewidth=2, zorder=10, alpha=0.95)

            # 末端执行器
            self.ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2],
                           s=180, c='#ff6b81', edgecolors='#ffffff',
                           linewidth=2, zorder=11, marker='*')

            # 末端坐标系
            origin = pose.t
            R = pose.R
            axis_length = 0.08

            x_end = origin + R[:, 0] * axis_length
            y_end = origin + R[:, 1] * axis_length
            z_end = origin + R[:, 2] * axis_length

            self.ax.plot([origin[0], x_end[0]], [origin[1], x_end[1]], [origin[2], x_end[2]],
                        '#ff4757', linewidth=4, solid_capstyle='round')
            self.ax.plot([origin[0], y_end[0]], [origin[1], y_end[1]], [origin[2], y_end[2]],
                        '#2ed573', linewidth=4, solid_capstyle='round')
            self.ax.plot([origin[0], z_end[0]], [origin[1], z_end[1]], [origin[2], z_end[2]],
                        '#1e90ff', linewidth=4, solid_capstyle='round')

            # 信息显示
            rpy = np.rad2deg(pose.rpy())
            info_lines = [
                f"J1:{angles[0]:.0f}° J2:{angles[1]:.0f}° J3:{angles[2]:.0f}° J4:{angles[3]:.0f}° J5:{angles[4]:.0f}°",
                f"Pos: X={pose.t[0]:.3f}m Y={pose.t[1]:.3f}m Z={pose.t[2]:.3f}m",
                f"RPY: R={rpy[0]:.0f}° P={rpy[1]:.0f}° Y={rpy[2]:.0f}°",
            ]

            self.ax.text2D(0.02, 0.95, "SO100 实时监控", transform=self.ax.transAxes,
                          color='#00d2d3', fontsize=14, weight='bold')
            self.ax.text2D(0.02, 0.90, info_lines[0], transform=self.ax.transAxes,
                          color='#ffffff', fontsize=10, family='monospace')
            self.ax.text2D(0.02, 0.87, info_lines[1], transform=self.ax.transAxes,
                          color='#aaaaaa', fontsize=9, family='monospace')
            self.ax.text2D(0.02, 0.84, info_lines[2], transform=self.ax.transAxes,
                          color='#aaaaaa', fontsize=9, family='monospace')

        except Exception as e:
            print(f"[ERROR] {e}")

    def show(self, interval: float = 0.1):
        anim = FuncAnimation(self.fig, self.update_frame, interval=interval*1000,
                             blit=False, cache_frame_data=False)
        plt.show()


def visual_mode(monitor: RobotMonitor, interval: float = 0.1):
    """3D 可视化模式"""
    print("\n" + Colors.title("════════════════════════════════════════════════════════════"))
    print(Colors.title("              3D 可视化模式"))
    print(Colors.title("════════════════════════════════════════════════════════════"))
    print(f"\n{Colors.info('关闭 3D 窗口退出')}\n")

    viz = Visualizer(monitor)
    viz.show(interval=interval)


def snapshot_mode(monitor: RobotMonitor, output: str = "robot_snapshot.png"):
    """快照模式"""
    print(f"\n{Colors.info('[INFO]')} 生成当前状态快照...")
    print(f"{Colors.DIM}      保存到: {output}{Colors.RESET}")

    viz = Visualizer(monitor)
    viz.update_frame(0)
    plt.savefig(output, dpi=100, bbox_inches='tight', facecolor='#1a1a2e')
    print(f"{Colors.ok(SYMBOLS['ok'])} 快照已保存")


def print_header(title: str):
    """打印标题头"""
    print(Colors.RESET)
    print("\n" + Colors.title("╔════════════════════════════════════════════════════════════╗"))
    print(Colors.title(f"║  {title:<58}║"))
    print(Colors.title("╚════════════════════════════════════════════════════════════╝"))


def print_menu_item(index: int, text: str, desc: str = "", hotkey: str = ""):
    """打印菜单项"""
    hotkey_str = f" {Colors.BRIGHT_YELLOW}[{hotkey}]{Colors.RESET}" if hotkey else ""
    desc_str = f"{Colors.DIM} - {desc}{Colors.RESET}" if desc else ""
    print(f"  {Colors.BRIGHT_CYAN}{index}.{Colors.RESET} {Colors.BRIGHT_WHITE}{text}{Colors.RESET}{desc_str}{hotkey_str}")


def show_menu():
    """显示主菜单"""
    os.system('cls' if os.name == 'nt' else 'clear')

    print_header("SO100 机械臂坐标系查看器")

    print(f"\n{Colors.BRIGHT_CYAN}┌─ 运行模式 ─────────────────────────────────┐{Colors.RESET}")

    print_menu_item(1, "文本模式", "紧凑单行实时显示", "T")
    print_menu_item(2, "详细模式", "显示所有关节状态和可视化条", "D")
    print_menu_item(3, "3D 可视化", "图形化显示机械臂", "V")
    print_menu_item(4, "快照模式", "保存当前状态为图片", "S")

    print(f"\n{Colors.BRIGHT_CYAN}┌─ 设置 ─────────────────────────────────────┐{Colors.RESET}")
    print_menu_item(5, "设置", "更改端口或模式", ",")

    print(f"\n{Colors.BRIGHT_CYAN}┌─ 其他 ─────────────────────────────────────┐{Colors.RESET}")
    print_menu_item(0, "退出", "", "Q")

    print(f"\n{Colors.DIM}{'─' * 50}{Colors.RESET}")
    print(f"{Colors.BRIGHT_WHITE}  当前配置:{Colors.RESET}")
    mode_str = f"{Colors.BRIGHT_YELLOW}模拟{Colors.DIM}(无硬件){Colors.RESET}" if PORT_CONFIG['simulation'] else f"{Colors.BRIGHT_GREEN}真实硬件{Colors.RESET}"
    torque_str = f"{Colors.BRIGHT_GREEN}释放{Colors.DIM}(可拖动){Colors.RESET}" if PORT_CONFIG['auto_freedrag'] else f"{Colors.BRIGHT_YELLOW}锁定{Colors.RESET}"
    print(f"    串口: {Colors.BRIGHT_CYAN}{PORT_CONFIG['port']}{Colors.RESET}  │  模式: {mode_str}  │  扭矩: {torque_str}")
    print(f"{Colors.DIM}{'─' * 50}{Colors.RESET}\n")

    print(f"{Colors.DIM}快捷键: 直接输入字母或数字{Colors.RESET}\n")


def settings_menu():
    """设置菜单"""
    while True:
        os.system('cls' if os.name == 'nt' else 'clear')

        print_header("系统设置")

        print(f"\n{Colors.DIM}当前配置:{Colors.RESET}\n")
        print(f"  {Colors.BRIGHT_CYAN}1.{Colors.RESET} 串口:     {Colors.BRIGHT_WHITE}{PORT_CONFIG['port']}{Colors.RESET}")
        print(f"  {Colors.BRIGHT_CYAN}2.{Colors.RESET} 模式:     {Colors.BRIGHT_WHITE}{'模拟 (无硬件)' if PORT_CONFIG['simulation'] else '真实硬件'}{Colors.RESET}")
        print(f"  {Colors.BRIGHT_CYAN}3.{Colors.RESET} 自动释放: {Colors.BRIGHT_WHITE}{'是' if PORT_CONFIG['auto_freedrag'] else '否'}{Colors.RESET}")
        print(f"\n  {Colors.BRIGHT_CYAN}0.{Colors.RESET} 返回")

        choice = input(f"\n{Colors.info('→')} 请选择要修改的项: ").strip()

        if choice in ['0', 'q', 'Q', 'b', 'B']:
            break
        elif choice == '1':
            port = input(f"  输入串口 (当前 {PORT_CONFIG['port']}): ").strip()
            if port:
                PORT_CONFIG['port'] = port.upper()
                print(f"{Colors.ok(SYMBOLS['ok'])} 串口已更新")
        elif choice == '2':
            PORT_CONFIG['simulation'] = not PORT_CONFIG['simulation']
            mode = '模拟模式' if PORT_CONFIG['simulation'] else '真实硬件'
            print(f"{Colors.ok(SYMBOLS['ok'])} 已切换到 {mode}")
        elif choice == '3':
            PORT_CONFIG['auto_freedrag'] = not PORT_CONFIG['auto_freedrag']
            state = '释放 (可拖动)' if PORT_CONFIG['auto_freedrag'] else '锁定'
            print(f"{Colors.ok(SYMBOLS['ok'])} 自动释放扭矩: {state}")

        time.sleep(0.5)


def run_monitor(monitor, choice):
    """执行监控功能"""
    try:
        monitor.connect()

        # 检测电机
        if not PORT_CONFIG['simulation']:
            print(f"\n{Colors.BRIGHT_CYAN}┌─ 硬件检测 ─────────────────────────────────┐{Colors.RESET}")

            # 检测电机在线状态
            print(f"\n  {Colors.DIM}电机状态:{Colors.RESET}")
            online_count = 0
            for motor_id in MOTOR_IDS:
                _, result, _ = monitor.packet_handler.ping(monitor.port_handler, motor_id)
                if result == 0:
                    print(f"    {Colors.BRIGHT_GREEN}{SYMBOLS['motor']}{Colors.RESET} Motor {motor_id}: {Colors.ok('在线')}")
                    online_count += 1
                else:
                    print(f"    {Colors.DIM}{SYMBOLS['motor']}{Colors.RESET} Motor {motor_id}: {Colors.error('离线')}")

            print(f"\n  {Colors.DIM}扭矩状态:{Colors.RESET}")
            torque_states = []
            for motor_id in MOTOR_IDS[:5]:
                is_enabled = monitor.get_torque_state(motor_id)
                state_str = f"{Colors.warn(SYMBOLS['torque_on'])} 锁定" if is_enabled else f"{Colors.ok(SYMBOLS['torque_off'])} 释放"
                torque_states.append(is_enabled)
                print(f"    Motor {motor_id}: {state_str}")

            any_locked = any(torque_states)
            if any_locked:
                print(f"\n  {Colors.warn('[WARN]')} 部分电机处于锁定状态")
            else:
                print(f"\n  {Colors.ok(SYMBOLS['ok'])} 所有电机已释放")

            print(f"\n{Colors.BRIGHT_CYAN}└─────────────────────────────────────────────┘{Colors.RESET}\n")

        # 默认释放扭矩
        if PORT_CONFIG['auto_freedrag'] and not PORT_CONFIG['simulation']:
            print(f"{Colors.ok(SYMBOLS['torque_off'])} 释放扭矩 - 可以手动拖动机械臂\n")
            monitor.disable_all_torque()

        # 执行选择的功能
        if choice == '1':
            text_mode(monitor)
        elif choice == '2':
            detailed_text_mode(monitor)
        elif choice == '3':
            visual_mode(monitor)
        elif choice == '4':
            snapshot_mode(monitor)

    except KeyboardInterrupt:
        print(f"\n{Colors.info('[返回]')}")
    except Exception as e:
        print(f"{Colors.error('[ERROR]')} {e}")
        import traceback
        traceback.print_exc()
    finally:
        monitor.disconnect()


def main():
    import argparse

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--port', type=str)
    parser.add_argument('--simulation', '--sim', action='store_true')
    parser.add_argument('--no-freedrag', action='store_true')
    parser.add_argument('--mode', type=str, choices=['text', 'detail', 'visual', 'snapshot'])
    parser.add_argument('--help', '-h', action='store_true')

    args, _ = parser.parse_known_args()

    if args.help:
        print(f"\n{Colors.BRIGHT_CYAN}SO100 机械臂坐标系查看器{Colors.RESET}")
        print(f"\n{Colors.DIM}用法:{Colors.RESET}")
        print(f"  python arm_viewer.py [选项]")
        print(f"\n{Colors.DIM}选项:{Colors.RESET}")
        print(f"  --port <串口>        设置串口 (默认: COM7)")
        print(f"  --simulation, --sim  模拟模式")
        print(f"  --no-freedrag        不自动释放扭矩")
        print(f"  --mode <模式>        直接启动模式")
        print(f"                       text   - 文本模式")
        print(f"                       detail - 详细模式")
        print(f"                       visual - 3D 可视化")
        print(f"                       snapshot - 快照模式")
        print(f"\n{Colors.DIM}示例:{Colors.RESET}")
        print(f"  python arm_viewer.py --mode text")
        print(f"  python arm_viewer.py --port COM8 --simulation")
        return

    # 应用命令行参数
    if args.port:
        PORT_CONFIG['port'] = args.port
    if args.simulation:
        PORT_CONFIG['simulation'] = True
    if args.no_freedrag:
        PORT_CONFIG['auto_freedrag'] = False

    # 直接指定模式时跳过菜单
    if args.mode:
        monitor = RobotMonitor(PORT_CONFIG['port'], simulation=PORT_CONFIG['simulation'])
        mode_map = {'text': '1', 'detail': '2', 'visual': '3', 'snapshot': '4'}
        run_monitor(monitor, mode_map.get(args.mode, '1'))
        return

    # 交互式菜单
    while True:
        show_menu()
        choice = input(f"{Colors.info('→')} 请输入选项: ").strip().lower()

        if choice in ['0', 'q', 'exit', 'quit']:
            print(f"\n{Colors.info('[退出]')} 再见!")
            break
        elif choice in [',', '5', 's', 'S', 'setting']:
            settings_menu()
        elif choice in ['1', 't', 'T']:
            monitor = RobotMonitor(PORT_CONFIG['port'], simulation=PORT_CONFIG['simulation'])
            run_monitor(monitor, '1')
        elif choice in ['2', 'd', 'D']:
            monitor = RobotMonitor(PORT_CONFIG['port'], simulation=PORT_CONFIG['simulation'])
            run_monitor(monitor, '2')
        elif choice in ['3', 'v', 'V']:
            monitor = RobotMonitor(PORT_CONFIG['port'], simulation=PORT_CONFIG['simulation'])
            run_monitor(monitor, '3')
        elif choice in ['4', 's', 'S']:
            monitor = RobotMonitor(PORT_CONFIG['port'], simulation=PORT_CONFIG['simulation'])
            run_monitor(monitor, '4')


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{Colors.info('[退出]')}")
    finally:
        print(Colors.RESET)
