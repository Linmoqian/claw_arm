#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂抓放任务 - 真实硬件版本
在两个点位之间进行抓取-放置循环运动
"""

import time
import sys
import os
import numpy as np

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
    from scipy.optimize import minimize
    LIBS_AVAILABLE = True
except ImportError as e:
    print(f"[ERROR] 缺少依赖库: {e}")
    print("请运行: pip install roboticstoolbox-python spatialmath-python scipy")
    sys.exit(1)


# ───────── 常量 ─────────
SERVO_CENTER = 2047
SERVO_RANGE = 270.0
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

# DH 参数
DH_PARAMS = [
    (0.0,    np.pi/2,  0.0165,  np.pi/2),
    (0.0,    -np.pi/2, 0.1478,  0.0),
    (0.0,    0.0,      0.14057, 0.0),
    (0.0,    0.0,      0.1349,  0.0),
    (0.0,    np.pi/2,  0.0,     0.0),
]

# 关节安全限制 (度)
JOINT_LIMITS = [
    (-114.6, 114.6),   # J1
    (0.0, 200.5),      # J2
    (-180.0, 0.0),     # J3
    (-143.2, 68.8),    # J4
    (-180.0, 180.0),   # J5
]

# 夹爪位置
GRIPPER_OPEN = 2800   # 张开
GRIPPER_CLOSE = 1200  # 闭合


# ANSI 颜色
class C:
    RESET = '\033[0m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    DIM = '\033[2m'


def angle_to_position(angle_deg: float) -> int:
    """角度转舵机位置"""
    pos = int(SERVO_CENTER + (angle_deg * 4095 / SERVO_RANGE))
    return max(0, min(4095, pos))


class RobotController:
    """机械臂控制器"""

    def __init__(self, port: str = "COM7", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.connected = False

        # 创建运动学模型
        self.robot = rtb.DHRobot(
            [rtb.RevoluteDH(a=row[0], alpha=row[1], d=row[2], offset=row[3])
             for row in DH_PARAMS],
            name="SO100_DH",
            tool=SE3(0, 0, 0.0601)
        )
        self.current_q = np.deg2rad([0, 30, -45, 30, 0])

    def connect(self):
        """连接硬件"""
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)

        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"无法设置波特率 {self.baudrate}")

        print(f"{C.GREEN}[OK]{C.RESET} 已连接到 {self.port}")
        self.connected = True

    def disconnect(self):
        """断开连接"""
        if self.port_handler:
            # 启用所有扭矩防止掉落
            for motor_id in MOTOR_IDS:
                self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)
            time.sleep(0.1)
            self.port_handler.closePort()
            print(f"{C.GREEN}[OK]{C.RESET} 已断开连接 (扭矩已恢复)")
        self.connected = False

    def enable_torque(self, motor_id: int):
        """启用扭矩"""
        if self.connected:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)

    def disable_torque(self, motor_id: int):
        """禁用扭矩"""
        if self.connected:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 0)

    def write_position(self, motor_id: int, position: int):
        """写入位置"""
        if self.connected:
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, 42, position)

    def sync_write_positions(self, positions: list):
        """同步写入多个关节位置"""
        if not self.connected:
            return

        for motor_id, pos in zip(MOTOR_IDS[:5], positions[:5]):
            self.write_position(motor_id, pos)
        time.sleep(0.002)  # 短暂延迟确保指令发出

    def move_to_joints(self, joint_angles: list, duration: float = 0.5):
        """移动到目标关节角度"""
        if not self.connected:
            return

        # 限制在安全范围内
        joint_angles = np.array(joint_angles)
        for i, (angle, (min_val, max_val)) in enumerate(zip(joint_angles, JOINT_LIMITS)):
            joint_angles[i] = max(min_val, min(max_val, angle))

        # 转换为舵机位置
        positions = [angle_to_position(a) for a in joint_angles]

        # 写入位置
        self.sync_write_positions(positions)
        time.sleep(duration)

    def set_gripper(self, open_state: bool = True):
        """设置夹爪状态"""
        if not self.connected:
            return
        pos = GRIPPER_OPEN if open_state else GRIPPER_CLOSE
        self.write_position(6, pos)
        time.sleep(0.5)  # 等待夹爪动作完成

    def inverse_kinematics(self, target_pose: SE3, initial_q: np.ndarray = None) -> np.ndarray:
        """数值逆运动学求解"""
        if initial_q is None:
            initial_q = self.current_q

        def objective(q):
            current_pose = self.robot.fkine(q)
            pos_error = np.sum((current_pose.t - target_pose.t) ** 2)
            rot_error = np.sum((current_pose.R - target_pose.R) ** 2)
            return pos_error + 0.1 * rot_error

        result = minimize(
            objective,
            initial_q,
            method='SLSQP',
            bounds=[np.deg2rad([min_val, max_val]) for min_val, max_val in JOINT_LIMITS],
            options={'ftol': 1e-4, 'maxiter': 50}
        )

        if result.success:
            return result.x
        else:
            return initial_q

    def get_trajectory(self, start_pose: SE3, end_pose: SE3, steps: int) -> list:
        """获取笛卡尔直线轨迹"""
        return list(rtb.ctraj(start_pose, end_pose, steps))


def print_header(text: str):
    print(f"\n{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  {text}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}\n")


# 全局配置（由命令行参数设置）
port = "COM7"
cycles = 3
steps = 20
step_delay = 0.1
no_confirm = False


def run_pick_place():
    """运行抓放任务"""
    global port, cycles, steps, step_delay, no_confirm

    print_header("SO100 机械臂抓放任务 - 真实硬件")

    # 定义点位（使用关节角度更可靠）
    # 初始/安全位置
    safe_joints = [0, 45, -60, 45, 0]  # 度

    # 抓取位置
    pick_joints = [0, 50, -50, 50, 0]

    # 放置位置
    place_joints = [0, 40, -70, 60, 0]

    print(f"{C.CYAN}[关节点位设置]{C.RESET}")
    print(f"  安全点: {C.YELLOW}{safe_joints}{C.RESET}°")
    print(f"  抓取点: {C.GREEN}{pick_joints}{C.RESET}°")
    print(f"  放置点: {C.BLUE}{place_joints}{C.RESET}°")

    print(f"\n{C.CYAN}[运动参数]{C.RESET}")
    print(f"  循环次数: {cycles}")
    print(f"  插值步数: {steps}")
    print(f"  单程时间: 约{steps * step_delay:.1f}秒")

    # 创建控制器
    ctrl = RobotController(port)

    try:
        ctrl.connect()

        # 检测电机
        print(f"\n{C.CYAN}[硬件检测]{C.RESET}")
        for motor_id in MOTOR_IDS:
            _, result, _ = ctrl.packet_handler.ping(ctrl.port_handler, motor_id)
            status = f"{C.GREEN}在线{C.RESET}" if result == 0 else f"{C.RED}离线{C.RESET}"
            print(f"  Motor {motor_id}: {status}")

        # 启用扭矩
        print(f"\n{C.YELLOW}[INFO]{C.RESET} 启用电机扭矩...")
        for motor_id in MOTOR_IDS:
            ctrl.enable_torque(motor_id)
        time.sleep(0.5)

        if not no_confirm:
            print(f"\n{C.RED}{C.BOLD}[WARNING]{C.RESET} 确保机械臂工作空间内无障碍物！")
            print(f"{C.YELLOW}[INFO]{C.RESET} 夹爪需要在两个点位之间移动物体")
            input(f"\n{C.DIM}按 Enter 开始，Ctrl+C 紧急停止...{C.RESET}")
        else:
            print(f"\n{C.YELLOW}[INFO]{C.RESET} 自动模式开始运行...")

        print(f"\n{C.GREEN}[START]{C.RESET} 开始抓放任务...\n")

        # 移动到安全点
        print(f"{C.DIM}→ 移动到安全点{C.RESET}")
        ctrl.move_to_joints(safe_joints, 1.0)

        for cycle in range(cycles):
            print(f"\n{C.BOLD}第 {cycle + 1}/{cycles} 次循环{C.RESET}")

            # 1. 安全点 -> 抓取点
            print(f"  {C.DIM}→ 移动到抓取点{C.RESET}")
            for i in range(steps):
                ratio = (i + 1) / steps
                angles = [
                    safe_joints[j] + (pick_joints[j] - safe_joints[j]) * ratio
                    for j in range(5)
                ]
                ctrl.move_to_joints(angles, step_delay)

            # 2. 闭合夹爪
            print(f"  {C.YELLOW}→ 闭合夹爪{C.RESET}")
            ctrl.set_gripper(False)

            # 3. 抓取点 -> 安全点
            print(f"  {C.DIM}→ 移动到安全点{C.RESET}")
            for i in range(steps):
                ratio = (i + 1) / steps
                angles = [
                    pick_joints[j] + (safe_joints[j] - pick_joints[j]) * ratio
                    for j in range(5)
                ]
                ctrl.move_to_joints(angles, step_delay)

            # 4. 安全点 -> 放置点
            print(f"  {C.DIM}→ 移动到放置点{C.RESET}")
            for i in range(steps):
                ratio = (i + 1) / steps
                angles = [
                    safe_joints[j] + (place_joints[j] - safe_joints[j]) * ratio
                    for j in range(5)
                ]
                ctrl.move_to_joints(angles, step_delay)

            # 5. 张开夹爪
            print(f"  {C.GREEN}→ 张开夹爪{C.RESET}")
            ctrl.set_gripper(True)

            # 6. 放置点 -> 安全点
            print(f"  {C.DIM}→ 移动到安全点{C.RESET}")
            for i in range(steps):
                ratio = (i + 1) / steps
                angles = [
                    place_joints[j] + (safe_joints[j] - place_joints[j]) * ratio
                    for j in range(5)
                ]
                ctrl.move_to_joints(angles, step_delay)

            print(f"  {C.GREEN}[OK] 第 {cycle + 1} 次循环完成{C.RESET}")

        print(f"\n{C.GREEN}[完成]{C.RESET} 所有循环完成！")

        # 自动回到初始位置
        print(f"{C.DIM}返回初始位置...{C.RESET}")
        ctrl.move_to_joints([0, 0, 0, 0, 0], 1.5)

    except KeyboardInterrupt:
        print(f"\n\n{C.YELLOW}[中断]{C.RESET} 用户中断")
    except Exception as e:
        print(f"\n{C.RED}[ERROR]{C.RESET} {e}")
        import traceback
        traceback.print_exc()
    finally:
        ctrl.disconnect()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="SO100 抓放任务")
    parser.add_argument('--port', type=str, default='COM7', help='串口')
    parser.add_argument('--cycles', type=int, default=3, help='循环次数')
    parser.add_argument('--steps', type=int, default=20, help='插值步数')
    parser.add_argument('--delay', type=float, default=0.1, help='每步延迟(秒)')
    parser.add_argument('--no-confirm', action='store_true', help='跳过确认直接运行')

    args = parser.parse_args()

    # 更新全局变量
    global port, cycles, steps, step_delay, no_confirm
    port = args.port
    cycles = args.cycles
    steps = args.steps
    step_delay = args.delay
    no_confirm = args.no_confirm

    run_pick_place()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{C.CYAN}[再见]{C.RESET}")
    finally:
        print(C.RESET)
