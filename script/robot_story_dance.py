#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
机械臂之舞 - 苏醒、寻找、抓取、放置、休息
带 S 曲线平滑轨迹插值
"""

import sys
import os
import time
import math

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from SDK import PortHandler, PacketHandler

# Windows 启用 ANSI
if os.name == 'nt':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 2077)

# ANSI 颜色
class C:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RESET = '\033[0m'

PORT = "COM7"
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

SERVO_CENTER = 2047
SERVO_RANGE = 270.0

# 夹爪位置
GRIPPER_OPEN = 2800
GRIPPER_CLOSE = 1200


def angle_to_position(angle_deg: float) -> int:
    """角度转舵机位置"""
    pos = int(SERVO_CENTER + (angle_deg * 4095 / SERVO_RANGE))
    return max(0, min(4095, pos))


def positions_from_angles(angles: list) -> list:
    """角度列表转位置列表"""
    return [angle_to_position(a) for a in angles]


def scurve(t: float) -> float:
    """S 曲线插值函数 (平滑加减速)

    t: 0 到 1 之间的归一化时间
    返回: 平滑后的 0 到 1
    """
    if t <= 0:
        return 0.0
    if t >= 1:
        return 1.0
    # 使用平滑的 S 曲线: 3t² - 2t³
    return t * t * (3 - 2 * t)


def interpolate_pose(start, end, t):
    """插值两个姿态"""
    return [s + (e - s) * t for s, e in zip(start, end)]


# ============================================================================
# 动作序列定义
# ============================================================================

# 1. 盘着/休息姿态
POSE_RESTING = [0, -80, 80, -60, 0, -50]

# 2. 苏醒过程
POSES_WAKE_UP = [
    [0, -80, 80, -60, 0, -50],
    [0, -60, 60, -45, 0, -40],
    [0, -40, 40, -30, 0, -30],
    [0, -20, 20, -15, 0, -20],
    [0, 0, 0, 0, 0, 0],
]

# 3. 观察四周
POSES_LOOKING = [
    [0, 0, 0, 0, 0, 0],
    [-45, 0, 0, 0, 0, 0],
    [-60, 10, -10, 0, 0, 0],
    [-45, 0, 0, 0, 0, 0],
    [45, 0, 0, 0, 0, 0],
    [60, 10, -10, 0, 0, 0],
    [45, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
]

# 4. 发现目标
POSES_DISCOVER = [
    [0, 0, 0, 0, 0, 0],
    [10, -20, 20, -30, 0, -40],
    [5, -50, 40, -60, -20, -45],
]

# 5. 准备抓取
POSE_PREPARE = [4.5, -73.9, 63.7, -76.1, -58.8, -50]

# 6. 抓取动作
POSES_GRAB = [
    [4.5, -60, 50, -76.1, -58.8, 0],
    [4.5, -73.9, 63.7, -76.1, -58.8, 0],
    [4.5, -73.9, 63.7, -76.1, -58.8, -50],
]

# 7. 抬起并移动
POSES_LIFT_MOVE = [
    [4.5, -73.9, 63.7, -76.1, -58.8, -50],
    [4.5, -40, 30, -40, -58.8, -50],
    [0, -40, 30, -40, -58.8, -50],
    [-60, -40, 30, -40, -58.8, -50],
    [-90, -30, 20, -30, -30, -50],
]

# 8. 放置动作
POSES_PLACE = [
    [-90, -30, 20, -30, -30, -50],
    [-90, -35, 25, -35, -30, 0],
    [-90, -30, 20, -30, -30, 0],
]

# 9. 满意地休息
POSES_REST_AGAIN = [
    [-90, -30, 20, -30, -30, 0],
    [-45, -40, 40, -45, -15, -20],
    [0, -60, 60, -60, 0, -35],
    [0, -80, 80, -60, 0, -50],
]


class RobotDancer:
    """机械舞者 - 带平滑轨迹控制"""

    def __init__(self, port: str):
        self.port = port
        self.port_handler = None
        self.packet_handler = None
        self.connected = False

    def connect(self):
        """连接硬件"""
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)

        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(1000000):
            raise ConnectionError(f"无法设置波特率")

        print(f"{C.GREEN}[OK]{C.RESET} 已连接到 {self.port}")
        self.connected = True

    def disconnect(self):
        """断开连接"""
        if self.port_handler:
            try:
                for motor_id in MOTOR_IDS:
                    self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)
                time.sleep(0.1)
                self.port_handler.closePort()
            except:
                pass
        self.connected = False

    def enable_torque(self, motor_id: int):
        if self.connected:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)

    def write_position(self, motor_id: int, position: int):
        if self.connected:
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, 42, position)

    def move_to(self, angles: list, duration: float = 0.5):
        """直接移动到目标角度"""
        if not self.connected:
            return

        positions = positions_from_angles(angles)
        for motor_id, pos in zip(MOTOR_IDS, positions):
            self.write_position(motor_id, pos)
        time.sleep(duration)

    def move_smooth(self, start_angles: list, end_angles: list,
                   duration: float = 1.0, desc: str = ""):
        """S 曲线平滑移动

        使用 S 曲线插值实现平滑的加减速效果
        优化步数和延迟以减少抖动
        """
        if not self.connected:
            return

        # 根据移动距离动态调整步数 - 减少步数避免指令堆积
        max_diff = max(abs(e - s) for s, e in zip(start_angles, end_angles))
        steps = int(max_diff / 5) + 8  # 减少步数，每5度一步
        steps = min(max(steps, 8), 30)  # 限制在 8-30 步之间

        # 增加基础延迟，给电机足够响应时间
        base_delay = 0.04  # 40ms 基础延迟
        step_delay = max(duration / steps, base_delay)

        for i in range(steps + 1):
            t_raw = i / steps
            t = scurve(t_raw)  # S 曲线平滑

            current = interpolate_pose(start_angles, end_angles, t)
            positions = positions_from_angles(current)

            # 发送位置指令
            for motor_id, pos in zip(MOTOR_IDS, positions):
                self.write_position(motor_id, pos)

            # 确保指令发送完成后再延迟
            time.sleep(step_delay)

        if desc:
            joint_str = " ".join([f"{a:6.1f}" for a in end_angles])
            print(f"  {C.DIM}→ {desc}{C.RESET}  [{joint_str}]")

    def set_gripper(self, open_state: bool = True):
        """设置夹爪"""
        pos = GRIPPER_OPEN if open_state else GRIPPER_CLOSE
        self.write_position(6, pos)
        state_str = f"{C.GREEN}张开{C.RESET}" if open_state else f"{C.YELLOW}闭合{C.RESET}"
        print(f"  {C.DIM}→ 夹爪{state_str}{C.RESET}")
        time.sleep(0.3)

    # ============================================================================
    # 舞蹈动作
    # ============================================================================

    def act_resting(self):
        """盘着休息"""
        print(f"\n{C.BLUE}{C.BOLD}【场景 1: 休息中】{C.RESET}")
        print(f"{C.DIM}机械臂盘着身子，静静地等待着...{C.RESET}")
        self.move_to(POSE_RESTING, 1.0)

    def act_wake_up(self):
        """苏醒"""
        print(f"\n{C.CYAN}{C.BOLD}【场景 2: 苏醒】{C.RESET}")
        print(f"{C.DIM}慢慢地...睁开眼睛...伸展身体...{C.RESET}")

        for i in range(len(POSES_WAKE_UP) - 1):
            self.move_smooth(
                POSES_WAKE_UP[i],
                POSES_WAKE_UP[i + 1],
                duration=2.0,  # 更慢
                desc=f"伸展 {i+1}"
            )

        print(f"  {C.GREEN}★ 机械臂苏醒了！{C.RESET}")
        time.sleep(0.5)

    def act_looking_around(self):
        """观察四周"""
        print(f"\n{C.CYAN}{C.BOLD}【场景 3: 寻找】{C.RESET}")
        print(f"{C.DIM}机械臂好奇地观察着四周...{C.RESET}")

        for i in range(len(POSES_LOOKING) - 1):
            self.move_smooth(
                POSES_LOOKING[i],
                POSES_LOOKING[i + 1],
                duration=1.2,  # 更慢
                desc=""  # 观察时不打印每一步
            )

    def act_discover(self):
        """发现目标"""
        print(f"\n{C.YELLOW}{C.BOLD}【场景 4: 发现】{C.RESET}")
        print(f"{C.DIM}等等...那是什么？{C.RESET}")

        for i in range(len(POSES_DISCOVER) - 1):
            self.move_smooth(
                POSES_DISCOVER[i],
                POSES_DISCOVER[i + 1],
                duration=1.5,  # 更慢
                desc="凑近观察" if i == 1 else ""
            )

        print(f"  {C.YELLOW}★ 发现了绿色水瓶！{C.RESET}")
        time.sleep(0.5)

    def act_prepare_grab(self):
        """准备抓取"""
        print(f"\n{C.YELLOW}{C.BOLD}【场景 5: 准备】{C.RESET}")
        print(f"{C.DIM}调整姿态，准备抓取...{C.RESET}")

        self.move_smooth(
            POSES_DISCOVER[-1],
            POSE_PREPARE,
            duration=2.5,  # 更慢
            desc="就位"
        )

    def act_grab(self):
        """抓取"""
        print(f"\n{C.GREEN}{C.BOLD}【场景 6: 抓取】{C.RESET}")
        print(f"{C.DIM}小心翼翼地...{C.RESET}")

        # 张开夹爪
        self.move_smooth(POSES_GRAB[0], POSES_GRAB[0], duration=0.5)
        self.set_gripper(True)

        # 下探
        self.move_smooth(POSES_GRAB[0], POSES_GRAB[1], duration=1.5, desc="下探")

        # 闭合夹爪
        self.move_smooth(POSES_GRAB[1], POSES_GRAB[2], duration=0.8, desc="抓住！")
        self.set_gripper(False)

        print(f"  {C.GREEN}★ 抓住了！{C.RESET}")
        time.sleep(0.5)

    def act_lift_and_move(self):
        """抬起并移动"""
        print(f"\n{C.GREEN}{C.BOLD}【场景 7: 移动】{C.RESET}")
        print(f"{C.DIM}抓着水瓶，移动到放置点...{C.RESET}")

        for i in range(len(POSES_LIFT_MOVE) - 1):
            self.move_smooth(
                POSES_LIFT_MOVE[i],
                POSES_LIFT_MOVE[i + 1],
                duration=2.0,  # 更慢
                desc=f"移动 {i+1}"
            )

    def act_place(self):
        """放置"""
        print(f"\n{C.GREEN}{C.BOLD}【场景 8: 放置】{C.RESET}")
        print(f"{C.DIM}轻轻地...放下...{C.RESET}")

        self.move_smooth(POSES_PLACE[0], POSES_PLACE[1], duration=1.2, desc="张开")
        self.set_gripper(True)
        self.move_smooth(POSES_PLACE[1], POSES_PLACE[2], duration=0.8, desc="松开")

        print(f"  {C.GREEN}★ 放置完成！{C.RESET}")
        time.sleep(0.5)

    def act_rest_again(self):
        """再次休息"""
        print(f"\n{C.BLUE}{C.BOLD}【场景 9: 满意的休息】{C.RESET}")
        print(f"{C.DIM}任务完成...盘起来休息...{C.RESET}")

        for i in range(len(POSES_REST_AGAIN) - 1):
            self.move_smooth(
                POSES_REST_AGAIN[i],
                POSES_REST_AGAIN[i + 1],
                duration=2.0,  # 更慢
                desc=f"盘起来 {i+1}"
            )

        print(f"\n  {C.BLUE}★ 机械臂满意地睡着了...{C.RESET}")

    def perform_full_story(self):
        """表演完整故事"""
        print(f"\n{C.BOLD}{C.CYAN}{'═' * 60}{C.RESET}")
        print(f"{C.BOLD}{C.CYAN}        机械臂之舞 - 一个完整的故事{C.RESET}")
        print(f"{C.BOLD}{C.CYAN}{'═' * 60}{C.RESET}")

        self.act_resting()
        time.sleep(1)

        self.act_wake_up()
        time.sleep(1)

        self.act_looking_around()
        time.sleep(0.5)

        self.act_discover()
        time.sleep(0.5)

        self.act_prepare_grab()
        time.sleep(0.3)

        self.act_grab()
        time.sleep(0.3)

        self.act_lift_and_move()
        time.sleep(0.3)

        self.act_place()
        time.sleep(0.5)

        self.act_rest_again()

        print(f"\n{C.BOLD}{C.GREEN}{'═' * 60}{C.RESET}")
        print(f"{C.BOLD}{C.GREEN}                  表演结束！{C.RESET}")
        print(f"{C.BOLD}{C.GREEN}{'═' * 60}{C.RESET}\n")


def main():
    print(f"\n{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  机械臂之舞 - 苏醒·寻找·抓取·放置·休息{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  (S曲线平滑轨迹){C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}\n")

    dancer = RobotDancer(PORT)

    try:
        dancer.connect()

        # 检测电机
        print(f"\n{C.CYAN}[检测电机]{C.RESET}")
        for motor_id in MOTOR_IDS:
            _, result, _ = dancer.packet_handler.ping(dancer.port_handler, motor_id)
            status = f"{C.GREEN}在线{C.RESET}" if result == 0 else f"{C.RED}离线{C.RESET}"
            print(f"  Motor {motor_id}: {status}")

        # 启用扭矩
        print(f"\n{C.YELLOW}[INFO]{C.RESET} 启用电机扭矩...")
        for motor_id in MOTOR_IDS:
            dancer.enable_torque(motor_id)
        time.sleep(0.5)

        input(f"\n{C.DIM}按 Enter 开始表演，Ctrl+C 紧急停止...{C.RESET}\n")

        dancer.perform_full_story()

    except KeyboardInterrupt:
        print(f"\n\n{C.YELLOW}[中断]{C.RESET} 表演被中断")
    except Exception as e:
        print(f"\n{C.RED}[ERROR]{C.RESET} {e}")
        import traceback
        traceback.print_exc()
    finally:
        dancer.disconnect()
        print(f"\n{C.CYAN}[再见]{C.RESET}")


if __name__ == "__main__":
    main()
