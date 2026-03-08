#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 运动学模型硬件演示
在真实机械上演示正运动学、逆运动学、轨迹规划等功能
"""

import sys
import os
import time
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from SDK import PortHandler, PacketHandler
from script.so100_kinematics_model import (
    SO100Kinematics,
    ServoConverter,
    URDFParameters
)

# Windows 终端颜色
if os.name == 'nt':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 2077)


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

# 蜷缩姿态 (安全待命姿态)
HOME_POSE = [0, -73, 63, -60, 0, -50]  # 从示教数据中提取的待命姿态


class HardwareController:
    """硬件控制器"""

    def __init__(self, port=PORT):
        self.port = port
        self.port_handler = None
        self.packet_handler = None
        self.connected = False

    def connect(self):
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)

        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(1000000):
            raise ConnectionError(f"无法设置波特率")

        print(f"{C.GREEN}[OK]{C.RESET} 已连接到 {self.port}")
        self.connected = True
        return self

    def disconnect(self):
        if self.port_handler:
            try:
                # 释放所有扭矩
                for motor_id in MOTOR_IDS:
                    self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 0)
                time.sleep(0.1)
                self.port_handler.closePort()
            except:
                pass
        self.connected = False

    def enable_torque(self, motor_id):
        if self.connected:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)

    def disable_torque(self, motor_id):
        if self.connected:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 0)

    def read_position(self, motor_id):
        if self.connected:
            data, result, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, 56
            )
            if result == 0:
                return data
        return None

    def write_position(self, motor_id, position):
        if self.connected:
            self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, 42, position
            )

    def move_to_angles(self, angles_deg, duration=1.0):
        """移动到指定角度(度)"""
        positions = [ServoConverter.angle_to_position(a) for a in angles_deg]

        steps = int(duration / 0.05)
        if steps < 1:
            steps = 1

        # 获取当前位置
        current_pos = []
        for motor_id in MOTOR_IDS:
            pos = self.read_position(motor_id)
            if pos is None:
                pos = 2047
            current_pos.append(pos)

        # 平滑移动
        for i in range(steps + 1):
            t = i / steps
            # S曲线
            smooth_t = t * t * (3 - 2 * t)

            for j, (curr, target) in enumerate(zip(current_pos, positions)):
                pos = int(curr + (target - curr) * smooth_t)
                self.write_position(MOTOR_IDS[j], pos)

            time.sleep(duration / steps)

    def set_gripper(self, open_state=True, duration=0.5):
        """控制夹爪"""
        if open_state:
            pos = 2800  # 张开
        else:
            pos = 1200  # 闭合

        steps = int(duration / 0.05)
        for i in range(steps + 1):
            t = i / steps
            smooth_t = t * t * (3 - 2 * t)
            current = self.read_position(6) or 2047
            p = int(current + (pos - current) * smooth_t)
            self.write_position(6, p)
            time.sleep(duration / steps)


def print_section(title):
    print(f"\n{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  {title}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}\n")


def demo1_forward_kinematics_live(model, hw):
    """演示1: 实时正运动学 - 读取当前关节角度，显示末端位置"""
    print_section("演示1: 实时正运动学")

    print("读取当前关节位置...")
    current_pos = []
    for motor_id in MOTOR_IDS:
        pos = hw.read_position(motor_id)
        if pos is not None:
            current_pos.append(pos)
        else:
            current_pos.append(2047)

    # 转换为角度
    current_angles = [ServoConverter.position_to_angle(p) for p in current_pos]
    current_rad = np.deg2rad(current_angles)

    print(f"\n当前关节角度 (度):")
    for i, angle in enumerate(current_angles):
        print(f"  J{i+1}: {angle:6.1f}°")

    # 计算正运动学
    pose = model.get_end_effector_pose(current_rad)
    pos = np.array(pose['position']) * 1000
    rpy = np.rad2deg(pose['orientation']['rpy'])

    print(f"\n{C.GREEN}末端执行器位姿:{C.RESET}")
    print(f"  位置 (mm): X={pos[0]:7.1f}, Y={pos[1]:7.1f}, Z={pos[2]:7.1f}")
    print(f"  姿态 (度): Roll={rpy[0]:6.1f}, Pitch={rpy[1]:6.1f}, Yaw={rpy[2]:6.1f}")


def demo2_presets(model, hw):
    """演示2: 预设姿态 - 移动到几个关键姿态"""
    print_section("演示2: 预设姿态演示")

    presets = {
        "蜷缩(待命)": HOME_POSE,
        "半伸展": [0, -30, 30, -30, 0, -30],
        "伸展": [0, -20, 20, -20, 0, -20],
        "右伸展": [45, -30, 30, -30, 0, -30],
        "左伸展": [-45, -30, 30, -30, 0, -30],
    }

    print("将依次移动到以下姿态:")
    for i, (name, angles) in enumerate(presets.items(), 1):
        print(f"  {i}. {name}: {angles}")

    input(f"\n{C.YELLOW}按 Enter 开始演示...{C.RESET}")

    for name, angles in presets.items():
        print(f"\n{C.CYAN}→ {name}{C.RESET}: {angles}")
        hw.move_to_angles(angles, duration=1.0)
        time.sleep(0.5)


def demo3_inverse_kinematics(model, hw):
    """演示3: 逆运动学 - 指定目标位置，机械臂移动过去"""
    print_section("演示3: 逆运动学")

    # 定义几个目标点
    targets = [
        ("正前方", [100, 50, 100]),
        ("右侧", [100, -50, 100]),
        ("左侧", [100, 150, 100]),
        ("高处", [50, 50, 150]),
    ]

    print("目标位置:")
    for i, (name, pos) in enumerate(targets, 1):
        print(f"  {i}. {name}: {pos} mm")

    input(f"\n{C.YELLOW}按 Enter 开始演示...{C.RESET}")

    # 获取当前姿态作为初始猜测
    current_pos = [hw.read_position(m) or 2047 for m in MOTOR_IDS]
    q0 = np.deg2rad([ServoConverter.position_to_angle(p) for p in current_pos])

    for name, target_mm in targets:
        print(f"\n{C.CYAN}→ 目标: {name} {target_mm} mm{C.RESET}")

        target_m = np.array(target_mm) / 1000

        # 求解逆运动学
        try:
            q_solution = model.inverse_kinematics(target_m, q0=q0, method='optimize')
            q_solution_deg = np.rad2deg(q_solution)

            # 检查限位
            limits = list(URDFParameters.JOINT_LIMITS_DEG.values())
            valid = True
            for i, (angle, (low, high)) in enumerate(zip(q_solution_deg, limits)):
                if not (low <= angle <= high):
                    print(f"  {C.RED}J{i+1}={angle:.1f}° 超出限位 [{low}, {high}]{C.RESET}")
                    valid = False

            if valid:
                print(f"  求解角度(度): {q_solution_deg.round(1).tolist()}")

                # 移动
                hw.move_to_angles(q_solution_deg, duration=1.5)
                time.sleep(0.5)

                # 验证
                T = model.forward_kinematics(q_solution)
                actual_pos = T[:3, 3] * 1000
                error = np.linalg.norm(target_mm - actual_pos)
                print(f"  实际位置: {actual_pos.round(1).tolist()} mm")
                print(f"  误差: {error:.1f} mm")

                # 更新初始猜测
                q0 = q_solution
            else:
                print(f"  {C.YELLOW}跳过此点{C.RESET}")

        except Exception as e:
            print(f"  {C.RED}求解失败: {e}{C.RESET}")


def demo4_trajectory(model, hw):
    """演示4: 轨迹规划 - 平滑移动"""
    print_section("演示4: 轨迹规划演示")

    # 起点和终点
    start_deg = [0, -30, 30, -45, 0, -30]
    end_deg = [45, -60, 60, -90, 45, -60]

    print(f"起点: {start_deg}")
    print(f"终点: {end_deg}")

    input(f"\n{C.YELLOW}按 Enter 开始演示...{C.RESET}")

    # 生成轨迹
    start_rad = np.deg2rad(start_deg)
    end_rad = np.deg2rad(end_deg)

    num_points = 30
    traj = model.scurve_trajectory(start_rad, end_rad, num_points)

    print(f"\n{C.CYAN}执行S曲线轨迹 ({num_points}点)...{C.RESET}")

    for i, q_rad in enumerate(traj):
        q_deg = np.rad2deg(q_rad)
        positions = [ServoConverter.angle_to_position(a) for a in q_deg]

        for motor_id, pos in zip(MOTOR_IDS, positions):
            hw.write_position(motor_id, pos)

        # 显示进度
        if i % 5 == 0:
            progress = int((i / num_points) * 100)
            print(f"  进度: {progress:3d}%")

        time.sleep(0.05)

    print(f"  {C.GREEN}完成!{C.RESET}")
    time.sleep(0.5)

    # 回到起点
    print(f"\n{C.CYAN}返回起点...{C.RESET}")
    traj_return = model.scurve_trajectory(end_rad, start_rad, num_points)
    for q_rad in traj_return:
        q_deg = np.rad2deg(q_rad)
        positions = [ServoConverter.angle_to_position(a) for a in q_deg]
        for motor_id, pos in zip(MOTOR_IDS, positions):
            hw.write_position(motor_id, pos)
        time.sleep(0.05)
    print(f"  {C.GREEN}完成!{C.RESET}")


def demo5_pick_and_place(model, hw):
    """演示5: 抓取任务 - 完整的抓取放置流程"""
    print_section("演示5: 抓取任务演示")

    # 定义任务点
    grasp_pos = [80, 100, 50]   # 抓取点 mm
    place_pos = [-80, 100, 80]  # 放置点 mm
    lift_height = 100           # 抬起高度 mm

    print(f"抓取点: {grasp_pos} mm")
    print(f"放置点: {place_pos} mm")
    print(f"抬起高度: {lift_height} mm")

    input(f"\n{C.YELLOW}按 Enter 开始演示...{C.RESET}")

    # 待命 (蜷缩姿态)
    print(f"\n{C.CYAN}→ 待命姿态(蜷缩){C.RESET}")
    hw.move_to_angles(HOME_POSE, duration=1.0)
    time.sleep(0.5)

    # 张开夹爪
    print(f"  {C.DIM}张开夹爪{C.RESET}")
    hw.set_gripper(True, 0.3)

    # 移动到抓取点上方
    grasp_above = grasp_pos.copy()
    grasp_above[2] += lift_height
    print(f"\n{C.CYAN}→ 抓取点上方{C.RESET}")
    q = model.inverse_kinematics(np.array(grasp_above)/1000)
    hw.move_to_angles(np.rad2deg(q), duration=1.0)
    time.sleep(0.3)

    # 下降到抓取点
    print(f"{C.CYAN}→ 下降抓取{C.RESET}")
    q = model.inverse_kinematics(np.array(grasp_pos)/1000)
    hw.move_to_angles(np.rad2deg(q), duration=1.0)
    time.sleep(0.3)

    # 闭合夹爪
    print(f"  {C.GREEN}闭合夹爪{C.RESET}")
    hw.set_gripper(False, 0.5)
    time.sleep(0.3)

    # 抬起
    print(f"{C.CYAN}→ 抬起{C.RESET}")
    q = model.inverse_kinematics(np.array(grasp_above)/1000)
    hw.move_to_angles(np.rad2deg(q), duration=1.0)
    time.sleep(0.3)

    # 移动到放置点上方
    place_above = place_pos.copy()
    place_above[2] += lift_height
    print(f"\n{C.CYAN}→ 放置点上方{C.RESET}")
    q = model.inverse_kinematics(np.array(place_above)/1000)
    hw.move_to_angles(np.rad2deg(q), duration=1.5)
    time.sleep(0.3)

    # 下降
    print(f"{C.CYAN}→ 下降放置{C.RESET}")
    q = model.inverse_kinematics(np.array(place_pos)/1000)
    hw.move_to_angles(np.rad2deg(q), duration=1.0)
    time.sleep(0.3)

    # 张开夹爪
    print(f"  {C.GREEN}释放物体{C.RESET}")
    hw.set_gripper(True, 0.5)
    time.sleep(0.3)

    # 抬起
    print(f"{C.CYAN}→ 抬起{C.RESET}")
    q = model.inverse_kinematics(np.array(place_above)/1000)
    hw.move_to_angles(np.rad2deg(q), duration=1.0)
    time.sleep(0.3)

    # 回到待命
    print(f"\n{C.CYAN}→ 回到待命{C.RESET}")
    hw.move_to_angles(home, duration=1.0)

    print(f"\n{C.GREEN}抓取任务完成!{C.RESET}")


def demo6_gripper_test(model, hw):
    """演示6: 夹爪测试"""
    print_section("演示6: 夹爪测试")

    print("夹爪开合测试 (5次)")

    for i in range(5):
        print(f"\n第 {i+1} 次:")

        print(f"  {C.YELLOW}张开...{C.RESET}")
        hw.set_gripper(True, 0.4)
        time.sleep(0.5)

        print(f"  {C.GREEN}闭合...{C.RESET}")
        hw.set_gripper(False, 0.4)
        time.sleep(0.5)

    print(f"\n{C.GREEN}夹爪测试完成!{C.RESET}")


def demo7_workspace_scan(model, hw):
    """演示7: 工作空间扫描"""
    print_section("演示7: 工作空间扫描")

    print("机械臂将扫描工作空间边界...")

    input(f"\n{C.YELLOW}按 Enter 开始...{C.RESET}")

    # 定义扫描点
    scan_points = [
        [0, -30, 30, -30, 0, -30],      # 前
        [45, -30, 30, -30, 0, -30],     # 右前
        [90, -30, 30, -30, 0, -30],     # 右
        [45, -30, 30, -30, 0, -30],     # 右前
        [0, -30, 30, -30, 0, -30],      # 前
        [-45, -30, 30, -30, 0, -30],    # 左前
        [-90, -30, 30, -30, 0, -30],    # 左
        [-45, -30, 30, -30, 0, -30],    # 左前
        [0, -30, 30, -30, 0, -30],      # 前
        [0, -60, 60, -60, 0, -30],      # 高伸展
        [0, -30, 30, -30, 0, -30],      # 前
        [0, -80, 80, -90, 0, -30],      # 蜷缩
    ]

    for i, angles in enumerate(scan_points):
        print(f"\n点 {i+1}/12: {angles}")

        # 移动
        hw.move_to_angles(angles, duration=0.5)

        # 读取并显示末端位置
        current_pos = [hw.read_position(m) or 2047 for m in MOTOR_IDS]
        current_rad = np.deg2rad([ServoConverter.position_to_angle(p) for p in current_pos])
        pose = model.get_end_effector_pose(current_rad)
        pos = np.array(pose['position']) * 1000

        print(f"  末端位置: ({pos[0]:6.1f}, {pos[1]:6.1f}, {pos[2]:6.1f}) mm")
        print(f"  距基座: {np.linalg.norm(pos):6.1f} mm")

        time.sleep(0.3)

    # 回到蜷缩姿态
    print(f"\n{C.CYAN}回到蜷缩姿态...{C.RESET}")
    hw.move_to_angles(HOME_POSE, duration=1.0)


def main():
    print(f"\n{C.BOLD}{C.CYAN}{'╔' + '═'*58 + '╗'}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'║' + ' '*8}SO100 运动学模型 - 硬件演示{' '*22 + '║'}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'╚' + '═'*58 + '╝'}{C.RESET}")

    # 硬件连接
    print(f"\n{C.YELLOW}连接硬件...{C.RESET}")
    hw = HardwareController(PORT)

    try:
        hw.connect()

        # 检测电机
        print(f"\n{C.CYAN}[检测电机]{C.RESET}")
        online = []
        for motor_id in MOTOR_IDS:
            _, result, _ = hw.packet_handler.ping(hw.port_handler, motor_id)
            if result == 0:
                online.append(motor_id)
                print(f"  Motor {motor_id}: {C.GREEN}在线{C.RESET}")
            else:
                print(f"  Motor {motor_id}: {C.RED}离线{C.RESET}")

        if len(online) < 6:
            print(f"\n{C.RED}警告: 部分电机离线!{C.RESET}")

        # 启用扭矩
        print(f"\n{C.YELLOW}启用扭矩...{C.RESET}")
        for motor_id in MOTOR_IDS:
            hw.enable_torque(motor_id)
        time.sleep(0.5)

        # 创建运动学模型
        model = SO100Kinematics()

        # 演示菜单
        while True:
            print(f"\n{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
            print(f"{C.BOLD}{C.CYAN}  演示菜单{C.RESET}")
            print(f"{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
            print(f"  1. 实时正运动学 (读取当前位置，显示末端坐标)")
            print(f"  2. 预设姿态演示 (蜷缩、半伸展、伸展等)")
            print(f"  3. 逆运动学演示 (指定目标位置)")
            print(f"  4. 轨迹规划演示 (S曲线平滑移动)")
            print(f"  5. 抓取任务演示 (完整抓取流程)")
            print(f"  6. 夹爪测试")
            print(f"  7. 工作空间扫描")
            print(f"  8. 自由拖动模式 (释放扭矩)")
            print(f"  0. 退出")
            print()

            choice = input(f"{C.YELLOW}选择演示 (0-8): {C.RESET}").strip()

            if choice == '0':
                break
            elif choice == '1':
                demo1_forward_kinematics_live(model, hw)
            elif choice == '2':
                demo2_presets(model, hw)
            elif choice == '3':
                demo3_inverse_kinematics(model, hw)
            elif choice == '4':
                demo4_trajectory(model, hw)
            elif choice == '5':
                demo5_pick_and_place(model, hw)
            elif choice == '6':
                demo6_gripper_test(model, hw)
            elif choice == '7':
                demo7_workspace_scan(model, hw)
            elif choice == '8':
                print(f"\n{C.YELLOW}释放扭矩，现在可以手动拖动机械臂{C.RESET}")
                print(f"{C.DIM}按 Ctrl+C 退出拖动模式{C.RESET}")
                for motor_id in MOTOR_IDS:
                    hw.disable_torque(motor_id)
                try:
                    while True:
                        time.sleep(1)
                        # 显示当前位置
                        current_pos = [hw.read_position(m) for m in MOTOR_IDS]
                        angles = [ServoConverter.position_to_angle(p) for p in current_pos]
                        print(f"\r当前: [{angles[0]:5.1f}, {angles[1]:5.1f}, {angles[2]:5.1f}, "
                              f"{angles[3]:5.1f}, {angles[4]:5.1f}, {angles[5]:5.1f}]  ", end="", flush=True)
                except KeyboardInterrupt:
                    print(f"\n\n{C.YELLOW}重新启用扭矩...{C.RESET}")
                    for motor_id in MOTOR_IDS:
                        hw.enable_torque(motor_id)
                    time.sleep(0.5)
            else:
                print(f"{C.RED}无效选择{C.RESET}")

        # 回到蜷缩姿态
        print(f"\n{C.CYAN}回到蜷缩姿态...{C.RESET}")
        hw.move_to_angles(HOME_POSE, duration=1.5)

    except KeyboardInterrupt:
        print(f"\n\n{C.YELLOW}[中断]{C.RESET}")
        print(f"\n{C.CYAN}回到蜷缩姿态...{C.RESET}")
        hw.move_to_angles(HOME_POSE, duration=1.5)
    except Exception as e:
        print(f"\n{C.RED}[ERROR]{C.RESET} {e}")
        print(f"\n{C.CYAN}回到蜷缩姿态...{C.RESET}")
        try:
            hw.move_to_angles(HOME_POSE, duration=1.5)
        except:
            pass
        import traceback
        traceback.print_exc()
    finally:
        hw.disconnect()
        print(f"\n{C.CYAN}[再见]{C.RESET}")


if __name__ == "__main__":
    main()
