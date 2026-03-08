#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂实时坐标系显示
读取当前关节角度，计算并显示末端坐标系
"""

import time
import sys
import os
import numpy as np
from threading import Thread
import traceback

# Windows 启用 ANSI 转义序列支持
if os.name == 'nt':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from SDK import PortHandler, PacketHandler

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d import Axes3D
    IK_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] 可视化库未安装: {e}")
    print("请运行: pip install roboticstoolbox-python spatialmath-python matplotlib")
    IK_AVAILABLE = False
    sys.exit(1)


# ───────── 常量 ─────────
SERVO_CENTER = 2047
SERVO_RANGE = 270.0

# 电机 ID 映射
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = ['rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper']

# DH 参数
DH_PARAMS = [
    (0.0,    np.pi/2,  0.0165,  np.pi/2),     # J1: rotation
    (0.0,    -np.pi/2, 0.1478,  0.0),         # J2: pitch
    (0.0,    0.0,      0.14057, 0.0),         # J3: elbow
    (0.0,    0.0,      0.1349,  0.0),         # J4: wrist_pitch
    (0.0,    np.pi/2,  0.0,     0.0),         # J5: wrist_roll
]


def position_to_angle(pos: int, center: int = 2047, range_deg: float = 270) -> float:
    """舵机位置转角度"""
    return (pos - center) * (range_deg / 4095)


class RobotMonitor:
    """机械臂实时监控器"""

    def __init__(self, port: str = "COM7", baudrate: int = 1000000, simulation: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.running = False
        self.simulation = simulation  # 模拟模式

        # 创建运动学模型
        self.robot = rtb.DHRobot(
            [rtb.RevoluteDH(a=row[0], alpha=row[1], d=row[2], offset=row[3])
             for row in DH_PARAMS],
            name="SO100_DH",
            tool=SE3(0, 0, 0.0601)
        )

        # 当前状态
        self.current_angles = [0.0, 10.0, -20.0, 15.0, 0.0]  # 默认姿态
        self.end_effector_pose = SE3()
        self.sim_angle = 0.0  # 模拟模式的动画角度

    def connect(self):
        """连接硬件"""
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)

        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"无法设置波特率 {self.baudrate}")

        print(f"[OK] 已连接到 {self.port}")

    def enable_torque(self, motor_id: int):
        """启用单个电机扭矩"""
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, 40, 1)

    def disable_torque(self, motor_id: int):
        """禁用单个电机扭矩"""
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, 40, 0)

    def get_torque_state(self, motor_id: int) -> bool:
        """读取电机扭矩状态"""
        data, result, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, 40)
        return data == 1 if result == 0 else False

    def enable_all_torque(self):
        """启用所有电机扭矩"""
        for motor_id in MOTOR_IDS:
            self.enable_torque(motor_id)
        print("[TORQUE] 所有电机已锁定")

    def disable_all_torque(self):
        """禁用所有电机扭矩（可手动拖动）"""
        for motor_id in MOTOR_IDS:
            self.disable_torque(motor_id)
        print("[TORQUE] 所有电机已释放 - 可以手动拖动机械臂")

    def disconnect(self):
        """断开连接（断开前启用扭矩防止机械臂掉落）"""
        self.running = False
        if self.port_handler:
            # 先启用扭矩防止机械臂掉落
            for motor_id in MOTOR_IDS:
                self.enable_torque(motor_id)
            time.sleep(0.1)
            self.port_handler.closePort()
            print("[OK] 已断开连接 (扭矩已恢复)")

    def read_joint_positions(self) -> list:
        """读取所有关节位置"""
        if self.simulation:
            # 模拟模式：生成正弦波运动
            import time
            t = time.time()
            return [
                2047 + int(500 * np.sin(t)),           # J1
                2047 + int(300 * np.sin(t * 0.7)),     # J2
                2047 + int(400 * np.sin(t * 0.5)),     # J3
                2047 + int(200 * np.sin(t * 1.2)),     # J4
                2047 + int(150 * np.sin(t * 0.8)),     # J5
            ]

        positions = []
        for motor_id in MOTOR_IDS[:5]:  # 只读前5个关节
            try:
                data, result, _ = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, 56)
                positions.append(data if result == 0 else 2047)
            except Exception as e:
                print(f"[WARN] 读取 Motor {motor_id} 失败: {e}")
                positions.append(2047)
        # 确保返回5个值
        while len(positions) < 5:
            positions.append(2047)
        return positions

    def update(self):
        """更新当前状态"""
        try:
            positions = self.read_joint_positions()
            # 转换为角度
            self.current_angles = [position_to_angle(p) for p in positions]

            # 确保有5个角度值
            if len(self.current_angles) != 5:
                print(f"[WARN] 角度数量错误: {len(self.current_angles)}，使用默认值")
                self.current_angles = [0.0] * 5

            # 计算正向运动学
            q = np.array(self.current_angles)  # 确保是 numpy 数组
            q_rad = np.deg2rad(q)
            self.end_effector_pose = self.robot.fkine(q_rad)

            return list(self.current_angles), self.end_effector_pose
        except Exception as e:
            print(f"[ERROR] 更新失败: {e}")
            import traceback
            traceback.print_exc()
            # 返回默认值
            return [0.0, 10.0, -20.0, 15.0, 0.0], SE3()

    def get_coordinate_frame(self) -> tuple:
        """获取末端坐标系信息"""
        pose = self.end_effector_pose
        origin = pose.t  # 原点位置
        R = pose.R       # 旋转矩阵

        # 坐标轴方向
        x_axis = R[:, 0]  # X轴
        y_axis = R[:, 1]  # Y轴
        z_axis = R[:, 2]  # Z轴

        return origin, x_axis, y_axis, z_axis

    def print_status(self):
        """打印当前状态（紧凑模式，最小化重绘）"""
        angles, pose = self.update()
        rpy = np.rad2deg(pose.rpy())
        R = pose.R

        # 单行格式，使用 \r 覆盖当前行
        status = (
            f"Pos:({pose.t[0]:.3f},{pose.t[1]:.3f},{pose.t[2]:.3f})m | "
            f"RPY:({rpy[0]:.0f},{rpy[1]:.0f},{rpy[2]:.0f})° | "
            f"J:({angles[0]:.0f},{angles[1]:.0f},{angles[2]:.0f},{angles[3]:.0f},{angles[4]:.0f})°"
        )
        print("\r" + status + " " * 20, end="", flush=True)


class Visualizer:
    """3D 可视化器"""

    def __init__(self, monitor: RobotMonitor):
        self.monitor = monitor
        self.frame_count = 0

        # 创建图形
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # 设置样式
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('SO100 End Effector Coordinate Frame')

        # 设置坐标轴范围
        limit = 0.5
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, 0.6)

    def update_frame(self, frame):
        """更新动画帧 - 每次清除重绘"""
        try:
            # 清除上一帧
            self.ax.cla()

            # 重新设置坐标轴
            limit = 0.5
            self.ax.set_xlim(-limit, limit)
            self.ax.set_ylim(-limit, limit)
            self.ax.set_zlim(0, 0.6)
            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')
            self.ax.set_zlabel('Z (m)')

            # 绘制参考平面 (地面)
            xx, yy = np.meshgrid(np.linspace(-limit, limit, 5),
                                 np.linspace(-limit, limit, 5))
            zz = np.zeros_like(xx)
            self.ax.plot_surface(xx, yy, zz, alpha=0.1, color='gray')

            # 更新监控数据
            angles, pose = self.monitor.update()

            # 调试输出
            if self.frame_count == 0:
                print(f"[DEBUG] angles: {angles}")
                print(f"[DEBUG] pose.t: {pose.t}")
            self.frame_count += 1

            # 确保 angles 是长度为5的列表
            if not isinstance(angles, (list, tuple)):
                angles = [angles]
            angles = list(angles)[:5]
            while len(angles) < 5:
                angles.append(0.0)

            # 计算所有关节位置 (使用与静态测试相同的方法)
            points = [[0, 0, 0]]  # 基座
            q = np.deg2rad(angles)

            # 对于每个关节，创建一个全零数组，只设置前i+1个角度
            for i in range(5):
                q_work = np.zeros(5)
                q_work[:i+1] = q[:i+1]
                pt = self.monitor.robot.fkine(q_work).t
                points.append([pt[0], pt[1], pt[2]])

            points = np.array(points)

            # 绘制机械臂连杆
            self.ax.plot(points[:, 0], points[:, 1], points[:, 2],
                        'o-', linewidth=4, markersize=8, color='steelblue', label='Robot Arm')

            # 绘制末端坐标系
            origin = pose.t
            R = pose.R
            axis_length = 0.1

            # X轴 (红色)
            x_end = origin + R[:, 0] * axis_length
            self.ax.plot([origin[0], x_end[0]], [origin[1], x_end[1]], [origin[2], x_end[2]],
                        'r-', linewidth=3, label='X')

            # Y轴 (绿色)
            y_end = origin + R[:, 1] * axis_length
            self.ax.plot([origin[0], y_end[0]], [origin[1], y_end[1]], [origin[2], y_end[2]],
                        'g-', linewidth=3, label='Y')

            # Z轴 (蓝色)
            z_end = origin + R[:, 2] * axis_length
            self.ax.plot([origin[0], z_end[0]], [origin[1], z_end[1]], [origin[2], z_end[2]],
                        'b-', linewidth=3, label='Z')

            # 添加标题显示当前状态
            rpy = np.rad2deg(pose.rpy())
            title = f"Pos:({pose.t[0]:.2f},{pose.t[1]:.2f},{pose.t[2]:.2f}) RPY:({rpy[0]:.0f},{rpy[1]:.0f},{rpy[2]:.0f})°"
            self.ax.set_title(title)

        except Exception as e:
            print(f"[ERROR] {e}")
            import traceback
            traceback.print_exc()

    def show(self, interval: float = 0.1):
        """显示动画"""
        anim = FuncAnimation(self.fig, self.update_frame, interval=interval*1000,
                             blit=False, cache_frame_data=False)
        plt.show()


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="SO100 机械臂实时坐标系显示",
        epilog="""
示例:
  # 模拟模式（无需硬件，测试可视化）
  python realtime_coordinate_frame.py --mode visual --simulation

  # 文本模式监控
  python realtime_coordinate_frame.py --mode text

  # 3D 可视化 + 自由拖动
  python realtime_coordinate_frame.py --mode visual --freedrag
        """
    )
    parser.add_argument('--port', type=str, default='COM7', help='串口')
    parser.add_argument('--mode', type=str, choices=['text', 'visual', 'snapshot'],
                        default='text', help='显示模式 (snapshot=保存单帧图片)')
    parser.add_argument('--interval', type=float, default=0.1,
                        help='刷新间隔 (秒)')
    parser.add_argument('--freedrag', action='store_true',
                        help='释放扭矩，允许手动拖动机械臂')
    parser.add_argument('--simulation', '--sim', action='store_true',
                        help='模拟模式（无需硬件）')
    parser.add_argument('--output', type=str, default='robot_snapshot.png',
                        help='snapshot 模式输出图片路径')

    args = parser.parse_args()

    # 创建监控器
    monitor = RobotMonitor(args.port, simulation=args.simulation)

    try:
        monitor.connect()

        # 检测电机
        print("[检测电机]")
        for motor_id in MOTOR_IDS:
            _, result, _ = monitor.packet_handler.ping(monitor.port_handler, motor_id)
            status = "OK" if result == 0 else "NG"
            print(f"  Motor {motor_id}: {status}")

        # 显示当前扭矩状态
        print("\n[扭矩状态]")
        torque_states = []
        for motor_id in MOTOR_IDS[:5]:  # 只检查前5个关节
            is_enabled = monitor.get_torque_state(motor_id)
            state_str = "锁定" if is_enabled else "释放"
            torque_states.append(is_enabled)
            print(f"  Motor {motor_id}: {state_str}")

        any_locked = any(torque_states)
        if any_locked:
            print("\n  [警告] 部分或全部电机处于锁定状态")
            print("  如需手动拖动，请使用 --freedrag 参数")
        else:
            print("\n  [OK] 所有电机已释放，可以手动拖动")
        print()

        # 自由拖动模式 - 释放扭矩
        if args.freedrag:
            print("=" * 50)
            print("  自由拖动模式")
            print("=" * 50)
            print()
            monitor.disable_all_torque()
            print()
            print("现在可以手动拖动机械臂")
            print("移动机械臂时，坐标系将实时更新")
            print("(Ctrl+C 退出)")
            print()

        if args.mode == 'text':
            # 文本模式
            print("=" * 60)
            print("  SO100 机械臂实时坐标系监控 (紧凑模式)")
            print("=" * 60)
            print("\n显示格式:")
            print("  Pos: (X, Y, Z) 位置 (米)")
            print("  RPY: (Roll, Pitch, Yaw) 姿态 (度)")
            print("  J:   (J1, J2, J3, J4, J5) 关节角度 (度)")
            print()
            if args.freedrag:
                print("[扭矩已释放] 可以手动拖动机械臂")
            print("\n实时更新中... (Ctrl+C 退出)\n")

            monitor.running = True
            try:
                while monitor.running:
                    monitor.print_status()
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                monitor.running = False
                print("\r" + " " * 100 + "\r", end="")  # 清除行
                print("\n[退出]")

        elif args.mode == 'snapshot':
            # 快照模式 - 保存单帧图片
            print(f"\n生成当前状态快照...")
            print(f"保存到: {args.output}")

            viz = Visualizer(monitor)
            viz.update_frame(0)  # 绘制一帧
            plt.savefig(args.output, dpi=100, bbox_inches='tight')
            print(f"[OK] 快照已保存")

        else:
            # 3D 可视化模式
            print("=" * 60)
            print("  3D 可视化模式")
            print("=" * 60)
            if args.freedrag:
                print("\n[扭矩已释放] 可以手动拖动机械臂")
                print("拖动机械臂时，3D视图将实时更新")
            else:
                print("\n[注意] 电机处于锁定状态")
                print("如需手动拖动，请添加 --freedrag 参数")
            print("\n关闭 3D 窗口退出...\n")

            viz = Visualizer(monitor)
            viz.show(interval=args.interval)

    except KeyboardInterrupt:
        print("\n[退出]")
    except Exception as e:
        print(f"[ERROR] {e}")
        traceback.print_exc()
    finally:
        monitor.disconnect()


if __name__ == "__main__":
    main()
