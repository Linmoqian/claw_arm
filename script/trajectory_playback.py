#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
轨迹回放 - 平滑优化版本
使用轨迹平滑、速度限制和S曲线插值消除抖动
"""

import sys
import os
import time
import json
import numpy as np
from scipy.interpolate import CubicSpline

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

# 运动限制（度/秒）
MAX_VELOCITY = [60, 80, 100, 80, 120, 180]  # 每个关节的最大速度
MAX_ACCELERATION = [120, 160, 200, 160, 240, 360]  # 最大加速度


def angle_to_position(angle_deg: float) -> int:
    """角度转舵机位置"""
    pos = int(SERVO_CENTER + (angle_deg * 4095 / SERVO_RANGE))
    return max(0, min(4095, pos))


class TrajectorySmoother:
    """轨迹平滑器"""

    def __init__(self):
        pass

    def moving_average(self, trajectory: list, window_size: int = 5) -> list:
        """移动平均平滑

        Args:
            trajectory: 原始轨迹
            window_size: 窗口大小

        Returns:
            平滑后的轨迹
        """
        if len(trajectory) < window_size:
            return trajectory

        smoothed = []
        half_window = window_size // 2

        for i in range(len(trajectory)):
            # 计算窗口范围
            start = max(0, i - half_window)
            end = min(len(trajectory), i + half_window + 1)

            # 对窗口内的角度求平均
            angles_sum = [0.0] * 6
            for j in range(start, end):
                for k in range(6):
                    angles_sum[k] += trajectory[j]['angles'][k]

            count = end - start
            avg_angles = [angles_sum[k] / count for k in range(6)]

            # 创建新帧
            new_frame = {
                'time': trajectory[i]['time'],
                'frame': trajectory[i]['frame'],
                'angles': avg_angles,
                'positions': trajectory[i].get('positions', {})
            }
            smoothed.append(new_frame)

        return smoothed

    def resample_trajectory(self, trajectory: list, target_fps: float = 30) -> list:
        """重采样轨迹到固定帧率

        使用三次样条插值
        """
        if len(trajectory) < 4:
            return trajectory

        # 提取时间和角度
        times = [f['time'] for f in trajectory]
        angles_list = [[f['angles'][i] for f in trajectory] for i in range(6)]

        # 创建新的时间序列
        total_time = times[-1]
        new_times = np.arange(0, total_time, 1.0 / target_fps)

        resampled = []

        # 对每个关节进行样条插值
        try:
            splines = []
            for i in range(6):
                # 使用三次样条插值
                spline = CubicSpline(times, angles_list[i], bc_type='natural')
                splines.append(spline)

            # 生成新的轨迹点
            for t in new_times:
                new_angles = [float(spline(t)) for spline in splines]

                new_frame = {
                    'time': float(t),
                    'frame': len(resampled),
                    'angles': new_angles,
                    'positions': {}
                }
                resampled.append(new_frame)

            return resampled

        except Exception as e:
            # 样条插值失败，返回原始轨迹
            print(f"{C.YELLOW}[WARN]{C.RESET} 样条插值失败: {e}")
            return trajectory

    def limit_velocity(self, trajectory: list, max_vel: list) -> list:
        """速度限制

        确保相邻帧之间的速度不超过最大值
        """
        if len(trajectory) < 2:
            return trajectory

        result = [trajectory[0]]

        for i in range(1, len(trajectory)):
            prev = result[-1]
            curr = trajectory[i]

            dt = curr['time'] - prev['time']
            if dt <= 0:
                dt = 0.001  # 避免除零

            # 计算每个关节的速度
            constrained_angles = []
            modified = False

            for j in range(6):
                prev_angle = prev['angles'][j]
                curr_angle = curr['angles'][j]

                velocity = abs(curr_angle - prev_angle) / dt

                if velocity > max_vel[j]:
                    # 限制速度
                    max_change = max_vel[j] * dt
                    if curr_angle > prev_angle:
                        new_angle = prev_angle + max_change
                    else:
                        new_angle = prev_angle - max_change
                    constrained_angles.append(new_angle)
                    modified = True
                else:
                    constrained_angles.append(curr_angle)

            new_frame = {
                'time': curr['time'],
                'frame': curr['frame'],
                'angles': constrained_angles,
                'positions': curr.get('positions', {})
            }
            result.append(new_frame)

        return result

    def scurve_profile(self, start: float, end: float, t: float) -> float:
        """S曲线速度规划

        Args:
            start: 起始值
            end: 目标值
            t: 归一化时间 [0, 1]

        Returns:
            插值后的值
        """
        # 使用平滑step函数: 3t² - 2t³
        smooth_t = t * t * (3 - 2 * t)
        return start + (end - start) * smooth_t


class TrajectoryPlayer:
    """轨迹回放器 - 优化版"""

    def __init__(self, port: str):
        self.port = port
        self.port_handler = None
        self.packet_handler = None
        self.connected = False
        self.smoother = TrajectorySmoother()

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

    def move_to_angles(self, angles: list, duration: float = 0.05):
        """移动到目标角度"""
        if not self.connected:
            return

        positions = [angle_to_position(a) for a in angles]

        for motor_id, pos in zip(MOTOR_IDS, positions):
            self.write_position(motor_id, pos)

        time.sleep(duration)

    def preprocess_trajectory(self, trajectory: list,
                             smooth_window: int = 3,
                             resample_fps: int = 30,
                             limit_velocity: bool = True) -> list:
        """预处理轨迹

        1. 移动平均平滑
        2. 重采样到固定帧率
        3. 速度限制
        """
        print(f"{C.CYAN}[预处理]{C.RESET} 原始轨迹 {len(trajectory)} 帧")

        # 1. 移动平均平滑
        if smooth_window > 1:
            trajectory = self.smoother.moving_average(trajectory, smooth_window)
            print(f"  移动平均平滑: {smooth_window} 点窗口")

        # 2. 重采样
        if resample_fps > 0:
            trajectory = self.smoother.resample_trajectory(trajectory, resample_fps)
            print(f"  重采样: {resample_fps} Hz → {len(trajectory)} 帧")

        # 3. 速度限制
        if limit_velocity:
            trajectory = self.smoother.limit_velocity(trajectory, MAX_VELOCITY)
            print(f"  速度限制: 已应用")

        return trajectory

    def play_trajectory_smooth(self, trajectory: list, speed: float = 1.0,
                              interpolate_steps: int = 2):
        """平滑播放轨迹

        Args:
            trajectory: 轨迹数据
            speed: 播放速度倍数
            interpolate_steps: 每帧之间的插值步数
        """
        if not trajectory:
            return

        total_frames = len(trajectory)
        total_time = trajectory[-1]['time']
        estimated_duration = total_time / speed

        print(f"\n{C.CYAN}[播放]{C.RESET} {total_frames} 帧, 预计 {estimated_duration:.1f} 秒")

        # 先移动到起始位置
        print(f"{C.YELLOW}[准备]{C.RESET} 移动到起始位置...")
        start_angles = trajectory[0]['angles']
        self.move_to_angles(start_angles, 1.0)
        time.sleep(0.5)

        print(f"{C.GREEN}[开始]{C.RESET}\n")

        start_time = time.time()
        last_display_time = start_time

        # 预计算每帧之间的时间间隔
        for i in range(len(trajectory) - 1):
            current_frame = trajectory[i]
            next_frame = trajectory[i + 1]

            current_angles = current_frame['angles']
            next_angles = next_frame['angles']

            # 计算目标时间间隔
            target_dt = (next_frame['time'] - current_frame['time']) / speed

            # 计算实际需要的时间（考虑插值）
            actual_dt = target_dt / interpolate_steps

            # 检测是否有大跳跃
            max_diff = max(abs(next_angles[j] - current_angles[j]) for j in range(6))

            if max_diff > 50:
                # 大跳跃：使用更多插值步数
                steps = interpolate_steps * 3
            else:
                steps = interpolate_steps

            for step in range(steps + 1):
                t = step / steps
                # 使用 S 曲线插值
                smooth_t = t * t * (3 - 2 * t)

                interpolated = [
                    current_angles[j] + (next_angles[j] - current_angles[j]) * smooth_t
                    for j in range(6)
                ]

                # 移动到插值点
                positions = [angle_to_position(a) for a in interpolated]
                for motor_id, pos in zip(MOTOR_IDS, positions):
                    self.write_position(motor_id, pos)

                # 精确控制延迟
                time.sleep(actual_dt)

            # 更新显示
            now = time.time()
            if now - last_display_time >= 0.5:
                elapsed = now - start_time
                progress = (i + 1) / total_frames * 100
                a = next_angles
                joint_str = " | ".join([f"J{j+1}:{a[j]:5.1f}°" for j in range(6)])
                print(f"\r{C.DIM}[{elapsed:5.1f}s] {progress:5.1f}% {C.RESET}{joint_str}  ", end="", flush=True)
                last_display_time = now

        actual_duration = time.time() - start_time
        print(f"\n\n{C.GREEN}[完成]{C.RESET} 用时: {actual_duration:.1f} 秒")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="轨迹回放 - 平滑优化版")
    parser.add_argument('file', nargs='?', help='轨迹文件 (JSON)')
    parser.add_argument('--speed', type=float, default=1.0, help='播放速度倍数 (默认1.0)')
    parser.add_argument('--port', type=str, default='COM7', help='串口')
    parser.add_argument('--no-confirm', action='store_true', help='跳过确认直接播放')
    parser.add_argument('--smooth', type=int, default=3, help='平滑窗口大小 (默认3)')
    parser.add_argument('--fps', type=int, default=30, help='重采样帧率 (默认30)')
    parser.add_argument('--interpolate', type=int, default=3, help='插值步数 (默认3，越大越平滑)')

    args = parser.parse_args()

    print(f"\n{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  轨迹回放 - 平滑优化版{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}\n")

    # 查找最新的轨迹文件
    if not args.file:
        import glob
        files = glob.glob(os.path.join(os.path.dirname(__file__), "teaching_record_*.json"))
        if files:
            files.sort()
            args.file = files[-1]
            print(f"{C.DIM}使用最新文件: {os.path.basename(args.file)}{C.RESET}\n")
        else:
            print(f"{C.RED}[ERROR]{C.RESET} 未找到轨迹文件")
            return

    # 读取轨迹文件
    print(f"{C.CYAN}[读取]{C.RESET} {args.file}")
    with open(args.file, 'r', encoding='utf-8') as f:
        data = json.load(f)

    print(f"{C.DIM}  时长: {data['duration_sec']}秒 | 帧数: {data['total_frames']} → {data['compressed_frames']}{C.RESET}")

    # 使用压缩轨迹
    trajectory = data['compressed_trajectory']

    player = TrajectoryPlayer(args.port)

    try:
        player.connect()

        # 检测电机
        print(f"\n{C.CYAN}[检测电机]{C.RESET}")
        for motor_id in MOTOR_IDS:
            _, result, _ = player.packet_handler.ping(player.port_handler, motor_id)
            status = f"{C.GREEN}在线{C.RESET}" if result == 0 else f"{C.RED}离线{C.RESET}"
            print(f"  Motor {motor_id}: {status}")

        # 启用扭矩
        print(f"\n{C.YELLOW}[INFO]{C.RESET} 启用电机扭矩...")
        for motor_id in MOTOR_IDS:
            player.enable_torque(motor_id)
        time.sleep(0.5)

        # 预处理轨迹
        trajectory = player.preprocess_trajectory(
            trajectory,
            smooth_window=args.smooth,
            resample_fps=args.fps
        )

        if not args.no_confirm:
            input(f"\n{C.DIM}按 Enter 开始播放，Ctrl+C 紧急停止...{C.RESET}\n")

        # 播放轨迹
        player.play_trajectory_smooth(
            trajectory,
            speed=args.speed,
            interpolate_steps=args.interpolate
        )

    except KeyboardInterrupt:
        print(f"\n\n{C.YELLOW}[中断]{C.RESET} 播放被中断")
    except Exception as e:
        print(f"\n{C.RED}[ERROR]{C.RESET} {e}")
        import traceback
        traceback.print_exc()
    finally:
        player.disconnect()
        print(f"\n{C.CYAN}[再见]{C.RESET}")


if __name__ == "__main__":
    main()
