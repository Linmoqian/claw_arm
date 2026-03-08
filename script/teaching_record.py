#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
示教录制 - 连续轨迹录制
释放扭矩后以 24Hz 频率连续记录关节位置
按 Ctrl+C 退出并保存完整轨迹
"""

import sys
import os
import time
import json
from datetime import datetime

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

# 录制参数
SAMPLE_RATE = 24  # 每秒采样次数
SAMPLE_INTERVAL = 1.0 / SAMPLE_RATE  # 采样间隔 (秒)

# 关节名称
JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper"
}


def position_to_angle(pos: int) -> float:
    """位置转角度"""
    SERVO_CENTER = 2047
    SERVO_RANGE = 270.0
    return (pos - SERVO_CENTER) * SERVO_RANGE / 4095


def read_all_positions(port_handler, packet_handler):
    """读取所有关节位置"""
    positions = {}
    for motor_id in MOTOR_IDS:
        data, result, _ = packet_handler.read2ByteTxRx(port_handler, motor_id, 56)
        if result == 0:
            positions[motor_id] = data
    return positions


def positions_to_angles(positions: dict) -> list:
    """转换位置为角度列表 [J1, J2, J3, J4, J5, J6]"""
    angles = []
    for motor_id in MOTOR_IDS:
        if motor_id in positions:
            angles.append(round(position_to_angle(positions[motor_id]), 2))
        else:
            angles.append(0.0)
    return angles


def compress_trajectory(trajectory: list, threshold: float = 0.5) -> list:
    """压缩轨迹 - 移除变化小于阈值的数据点

    Args:
        trajectory: 原始轨迹数据
        threshold: 角度变化阈值（度），小于此值视为无变化

    Returns:
        压缩后的轨迹
    """
    if not trajectory:
        return trajectory

    compressed = [trajectory[0]]  # 保留第一个点

    for frame in trajectory[1:]:
        last = compressed[-1]
        # 检查任一关节变化是否超过阈值
        significant = False
        for i in range(6):
            if abs(frame['angles'][i] - last['angles'][i]) > threshold:
                significant = True
                break

        if significant:
            compressed.append(frame)

    # 总是保留最后一个点
    if trajectory[-1] != compressed[-1]:
        compressed.append(trajectory[-1])

    return compressed


def main():
    print(f"\n{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  示教轨迹录制 - 连续记录 ({SAMPLE_RATE}Hz){C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}\n")

    print(f"{C.CYAN}[操作说明]{C.RESET}")
    print(f"  1. 扭矩已释放，可以手动拖动机械臂")
    print(f"  2. 系统以 {C.GREEN}{SAMPLE_RATE}Hz{C.RESET} 频率连续记录位置")
    print(f"  3. 完成后按 {C.YELLOW}Ctrl+C{C.RESET} 退出并保存轨迹\n")

    port_handler = PortHandler(PORT)
    packet_handler = PacketHandler(0.0)

    # 轨迹数据
    trajectory = []
    start_time = None
    frame_count = 0

    try:
        if not port_handler.openPort():
            print(f"{C.RED}[ERROR]{C.RESET} 无法打开端口 {PORT}")
            input("按 Enter 退出...")
            return

        if not port_handler.setBaudRate(1000000):
            print(f"{C.RED}[ERROR]{C.RESET} 无法设置波特率")
            input("按 Enter 退出...")
            return

        print(f"{C.GREEN}[OK]{C.RESET} 已连接到 {PORT}")

        # 检测电机
        print(f"\n{C.CYAN}[检测电机]{C.RESET}")
        for motor_id in MOTOR_IDS:
            _, result, _ = packet_handler.ping(port_handler, motor_id)
            status = f"{C.GREEN}在线{C.RESET}" if result == 0 else f"{C.RED}离线{C.RESET}"
            print(f"  Motor {motor_id}: {status}")

        # 释放扭矩
        print(f"\n{C.YELLOW}[INFO]{C.RESET} 释放扭矩中...")
        for motor_id in MOTOR_IDS:
            packet_handler.write1ByteTxRx(port_handler, motor_id, 40, 0)
        print(f"{C.GREEN}[OK]{C.RESET} 所有电机已释放\n")

        print(f"{C.BOLD}{C.GREEN}开始录制...{C.RESET}")
        print(f"{C.DIM}实时显示中 (按 Ctrl+C 停止){C.RESET}\n")

        start_time = time.time()
        last_display_time = start_time

        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            # 读取位置
            positions = read_all_positions(port_handler, packet_handler)
            angles = positions_to_angles(positions)

            # 记录数据
            frame = {
                'time': round(elapsed, 3),
                'frame': frame_count,
                'positions': positions,
                'angles': angles
            }
            trajectory.append(frame)
            frame_count += 1

            # 每秒更新一次显示（避免闪烁）
            if current_time - last_display_time >= 0.5:
                joint_str = " | ".join([
                    f"J{i}: {angles[i-1]:6.1f}°"
                    for i in range(1, 7)
                ])
                print(f"\r{C.DIM}[{elapsed:5.1f}s] [{frame_count:4d}帧] {C.RESET}{joint_str}  ", end="", flush=True)
                last_display_time = current_time

            # 精确控制采样率
            time.sleep(SAMPLE_INTERVAL)

    except KeyboardInterrupt:
        elapsed = time.time() - start_time if start_time else 0

        print(f"\n\n{C.YELLOW}[INFO]{C.RESET} 录制停止")

        # 先恢复扭矩（端口还没关）
        try:
            for motor_id in MOTOR_IDS:
                packet_handler.write1ByteTxRx(port_handler, motor_id, 40, 1)
            time.sleep(0.1)
        except:
            pass

        # 关闭端口
        try:
            port_handler.closePort()
        except:
            pass

        print(f"{C.GREEN}[OK]{C.RESET} 扭矩已恢复")

        # 压缩轨迹
        print(f"\n{C.DIM}压缩轨迹中...{C.RESET}")
        compressed = compress_trajectory(trajectory, threshold=0.5)
        compression_ratio = len(compressed) / len(trajectory) * 100 if trajectory else 0

        # 保存记录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"teaching_record_{timestamp}.json"
        filepath = os.path.join(os.path.dirname(__file__), filename)

        record_data = {
            "timestamp": datetime.now().isoformat(),
            "sample_rate": SAMPLE_RATE,
            "duration_sec": round(elapsed, 2),
            "total_frames": len(trajectory),
            "compressed_frames": len(compressed),
            "compression_ratio": f"{compression_ratio:.1f}%",
            "raw_trajectory": trajectory,
            "compressed_trajectory": compressed
        }

        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(record_data, f, indent=2, ensure_ascii=False)

        print(f"\n{C.GREEN}[已保存]{C.RESET} {filename}")

        print(f"\n{C.BOLD}{C.CYAN}录制统计:{C.RESET}")
        print(f"  时长: {record_data['duration_sec']} 秒")
        print(f"  原始帧数: {record_data['total_frames']}")
        print(f"  压缩后: {record_data['compressed_frames']} 帧")
        print(f"  压缩率: {record_data['compression_ratio']}")

        if compressed:
            print(f"\n{C.BOLD}{C.CYAN}轨迹概要 (首尾5帧):{C.RESET}")
            print(f"  {'帧号':>5s} | {'时间':>7s} | J1     | J2     | J3     | J4     | J5     | J6")
            print(f"  {'-'*60}")

            # 前5帧
            for frame in compressed[:5]:
                angles = frame['angles']
                line = f"  {frame['frame']:5d} | {frame['time']:6.2f}s | "
                line += " | ".join([f"{a:6.1f}" for a in angles])
                print(line)

            if len(compressed) > 10:
                print(f"  {'...':^60s}")

            # 后5帧
            for frame in compressed[-5:]:
                angles = frame['angles']
                line = f"  {frame['frame']:5d} | {frame['time']:6.2f}s | "
                line += " | ".join([f"{a:6.1f}" for a in angles])
                print(line)

    except Exception as e:
        print(f"\n\n{C.RED}[ERROR]{C.RESET} {e}")
        import traceback
        traceback.print_exc()

        # 确保恢复扭矩
        try:
            for motor_id in MOTOR_IDS:
                packet_handler.write1ByteTxRx(port_handler, motor_id, 40, 1)
            port_handler.closePort()
        except:
            pass

    finally:
        print(f"\n{C.CYAN}[再见]{C.RESET}")


if __name__ == "__main__":
    main()
