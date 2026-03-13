#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
扫描并检测连接到 SO100 的电机
"""

import sys
import os

# 添加 lerobot 源码路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import time

try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
except ImportError as e:
    print(f"无法导入 Feetech 库: {e}")
    exit(1)

def scan_motors_on_port(port, baudrate=1000000):
    """扫描端口上所有可能 ID 的电机"""
    print(f"\n扫描端口 {port}，波特率 {baudrate}...")

    # 先尝试直接连接（不指定电机列表）
    try:
        bus = FeetechMotorsBus(
            port=port,
            motors={},  # 空电机列表
        )

        bus.connect()

        # 尝试扫描 ID 1-253
        found_motors = []

        print("\n扫描电机 ID (1-253)...")
        for motor_id in range(1, 254):
            try:
                # 尝试读取电机型号
                model = bus.read("Model_Number", motor_id)
                if model is not None:
                    print(f"  发现电机 ID {motor_id}: 型号 {model}")
                    found_motors.append(motor_id)
            except Exception:
                pass

        bus.disconnect()

        if found_motors:
            print(f"\n共发现 {len(found_motors)} 个电机:")
            for mid in found_motors:
                print(f"  ID: {mid}")
        else:
            print("\n未发现任何电机。")
            print("\n可能的原因:")
            print("  1. 电机没有供电")
            print("  2. 电机连接线有问题")
            print("  3. 控制器板有问题")
            print("  4. 需要先设置电机 ID（全新电机）")

        return found_motors

    except Exception as e:
        print(f"扫描失败: {e}")
        import traceback
        traceback.print_exc()
        return []

def main():
    print("=" * 60)
    print("SO100 电机扫描工具")
    print("=" * 60)

    port = "COM7"

    # 尝试不同的波特率
    baudrates = [1000000, 57600, 115200, 9600]

    for baudrate in baudrates:
        print(f"\n尝试波特率 {baudrate}...")
        motors = scan_motors_on_port(port, baudrate)
        if motors:
            print(f"\n找到 {len(motors)} 个电机！")
            return

    print("\n" + "=" * 60)
    print("扫描完成")
    print("=" * 60)

if __name__ == "__main__":
    main()
