#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂抓取-放置演示
从右前方抓取物体，放到左前方
使用预设的舵机位置（基于运动学计算 + 手动调整）
"""

import sys
import os
import time

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from SDK import PortHandler, PacketHandler


class PickPlaceController:
    """抓取放置控制器"""

    # 校准数据 (从 so100_calibration.txt)
    ZERO_POSITIONS = {
        1: 2123,  # shoulder_pan
        2: 1966,  # shoulder_lift
        3: 2077,  # elbow_flex
        4: 2082,  # wrist_flex
        5: 2041,  # wrist_roll
        6: 2006,  # gripper
    }

    JOINT_RANGES = {
        1: (741, 3457),
        2: (911, 3304),
        3: (789, 3009),
        4: (874, 3199),
        5: (0, 4095),
        6: (1241, 2799),
    }

    # 舵机寄存器
    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56

    # 预设位置 (基于运动学计算)
    POSES = {
        # 零点位置
        "home": [2123, 1966, 2077, 2082, 2041, 2006],

        # 右前方位置 (y负方向)
        # 位置1: 右前方上方 (接近位置)
        "right_above": [2500, 1800, 1200, 1500, 1900, 2500],
        # 位置2: 右前方抓取位置
        "right_pick": [2500, 1900, 1000, 1200, 1900, 2500],

        # 中间过渡位置
        "center": [2123, 1700, 1400, 1600, 2041, 2500],

        # 左前方位置 (y正方向)
        # 位置3: 左前方上方 (接近位置)
        "left_above": [1700, 1800, 1200, 1500, 2200, 2500],
        # 位置4: 左前方放置位置
        "left_place": [1700, 1900, 1000, 1200, 2200, 2500],
    }

    def __init__(self, port: str = "COM7", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None

    def connect(self):
        """连接机械臂"""
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)

        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"无法设置波特率 {self.baudrate}")

        print(f"[OK] 已连接到 {self.port}")

    def disconnect(self):
        """断开连接"""
        if self.port_handler:
            for m in range(1, 7):
                self.disable_torque(m)
            self.port_handler.closePort()
            print("[OK] 已断开连接")

    def enable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.TORQUE_ENABLE, 0)

    def enable_all(self):
        for m in range(1, 7):
            self.enable_torque(m)

    def set_position(self, motor_id: int, position: int):
        """设置舵机位置（带范围限制）"""
        min_pos, max_pos = self.JOINT_RANGES.get(motor_id, (0, 4095))
        position = max(min_pos, min(max_pos, int(position)))
        self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, self.GOAL_POSITION, position)

    def get_position(self, motor_id: int) -> int:
        data, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, self.PRESENT_POSITION)
        return data if result == 0 else 0

    def get_all_positions(self) -> list:
        return [self.get_position(m) for m in range(1, 7)]

    def move_to_pose(self, pose_name: str, steps: int = 30, dt: float = 0.02):
        """移动到预设位置"""
        if pose_name not in self.POSES:
            print(f"[ERR] 未知位置: {pose_name}")
            return

        target = self.POSES[pose_name]
        start = self.get_all_positions()

        print(f"[MOVE] -> {pose_name}: {target}")

        # S曲线平滑插值
        for step in range(1, steps + 1):
            t = step / steps
            smooth_t = t * t * (3 - 2 * t)  # S曲线

            for m in range(1, 7):
                pos = int(start[m-1] + (target[m-1] - start[m-1]) * smooth_t)
                self.set_position(m, pos)
            time.sleep(dt)

    def move_to_positions(self, positions: list, steps: int = 30, dt: float = 0.02):
        """移动到指定舵机位置"""
        start = self.get_all_positions()

        # S曲线平滑插值
        for step in range(1, steps + 1):
            t = step / steps
            smooth_t = t * t * (3 - 2 * t)

            for m in range(1, 7):
                pos = int(start[m-1] + (positions[m-1] - start[m-1]) * smooth_t)
                self.set_position(m, pos)
            time.sleep(dt)

    def open_gripper(self, width: int = 2500):
        """张开夹爪"""
        self.set_position(6, width)
        time.sleep(0.3)
        print(f"[GRIPPER] 张开 -> {width}")

    def close_gripper(self, width: int = 1600):
        """闭合夹爪"""
        self.set_position(6, width)
        time.sleep(0.3)
        print(f"[GRIPPER] 闭合 -> {width}")

    def pick_and_place(self):
        """执行抓取-放置任务"""

        print("\n" + "=" * 60)
        print("开始抓取-放置任务")
        print("=" * 60)

        # ===== 抓取阶段 =====
        print("\n--- 抓取阶段 (右前方) ---")

        # 步骤1: 回到零点
        print("\n[步骤1] 回到零点")
        self.move_to_pose("home", steps=30)
        time.sleep(0.3)

        # 步骤2: 移动到右前方上方
        print("\n[步骤2] 移动到右前方上方")
        self.move_to_pose("right_above", steps=40)
        time.sleep(0.3)

        # 步骤3: 张开夹爪
        print("\n[步骤3] 张开夹爪")
        self.open_gripper(2500)
        time.sleep(0.2)

        # 步骤4: 下降到抓取位置
        print("\n[步骤4] 下降到抓取位置")
        self.move_to_pose("right_pick", steps=25)
        time.sleep(0.3)

        # 步骤5: 闭合夹爪抓取
        print("\n[步骤5] 闭合夹爪抓取")
        self.close_gripper(1600)
        time.sleep(0.5)

        # 步骤6: 提升
        print("\n[步骤6] 提升")
        self.move_to_pose("right_above", steps=25)
        time.sleep(0.3)

        # ===== 移动阶段 =====
        print("\n--- 移动阶段 ---")

        # 步骤7: 经过中间位置
        print("\n[步骤7] 移动到中间过渡位置")
        self.move_to_pose("center", steps=40)
        time.sleep(0.3)

        # ===== 放置阶段 =====
        print("\n--- 放置阶段 (左前方) ---")

        # 步骤8: 移动到左前方上方
        print("\n[步骤8] 移动到左前方上方")
        self.move_to_pose("left_above", steps=40)
        time.sleep(0.3)

        # 步骤9: 下降到放置位置
        print("\n[步骤9] 下降到放置位置")
        self.move_to_pose("left_place", steps=25)
        time.sleep(0.3)

        # 步骤10: 张开夹爪释放
        print("\n[步骤10] 张开夹爪释放")
        self.open_gripper(2500)
        time.sleep(0.5)

        # 步骤11: 提升
        print("\n[步骤11] 提升")
        self.move_to_pose("left_above", steps=25)
        time.sleep(0.3)

        # 步骤12: 回到零点
        print("\n[步骤12] 回到零点")
        self.move_to_pose("home", steps=40)

        print("\n" + "=" * 60)
        print("抓取-放置任务完成!")
        print("=" * 60)

    def calibrate_positions(self):
        """交互式校准预设位置"""
        print("\n" + "=" * 60)
        print("交互式位置校准")
        print("=" * 60)
        print("使用此功能来手动调整预设位置")
        print("命令: 数字(1-6)+/- 调整, s 保存当前, q 退出")

        # 释放扭矩，允许手动移动
        print("\n释放扭矩，请手动移动机械臂到目标位置...")
        for m in range(1, 7):
            self.disable_torque(m)

        current_pose_name = "custom"
        while True:
            pos = self.get_all_positions()
            print(f"\n当前位置: {pos}")

            cmd = input("输入命令: ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 's':
                name = input("输入位置名称: ").strip()
                self.POSES[name] = pos.copy()
                print(f"[OK] 已保存位置 '{name}': {pos}")
            elif cmd[0].isdigit() and len(cmd) >= 2:
                motor_id = int(cmd[0])
                if 1 <= motor_id <= 6:
                    delta = 20 if '+' in cmd else -20 if '-' in cmd else 0
                    if delta:
                        # 启用扭矩
                        self.enable_all()
                        new_pos = pos[motor_id-1] + delta
                        self.set_position(motor_id, new_pos)
                        time.sleep(0.1)
                        # 再次释放扭矩
                        for m in range(1, 7):
                            self.disable_torque(m)

        # 恢复扭矩
        self.enable_all()
        print("\n校准结束")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='SO100 机械臂抓取-放置演示')
    parser.add_argument('--run', action='store_true', help='直接执行抓取-放置任务')
    parser.add_argument('--home', action='store_true', help='回到零点')
    parser.add_argument('--calib', action='store_true', help='交互式校准位置')
    args = parser.parse_args()

    print("=" * 60)
    print("SO100 机械臂抓取-放置演示")
    print("=" * 60)

    controller = PickPlaceController(port="COM7")

    try:
        controller.connect()
        controller.enable_all()
        print("[OK] 扭矩已启用")

        if args.run:
            # 直接执行抓取-放置
            print("\n直接执行抓取-放置任务...")
            controller.pick_and_place()
        elif args.home:
            # 回到零点
            print("\n回到零点...")
            controller.move_to_pose("home")
        elif args.calib:
            # 交互式校准
            controller.calibrate_positions()
        else:
            # 交互模式
            print("\n可用命令:")
            print("  1 - 执行抓取-放置任务")
            print("  2 - 回到零点")
            print("  3 - 交互式校准位置")
            print("  q - 退出")

            while True:
                cmd = input("\n请选择: ").strip().lower()

                if cmd == '1':
                    input("\n按 Enter 开始执行抓取-放置任务...")
                    controller.pick_and_place()
                elif cmd == '2':
                    print("\n回到零点...")
                    controller.move_to_pose("home")
                elif cmd == '3':
                    controller.calibrate_positions()
                elif cmd == 'q':
                    break
                else:
                    print("无效命令")

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
