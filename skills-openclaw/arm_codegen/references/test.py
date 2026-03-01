#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂底层控制（综合版）
直接使用 scservo_sdk 控制，包含电机检测、控制测试、校准三种模式
"""

from scservo_sdk import PortHandler, PacketHandler
import time
import threading


class SO100Controller:
    """SO100 机械臂控制器"""

    # STS3215 寄存器地址
    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56
    MAX_POSITION = 4095

    # 电机名称
    MOTOR_NAMES = {
        1: "shoulder_pan",
        2: "shoulder_lift",
        3: "elbow_flex",
        4: "wrist_flex",
        5: "wrist_roll",
        6: "gripper"
    }

    def __init__(self, port: str, baudrate: int = 1000000):
        """
        初始化控制器

        Args:
            port: 串口名称（如 'COM7'）
            baudrate: 波特率（默认 1000000）
        """
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None

    def connect(self):
        """连接到机械臂"""
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
            for motor_id in range(1, 7):
                self.disable_torque(motor_id)
            self.port_handler.closePort()
            print("[OK] 已断开连接")

    def enable_torque(self, motor_id: int):
        """启用电机扭矩"""
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id,
            self.TORQUE_ENABLE, 1
        )

    def disable_torque(self, motor_id: int):
        """禁用电机扭矩"""
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id,
            self.TORQUE_ENABLE, 0
        )

    def set_position(self, motor_id: int, position: int):
        """设置目标位置"""
        position = max(0, min(self.MAX_POSITION, position))
        self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id,
            self.GOAL_POSITION, position
        )

    def get_position(self, motor_id: int) -> int:
        """读取当前位置"""
        data, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id,
            self.PRESENT_POSITION
        )
        return data if result == 0 else 0

    def ping(self, motor_id: int) -> bool:
        """检测电机是否存在"""
        _, result, _ = self.packet_handler.ping(self.port_handler, motor_id)
        return result == 0


def mode_detection(controller: SO100Controller):
    """模式1：电机检测"""
    print("\n" + "=" * 70)
    print("模式 1：电机检测")
    print("=" * 70)

    print("\n扫描电机...")
    found_motors = []

    for motor_id in range(1, 7):
        if controller.ping(motor_id):
            found_motors.append(motor_id)
            print(f"  [OK] 电机 {motor_id} ({controller.MOTOR_NAMES[motor_id]})")
        else:
            print(f"  [--] 电机 {motor_id} ({controller.MOTOR_NAMES[motor_id]})")

    print(f"\n检测到 {len(found_motors)}/6 个电机")

    if found_motors:
        print("\n读取当前位置:")
        for motor_id in found_motors:
            pos = controller.get_position(motor_id)
            print(f"  电机 {motor_id} ({controller.MOTOR_NAMES[motor_id]}): {pos}")

    if len(found_motors) == 6:
        print("\n[OK] 所有电机正常！")
    elif len(found_motors) > 0:
        print(f"\n! 警告：只检测到 {len(found_motors)}/6 个电机")
    else:
        print("\n! 错误：未检测到任何电机")


def mode_control_test(controller: SO100Controller):
    """模式2：控制测试"""
    print("\n" + "=" * 70)
    print("模式 2：控制测试")
    print("=" * 70)

    # 检测电机
    print("\n检测电机...")
    found_motors = []
    for motor_id in range(1, 7):
        if controller.ping(motor_id):
            found_motors.append(motor_id)

    if len(found_motors) < 6:
        print(f"错误：只检测到 {len(found_motors)}/6 个电机")
        return

    print("[OK] 所有 6 个电机检测成功")

    # 启用扭矩
    print("\n启用扭矩...")
    for motor_id in range(1, 7):
        controller.enable_torque(motor_id)
    print("  所有电机扭矩已启用")

    # 测试运动序列
    test_sequences = [
        {
            "name": "零点位置",
            "positions": {1: 2047, 2: 2047, 3: 2047, 4: 2047, 5: 2047, 6: 2000}
        },
        {
            "name": "伸展位置",
            "positions": {1: 2500, 2: 2500, 3: 1500, 4: 1500, 5: 2047, 6: 3000}
        },
        {
            "name": "收缩位置",
            "positions": {1: 1500, 2: 1500, 3: 2500, 4: 2500, 5: 2500, 6: 1000}
        },
        {
            "name": "回到零点",
            "positions": {1: 2047, 2: 2047, 3: 2047, 4: 2047, 5: 2047, 6: 2047}
        }
    ]

    for i, seq in enumerate(test_sequences, 1):
        print(f"\n[{i}/{len(test_sequences)}] 移动到 {seq['name']}...")
        for motor_id, pos in seq['positions'].items():
            controller.set_position(motor_id, pos)

        time.sleep(2)

        # 显示当前位置
        print("  当前位置:")
        for motor_id in range(1, 7):
            pos = controller.get_position(motor_id)
            print(f"    {controller.MOTOR_NAMES[motor_id]}: {pos}")

    print("\n" + "=" * 70)
    print("[OK] 控制测试完成！")
    print("=" * 70)


def mode_calibration(controller: SO100Controller):
    """模式3：校准"""
    print("\n" + "=" * 70)
    print("模式 3：校准")
    print("=" * 70)

    # 检测电机
    print("\n检测电机...")
    found_motors = []
    for motor_id in range(1, 7):
        if controller.ping(motor_id):
            found_motors.append(motor_id)

    if len(found_motors) < 6:
        print(f"错误：只检测到 {len(found_motors)}/6 个电机")
        return

    print("[OK] 所有 6 个电机检测成功")

    # 步骤 1：禁用扭矩
    print("\n[步骤 1] 禁用扭矩")
    for motor_id in range(1, 7):
        controller.disable_torque(motor_id)
    print("  所有电机扭矩已禁用，现在可以手动移动机械臂")

    # 步骤 2：移动到中间位置
    print("\n[步骤 2] 将机械臂移动到中间位置")
    print("  提示：大臂约45度，小臂约90度，手腕水平")
    input("  准备好后按 Enter...")

    # 记录零点
    print("\n[步骤 3] 记录零点位置")
    zero_positions = {}
    for motor_id in range(1, 7):
        pos = controller.get_position(motor_id)
        zero_positions[motor_id] = pos
        print(f"  {controller.MOTOR_NAMES[motor_id]}: {pos}")

    # 步骤 3：连续记录运动范围
    print("\n[步骤 4] 记录运动范围")
    print("  请逐个移动所有关节（除了 wrist_roll）通过它们的完整运动范围")
    print("  系统将持续记录并实时显示最小值、当前值、最大值")
    print("\n准备好后按 Enter 开始记录...")
    input()

    # 初始化范围
    ranges = {motor_id: [4095, 0] for motor_id in range(1, 7)}
    ranges[5] = [0, 4095]  # wrist_roll 全范围

    # 记录状态
    recording = True
    stop_event = threading.Event()

    def record_loop():
        """记录循环"""
        nonlocal recording
        while recording:
            for motor_id in range(1, 7):
                pos = controller.get_position(motor_id)
                if pos > 0:
                    # 更新最小/最大值
                    if pos < ranges[motor_id][0]:
                        ranges[motor_id][0] = pos
                    if pos > ranges[motor_id][1]:
                        ranges[motor_id][1] = pos
            time.sleep(0.1)  # 10 Hz 采样

    def display_loop():
        """显示循环"""
        nonlocal recording
        while recording:
            # 清屏（Windows）
            import os
            os.system('cls' if os.name == 'nt' else 'clear')

            print("\n" + "=" * 70)
            print("实时记录中... 按 Enter 停止")
            print("=" * 70)
            print("\n{:<15} {:>8} {:>8} {:>8} {:>8}".format(
                "电机", "最小值", "当前值", "最大值", "范围"
            ))
            print("-" * 70)

            for motor_id in range(1, 7):
                pos = controller.get_position(motor_id)
                min_pos = ranges[motor_id][0]
                max_pos = ranges[motor_id][1]
                range_size = max_pos - min_pos

                print("{:<15} {:>8} {:>8} {:>8} {:>8}".format(
                    controller.MOTOR_NAMES[motor_id],
                    min_pos,
                    pos,
                    max_pos,
                    range_size
                ))

            print("\n提示：逐个移动每个关节（除 wrist_roll）通过完整运动范围")
            time.sleep(0.2)  # 5 Hz 刷新

    # 启动记录和显示线程
    record_thread = threading.Thread(target=record_loop)
    display_thread = threading.Thread(target=display_loop)

    record_thread.start()
    display_thread.start()

    # 等待用户停止
    input()

    # 停止记录
    recording = False
    record_thread.join()
    display_thread.join()

    # 显示结果
    print("\n" + "=" * 70)
    print("校准完成！")
    print("=" * 70)

    print("\n零点位置:")
    for motor_id, pos in zero_positions.items():
        print(f"  {controller.MOTOR_NAMES[motor_id]}: {pos}")

    print("\n运动范围:")
    for motor_id in range(1, 7):
        min_pos, max_pos = ranges[motor_id]
        range_size = max_pos - min_pos
        print(f"  {controller.MOTOR_NAMES[motor_id]}: {min_pos} - {max_pos} (范围: {range_size})")

    # 保存校准数据
    filename = "so100_calibration.txt"
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("# SO100 机械臂校准数据\n")
        f.write("# 生成时间: " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n\n")
        f.write("# 零点位置\n")
        for motor_id, pos in zero_positions.items():
            f.write(f"zero_{controller.MOTOR_NAMES[motor_id]} = {pos}\n")

        f.write("\n# 运动范围\n")
        for motor_id, (min_p, max_p) in ranges.items():
            f.write(f"range_{controller.MOTOR_NAMES[motor_id]} = {min_p}, {max_p}\n")

    print(f"\n[OK] 校准数据已保存到: {filename}")

    # 启用扭矩
    print("\n启用扭矩...")
    for motor_id in range(1, 7):
        controller.enable_torque(motor_id)
    print("  所有电机扭矩已启用")


def main():
    """主函数"""
    PORT = "COM7"  # 修改为您的串口

    print("\n" + "=" * 70)
    print("SO100 机械臂底层控制")
    print("=" * 70)
    print("1. 电机检测")
    print("2. 控制测试")
    print("3. 校准")
    print("=" * 70)

    choice = input("\n请选择模式 (1/2/3): ").strip()

    if choice not in ["1", "2", "3"]:
        print("无效选项")
        return

    # 创建控制器
    controller = SO100Controller(PORT)

    try:
        # 连接
        controller.connect()

        # 执行对应模式
        if choice == "1":
            mode_detection(controller)
        elif choice == "2":
            mode_control_test(controller)
        elif choice == "3":
            mode_calibration(controller)

    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 断开连接
        controller.disconnect()


if __name__ == "__main__":
    main()
