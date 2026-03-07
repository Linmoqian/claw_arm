#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂从左抓取到右放置脚本
动作序列：回零 -> 张开夹爪 -> 左侧预抓 -> 抓取 -> 抬起 -> 右侧放置 -> 释放 -> 回零
"""

from scservo_sdk import PortHandler, PacketHandler
import time


class GrabController:
    """抓取控制器"""

    # STS3215 寄存器地址
    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56
    MAX_POSITION = 4095

    # 电机 ID 映射
    MOTOR_IDS = {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }

    def __init__(self, port: str = "COM7", baudrate: int = 1000000):
        """
        初始化控制器

        Args:
            port: 串口名称
            baudrate: 波特率
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
            for motor_id in self.MOTOR_IDS.values():
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

    def enable_all_torque(self):
        """启用所有电机扭矩"""
        print("[启用扭矩] 所有电机...")
        for motor_id in self.MOTOR_IDS.values():
            self.enable_torque(motor_id)
        time.sleep(0.1)

    def move_to_pose(self, pose: dict, name: str = "", delay: float = 1.5):
        """
        移动到指定姿态

        Args:
            pose: 关节位置字典 {"joint_name": position, ...}
            name: 动作名称（用于打印）
            delay: 等待时间（秒）
        """
        if name:
            print(f"[{name}]")

        # 发送所有关节目标位置
        for joint_name, position in pose.items():
            motor_id = self.MOTOR_IDS[joint_name]
            self.set_position(motor_id, position)

        # 等待动作完成
        time.sleep(delay)


def main():
    """主函数：执行左抓右放动作序列"""

    # 动作序列定义
    poses = {
        "零位": {
            "shoulder_pan": 2047,
            "shoulder_lift": 2047,
            "elbow_flex": 2047,
            "wrist_flex": 2047,
            "wrist_roll": 2047,
            "gripper": 3000,  # 张开
        },
        "左侧预抓": {
            "shoulder_pan": 1500,
            "shoulder_lift": 2200,
            "elbow_flex": 1800,
            "wrist_flex": 2000,
            "wrist_roll": 2047,
            "gripper": 3000,  # 张开
        },
        "左侧抓取": {
            "shoulder_pan": 1500,
            "shoulder_lift": 2200,
            "elbow_flex": 1800,
            "wrist_flex": 2000,
            "wrist_roll": 2047,
            "gripper": 1000,  # 闭合
        },
        "抬起": {
            "shoulder_pan": 1500,
            "shoulder_lift": 1800,
            "elbow_flex": 1800,
            "wrist_flex": 2000,
            "wrist_roll": 2047,
            "gripper": 1000,  # 保持闭合
        },
        "右侧放置": {
            "shoulder_pan": 2600,
            "shoulder_lift": 2200,
            "elbow_flex": 1800,
            "wrist_flex": 2000,
            "wrist_roll": 2047,
            "gripper": 1000,  # 保持闭合
        },
        "释放": {
            "shoulder_pan": 2600,
            "shoulder_lift": 2200,
            "elbow_flex": 1800,
            "wrist_flex": 2000,
            "wrist_roll": 2047,
            "gripper": 3000,  # 张开
        },
    }

    # 创建控制器
    controller = GrabController(port="COM7", baudrate=1000000)

    try:
        # 连接
        print("=" * 60)
        print("SO100 机械臂 - 左抓右放")
        print("=" * 60)
        controller.connect()

        # 启用所有电机扭矩
        controller.enable_all_torque()

        # 执行动作序列
        sequence = [
            ("零位", poses["零位"], 2.0),
            ("左侧预抓", poses["左侧预抓"], 1.5),
            ("左侧抓取", poses["左侧抓取"], 1.0),
            ("抬起", poses["抬起"], 1.5),
            ("右侧放置", poses["右侧放置"], 1.5),
            ("释放", poses["释放"], 1.0),
            ("回零", poses["零位"], 2.0),
        ]

        for step, (name, pose, delay) in enumerate(sequence, 1):
            print(f"\n[步骤 {step}/{len(sequence)}] {name}")
            controller.move_to_pose(pose, name, delay)

            # 显示当前位置
            print("  当前位置:")
            for joint_name, motor_id in controller.MOTOR_IDS.items():
                pos = controller.get_position(motor_id)
                print(f"    {joint_name}: {pos}")

        print("\n" + "=" * 60)
        print("[完成] 抓取-放置动作序列执行完毕！")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n[中断] 用户中断执行")

    except Exception as e:
        print(f"\n[错误] {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 断开连接
        controller.disconnect()


if __name__ == "__main__":
    main()
