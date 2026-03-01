#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂测试脚本
连接到 COM7 端口并让机械臂执行简单的动作
"""

import sys
import os

# 添加 lerobot 源码路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig
import time

def main():
    print("=" * 60)
    print("SO100 机械臂测试程序")
    print("=" * 60)

    # 配置机器人
    config = SO100FollowerConfig(
        port="COM7",  # 使用 COM7 端口
        id="my_so100_arm",
        disable_torque_on_disconnect=True,
    )

    # 创建机器人实例
    robot = SO100Follower(config)

    try:
        # 连接机器人（不重新校准）
        print("\n正在连接到机械臂...")
        robot.connect(calibrate=False)
        print("[OK] 机械臂连接成功！")

        # 读取当前位置
        print("\n读取当前位置...")
        observation = robot.get_observation()
        print("当前关节位置:")
        for key, value in observation.items():
            if key.endswith('.pos'):
                print(f"  {key}: {value:.2f}")

        # 测试动作：让机械臂做一个简单的动作
        print("\n执行测试动作...")
        print("注意：机械臂即将开始移动，请确保周围没有障碍物！")
        time.sleep(2)

        # 动作1：稍微移动每个关节
        test_actions = [
            {
                "shoulder_pan.pos": 10.0,
                "shoulder_lift.pos": 10.0,
                "elbow_flex.pos": 10.0,
                "wrist_flex.pos": 10.0,
                "wrist_roll.pos": 10.0,
                "gripper.pos": 50.0,
            },
            {
                "shoulder_pan.pos": -10.0,
                "shoulder_lift.pos": -10.0,
                "elbow_flex.pos": -10.0,
                "wrist_flex.pos": -10.0,
                "wrist_roll.pos": -10.0,
                "gripper.pos": 0.0,
            },
            {
                "shoulder_pan.pos": 0.0,
                "shoulder_lift.pos": 0.0,
                "elbow_flex.pos": 0.0,
                "wrist_flex.pos": 0.0,
                "wrist_roll.pos": 0.0,
                "gripper.pos": 0.0,
            }
        ]

        for i, action in enumerate(test_actions, 1):
            print(f"\n动作 {i}/3:")
            print(f"  目标位置: {action}")
            robot.send_action(action)
            time.sleep(1.5)  # 等待动作完成

        # 读取最终位置
        print("\n读取最终位置...")
        observation = robot.get_observation()
        print("最终关节位置:")
        for key, value in observation.items():
            if key.endswith('.pos'):
                print(f"  {key}: {value:.2f}")

        print("\n" + "=" * 60)
        print("[OK] 测试完成！机械臂运行正常。")
        print("=" * 60)

    except Exception as e:
        print(f"\n[ERROR] 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 断开连接
        print("\n正在断开连接...")
        try:
            robot.disconnect()
            print("[OK] 已安全断开连接。")
        except:
            pass

if __name__ == "__main__":
    main()
