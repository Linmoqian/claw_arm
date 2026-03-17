"""
read_joint.py - 读取关节角度示例

功能：实时读取并显示机械臂各关节的当前位置

学习目标：
  - 理解舵机通信协议
  - 掌握多电机读取方法
  - 了解关节角度范围

硬件要求：SO-100 机械臂已连接

使用方法：
  python examples/basic/read_joint.py
"""
import sys
import os
import time

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from SDK import PortHandler, PacketHandler

# ============ 配置区 ============
SERIAL_PORT = "COM12"    # 修改为你的串口
BAUDRATE = 1000000       # 波特率

# 关节配置
JOINTS = [
    {"id": 1, "name": "base", "min": 0, "max": 4095},
    {"id": 2, "name": "shoulder", "min": 0, "max": 4095},
    {"id": 3, "name": "elbow", "min": 0, "max": 4095},
    {"id": 4, "name": "wrist_pitch", "min": 0, "max": 4095},
    {"id": 5, "name": "wrist_roll", "min": 0, "max": 4095},
    {"id": 6, "name": "gripper", "min": 0, "max": 4095},
]

# ============ 主程序 ============
def main():
    print("=" * 60)
    print("  关节角度读取 - 实时监控")
    print("=" * 60)

    # 连接串口
    print(f"\n连接串口: {SERIAL_PORT} @ {BAUDRATE}")
    port = PortHandler(SERIAL_PORT)
    port.setBaudRate(BAUDRATE)

    if not port.openPort():
        print(f"❌ 无法打开串口 {SERIAL_PORT}")
        return

    pkt = PacketHandler(0.0)
    print("✅ 连接成功\n")

    try:
        print("按 Ctrl+C 退出\n")
        print("-" * 60)
        print(f"{'关节':<15} {'ID':<5} {'位置':<10} {'角度°':<10}")
        print("-" * 60)

        while True:
            # 读取所有关节
            readings = []
            for joint in JOINTS:
                position, result, _ = pkt.read2ByteTxRx(
                    port, joint["id"], 56  # 56 = PRESENT_POSITION
                )

                if result == 0:
                    # 转换为角度 (0-4095 -> 0-360°)
                    angle = (position / 4095.0) * 360
                    readings.append({
                        "name": joint["name"],
                        "id": joint["id"],
                        "position": position,
                        "angle": angle,
                        "status": "OK"
                    })
                else:
                    readings.append({
                        "name": joint["name"],
                        "id": joint["id"],
                        "position": "---",
                        "angle": "---",
                        "status": "ERR"
                    })

            # 清屏并显示
            print("\033[H\033[J", end="")  # ANSI 清屏
            print("=" * 60)
            print("  关节角度读取 - 实时监控")
            print("=" * 60)
            print(f"{'关节':<15} {'ID':<5} {'位置':<10} {'角度°':<10}")
            print("-" * 60)

            for r in readings:
                if r["status"] == "OK":
                    print(f"{r['name']:<15} {r['id']:<5} {r['position']:<10} {r['angle']:<10.1f}")
                else:
                    print(f"{r['name']:<15} {r['id']:<5} {'---':<10} {'---':<10}")

            print("-" * 60)
            print("按 Ctrl+C 退出")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n⚠️ 用户中断")

    finally:
        port.closePort()
        print("✅ 已断开连接")


if __name__ == "__main__":
    main()
