"""
hello_arm.py - 最简机械臂控制示例

功能：让机械臂执行一个简单的挥手动作

学习目标：
  - 理解 SDK 初始化流程
  - 掌握关节控制基础
  - 了解安全操作规范

硬件要求：SO-100 机械臂已连接

使用方法：
  python examples/basic/hello_arm.py
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
MOTOR_ID = 2             # 要控制的电机 ID (2 = 肩关节)

# ============ 主程序 ============
def main():
    print("=" * 50)
    print("  Hello Arm - 机械臂入门示例")
    print("=" * 50)

    # 1. 连接串口
    print(f"\n[1] 连接串口: {SERIAL_PORT} @ {BAUDRATE}")
    port = PortHandler(SERIAL_PORT)
    port.setBaudRate(BAUDRATE)

    if not port.openPort():
        print(f"❌ 无法打开串口 {SERIAL_PORT}")
        print("   请检查：")
        print("   - 串口号是否正确")
        print("   - 驱动是否安装")
        print("   - 是否被其他程序占用")
        return

    print("✅ 串口连接成功")

    # 2. 创建通信对象
    pkt = PacketHandler(0.0)

    try:
        # 3. 启用电机扭矩
        print(f"\n[2] 启用电机 {MOTOR_ID}")
        pkt.write1ByteTxRx(port, MOTOR_ID, 40, 1)  # 40 = TORQUE_ENABLE
        print("✅ 扭矩已启用")

        # 4. 读取当前位置
        print(f"\n[3] 读取当前位置")
        position, result, _ = pkt.read2ByteTxRx(port, MOTOR_ID, 56)  # 56 = PRESENT_POSITION
        if result == 0:
            print(f"✅ 当前位置: {position}")
        else:
            print(f"⚠️ 无法读取位置，使用默认值")
            position = 2048

        # 5. 执行挥手动作
        print(f"\n[4] 执行挥手动作")
        target1 = min(4095, position + 500)
        target2 = max(0, position - 500)

        print(f"   移动到 {target1}")
        pkt.write2ByteTxRx(port, MOTOR_ID, 42, target1)  # 42 = GOAL_POSITION
        time.sleep(1)

        print(f"   移动到 {target2}")
        pkt.write2ByteTxRx(port, MOTOR_ID, 42, target2)
        time.sleep(1)

        print(f"   返回原位 {position}")
        pkt.write2ByteTxRx(port, MOTOR_ID, 42, position)
        time.sleep(1)

        print("✅ 动作完成")

    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")

    finally:
        # 6. 安全关闭
        print(f"\n[5] 安全关闭")
        pkt.write1ByteTxRx(port, MOTOR_ID, 40, 0)  # 禁用扭矩
        port.closePort()
        print("✅ 已断开连接")
        print("\n🎉 Hello Arm 完成！")


if __name__ == "__main__":
    main()
