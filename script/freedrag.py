#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
释放机械臂扭矩 - 允许手动拖动
"""

import sys
import os
import time

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
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RESET = '\033[0m'

PORT = "COM7"
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

def main():
    print(f"\n{C.BOLD}{C.CYAN}{'=' * 50}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  释放机械臂扭矩 - 可手动拖动{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'=' * 50}{C.RESET}\n")

    port_handler = PortHandler(PORT)
    packet_handler = PacketHandler(0.0)

    try:
        if not port_handler.openPort():
            print(f"{C.RED}[ERROR]{C.RESET} 无法打开端口 {PORT}")
            print(f"{C.YELLOW}提示:{C.RESET} 请关闭其他程序后重试")
            input("按 Enter 退出...")
            return

        if not port_handler.setBaudRate(1000000):
            print(f"{C.RED}[ERROR]{C.RESET} 无法设置波特率")
            input("按 Enter 退出...")
            return

        print(f"{C.GREEN}[OK]{C.RESET} 已连接到 {PORT}")

        print(f"\n{C.CYAN}[检测电机]{C.RESET}")
        for motor_id in MOTOR_IDS:
            _, result, _ = packet_handler.ping(port_handler, motor_id)
            status = f"{C.GREEN}在线{C.RESET}" if result == 0 else f"{C.RED}离线{C.RESET}"
            print(f"  Motor {motor_id}: {status}")

        print(f"\n{C.YELLOW}[INFO]{C.RESET} 释放扭矩中...")

        for motor_id in MOTOR_IDS:
            packet_handler.write1ByteTxRx(port_handler, motor_id, 40, 0)

        print(f"{C.GREEN}[OK]{C.RESET} 所有电机已释放")
        print(f"\n{C.BOLD}{C.YELLOW}现在可以手动拖动机械臂！{C.RESET}")
        print(f"{C.DIM}调整到合适位置后，告诉我当前各关节的大致角度{C.RESET}")
        print(f"{C.DIM}格式：J1 J2 J3 J4 J5（度）{C.RESET}")

        print(f"\n{C.CYAN}扭矩状态检查:{C.RESET}")
        for motor_id in MOTOR_IDS[:5]:
            data, result, _ = packet_handler.read1ByteTxRx(port_handler, motor_id, 40)
            state = "释放" if data == 0 else "锁定"
            color = f"{C.GREEN}" if data == 0 else f"{C.RED}"
            print(f"  Motor {motor_id}: {color}{state}{C.RESET}")

        print(f"\n{C.DIM}提示: 按 Ctrl+C 退出，退出时会自动恢复扭矩{C.RESET}")

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print(f"\n\n{C.YELLOW}[INFO]{C.RESET} 正在恢复扭矩...")

        # 恢复扭矩
        for motor_id in MOTOR_IDS:
            packet_handler.write1ByteTxRx(port_handler, motor_id, 40, 1)

        print(f"{C.GREEN}[OK]{C.RESET} 扭矩已恢复")

        port_handler.closePort()
        print(f"{C.GREEN}[OK]{C.RESET} 已断开连接")

    except Exception as e:
        print(f"\n{C.RED}[ERROR]{C.RESET} {e}")

    finally:
        print(f"\n{C.CYAN}[再见]{C.RESET}")


if __name__ == "__main__":
    main()
