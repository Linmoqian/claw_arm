#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TUI 演示程序
展示终端用户界面的各种组件和交互模式
"""

import os
import sys
import time
import threading
from datetime import datetime
from typing import List, Tuple, Dict, Callable

# Windows 终端 ANSI 支持和 UTF-8 编码
if os.name == 'nt':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    # 启用虚拟终端序列
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    # 设置控制台代码页为 UTF-8
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except:
        pass


# ─────────────────────────────────────────────────────────────────
# ANSI 颜色定义 (语义化配色)
# ─────────────────────────────────────────────────────────────────
class Color:
    """ANSI 颜色常量"""
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"

    # 语义化颜色
    SUCCESS = "\033[92m"    # 绿色 - 成功
    WARNING = "\033[93m"    # 黄色 - 警告
    ERROR = "\033[91m"      # 红色 - 错误
    INFO = "\033[96m"       # 青色 - 交互提示
    HIGHLIGHT = "\033[94m"  # 蓝色 - 高亮
    MUTED = "\033[90m"      # 灰色 - 次要信息


def success(msg: str) -> str:
    return f"{Color.SUCCESS}{msg}{Color.RESET}"

def warning(msg: str) -> str:
    return f"{Color.WARNING}{msg}{Color.RESET}"

def error(msg: str) -> str:
    return f"{Color.ERROR}{msg}{Color.RESET}"

def info(msg: str) -> str:
    return f"{Color.INFO}{msg}{Color.RESET}"

def highlight(msg: str) -> str:
    return f"{Color.HIGHLIGHT}{msg}{Color.RESET}"


# ─────────────────────────────────────────────────────────────────
# 终端控制
# ─────────────────────────────────────────────────────────────────
class Terminal:
    """终端控制工具"""

    CURSOR_HOME = "\033[H"
    CLEAR_SCREEN = "\033[2J"
    CLEAR_BELOW = "\033[J"
    CLEAR_LINE = "\033[2K"
    HIDE_CURSOR = "\033[?25l"
    SHOW_CURSOR = "\033[?25h"

    @staticmethod
    def clear():
        """清屏"""
        os.system('cls' if os.name == 'nt' else 'clear')

    @staticmethod
    def move_to(row: int, col: int):
        """移动光标到指定位置"""
        print(f"\033[{row};{col}H", end="")

    @staticmethod
    def refresh(lines: List[str]):
        """无闪烁刷新屏幕"""
        print(Terminal.CURSOR_HOME + "\n".join(lines) + Terminal.CLEAR_BELOW,
              end="", flush=True)

    @staticmethod
    def print_at(row: int, col: int, text: str):
        """在指定位置打印"""
        Terminal.move_to(row, col)
        print(text, end="")


# ─────────────────────────────────────────────────────────────────
# UI 组件
# ─────────────────────────────────────────────────────────────────
class UI:
    """UI 组件库"""

    BOX_WIDTH = 60

    @staticmethod
    def box_title(title: str, width: int = None):
        """绘制标题框"""
        w = width or UI.BOX_WIDTH
        pad = w - 4 - len(title)
        left = pad // 2
        right = pad - left
        print(f"\n{Color.BOLD}{'┌' + '─' * (w - 2) + '┐'}")
        print(f"│{' ' * left} {title} {' ' * right}│")
        print(f"{'└' + '─' * (w - 2) + '┘'}{Color.RESET}\n")

    @staticmethod
    def menu(items: List[Tuple[str, str]], indent: int = 2):
        """打印菜单项"""
        sp = " " * indent
        for key, desc in items:
            print(f"{sp}{info(key)}  {desc}")

    @staticmethod
    def progress_bar(value: float, max_val: float = 100, width: int = 20) -> str:
        """ASCII 进度条"""
        ratio = max(0.0, min(1.0, value / max(max_val, 1)))
        filled = int(ratio * width)
        return f"[{'█' * filled}{'░' * (width - filled)}]"

    @staticmethod
    def spinner(frame: int) -> str:
        """加载动画"""
        frames = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
        return frames[frame % len(frames)]

    @staticmethod
    def table(headers: List[str], rows: List[List[str]], indent: int = 2):
        """绘制表格"""
        sp = " " * indent

        # 计算列宽
        col_widths = [len(h) for h in headers]
        for row in rows:
            for i, cell in enumerate(row):
                col_widths[i] = max(col_widths[i], len(cell))

        # 表头
        header_line = "  ".join(h.ljust(w) for h, w in zip(headers, col_widths))
        separator = "  ".join("─" * w for w in col_widths)

        print(f"{sp}{Color.BOLD}{header_line}{Color.RESET}")
        print(f"{sp}{Color.MUTED}{separator}{Color.RESET}")

        # 数据行
        for row in rows:
            line = "  ".join(cell.ljust(w) for cell, w in zip(row, col_widths))
            print(f"{sp}{line}")

    @staticmethod
    def status_badge(status: str) -> str:
        """状态徽章"""
        badges = {
            "online": success("● 在线"),
            "offline": error("○ 离线"),
            "running": info("▶ 运行中"),
            "stopped": warning("■ 已停止"),
            "success": success("✓ 成功"),
            "failed": error("✗ 失败"),
            "pending": warning("◯ 等待中"),
        }
        return badges.get(status.lower(), f"[{status}]")


# ─────────────────────────────────────────────────────────────────
# 演示模块
# ─────────────────────────────────────────────────────────────────

def demo_colors():
    """演示1: 颜色系统"""
    UI.box_title("颜色系统演示")

    print(f"  语义化配色:")
    print(f"    {success('成功')} - 操作成功、确认信息")
    print(f"    {warning('警告')} - 警告提示、需注意")
    print(f"    {error('错误')} - 错误信息、失败状态")
    print(f"    {info('提示')} - 交互提示、引导信息")
    print(f"    {highlight('高亮')} - 重点内容、链接")

    print(f"\n  文本样式:")
    print(f"    {Color.BOLD}粗体文本{Color.RESET}")
    print(f"    {Color.DIM}暗淡文本{Color.RESET}")
    print(f"    {Color.MUTED}次要信息{Color.RESET}")

    print(f"\n  状态徽章:")
    print(f"    {UI.status_badge('online')}  {UI.status_badge('offline')}")
    print(f"    {UI.status_badge('running')}  {UI.status_badge('stopped')}")
    print(f"    {UI.status_badge('success')}  {UI.status_badge('failed')}")

    input(f"\n  {Color.DIM}按 Enter 继续...{Color.RESET}")


def demo_progress():
    """演示2: 进度条与动画"""
    UI.box_title("进度条演示")

    print(f"  模拟任务进度:\n")

    tasks = [
        ("加载配置", 0.8),
        ("连接设备", 1.0),
        ("初始化系统", 0.6),
        ("同步数据", 0.3),
    ]

    for name, progress in tasks:
        bar = UI.progress_bar(progress * 100, 100, 30)
        pct = f"{progress * 100:5.1f}%"
        print(f"    {name:<12} {bar} {info(pct)}")

    print(f"\n  动态进度演示:")
    print(f"    处理中", end="", flush=True)

    for i in range(101):
        bar = UI.progress_bar(i, 100, 25)
        print(f"\r    处理中 {bar} {info(f'{i:3d}%')}", end="", flush=True)
        time.sleep(0.02)

    print(f"\n    {success('完成')}")
    input(f"\n  {Color.DIM}按 Enter 继续...{Color.RESET}")


def demo_table():
    """演示3: 表格组件"""
    UI.box_title("表格组件演示")

    headers = ["设备ID", "名称", "状态", "位置"]
    rows = [
        ["1", "肩部旋转", "在线", "2047"],
        ["2", "肩部抬升", "在线", "1850"],
        ["3", "肘部弯曲", "在线", "2100"],
        ["4", "腕部弯曲", "离线", "----"],
        ["5", "腕部旋转", "在线", "2047"],
        ["6", "夹爪", "在线", "1800"],
    ]

    UI.table(headers, rows)

    print(f"\n  统计: {success('5/6')} 设备在线")
    input(f"\n  {Color.DIM}按 Enter 继续...{Color.RESET}")


def demo_realtime_monitor():
    """演示4: 实时监控"""
    UI.box_title("实时监控演示")

    print(info("模拟实时数据更新 (按 Ctrl+C 退出)\n"))

    running = True

    def generate_data():
        """生成模拟数据"""
        import random
        return {
            "cpu": random.uniform(20, 80),
            "memory": random.uniform(40, 70),
            "temp": random.uniform(35, 55),
            "speed": random.randint(100, 500),
        }

    print(Terminal.HIDE_CURSOR, end="", flush=True)
    Terminal.clear()

    try:
        frame = 0
        while running:
            data = generate_data()
            now = datetime.now().strftime("%H:%M:%S")

            lines = [
                "",
                f"  {Color.BOLD}◉ 实时监控{Color.RESET}  {Color.DIM}{now}{Color.RESET}",
                "",
                f"  {'指标':<12} {'当前值':>10}  {'状态'}",
                f"  {'─' * 40}",
                f"  {'CPU 使用率':<12} {data['cpu']:>9.1f}%  {UI.progress_bar(data['cpu'], 100, 15)}",
                f"  {'内存使用':<12} {data['memory']:>9.1f}%  {UI.progress_bar(data['memory'], 100, 15)}",
                f"  {'温度':<12} {data['temp']:>9.1f}°C {UI.progress_bar(data['temp'], 60, 15)}",
                f"  {'速度':<12} {data['speed']:>9d} rpm",
                "",
                f"  {Color.DIM}更新频率: 5Hz  |  帧: {frame}{Color.RESET}",
            ]

            Terminal.refresh(lines)
            frame += 1
            time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        print(Terminal.SHOW_CURSOR, end="", flush=True)

    print(f"\n  {success('监控已停止')}")
    input(f"  {Color.DIM}按 Enter 继续...{Color.RESET}")


def demo_menu_system():
    """演示5: 多级菜单系统"""
    UI.box_title("多级菜单演示")

    menus = {
        "main": {
            "title": "主菜单",
            "items": [
                ("1", "系统设置"),
                ("2", "设备管理"),
                ("3", "数据查看"),
                ("q", "返回"),
            ]
        },
        "settings": {
            "title": "系统设置",
            "items": [
                ("1", "网络配置"),
                ("2", "显示设置"),
                ("3", "高级选项"),
                ("b", "返回上级"),
            ]
        },
        "devices": {
            "title": "设备管理",
            "items": [
                ("1", "扫描设备"),
                ("2", "设备列表"),
                ("3", "连接测试"),
                ("b", "返回上级"),
            ]
        },
    }

    current_menu = "main"
    menu_stack = []

    while True:
        menu = menus.get(current_menu, menus["main"])
        UI.box_title(menu["title"])
        UI.menu(menu["items"])
        print()

        choice = input(f"  {info('请选择>')}{Color.RESET} ").strip().lower()

        if choice == "q" and current_menu == "main":
            break
        elif choice == "b" and menu_stack:
            current_menu = menu_stack.pop()
        elif choice == "1" and current_menu == "main":
            menu_stack.append(current_menu)
            current_menu = "settings"
        elif choice == "2" and current_menu == "main":
            menu_stack.append(current_menu)
            current_menu = "devices"
        elif choice.isdigit():
            print(f"\n  {info('执行操作:')} {menu['items'][int(choice)-1][1]}")
            input(f"  {Color.DIM}按 Enter 继续...{Color.RESET}")


def demo_form_input():
    """演示6: 表单输入"""
    UI.box_title("表单输入演示")

    print(f"  {Color.DIM}提示: 按 Tab 切换字段, Enter 确认{Color.RESET}\n")

    fields = [
        ("设备名称", "SO100-Arm"),
        ("端口", "COM7"),
        ("波特率", "1000000"),
        ("超时(ms)", "1000"),
    ]

    values = {}

    for label, default in fields:
        prompt = f"  {info(label)}{Color.DIM}[{default}]{Color.RESET}: "
        value = input(prompt).strip()
        values[label] = value if value else default

    print(f"\n  {success('配置摘要:')}")
    for label, value in values.items():
        print(f"    {label}: {highlight(value)}")

    input(f"\n  {Color.DIM}按 Enter 继续...{Color.RESET}")


def demo_logging():
    """演示7: 日志输出"""
    UI.box_title("日志输出演示")

    logs = [
        ("INFO", "系统启动中..."),
        ("INFO", "加载配置文件: config.yaml"),
        ("SUCCESS", "配置加载完成"),
        ("INFO", "连接设备 COM7..."),
        ("WARNING", "连接超时, 正在重试..."),
        ("SUCCESS", "设备连接成功"),
        ("INFO", "检测电机..."),
        ("SUCCESS", "电机 1-6 全部在线"),
        ("INFO", "启用扭矩..."),
        ("SUCCESS", "系统就绪"),
    ]

    for level, msg in logs:
        timestamp = datetime.now().strftime("%H:%M:%S")

        if level == "INFO":
            prefix = info("[INFO]")
        elif level == "SUCCESS":
            prefix = success("[OK]")
        elif level == "WARNING":
            prefix = warning("[WARN]")
        elif level == "ERROR":
            prefix = error("[ERR]")
        else:
            prefix = f"[{level}]"

        print(f"  {Color.DIM}{timestamp}{Color.RESET} {prefix} {msg}")
        time.sleep(0.15)

    input(f"\n  {Color.DIM}按 Enter 继续...{Color.RESET}")


# ─────────────────────────────────────────────────────────────────
# 主程序
# ─────────────────────────────────────────────────────────────────

def main():
    """主函数"""
    demos = [
        ("1", "颜色系统", demo_colors),
        ("2", "进度条与动画", demo_progress),
        ("3", "表格组件", demo_table),
        ("4", "实时监控", demo_realtime_monitor),
        ("5", "多级菜单系统", demo_menu_system),
        ("6", "表单输入", demo_form_input),
        ("7", "日志输出", demo_logging),
    ]

    while True:
        Terminal.clear()
        UI.box_title("TUI 组件演示")

        print(f"  本程序展示终端用户界面的各种组件和交互模式\n")

        UI.menu([(k, d) for k, d, _ in demos])
        UI.menu([("q", "退出")])

        print()
        choice = input(f"  {info('请选择>')}{Color.RESET} ").strip().lower()

        if choice == "q":
            break

        demo_func = next((f for k, _, f in demos if k == choice), None)
        if demo_func:
            Terminal.clear()
            try:
                demo_func()
            except KeyboardInterrupt:
                print(f"\n  {warning('演示已中断')}")
                input(f"  {Color.DIM}按 Enter 继续...{Color.RESET}")

    Terminal.clear()
    print(f"\n  {success('再见!')}\n")


if __name__ == "__main__":
    main()
