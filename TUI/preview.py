#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TUI 组件预览 (非交互式)
快速预览所有TUI组件效果
"""

import os
import sys
import time
from datetime import datetime

# Windows 终端配置
if os.name == 'nt':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except:
        pass


# ─────────────────────────────────────────────────────────────────
# 颜色定义
# ─────────────────────────────────────────────────────────────────
class C:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    SUCCESS = "\033[92m"
    WARNING = "\033[93m"
    ERROR = "\033[91m"
    INFO = "\033[96m"
    HIGHLIGHT = "\033[94m"
    MUTED = "\033[90m"


def main():
    print(f"\n{C.BOLD}{C.INFO}{'┌' + '─'*56 + '┐'}{C.RESET}")
    print(f"{C.BOLD}{C.INFO}│{'TUI 组件预览':^56}│{C.RESET}")
    print(f"{C.BOLD}{C.INFO}{'└' + '─'*56 + '┘'}{C.RESET}\n")

    # 1. 颜色系统
    print(f"{C.BOLD}  颜色系统{C.RESET}")
    print(f"    {C.SUCCESS}● 成功{C.RESET}  {C.WARNING}● 警告{C.RESET}  {C.ERROR}● 错误{C.RESET}  {C.INFO}● 提示{C.RESET}")
    print(f"    {C.HIGHLIGHT}● 高亮{C.RESET}  {C.MUTED}● 次要{C.RESET}  {C.BOLD}● 粗体{C.RESET}  {C.DIM}● 暗淡{C.RESET}")

    # 2. 状态徽章
    print(f"\n{C.BOLD}  状态徽章{C.RESET}")
    print(f"    {C.SUCCESS}✓ 在线{C.RESET}  {C.ERROR}✗ 离线{C.RESET}  {C.INFO}▶ 运行中{C.RESET}  {C.WARNING}◯ 等待{C.RESET}")

    # 3. 进度条
    print(f"\n{C.BOLD}  进度条{C.RESET}")
    print(f"    加载中   {C.INFO}[████████████████░░░░░░░░]{C.RESET} {C.INFO}66.7%{C.RESET}")
    print(f"    下载中   {C.SUCCESS}[████████████████████████]{C.RESET} {C.SUCCESS}100%{C.RESET}")
    print(f"    处理中   {C.WARNING}[████░░░░░░░░░░░░░░░░░░]{C.RESET} {C.WARNING}16.7%{C.RESET}")

    # 4. 表格
    print(f"\n{C.BOLD}  表格组件{C.RESET}")
    print(f"    {'ID':<6} {'名称':<12} {'状态':<10} {'值':>8}")
    print(f"    {C.MUTED}{'─'*42}{C.RESET}")
    print(f"    {'1':<6} {'肩部旋转':<12} {C.SUCCESS}在线{C.RESET}     {'2047':>8}")
    print(f"    {'2':<6} {'肩部抬升':<12} {C.SUCCESS}在线{C.RESET}     {'1850':>8}")
    print(f"    {'3':<6} {'肘部弯曲':<12} {C.ERROR}离线{C.RESET}     {'----':>8}")
    print(f"    {'4':<6} {'夹爪':<12} {C.SUCCESS}在线{C.RESET}     {'1800':>8}")

    # 5. 日志输出
    print(f"\n{C.BOLD}  日志格式{C.RESET}")
    now = datetime.now().strftime("%H:%M:%S")
    print(f"    {C.MUTED}{now}{C.RESET} {C.INFO}[INFO]{C.RESET} 系统启动中...")
    print(f"    {C.MUTED}{now}{C.RESET} {C.SUCCESS}[OK]{C.RESET}   配置加载完成")
    print(f"    {C.MUTED}{now}{C.RESET} {C.WARNING}[WARN]{C.RESET} 连接超时, 重试中...")
    print(f"    {C.MUTED}{now}{C.RESET} {C.ERROR}[ERR]{C.RESET}  设备无响应")

    # 6. 菜单样式
    print(f"\n{C.BOLD}  菜单样式{C.RESET}")
    print(f"    {C.INFO}1{C.RESET}  电机检测")
    print(f"    {C.INFO}2{C.RESET}  位置控制")
    print(f"    {C.INFO}3{C.RESET}  轨迹回放")
    print(f"    {C.INFO}q{C.RESET}  退出")

    # 7. 交互提示
    print(f"\n{C.BOLD}  交互提示{C.RESET}")
    print(f"    {C.INFO}请选择>{C.RESET} _")

    print(f"\n{C.MUTED}  运行 tui_demo.py 可体验完整交互功能{C.RESET}\n")


if __name__ == "__main__":
    main()
