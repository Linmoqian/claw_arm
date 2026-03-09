#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CV 公共工具模块
提供终端彩色输出功能
"""


class Colors:
    """终端彩色输出常量"""
    GREEN = '\033[92m'      # 成功
    YELLOW = '\033[93m'     # 警告
    RED = '\033[91m'        # 错误
    CYAN = '\033[96m'       # 交互提示
    BLUE = '\033[94m'       # 高亮
    GRAY = '\033[90m'       # 次要信息
    RESET = '\033[0m'


def print_colored(message: str, color: str = Colors.RESET) -> None:
    """打印彩色文本"""
    print(f"{color}{message}{Colors.RESET}")


class ColorOutput:
    """封装彩色打印方法"""

    @staticmethod
    def success(message: str) -> None:
        """成功信息 (绿色)"""
        print_colored(message, Colors.GREEN)

    @staticmethod
    def warning(message: str) -> None:
        """警告信息 (黄色)"""
        print_colored(message, Colors.YELLOW)

    @staticmethod
    def error(message: str) -> None:
        """错误信息 (红色)"""
        print_colored(message, Colors.RED)

    @staticmethod
    def info(message: str) -> None:
        """交互提示 (青色)"""
        print_colored(message, Colors.CYAN)

    @staticmethod
    def highlight(message: str) -> None:
        """高亮信息 (蓝色)"""
        print_colored(message, Colors.BLUE)

    @staticmethod
    def secondary(message: str) -> None:
        """次要信息 (灰色)"""
        print_colored(message, Colors.GRAY)
