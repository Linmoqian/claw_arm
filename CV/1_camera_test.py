#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
摄像头测试脚本
实时显示摄像头画面，并在画面上叠加状态信息
"""

import cv2
import time

# ───────── ANSI 颜色 ─────────
CLR = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
CYAN = "\033[96m"


def ok(msg: str) -> str:
    return f"{GREEN}[OK]{CLR} {msg}"


def warn(msg: str) -> str:
    return f"{YELLOW}[!!]{CLR} {msg}"


def err(msg: str) -> str:
    return f"{RED}[ERR]{CLR} {msg}"


def info(msg: str) -> str:
    return f"{CYAN}[>>]{CLR} {msg}"


def main():
    camera_index = 0

    print(info(f"正在打开摄像头 (索引 {camera_index})..."))
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(err("无法打开摄像头"))
        return

    print(ok("摄像头已连接"))

    try:
        # 获取摄像头参数
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps_target = int(cap.get(cv2.CAP_PROP_FPS))

        print(ok(f"分辨率: {width}x{height}"))
        print(ok(f"目标帧率: {fps_target} FPS"))
        print(info("按 q 键退出"))

        # FPS 计算
        prev_time = time.time()
        fps = 0.0

        while True:
            ret, frame = cap.read()
            if not ret:
                print(err("无法读取画面"))
                break

            # 计算 FPS
            curr_time = time.time()
            fps = 1.0 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0.0
            prev_time = curr_time

            # 在画面上叠加状态信息
            status_text = f"Camera: {camera_index} | {width}x{height} | FPS: {fps:.1f}"
            cv2.putText(
                frame,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                frame,
                "Press 'q' to quit",
                (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow("Camera Test", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print(info("用户退出"))
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(ok("摄像头已释放"))


if __name__ == "__main__":
    main()
