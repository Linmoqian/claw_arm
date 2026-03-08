#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂视觉引导抓取
使用相机识别绿色物体并控制机械臂抓取
"""

import time
import sys
import os
import numpy as np
import cv2

# Windows 启用 ANSI 转义序列支持
if os.name == 'nt':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 2077)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from SDK import PortHandler, PacketHandler


# ───────── 常量 ─────────
SERVO_CENTER = 2047
SERVO_RANGE = 270.0
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

# 关节安全限制 (度)
JOINT_LIMITS = [
    (-114.6, 114.6),   # J1
    (0.0, 200.5),      # J2
    (-180.0, 0.0),     # J3
    (-143.2, 68.8),    # J4
    (-180.0, 180.0),   # J5
]

# 夹爪位置
GRIPPER_OPEN = 2800
GRIPPER_CLOSE = 1200

# 绿色 HSV 范围 (OpenCV HSV: H=0-180, S=0-255, V=0-255)
# 绿色在 HSV 中 H 范围约为 35-85 (对应 70-170 度)
GREEN_LOWER = (35, 50, 50)
GREEN_UPPER = (85, 255, 255)

# ANSI 颜色
class C:
    RESET = '\033[0m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    DIM = '\033[2m'


def angle_to_position(angle_deg: float) -> int:
    """角度转舵机位置"""
    pos = int(SERVO_CENTER + (angle_deg * 4095 / SERVO_RANGE))
    return max(0, min(4095, pos))


class RobotController:
    """机械臂控制器"""

    def __init__(self, port: str = "COM7", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.connected = False

    def connect(self):
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)

        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"无法设置波特率 {self.baudrate}")

        print(f"{C.GREEN}[OK]{C.RESET} 已连接到 {self.port}")
        self.connected = True

    def disconnect(self):
        if self.port_handler:
            try:
                for motor_id in MOTOR_IDS:
                    self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)
                time.sleep(0.1)
                self.port_handler.closePort()
                print(f"{C.GREEN}[OK]{C.RESET} 已断开连接")
            except:
                pass  # 忽略断开时的错误
        self.connected = False

    def enable_torque(self, motor_id: int):
        if self.connected:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 1)

    def disable_torque(self, motor_id: int):
        if self.connected:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 40, 0)

    def write_position(self, motor_id: int, position: int):
        if self.connected:
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, 42, position)

    def move_to_joints(self, joint_angles: list, duration: float = 0.5):
        """移动到目标关节角度"""
        if not self.connected:
            return

        # 限制在安全范围内
        joint_angles = np.array(joint_angles)
        for i, (angle, (min_val, max_val)) in enumerate(zip(joint_angles, JOINT_LIMITS)):
            joint_angles[i] = max(min_val, min(max_val, angle))

        # 转换为舵机位置
        positions = [angle_to_position(a) for a in joint_angles]

        # 写入位置
        for motor_id, pos in zip(MOTOR_IDS[:5], positions):
            self.write_position(motor_id, pos)

        time.sleep(duration)

    def set_gripper(self, open_state: bool = True):
        """设置夹爪"""
        if not self.connected:
            return
        pos = GRIPPER_OPEN if open_state else GRIPPER_CLOSE
        self.write_position(6, pos)
        time.sleep(0.5)


class GreenObjectDetector:
    """绿色物体检测器"""

    def __init__(self, camera_id: int = 0):
        self.camera_id = camera_id
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)

    def open_camera(self):
        """打开摄像头"""
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 {self.camera_id}")

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.frame_center = (self.frame_width // 2, self.frame_height // 2)

            print(f"{C.GREEN}[OK]{C.RESET} 摄像头已打开 ({self.frame_width}x{self.frame_height})")

    def detect_green_object(self, min_area: int = 1000):
        """检测绿色物体

        Returns:
            (center_x, center_y, width, height) 或 None
        """
        if self.cap is None:
            self.open_camera()

        ret, frame = self.cap.read()
        if not ret:
            return None

        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 创建绿色掩码
        mask = cv2.inRange(hsv, np.array(GREEN_LOWER), np.array(GREEN_UPPER))

        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area and area > best_area:
                best_area = area
                best_contour = contour

        if best_contour is None:
            return None

        # 计算轮廓中心
        M = cv2.moments(best_contour)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 边界框
        x, y, w, h = cv2.boundingRect(best_contour)

        return (cx, cy, w, h, frame)

    def release(self):
        """释放资源"""
        if self.cap:
            self.cap.release()
        try:
            cv2.destroyAllWindows()
        except:
            pass


def pixel_to_joint_offset(pixel_x: int, pixel_y: int, frame_width: int, frame_height: int) -> tuple:
    """
    将像素偏移转换为关节角度偏移

    假设相机固定在机械臂前方，向下俯视
    - X偏移 -> 关节1旋转
    - Y偏移 -> 关节2/3调整
    """
    # 计算相对于图像中心的偏移
    dx = (pixel_x - frame_width // 2) / (frame_width // 2)  # -1 到 1
    dy = (pixel_y - frame_height // 2) / (frame_height // 2)  # -1 到 1

    # 简单映射（可根据实际情况调整）
    j1_offset = dx * 15   # 左右偏移
    j2_offset = dy * 10   # 前后偏移
    j3_offset = dy * 15   # 上下偏移
    j4_offset = dy * 10   # 手腕调整

    return (j1_offset, j2_offset, j3_offset, j4_offset)


def show_detection_frame(frame, cx, cy, bbox, width, height):
    """显示检测结果"""
    frame_copy = frame.copy()
    x, y, w, h = bbox

    # 绘制边界框
    cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 绘制中心点
    cv2.circle(frame_copy, (cx, cy), 8, (0, 0, 255), -1)
    cv2.circle(frame_copy, (cx, cy), 15, (255, 255, 0), 2)

    # 绘制图像中心十字
    center_x, center_y = width // 2, height // 2
    cv2.line(frame_copy, (center_x - 30, center_y), (center_x + 30, center_y), (255, 255, 0), 1)
    cv2.line(frame_copy, (center_x, center_y - 30), (center_x, center_y + 30), (255, 255, 0), 1)

    # 计算偏移
    offset_x = cx - center_x
    offset_y = cy - center_y

    # 显示信息
    info = f"Offset: ({offset_x}, {offset_y})"
    cv2.putText(frame_copy, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Green Object Detection", frame_copy)


def run_vision_guided_grasp():
    """运行视觉引导抓取"""
    print(f"\n{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}  SO100 视觉引导抓取 - 识别绿色物体{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}{'=' * 60}{C.RESET}\n")

    # 默认参数
    camera_id = 0
    port = "COM7"
    cycles = 5
    step_delay = 0.1

    # 基础抓取姿态（关节角度）
    # J4 手腕俯仰调整为0°，让夹爪横向
    base_joints = [0, 50, -60, 0, 0]  # 基础抓取姿态，夹爪横向

    print(f"{C.CYAN}[配置]{C.RESET}")
    print(f"  摄像头: {camera_id}")
    print(f"  串口: {port}")
    print(f"  循环次数: {cycles}")
    print(f"  检测颜色: 绿色 HSV({GREEN_LOWER} - {GREEN_UPPER})")
    print(f"\n{C.YELLOW}[说明]{C.RESET}")
    print(f"  - 绿色物体将被自动检测")
    print(f"  - 机械臂将根据物体位置调整姿态")
    print(f"  - 按相机视图中心校准位置")

    # 创建检测器和控制器
    detector = GreenObjectDetector(camera_id)
    ctrl = RobotController(port)
    ctrl.connected = False  # 初始化为未连接

    try:
        # 连接硬件
        ctrl.connect()

        # 检测电机
        print(f"\n{C.CYAN}[硬件检测]{C.RESET}")
        for motor_id in MOTOR_IDS:
            _, result, _ = ctrl.packet_handler.ping(ctrl.port_handler, motor_id)
            status = f"{C.GREEN}在线{C.RESET}" if result == 0 else f"{C.RED}离线{C.RESET}"
            print(f"  Motor {motor_id}: {status}")

        # 启用扭矩
        print(f"\n{C.YELLOW}[INFO]{C.RESET} 启用电机扭矩...")
        for motor_id in MOTOR_IDS:
            ctrl.enable_torque(motor_id)
        time.sleep(0.5)

        # 先回到中位
        print(f"\n{C.CYAN}[INFO]{C.RESET} 机械臂回到中位...")
        ctrl.move_to_joints([0, 0, 0, 0, 0], 1.0)
        time.sleep(0.5)

        # 再移动到基础抓取姿态
        print(f"{C.CYAN}[INFO]{C.RESET} 移动到基础抓取姿态...")
        ctrl.move_to_joints(base_joints, 1.0)

        # input(f"\n{C.DIM}按 Enter 开始视觉引导抓取...{C.RESET}")  # 移除交互输入

        print(f"\n{C.GREEN}[START]{C.RESET} 开始视觉引导抓取...\n")

        # 不使用OpenCV窗口，直接运行检测

        lost_count = 0
        max_lost = 30

        for cycle in range(cycles):
            print(f"{C.BOLD}循环 {cycle + 1}/{cycles}{C.RESET}")

            # 检测绿色物体
            result = detector.detect_green_object()

            if result is None:
                lost_count += 1
                print(f"  {C.YELLOW}未检测到绿色物体{C.RESET} ({lost_count}/{max_lost})")
                if lost_count >= max_lost:
                    print(f"\n{C.RED}[ERROR]{C.RESET} 连续未检测到物体，停止")
                    break
                cv2.waitKey(30)
                continue

            lost_count = 0
            cx, cy, w, h, frame = result

            # 不显示检测结果窗口（OpenCV GUI不可用）
            # show_detection_frame(frame, cx, cy, (cx - w//2, cy - h//2), detector.frame_width, detector.frame_height)

            # 计算关节偏移
            j1_off, j2_off, j3_off, j4_off = pixel_to_joint_offset(
                cx, cy, detector.frame_width, detector.frame_height
            )

            # 计算目标关节角度
            target_joints = [
                base_joints[0] + j1_off,
                base_joints[1] + j2_off,
                base_joints[2] + j3_off,
                base_joints[3] + j4_off,
                base_joints[4],
            ]

            print(f"  目标关节: {[f'{j:.1f}' for j in target_joints]}")

            # 移动到目标位置
            ctrl.move_to_joints(target_joints, step_delay)

            # 闭合夹爪
            print(f"  {C.YELLOW}→ 闭合夹爪{C.RESET}")
            ctrl.set_gripper(False)
            time.sleep(0.5)

            # 抬起
            print(f"  {C.DIM}→ 抬起{C.RESET}")
            lift_joints = [target_joints[0], target_joints[1] - 20, target_joints[2] + 20, target_joints[3], target_joints[4]]
            ctrl.move_to_joints(lift_joints, 0.5)

            time.sleep(0.5)

            # 张开夹爪
            print(f"  {C.GREEN}→ 张开夹爪{C.RESET}")
            ctrl.set_gripper(True)

            # 回到基础姿态
            print(f"  {C.DIM}→ 回到基础姿态{C.RESET}")
            ctrl.move_to_joints(base_joints, 0.5)

            # 检测按键（非阻塞，仅检查是否需要退出）
            # 由于cv2.waitKey不可用，自动完成所有循环

            print(f"  {C.GREEN}[OK] 循环完成{C.RESET}\n")

            # if cv2.waitKey(100) & 0xFF == ord('q'):
            #     print(f"\n{C.YELLOW}[中断]{C.RESET} 用户取消")
            #     break

        cv2.destroyAllWindows()

        # 回到初始位置
        print(f"\n{C.DIM}回到初始位置...{C.RESET}")
        ctrl.move_to_joints([0, 0, 0, 0, 0], 1.5)
        print(f"\n{C.GREEN}[完成]{C.RESET} 视觉引导抓取完成！")

    except KeyboardInterrupt:
        print(f"\n\n{C.YELLOW}[中断]{C.RESET} 用户中断")
    except Exception as e:
        print(f"\n{C.RED}[ERROR]{C.RESET} {e}")
        import traceback
        traceback.print_exc()
    finally:
        ctrl.disconnect()
        detector.release()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="SO100 视觉引导抓取")
    parser.add_argument('--camera', type=int, default=0, help='摄像头ID')
    parser.add_argument('--port', type=str, default='COM7', help='串口')
    parser.add_argument('--cycles', type=int, default=5, help='循环次数')
    parser.add_argument('--delay', type=float, default=0.1, help='动作延迟(秒)')

    args = parser.parse_args()

    # 更新全局变量
    global cycles, step_delay
    cycles = args.cycles
    step_delay = args.delay

    run_vision_guided_grasp()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{C.CYAN}[再见]{C.RESET}")
    finally:
        print(C.RESET)
