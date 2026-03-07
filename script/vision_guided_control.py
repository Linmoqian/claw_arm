#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 视觉引导笛卡尔控制
整合视觉检测、运动学解算和硬件控制
"""

import sys
import os
import time
import json
import cv2
import numpy as np
from typing import Optional, Tuple, List

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from SDK import PortHandler, PacketHandler

import roboticstoolbox as rtb
from spatialmath import SE3
from scipy.optimize import minimize


# ───────── 运动学模型 ─────────
class SO100Kinematics:
    """SO100 运动学模型"""

    DH_PARAMS = [
        (0.0, np.pi/2, 0.0165, np.pi/2),     # J1: rotation
        (0.0, -np.pi/2, 0.1478, 0.0),         # J2: pitch
        (0.0, 0.0, 0.14057, 0.0),             # J3: elbow
        (0.0, 0.0, 0.1349, 0.0),              # J4: wrist_pitch
        (0.0, np.pi/2, 0.0, 0.0),             # J5: wrist_roll
    ]

    JOINT_LIMITS = [
        (-170, 170),   # rotation
        (-85, 85),     # pitch
        (-150, 150),   # elbow
        (-120, 120),   # wrist_pitch
        (-180, 180),   # wrist_roll
    ]

    MOTOR_MAP = {'rotation': 1, 'pitch': 2, 'elbow': 3, 'wrist_pitch': 4, 'wrist_roll': 5}

    def __init__(self):
        self.robot = rtb.DHRobot(
            [rtb.RevoluteDH(a=row[0], alpha=row[1], d=row[2], offset=row[3]) for row in self.DH_PARAMS],
            name='SO100',
            tool=SE3(0, 0, 0.0601)  # 末端到夹爪的距离
        )

        qlim_matrix = np.array([[np.deg2rad(lo), np.deg2rad(hi)] for lo, hi in self.JOINT_LIMITS])
        self.robot.qlim = qlim_matrix.T

    def fk(self, joint_angles: dict) -> SE3:
        """正向运动学"""
        q = np.zeros(5)
        q[0] = np.deg2rad(joint_angles.get('rotation', 0))
        q[1] = np.deg2rad(joint_angles.get('pitch', 0))
        q[2] = np.deg2rad(joint_angles.get('elbow', 0))
        q[3] = np.deg2rad(joint_angles.get('wrist_pitch', 0))
        q[4] = np.deg2rad(joint_angles.get('wrist_roll', 0))
        return self.robot.fkine(q)

    def ik(self, target_pos: np.ndarray, current_angles: dict) -> Optional[dict]:
        """逆向运动学 - 位置控制"""
        def objective(q):
            angles = {'rotation': np.rad2deg(q[0]), 'pitch': np.rad2deg(q[1]),
                     'elbow': np.rad2deg(q[2]), 'wrist_pitch': np.rad2deg(q[3]), 'wrist_roll': np.rad2deg(q[4])}
            pose = self.fk(angles)
            return np.sum((pose.t - target_pos)**2)

        q0 = np.array([np.deg2rad(current_angles.get(k, 0)) for k in ['rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll']])
        bounds = [(np.deg2rad(lo), np.deg2rad(hi)) for lo, hi in self.JOINT_LIMITS]

        result = minimize(objective, q0, method='SLSQP', bounds=bounds, options={'ftol': 1e-6, 'maxiter': 100})

        if result.success and result.fun < 0.001:  # 误差小于1mm
            q = result.x
            return {
                'rotation': np.rad2deg(q[0]),
                'pitch': np.rad2deg(q[1]),
                'elbow': np.rad2deg(q[2]),
                'wrist_pitch': np.rad2deg(q[3]),
                'wrist_roll': np.rad2deg(q[4]),
            }
        return None


# ───────── 硬件控制器 ─────────
class SO100Hardware:
    """SO100 硬件控制"""

    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56

    def __init__(self, port: str = "COM7"):
        self.port = port
        self.port_handler = None
        self.packet_handler = None

    def connect(self):
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)
        if not self.port_handler.openPort():
            raise RuntimeError(f"Cannot open port {self.port}")
        self.port_handler.setBaudRate(1000000)
        print(f"[OK] Connected to {self.port}")

        # 检测电机
        found = self.ping_all()
        print(f"[OK] Motors detected: {found}/6")
        if len(found) < 6:
            print(f"[WARN] Some motors offline!")

        self.enable_all()

    def disconnect(self):
        if self.port_handler:
            self.disable_all()
            self.port_handler.closePort()
            print("[OK] Disconnected")

    def enable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.TORQUE_ENABLE, 0)

    def enable_all(self):
        for m in range(1, 7):
            self.enable_torque(m)
        print("[OK] Torque enabled")

    def disable_all(self):
        for m in range(1, 7):
            self.disable_torque(m)

    def set_position(self, motor_id: int, position: int):
        position = max(0, min(4095, position))
        self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, self.GOAL_POSITION, position)

    def get_position(self, motor_id: int) -> int:
        data, result, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, self.PRESENT_POSITION)
        return data if result == 0 else 2047

    def ping(self, motor_id: int) -> bool:
        _, result, _ = self.packet_handler.ping(self.port_handler, motor_id)
        return result == 0

    def ping_all(self) -> List[int]:
        return [m for m in range(1, 7) if self.ping(m)]

    def set_joint_angles(self, joint_angles: dict):
        """设置关节角度"""
        for name, angle in joint_angles.items():
            if name in SO100Kinematics.MOTOR_MAP:
                motor_id = SO100Kinematics.MOTOR_MAP[name]
                pos = angle_to_position(angle)
                self.set_position(motor_id, pos)

    def get_joint_angles(self) -> dict:
        """获取当前关节角度"""
        positions = {m: self.get_position(m) for m in range(1, 7)}
        return {name: position_to_angle(positions[motor_id])
                for name, motor_id in SO100Kinematics.MOTOR_MAP.items()}

    def open_gripper(self, pos: int = 2800):
        """打开夹爪"""
        self.set_position(6, pos)
        time.sleep(0.3)

    def close_gripper(self, pos: int = 1500):
        """关闭夹爪"""
        self.set_position(6, pos)
        time.sleep(0.3)


# ───────── 坐标转换 ─────────
def angle_to_position(angle_deg: float, center: int = 2047, range_deg: float = 270) -> int:
    """角度 -> 舵机位置"""
    return int(center + angle_deg * (4095 / range_deg))

def position_to_angle(pos: int, center: int = 2047, range_deg: float = 270) -> float:
    """舵机位置 -> 角度"""
    return (pos - center) * (range_deg / 4095)


# ───────── 视觉检测器 (简化版) ─────────
class SimpleVisionDetector:
    """简化的视觉检测器"""

    def __init__(self, camera_id: int = 0, calibration_file: str = None):
        self.camera_id = camera_id
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center = (320, 240)

        # 标定参数
        self.calibration_file = calibration_file or "skills/vision-control/hand_eye_calibration.json"
        self.calibration_data = self._load_calibration()

        # YOLO 模型
        self.model = None

    def _load_calibration(self) -> dict:
        """加载手眼标定数据"""
        default_calib = {
            "camera_offset_x": 0.17,    # 相机相对底座的X偏移 (m)
            "camera_offset_y": 0.0,     # 相机相对底座的Y偏移 (m)
            "camera_height": 0.3,       # 相机高度 (m)
            "scale_x": 0.0003,          # 像素到X方向的转换系数 (m/pixel)
            "scale_y": 0.0003,          # 像素到Y方向的转换系数 (m/pixel)
            "workplane_height": 0.05,   # 工作平面高度 (m)
        }

        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    data = json.load(f)
                    # 从手眼矩阵提取平移
                    if "translation" in data:
                        default_calib["camera_offset_x"] = 0.17
                        default_calib["camera_offset_y"] = 0.0
                return default_calib
            except:
                pass

        return default_calib

    def open_camera(self):
        """打开摄像头"""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {self.camera_id}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)
        print(f"[OK] Camera opened: {self.frame_width}x{self.frame_height}")

    def load_yolo(self, model_path: str = "yolov8n.pt"):
        """加载 YOLO 模型"""
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            print(f"[OK] YOLO model loaded: {model_path}")
        except ImportError:
            print("[WARN] ultralytics not installed, using color detection")
            self.model = None

    def detect(self, target_class: str = "cup", confidence: float = 0.5) -> Optional[dict]:
        """
        检测物体并返回笛卡尔坐标

        Returns:
            {"x": float, "y": float, "z": float, "pixel": (x, y), "class": str, "confidence": float}
        """
        if self.cap is None:
            self.open_camera()

        ret, frame = self.cap.read()
        if not ret:
            return None

        results = None

        # 使用 YOLO 检测
        if self.model is not None:
            yolo_results = self.model(frame, conf=confidence, verbose=False)
            if yolo_results and len(yolo_results) > 0:
                results = yolo_results[0]

        best_detection = None

        if results is not None:
            # 解析 YOLO 结果
            boxes = results.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                conf = float(box.conf[0])

                # 匹配目标类别
                if target_class.lower() not in class_name.lower():
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                pixel_x = int((x1 + x2) / 2)
                pixel_y = int((y1 + y2) / 2)

                best_detection = {
                    "pixel": (pixel_x, pixel_y),
                    "class": class_name,
                    "confidence": conf,
                    "bbox": (int(x1), int(y1), int(x2), int(y2))
                }
                break
        else:
            # 颜色检测（红色物体）
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # 红色范围
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 500:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        x, y, w, h = cv2.boundingRect(largest)
                        best_detection = {
                            "pixel": (cx, cy),
                            "class": "red_object",
                            "confidence": 1.0,
                            "bbox": (x, y, x+w, y+h)
                        }

        if best_detection:
            # 像素坐标转笛卡尔坐标
            pixel_x, pixel_y = best_detection["pixel"]

            # 归一化像素坐标（相对于中心）
            norm_x = (pixel_x - self.frame_center[0]) / self.frame_center[0]
            norm_y = (pixel_y - self.frame_center[1]) / self.frame_center[1]

            # 转换到机械臂坐标
            arm_x = self.calibration_data["camera_offset_x"] + norm_x * 0.15  # +/- 7.5cm
            arm_y = self.calibration_data["camera_offset_y"] + norm_y * 0.15
            arm_z = self.calibration_data["workplane_height"]

            best_detection["x"] = arm_x
            best_detection["y"] = arm_y
            best_detection["z"] = arm_z

            # 绘制检测框
            frame_copy = frame.copy()
            x1, y1, x2, y2 = best_detection.get("bbox", (pixel_x-20, pixel_y-20, pixel_x+20, pixel_y+20))
            cv2.rectangle(frame_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame_copy, (pixel_x, pixel_y), 5, (0, 0, 255), -1)
            cv2.putText(frame_copy, f"{best_detection['class']}: ({arm_x:.3f}, {arm_y:.3f})",
                       (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow("Vision Detection", frame_copy)
            cv2.waitKey(1)

            return best_detection

        return None

    def release(self):
        """释放摄像头"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


# ───────── 主控制类 ─────────
class VisionGuidedArm:
    """视觉引导机械臂控制"""

    def __init__(self, port: str = "COM7", camera_id: int = 0):
        self.kinematics = SO100Kinematics()
        self.hardware = SO100Hardware(port)
        self.vision = SimpleVisionDetector(camera_id)
        self.running = False

    def connect(self):
        """连接所有设备"""
        print("="*60)
        print("  SO100 Vision-Guided Cartesian Control")
        print("="*60)

        self.hardware.connect()
        self.vision.open_camera()

        # 可选：加载 YOLO
        try:
            self.vision.load_yolo()
        except:
            print("[INFO] YOLO not available, using color detection")

    def disconnect(self):
        """断开所有设备"""
        self.vision.release()
        self.hardware.disconnect()

    def move_to_cartesian(self, x: float, y: float, z: float) -> bool:
        """移动到笛卡尔坐标"""
        current = self.hardware.get_joint_angles()

        # IK 解算
        result = self.kinematics.ik(np.array([x, y, z]), current)

        if result:
            self.hardware.set_joint_angles(result)
            time.sleep(0.5)

            # 验证位置
            actual_angles = self.hardware.get_joint_angles()
            pose = self.kinematics.fk(actual_angles)
            error = np.linalg.norm(pose.t - np.array([x, y, z]))

            print(f"[Move] Target: ({x:.3f}, {y:.3f}, {z:.3f}) -> Actual: ({pose.t[0]:.3f}, {pose.t[1]:.3f}, {pose.t[2]:.3f}) Error: {error*1000:.1f}mm")
            return True
        else:
            print(f"[FAIL] IK no solution for ({x:.3f}, {y:.3f}, {z:.3f})")
            return False

    def pick(self, x: float, y: float, z: float, grip_z: float = None) -> bool:
        """抓取动作"""
        # 1. 移动到上方安全位置
        safe_z = max(z + 0.05, 0.15)
        print(f"\n[1/4] Moving to safe position: z={safe_z:.3f}")
        self.move_to_cartesian(x, y, safe_z)

        # 2. 下降到抓取高度
        grip_z = grip_z if grip_z else z
        print(f"[2/4] Descending to grip height: z={grip_z:.3f}")
        self.move_to_cartesian(x, y, grip_z)

        # 3. 闭合夹爪
        print("[3/4] Closing gripper")
        self.hardware.close_gripper(1500)

        # 4. 抬起
        print("[4/4] Lifting")
        self.move_to_cartesian(x, y, safe_z)

        return True

    def place(self, x: float, y: float, z: float) -> bool:
        """放置动作"""
        safe_z = max(z + 0.05, 0.15)

        # 1. 移动到上方
        print(f"\n[1/3] Moving above: ({x:.3f}, {y:.3f}, {safe_z:.3f})")
        self.move_to_cartesian(x, y, safe_z)

        # 2. 下降
        print(f"[2/3] Descending: z={z:.3f}")
        self.move_to_cartesian(x, y, z)

        # 3. 打开夹爪
        print("[3/3] Opening gripper")
        self.hardware.open_gripper(2800)
        self.move_to_cartesian(x, y, safe_z)

        return True

    def track_and_pick(self, target_class: str = "cup", max_attempts: int = 3) -> bool:
        """追踪并抓取物体"""
        print(f"\n[Tracking] Looking for '{target_class}'...")

        for attempt in range(max_attempts):
            detection = self.vision.detect(target_class)

            if detection:
                x, y, z = detection["x"], detection["y"], detection["z"]
                print(f"[Found] {detection['class']} at ({x:.3f}, {y:.3f}, {z:.3f}) confidence: {detection['confidence']:.2f}")

                # 执行抓取
                return self.pick(x, y, z)
            else:
                print(f"[Attempt {attempt+1}/{max_attempts}] No object detected")
                time.sleep(0.5)

        print("[FAIL] Object not found after {max_attempts} attempts")
        return False

    def run_demo(self, duration: float = 60):
        """运行演示模式"""
        self.running = True
        start_time = time.time()

        print(f"\n[Demo] Running for {duration}s...")
        print("Press Ctrl+C to stop\n")

        try:
            while self.running and (time.time() - start_time) < duration:
                # 检测并追踪
                detection = self.vision.detect("cup")

                if detection:
                    x, y, z = detection["x"], detection["y"], detection["z"]
                    # 只移动，不抓取
                    self.move_to_cartesian(x, y, 0.15)  # 安全高度

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n[Demo] Interrupted by user")

        self.running = False


# ───────── 主程序 ─────────
def main():
    import argparse

    parser = argparse.ArgumentParser(description="SO100 Vision-Guided Control")
    parser.add_argument('--port', type=str, default='COM7', help='Serial port')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID')
    parser.add_argument('--mode', type=str, default='demo',
                       choices=['demo', 'pick', 'track', 'test'], help='Operation mode')
    parser.add_argument('--target', type=str, default='cup', help='Target object class')
    parser.add_argument('--x', type=float, default=0.17, help='Target X (m)')
    parser.add_argument('--y', type=float, default=0.0, help='Target Y (m)')
    parser.add_argument('--z', type=float, default=0.05, help='Target Z (m)')
    parser.add_argument('--duration', type=int, default=60, help='Demo duration (s)')

    args = parser.parse_args()

    controller = VisionGuidedArm(args.port, args.camera)

    try:
        controller.connect()

        if args.mode == 'demo':
            controller.run_demo(args.duration)

        elif args.mode == 'pick':
            # 直接抓取指定坐标
            controller.pick(args.x, args.y, args.z)

        elif args.mode == 'track':
            # 追踪并抓取
            controller.track_and_pick(args.target)

        elif args.mode == 'test':
            # 测试运动学
            print("\n[Test] Forward Kinematics:")
            angles = controller.hardware.get_joint_angles()
            pose = controller.kinematics.fk(angles)
            print(f"  Joint angles: {angles}")
            print(f"  End effector: x={pose.t[0]:.3f}, y={pose.t[1]:.3f}, z={pose.t[2]:.3f}")

            print("\n[Test] Inverse Kinematics:")
            target = np.array([args.x, args.y, args.z])
            result = controller.kinematics.ik(target, angles)
            if result:
                print(f"  Target: ({args.x:.3f}, {args.y:.3f}, {args.z:.3f})")
                print(f"  Solution: {result}")

                # 测试移动
                response = input("\nExecute movement? (y/n): ")
                if response.lower() == 'y':
                    controller.move_to_cartesian(args.x, args.y, args.z)
            else:
                print("  No IK solution")

    except KeyboardInterrupt:
        print("\n[Interrupted]")

    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
