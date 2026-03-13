"""
PnP 位姿估计 - 绿色瓶盖抓取
"""
import cv2
import numpy as np
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from SDK import PortHandler, PacketHandler


class Camera:
    """相机参数"""

    def __init__(self):
        self.matrix = np.array([
            [353.51230935, 0, 348.17688948],
            [0, 348.98897773, 236.36002702],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist = np.array([0.10569028, -0.14866043, -0.00291527, 0.01176614, 0.06308175])


class GreenDetector:
    """绿色瓶盖检测器"""

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (35, 50, 50), (85, 255, 255))
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 100:
            return None, mask

        M = cv2.moments(c)
        if M["m00"] == 0:
            return None, mask

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy), mask


class PoseEstimator:
    """PnP 位姿估计器 - 28mm 瓶盖"""

    def __init__(self, camera: Camera):
        self.camera = camera
        r = 14  # 半径 14mm
        self.object_points = np.array([
            [-r, -r, 0], [r, -r, 0], [r, r, 0], [-r, r, 0]
        ], dtype=np.float64)

    def estimate(self, center):
        cx, cy = center
        r = 20  # 像素半径
        image_points = np.array([
            [cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]
        ], dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            self.object_points, image_points,
            self.camera.matrix, self.camera.dist
        )
        if not ok:
            return None, None

        R, _ = cv2.Rodrigues(rvec)
        return R, tvec


class ArmKinematics:
    """SO100 正运动学"""

    def __init__(self):
        self.joint_limits = [
            (-2.0, 2.0), (0.0, 3.5), (-3.14158, 0.0),
            (-2.5, 1.2), (-3.14158, 3.14158), (-0.2, 2.0)
        ]

    def rotation_matrix(self, axis: str, angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        if axis == 'x':
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        elif axis == 'y':
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def joint_transform(self, joint_id: int, angle: float) -> np.ndarray:
        T = np.eye(4)
        if joint_id == 1:
            T[:3, :3] = self.rotation_matrix('y', angle)
            T[:3, 3] = [0, -0.0452, 0.0165]
        elif joint_id == 2:
            T[:3, :3] = self.rotation_matrix('x', -1.8) @ self.rotation_matrix('x', angle)
            T[:3, 3] = [0, 0.1025, 0.0306]
        elif joint_id == 3:
            T[:3, :3] = self.rotation_matrix('x', np.pi/2) @ self.rotation_matrix('x', angle)
            T[:3, 3] = [0, 0.11257, 0.028]
        elif joint_id == 4:
            T[:3, :3] = self.rotation_matrix('x', -1.0) @ self.rotation_matrix('x', angle)
            T[:3, 3] = [0, 0.0052, 0.1349]
        elif joint_id == 5:
            T[:3, :3] = self.rotation_matrix('y', np.pi/2) @ self.rotation_matrix('y', angle)
            T[:3, 3] = [0, -0.0601, 0]
        elif joint_id == 6:
            T[:3, :3] = self.rotation_matrix('z', np.pi) @ self.rotation_matrix('z', angle)
            T[:3, 3] = [-0.0202, -0.0244, 0]
        return T

    def forward(self, joint_angles: list) -> tuple:
        T = np.eye(4)
        for i, angle in enumerate(joint_angles):
            T = T @ self.joint_transform(i + 1, angle)
        return T[:3, :3], T[:3, 3]

    def inverse_simple(self, target_position: np.ndarray) -> list:
        q = [0.0, 1.5, -1.0, 0.0, 0.0, 0.0]
        _, current_pos = self.forward(q)

        delta = 0.001
        J = np.zeros((3, 6))
        for j in range(6):
            q_plus = q.copy()
            q_plus[j] += delta
            _, pos_plus = self.forward(q_plus)
            J[:, j] = (pos_plus - current_pos) / delta

        error = target_position - current_pos
        dq = np.linalg.pinv(J) @ error
        for j in range(6):
            q[j] = np.clip(q[j] + dq[j], self.joint_limits[j][0], self.joint_limits[j][1])
        return q


class MotorController:
    """电机控制器"""

    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56

    def __init__(self, port: str = "COM12", baudrate: int = 1000000):
        self.port = PortHandler(port)
        self.pkt = PacketHandler(0.0)
        self.connected = False

    def connect(self) -> bool:
        if self.port.openPort() and self.port.setBaudRate(1000000):
            self.connected = True
            return True
        return False

    def disconnect(self):
        if self.connected:
            for i in range(1, 7):
                self.disable_torque(i)
            self.port.closePort()

    def enable_torque(self, motor_id: int):
        self.pkt.write1ByteTxRx(self.port, motor_id, self.TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int):
        self.pkt.write1ByteTxRx(self.port, motor_id, self.TORQUE_ENABLE, 0)

    def set_position(self, motor_id: int, position: int):
        position = max(0, min(4095, position))
        self.pkt.write2ByteTxRx(self.port, motor_id, self.GOAL_POSITION, position)

    def get_position(self, motor_id: int) -> int:
        data, result, _ = self.pkt.read2ByteTxRx(self.port, motor_id, self.PRESENT_POSITION)
        return data if result == 0 else -1

    def angle_to_pos(self, angle: float, motor_id: int = 1) -> int:
        """弧度 → 位置 (0-4095)"""
        return int(2047 + angle * 651.5)

    def move_angles(self, angles: list):
        """移动到指定关节角度 (弧度)"""
        for i, angle in enumerate(angles):
            pos = self.angle_to_pos(angle, i + 1)
            self.set_position(i + 1, pos)


class Arm:
    """机械臂控制器"""

    def __init__(self, motor: MotorController):
        self.motor = motor
        self.kinematics = ArmKinematics()
        # 手眼标定结果
        self.R_cam2base = np.array([
            [0.99412471, -0.06886927, -0.08350505],
            [0.1068609, 0.50166444, 0.85843668],
            [-0.0172284, -0.86231654, 0.50607644]
        ])
        self.t_cam2base = np.array([28.40029296, -39.21941828, 42.11770538])

    def camera_to_base(self, t_cam: np.ndarray) -> np.ndarray:
        """相机坐标 → 基座坐标 (mm)"""
        return self.R_cam2base @ t_cam.flatten() + self.t_cam2base

    def move_to(self, t_cam: np.ndarray):
        """移动到相机坐标系下的位置"""
        t_base = self.camera_to_base(t_cam)
        # mm → m
        t_base_m = t_base / 1000
        joint_angles = self.kinematics.inverse_simple(t_base_m)

        print(f"[Arm] 目标: ({t_base[0]:.0f}, {t_base[1]:.0f}, {t_base[2]:.0f}) mm")
        print(f"[Arm] 关节: {[f'{np.degrees(a):.1f}' for a in joint_angles]} deg")

        if self.motor.connected:
            self.motor.move_angles(joint_angles)

    def grasp(self, close: bool = True):
        """抓取/释放"""
        pos = 1000 if close else 2500
        if self.motor.connected:
            self.motor.set_position(6, pos)
        print(f"[Gripper] {'闭合' if close else '张开'}")


def main():
    camera = Camera()
    detector = GreenDetector()
    estimator = PoseEstimator(camera)
    motor = MotorController("COM12")

    if not motor.connect():
        print("警告: 无法连接电机，仅显示位置")

    arm = Arm(motor)

    # 启用扭矩
    for i in range(1, 7):
        motor.enable_torque(i)

    cap = cv2.VideoCapture(1)
    print("操作: 空格=抓取  g=释放  q=退出\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            center, mask = detector.detect(frame)
            tvec = None

            if center:
                cx, cy = center
                cv2.line(frame, (cx-20, cy), (cx+20, cy), (0, 255, 0), 2)
                cv2.line(frame, (cx, cy-20), (cx, cy+20), (0, 255, 0), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                R, tvec = estimator.estimate(center)
                if tvec is not None:
                    x, y, z = tvec.flatten()
                    cv2.putText(frame, f"({x:.0f}, {y:.0f}, {z:.0f})mm",
                                (cx+15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("PnP", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord(' ') and tvec is not None:
                arm.move_to(tvec)
                time.sleep(1)
                arm.grasp(True)
            elif key == ord('g'):
                arm.grasp(False)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        motor.disconnect()


if __name__ == "__main__":
    main()
