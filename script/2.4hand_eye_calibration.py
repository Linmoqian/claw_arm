"""
手眼标定 (眼在手上) - 计算相机与机械臂基座的变换关系
"""
import cv2
import numpy as np
import os
import sys
import json
import glob

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

PHOTO_DIR = os.path.join(os.path.dirname(__file__), "..", "photo-base")

# 棋盘格参数 (A4纸: 210x297mm)
CHESS_ROWS = 6
CHESS_COLS = 9
SQUARE_SIZE = 30  # mm (A4纸上约 10x7 格，每格约30mm)

# 相机内参
CAMERA_MATRIX = np.array([
    [353.51230935, 0, 348.17688948],
    [0, 348.98897773, 236.36002702],
    [0, 0, 1]
], dtype=np.float64)

DIST_COEFFS = np.array([0.10569028, -0.14866043, -0.00291527, 0.01176614, 0.06308175])


class ArmKinematics:
    """SO100 正运动学"""

    def rotation_matrix(self, axis: str, angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        if axis == 'x':
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        elif axis == 'y':
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        else:
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

    def forward(self, joint_angles: list) -> np.ndarray:
        """6个关节角度 → 末端4x4变换矩阵"""
        T = np.eye(4)
        for i, angle in enumerate(joint_angles):
            T = T @ self.joint_transform(i + 1, angle)
        return T


def pos_to_angle(pos: int) -> float:
    """电机位置(0-4095) → 角度(弧度)"""
    return (pos - 2047) / 2047 * np.pi


def main():
    # 准备棋盘格3D点
    objp = np.zeros((CHESS_ROWS * CHESS_COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESS_COLS, 0:CHESS_ROWS].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    # 收集数据
    R_gripper2base = []  # 末端相对于基座的旋转
    t_gripper2base = []  # 末端相对于基座的平移
    R_target2cam = []    # 标定板相对于相机的旋转
    t_target2cam = []    # 标定板相对于相机的平移

    kinematics = ArmKinematics()

    images = sorted(glob.glob(os.path.join(PHOTO_DIR, "*.jpg")))
    print(f"找到 {len(images)} 张图片\n")

    valid = 0
    for img_path in images:
        num = os.path.basename(img_path).split('.')[0]
        json_path = os.path.join(PHOTO_DIR, f"{num}.json")

        if not os.path.exists(json_path):
            print(f"✗ {num}.jpg - 缺少JSON")
            continue

        # 读取图片，检测棋盘格
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS), None)

        if not ret:
            print(f"✗ {num}.jpg - 未检测到棋盘")
            continue

        # 亚像素精化
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        # PnP: 标定板相对于相机
        ok, rvec, tvec = cv2.solvePnP(objp, corners, CAMERA_MATRIX, DIST_COEFFS)
        if not ok:
            print(f"✗ {num}.jpg - PnP失败")
            continue

        R_tc, _ = cv2.Rodrigues(rvec)

        # 读取电机位置，计算末端相对于基座
        with open(json_path, 'r') as f:
            data = json.load(f)

        positions = data['positions']
        angles = [pos_to_angle(positions[str(i)]) for i in range(1, 7)]
        T_gb = kinematics.forward(angles)

        R_gripper2base.append(T_gb[:3, :3])
        t_gripper2base.append(T_gb[:3, 3])
        R_target2cam.append(R_tc)
        t_target2cam.append(tvec.flatten())

        valid += 1
        print(f"✓ {num}.jpg")

    print(f"\n有效数据: {valid} 组")

    if valid < 5:
        print("数据不足，至少需要5组")
        return

    # 手眼标定 (眼在手外: CALIB_HAND_EYE_TSAI)
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    print("\n" + "=" * 50)
    print("相机到基座的变换 (眼在手上):")
    print("\n旋转矩阵 R_cam2base:")
    print(R_cam2base)
    print("\n平移向量 t_cam2base (mm):")
    print(t_cam2base)

    # 保存结果
    result = {
        "R_cam2base": R_cam2base.tolist(),
        "t_cam2base": t_cam2base.tolist()
    }
    save_path = os.path.join(PHOTO_DIR, "hand_eye_calibration.json")
    with open(save_path, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"\n已保存: {save_path}")


if __name__ == "__main__":
    main()
