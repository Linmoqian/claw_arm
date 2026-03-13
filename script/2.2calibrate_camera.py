"""
相机内参标定计算 - 使用棋盘格图片
"""
import cv2
import numpy as np
import os
import glob

PHOTO_DIR = r"d:\project\claw_arm-1\photo"

# 棋盘格参数 (内角点数量)
CHESS_ROWS = 6  # 行
CHESS_COLS = 9  # 列

# A4纸上的实际方格大小 (mm)，10x7格，每格约30mm
SQUARE_SIZE = 30


def main():
    # 准备物体点 (3D)
    objp = np.zeros((CHESS_ROWS * CHESS_COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESS_COLS, 0:CHESS_ROWS].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []  # 3D点
    imgpoints = []  # 2D点

    images = glob.glob(os.path.join(PHOTO_DIR, "*.jpg"))
    if not images:
        print(f"未找到图片: {PHOTO_DIR}")
        return

    print(f"找到 {len(images)} 张图片")

    for path in images:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (CHESS_COLS, CHESS_ROWS), None)

        if ret:
            objpoints.append(objp)
            # 亚像素精化
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            imgpoints.append(corners)

            # 显示
            cv2.drawChessboardCorners(img, (CHESS_COLS, CHESS_ROWS), corners, ret)
            cv2.imshow("Detect", img)
            cv2.waitKey(300)
            print(f"✓ {os.path.basename(path)}")
        else:
            print(f"✗ {os.path.basename(path)} - 未检测到棋盘")

    cv2.destroyAllWindows()

    if not objpoints:
        print("没有检测到任何棋盘格")
        return

    # 标定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\n" + "=" * 40)
    print("相机内参矩阵:")
    print(mtx)
    print("\n畸变系数:")
    print(dist.ravel())
    print(f"\n重投影误差: {ret:.4f}")

    # 保存结果
    np.savez(os.path.join(PHOTO_DIR, "calibration.npz"),
             mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
    print(f"\n已保存: {PHOTO_DIR}\\calibration.npz")


if __name__ == "__main__":
    main()
