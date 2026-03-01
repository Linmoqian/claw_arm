#!/usr/bin/env python3
"""
vision-grasp skill 主脚本
===========================
视觉定位抓取技能：通过摄像头识别目标物体位置，驱动机械臂完成抓取。

支持三种检测后端：
  1. YOLOv8 本地推理
  2. 通义千问视觉大模型 API
  3. OpenCV 颜色检测（轻量回退）

适配机型：Galaxea R1 Lite （3 摄像头）
"""

import argparse
import json
import logging
import math
import os
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# ============================================================
#  日志
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("vision-grasp")


# ============================================================
#  数据结构
# ============================================================

class CameraType(Enum):
    """摄像头类型"""
    HIGH = "high"               # 全局俯视摄像头
    LEFT_WRIST = "left_wrist"   # 左臂腕部摄像头
    RIGHT_WRIST = "right_wrist" # 右臂腕部摄像头


class DetectorType(Enum):
    """检测后端类型"""
    YOLO = "yolo"
    QWEN_VL = "qwen_vl"
    COLOR = "color"


@dataclass
class BoundingBox:
    """检测边界框"""
    x1: float  # 左上角 x
    y1: float  # 左上角 y
    x2: float  # 右下角 x
    y2: float  # 右下角 y

    @property
    def center(self) -> Tuple[float, float]:
        """边界框中心像素坐标"""
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass
class Detection:
    """单个检测结果"""
    label: str          # 物体类别名称
    confidence: float   # 置信度 [0, 1]
    bbox: BoundingBox   # 边界框（像素坐标）
    mask: Optional[np.ndarray] = None  # 可选的分割掩码

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "confidence": round(self.confidence, 3),
            "bbox": {
                "x1": round(self.bbox.x1, 1),
                "y1": round(self.bbox.y1, 1),
                "x2": round(self.bbox.x2, 1),
                "y2": round(self.bbox.y2, 1),
            },
            "center_px": [round(c, 1) for c in self.bbox.center],
        }


@dataclass
class Pose3D:
    """3D 位姿"""
    x: float = 0.0   # 米
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0   # 弧度
    pitch: float = 0.0
    yaw: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw])

    def to_dict(self) -> dict:
        return {
            "position_m": {"x": round(self.x, 4), "y": round(self.y, 4), "z": round(self.z, 4)},
            "orientation_rad": {"roll": round(self.roll, 4), "pitch": round(self.pitch, 4), "yaw": round(self.yaw, 4)},
        }


@dataclass
class GraspPlan:
    """抓取规划结果"""
    target_label: str
    detection: Detection
    object_pose: Pose3D          # 物体在基座标系下的 3D 位姿
    pre_grasp_pose: Pose3D       # 预抓取位姿（物体上方）
    grasp_pose: Pose3D           # 抓取位姿
    joint_angles_pre: Optional[List[float]] = None   # 预抓取关节角度
    joint_angles_grasp: Optional[List[float]] = None # 抓取关节角度


# ============================================================
#  相机参数与标定
# ============================================================

@dataclass
class CameraIntrinsics:
    """相机内参"""
    fx: float = 600.0   # 焦距 x (像素)
    fy: float = 600.0   # 焦距 y (像素)
    cx: float = 640.0   # 主点 x (像素)
    cy: float = 360.0   # 主点 y (像素)
    width: int = 1280
    height: int = 720

    @property
    def K(self) -> np.ndarray:
        """3×3 相机内参矩阵"""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1],
        ], dtype=np.float64)


# 默认相机内参（需根据实际标定结果替换）
DEFAULT_INTRINSICS = {
    CameraType.HIGH: CameraIntrinsics(fx=600, fy=600, cx=640, cy=360),
    CameraType.LEFT_WRIST: CameraIntrinsics(fx=600, fy=600, cx=640, cy=360),
    CameraType.RIGHT_WRIST: CameraIntrinsics(fx=600, fy=600, cx=640, cy=360),
}

# 默认手眼标定矩阵（相机 → 基座标系，4×4 齐次变换）
# 实际使用前需替换为标定结果
DEFAULT_EXTRINSICS = {
    CameraType.HIGH: np.eye(4, dtype=np.float64),
    CameraType.LEFT_WRIST: np.eye(4, dtype=np.float64),
    CameraType.RIGHT_WRIST: np.eye(4, dtype=np.float64),
}

# 默认深度估计值（米），无深度相机时的回退
DEFAULT_DEPTH_M = 0.5


# ============================================================
#  图像采集
# ============================================================

class CameraCapture:
    """摄像头图像采集"""

    def __init__(self, camera_type: CameraType, device_id: int = 0):
        self.camera_type = camera_type
        self.device_id = device_id
        self._cap = None

    def open(self) -> bool:
        """打开摄像头"""
        try:
            import cv2
            # 根据摄像头类型选择设备 ID
            device_map = {
                CameraType.HIGH: self.device_id,
                CameraType.LEFT_WRIST: self.device_id + 1,
                CameraType.RIGHT_WRIST: self.device_id + 2,
            }
            dev = device_map.get(self.camera_type, self.device_id)
            self._cap = cv2.VideoCapture(dev)

            if not self._cap.isOpened():
                logger.error(f"无法打开摄像头: {self.camera_type.value} (设备 {dev})")
                return False

            # 设置分辨率 1280×720
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

            logger.info(f"摄像头已打开: {self.camera_type.value} (设备 {dev})")
            return True
        except ImportError:
            logger.error("缺少 opencv-python，请执行: pip install opencv-python")
            return False

    def capture(self) -> Optional[np.ndarray]:
        """
        采集一帧图像

        返回:
            BGR 格式的 numpy 数组 (720, 1280, 3)，失败返回 None
        """
        if self._cap is None or not self._cap.isOpened():
            logger.error("摄像头未打开")
            return None

        ret, frame = self._cap.read()
        if not ret:
            logger.error("图像采集失败")
            return None
        return frame

    def close(self) -> None:
        """关闭摄像头"""
        if self._cap:
            self._cap.release()
            logger.info(f"摄像头已关闭: {self.camera_type.value}")


# ============================================================
#  检测器基类
# ============================================================

class ObjectDetector(ABC):
    """物体检测器抽象基类"""

    @abstractmethod
    def detect(self, image: np.ndarray, target: str,
               confidence_threshold: float = 0.5) -> List[Detection]:
        """
        在图像中检测目标物体

        参数:
            image: BGR 图像
            target: 目标物体描述
            confidence_threshold: 置信度阈值

        返回:
            检测结果列表
        """
        ...


# ============================================================
#  检测器实现 1：YOLOv8
# ============================================================

class YOLODetector(ObjectDetector):
    """
    使用 YOLOv8 进行本地物体检测。

    支持 COCO 80 类预训练模型，也可使用自定义训练权重。
    """

    # COCO 常见物体到中文名的映射（部分）
    LABEL_ZH_MAP = {
        "cup": "杯子", "bottle": "瓶子", "bowl": "碗",
        "apple": "苹果", "banana": "香蕉", "orange": "橙子",
        "book": "书", "cell phone": "手机", "keyboard": "键盘",
        "mouse": "鼠标", "scissors": "剪刀", "toothbrush": "牙刷",
        "spoon": "勺子", "knife": "刀", "fork": "叉子",
        "chair": "椅子", "remote": "遥控器", "laptop": "笔记本电脑",
        "person": "人",
    }

    def __init__(self, model_path: str = "yolov8n.pt"):
        self.model_path = model_path
        self._model = None

    def _load_model(self) -> bool:
        """懒加载模型"""
        if self._model is not None:
            return True
        try:
            from ultralytics import YOLO
            self._model = YOLO(self.model_path)
            logger.info(f"YOLOv8 模型已加载: {self.model_path}")
            return True
        except ImportError:
            logger.error("缺少 ultralytics，请执行: pip install ultralytics")
            return False
        except Exception as e:
            logger.error(f"模型加载失败: {e}")
            return False

    def _match_target(self, label: str, target: str) -> bool:
        """
        判断检测到的标签是否匹配用户指定的目标。
        支持英文标签和中文描述的模糊匹配。
        """
        label_lower = label.lower()
        target_lower = target.lower()

        # 英文直接匹配
        if target_lower in label_lower or label_lower in target_lower:
            return True

        # 中文映射匹配
        zh_name = self.LABEL_ZH_MAP.get(label_lower, "")
        if zh_name and target in zh_name:
            return True
        # 反向：用户输入中文，检查是否对应某个英文标签
        for eng, zh in self.LABEL_ZH_MAP.items():
            if target in zh and eng == label_lower:
                return True

        return False

    def detect(self, image: np.ndarray, target: str,
               confidence_threshold: float = 0.5) -> List[Detection]:
        """YOLOv8 检测"""
        if not self._load_model():
            return []

        results = self._model(image, verbose=False)
        detections = []

        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                if conf < confidence_threshold:
                    continue

                cls_id = int(boxes.cls[i])
                label = self._model.names[cls_id]

                if not self._match_target(label, target):
                    continue

                xyxy = boxes.xyxy[i].cpu().numpy()
                det = Detection(
                    label=label,
                    confidence=conf,
                    bbox=BoundingBox(
                        x1=float(xyxy[0]), y1=float(xyxy[1]),
                        x2=float(xyxy[2]), y2=float(xyxy[3]),
                    ),
                )
                detections.append(det)

        # 按置信度降序排列
        detections.sort(key=lambda d: d.confidence, reverse=True)
        logger.info(f"YOLOv8 检测到 {len(detections)} 个 '{target}' 目标")
        return detections


# ============================================================
#  检测器实现 2：通义千问视觉大模型
# ============================================================

class QwenVLDetector(ObjectDetector):
    """
    使用通义千问视觉大模型 API 进行开放词汇物体检测。

    可以识别任意用自然语言描述的物体，不限于预定义类别。
    需要设置环境变量 DASHSCOPE_API_KEY。
    """

    def __init__(self, api_key: Optional[str] = None,
                 model: str = "qwen-vl-max"):
        self.api_key = api_key or os.environ.get("DASHSCOPE_API_KEY", "")
        self.model = model

    def detect(self, image: np.ndarray, target: str,
               confidence_threshold: float = 0.5) -> List[Detection]:
        """通过视觉大模型 API 检测目标"""
        if not self.api_key:
            logger.error(
                "未设置 API Key，请设置环境变量 DASHSCOPE_API_KEY 或在参数中传入"
            )
            return []

        try:
            import base64
            import cv2
            from openai import OpenAI

            # 图像编码为 base64
            _, buffer = cv2.imencode(".jpg", image)
            img_b64 = base64.b64encode(buffer).decode("utf-8")

            client = OpenAI(
                api_key=self.api_key,
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
            )

            prompt = (
                f"请在图像中找到「{target}」。\n"
                "如果找到了，请用 JSON 格式返回边界框坐标，格式如下：\n"
                '{"found": true, "objects": [{"label": "物体名", '
                '"bbox": [x1, y1, x2, y2], "confidence": 0.95}]}\n'
                "坐标为像素值。如果没找到，返回：\n"
                '{"found": false, "objects": []}'
            )

            response = client.chat.completions.create(
                model=self.model,
                messages=[{
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{img_b64}",
                            },
                        },
                        {"type": "text", "text": prompt},
                    ],
                }],
                max_tokens=500,
            )

            reply = response.choices[0].message.content.strip()
            # 尝试提取 JSON
            result = self._parse_json_response(reply)

            detections = []
            if result and result.get("found"):
                for obj in result.get("objects", []):
                    bbox_vals = obj.get("bbox", [0, 0, 0, 0])
                    conf = obj.get("confidence", 0.8)
                    if conf < confidence_threshold:
                        continue
                    det = Detection(
                        label=obj.get("label", target),
                        confidence=conf,
                        bbox=BoundingBox(
                            x1=float(bbox_vals[0]),
                            y1=float(bbox_vals[1]),
                            x2=float(bbox_vals[2]),
                            y2=float(bbox_vals[3]),
                        ),
                    )
                    detections.append(det)

            logger.info(f"千问视觉检测到 {len(detections)} 个 '{target}' 目标")
            return detections

        except ImportError:
            logger.error("缺少 openai 库，请执行: pip install openai")
            return []
        except Exception as e:
            logger.error(f"千问视觉 API 调用失败: {e}")
            return []

    @staticmethod
    def _parse_json_response(text: str) -> Optional[dict]:
        """从大模型回复中提取 JSON"""
        import re
        # 尝试直接解析
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass
        # 尝试从 ``` 代码块中提取
        match = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
        if match:
            try:
                return json.loads(match.group(1))
            except json.JSONDecodeError:
                pass
        # 尝试找到第一个 { ... }
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if match:
            try:
                return json.loads(match.group(0))
            except json.JSONDecodeError:
                pass
        return None


# ============================================================
#  检测器实现 3：OpenCV 颜色检测（轻量回退）
# ============================================================

class ColorDetector(ObjectDetector):
    """
    基于 HSV 颜色空间的简单物体检测。

    不需要任何模型或网络，适用于颜色特征明显的物体。
    支持的颜色关键词：红、蓝、绿、黄、橙、白、黑
    """

    # 颜色名 → HSV 范围 (H: 0-179, S: 0-255, V: 0-255)
    COLOR_RANGES = {
        "红": [
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([170, 100, 100]), np.array([179, 255, 255])),
        ],
        "red": [
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([170, 100, 100]), np.array([179, 255, 255])),
        ],
        "蓝": [(np.array([100, 100, 100]), np.array([130, 255, 255]))],
        "blue": [(np.array([100, 100, 100]), np.array([130, 255, 255]))],
        "绿": [(np.array([35, 100, 100]), np.array([85, 255, 255]))],
        "green": [(np.array([35, 100, 100]), np.array([85, 255, 255]))],
        "黄": [(np.array([20, 100, 100]), np.array([35, 255, 255]))],
        "yellow": [(np.array([20, 100, 100]), np.array([35, 255, 255]))],
        "橙": [(np.array([10, 100, 100]), np.array([20, 255, 255]))],
        "orange": [(np.array([10, 100, 100]), np.array([20, 255, 255]))],
    }

    def _extract_color_keyword(self, target: str) -> Optional[str]:
        """从目标描述中提取颜色关键词"""
        for color_name in self.COLOR_RANGES:
            if color_name in target.lower():
                return color_name
        return None

    def detect(self, image: np.ndarray, target: str,
               confidence_threshold: float = 0.5) -> List[Detection]:
        """基于颜色的物体检测"""
        try:
            import cv2
        except ImportError:
            logger.error("缺少 opencv-python")
            return []

        color_key = self._extract_color_keyword(target)
        if not color_key:
            logger.warning(f"无法从 '{target}' 中提取颜色关键词，"
                           f"支持: {list(self.COLOR_RANGES.keys())}")
            return []

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        ranges = self.COLOR_RANGES[color_key]

        # 合并多个颜色范围的掩码
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        img_area = image.shape[0] * image.shape[1]

        for contour in contours:
            area = cv2.contourArea(contour)
            # 过滤过小的区域 (< 0.1% 图像面积)
            if area < img_area * 0.001:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            # 用面积占比作为"置信度"的近似
            confidence = min(1.0, area / (img_area * 0.05))
            if confidence < confidence_threshold:
                continue

            det = Detection(
                label=f"{color_key}_object",
                confidence=confidence,
                bbox=BoundingBox(
                    x1=float(x), y1=float(y),
                    x2=float(x + w), y2=float(y + h),
                ),
            )
            detections.append(det)

        detections.sort(key=lambda d: d.bbox.area, reverse=True)
        logger.info(f"颜色检测到 {len(detections)} 个 '{color_key}' 目标")
        return detections


# ============================================================
#  坐标变换与抓取规划
# ============================================================

class GraspPlanner:
    """
    抓取规划器

    将 2D 检测结果转换为 3D 抓取位姿，
    并通过逆运动学计算关节角度。
    """

    def __init__(
        self,
        camera_type: CameraType = CameraType.HIGH,
        intrinsics: Optional[CameraIntrinsics] = None,
        extrinsics: Optional[np.ndarray] = None,
        default_depth: float = DEFAULT_DEPTH_M,
        grasp_height: float = 0.05,
    ):
        self.camera_type = camera_type
        self.intrinsics = intrinsics or DEFAULT_INTRINSICS[camera_type]
        self.extrinsics = extrinsics if extrinsics is not None else DEFAULT_EXTRINSICS[camera_type]
        self.default_depth = default_depth
        self.grasp_height = grasp_height

    def pixel_to_camera(self, u: float, v: float,
                        depth: Optional[float] = None) -> np.ndarray:
        """
        像素坐标 → 相机坐标系 3D 点

        参数:
            u, v: 像素坐标
            depth: 深度值 (米)，None 时使用默认值

        返回:
            [X, Y, Z] 相机坐标系下的 3D 点
        """
        d = depth if depth is not None else self.default_depth
        K = self.intrinsics
        X = (u - K.cx) * d / K.fx
        Y = (v - K.cy) * d / K.fy
        Z = d
        return np.array([X, Y, Z, 1.0])

    def camera_to_base(self, point_cam: np.ndarray) -> np.ndarray:
        """
        相机坐标系 → 机械臂基座标系

        参数:
            point_cam: [X, Y, Z, 1] 齐次坐标

        返回:
            [X, Y, Z] 基座标系下的 3D 点
        """
        point_base = self.extrinsics @ point_cam
        return point_base[:3]

    def plan_grasp(self, detection: Detection,
                   depth: Optional[float] = None) -> GraspPlan:
        """
        根据检测结果规划抓取

        参数:
            detection: 检测结果
            depth: 物体深度 (米)

        返回:
            GraspPlan 抓取规划
        """
        u, v = detection.bbox.center

        # 像素 → 相机 → 基座标
        point_cam = self.pixel_to_camera(u, v, depth)
        point_base = self.camera_to_base(point_cam)

        # 物体位姿（朝下抓取，默认 pitch=π 即末端执行器朝下）
        object_pose = Pose3D(
            x=point_base[0],
            y=point_base[1],
            z=point_base[2],
        )

        # 预抓取位姿（物体上方 grasp_height 处）
        pre_grasp_pose = Pose3D(
            x=point_base[0],
            y=point_base[1],
            z=point_base[2] + self.grasp_height,
            pitch=math.pi,  # 末端朝下
        )

        # 抓取位姿
        grasp_pose = Pose3D(
            x=point_base[0],
            y=point_base[1],
            z=point_base[2],
            pitch=math.pi,
        )

        # 逆运动学计算关节角度
        joint_angles_pre = self._inverse_kinematics(pre_grasp_pose)
        joint_angles_grasp = self._inverse_kinematics(grasp_pose)

        plan = GraspPlan(
            target_label=detection.label,
            detection=detection,
            object_pose=object_pose,
            pre_grasp_pose=pre_grasp_pose,
            grasp_pose=grasp_pose,
            joint_angles_pre=joint_angles_pre,
            joint_angles_grasp=joint_angles_grasp,
        )

        logger.info(
            f"抓取规划完成: 目标={detection.label}, "
            f"位置=({object_pose.x:.3f}, {object_pose.y:.3f}, {object_pose.z:.3f})m"
        )
        return plan

    def _inverse_kinematics(self, pose: Pose3D) -> Optional[List[float]]:
        """
        逆运动学求解（简化几何解法）

        输出格式：SO100 位置值（-100 ~ 100），与 arm-control 技能
        的 send_action 接口直接兼容。

        注意：这里提供一个简化的几何解法框架。
        实际使用中应替换为基于精确 DH 参数的求解器。

        返回:
            5 个关节位置值 [shoulder_pan, shoulder_lift, elbow_flex,
            wrist_flex, wrist_roll]，无解返回 None
            （gripper 由抓取流程单独控制，不含在 IK 输出中）
        """
        # SO100 关节限位（与 arm-control JOINT_REGISTRY 一致）
        # 格式: (name, min_pos, max_pos)
        JOINT_LIMITS = [
            ("shoulder_pan",  -100.0, 100.0),
            ("shoulder_lift", -100.0, 100.0),
            ("elbow_flex",    -100.0, 100.0),
            ("wrist_flex",    -100.0, 100.0),
            ("wrist_roll",    -100.0, 100.0),
        ]

        try:
            target = pose.to_array()[:3]  # [x, y, z] 米

            # --- 简化 5-DOF IK（SO100 为 5 旋转 + 1 夹爪）---
            # 将空间坐标映射到 SO100 的 -100~100 位置范围
            # 实际项目中需替换为精确 IK

            # 近似工作空间半径 (米)
            WORKSPACE_RADIUS = 0.3

            # 1. 底座旋转: atan2 映射到 -100~100
            angle_base = math.atan2(target[1], target[0])
            shoulder_pan = (angle_base / math.pi) * 100.0

            # 2. 大臂升降: 由高度决定
            height_ratio = target[2] / WORKSPACE_RADIUS
            shoulder_lift = max(-1.0, min(1.0, height_ratio)) * 100.0

            # 3. 小臂弯曲: 由水平距离决定
            reach = math.sqrt(target[0] ** 2 + target[1] ** 2)
            reach_ratio = reach / WORKSPACE_RADIUS
            elbow_flex = -max(0.0, min(1.0, reach_ratio)) * 100.0

            # 4. 手腕俯仰: 由末端俯仰目标决定
            wrist_flex = 0.0
            if pose.pitch != 0:
                wrist_flex = (pose.pitch / math.pi) * 100.0

            # 5. 手腕旋转: 由末端偏航目标决定
            wrist_roll = 0.0
            if pose.yaw != 0:
                wrist_roll = (pose.yaw / math.pi) * 100.0

            joints = [shoulder_pan, shoulder_lift, elbow_flex,
                      wrist_flex, wrist_roll]

            # ---- 逐关节限位校验 ----
            for i, (name, lo, hi) in enumerate(JOINT_LIMITS):
                if joints[i] < lo or joints[i] > hi:
                    logger.warning(
                        f"IK 关节 {name} 超限: {joints[i]:.2f} "
                        f"不在 [{lo}, {hi}] 范围内，已裁剪"
                    )
                    joints[i] = max(lo, min(hi, joints[i]))

            logger.debug(
                f"IK 求解结果 (SO100 位置值): {[round(j, 2) for j in joints]}"
            )
            return joints

        except (ValueError, ZeroDivisionError) as e:
            logger.warning(f"逆运动学求解失败: {e}")
            return None


# ============================================================
#  可视化
# ============================================================

def visualize_detections(image: np.ndarray, detections: List[Detection],
                         window_name: str = "vision-grasp") -> np.ndarray:
    """
    在图像上绘制检测结果

    返回标注后的图像副本
    """
    try:
        import cv2
    except ImportError:
        return image

    vis = image.copy()
    for det in detections:
        x1, y1 = int(det.bbox.x1), int(det.bbox.y1)
        x2, y2 = int(det.bbox.x2), int(det.bbox.y2)
        cx, cy = int(det.bbox.center[0]), int(det.bbox.center[1])

        # 绘制边界框
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 绘制中心点
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        # 标签
        label_text = f"{det.label} {det.confidence:.2f}"
        cv2.putText(vis, label_text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return vis


# ============================================================
#  核心流程：视觉定位抓取
# ============================================================

class VisionGraspPipeline:
    """
    视觉定位抓取完整流程

    串联：图像采集 → 目标检测 → 坐标转换 → 抓取规划 → 调用 arm-control 执行
    """

    def __init__(
        self,
        camera_type: CameraType = CameraType.HIGH,
        detector_type: DetectorType = DetectorType.YOLO,
        arm: str = "left",
        device_id: int = 0,
        confidence: float = 0.5,
        grasp_height: float = 0.05,
        show_preview: bool = False,
        yolo_model: str = "yolov8n.pt",
        qwen_api_key: Optional[str] = None,
    ):
        self.camera_type = camera_type
        self.detector_type = detector_type
        self.arm = arm
        self.confidence = confidence
        self.show_preview = show_preview

        # 初始化组件
        self.camera = CameraCapture(camera_type, device_id)
        self.planner = GraspPlanner(
            camera_type=camera_type,
            grasp_height=grasp_height,
        )

        # 初始化检测器
        if detector_type == DetectorType.YOLO:
            self.detector: ObjectDetector = YOLODetector(model_path=yolo_model)
        elif detector_type == DetectorType.QWEN_VL:
            self.detector = QwenVLDetector(api_key=qwen_api_key)
        elif detector_type == DetectorType.COLOR:
            self.detector = ColorDetector()
        else:
            raise ValueError(f"不支持的检测后端: {detector_type}")

    def detect_only(self, target: str, max_retries: int = 3) -> Optional[List[Detection]]:
        """
        仅执行检测，不抓取

        参数:
            target: 目标物体描述
            max_retries: 最大重试次数

        返回:
            检测结果列表，失败返回 None
        """
        if not self.camera.open():
            return None

        try:
            for attempt in range(max_retries):
                logger.info(f"检测尝试 {attempt + 1}/{max_retries}: 目标='{target}'")

                frame = self.camera.capture()
                if frame is None:
                    time.sleep(0.5)
                    continue

                detections = self.detector.detect(
                    frame, target, self.confidence
                )

                if detections:
                    if self.show_preview:
                        self._show_preview(frame, detections)
                    return detections

                logger.warning(f"第 {attempt + 1} 次检测未找到 '{target}'")
                time.sleep(1.0)

            logger.error(f"经过 {max_retries} 次尝试，未检测到 '{target}'")
            return None
        finally:
            self.camera.close()

    def run(self, target: str, auto_execute: bool = True,
            arm_port: str = "COM7") -> Optional[GraspPlan]:
        """
        执行完整的视觉定位抓取流程

        参数:
            target: 目标物体描述，如 "水杯"、"红色方块"
            auto_execute: 检测到后是否自动执行抓取
            arm_port: SO100 机械臂串口端口 (默认 COM7)

        返回:
            GraspPlan 抓取规划结果
        """
        logger.info(f"{'=' * 50}")
        logger.info(f"开始视觉定位抓取: 目标='{target}'")
        logger.info(f"摄像头={self.camera_type.value}, 检测器={self.detector_type.value}")
        logger.info(f"{'=' * 50}")

        # ---- 步骤 1: 图像采集与检测 ----
        detections = self.detect_only(target)
        if not detections:
            return None

        best = detections[0]
        logger.info(
            f"选择最佳检测结果: {best.label} "
            f"(置信度 {best.confidence:.2f}, "
            f"中心 {best.bbox.center})"
        )

        # ---- 步骤 2: 抓取规划 ----
        plan = self.planner.plan_grasp(best)

        if plan.joint_angles_grasp is None:
            logger.error("逆运动学求解失败，目标可能在工作空间外")
            return plan

        logger.info(f"物体位置 (基座标系): {plan.object_pose.to_dict()}")
        logger.info(f"预抓取关节角度: {plan.joint_angles_pre}")
        logger.info(f"抓取关节角度: {plan.joint_angles_grasp}")

        # ---- 步骤 3: 执行抓取 ----
        if auto_execute:
            self._execute_grasp(plan, arm_port)

        return plan

    def _execute_grasp(self, plan: GraspPlan, arm_port: str) -> bool:
        """
        调用 arm-control 的 SO100Controller 执行抓取动作

        流程:
          1. 打开夹爪
          2. 移动到预抓取位置
          3. 下降到抓取位置
          4. 关闭夹爪
          5. 抬起
        """
        logger.info("开始执行抓取动作...")

        try:
            # 动态导入 arm-control 技能
            arm_control_path = Path(__file__).parent.parent.parent / "arm-control" / "scripts"
            sys.path.insert(0, str(arm_control_path))
            from main import create_controller

            # 创建并连接控制器
            ctrl = create_controller(port=arm_port)
            if not ctrl.connect():
                logger.error("SO100 机械臂连接失败")
                return False

            try:
                # IK 输出是 5 个关节值 [shoulder_pan, shoulder_lift,
                # elbow_flex, wrist_flex, wrist_roll]
                joint_names = [
                    "shoulder_pan", "shoulder_lift", "elbow_flex",
                    "wrist_flex", "wrist_roll",
                ]

                def _joints_to_action(angles: List[float]) -> Dict[str, float]:
                    """将 IK 关节列表转为 SO100 动作字典"""
                    action = {}
                    for i, name in enumerate(joint_names):
                        if i < len(angles):
                            action[f"{name}.pos"] = angles[i]
                    return action

                # 步骤 1: 打开夹爪
                logger.info("[1/5] 打开夹爪")
                ctrl.open_gripper(80.0)
                time.sleep(1.0)

                # 步骤 2: 移动到预抓取位置
                if plan.joint_angles_pre:
                    logger.info("[2/5] 移动到预抓取位置")
                    action = _joints_to_action(plan.joint_angles_pre)
                    action["gripper.pos"] = 80.0
                    ctrl.move_to(action, wait=2.0)

                # 步骤 3: 下降到抓取位置
                if plan.joint_angles_grasp:
                    logger.info("[3/5] 下降到抓取位置")
                    action = _joints_to_action(plan.joint_angles_grasp)
                    action["gripper.pos"] = 80.0
                    ctrl.move_to(action, wait=1.5)

                # 步骤 4: 关闭夹爪
                logger.info("[4/5] 关闭夹爪")
                ctrl.close_gripper(5.0)
                time.sleep(1.0)

                # 步骤 5: 抬起（回到预抓取位置）
                if plan.joint_angles_pre:
                    logger.info("[5/5] 抬起物体")
                    action = _joints_to_action(plan.joint_angles_pre)
                    action["gripper.pos"] = 5.0  # 保持夹紧
                    ctrl.move_to(action, wait=2.0)

                logger.info("抓取执行完成!")
                return True

            except KeyboardInterrupt:
                logger.warning("用户中断，紧急停止")
                ctrl.emergency_stop()
                return False
            finally:
                ctrl.disconnect()

        except ImportError as e:
            logger.error(f"无法导入 arm-control 技能: {e}")
            logger.info("抓取规划已完成，但无法自动执行。"
                        "请手动调用 arm-control 传入关节值。")
            return False

    def _show_preview(self, image: np.ndarray,
                      detections: List[Detection]) -> None:
        """显示检测结果预览"""
        try:
            import cv2
            vis = visualize_detections(image, detections)
            cv2.imshow("vision-grasp 检测结果", vis)
            cv2.waitKey(3000)
            cv2.destroyAllWindows()
        except Exception:
            pass


# ============================================================
#  命令行入口
# ============================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OpenClaw 视觉定位抓取技能",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # YOLOv8 检测 + 自动抓取
  python main.py --target 水杯 --detector yolo --auto-execute

  # 千问视觉 API 检测（开放词汇）
  python main.py --target "桌上的红色马克杯" --detector qwen_vl

  # 仅检测不抓取
  python main.py --target 瓶子 --no-auto-execute --show-preview

  # 颜色检测
  python main.py --target "红色物体" --detector color

  # 指定摄像头
  python main.py --target 杯子 --camera high

  # 指定机械臂串口
  python main.py --target 杯子 --arm-port COM7 --auto-execute
        """,
    )

    parser.add_argument(
        "--target", type=str, required=True,
        help="目标物体描述，如 '水杯'、'红色方块'、'最近的瓶子'",
    )
    parser.add_argument(
        "--camera", type=str, default="high",
        choices=["high", "left_wrist", "right_wrist"],
        help="使用的摄像头 (默认 high)",
    )
    parser.add_argument(
        "--detector", type=str, default="yolo",
        choices=["yolo", "qwen_vl", "color"],
        help="检测后端 (默认 yolo)",
    )
    parser.add_argument(
        "--arm", type=str, default="default",
        help="机械臂标识 (SO100 单臂，默认 default)",
    )
    parser.add_argument(
        "--confidence", type=float, default=0.5,
        help="检测置信度阈值 (默认 0.5)",
    )
    parser.add_argument(
        "--grasp-height", type=float, default=0.05,
        help="抓取接近高度，米 (默认 0.05)",
    )
    parser.add_argument(
        "--auto-execute", action="store_true", default=True,
        help="检测到后自动执行抓取 (默认开启)",
    )
    parser.add_argument(
        "--no-auto-execute", dest="auto_execute", action="store_false",
        help="仅检测不抓取",
    )
    parser.add_argument(
        "--show-preview", action="store_true", default=False,
        help="显示检测可视化窗口",
    )
    parser.add_argument(
        "--device-id", type=int, default=0,
        help="摄像头设备 ID (默认 0)",
    )
    parser.add_argument(
        "--yolo-model", type=str, default="yolov8n.pt",
        help="YOLOv8 模型路径 (默认 yolov8n.pt)",
    )
    parser.add_argument(
        "--qwen-api-key", type=str, default=None,
        help="通义千问 API Key (也可通过 DASHSCOPE_API_KEY 环境变量设置)",
    )
    parser.add_argument(
        "--arm-port", type=str, default="COM7",
        help="SO100 机械臂串口端口 (默认 COM7)",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    camera_type = CameraType(args.camera)
    detector_type = DetectorType(args.detector)

    pipeline = VisionGraspPipeline(
        camera_type=camera_type,
        detector_type=detector_type,
        arm=args.arm,
        device_id=args.device_id,
        confidence=args.confidence,
        grasp_height=args.grasp_height,
        show_preview=args.show_preview,
        yolo_model=args.yolo_model,
        qwen_api_key=args.qwen_api_key,
    )

    if args.auto_execute:
        plan = pipeline.run(
            target=args.target,
            auto_execute=True,
            arm_port=args.arm_port,
        )
    else:
        detections = pipeline.detect_only(args.target)
        if detections:
            print("\n检测结果:")
            for i, det in enumerate(detections):
                print(f"  [{i + 1}] {json.dumps(det.to_dict(), ensure_ascii=False)}")

            # 对最佳结果做抓取规划（但不执行）
            plan = pipeline.planner.plan_grasp(detections[0])
            print("\n抓取规划:")
            print(f"  物体位姿: {json.dumps(plan.object_pose.to_dict(), ensure_ascii=False)}")
            print(f"  预抓取关节: {plan.joint_angles_pre}")
            print(f"  抓取关节:   {plan.joint_angles_grasp}")
        else:
            plan = None

    if plan:
        logger.info("任务完成")
    else:
        logger.error("任务失败")
        sys.exit(1)


if __name__ == "__main__":
    main()
