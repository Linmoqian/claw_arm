#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂视觉检测器
使用YOLOv8或通义千问视觉API进行物体检测,并提供多格式坐标输出
"""

import sys
import os
import json
import math
import time
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional, Union
from dataclasses import dataclass

# 添加项目根目录到路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, project_root)


@dataclass
class DetectionResult:
    """检测结果数据类"""
    class_name: str
    confidence: float
    pixel: Tuple[int, int]
    bounding_box: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    arm_3d: Optional[Tuple[float, float, float]] = None  # (x, y, z) in mm
    polar: Optional[Tuple[float, float]] = None  # (angle_degrees, distance_mm)


class VisionDetector:
    """视觉检测器 - 支持多种检测后端和坐标输出格式"""

    def __init__(
        self,
        camera_id: int = 0,
        detector: str = "yolo",
        output_format: str = "all",
        calibration_file: Optional[str] = None,
        model_path: Optional[str] = None
    ):
        """
        初始化视觉检测器

        Args:
            camera_id: 摄像头设备ID
            detector: 检测后端类型 ("yolo", "qwen_vl", "color")
            output_format: 输出格式 ("pixel", "3d", "polar", "all")
            calibration_file: 标定数据文件路径
            model_path: YOLO模型路径(默认使用yolov8n.pt)
        """
        self.camera_id = camera_id
        self.detector_type = detector
        self.output_format = output_format
        self.calibration_file = calibration_file or "skills/vision-control/calibration_data.json"

        # 摄像头
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center = (self.frame_width // 2, self.frame_height // 2)

        # 检测模型
        self.model = None
        self.model_path = model_path or "yolov8n.pt"

        # 标定数据
        self.calibration_data = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.hand_eye_matrix = None

        # 通义千问API配置
        self.qwen_client = None
        self.qwen_api_key = None

    def load_model(self):
        """加载检测模型"""
        if self.detector_type == "yolo":
            self._load_yolo_model()
        elif self.detector_type == "qwen_vl":
            self._load_qwen_model()
        elif self.detector_type == "color":
            print("[VisionDetector] 使用颜色检测模式,无需加载模型")
        else:
            raise ValueError(f"不支持的检测器类型: {self.detector_type}")

        # 加载标定数据
        self._load_calibration()

    def _load_yolo_model(self):
        """加载YOLOv8模型"""
        try:
            from ultralytics import YOLO
            print(f"[VisionDetector] 加载YOLOv8模型: {self.model_path}")
            self.model = YOLO(self.model_path)
            print("[VisionDetector] YOLOv8模型加载成功")
        except ImportError:
            print("[VisionDetector] 错误: 未安装ultralytics库")
            print("请运行: pip install ultralytics")
            raise
        except Exception as e:
            print(f"[VisionDetector] 加载YOLO模型失败: {e}")
            raise

    def _load_qwen_model(self):
        """加载通义千问视觉API"""
        try:
            from openai import OpenAI

            # 从环境变量或配置文件读取API密钥
            api_key = os.getenv("DASHSCOPE_API_KEY")
            if not api_key:
                # 尝试从项目配置读取
                config_file = os.path.join(project_root, "config", "api_keys.json")
                if os.path.exists(config_file):
                    with open(config_file, 'r') as f:
                        config = json.load(f)
                        api_key = config.get("dashscope_api_key")

            if not api_key:
                raise ValueError("未找到通义千问API密钥,请设置环境变量DASHSCOPE_API_KEY")

            self.qwen_client = OpenAI(
                api_key=api_key,
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
            )
            print("[VisionDetector] 通义千问视觉API已连接")
        except ImportError:
            print("[VisionDetector] 错误: 未安装openai库")
            print("请运行: pip install openai")
            raise

    def _load_calibration(self):
        """加载标定数据"""
        if os.path.exists(self.calibration_file):
            with open(self.calibration_file, 'r', encoding='utf-8') as f:
                self.calibration_data = json.load(f)

            # 加载相机内参
            if "camera_matrix" in self.calibration_data:
                self.camera_matrix = np.array(self.calibration_data["camera_matrix"])
                self.dist_coeffs = np.array(self.calibration_data.get("dist_coeffs", [0, 0, 0, 0, 0]))

            # 加载手眼标定矩阵
            if "hand_eye_matrix" in self.calibration_data:
                self.hand_eye_matrix = np.array(self.calibration_data["hand_eye_matrix"])

            print(f"[VisionDetector] 已加载标定数据: {self.calibration_file}")
        else:
            print(f"[VisionDetector] 标定文件不存在: {self.calibration_file}")
            print("警告: 坐标转换精度可能不准确,建议运行标定程序")

    def open_camera(self):
        """打开摄像头"""
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)

            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 {self.camera_id}")

            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

            # 更新实际分辨率
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.frame_center = (self.frame_width // 2, self.frame_height // 2)

            print(f"[VisionDetector] 摄像头已打开 (分辨率: {self.frame_width}x{self.frame_height})")

    def capture_frame(self) -> np.ndarray:
        """捕获一帧图像"""
        if self.cap is None:
            self.open_camera()

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("无法从摄像头读取图像")

        return frame

    def detect_objects(
        self,
        target: str,
        confidence: float = 0.5,
        max_results: int = 10,
        color_lower: Optional[Tuple[int, int, int]] = None,
        color_upper: Optional[Tuple[int, int, int]] = None
    ) -> List[DetectionResult]:
        """
        检测物体

        Args:
            target: 目标描述(类别名称或颜色)
            confidence: 置信度阈值
            max_results: 最大结果数量
            color_lower: HSV颜色下限(仅用于color检测器)
            color_upper: HSV颜色上限(仅用于color检测器)

        Returns:
            检测结果列表
        """
        if self.detector_type == "yolo":
            return self._detect_yolo(target, confidence, max_results)
        elif self.detector_type == "qwen_vl":
            return self._detect_qwen_vl(target, confidence, max_results)
        elif self.detector_type == "color":
            if color_lower is None or color_upper is None:
                raise ValueError("颜色检测需要提供color_lower和color_upper参数")
            return self._detect_color(color_lower, color_upper, max_results)
        else:
            raise ValueError(f"不支持的检测器类型: {self.detector_type}")

    def _detect_yolo(
        self,
        target: str,
        confidence: float,
        max_results: int
    ) -> List[DetectionResult]:
        """使用YOLOv8检测物体"""
        frame = self.capture_frame()

        # YOLO推理
        results = self.model(frame, conf=confidence, verbose=False)

        detection_results = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                # 获取类别和置信度
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                conf = float(box.conf[0])

                # 简单的类别匹配(支持中英文)
                if target.lower() not in class_name.lower() and class_name.lower() not in target.lower():
                    continue

                # 获取边界框
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # 计算中心点像素坐标
                pixel_x = (x1 + x2) // 2
                pixel_y = (y1 + y2) // 2

                # 创建检测结果
                detection = DetectionResult(
                    class_name=class_name,
                    confidence=conf,
                    pixel=(pixel_x, pixel_y),
                    bounding_box=(x1, y1, x2, y2)
                )

                # 计算额外坐标
                if self.output_format in ["3d", "all"]:
                    detection.arm_3d = self._pixel_to_arm_3d(pixel_x, pixel_y, (x2-x1, y2-y1))

                if self.output_format in ["polar", "all"]:
                    detection.polar = self._pixel_to_polar(pixel_x, pixel_y)

                detection_results.append(detection)

                if len(detection_results) >= max_results:
                    break

            if len(detection_results) >= max_results:
                break

        # 按置信度排序
        detection_results.sort(key=lambda x: x.confidence, reverse=True)

        return detection_results

    def _detect_qwen_vl(
        self,
        target: str,
        confidence: float,
        max_results: int
    ) -> List[DetectionResult]:
        """使用通义千问视觉API检测物体"""
        frame = self.capture_frame()

        # 保存图像到临时文件
        temp_image_path = "/tmp/detection_image.jpg"
        cv2.imwrite(temp_image_path, frame)

        # 读取图像并编码为base64
        import base64
        with open(temp_image_path, 'rb') as f:
            image_data = base64.b64encode(f.read()).decode('utf-8')

        # 构建提示词
        prompt = f"""请检测图像中的"{target}"物体。
返回JSON格式:
{{
    "objects": [
        {{
            "class": "物体类别",
            "confidence": 0.95,
            "bbox": [x1, y1, x2, y2]
        }}
    ]
}}
只返回JSON,不要其他内容。"""

        try:
            # 调用API
            response = self.qwen_client.chat.completions.create(
                model="qwen-vl-plus",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_data}"}}
                        ]
                    }
                ]
            )

            # 解析响应
            result_text = response.choices[0].message.content
            # 清理可能的markdown标记
            if "```json" in result_text:
                result_text = result_text.split("```json")[1].split("```")[0].strip()
            elif "```" in result_text:
                result_text = result_text.split("```")[1].split("```")[0].strip()

            result_data = json.loads(result_text)
            detection_results = []

            for obj in result_data.get("objects", []):
                x1, y1, x2, y2 = obj["bbox"]
                pixel_x = (x1 + x2) // 2
                pixel_y = (y1 + y2) // 2
                conf = obj.get("confidence", 1.0)

                if conf < confidence:
                    continue

                detection = DetectionResult(
                    class_name=obj.get("class", target),
                    confidence=conf,
                    pixel=(pixel_x, pixel_y),
                    bounding_box=(x1, y1, x2, y2)
                )

                if self.output_format in ["3d", "all"]:
                    detection.arm_3d = self._pixel_to_arm_3d(pixel_x, pixel_y, (x2-x1, y2-y1))

                if self.output_format in ["polar", "all"]:
                    detection.polar = self._pixel_to_polar(pixel_x, pixel_y)

                detection_results.append(detection)

                if len(detection_results) >= max_results:
                    break

            return detection_results

        except Exception as e:
            print(f"[VisionDetector] 通义千问API调用失败: {e}")
            return []

    def _detect_color(
        self,
        color_lower: Tuple[int, int, int],
        color_upper: Tuple[int, int, int],
        max_results: int,
        min_area: int = 500
    ) -> List[DetectionResult]:
        """使用颜色检测物体"""
        frame = self.capture_frame()

        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 创建颜色掩码
        mask = cv2.inRange(hsv, np.array(color_lower), np.array(color_upper))

        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detection_results = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            # 计算轮廓中心
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 边界框
            x, y, w, h = cv2.boundingRect(contour)

            detection = DetectionResult(
                class_name="color_object",
                confidence=1.0,  # 颜色检测不提供置信度
                pixel=(cx, cy),
                bounding_box=(x, y, x+w, y+h)
            )

            if self.output_format in ["3d", "all"]:
                detection.arm_3d = self._pixel_to_arm_3d(cx, cy, (w, h))

            if self.output_format in ["polar", "all"]:
                detection.polar = self._pixel_to_polar(cx, cy)

            detection_results.append(detection)

            if len(detection_results) >= max_results:
                break

        # 按面积排序
        detection_results.sort(key=lambda x: x.bounding_box[2] * x.bounding_box[3], reverse=True)

        return detection_results

    def _pixel_to_arm_3d(
        self,
        pixel_x: int,
        pixel_y: int,
        bbox_size: Tuple[int, int]
    ) -> Optional[Tuple[float, float, float]]:
        """
        将像素坐标转换为机械臂3D坐标

        Args:
            pixel_x: 像素X坐标
            pixel_y: 像素Y坐标
            bbox_size: 边界框尺寸 (width, height)

        Returns:
            (x, y, z) 机械臂坐标(mm)
        """
        if self.calibration_data is None:
            print("[VisionDetector] 警告: 未加载标定数据,使用默认转换参数")
            # 使用简化的默认转换
            norm_x = (pixel_x - self.frame_center[0]) / self.frame_center[0]
            norm_y = (pixel_y - self.frame_center[1]) / self.frame_center[1]

            # 简化的映射(假设相机高度300mm)
            arm_x = 200 + norm_x * 100
            arm_y = norm_y * 100
            arm_z = 50.0  # 默认抓取高度

            return (arm_x, arm_y, arm_z)

        # 使用标定数据进行转换
        # 1. 像素坐标归一化
        norm_x = (pixel_x - self.frame_center[0]) / self.frame_center[0]
        norm_y = (pixel_y - self.frame_center[1]) / self.frame_center[1]

        # 2. 使用标定参数计算相机坐标
        scale_x = self.calibration_data.get("scale_x", 100.0)
        scale_y = self.calibration_data.get("scale_y", 100.0)
        offset_x = self.calibration_data.get("offset_x", 200.0)
        offset_y = self.calibration_data.get("offset_y", 0.0)

        cam_x = offset_x + norm_x * scale_x
        cam_y = offset_y + norm_y * scale_y

        # 3. 深度估计(基于物体尺寸)
        # 假设物体实际高度约为100mm
        object_height_mm = 100.0
        bbox_height = bbox_size[1]
        focal_length = self.calibration_data.get("focal_length", 500.0)

        # Z = (f * h) / H_pixel
        depth = (focal_length * object_height_mm) / bbox_height

        # 4. 应用手眼标定矩阵转换到机械臂坐标系
        if self.hand_eye_matrix is not None:
            # 简化的手眼转换(仅平移)
            tx = self.hand_eye_matrix[0, 3]
            ty = self.hand_eye_matrix[1, 3]
            tz = self.hand_eye_matrix[2, 3]

            arm_x = cam_x + tx
            arm_y = cam_y + ty
            arm_z = depth + tz
        else:
            # 无手眼标定,直接使用相机坐标
            arm_x = cam_x
            arm_y = cam_y
            arm_z = depth

        return (arm_x, arm_y, arm_z)

    def _pixel_to_polar(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """
        将像素坐标转换为极坐标(相对于图像中心)

        Args:
            pixel_x: 像素X坐标
            pixel_y: 像素Y坐标

        Returns:
            (angle_degrees, distance_pixels)
        """
        # 计算相对于中心的偏移
        dx = pixel_x - self.frame_center[0]
        dy = pixel_y - self.frame_center[1]

        # 计算距离
        distance = math.sqrt(dx**2 + dy**2)

        # 计算角度(相对于正下方,顺时针为正)
        angle_rad = math.atan2(dx, dy)  # 注意: atan2(x, y)使0度指向下方
        angle_deg = math.degrees(angle_rad)

        return (angle_deg, distance)

    def get_best_match(self, results: List[DetectionResult]) -> Optional[DetectionResult]:
        """获取最佳匹配结果(置信度最高的)"""
        if not results:
            return None
        return max(results, key=lambda x: x.confidence)

    def get_polar_coord(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """获取像素坐标的极坐标表示"""
        return self._pixel_to_polar(pixel_x, pixel_y)

    def filter_by_class(
        self,
        results: List[DetectionResult],
        class_name: str
    ) -> List[DetectionResult]:
        """按类别过滤结果"""
        return [r for r in results if r.class_name.lower() == class_name.lower()]

    def sort_by_distance(self, results: List[DetectionResult]) -> List[DetectionResult]:
        """按距离排序(从近到远)"""
        return sorted(results, key=lambda x: x.polar[1] if x.polar else float('inf'))

    def show_results(
        self,
        results: List[DetectionResult],
        annotate: bool = True,
        wait_key: bool = True
    ):
        """可视化检测结果"""
        frame = self.capture_frame()
        frame_copy = frame.copy()

        for i, result in enumerate(results):
            x1, y1, x2, y2 = result.bounding_box

            # 绘制边界框
            color = (0, 255, 0)  # 绿色
            cv2.rectangle(frame_copy, (x1, y1), (x2, y2), color, 2)

            if annotate:
                # 标签文本
                label = f"{result.class_name}: {result.confidence:.2f}"

                # 如果有3D坐标,显示
                if result.arm_3d:
                    label += f"\n3D: ({result.arm_3d[0]:.0f}, {result.arm_3d[1]:.0f}, {result.arm_3d[2]:.0f})"

                # 如果有极坐标,显示
                if result.polar:
                    label += f"\nPolar: {result.polar[0]:.1f}°, {result.polar[1]:.0f}px"

                # 绘制标签背景
                y_text = y1 - 10 if y1 > 30 else y1 + 10
                lines = label.split('\n')
                for j, line in enumerate(lines):
                    cv2.putText(frame_copy, line, (x1, y_text - j * 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 绘制中心点
            pixel_x, pixel_y = result.pixel
            cv2.circle(frame_copy, (pixel_x, pixel_y), 5, (0, 0, 255), -1)

        # 显示图像中心十字线
        cx, cy = self.frame_center
        cv2.line(frame_copy, (cx - 20, cy), (cx + 20, cy), (255, 255, 0), 1)
        cv2.line(frame_copy, (cx, cy - 20), (cx, cy + 20), (255, 255, 0), 1)

        cv2.imshow("Detection Results", frame_copy)

        if wait_key:
            print("按任意键关闭窗口...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def track_object(
        self,
        target: str,
        duration: float = 30.0,
        max_lost_frames: int = 10,
        **detect_kwargs
    ):
        """
        实时追踪物体

        Args:
            target: 目标描述
            duration: 追踪时长(秒)
            max_lost_frames: 最大丢失帧数
            **detect_kwargs: 传递给detect_objects的参数
        """
        start_time = time.time()
        lost_count = 0
        last_position = None

        print(f"[VisionDetector] 开始追踪物体: {target}")

        while time.time() - start_time < duration:
            results = self.detect_objects(target, **detect_kwargs)

            if results:
                best = self.get_best_match(results)
                lost_count = 0

                # 显示结果
                self.show_results([best], annotate=True, wait_key=False)

                # 计算移动速度
                if last_position:
                    dx = best.pixel[0] - last_position[0]
                    dy = best.pixel[1] - last_position[1]
                    speed = math.sqrt(dx**2 + dy**2)
                    print(f"位置: {best.pixel}, 速度: {speed:.1f} px/frame")

                last_position = best.pixel
            else:
                lost_count += 1
                print(f"丢失目标 ({lost_count}/{max_lost_frames})")

                if lost_count >= max_lost_frames:
                    print("[VisionDetector] 目标丢失超过阈值,停止追踪")
                    break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[VisionDetector] 用户中断追踪")
                break

            time.sleep(0.03)  # ~30fps

        cv2.destroyAllWindows()
        print("[VisionDetector] 追踪结束")

    def release(self):
        """释放资源"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("[VisionDetector] 资源已释放")


# 使用示例
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="SO100 视觉检测器")
    parser.add_argument("--camera", type=int, default=0, help="摄像头ID")
    parser.add_argument("--detector", type=str, default="yolo", choices=["yolo", "qwen_vl", "color"],
                       help="检测器类型")
    parser.add_argument("--target", type=str, default="cup", help="检测目标")
    parser.add_argument("--confidence", type=float, default=0.5, help="置信度阈值")
    parser.add_argument("--output", type=str, default="all", choices=["pixel", "3d", "polar", "all"],
                       help="输出格式")
    parser.add_argument("--preview", action="store_true", help="显示预览窗口")

    args = parser.parse_args()

    # 创建检测器
    detector = VisionDetector(
        camera_id=args.camera,
        detector=args.detector,
        output_format=args.output
    )

    try:
        # 加载模型
        detector.load_model()

        # 检测物体
        print(f"\n检测目标: {args.target}")
        results = detector.detect_objects(
            target=args.target,
            confidence=args.confidence
        )

        if results:
            print(f"\n找到 {len(results)} 个目标:\n")

            for i, obj in enumerate(results, 1):
                print(f"[{i}] 类别: {obj.class_name}, 置信度: {obj.confidence:.2f}")
                print(f"    像素坐标: {obj.pixel}")
                print(f"    边界框: {obj.bounding_box}")

                if obj.arm_3d:
                    print(f"    3D坐标: ({obj.arm_3d[0]:.1f}, {obj.arm_3d[1]:.1f}, {obj.arm_3d[2]:.1f}) mm")

                if obj.polar:
                    print(f"    极坐标: 角度={obj.polar[0]:.1f}°, 距离={obj.polar[1]:.1f} px")

                print()

            # 显示结果
            if args.preview:
                detector.show_results(results, annotate=True)
        else:
            print(f"未检测到目标: {args.target}")

    finally:
        detector.release()
