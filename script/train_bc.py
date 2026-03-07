#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 行为克隆训练脚本
基于 LeRobot 框架，适配 SO100 机械臂
"""

import os
import sys
import json
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from SDK import PortHandler, PacketHandler


# ===================== LeRobot 数据集加载器 =====================
class LeRobotDataset:
    """加载 LeRobot 格式数据集"""

    def __init__(self, dataset_path: str):
        self.dataset_path = Path(dataset_path)
        self.episodes = []
        self.metadata = self._load_metadata()

    def _load_metadata(self) -> dict:
        """加载元数据"""
        meta_file = self.dataset_path / "meta" / "info.json"
        if meta_file.exists():
            with open(meta_file) as f:
                return json.load(f)
        return {}

    def load_episodes(self, split: str = "train", max_episodes: Optional[int] = None):
        """加载 episodes"""
        import pandas as pd

        data_path = self.dataset_path / "data"
        if not data_path.exists():
            raise FileNotFoundError(f"Data path not found: {data_path}")

        # 加载所有 parquet 文件
        parquet_files = sorted(data_path.glob("**/episode_*.parquet"))

        if max_episodes:
            parquet_files = parquet_files[:max_episodes]

        print(f"[Dataset] Loading {len(parquet_files)} episodes...")

        for parquet_file in parquet_files:
            df = pd.read_parquet(parquet_file)
            self.episodes.append({
                'data': df,
                'file': str(parquet_file),
                'episode_index': df['episode_index'].iloc[0] if 'episode_index' in df.columns else 0
            })

        print(f"[Dataset] Loaded {len(self.episodes)} episodes, {sum(len(e['data']) for e in self.episodes)} frames")

        return self.episodes

    def get_statistics(self) -> dict:
        """获取数据集统计信息"""
        if not self.episodes:
            self.load_episodes()

        total_frames = sum(len(e['data']) for e in self.episodes)
        num_episodes = len(self.episodes)

        # 动作统计
        all_actions = []
        for ep in self.episodes:
            all_actions.extend(ep['data']['action'].tolist())

        actions_array = np.array(all_actions)
        action_means = np.mean(actions_array, axis=0)
        action_stds = np.std(actions_array, axis=0)

        return {
            'num_episodes': num_episodes,
            'total_frames': total_frames,
            'action_dim': actions_array.shape[1] if len(actions_array) > 0 else 0,
            'action_means': action_means,
            'action_stds': action_stds,
        }

    def get_action_sequence(self, episode_idx: int) -> np.ndarray:
        """获取动作序列"""
        if 0 <= episode_idx < len(self.episodes):
            return self.episodes[episode_idx]['data']['action'].values
        raise IndexError(f"Episode {episode_idx} out of range")


# ===================== SO100 适配器 =====================
class SO100Adapter:
    """
    将双臂数据适配到 SO100 单臂

    策略：只使用右臂数据，映射到 SO100 关节
    """

    # 数据集关节 -> SO100 关节的映射
    # galaxea_r1_lite 右臂 (7 DOF) -> SO100 (5 DOF + gripper)
    JOINT_MAPPING = {
        # 数据集右臂关节 -> SO100 关节
        0: None,      # right_arm_joint_1 -> rotation (需要角度转换)
        1: None,      # right_arm_joint_2 -> pitch
        2: None,      # right_arm_joint_3 -> elbow
        3: None,      # right_arm_joint_4 -> wrist_pitch
        4: None,      # right_arm_joint_5 -> wrist_roll
        5: None,      # right_arm_joint_6 -> (SO100 没有)
        6: 'gripper',  # right_gripper_open -> gripper
    }

    # SO100 关节限制 (度)
    SO100_LIMITS = [
        (-170, 170),   # rotation
        (-85, 85),     # pitch
        (-150, 150),   # elbow
        (-120, 120),   # wrist_pitch
        (-180, 180),   # wrist_roll
        (0, 100),      # gripper (0-100%)
    ]

    @staticmethod
    def map_action_to_so100(action: np.ndarray) -> dict:
        """
        将数据集动作映射到 SO100 关节角度

        Args:
            action: 14维动作数组 [左臂7维, 右臂7维]

        Returns:
            SO100 关节角度字典
        """
        # 提取右臂数据 (后7维)
        right_arm = action[7:]  # [j1, j2, j3, j4, j5, j6, gripper]

        # 转换弧度到角度
        right_arm_deg = np.rad2deg(right_arm)

        # SO100 关节角度（简化映射）
        so100_angles = {
            'rotation': right_arm_deg[0],   # 直接使用
            'pitch': right_arm_deg[1],      # 直接使用
            'elbow': right_arm_deg[2],      # 直接使用
            'wrist_pitch': right_arm_deg[3], # 直接使用
            'wrist_roll': right_arm_deg[4],  # 直接使用
            'gripper': right_arm_deg[6] * 100, # 转换到 0-100%
        }

        # 限制到有效范围
        for i, (key, (lo, hi)) in enumerate(so100_angles.items()):
            if key == 'gripper':
                so100_angles[key] = np.clip(so100_angles[key], 0, 100)
            else:
                so100_angles[key] = np.clip(so100_angles[key], lo, hi)

        return so100_angles

    @staticmethod
    def angle_to_position(angle_deg: float, center: int = 2047, range_deg: float = 270) -> int:
        """角度 -> 舵机位置"""
        return int(center + angle_deg * (4095 / range_deg))

    @staticmethod
    def position_to_angle(pos: int, center: int = 2047, range_deg: float = 270) -> float:
        """舵机位置 -> 角度"""
        return (pos - center) * (range_deg / 4095)


# ===================== SO100 硬件控制 =====================
class SO100Controller:
    """SO100 硬件控制器"""

    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56

    MOTOR_MAP = {
        'rotation': 1,
        'pitch': 2,
        'elbow': 3,
        'wrist_pitch': 4,
        'wrist_roll': 5,
        'gripper': 6,
    }

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
        print(f"[Hardware] Connected to {self.port}")

        # 检测电机
        found = self.ping_all()
        print(f"[Hardware] Motors: {found}/6")

        self.enable_all()

    def disconnect(self):
        self.disable_all()
        if self.port_handler:
            self.port_handler.closePort()
        print("[Hardware] Disconnected")

    def enable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.TORQUE_ENABLE, 0)

    def enable_all(self):
        for m in range(1, 7):
            self.enable_torque(m)

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

    def execute_action(self, action_dict: dict):
        """执行动作"""
        for name, value in action_dict.items():
            if name in self.MOTOR_MAP:
                motor_id = self.MOTOR_MAP[name]
                pos = SO100Adapter.angle_to_position(value)
                self.set_position(motor_id, pos)

    def get_current_state(self) -> dict:
        """获取当前状态"""
        positions = {m: self.get_position(m) for m in range(1, 7)}
        angles = {name: SO100Adapter.position_to_angle(positions[motor_id])
                  for name, motor_id in self.MOTOR_MAP.items()}
        return angles


# ===================== 行为克隆训练 =====================
class BehaviorCloningTrainer:
    """
    行为克隆训练器

    可以使用多种方法：
    1. 简单的 MLP 策略网络
    2. Transformer 策略
    3. Diffusion Policy
    """

    def __init__(self, state_dim: int = 5, action_dim: int = 6):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.model = None

    def create_mlp_policy(self, hidden_dims: List[int] = [256, 256, 256]):
        """创建简单的 MLP 策略网络"""
        import torch
        import torch.nn as nn

        class MLPolicy(nn.Module):
            def __init__(self, state_dim, action_dim, hidden_dims):
                super().__init__()
                layers = []
                input_dim = state_dim
                for hidden_dim in hidden_dims:
                    layers.append(nn.Linear(input_dim, hidden_dim))
                    layers.append(nn.ReLU())
                    input_dim = hidden_dim
                layers.append(nn.Linear(input_dim, action_dim))
                self.network = nn.Sequential(*layers)

            def forward(self, x):
                return self.network(x)

        self.model = MLPolicy(self.state_dim, self.action_dim, hidden_dims)
        return self.model

    def train(self, dataset: LeRobotDataset, epochs: int = 100, batch_size: int = 64):
        """
        训练策略网络

        Args:
            dataset: LeRobot 数据集
            epochs: 训练轮数
            batch_size: 批次大小
        """
        import torch
        import torch.nn as nn
        import torch.optim as optim
        from torch.utils.data import DataLoader, TensorDataset

        # 准备数据
        states = []
        actions = []

        for ep in dataset.episodes:
            df = ep['data']
            # 处理 observation.state - 检查数据结构
            state_col = df['observation.state']

            # 处理 action
            action_col = df['action']

            # 检查第一行数据结构
            if len(state_col) > 0:
                first_state = state_col.iloc[0]
                first_action = action_col.iloc[0]

                # 确定数据维度
                if isinstance(first_state, (list, np.ndarray)):
                    state_dim = len(first_state)
                else:
                    state_dim = 1

                # 处理每一帧
                for i in range(len(df)):
                    state = state_col.iloc[i]
                    action = action_col.iloc[i]

                    # 提取右臂数据 (索引7-12是右臂前6个关节，索引13是夹爪)
                    # state = [左臂0-6, 右臂7-12, 左夹爪, 右夹爪]
                    if state_dim >= 14:
                        right_state = state[7:13]  # 右臂6个关节 (弧度)
                    elif state_dim == 7:
                        right_state = state[0:6]
                    else:
                        continue

                    # SO100 状态 (5个关节 + 忽略 wrist_roll 第6关节)
                    # 简化：使用前5个关节
                    so100_state = right_state[:5]  # 弧度
                    states.append(so100_state)

                    # 提取右臂动作
                    if isinstance(action, (list, np.ndarray)):
                        if len(action) >= 14:
                            right_action = action[7:13]  # 右臂6个关节
                        elif len(action) == 7:
                            right_action = action[0:6]
                        else:
                            continue
                    else:
                        right_action = [action] * 6

                    # SO100 动作
                    so100_action = SO100Adapter.map_action_to_so100(np.array(right_action))
                    actions.append([so100_action[k] for k in ['rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper']])

        states = np.array(states, dtype=np.float32)
        actions = np.array(actions, dtype=np.float32)

        print(f"[Train] Prepared data: states={states.shape}, actions={actions.shape}")

        # 创建模型
        self.model = self.create_mlp_policy()
        criterion = nn.MSELoss()
        optimizer = optim.Adam(self.model.parameters(), lr=1e-3)

        # 训练
        dataset = TensorDataset(torch.from_numpy(states), torch.from_numpy(actions))
        loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

        print(f"[Train] Starting training: {epochs} epochs, {len(loader)} batches")
        self.model.train()

        for epoch in range(epochs):
            total_loss = 0
            for batch_states, batch_actions in loader:
                optimizer.zero_grad()
                predictions = self.model(batch_states)
                loss = criterion(predictions, batch_actions)
                loss.backward()
                optimizer.step()
                total_loss += loss.item()

            if (epoch + 1) % 10 == 0:
                avg_loss = total_loss / len(loader)
                print(f"[Train] Epoch {epoch+1}/{epochs}, Loss: {avg_loss:.6f}")

        print("[Train] Training complete!")

    def save_model(self, save_path: str):
        """保存模型"""
        import torch
        torch.save(self.model.state_dict(), save_path)
        print(f"[Train] Model saved to {save_path}")

    def load_model(self, load_path: str):
        """加载模型"""
        import torch
        self.model.load_state_dict(torch.load(load_path, weights_only=True))
        print(f"[Train] Model loaded from {load_path}")

    def predict(self, state: np.ndarray) -> dict:
        """预测动作"""
        import torch
        self.model.eval()
        with torch.no_grad():
            state_tensor = torch.from_numpy(state.astype(np.float32)).unsqueeze(0)
            action_tensor = self.model(state_tensor)
            action = action_tensor.squeeze(0).numpy()

        return {
            'rotation': action[0],
            'pitch': action[1],
            'elbow': action[2],
            'wrist_pitch': action[3],
            'wrist_roll': action[4],
            'gripper': action[5],
        }


# ===================== 主程序 =====================
def main():
    parser = argparse.ArgumentParser(description="SO100 行为克隆训练")
    parser.add_argument('--dataset', type=str, default='data/R1_Lite_pour_water', help='数据集路径')
    parser.add_argument('--mode', type=str, default='analyze',
                       choices=['analyze', 'train', 'play', 'collect'], help='运行模式')
    parser.add_argument('--port', type=str, default='COM7', help='串口')
    parser.add_argument('--episodes', type=int, default=10, help='加载的 episode 数量')
    parser.add_argument('--epochs', type=int, default=50, help='训练轮数')
    parser.add_argument('--model', type=str, default='models/so100_bc.pth', help='模型路径')

    args = parser.parse_args()

    print("="*60)
    print("    SO100 Behavior Cloning Training")
    print("="*60)

    if args.mode == 'analyze':
        # 分析数据集
        dataset = LeRobotDataset(args.dataset)
        stats = dataset.get_statistics()
        print(f"\n[Dataset Statistics]")
        print(f"  Episodes: {stats['num_episodes']}")
        print(f"  Total Frames: {stats['total_frames']}")
        print(f"  Action Dim: {stats['action_dim']}")
        print(f"  Action Mean: {stats['action_means'][:5]}...")  # 只显示前5个
        print(f"  Action Std:  {stats['action_stds'][:5]}...")

        # 显示第一个动作序列
        if len(dataset.episodes) > 0:
            action_seq = dataset.get_action_sequence(0)
            print(f"\n[Action Sequence] Episode 0, first 10 steps:")
            for i, action in enumerate(action_seq[:10]):
                print(f"  Step {i}: {action[:7]}...")  # 只显示右臂

    elif args.mode == 'train':
        # 训练策略
        dataset = LeRobotDataset(args.dataset)
        dataset.load_episodes(max_episodes=args.episodes)

        trainer = BehaviorCloningTrainer()
        trainer.train(dataset, epochs=args.epochs)

        # 保存模型
        os.makedirs(os.path.dirname(args.model) or '.', exist_ok=True)
        trainer.save_model(args.model)

    elif args.mode == 'play':
        # 使用训练好的模型控制机械臂
        if not os.path.exists(args.model):
            print(f"[Error] Model not found: {args.model}")
            return

        # 创建控制器
        controller = SO100Controller(args.port)
        controller.connect()

        try:
            # 加载模型
            trainer = BehaviorCloningTrainer()
            trainer.load_model(args.model)

            # 获取当前状态
            current = controller.get_current_state()
            state_array = np.array([current[k] for k in ['rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll']])

            # 预测并执行
            print("\n[Play] Predicting action...")
            action = trainer.predict(state_array)
            print(f"  Action: {action}")

            controller.execute_action(action)

            time.sleep(1)

        finally:
            controller.disconnect()

    elif args.mode == 'collect':
        # 收集 SO100 数据
        print("\n[Collect] Collecting SO100 demonstration data...")
        print("Instructions:")
        print("  1. Control the arm manually or via teleoperation")
        print("  2. Press Enter to record each frame")
        print("  3. Type 'done' to finish")

        controller = SO100Controller(args.port)
        controller.connect()

        try:
            episodes = []
            current_episode = []

            while True:
                cmd = input("\ncollect> ").strip()
                if cmd == 'done':
                    if current_episode:
                        episodes.append(current_episode)
                    break
                elif cmd == 'new':
                    if current_episode:
                        episodes.append(current_episode)
                        print(f"[Collect] Episode {len(episodes)} saved with {len(current_episode)} frames")
                    current_episode = []
                    print("[Collect] Started new episode")
                else:
                    # 记录当前状态
                    state = controller.get_current_state()
                    state_array = [state[k] for k in ['rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper']]
                    current_episode.append(state_array)
                    print(f"  Recorded: {state}")

            # 保存数据
            if episodes:
                import pandas as pd
                output_path = 'data/so100_demos/episodes.parquet'
                os.makedirs('data/so100_demos', exist_ok=True)

                all_data = []
                for i, ep in enumerate(episodes):
                    df = pd.DataFrame(ep, columns=['rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper'])
                    df['episode_index'] = i
                    df['frame_index'] = range(len(df))
                    all_data.append(df)

                combined = pd.concat(all_data, ignore_index=True)
                combined.to_parquet(output_path, index=False)
                print(f"[Collect] Saved {len(episodes)} episodes to {output_path}")

        finally:
            controller.disconnect()


if __name__ == "__main__":
    main()
