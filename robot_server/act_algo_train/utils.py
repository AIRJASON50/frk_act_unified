#!/usr/bin/env python3
"""
ACT Data Processing Utilities - Franka Single Arm

================================================================================
FILE DESCRIPTION:
本文件包含ACT训练的数据处理工具函数，专门为Franka单臂系统设计。
主要负责HDF5数据集的加载、预处理、归一化和批量化处理。

MAIN FUNCTIONALITY:
1. HDF5数据集加载和解析
2. 数据归一化和预处理
3. 训练/验证数据集分割
4. 批量数据加载和管理
5. 机器人状态和动作的统计计算

KEY COMPONENTS:
- EpisodicDataset: Episode数据集类
- load_data(): 主要数据加载函数
- get_norm_stats(): 数据统计计算
- 环境和机器人辅助函数

DATA PIPELINE:
原始HDF5 → Episode解析 → 数据归一化 → DataLoader批量化 → 训练使用

INPUT DATA FORMAT:
HDF5结构:
- /observations/qpos: (T, 8) - 关节位置
- /observations/qvel: (T, 8) - 关节速度  
- /observations/images/top: (T, 480, 640, 3) - RGB图像
- /action: (T, 8) - 动作序列
- sim: bool - 仿真标志

OUTPUT DATA FORMAT:
- image_data: (batch, cameras, 3, H, W) - 归一化RGB图像
- qpos_data: (batch, 8) - 归一化关节状态
- action_data: (batch, 100, 8) - 归一化动作序列
- is_pad: (batch, 100) - 填充掩码

FRANKA ADAPTATION:
✅ 8维状态/动作空间处理
✅ 单相机图像处理
✅ 100步动作chunk采样
✅ 符合Franka数据格式
✅ 变长episode支持

DATA NORMALIZATION:
- 图像: [0,255] → [0,1] → ImageNet标准化
- 状态/动作: 零均值单位方差归一化
- 填充处理: 自动检测和标记
================================================================================
"""

import numpy as np
import torch
import os
from .franka_constants import DT
import h5py
from torch.utils.data import TensorDataset, DataLoader

import IPython
e = IPython.embed

class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_ids, dataset_dir, camera_names, norm_stats):
        super(EpisodicDataset).__init__()
        self.episode_ids = episode_ids
        self.dataset_dir = dataset_dir
        self.camera_names = camera_names
        self.norm_stats = norm_stats
        self.is_sim = None
        self.__getitem__(0) # initialize self.is_sim

    def __len__(self):
        return len(self.episode_ids)

    def __getitem__(self, index):
        episode_id = self.episode_ids[index]
        # 适配实际文件名格式
        episode_files = sorted([f for f in os.listdir(self.dataset_dir) if f.endswith('.hdf5')])
        if episode_id < len(episode_files):
            dataset_path = os.path.join(self.dataset_dir, episode_files[episode_id])
        else:
            # 备选格式
            dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')
        
        with h5py.File(dataset_path, 'r') as root:
            is_sim = root.attrs['sim']
            episode_len = root['/action'].shape[0]
            
            # 随机选择一个起始时间点
            max_start = max(0, episode_len - 100)  # 确保至少有100个时间步
            start_ts = np.random.choice(max_start + 1) if max_start > 0 else 0
            
            # 获取单个时间步的观测
            qpos = root['/observations/qpos'][start_ts]
            qvel = root['/observations/qvel'][start_ts]
            
            # 获取相机图像
            image_dict = dict()
            for cam_name in self.camera_names:
                image_dict[cam_name] = root[f'/observations/images/{cam_name}'][start_ts]
            
            # 获取固定长度的动作序列 (chunk_size = 100)
            chunk_size = 100
            end_ts = min(start_ts + chunk_size, episode_len)
            action = root['/action'][start_ts:end_ts]
            
            # 如果不足100步，用padding填充
            if action.shape[0] < chunk_size:
                padding_len = chunk_size - action.shape[0]
                padding = np.zeros((padding_len, action.shape[1]), dtype=action.dtype)
                action = np.concatenate([action, padding], axis=0)
            
            # 创建padding mask
            is_pad = np.zeros(chunk_size, dtype=bool)
            if end_ts - start_ts < chunk_size:
                is_pad[end_ts - start_ts:] = True

        self.is_sim = is_sim

        # 处理多相机图像
        all_cam_images = []
        for cam_name in self.camera_names:
            all_cam_images.append(image_dict[cam_name])
        all_cam_images = np.stack(all_cam_images, axis=0)

        # 转换为torch张量并确保数据类型
        image_data = torch.from_numpy(all_cam_images).float()
        qpos_data = torch.from_numpy(qpos).float()
        action_data = torch.from_numpy(action).float()
        is_pad = torch.from_numpy(is_pad).bool()

        # 图像通道调整: (cameras, height, width, channels) -> (cameras, channels, height, width)
        image_data = torch.einsum('k h w c -> k c h w', image_data)

        # 归一化
        image_data = image_data / 255.0
        action_data = (action_data - torch.from_numpy(self.norm_stats["action_mean"]).float()) / torch.from_numpy(self.norm_stats["action_std"]).float()
        qpos_data = (qpos_data - torch.from_numpy(self.norm_stats["qpos_mean"]).float()) / torch.from_numpy(self.norm_stats["qpos_std"]).float()

        return image_data, qpos_data, action_data, is_pad


def get_norm_stats(dataset_dir, num_episodes):
    all_qpos_data = []
    all_action_data = []
    
    # 获取所有episode文件
    episode_files = [f for f in os.listdir(dataset_dir) if f.startswith('episode_') and f.endswith('.hdf5')]
    
    for episode_idx in range(min(num_episodes, len(episode_files))):
        dataset_path = os.path.join(dataset_dir, episode_files[episode_idx])
        try:
            with h5py.File(dataset_path, 'r') as root:
                qpos = root['/observations/qpos'][()]
                action = root['/action'][()]
            all_qpos_data.append(torch.from_numpy(qpos))
            all_action_data.append(torch.from_numpy(action))
        except Exception as e:
            print(f"Error loading {episode_files[episode_idx]}: {e}")
            continue
    
    if not all_qpos_data:
        raise ValueError("No valid episode data found for computing normalization stats")
        
    # 处理不同长度的episode - 将所有数据concatenate而不是stack
    all_qpos_data = torch.cat(all_qpos_data, dim=0)  # (total_steps, 8)
    all_action_data = torch.cat(all_action_data, dim=0)  # (total_steps, 8)

    # normalize action data
    action_mean = all_action_data.mean(dim=0, keepdim=True)  # (1, 8)
    action_std = all_action_data.std(dim=0, keepdim=True)   # (1, 8)  
    action_std = torch.clip(action_std, 1e-2, np.inf) # clipping

    # normalize qpos data
    qpos_mean = all_qpos_data.mean(dim=0, keepdim=True)     # (1, 8)
    qpos_std = all_qpos_data.std(dim=0, keepdim=True)      # (1, 8)
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf) # clipping

    stats = {"action_mean": action_mean.numpy().squeeze(), "action_std": action_std.numpy().squeeze(),
             "qpos_mean": qpos_mean.numpy().squeeze(), "qpos_std": qpos_std.numpy().squeeze(),
             "example_qpos": qpos}

    return stats


def load_data(dataset_dir, num_episodes, camera_names, batch_size_train, batch_size_val):
    print(f'\nData from: {dataset_dir}\n')
    # obtain train test split
    train_ratio = 0.8
    shuffled_indices = np.random.permutation(num_episodes)
    train_indices = shuffled_indices[:int(train_ratio * num_episodes)]
    val_indices = shuffled_indices[int(train_ratio * num_episodes):]

    # obtain normalization stats for qpos and action
    norm_stats = get_norm_stats(dataset_dir, num_episodes)

    # construct dataset and dataloader
    train_dataset = EpisodicDataset(train_indices, dataset_dir, camera_names, norm_stats)
    val_dataset = EpisodicDataset(val_indices, dataset_dir, camera_names, norm_stats)
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size_train, shuffle=True, pin_memory=True, num_workers=0)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val, shuffle=True, pin_memory=True, num_workers=0)

    return train_dataloader, val_dataloader, norm_stats, train_dataset.is_sim


### env utils

def sample_box_pose():
    x_range = [0.0, 0.2]
    y_range = [0.4, 0.6]
    z_range = [0.05, 0.05]

    ranges = np.vstack([x_range, y_range, z_range])
    cube_position = np.random.uniform(ranges[:, 0], ranges[:, 1])

    cube_quat = np.array([1, 0, 0, 0])
    return np.concatenate([cube_position, cube_quat])

def sample_insertion_pose():
    # Peg
    x_range = [0.1, 0.2]
    y_range = [0.4, 0.6]
    z_range = [0.05, 0.05]

    ranges = np.vstack([x_range, y_range, z_range])
    peg_position = np.random.uniform(ranges[:, 0], ranges[:, 1])

    peg_quat = np.array([1, 0, 0, 0])
    peg_pose = np.concatenate([peg_position, peg_quat])

    # Socket
    x_range = [-0.2, -0.1]
    y_range = [0.4, 0.6]
    z_range = [0.05, 0.05]

    ranges = np.vstack([x_range, y_range, z_range])
    socket_position = np.random.uniform(ranges[:, 0], ranges[:, 1])

    socket_quat = np.array([1, 0, 0, 0])
    socket_pose = np.concatenate([socket_position, socket_quat])

    return peg_pose, socket_pose

### helper functions

def compute_dict_mean(epoch_dicts):
    result = {k: None for k in epoch_dicts[0]}
    num_items = len(epoch_dicts)
    for k in result:
        value_sum = 0
        for epoch_dict in epoch_dicts:
            value_sum += epoch_dict[k]
        result[k] = value_sum / num_items
    return result

def detach_dict(d):
    new_d = dict()
    for k, v in d.items():
        if isinstance(v, torch.Tensor):
            new_d[k] = v.detach().cpu().numpy()
        else:
            new_d[k] = v
    return new_d

def set_seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)
