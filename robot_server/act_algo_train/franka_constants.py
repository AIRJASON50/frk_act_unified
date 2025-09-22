#!/usr/bin/env python3
"""
Franka Single Arm Constants - ACT Training Configuration

================================================================================
FILE DESCRIPTION:
本文件定义了Franka单臂机器人ACT训练的所有常量配置，包括机器人硬件参数、
任务配置、模型超参数和数据路径等。这是整个ACT训练系统的配置中心。

MAIN FUNCTIONALITY:
1. 机器人硬件参数定义（关节限制、夹爪参数等）
2. 任务配置管理（数据集路径、episode配置等）
3. ACT模型默认超参数
4. 训练和评估的阈值设置

KEY COMPONENTS:
- FRANKA_TASK_CONFIGS: 任务配置字典
- 关节和夹爪物理限制参数
- 图像尺寸和相机配置
- 模型架构默认参数
- 归一化和反归一化函数

INPUT/OUTPUT:
输入: 无直接输入（配置文件）
输出: 
- 任务配置字典
- 机器人物理参数
- 模型超参数
- 归一化函数

FRANKA SPECIFICATIONS:
✅ 7DOF机械臂 + 2DOF平行夹爪 = 8维动作空间
✅ 关节限制: 符合Franka Emika Panda规格
✅ 夹爪范围: 0-0.08m（标准Franka夹爪）
✅ 控制频率: 50Hz (DT=0.02s)
✅ 图像输入: 480×640×3 RGB

ADAPTATION STATUS:
✅ 完全适配Franka单臂系统
✅ 8维状态/动作空间配置
✅ 单相机配置（top视角）
✅ 符合Franka物理限制

AUTHORS: Franka ACT Team
VERSION: v1.0 - Franka Single Arm Specialized
LAST UPDATED: 2025-09-20
================================================================================
"""

import os
import pathlib

### Task parameters
DATA_DIR = '/home/wujielin/CascadeProjects/data/act_training/datasets'

# Franka single arm task configurations
FRANKA_TASK_CONFIGS = {
    'franka_pick_place': {
        'dataset_dir': os.path.join(DATA_DIR, 'act_0918'),
        'num_episodes': 20,  # 我们有20个episode
        'episode_len': 400,  # 典型的pick-and-place长度
        'camera_names': ['top']  # 单相机配置
    },
    'act_data': {
        'dataset_dir': os.path.join(DATA_DIR, 'act_data'),
        'num_episodes': 50,  # 新的笛卡尔数据集有50个episode
        'episode_len': 381,  # 根据验证结果的实际长度
        'camera_names': ['top']  # 单相机配置
    },
    
    'sim_transfer_cube_scripted': {
        'dataset_dir': os.path.join(DATA_DIR, 'sim_transfer_cube_scripted'),
        'num_episodes': 27,  # 检测到27个文件
        'episode_len': 400,
        'camera_names': ['top']
    }
}

### Franka Robot Constants
DT = 0.02  # 50Hz control frequency

# Franka joint names (7 DOF)
FRANKA_JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", 
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
]

# Joint position limits (radians) - from Franka documentation
JOINT_LIMITS = {
    'lower': [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    'upper': [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
}

# Joint velocity limits (rad/s)
JOINT_VELOCITY_LIMITS = [2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610]

# Joint effort limits (Nm)
JOINT_EFFORT_LIMITS = [87, 87, 87, 87, 12, 12, 12]

# Franka gripper parameters (parallel jaw gripper)
GRIPPER_WIDTH_MIN = 0.0  # meters (fully closed)
GRIPPER_WIDTH_MAX = 0.08  # meters (fully open)
GRIPPER_FORCE_MIN = 0.0  # Newtons
GRIPPER_FORCE_MAX = 70.0  # Newtons
GRIPPER_SPEED = 0.1  # meters/second

# Default grasp parameters for dataset collection
DEFAULT_GRASP_WIDTH = 0.035  # meters (for small objects)
DEFAULT_GRASP_FORCE = 15.0  # Newtons
DEFAULT_GRASP_SPEED = 0.05  # meters/second

# Home position for Franka (joint positions in radians)
FRANKA_HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Image dimensions from camera
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_CHANNELS = 3

### Helper functions for normalization

def normalize_gripper_width(width):
    """Normalize gripper width to [0, 1]"""
    return (width - GRIPPER_WIDTH_MIN) / (GRIPPER_WIDTH_MAX - GRIPPER_WIDTH_MIN)

def unnormalize_gripper_width(normalized):
    """Unnormalize gripper width from [0, 1] to actual meters"""
    return normalized * (GRIPPER_WIDTH_MAX - GRIPPER_WIDTH_MIN) + GRIPPER_WIDTH_MIN

def normalize_joint_position(pos, joint_idx):
    """Normalize joint position to [-1, 1]"""
    lower = JOINT_LIMITS['lower'][joint_idx]
    upper = JOINT_LIMITS['upper'][joint_idx]
    return 2.0 * (pos - lower) / (upper - lower) - 1.0

def unnormalize_joint_position(normalized, joint_idx):
    """Unnormalize joint position from [-1, 1] to actual radians"""
    lower = JOINT_LIMITS['lower'][joint_idx]
    upper = JOINT_LIMITS['upper'][joint_idx]
    return (normalized + 1.0) * (upper - lower) / 2.0 + lower

### ACT specific constants
STATE_DIM = 8  # 7 joints + 1 gripper
ACTION_DIM = 8  # 7 joints + 1 gripper
CHUNK_SIZE = 100  # ACT chunk size for action prediction

# Model architecture defaults (can be overridden)
DEFAULT_HIDDEN_DIM = 512
DEFAULT_DIM_FEEDFORWARD = 3200
DEFAULT_NUM_ENCODER_LAYERS = 4
DEFAULT_NUM_DECODER_LAYERS = 7
DEFAULT_NUM_HEADS = 8

# Training defaults
DEFAULT_BATCH_SIZE = 8
DEFAULT_LR = 1e-5
DEFAULT_KL_WEIGHT = 10
DEFAULT_NUM_EPOCHS = 2000
DEFAULT_SEED = 0

# Evaluation thresholds
POSITION_ERROR_THRESHOLD = 0.01  # meters
GRIPPER_ERROR_THRESHOLD = 0.005  # meters
SUCCESS_RATE_THRESHOLD = 0.8  # 80% success for good model

print(f"Franka constants loaded. Data directory: {DATA_DIR}")
