# ACT算法模块架构文档

## 📁 目录结构

```
act_algo_train/
├── README.md                 # 本文档
├── franka_constants.py       # Franka机器人常量配置
├── imitate_episodes.py       # 主训练脚本
├── policy.py                 # 策略封装类
├── utils.py                  # 数据处理工具
└── detr/                     # DETR核心模块
    ├── models/               # 模型定义
    │   ├── __init__.py       # 模型构建入口
    │   ├── backbone.py       # CNN骨干网络
    │   ├── detr_vae.py      # 核心ACT模型
    │   ├── position_encoding.py # 位置编码
    │   └── transformer.py   # Transformer架构
    └── util/
        └── misc.py          # 工具函数
```

## 🔧 核心文件功能

### 配置层
- **`franka_constants.py`**: Franka机器人硬件参数、任务配置、模型超参数

### 训练层  
- **`imitate_episodes.py`**: 主训练入口，数据加载→模型训练→保存
- **`policy.py`**: 策略类封装，损失计算，优化器管理
- **`utils.py`**: HDF5数据加载，归一化，DataLoader创建

### 模型层
- **`detr_vae.py`**: ACT核心模型(DETR+VAE)
- **`transformer.py`**: Transformer编码器-解码器
- **`backbone.py`**: ResNet视觉特征提取
- **`position_encoding.py`**: 空间/时序位置编码

## 🔄 协同工作流

### 训练流程
```
1. imitate_episodes.py (启动)
   ↓
2. franka_constants.py (读取配置)
   ↓  
3. utils.py (加载HDF5数据)
   ↓
4. policy.py (创建ACT策略)
   ↓
5. detr_vae.py (构建模型)
   ↓
6. backbone.py + transformer.py (组装架构)
   ↓
7. 训练循环 (前向→损失→优化)
```

### 推理流程
```
1. 加载训练好的模型检查点
2. policy.py 创建策略实例
3. 输入: qpos(8) + RGB图像
4. 输出: 100步动作序列
```

## 📊 数据流

```
HDF5 Episodes → utils.py → DataLoader
                               ↓
qpos(8) + RGB(480×640×3) → policy.py → ACT Model
                               ↓
                        actions(100×8)
```

## 🎯 Franka适配要点

- **状态维度**: 8 (7关节+1夹爪)
- **图像输入**: 单相机俯视 480×640×3
- **动作预测**: 100步chunk，50Hz控制
- **模型规模**: 83.91M参数

## 🚀 快速使用

```bash
# 训练
python -m act_algo_train.imitate_episodes --task_name franka_pick_place --policy_class ACT

# 自定义参数
python -m act_algo_train.imitate_episodes \
    --task_name franka_pick_place \
    --policy_class ACT \
    --batch_size 4 \
    --num_epochs 500 \
    --lr 1e-5 \
    --kl_weight 10
```

---
*架构版本: v1.0 | 适配: Franka单臂 | 更新: 2025-09-20*
