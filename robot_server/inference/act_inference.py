#!/usr/bin/env python3
"""
ACT模型推理模块
基于训练好的Franka单臂ACT模型进行推理
"""

import torch
import torchvision.transforms as transforms
import numpy as np
import pickle
import time
import os
import sys
from pathlib import Path
from typing import Dict, Any, Tuple, Optional
import traceback

# 添加路径以导入ACT组件
current_dir = Path(__file__).parent
robot_server_dir = current_dir.parent
sys.path.append(str(robot_server_dir))

from act_algo_train.policy import ACTPolicy
from act_algo_train.franka_constants import STATE_DIM, ACTION_DIM, CHUNK_SIZE

# 常量定义
class ACTConstants:
    STATE_DIM = 8
    ACTION_DIM = 8  
    CHUNK_SIZE = 100
    IMAGE_HEIGHT = 480
    IMAGE_WIDTH = 640

class ACTConfig:
    @staticmethod
    def get_model_config():
        return {
            'lr': 1e-5,
            'num_queries': 100,
            'kl_weight': 10,
            'hidden_dim': 512,
            'dim_feedforward': 3200,
            'lr_backbone': 1e-5,
            'backbone': 'resnet18',
            'enc_layers': 4,
            'dec_layers': 7,
            'nheads': 8,
            'camera_names': ['top']
        }
    
    @staticmethod
    def get_default_model_path():
        return "/home/wujielin/CascadeProjects/data/act_training/models/checkpoints/franka_pick_place"

class ACTInferenceEngine:
    """ACT推理引擎"""
    
    def __init__(self, model_path: Optional[str] = None, device: str = "auto"):
        """
        初始化推理引擎
        
        Args:
            model_path: 模型检查点路径，如果为None则使用默认路径
            device: 推理设备，"auto"自动选择，"cuda"或"cpu"
        """
        self.model = None
        self.device = self._setup_device(device)
        self.model_path = model_path or ACTConfig.get_default_model_path()
        self.is_loaded = False
        
        # 数据标准化参数
        self.stats = None
        self.pre_process = None
        self.post_process = None
        
        # 性能统计
        self.inference_count = 0
        self.total_inference_time = 0.0
        
        print(f"🤖 ACT推理引擎初始化")
        print(f"   模型路径: {self.model_path}")
        print(f"   推理设备: {self.device}")
    
    def _setup_device(self, device: str) -> torch.device:
        """设置推理设备"""
        if device == "auto":
            if torch.cuda.is_available():
                device = "cuda"
                print(f"🚀 检测到CUDA，使用GPU加速")
            else:
                device = "cpu"
                print(f"💻 使用CPU推理")
        
        return torch.device(device)
    
    def load_model(self) -> bool:
        """
        加载ACT模型
        
        Returns:
            bool: 加载是否成功
        """
        try:
            print(f"📦 开始加载ACT模型...")
            
            # 查找最佳的检查点
            model_file = self._find_best_checkpoint("best")
            if model_file is None:
                raise FileNotFoundError(f"未找到模型文件: {self.model_path}")
            
            print(f"📄 使用模型文件: {model_file.name}")
            
            # 显示模型选择信息
            self._print_model_info(model_file)
            
            # 创建模型配置
            config = ACTConfig.get_model_config()
            
            # 创建策略模型
            self.model = ACTPolicy(config)
            
            # 加载检查点
            checkpoint = torch.load(model_file, map_location=self.device)
            
            # 检查checkpoint格式并加载权重
            if 'model_state_dict' in checkpoint:
                self.model.load_state_dict(checkpoint['model_state_dict'])
                print(f"📊 检查点格式: 标准格式")
            else:
                # 直接是模型权重
                self.model.load_state_dict(checkpoint)
                print(f"📊 检查点格式: 权重格式")
            
            # 设置为评估模式并移到指定设备
            self.model.eval()
            if hasattr(self.model, 'to'):
                self.model = self.model.to(self.device)
            
            # 加载数据统计信息并设置标准化函数
            if not self._load_dataset_stats():
                print("⚠️  未找到数据统计文件，将使用原始数据")
            
            # 验证模型
            self._validate_model()
            
            self.is_loaded = True
            print(f"✅ ACT模型加载成功！")
            print(f"   参数数量: {self._count_parameters():.2f}M")
            print(f"   输入维度: qpos({STATE_DIM}) + image({ACTConstants.IMAGE_HEIGHT}×{ACTConstants.IMAGE_WIDTH}×3)")
            print(f"   输出维度: actions({CHUNK_SIZE}×{ACTION_DIM})")
            print(f"   设备: {self.device}")
            print(f"   图像预处理: ImageNet标准化（与训练时一致）")
            norm_status = "已启用" if self.stats is not None else "未启用"
            print(f"   数据标准化: {norm_status} ✨")
            
            # 打印可用模型列表
            self._print_available_models()
            
            return True
            
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            traceback.print_exc()
            return False
    
    def _find_best_checkpoint(self, selection_mode: str = "best") -> Optional[Path]:
        """查找最佳检查点文件
        
        Args:
            selection_mode: 选择模式
                - "best": 优先选择policy_best.ckpt，如不存在则选择最新epoch
                - "latest": 选择最新epoch的模型
                - "specific": 使用完整路径指定的模型文件
        """
        model_path = Path(self.model_path)
        
        if model_path.is_file():
            print(f"📁 使用指定模型文件: {model_path.name}")
            return model_path
        elif model_path.is_dir():
            print(f"📁 在目录中查找模型: {model_path}")
            
            if selection_mode == "best":
                # 优先查找best模型
                best_file = model_path / "policy_best.ckpt"
                if best_file.exists():
                    print(f"✅ 找到最佳模型: policy_best.ckpt (训练过程中验证损失最小)")
                    return best_file
                
                # 如果没有best，查找last
                last_file = model_path / "policy_last.ckpt"
                if last_file.exists():
                    print(f"✅ 找到最终模型: policy_last.ckpt (最后一轮训练)")
                    return last_file
            
            # 查找epoch模型文件
            ckpt_files = list(model_path.glob('policy_epoch_*.ckpt'))
            if not ckpt_files:
                print(f"❌ 未找到任何模型文件")
                return None
            
            # 按epoch排序
            ckpt_files.sort(key=lambda x: int(x.stem.split('_')[2]))
            
            if selection_mode == "latest" or selection_mode == "best":
                selected_file = ckpt_files[-1]
                epoch_num = int(selected_file.stem.split('_')[2])
                print(f"✅ 选择最新epoch模型: {selected_file.name} (epoch {epoch_num})")
                return selected_file
                
        else:
            print(f"❌ 路径不存在: {model_path}")
            return None
    
    def _find_latest_checkpoint(self) -> Optional[Path]:
        """向后兼容的方法"""
        return self._find_best_checkpoint("latest")
    
    def _load_dataset_stats(self) -> bool:
        """加载数据集统计信息用于数据标准化"""
        try:
            # 查找统计文件
            model_dir = Path(self.model_path)
            if model_dir.is_file():
                model_dir = model_dir.parent
            
            stats_file = model_dir / "dataset_stats.pkl"
            if not stats_file.exists():
                print(f"⚠️  统计文件不存在: {stats_file}")
                return False
            
            # 加载统计数据
            with open(stats_file, 'rb') as f:
                self.stats = pickle.load(f)
            
            # 设置标准化函数
            qpos_mean = self.stats['qpos_mean']
            qpos_std = self.stats['qpos_std']
            action_mean = self.stats['action_mean']
            action_std = self.stats['action_std']
            
            self.pre_process = lambda qpos: (qpos - qpos_mean) / qpos_std
            self.post_process = lambda action: action * action_std + action_mean
            
            print(f"✅ 数据统计加载成功")
            print(f"   qpos_mean: {qpos_mean[:4]}... (8D)")
            print(f"   qpos_std: {qpos_std[:4]}... (8D)")
            print(f"   action_mean: {action_mean[:4]}... (8D)")
            print(f"   action_std: {action_std[:4]}... (8D)")
            print(f"   夹爪动作范围: mean={action_mean[7]:.3f}, std={action_std[7]:.3f}")
            
            return True
            
        except Exception as e:
            print(f"❌ 加载数据统计失败: {e}")
            return False
    
    def _validate_model(self):
        """验证模型是否正确加载"""
        print(f"🧪 验证模型功能...")
        
        # 创建测试数据
        batch_size = 1
        qpos = torch.randn(batch_size, STATE_DIM).to(self.device)
        
        # 创建标准化的图像数据（模拟实际输入）
        test_image = torch.randn(batch_size, 1, 3, ACTConstants.IMAGE_HEIGHT, ACTConstants.IMAGE_WIDTH).to(self.device)
        # 应用ImageNet标准化
        normalize_transform = transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225]
        )
        for b in range(batch_size):
            for c in range(1):  # camera数量
                test_image[b, c] = normalize_transform(torch.rand(3, ACTConstants.IMAGE_HEIGHT, ACTConstants.IMAGE_WIDTH).to(self.device))
        
        # 测试推理
        with torch.no_grad():
            actions = self.model(qpos, test_image)
        
        # 验证输出维度
        expected_shape = (batch_size, CHUNK_SIZE, ACTION_DIM)
        if actions.shape != expected_shape:
            raise RuntimeError(f"模型输出维度错误: 期望{expected_shape}, 实际{actions.shape}")
        
        # 检查输出范围是否合理
        action_range = [actions.min().item(), actions.max().item()]
        print(f"✅ 模型验证通过: 输出形状{actions.shape}, 动作范围{action_range}")
        
        # 如果动作范围异常，给出警告
        if abs(action_range[0]) > 10 or abs(action_range[1]) > 10:
            print(f"⚠️ 模型输出范围异常，可能存在预处理问题")
    
    def _print_model_info(self, model_file: Path):
        """打印模型详细信息"""
        try:
            # 获取文件大小
            file_size_mb = model_file.stat().st_size / (1024 * 1024)
            print(f"   文件大小: {file_size_mb:.1f}MB")
            
            # 如果是epoch模型，提取epoch信息
            if "epoch" in model_file.stem:
                epoch_num = int(model_file.stem.split('_')[2])
                print(f"   训练轮次: Epoch {epoch_num}")
            elif "best" in model_file.stem:
                print(f"   模型类型: 验证损失最小的模型 ⭐ 推荐")
            elif "last" in model_file.stem:
                print(f"   模型类型: 最终训练模型")
                
        except Exception as e:
            print(f"   信息获取失败: {e}")
    
    def _print_available_models(self):
        """打印可用的模型列表"""
        try:
            model_dir = Path(self.model_path)
            if model_dir.is_dir():
                print(f"📋 可用模型列表:")
                
                # 查找best和last模型
                special_files = ["policy_best.ckpt", "policy_last.ckpt"]
                for filename in special_files:
                    file_path = model_dir / filename
                    if file_path.exists():
                        size_mb = file_path.stat().st_size / (1024 * 1024)
                        print(f"   📄 {filename} ({size_mb:.1f}MB)")
                
                # 查找epoch模型
                epoch_files = sorted(model_dir.glob('policy_epoch_*.ckpt'), 
                                   key=lambda x: int(x.stem.split('_')[2]))
                if epoch_files:
                    print(f"   📄 Epoch模型: {len(epoch_files)}个 (Epoch {int(epoch_files[0].stem.split('_')[2])}-{int(epoch_files[-1].stem.split('_')[2])})")
                    
        except Exception as e:
            print(f"   模型列表获取失败: {e}")
    
    def _count_parameters(self) -> float:
        """计算模型参数数量(M)"""
        if self.model is None:
            return 0.0
        
        total_params = sum(p.numel() for p in self.model.parameters())
        return total_params / 1e6
    
    def predict(self, qpos: np.ndarray, image: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        执行ACT推理
        
        Args:
            qpos: 机器人关节位置 (8,)
            image: RGB图像 (H, W, 3)
            
        Returns:
            Tuple[np.ndarray, float]: (动作序列(100, 8), 推理耗时(ms))
        """
        if not self.is_loaded:
            raise RuntimeError("模型未加载，请先调用load_model()")
        
        start_time = time.time()
        
        try:
            # 数据预处理
            preprocess_start = time.time()
            qpos_tensor, image_tensor = self._preprocess_input(qpos, image)
            preprocess_time = (time.time() - preprocess_start) * 1000
            
            # 调试信息：输入数据分析
            print(f"🔍 输入数据分析:")
            print(f"   qpos原始: shape{qpos.shape}, 范围[{qpos.min():.3f}, {qpos.max():.3f}]")
            print(f"   qpos张量: shape{qpos_tensor.shape}, device={qpos_tensor.device}")
            print(f"   image原始: shape{image.shape}, dtype={image.dtype}, 像素范围[{image.min()}, {image.max()}]")
            print(f"   image张量: shape{image_tensor.shape}, device={image_tensor.device}")
            print(f"   image标准化: 使用ImageNet标准化 (mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])")
            print(f"   预处理耗时: {preprocess_time:.2f}ms")
            
            # 推理
            model_start = time.time()
            with torch.no_grad():
                actions_tensor = self.model(qpos_tensor, image_tensor)
            model_time = (time.time() - model_start) * 1000
            
            # 调试信息：模型输出分析
            print(f"🤖 模型输出分析:")
            print(f"   输出张量: shape{actions_tensor.shape}, device={actions_tensor.device}")
            print(f"   输出范围: [{actions_tensor.min().item():.3f}, {actions_tensor.max().item():.3f}]")
            print(f"   模型推理耗时: {model_time:.2f}ms")
            
            # 后处理
            postprocess_start = time.time()
            actions = self._postprocess_output(actions_tensor)
            postprocess_time = (time.time() - postprocess_start) * 1000
            
            # 计算总耗时
            total_time = (time.time() - start_time) * 1000  # ms
            
            # 调试信息：输出动作分析
            print(f"📊 动作序列分析:")
            print(f"   动作形状: {actions.shape} (chunk_size={CHUNK_SIZE}, action_dim={ACTION_DIM})")
            print(f"   动作范围: [{actions.min():.3f}, {actions.max():.3f}]")
            print(f"   关节动作范围: [{actions[:, :7].min():.3f}, {actions[:, :7].max():.3f}]")
            print(f"   夹爪动作范围: [{actions[:, 7].min():.3f}, {actions[:, 7].max():.3f}]")
            print(f"   后处理耗时: {postprocess_time:.2f}ms")
            
            # 性能分解统计
            print(f"⏱️ 性能分解统计:")
            print(f"   预处理: {preprocess_time:.2f}ms ({preprocess_time/total_time*100:.1f}%)")
            print(f"   模型推理: {model_time:.2f}ms ({model_time/total_time*100:.1f}%)")
            print(f"   后处理: {postprocess_time:.2f}ms ({postprocess_time/total_time*100:.1f}%)")
            print(f"   总耗时: {total_time:.2f}ms")
            
            # 更新统计
            self.inference_count += 1
            self.total_inference_time += total_time
            
            return actions, total_time
            
        except Exception as e:
            raise RuntimeError(f"推理失败: {e}")
    
    def _preprocess_input(self, qpos: np.ndarray, image: np.ndarray) -> Tuple[torch.Tensor, torch.Tensor]:
        """预处理输入数据"""
        # 验证输入
        if qpos.shape != (STATE_DIM,):
            raise ValueError(f"qpos维度错误: 期望({STATE_DIM},), 实际{qpos.shape}")
        
        # qpos数据标准化
        if self.pre_process is not None:
            qpos_normalized = self.pre_process(qpos)
            print(f"   qpos标准化: {qpos[:3]} -> {qpos_normalized[:3]} (前3个关节)")
        else:
            qpos_normalized = qpos
            print(f"   qpos未标准化: {qpos[:3]} (前3个关节)")
        
        expected_image_shape = (ACTConstants.IMAGE_HEIGHT, ACTConstants.IMAGE_WIDTH, 3)
        if image.shape != expected_image_shape:
            raise ValueError(f"image维度错误: 期望{expected_image_shape}, 实际{image.shape}")
        
        # 检查qpos数据合理性
        if np.any(np.isnan(qpos)) or np.any(np.isinf(qpos)):
            raise ValueError(f"qpos包含非法数值: NaN={np.any(np.isnan(qpos))}, Inf={np.any(np.isinf(qpos))}")
        
        # 检查图像数据合理性
        if not (0 <= image.min() <= image.max() <= 255):
            print(f"⚠️ 图像像素值异常: 范围[{image.min()}, {image.max()}], 应该在[0, 255]")
        
        # 检查图像是否为RGB格式
        if image.dtype != np.uint8:
            print(f"⚠️ 图像数据类型异常: {image.dtype}, 应该是uint8")
        
        # 转换为张量
        qpos_tensor = torch.from_numpy(qpos_normalized).float().unsqueeze(0).to(self.device)  # (1, 8)
        
        # 图像预处理: (H, W, 3) -> (1, 1, 3, H, W) 并使用ImageNet标准化
        # 先转换为 [0, 1] 范围
        image_normalized = image.astype(np.float32) / 255.0  # [0, 1]
        image_tensor = torch.from_numpy(image_normalized).permute(2, 0, 1).to(self.device)  # (3, H, W)
        
        # 使用与训练时相同的ImageNet标准化
        normalize_transform = transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225]
        )
        image_tensor = normalize_transform(image_tensor)  # ImageNet标准化
        image_tensor = image_tensor.unsqueeze(0).unsqueeze(0)  # (1, 1, 3, H, W)
        
        return qpos_tensor, image_tensor
    
    def _postprocess_output(self, actions_tensor: torch.Tensor) -> np.ndarray:
        """后处理输出数据"""
        # 检查输出张量数据合理性
        if torch.any(torch.isnan(actions_tensor)) or torch.any(torch.isinf(actions_tensor)):
            print(f"⚠️ 模型输出包含非法数值!")
            print(f"   NaN: {torch.any(torch.isnan(actions_tensor)).item()}")
            print(f"   Inf: {torch.any(torch.isinf(actions_tensor)).item()}")
        
        # 转换为numpy数组
        actions_raw = actions_tensor.cpu().numpy()  # (1, 100, 8)
        actions_raw = actions_raw[0]  # (100, 8)
        
        # 动作反标准化
        if self.post_process is not None:
            # 逐步反标准化每个动作
            actions = np.zeros_like(actions_raw)
            for i in range(actions_raw.shape[0]):
                actions[i] = self.post_process(actions_raw[i])
            print(f"   动作反标准化: 原始[{actions_raw[0, :3]}] -> 结果[{actions[0, :3]}]")
        else:
            actions = actions_raw
            print(f"   动作未反标准化: {actions[0, :3]}")
        
        # 验证输出
        expected_shape = (CHUNK_SIZE, ACTION_DIM)
        if actions.shape != expected_shape:
            raise RuntimeError(f"输出维度错误: 期望{expected_shape}, 实际{actions.shape}")
        
        # 检查动作数据合理性
        if np.any(np.isnan(actions)) or np.any(np.isinf(actions)):
            raise RuntimeError(f"动作输出包含非法数值!")
        
        # 检查动作范围是否合理（简单检查）
        joint_actions = actions[:, :7]  # 关节动作
        gripper_actions = actions[:, 7]   # 夹爪动作
        
        # 检查关节动作范围 (一般在 -3 到 3 之间)
        joint_max = np.abs(joint_actions).max()
        if joint_max > 5.0:
            print(f"⚠️ 关节动作范围异常: {joint_max:.3f} > 5.0")
        elif joint_max > 3.0:
            print(f"ℹ️ 关节动作范围较大: {joint_max:.3f} (正常范围: ±3.0)")
        
        # 检查夹爪动作范围 (反标准化后应该在 0 到 0.08 之间)
        gripper_min, gripper_max = gripper_actions.min(), gripper_actions.max()
        if self.stats is not None:
            # 反标准化后的合理范围
            if gripper_min < -0.01 or gripper_max > 0.09:
                print(f"⚠️ 夹爪动作范围异常: [{gripper_min:.3f}, {gripper_max:.3f}] (期望范围: [0, 0.08])")
            elif gripper_min < 0 or gripper_max > 0.08:
                print(f"ℹ️ 夹爪动作范围较大: [{gripper_min:.3f}, {gripper_max:.3f}] (正常范围: [0, 0.08])")
        else:
            # 未标准化数据的范围检查
            if gripper_min < -0.1 or gripper_max > 0.1:
                print(f"⚠️ 夹爪动作范围异常: [{gripper_min:.3f}, {gripper_max:.3f}] (未标准化数据)")
        
        # 检查动作序列的连续性（第一步和后续步骤的差异）
        if actions.shape[0] > 1:
            action_diff = np.abs(actions[1:] - actions[:-1]).max()
            if action_diff > 1.0:
                print(f"ℹ️ 动作序列变化较大: 最大步间差异 {action_diff:.3f}")
        
        return actions
    
    def get_stats(self) -> Dict[str, Any]:
        """获取性能统计信息"""
        if self.inference_count == 0:
            avg_time = 0.0
        else:
            avg_time = self.total_inference_time / self.inference_count
        
        return {
            "is_loaded": self.is_loaded,
            "device": str(self.device),
            "model_path": str(self.model_path),
            "inference_count": self.inference_count,
            "total_time_ms": self.total_inference_time,
            "avg_time_ms": avg_time,
            "parameters_M": self._count_parameters() if self.is_loaded else 0.0,
            "data_normalization": self.stats is not None,
            "qpos_stats": {
                "mean": self.stats['qpos_mean'].tolist() if self.stats else None,
                "std": self.stats['qpos_std'].tolist() if self.stats else None
            } if self.stats else None,
            "action_stats": {
                "mean": self.stats['action_mean'].tolist() if self.stats else None,
                "std": self.stats['action_std'].tolist() if self.stats else None
            } if self.stats else None
        }
    
    def reset_stats(self):
        """重置性能统计"""
        self.inference_count = 0
        self.total_inference_time = 0.0
        print("📊 统计信息已重置")

# ============================================================================
# 测试代码
# ============================================================================

def test_inference_engine():
    """测试推理引擎"""
    print("🧪 开始测试ACT推理引擎...")
    
    # 创建推理引擎
    engine = ACTInferenceEngine()
    
    # 加载模型
    if not engine.load_model():
        print("❌ 模型加载失败，退出测试")
        return False
    
    # 创建测试数据
    qpos = np.random.randn(8) * 0.1  # 小幅度随机位置
    image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)  # 随机图像
    
    print(f"🎯 执行推理测试...")
    
    try:
        # 执行推理
        actions, inference_time = engine.predict(qpos, image)
        
        print(f"✅ 推理成功!")
        print(f"   输入: qpos{qpos.shape}, image{image.shape}")
        print(f"   输出: actions{actions.shape}")
        print(f"   耗时: {inference_time:.2f}ms")
        print(f"   动作范围: [{actions.min():.3f}, {actions.max():.3f}]")
        
        # 打印统计信息
        stats = engine.get_stats()
        print(f"📊 性能统计: {stats}")
        
        return True
        
    except Exception as e:
        print(f"❌ 推理失败: {e}")
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_inference_engine()
    print(f"\n{'🎉 测试通过!' if success else '❌ 测试失败!'}")
