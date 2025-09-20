#!/usr/bin/env python3
"""
ACT模型推理模块
基于训练好的Franka单臂ACT模型进行推理
"""

import torch
import numpy as np
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
            
            # 查找最新的检查点
            model_file = self._find_latest_checkpoint()
            if model_file is None:
                raise FileNotFoundError(f"未找到模型文件: {self.model_path}")
            
            print(f"📄 使用模型文件: {model_file.name}")
            
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
            
            # 验证模型
            self._validate_model()
            
            self.is_loaded = True
            print(f"✅ ACT模型加载成功！")
            print(f"   参数数量: {self._count_parameters():.2f}M")
            print(f"   输入维度: qpos({STATE_DIM}) + image({ACTConstants.IMAGE_HEIGHT}×{ACTConstants.IMAGE_WIDTH}×3)")
            print(f"   输出维度: actions({CHUNK_SIZE}×{ACTION_DIM})")
            
            return True
            
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            traceback.print_exc()
            return False
    
    def _find_latest_checkpoint(self) -> Optional[Path]:
        """查找最新的检查点文件"""
        model_path = Path(self.model_path)
        
        if model_path.is_file():
            return model_path
        elif model_path.is_dir():
            # 查找目录中的检查点文件
            ckpt_files = list(model_path.glob('policy_epoch_*.ckpt'))
            if not ckpt_files:
                return None
            
            # 按epoch排序，取最新的
            ckpt_files.sort(key=lambda x: int(x.stem.split('_')[2]))
            return ckpt_files[-1]
        else:
            return None
    
    def _validate_model(self):
        """验证模型是否正确加载"""
        print(f"🧪 验证模型功能...")
        
        # 创建测试数据
        batch_size = 1
        qpos = torch.randn(batch_size, STATE_DIM).to(self.device)
        image = torch.randn(batch_size, 1, 3, ACTConstants.IMAGE_HEIGHT, ACTConstants.IMAGE_WIDTH).to(self.device)
        
        # 测试推理
        with torch.no_grad():
            actions = self.model(qpos, image)
        
        # 验证输出维度
        expected_shape = (batch_size, CHUNK_SIZE, ACTION_DIM)
        if actions.shape != expected_shape:
            raise RuntimeError(f"模型输出维度错误: 期望{expected_shape}, 实际{actions.shape}")
        
        print(f"✅ 模型验证通过: 输出形状{actions.shape}")
    
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
            qpos_tensor, image_tensor = self._preprocess_input(qpos, image)
            
            # 推理
            with torch.no_grad():
                actions_tensor = self.model(qpos_tensor, image_tensor)
            
            # 后处理
            actions = self._postprocess_output(actions_tensor)
            
            # 计算耗时
            inference_time = (time.time() - start_time) * 1000  # ms
            
            # 更新统计
            self.inference_count += 1
            self.total_inference_time += inference_time
            
            return actions, inference_time
            
        except Exception as e:
            raise RuntimeError(f"推理失败: {e}")
    
    def _preprocess_input(self, qpos: np.ndarray, image: np.ndarray) -> Tuple[torch.Tensor, torch.Tensor]:
        """预处理输入数据"""
        # 验证输入
        if qpos.shape != (STATE_DIM,):
            raise ValueError(f"qpos维度错误: 期望({STATE_DIM},), 实际{qpos.shape}")
        
        expected_image_shape = (ACTConstants.IMAGE_HEIGHT, ACTConstants.IMAGE_WIDTH, 3)
        if image.shape != expected_image_shape:
            raise ValueError(f"image维度错误: 期望{expected_image_shape}, 实际{image.shape}")
        
        # 转换为张量
        qpos_tensor = torch.from_numpy(qpos).float().unsqueeze(0).to(self.device)  # (1, 8)
        
        # 图像预处理: (H, W, 3) -> (1, 1, 3, H, W) 并归一化
        image_normalized = image.astype(np.float32) / 255.0  # [0, 1]
        image_tensor = torch.from_numpy(image_normalized).permute(2, 0, 1).unsqueeze(0).unsqueeze(0).to(self.device)  # (1, 1, 3, H, W)
        
        return qpos_tensor, image_tensor
    
    def _postprocess_output(self, actions_tensor: torch.Tensor) -> np.ndarray:
        """后处理输出数据"""
        # 转换为numpy数组
        actions = actions_tensor.cpu().numpy()  # (1, 100, 8)
        
        # 移除batch维度
        actions = actions[0]  # (100, 8)
        
        # 验证输出
        expected_shape = (CHUNK_SIZE, ACTION_DIM)
        if actions.shape != expected_shape:
            raise RuntimeError(f"输出维度错误: 期望{expected_shape}, 实际{actions.shape}")
        
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
            "parameters_M": self._count_parameters() if self.is_loaded else 0.0
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
