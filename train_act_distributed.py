#!/usr/bin/env python3
"""
Franka ACT 分布式训练主脚本
基于 SERL 架构，集成原版 ACT 算法和 AgentLace 分布式框架

参考计划文档：
- 阶段2：分布式训练集成 (lines 363-403)
- 将 SERL 的 RL 训练改为 ACT 的模仿学习
- 使用 AgentLace 处理分布式通信
"""

import argparse
import logging
import time
import yaml
import numpy as np
import torch
import torch.nn as nn
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple

# 原版 ACT 算法导入
from act_algo.policy import ACTPolicy
from act_algo.utils import get_norm_stats, load_data
from act_algo.imitate_episodes import make_policy

# SERL 架构导入
from agentlace.trainer import TrainerServer, TrainerClient
from agentlace.data.data_store import DataStore

# 环境适配器导入
from robot_env.franka_act_env import FrankaACTEnv

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ACTDistributedTrainer:
    """ACT 分布式训练器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.act_config = config['act_params']
        self.model_config = config['model_params']
        self.training_config = config['training_params']
        
        # 初始化 ACT 策略网络
        self.policy = None
        self.optimizer = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        logger.info(f"ACT Distributed Trainer initialized on device: {self.device}")
    
    def create_policy(self) -> ACTPolicy:
        """创建 ACT 策略网络"""
        # 从原版 ACT 创建策略
        policy = make_policy(self.act_config, self.model_config)
        policy.to(self.device)
        
        # 创建优化器
        self.optimizer = torch.optim.AdamW(
            policy.parameters(),
            lr=self.act_config['lr'],
            weight_decay=1e-4
        )
        
        logger.info(f"ACT Policy created: {policy}")
        return policy
    
    def compute_act_loss(self, policy: ACTPolicy, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        """计算 ACT 损失函数"""
        # 原版 ACT 损失计算
        # 参考 act_algo/imitate_episodes.py 中的训练逻辑
        
        # 提取观测和动作
        qpos = batch['qpos']  # (B, T, 7)
        qvel = batch['qvel']  # (B, T, 7) 
        images = batch['images']  # (B, T, C, H, W)
        actions = batch['actions']  # (B, T, 8)
        
        # 前向传播
        policy_output = policy(qpos, qvel, images, actions)
        
        # 计算损失
        l1_loss = nn.functional.l1_loss(policy_output['actions'], actions)
        kl_loss = policy_output.get('kl_loss', 0.0)
        
        total_loss = l1_loss + self.act_config['kl_weight'] * kl_loss
        
        return {
            'total_loss': total_loss,
            'l1_loss': l1_loss,
            'kl_loss': kl_loss
        }
    
    def format_demonstration_data(self, raw_data: List[Dict]) -> Dict[str, torch.Tensor]:
        """将原始示教数据格式化为 ACT 训练格式"""
        # 从 SERL RL 格式转换为 ACT 模仿学习格式
        # 参考计划文档 lines 365-385
        
        batch_size = len(raw_data)
        chunk_size = self.act_config['chunk_size']
        
        # 初始化批次张量
        qpos_batch = []
        qvel_batch = []
        images_batch = []
        actions_batch = []
        
        for episode_data in raw_data:
            # ACT 需要序列数据
            obs_sequence = episode_data['observations']  # 观测序列
            action_sequence = episode_data['actions']    # 动作序列
            
            # 提取关节状态
            qpos_seq = torch.tensor([obs['qpos'] for obs in obs_sequence], dtype=torch.float32)
            qvel_seq = torch.tensor([obs['qvel'] for obs in obs_sequence], dtype=torch.float32)
            
            # 提取图像数据
            images_seq = torch.stack([
                torch.tensor(obs['images']['cam_high'], dtype=torch.float32).permute(2, 0, 1) 
                for obs in obs_sequence
            ])
            
            # 动作序列
            actions_seq = torch.tensor(action_sequence, dtype=torch.float32)
            
            qpos_batch.append(qpos_seq)
            qvel_batch.append(qvel_seq)
            images_batch.append(images_seq)
            actions_batch.append(actions_seq)
        
        # 转换为张量批次
        batch = {
            'qpos': torch.stack(qpos_batch),
            'qvel': torch.stack(qvel_batch),
            'images': torch.stack(images_batch),
            'actions': torch.stack(actions_batch)
        }
        
        return batch


class ACTTrainingServer:
    """ACT 训练服务器（GPU训练机）"""
    
    def __init__(self, config_path: str, server_ip: str = "0.0.0.0", server_port: int = 5555):
        # 加载配置
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.trainer = ACTDistributedTrainer(self.config)
        self.policy = self.trainer.create_policy()
        
        # 初始化 AgentLace 训练服务器
        self.server = TrainerServer(
            trainer=self.trainer,
            server_ip=server_ip,
            server_port=server_port
        )
        
        # 训练状态
        self.current_epoch = 0
        self.total_epochs = self.config['act_params']['num_epochs']
        
        logger.info(f"ACT Training Server initialized on {server_ip}:{server_port}")
    
    def run_training_loop(self):
        """运行 ACT 模仿学习训练循环"""
        logger.info("Starting ACT training loop...")
        
        for epoch in range(self.total_epochs):
            self.current_epoch = epoch
            
            # 从客户端获取示教数据
            raw_demonstrations = self.server.collect_demonstrations()
            
            if not raw_demonstrations:
                logger.warning(f"No demonstrations received in epoch {epoch}")
                continue
            
            # 格式化为 ACT 训练数据
            batch = self.trainer.format_demonstration_data(raw_demonstrations)
            
            # 移动到GPU
            batch = {k: v.to(self.trainer.device) for k, v in batch.items()}
            
            # 前向传播和损失计算
            losses = self.trainer.compute_act_loss(self.policy, batch)
            
            # 反向传播
            self.trainer.optimizer.zero_grad()
            losses['total_loss'].backward()
            self.trainer.optimizer.step()
            
            # 发布更新的策略到客户端
            self.server.publish_policy(self.policy)
            
            # 日志记录
            if epoch % 10 == 0:
                logger.info(f"Epoch {epoch}/{self.total_epochs}: "
                          f"Total Loss: {losses['total_loss']:.4f}, "
                          f"L1 Loss: {losses['l1_loss']:.4f}, "
                          f"KL Loss: {losses['kl_loss']:.4f}")
            
            # 保存检查点
            if epoch % 100 == 0:
                self.save_checkpoint(epoch)
        
        logger.info("Training completed!")
    
    def save_checkpoint(self, epoch: int):
        """保存训练检查点"""
        checkpoint = {
            'epoch': epoch,
            'policy_state_dict': self.policy.state_dict(),
            'optimizer_state_dict': self.trainer.optimizer.state_dict(),
            'config': self.config
        }
        
        checkpoint_path = f"checkpoints/act_epoch_{epoch}.pth"
        torch.save(checkpoint, checkpoint_path)
        logger.info(f"Checkpoint saved: {checkpoint_path}")


class ACTTrainingClient:
    """ACT 训练客户端（Franka控制机）"""
    
    def __init__(self, config_path: str, server_ip: str, server_port: int = 5555, 
                 flask_server_url: str = "http://127.0.0.1:5000"):
        # 加载配置
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # 初始化环境
        self.env = FrankaACTEnv(
            server_url=flask_server_url,
            camera_names=self.config['model_params']['camera_names']
        )
        
        # 初始化 AgentLace 训练客户端
        self.client = TrainerClient(
            server_ip=server_ip,
            server_port=server_port
        )
        
        # 当前策略（从服务器接收）
        self.current_policy = None
        
        logger.info(f"ACT Training Client connected to server {server_ip}:{server_port}")
        logger.info(f"Flask server URL: {flask_server_url}")
    
    def collect_demonstrations(self, num_episodes: int = 10):
        """收集示教数据"""
        demonstrations = []
        
        for episode in range(num_episodes):
            logger.info(f"Collecting episode {episode + 1}/{num_episodes}")
            
            # 重置环境
            obs = self.env.reset()
            episode_data = {
                'observations': [obs],
                'actions': [],
                'episode_id': episode
            }
            
            done = False
            step = 0
            
            while not done and step < 500:  # 最大步数限制
                # 如果有当前策略，使用策略生成动作；否则使用随机动作
                if self.current_policy is not None:
                    action = self.get_policy_action(obs)
                else:
                    action = self.env.action_space.sample()
                
                # 执行动作
                next_obs, reward, done, info = self.env.step(action)
                
                # 记录数据
                episode_data['observations'].append(next_obs)
                episode_data['actions'].append(action.tolist())
                
                obs = next_obs
                step += 1
            
            demonstrations.append(episode_data)
            logger.info(f"Episode {episode + 1} completed with {step} steps")
        
        return demonstrations
    
    def get_policy_action(self, obs: Dict[str, Any]) -> np.ndarray:
        """使用当前策略生成动作"""
        if self.current_policy is None:
            return self.env.action_space.sample()
        
        # 转换观测格式
        qpos = torch.tensor(obs['qpos'], dtype=torch.float32).unsqueeze(0)
        qvel = torch.tensor(obs['qvel'], dtype=torch.float32).unsqueeze(0)
        
        # 处理图像
        images = []
        for cam_name in self.config['model_params']['camera_names']:
            img = torch.tensor(obs['images'][cam_name], dtype=torch.float32)
            img = img.permute(2, 0, 1).unsqueeze(0)  # (1, C, H, W)
            images.append(img)
        images = torch.cat(images, dim=0).unsqueeze(0)  # (1, num_cams*C, H, W)
        
        # 策略推理
        with torch.no_grad():
            action = self.current_policy.forward(qpos, qvel, images)
        
        return action.cpu().numpy().flatten()
    
    def run_data_collection(self):
        """运行数据收集循环"""
        logger.info("Starting data collection...")
        
        while True:
            try:
                # 接收更新的策略
                updated_policy = self.client.receive_policy()
                if updated_policy is not None:
                    self.current_policy = updated_policy
                    logger.info("Received policy update from server")
                
                # 收集示教数据
                demonstrations = self.collect_demonstrations(num_episodes=5)
                
                # 发送到服务器
                self.client.send_demonstrations(demonstrations)
                logger.info(f"Sent {len(demonstrations)} demonstrations to server")
                
                time.sleep(1)  # 短暂休息
                
            except KeyboardInterrupt:
                logger.info("Data collection stopped by user")
                break
            except Exception as e:
                logger.error(f"Error in data collection: {e}")
                time.sleep(5)  # 错误后等待重试


def main():
    parser = argparse.ArgumentParser(description='Franka ACT 分布式训练')
    parser.add_argument('--server', action='store_true', help='运行训练服务器模式')
    parser.add_argument('--client', action='store_true', help='运行训练客户端模式') 
    parser.add_argument('--config', type=str, default='configs/act_config.yaml', help='配置文件路径')
    parser.add_argument('--server_ip', type=str, default='127.0.0.1', help='服务器IP地址')
    parser.add_argument('--server_port', type=int, default=5555, help='服务器端口')
    parser.add_argument('--flask_url', type=str, default='http://127.0.0.1:5000', help='Flask服务器URL')
    
    args = parser.parse_args()
    
    if args.server:
        # 运行训练服务器
        logger.info("Starting ACT Training Server...")
        server = ACTTrainingServer(
            config_path=args.config,
            server_ip=args.server_ip,
            server_port=args.server_port
        )
        server.run_training_loop()
        
    elif args.client:
        # 运行训练客户端
        logger.info("Starting ACT Training Client...")
        client = ACTTrainingClient(
            config_path=args.config,
            server_ip=args.server_ip,
            server_port=args.server_port,
            flask_server_url=args.flask_url
        )
        client.run_data_collection()
        
    else:
        print("请指定 --server 或 --client 模式")
        parser.print_help()


if __name__ == "__main__":
    main()
