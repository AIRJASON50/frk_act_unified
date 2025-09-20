#!/usr/bin/env python3
"""
ACT Policy Classes - Franka Single Arm Robot Learning

================================================================================
FILE DESCRIPTION:
本文件定义了ACT训练的策略类，包括ACTPolicy和CNNMLPPolicy。
这些类封装了模型的前向传播、损失计算和优化器管理。

MAIN FUNCTIONALITY:
1. 策略模型的高级封装
2. 训练时的损失计算
3. 推理时的动作预测
4. 优化器管理和更新

KEY COMPONENTS:
- ACTPolicy: 基于Transformer+VAE的主要策略
- CNNMLPPolicy: 基于CNN+MLP的基线策略
- 损失函数: L1重建损失 + KL散度损失
- 图像预处理: ImageNet归一化

TRAINING WORKFLOW:
输入: qpos(8维) + RGB图像 + 真实动作序列
处理: 前向传播 → 损失计算 → 梯度更新
输出: 损失字典 + 预测动作

INFERENCE WORKFLOW:
输入: qpos(8维) + RGB图像
处理: 前向传播 → 动作预测
输出: 100步动作序列

INPUT SPECIFICATION:
- qpos: (batch, 8) - Franka关节状态
- image: (batch, cameras, 3, H, W) - RGB图像
- actions: (batch, 100, 8) - 动作序列（训练时）
- is_pad: (batch, 100) - 填充掩码（训练时）

OUTPUT SPECIFICATION:
训练时: 损失字典 {"l1": 重建损失, "kl": KL散度, "loss": 总损失}
推理时: (batch, 100, 8) 动作预测

FRANKA ADAPTATION:
✅ 8维状态/动作空间
✅ 100步动作chunk
✅ 单相机图像输入
✅ ImageNet预训练权重利用

AUTHORS: Tony Z. Zhao (Original ACT), Franka Team (Single Arm Adaptation)
VERSION: v1.0 - Franka Single Arm Specialized
LAST UPDATED: 2025-09-20
================================================================================
"""

import torch.nn as nn
from torch.nn import functional as F
import torchvision.transforms as transforms

from .detr.main import build_ACT_model_and_optimizer, build_CNNMLP_model_and_optimizer
import IPython
e = IPython.embed

class ACTPolicy(nn.Module):
    def __init__(self, args_override):
        super().__init__()
        model, optimizer = build_ACT_model_and_optimizer(args_override)
        self.model = model # CVAE decoder
        self.optimizer = optimizer
        self.kl_weight = args_override['kl_weight']
        print(f'KL Weight {self.kl_weight}')

    def __call__(self, qpos, image, actions=None, is_pad=None):
        env_state = None
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
        image = normalize(image)
        if actions is not None: # training time
            actions = actions[:, :self.model.num_queries]
            is_pad = is_pad[:, :self.model.num_queries]

            a_hat, is_pad_hat, (mu, logvar) = self.model(qpos, image, env_state, actions, is_pad)
            total_kld, dim_wise_kld, mean_kld = kl_divergence(mu, logvar)
            loss_dict = dict()
            all_l1 = F.l1_loss(actions, a_hat, reduction='none')
            l1 = (all_l1 * ~is_pad.unsqueeze(-1)).mean()
            loss_dict['l1'] = l1
            loss_dict['kl'] = total_kld[0]
            loss_dict['loss'] = loss_dict['l1'] + loss_dict['kl'] * self.kl_weight
            return loss_dict
        else: # inference time
            a_hat, _, (_, _) = self.model(qpos, image, env_state) # no action, sample from prior
            return a_hat

    def configure_optimizers(self):
        return self.optimizer


class CNNMLPPolicy(nn.Module):
    def __init__(self, args_override):
        super().__init__()
        model, optimizer = build_CNNMLP_model_and_optimizer(args_override)
        self.model = model # decoder
        self.optimizer = optimizer

    def __call__(self, qpos, image, actions=None, is_pad=None):
        env_state = None # TODO
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
        image = normalize(image)
        if actions is not None: # training time
            actions = actions[:, 0]
            a_hat = self.model(qpos, image, env_state, actions)
            mse = F.mse_loss(actions, a_hat)
            loss_dict = dict()
            loss_dict['mse'] = mse
            loss_dict['loss'] = loss_dict['mse']
            return loss_dict
        else: # inference time
            a_hat = self.model(qpos, image, env_state) # no action, sample from prior
            return a_hat

    def configure_optimizers(self):
        return self.optimizer

def kl_divergence(mu, logvar):
    batch_size = mu.size(0)
    assert batch_size != 0
    if mu.data.ndimension() == 4:
        mu = mu.view(mu.size(0), mu.size(1))
    if logvar.data.ndimension() == 4:
        logvar = logvar.view(logvar.size(0), logvar.size(1))

    klds = -0.5 * (1 + logvar - mu.pow(2) - logvar.exp())
    total_kld = klds.sum(1).mean(0, True)
    dimension_wise_kld = klds.mean(0)
    mean_kld = klds.mean(1).mean(0, True)

    return total_kld, dimension_wise_kld, mean_kld
