"""
DETR main build functions for ACT
Modified from original DETR repository
"""

import torch
import torch.nn as nn
from .models.detr_vae import build_encoder, build_decoder, build


def build_ACT_model_and_optimizer(args_override):
    """构建ACT模型和优化器"""
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # 默认参数
    default_args = {
        'hidden_dim': 512,
        'lr_backbone': 1e-5,
        'backbone': 'resnet18',
        'enc_layers': 4,
        'dec_layers': 7,
        'nheads': 8,
        'num_queries': 100,
        'dim_feedforward': 3200,
        'dropout': 0.1,
        'pre_norm': False,
        'kl_weight': 10.0,
        'lr': 1e-5,
        'weight_decay': 1e-4,
    }
    
    # 合并参数
    args = {**default_args, **args_override}
    
    # 构建模型
    model = build(args)
    model.to(device)
    
    # 构建优化器
    param_dicts = [
        {"params": [p for n, p in model.named_parameters() if "backbone" not in n and p.requires_grad]},
        {
            "params": [p for n, p in model.named_parameters() if "backbone" in n and p.requires_grad],
            "lr": args['lr_backbone'],
        },
    ]
    optimizer = torch.optim.AdamW(param_dicts, lr=args['lr'], weight_decay=args['weight_decay'])
    
    return model, optimizer


def build_CNNMLP_model_and_optimizer(args_override):
    """构建CNN+MLP模型和优化器（备用方案）"""
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # 简化的CNN+MLP实现
    class CNNMLP(nn.Module):
        def __init__(self, args):
            super().__init__()
            self.backbone = nn.Sequential(
                nn.Conv2d(3, 64, 7, 2, 3),
                nn.ReLU(),
                nn.AdaptiveAvgPool2d((1, 1)),
                nn.Flatten(),
            )
            self.mlp = nn.Sequential(
                nn.Linear(64 + 7, args['hidden_dim']),  # 64 (image) + 7 (qpos)
                nn.ReLU(),
                nn.Linear(args['hidden_dim'], args['chunk_size'] * 8),  # 8D action space
            )
            self.chunk_size = args.get('chunk_size', 100)
            
        def forward(self, qpos, image, actions=None, is_pad=None):
            # 处理图像
            if isinstance(image, dict):
                # 多相机情况，取第一个相机
                img_features = []
                for cam_name in image.keys():
                    img = image[cam_name]
                    if len(img.shape) == 3:
                        img = img.unsqueeze(0)
                    feat = self.backbone(img)
                    img_features.append(feat)
                img_feature = torch.cat(img_features, dim=-1)
            else:
                if len(image.shape) == 3:
                    image = image.unsqueeze(0)
                img_feature = self.backbone(image)
            
            # 合并qpos和图像特征
            combined = torch.cat([qpos, img_feature], dim=-1)
            output = self.mlp(combined)
            
            # 重塑为动作序列
            batch_size = qpos.shape[0]
            action_pred = output.view(batch_size, self.chunk_size, 8)
            
            return {
                'actions': action_pred,
                'kl_loss': torch.tensor(0.0, device=qpos.device)  # 占位符
            }
    
    # 默认参数
    default_args = {
        'hidden_dim': 512,
        'chunk_size': 100,
        'lr': 1e-4,
        'weight_decay': 1e-4,
    }
    
    args = {**default_args, **args_override}
    
    model = CNNMLP(args)
    model.to(device)
    
    optimizer = torch.optim.AdamW(model.parameters(), lr=args['lr'], weight_decay=args['weight_decay'])
    
    return model, optimizer
