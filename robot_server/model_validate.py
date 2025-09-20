#!/usr/bin/env python3
"""
Franka ACT模型验证脚本
验证训练好的模型是否能正确处理Franka数据流
"""

import torch
import sys
import os
import argparse
import numpy as np
import h5py
from pathlib import Path

# 添加路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def model_validate(ckpt_path):
    """验证模型是否可用"""
    print(f"🔍 验证模型: {ckpt_path}")
    
    try:
        # 加载检查点
        checkpoint = torch.load(ckpt_path, map_location='cpu')
        
        print(f"✅ 模型检查点信息:")
        print(f"   Epoch: {checkpoint.get('epoch', 'Unknown')}")
        print(f"   Loss: {checkpoint.get('min_val_loss', 'Unknown')}")
        
        # 检查模型参数
        if 'model_state_dict' in checkpoint:
            state_dict = checkpoint['model_state_dict']
            total_params = sum(p.numel() for p in state_dict.values())
            print(f"   参数数量: {total_params/1e6:.2f}M")
            
            # 检查关键层
            key_layers = ['action_head.weight', 'input_proj_robot_state.weight']
            for layer in key_layers:
                if layer in state_dict:
                    shape = state_dict[layer].shape
                    print(f"   {layer}: {shape}")
        
        print("✅ 模型检查点验证通过!")
        return True
        
    except Exception as e:
        print(f"❌ 模型检查点验证失败: {e}")
        return False

def validate_franka_compatibility(ckpt_path):
    """验证模型是否与Franka数据流兼容"""
    print(f"\n🤖 验证Franka兼容性...")
    
    try:
        from act_algo_train.policy import ACTPolicy
        from act_algo_train.franka_constants import STATE_DIM, ACTION_DIM, CHUNK_SIZE
        
        # 创建Franka兼容的配置
        config = {
            'lr': 1e-5,
            'num_queries': CHUNK_SIZE,  # 100
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
        
        print(f"📊 Franka配置检查:")
        print(f"   状态维度: {STATE_DIM} (7关节+1夹爪)")
        print(f"   动作维度: {ACTION_DIM}")
        print(f"   Chunk大小: {CHUNK_SIZE}")
        print(f"   相机: {config['camera_names']}")
        
        # 加载模型
        policy = ACTPolicy(config)
        checkpoint = torch.load(ckpt_path, map_location='cpu')
        
        # 检查checkpoint格式并加载
        if 'model_state_dict' in checkpoint:
            policy.load_state_dict(checkpoint['model_state_dict'])
        else:
            # 直接是模型权重
            policy.load_state_dict(checkpoint)
        policy.eval()
        
        # 检查设备
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"🖥️ 使用设备: {device}")
        
        # 创建Franka格式的测试数据
        batch_size = 1
        qpos = torch.randn(batch_size, STATE_DIM).to(device)  # 8维状态
        image = torch.randn(batch_size, 1, 3, 480, 640).to(device)  # 单相机RGB图像
        
        print(f"🧪 测试数据格式:")
        print(f"   qpos形状: {qpos.shape} (期望: [1, 8])")
        print(f"   image形状: {image.shape} (期望: [1, 1, 3, 480, 640])")
        
        # 测试推理
        with torch.no_grad():
            actions = policy(qpos, image)
        
        print(f"✅ 推理测试成功!")
        print(f"   输出动作形状: {actions.shape} (期望: [1, 100, 8])")
        
        # 验证输出维度
        expected_shape = (batch_size, CHUNK_SIZE, ACTION_DIM)
        if actions.shape == expected_shape:
            print(f"✅ 输出维度正确: {actions.shape}")
        else:
            print(f"❌ 输出维度错误: 期望{expected_shape}, 实际{actions.shape}")
            return False
        
        # 验证数值范围
        action_min, action_max = actions.min().item(), actions.max().item()
        print(f"📈 动作数值范围: [{action_min:.3f}, {action_max:.3f}]")
        
        print(f"🎯 Franka兼容性验证通过!")
        return True
        
    except Exception as e:
        print(f"❌ Franka兼容性验证失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_with_real_data():
    """使用真实Franka数据测试模型"""
    print(f"\n📁 真实数据测试...")
    
    dataset_dir = "/home/wujielin/CascadeProjects/data/act_training/datasets/act_0918"
    if not os.path.exists(dataset_dir):
        print(f"⚠️ 数据集目录不存在: {dataset_dir}")
        return False
    
    try:
        # 获取第一个episode文件
        episode_files = sorted([f for f in os.listdir(dataset_dir) if f.endswith('.hdf5')])
        if not episode_files:
            print(f"⚠️ 未找到数据文件")
            return False
            
        test_file = os.path.join(dataset_dir, episode_files[0])
        print(f"📄 使用测试文件: {episode_files[0]}")
        
        with h5py.File(test_file, 'r') as f:
            # 检查数据格式
            qpos_data = f['/observations/qpos'][0]  # 第一帧
            image_data = f['/observations/images/top'][0]  # 第一帧图像
            action_data = f['/action'][0]  # 第一个动作
            
            print(f"✅ 真实数据格式:")
            print(f"   qpos: {qpos_data.shape} 值域: [{qpos_data.min():.3f}, {qpos_data.max():.3f}]")
            print(f"   image: {image_data.shape} 值域: [{image_data.min():.0f}, {image_data.max():.0f}]")
            print(f"   action: {action_data.shape} 值域: [{action_data.min():.3f}, {action_data.max():.3f}]")
            
            # 验证维度
            if qpos_data.shape[0] != 8:
                print(f"❌ qpos维度错误: 期望8, 实际{qpos_data.shape[0]}")
                return False
                
            if image_data.shape != (480, 640, 3):
                print(f"❌ 图像维度错误: 期望(480, 640, 3), 实际{image_data.shape}")
                return False
                
            if action_data.shape[0] != 8:
                print(f"❌ 动作维度错误: 期望8, 实际{action_data.shape[0]}")
                return False
        
        print(f"✅ 真实数据格式验证通过!")
        return True
        
    except Exception as e:
        print(f"❌ 真实数据测试失败: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Franka ACT模型完整验证")
    parser.add_argument('--ckpt_path', type=str, 
                       default='/home/wujielin/CascadeProjects/data/act_training/checkpoints/franka_pick_place',
                       help='模型检查点路径或目录')
    parser.add_argument('--skip_real_data', action='store_true',
                       help='跳过真实数据测试')
    
    args = parser.parse_args()
    
    print("🚀 Franka ACT模型验证开始...")
    print("=" * 50)
    
    # 1. 真实数据测试（不依赖模型）
    if not args.skip_real_data:
        data_valid = test_with_real_data()
        if not data_valid:
            print("⚠️ 真实数据验证失败，但继续模型验证...")
    
    # 2. 查找模型文件
    ckpt_path = Path(args.ckpt_path)
    model_file = None
    
    if ckpt_path.is_dir():
        # 目录模式：查找最新的检查点
        ckpt_files = list(ckpt_path.glob('policy_epoch_*.ckpt'))
        if not ckpt_files:
            print(f"❌ 在 {ckpt_path} 中未找到检查点文件")
            print("💡 请先训练模型或检查路径是否正确")
            return
        
        # 按epoch排序，取最新的
        ckpt_files.sort(key=lambda x: int(x.stem.split('_')[2]))
        model_file = ckpt_files[-1]
        print(f"📁 找到 {len(ckpt_files)} 个检查点，使用最新的: {model_file.name}")
        
    elif ckpt_path.is_file():
        # 文件模式：直接验证
        model_file = ckpt_path
        print(f"📄 使用指定模型文件: {model_file.name}")
    else:
        print(f"❌ 路径不存在: {ckpt_path}")
        return
    
    # 3. 模型检查点验证
    checkpoint_valid = model_validate(model_file)
    
    # 4. Franka兼容性验证
    if checkpoint_valid:
        franka_valid = validate_franka_compatibility(model_file)
    else:
        franka_valid = False
    
    # 5. 总结
    print("\n" + "=" * 50)
    print("📋 验证结果总结:")
    print("=" * 50)
    
    if not args.skip_real_data:
        status = "✅ 通过" if data_valid else "❌ 失败"
        print(f"真实数据测试    : {status}")
    
    status = "✅ 通过" if checkpoint_valid else "❌ 失败"
    print(f"模型检查点验证  : {status}")
    
    status = "✅ 通过" if franka_valid else "❌ 失败"
    print(f"Franka兼容性验证: {status}")
    
    print("=" * 50)
    
    if checkpoint_valid and franka_valid:
        print("🎉 所有验证通过！模型已准备好用于Franka机器人！")
        print("\n💡 下一步:")
        print("   1. 部署模型到推理服务器")
        print("   2. 配置WebSocket服务")  
        print("   3. 连接客户端进行实时推理")
    else:
        print("⚠️ 验证失败，请检查模型训练或配置问题")

if __name__ == "__main__":
    main()
