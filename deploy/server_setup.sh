#!/bin/bash
# deploy/server_setup.sh
# Franka ACT 服务器端环境配置脚本
# 适用于GPU训练服务器，无需ROS环境

set -e

echo "========================================="
echo "Franka ACT 服务器端环境配置"
echo "========================================="

# 检查conda环境
if ! command -v conda &> /dev/null; then
    echo "❌ Conda未安装，请先安装Miniconda或Anaconda"
    exit 1
fi

# 激活或创建aloha环境
echo "🔧 配置Python环境..."
source $HOME/miniconda3/etc/profile.d/conda.sh
if ! conda env list | grep -q "aloha"; then
    echo "创建aloha conda环境..."
    conda create -n aloha python=3.9 -y
fi

conda activate aloha

# 服务器端依赖安装
echo "📦 安装服务器端Python依赖..."
pip install --upgrade pip

# 深度学习框架
pip install torch==2.0.0 torchvision==0.15.0 --index-url https://download.pytorch.org/whl/cu118

# 基础科学计算包
pip install numpy==1.24.3 scipy matplotlib seaborn
pip install gymnasium==1.1.1 opencv-python-headless
pip install pillow>=8.0.0 imageio

# Web框架和网络通信
pip install flask==2.3.2 requests==2.31.0
pip install websockets aiohttp

# 配置文件和数据处理
pip install pyyaml>=6.0 h5py wandb
pip install tqdm ipython

# 模型相关依赖
pip install transformers==4.30.0
pip install einops timm

echo "🧪 验证服务器端环境..."
python3 -c "
import torch
print(f'✓ PyTorch版本: {torch.__version__}')
print(f'✓ CUDA可用: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'✓ GPU数量: {torch.cuda.device_count()}')
    for i in range(torch.cuda.device_count()):
        print(f'  - GPU{i}: {torch.cuda.get_device_name(i)}')

import numpy as np
print(f'✓ NumPy版本: {np.__version__}')

import yaml, flask
print('✓ 配置和Web框架导入成功')
"

# 创建服务器端环境激活脚本
echo "📄 创建服务器端环境脚本..."
cat > activate_server_env.sh << 'EOF'
#!/bin/bash
# 服务器端环境激活脚本

source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate aloha

export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-"0,1,2,3"}
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512

export FRANKA_ACT_ROOT=$(pwd)
export PYTHONPATH=$FRANKA_ACT_ROOT:$FRANKA_ACT_ROOT/act_algo:$PYTHONPATH

echo "🖥️  服务器端环境已激活"
echo "   - Conda环境: aloha"
echo "   - CUDA设备: $CUDA_VISIBLE_DEVICES"
echo "   - 项目根目录: $FRANKA_ACT_ROOT"
echo "   - PyTorch版本: $(python -c 'import torch; print(torch.__version__)')"
EOF

chmod +x activate_server_env.sh

# 创建服务器启动脚本
echo "🚀 配置服务器启动脚本..."
cat > start_training_server.sh << 'EOF'
#!/bin/bash
# 服务器端训练启动脚本

set -e

# 参数设置
PORT=${1:-5555}
GPUS=${2:-"0,1,2,3"}
CKPT_DIR=${3:-"models/franka_act"}
CONFIG=${4:-"configs/act_config.yaml"}

echo "========================================="
echo "启动 Franka ACT 训练服务器"
echo "端口: $PORT"
echo "GPU设备: $GPUS"
echo "模型目录: $CKPT_DIR"
echo "配置文件: $CONFIG"
echo "========================================="

# 激活环境
source ./activate_server_env.sh

# 设置GPU
export CUDA_VISIBLE_DEVICES=$GPUS

# 创建输出目录
mkdir -p $CKPT_DIR
mkdir -p logs

# 启动训练服务器
echo "🚀 启动训练服务器..."
python3 train_act_distributed.py \
    --mode server \
    --port $PORT \
    --config $CONFIG \
    --ckpt_dir $CKPT_DIR \
    --log_dir logs \
    2>&1 | tee logs/server_$(date +%Y%m%d_%H%M%S).log

EOF

chmod +x start_training_server.sh

echo "✅ 服务器端环境配置完成！"
echo ""
echo "使用方法："
echo "1. 激活环境: source ./activate_server_env.sh"
echo "2. 启动训练: ./start_training_server.sh [端口] [GPU] [模型目录]"
echo ""
