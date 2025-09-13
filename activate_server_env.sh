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
