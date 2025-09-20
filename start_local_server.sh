#!/bin/bash
# 本地测试版服务器启动脚本

set -e

echo "🚀 启动ACT分布式推理服务器 (本地测试版)"
echo "=========================================="

# 激活环境
source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate aloha

# 设置环境变量
export CUDA_VISIBLE_DEVICES=0
export PYTHONPATH=$PWD:$PYTHONPATH

# 检查模型路径
MODEL_PATH="/home/wujielin/CascadeProjects/data/act_training/models/checkpoints/franka_pick_place"
if [[ ! -d "$MODEL_PATH" ]]; then
    echo "❌ 模型路径不存在: $MODEL_PATH"
    exit 1
fi

echo "✅ 模型路径验证通过: $MODEL_PATH"

# 启动服务器
echo "🌐 启动AgentLace推理服务器..."
cd robot_server/communication
python3 act_server.py \
    --port 5555 \
    --model_path "$MODEL_PATH" \
    --log_level INFO

echo "✅ 服务器已停止"
