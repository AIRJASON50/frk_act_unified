#!/bin/bash
# 本地测试版服务器启动脚本

set -e

echo "🚀 启动ACT分布式推理服务器 (本地测试版)"
echo "=========================================="

# 激活环境
source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate aloha

# 智能GPU选择逻辑：优先使用GPU 2、3，选择占用最小的
echo "🔍 检查GPU使用情况..."
GPU_INFO=$(nvidia-smi --query-gpu=index,memory.used --format=csv,noheader,nounits)
echo "$GPU_INFO"

# 获取GPU数量
GPU_COUNT=$(echo "$GPU_INFO" | wc -l)
echo "检测到 $GPU_COUNT 个GPU"

# 解析GPU 2和3的内存使用情况
GPU2_MEM=$(echo "$GPU_INFO" | awk 'NR==3 {print $2}' | tr -d ',')
GPU3_MEM=$(echo "$GPU_INFO" | awk 'NR==4 {print $2}' | tr -d ',')

if [[ $GPU_COUNT -ge 4 ]]; then
    # 如果有4个GPU，比较GPU 2和3的内存使用
    echo "GPU 2内存使用: ${GPU2_MEM}MB, GPU 3内存使用: ${GPU3_MEM}MB"
    
    if [[ $GPU2_MEM -le $GPU3_MEM ]]; then
        GPU_ID=2
        echo "✅ 选择GPU 2 (内存占用较少: ${GPU2_MEM}MB)"
    else
        GPU_ID=3
        echo "✅ 选择GPU 3 (内存占用较少: ${GPU3_MEM}MB)"
    fi
elif [[ $GPU_COUNT -ge 3 ]]; then
    # 如果只有3个GPU，使用GPU 2
    GPU_ID=2
    echo "✅ 选择GPU 2 (内存占用: ${GPU2_MEM}MB)"
else
    # 如果GPU数量不足，使用GPU 0
    GPU_ID=0
    echo "⚠️  GPU数量不足，选择GPU 0"
fi

# 设置环境变量
export CUDA_VISIBLE_DEVICES=$GPU_ID
export PYTHONPATH=$PWD:$PYTHONPATH

echo "🖥️  使用GPU: $GPU_ID"

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
