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

