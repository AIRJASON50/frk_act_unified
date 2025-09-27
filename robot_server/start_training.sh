#!/bin/bash

#===============================================================================
# Franka ACT Training Launcher Script
#
# MAIN FUNCTIONALITY:
# 启动Franka单臂ACT模型训练的便捷脚本，使用标准ACT架构
#
# USAGE:
# ./start_training.sh [task_name] [num_epochs] [batch_size]
#
# EXAMPLES:
# ./start_training.sh                              # 默认参数训练
# ./start_training.sh franka_pick_place 1000       # 指定任务和epoch数
# ./start_training.sh franka_pick_place 2000 4     # 完整参数
#
# PATHS:
# - 数据集: /home/wujielin/CascadeProjects/data/act_training/datasets/act_data (笛卡尔空间数据)
# - 模型: /home/wujielin/CascadeProjects/data/act_training/checkpoints
# - 日志: /home/wujielin/CascadeProjects/data/act_training/logs
#===============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default parameters (完整训练配置)
TASK_NAME=${1:-"act_data"}  # 使用新的笛卡尔空间数据集
NUM_EPOCHS=${2:-15000}  # 完整训练epoch数
BATCH_SIZE=${3:-64}     # 降低batch size适应笛卡尔数据

# ACT model parameters (based on memory notes)
HIDDEN_DIM=512
CHUNK_SIZE=100
KL_WEIGHT=10
DIM_FEEDFORWARD=3200
LR=5e-5

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    Franka ACT Training Launcher${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${GREEN}Task:${NC} $TASK_NAME"
echo -e "${GREEN}Epochs:${NC} $NUM_EPOCHS"
echo -e "${GREEN}Batch Size:${NC} $BATCH_SIZE"
echo ""
echo -e "${YELLOW}Model Configuration:${NC}"
echo "  Hidden Dim: $HIDDEN_DIM"
echo "  Chunk Size: $CHUNK_SIZE"
echo "  KL Weight: $KL_WEIGHT"
echo "  Feedforward Dim: $DIM_FEEDFORWARD"
echo "  Learning Rate: $LR"
echo ""

# Check if CUDA is available
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}GPU Status:${NC}"
    nvidia-smi --query-gpu=name,memory.free --format=csv,noheader
    echo ""
fi

# 设置数据路径（统一使用/data目录）
DATA_BASE="/home/wujielin/CascadeProjects/data/act_training"
CKPT_DIR="$DATA_BASE/checkpoints/$TASK_NAME"
LOG_DIR="$DATA_BASE/logs"

# Create necessary directories
echo -e "${BLUE}Creating directories...${NC}"
mkdir -p "$CKPT_DIR"
mkdir -p "$LOG_DIR"
echo "  Checkpoints: $CKPT_DIR"
echo "  Logs: $LOG_DIR"
echo ""

# Launch training using standard ACT module
echo -e "${BLUE}Starting training with standard ACT...${NC}"
echo "Command:"
echo "python -m act_algo_train.imitate_episodes \\"
echo "    --task_name $TASK_NAME \\"
echo "    --ckpt_dir $CKPT_DIR \\"
echo "    --policy_class ACT \\"
echo "    --batch_size $BATCH_SIZE \\"
echo "    --num_epochs $NUM_EPOCHS \\"
echo "    --lr $LR \\"
echo "    --kl_weight $KL_WEIGHT \\"
echo "    --chunk_size $CHUNK_SIZE \\"
echo "    --hidden_dim $HIDDEN_DIM \\"
echo "    --dim_feedforward $DIM_FEEDFORWARD \\"
echo "    --temporal_agg \\"
echo "    --seed 0"
echo ""
echo -e "${BLUE}========================================${NC}"

# Activate conda environment and execute training
source ~/miniconda3/etc/profile.d/conda.sh
conda activate aloha

# Execute training using the standard ACT module
python -m act_algo_train.imitate_episodes \
    --task_name "$TASK_NAME" \
    --ckpt_dir "$CKPT_DIR" \
    --policy_class ACT \
    --batch_size $BATCH_SIZE \
    --num_epochs $NUM_EPOCHS \
    --lr $LR \
    --kl_weight $KL_WEIGHT \
    --chunk_size $CHUNK_SIZE \
    --hidden_dim $HIDDEN_DIM \
    --dim_feedforward $DIM_FEEDFORWARD \
    --temporal_agg \
    --seed 0

# Check exit status
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}    Training Completed Successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo "Models saved in: $CKPT_DIR"
    echo "Logs saved in: $LOG_DIR"
    echo ""
    echo "To view latest model:"
    echo "  ls -la $CKPT_DIR/policy_epoch_*.ckpt"
    echo ""
    echo "To view training data stats:"
    echo "  cat $CKPT_DIR/dataset_stats.pkl"
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}    Training Failed!${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "Check logs for details:"
    echo "  ls -la logs/$TASK_NAME/"
fi
