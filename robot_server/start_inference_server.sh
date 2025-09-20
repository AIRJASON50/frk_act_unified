#!/bin/bash

#===============================================================================
# ACT分布式推理服务器启动脚本 - 基于AgentLace协议
#
# 使用方法:
# ./start_inference_server.sh                    # 默认配置启动
# ./start_inference_server.sh 5556              # 指定端口
# ./start_inference_server.sh 5555 /path/model  # 指定端口和模型路径
#===============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 参数解析
PORT=${1:-5555}
MODEL_PATH=${2:-"/home/wujielin/CascadeProjects/data/act_training/checkpoints/franka_pick_place"}
LOG_LEVEL=${3:-"INFO"}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   ACT分布式推理服务器 v2.0${NC}"
echo -e "${BLUE}   协议: AgentLace + RTX 4090优化${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 环境检查和激活
echo -e "${BLUE}🔧 环境检查...${NC}"

# 检查conda环境
if [[ "$CONDA_DEFAULT_ENV" != "aloha" ]]; then
    echo -e "${YELLOW}⚠️ 检测到未激活aloha环境，正在激活...${NC}"
    source ~/miniconda3/etc/profile.d/conda.sh
    conda activate aloha
    if [[ "$CONDA_DEFAULT_ENV" != "aloha" ]]; then
        echo -e "${RED}❌ 无法激活aloha环境，请手动激活后重试${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}✅ Conda环境: $CONDA_DEFAULT_ENV${NC}"

# 设置AgentLace环境变量
export ACT_SERVER_IP="10.16.49.124"
export ACT_SERVER_PORT="$PORT"
export CUDA_VISIBLE_DEVICES="0"
export PYTORCH_CUDA_ALLOC_CONF="max_split_size_mb:1024"

# 检查GPU状态
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}🚀 GPU状态:${NC}"
    GPU_INFO=$(nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader,nounits | head -1)
    echo "   $GPU_INFO"
    
    # 检查GPU内存
    FREE_MEM=$(echo $GPU_INFO | cut -d',' -f3 | tr -d ' ')
    if [[ $FREE_MEM -lt 8000 ]]; then
        echo -e "${YELLOW}⚠️ GPU可用内存较少: ${FREE_MEM}MB < 8GB${NC}"
    fi
else
    echo -e "${YELLOW}⚠️ 未检测到NVIDIA GPU，将使用CPU推理${NC}"
fi

echo ""
echo -e "${BLUE}📋 服务器配置:${NC}"
echo "   协议: AgentLace v0.0.2"
echo "   服务器IP: $ACT_SERVER_IP"
echo "   监听端口: $PORT (REQ-REP)"
echo "   广播端口: $((PORT + 1)) (PUB-SUB)"
echo "   模型路径: $MODEL_PATH"
echo "   日志级别: $LOG_LEVEL"
echo "   GPU优化: RTX 4090专用配置"
echo ""

# 检查模型文件
echo -e "${BLUE}🔍 模型文件检查...${NC}"
if [[ ! -d "$MODEL_PATH" ]]; then
    echo -e "${RED}❌ 模型目录不存在: $MODEL_PATH${NC}"
    echo "请确认模型已训练完成"
    exit 1
fi

MODEL_FILES=$(find "$MODEL_PATH" -name "policy_epoch_*.ckpt" | wc -l)
if [[ $MODEL_FILES -eq 0 ]]; then
    echo -e "${RED}❌ 未找到模型文件${NC}"
    exit 1
fi

LATEST_MODEL=$(find "$MODEL_PATH" -name "policy_epoch_*.ckpt" | sort -V | tail -1)
echo -e "${GREEN}✅ 找到 $MODEL_FILES 个模型文件${NC}"
echo -e "${GREEN}✅ 使用最新模型: $(basename "$LATEST_MODEL")${NC}"

# 检查AgentLace依赖
echo ""
echo -e "${BLUE}🔗 AgentLace依赖检查...${NC}"
python -c "
try:
    import zmq
    print('✅ ZeroMQ版本:', zmq.zmq_version())
    
    import sys
    sys.path.append('communication/agentlace')
    from agentlace.trainer import TrainerServer
    print('✅ AgentLace协议就绪')
    
    import torch
    print('✅ PyTorch版本:', torch.__version__)
    print('✅ CUDA可用:', torch.cuda.is_available())
    
except ImportError as e:
    print('❌ 依赖检查失败:', e)
    exit(1)
"

if [[ $? -ne 0 ]]; then
    echo -e "${RED}❌ 依赖检查失败，请运行 communication/server_setup.sh${NC}"
    exit 1
fi

# 创建日志目录
mkdir -p logs

echo ""
echo -e "${BLUE}🚀 启动ACT分布式推理服务器...${NC}"
echo -e "${YELLOW}使用 Ctrl+C 停止服务器${NC}"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 启动服务器
cd communication/
python act_server.py \
    --port "$PORT" \
    --model_path "$MODEL_PATH" \
    --log_level "$LOG_LEVEL" \
    2>&1 | tee ../logs/inference_server_$(date +%Y%m%d_%H%M%S).log

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${BLUE}    ACT推理服务器已停止${NC}"
echo -e "${BLUE}========================================${NC}"
