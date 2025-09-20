#!/bin/bash
# ACT分布式推理服务器配置脚本
# 基于agentlace协议的服务器端配置，充分利用RTX 4090 GPU

set -e

echo "========================================="
echo "ACT分布式推理服务器配置"
echo "版本: v2.0 - AgentLace协议"
echo "========================================="

# 读取配置文件
CONFIG_FILE="robot_config.yaml"
if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "❌ 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo "📋 读取配置文件: $CONFIG_FILE"

# 使用python解析YAML配置
python3 -c "
import yaml
import sys

try:
    with open('$CONFIG_FILE', 'r') as f:
        config = yaml.safe_load(f)
    
    # 网络配置
    network = config['network']
    print(f'export ACT_SERVER_IP=\"{network[\"server_ip\"]}\"')
    print(f'export ACT_SERVER_PORT=\"{network[\"server_port\"]}\"')
    print(f'export ACT_BROADCAST_PORT=\"{network[\"broadcast_port\"]}\"')
    print(f'export ACT_PROTOCOL_VERSION=\"{network[\"protocol_version\"]}\"')
    
    # GPU配置
    server_opt = config['server_optimization']
    print(f'export GPU_DEVICE=\"{server_opt[\"gpu_device\"]}\"')
    print(f'export MAX_BATCH_SIZE=\"{server_opt[\"max_batch_size\"]}\"')
    print(f'export INFERENCE_WORKERS=\"{server_opt[\"inference_workers\"]}\"')
    print(f'export MAX_GPU_MEMORY=\"{server_opt[\"max_gpu_memory\"]}\"')
    print(f'export PYTORCH_CUDA_ALLOC_CONF=\"{server_opt[\"pytorch_cuda_alloc_conf\"]}\"')
    print(f'export CUDA_LAUNCH_BLOCKING=\"{str(server_opt[\"cuda_launch_blocking\"]).lower()}\"')
    
    # ACT模型配置
    act_model = config['act_model']
    print(f'export ACT_CHUNK_SIZE=\"{act_model[\"chunk_size\"]}\"')
    print(f'export ACT_HIDDEN_DIM=\"{act_model[\"hidden_dim\"]}\"')
    print(f'export ACT_DIM_FEEDFORWARD=\"{act_model[\"dim_feedforward\"]}\"')
    print(f'export ACT_KL_WEIGHT=\"{act_model[\"kl_weight\"]}\"')
    print(f'export ACT_NUM_QUERIES=\"{act_model[\"num_queries\"]}\"')
    print(f'export ACT_INFERENCE_WORKERS=\"{server_opt[\"inference_workers\"]}\"')
    
    # 路径配置
    paths = config['paths']
    print(f'export ACT_MODEL_DIR=\"{paths[\"model_dir\"]}\"')
    print(f'export ACT_LOG_DIR=\"{paths[\"log_dir\"]}\"')
    
except Exception as e:
    print(f'echo \"❌ 配置文件解析失败: {e}\"', file=sys.stderr)
    sys.exit(1)
" > config_temp.sh

# 检查配置解析是否成功
if [[ $? -ne 0 ]]; then
    echo "❌ 配置文件解析失败"
    rm -f config_temp.sh
    exit 1
fi

# 加载配置变量
source config_temp.sh
rm -f config_temp.sh

echo "🔧 AgentLace协议配置:"
echo "   服务器IP: $ACT_SERVER_IP"
echo "   主端口: $ACT_SERVER_PORT (REQ-REP)"  
echo "   广播端口: $ACT_BROADCAST_PORT (PUB-SUB)"
echo "   协议版本: $ACT_PROTOCOL_VERSION"
echo ""
echo "⚡ 服务器性能配置:"
echo "   GPU设备: RTX 4090 (24GB)"
echo "   推理批次: $ACT_BATCH_SIZE"
echo "   工作进程: $ACT_INFERENCE_WORKERS"
echo "   模型路径: $ACT_MODEL_DIR"
echo ""

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

# AgentLace分布式框架依赖
pip install pyzmq>=24.0.0
pip install msgpack>=1.0.0
pip install cloudpickle>=2.0.0

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

import zmq
print(f'✓ ZeroMQ版本: {zmq.zmq_version()}')
print('✓ AgentLace通信协议就绪')
"

# 创建服务器端环境激活脚本
echo "📄 创建服务器端环境脚本..."
cat > activate_server_env.sh << 'EOF'
#!/bin/bash
# 服务器端环境激活脚本

source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate aloha

# AgentLace协议环境变量
export ACT_SERVER_IP="10.16.49.124"
export ACT_SERVER_PORT="5555" 
export ACT_BROADCAST_PORT="5556"

# GPU优化配置
export CUDA_VISIBLE_DEVICES="0"
export PYTORCH_CUDA_ALLOC_CONF="max_split_size_mb:1024"
export CUDA_LAUNCH_BLOCKING="0"

# 项目路径配置
export FRANKA_ACT_ROOT=$(pwd)
export PYTHONPATH=$FRANKA_ACT_ROOT:$FRANKA_ACT_ROOT/robot_server:$FRANKA_ACT_ROOT/communication:$PYTHONPATH

echo "🖥️  ACT推理服务器环境已激活"
echo "   - Conda环境: aloha"
echo "   - GPU设备: RTX 4090 (24GB)"
echo "   - 服务器地址: $ACT_SERVER_IP:$ACT_SERVER_PORT"
echo "   - 项目根目录: $FRANKA_ACT_ROOT"
echo "   - PyTorch版本: $(python -c 'import torch; print(torch.__version__)')"
EOF

chmod +x activate_server_env.sh

# AgentLace协议规范配置文件
echo "📋 创建AgentLace协议配置..."
cat > agentlace_config.py << 'EOF'
#!/usr/bin/env python3
"""
AgentLace协议配置 - ACT分布式推理系统
定义客户端和服务器之间的通信协议和数据格式
"""

from dataclasses import dataclass, field
from typing import Dict, Any, List
import numpy as np

# ============================================================================
# AgentLace协议配置
# ============================================================================

@dataclass
class ACTTrainerConfig:
    """ACT推理服务器AgentLace配置"""
    port_number: int = 5555              # 主通信端口(REQ-REP)
    broadcast_port: int = 5556           # 广播端口(PUB-SUB) 
    request_types: List[str] = field(default_factory=lambda: [
        "inference",                     # 推理请求
        "model_update",                 # 模型更新
        "server_status"                 # 服务器状态查询
    ])
    rate_limit: int = 1000              # 请求速率限制(req/s)
    version: str = "0.0.2"              # 协议版本

# ============================================================================
# 数据格式定义
# ============================================================================

# 推理请求数据格式
INFERENCE_REQUEST_FORMAT = {
    "type": "inference",
    "payload": {
        "qpos": [],                     # List[float] - 8维关节位置
        "qvel": [],                     # List[float] - 8维关节速度
        "image": "",                    # str - base64编码的图像
        "timestamp": 0.0,               # float - 时间戳
        "request_id": ""                # str - 请求ID
    }
}

# 推理响应数据格式  
INFERENCE_RESPONSE_FORMAT = {
    "success": True,                    # bool - 推理是否成功
    "actions": [],                      # List[List[float]] - 100x8动作序列
    "latency_ms": 0.0,                 # float - 推理延迟(毫秒)
    "error_msg": "",                   # str - 错误信息(如果有)
    "request_id": ""                   # str - 对应的请求ID
}

# 服务器状态响应格式
SERVER_STATUS_FORMAT = {
    "status": "running",               # str - 服务器状态
    "model_loaded": True,              # bool - 模型是否已加载
    "gpu_memory_mb": 0,                # int - GPU内存使用(MB)
    "inference_count": 0,              # int - 累计推理次数
    "avg_latency_ms": 0.0,            # float - 平均推理延迟
    "uptime_seconds": 0                # float - 运行时间(秒)
}

# ============================================================================
# 常量定义
# ============================================================================

class ACTConstants:
    """ACT系统常量"""
    
    # 网络配置
    SERVER_IP = "10.16.49.124"
    DEFAULT_PORT = 5555
    DEFAULT_BROADCAST_PORT = 5556
    
    # Franka机器人规格
    STATE_DIM = 8                      # 7关节 + 1夹爪
    ACTION_DIM = 8                     # 同状态维度
    CHUNK_SIZE = 100                   # ACT预测步数
    
    # 图像规格
    IMAGE_HEIGHT = 480
    IMAGE_WIDTH = 640
    IMAGE_CHANNELS = 3
    
    # 控制参数
    CONTROL_FREQ = 50                  # Hz
    DT = 0.02                         # 控制间隔(秒)

EOF

# 创建推理服务器启动脚本
echo "🚀 创建推理服务器启动脚本..."
cat > start_inference_server.sh << 'EOF'
#!/bin/bash
# ACT分布式推理服务器启动脚本
# 基于AgentLace协议的分布式推理架构

set -e

# 参数设置
SERVER_IP=${1:-"0.0.0.0"}      # AgentLace服务器监听IP
SERVER_PORT=${2:-5555}         # AgentLace服务器端口
TASK=${3:-"franka_pick_place"}  # 任务名称（模型路径）
GPU_ID=${4:-0}                 # GPU设备ID

echo "========================================="
echo "🚀 启动 ACT 分布式推理服务器..."
echo "AgentLace地址: $SERVER_IP:$SERVER_PORT"
echo "模型任务: $TASK"  
echo "GPU设备: $GPU_ID"
echo "========================================="

# 激活环境
source ./activate_server_env.sh

# 设置GPU设备
export CUDA_VISIBLE_DEVICES=$GPU_ID

# 检查GPU可用性
if command -v nvidia-smi &> /dev/null; then
    echo "🖥️  GPU状态检查："
    nvidia-smi --query-gpu=index,name,memory.used,memory.total --format=csv,noheader,nounits
else
    echo "⚠️  警告：未检测到NVIDIA GPU，将使用CPU推理"
fi

# 检查AgentLace
python -c "
try:
    from agentlace.trainer import TrainerServer, TrainerConfig
    print('✅ AgentLace导入成功')
except ImportError as e:
    print(f'❌ AgentLace导入失败: {e}')
    exit(1)
"

# 检查模型文件
MODEL_PATH="/home/wujielin/CascadeProjects/data/act_training/models/checkpoints/$TASK"
if [[ ! -d "$MODEL_PATH" ]]; then
    echo "❌ 错误：模型路径不存在 $MODEL_PATH"
    exit 1
else
    echo "✅ 模型路径验证: $MODEL_PATH"
    ls -la "$MODEL_PATH"/*.ckpt | head -3
fi

# 创建日志目录
mkdir -p logs
mkdir -p /tmp/act_server_logs

# 启动AgentLace分布式推理服务器
echo "🌐 启动AgentLace推理服务器..."
cd ../robot_server/communication/
python act_server.py \
    --port $SERVER_PORT \
    --model_path "$MODEL_PATH" \
    --config ../communication/robot_config.yaml \
    2>&1 | tee ../../logs/act_inference_server_$(date +%Y%m%d_%H%M%S).log &

SERVER_PID=$!

echo "========================================="
echo "✅ ACT 分布式推理服务器启动完成！"
echo ""
echo "🌐 服务器信息："
echo "- AgentLace进程ID: $SERVER_PID"  
echo "- 监听地址: $SERVER_IP:$SERVER_PORT"
echo "- 模型路径: $MODEL_PATH"
echo "- 日志文件: logs/act_inference_server_*.log"
echo ""
echo "🧪 测试命令："
echo "- 本地测试: python3 ../../test_act_client.py"
echo "- 性能测试: python3 ../robot_server/test_server.py"
echo "- 网络测试: nc -zv $SERVER_IP $SERVER_PORT"
echo ""
echo "📊 监控命令："
echo "- 查看进程: ps aux | grep act_server"
echo "- 查看日志: tail -f ../../logs/act_inference_server_*.log"
echo "- GPU监控: watch -n 1 nvidia-smi"
echo "- 网络连接: netstat -tulpn | grep $SERVER_PORT"
echo ""
echo "🛑 停止服务器: Ctrl+C 或 kill $SERVER_PID"
echo "========================================="

# 等待用户中断
trap 'echo "🛑 正在停止AgentLace推理服务器..."; kill $SERVER_PID; exit 0' INT

# 保持脚本运行，显示实时状态
echo "🔄 AgentLace推理服务器运行中，按 Ctrl+C 停止..."
while kill -0 $SERVER_PID 2>/dev/null; do
    sleep 15
    echo "$(date '+%H:%M:%S'): 🟢 AgentLace推理服务器运行中 (PID: $SERVER_PID)"
done

echo "🛑 AgentLace推理服务器已停止"

EOF

chmod +x start_inference_server.sh

echo "✅ ACT分布式推理服务器环境配置完成！"
echo ""
echo -e "${GREEN}🚀 使用方法：${NC}"
echo "1. 激活环境: source ./activate_server_env.sh"
echo "2. 启动推理服务器: ./start_inference_server.sh [端口] [模型路径]"
echo ""
echo -e "${BLUE}📋 系统信息：${NC}"
echo "   - 协议: AgentLace v0.0.2 (ZeroMQ)"
echo "   - 服务器IP: $ACT_SERVER_IP"
echo "   - 默认端口: $ACT_SERVER_PORT"
echo "   - GPU优化: RTX 4090 (24GB显存)"
echo "   - 推理批次: 最大16个并发请求"
echo ""
echo -e "${YELLOW}⚠️ 注意事项：${NC}"
echo "   - 确保模型已训练完成 (policy_epoch_*.ckpt)"
echo "   - 客户端需要连接到 $ACT_SERVER_IP:$ACT_SERVER_PORT"  
echo "   - 数据格式: qpos(8维) + image(480×640×3) → actions(100×8)"
echo ""
