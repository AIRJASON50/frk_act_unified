#!/bin/bash
# AgentLace 推理服务器启动脚本
# 功能：开启服务器的AgentLace网络端口，等待客户端连接

set -e

# 固定网络配置 - 基于当前机器网络状况
SERVER_IP="10.16.49.124"        # 主网卡IP，局域网可访问
SERVER_PORT="5555"              # AgentLace固定端口

# 可选参数
MODEL_PATH=${1:-"/home/wujielin/CascadeProjects/data/act_training/models/checkpoints/franka_pick_place"}

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

echo "=========================================" 
echo "🚀 启动 AgentLace 推理服务器"
echo "监听地址: $SERVER_IP:$SERVER_PORT"
echo "模型路径: $MODEL_PATH"
echo "GPU设备: $GPU_ID"
echo "========================================="

# 设置GPU设备
export CUDA_VISIBLE_DEVICES=$GPU_ID

# 设置项目路径
PROJECT_ROOT="/home/wujielin/CascadeProjects/projects/ws/frk_act_unified"
export PYTHONPATH="${PYTHONPATH}:${PROJECT_ROOT}:${PROJECT_ROOT}/communication/agentlace"

# 检查AgentLace依赖
python3 -c "
try:
    import zmq
    print('✅ ZeroMQ可用')
except ImportError:
    print('❌ ZeroMQ未安装，请运行: pip install pyzmq')
    exit(1)
" || exit 1
# 创建日志目录
mkdir -p logs

# 启动AgentLace推理服务器
echo "🌐 启动AgentLace推理服务器..."
cd ../robot_server/communication/
python3 act_server.py \
    --port $SERVER_PORT \
    --model_path "$MODEL_PATH" \
    2>&1 | tee ../../logs/agentlace_server_$(date +%Y%m%d_%H%M%S).log &

SERVER_PID=$!

echo "========================================="
echo "✅ AgentLace推理服务器已启动"
echo "进程ID: $SERVER_PID"
echo "监听地址: $SERVER_IP:$SERVER_PORT"
echo "等待客户端连接..."
echo "========================================="

# 等待用户中断
trap 'echo "🛑 正在停止AgentLace服务器..."; kill $SERVER_PID 2>/dev/null; exit 0' INT

# 保持脚本运行
wait $SERVER_PID
