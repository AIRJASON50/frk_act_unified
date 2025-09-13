#!/bin/bash
# scripts/start_act_server.sh
# ACT 训练服务器启动脚本（GPU训练机）
# 参考计划文档阶段2：分布式训练集成

set -e

# 参数设置
SERVER_IP=${1:-"0.0.0.0"}      # 服务器监听IP
SERVER_PORT=${2:-5555}         # 服务器端口
TASK=${3:-"franka_cube_transfer"}  # 任务名称
GPU_ID=${4:-0}                 # GPU设备ID

echo "========================================="
echo "启动 ACT 训练服务器..."
echo "服务器地址: $SERVER_IP:$SERVER_PORT"
echo "任务名称: $TASK"
echo "GPU设备: $GPU_ID"
echo "========================================="

# 检查conda环境
if [ -z "$CONDA_DEFAULT_ENV" ] || [ "$CONDA_DEFAULT_ENV" != "aloha" ]; then
    echo "激活aloha conda环境..."
    source ~/anaconda3/etc/profile.d/conda.sh
    conda activate aloha
fi

# 设置GPU设备
export CUDA_VISIBLE_DEVICES=$GPU_ID

# 检查GPU可用性
if command -v nvidia-smi &> /dev/null; then
    echo "GPU状态检查："
    nvidia-smi --query-gpu=index,name,memory.used,memory.total --format=csv,noheader,nounits
else
    echo "警告：未检测到NVIDIA GPU，将使用CPU训练"
fi

# 设置项目路径
PROJECT_ROOT="/home/wujielin/CascadeProjects/projects/ws/frk_act_unified"
cd $PROJECT_ROOT

echo "[1/3] 检查依赖和配置..."
# 检查Python依赖
python -c "import torch; print(f'PyTorch版本: {torch.__version__}')"
python -c "import torch; print(f'CUDA可用: {torch.cuda.is_available()}')"

# 检查配置文件
if [ ! -f "configs/act_config.yaml" ]; then
    echo "错误：配置文件 configs/act_config.yaml 不存在"
    exit 1
fi

echo "[2/3] 创建输出目录..."
# 创建必要的目录
mkdir -p checkpoints
mkdir -p logs
mkdir -p results

echo "[3/3] 启动 ACT 训练服务器..."
# 启动训练服务器，重定向日志
python train_act_distributed.py \
    --server \
    --server_ip $SERVER_IP \
    --server_port $SERVER_PORT \
    --config configs/act_config.yaml \
    2>&1 | tee logs/act_server_$(date +%Y%m%d_%H%M%S).log &

SERVER_PID=$!

echo "========================================="
echo "✅ ACT 训练服务器启动完成！"
echo ""
echo "服务器信息："
echo "- 进程ID: $SERVER_PID"
echo "- 监听地址: $SERVER_IP:$SERVER_PORT"
echo "- 日志文件: logs/act_server_*.log"
echo "- 模型保存: checkpoints/"
echo ""
echo "客户端连接命令："
echo "./scripts/start_franka_client.sh $(hostname -I | awk '{print $1}')"
echo ""
echo "监控命令："
echo "- 查看进程: ps aux | grep train_act_distributed"
echo "- 查看日志: tail -f logs/act_server_*.log"
echo "- GPU监控: watch -n 1 nvidia-smi"
echo ""
echo "停止服务器: Ctrl+C 或 kill $SERVER_PID"
echo "========================================="

# 等待用户中断
trap 'echo "正在停止训练服务器..."; kill $SERVER_PID; exit 0' INT

# 保持脚本运行，显示实时状态
while kill -0 $SERVER_PID 2>/dev/null; do
    sleep 10
    echo "$(date): ACT训练服务器运行中 (PID: $SERVER_PID)"
done

echo "ACT训练服务器已停止"
