#!/bin/bash
# AgentLace 客户端连接脚本
# 功能：连接AgentLace服务器，为数据收发提供协议

set -e

# 固定网络配置 - 连接到服务器
SERVER_IP="10.16.49.124"        # 服务器固定IP
SERVER_PORT="5555"              # 服务器固定端口

# 可选参数
MODE=${1:-"sim"}                # 运行模式：sim 或 real

echo "========================================="
echo "🤖 启动 AgentLace 客户端"
echo "服务器地址: $SERVER_IP:$SERVER_PORT"
echo "运行模式: $MODE"
echo "========================================="

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

# 测试AgentLace服务器连接
echo "🔗 测试与AgentLace服务器连接..."
python3 -c "
import sys
import socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    result = sock.connect_ex(('$SERVER_IP', $SERVER_PORT))
    sock.close()
    if result == 0:
        print('✅ AgentLace服务器连接正常')
        sys.exit(0)
    else:
        print('❌ 无法连接到AgentLace服务器')
        sys.exit(1)
except Exception as e:
    print(f'❌ 连接测试失败: {e}')
    sys.exit(1)
" || {
    echo "❌ AgentLace服务器不可达，请先启动服务器"
    echo "   启动命令: ./server_setup.sh"
    exit 1
}

# 启动AgentLace客户端
echo "🚀 启动AgentLace客户端..."
python3 ../test_act_client.py \
    --server_host $SERVER_IP \
    --server_port $SERVER_PORT \
    --mode $MODE \
    2>&1 | tee ../logs/agentlace_client_$(date +%Y%m%d_%H%M%S).log &

CLIENT_PID=$!

echo "========================================="
echo "✅ AgentLace客户端已启动"
echo "进程ID: $CLIENT_PID"
echo "连接地址: $SERVER_IP:$SERVER_PORT"
echo "通信协议就绪，等待推理请求..."
echo "========================================="

# 等待用户中断
trap 'echo "🛑 正在停止AgentLace客户端..."; kill $CLIENT_PID 2>/dev/null; exit 0' INT

# 保持脚本运行
wait $CLIENT_PID
