#!/bin/bash

# ACT推理控制系统测试脚本
# 
# 功能：启动Franka ACT推理控制环境并进行基本功能测试
# 
# 使用方法：
# ./test_act_inference.sh [server_url] [task_name]
# 
# 示例：
# ./test_act_inference.sh 10.16.49.124:5555 sim_transfer_cube_scripted

set -e  # 遇到错误立即退出

# 默认参数
DEFAULT_SERVER_URL="10.16.49.124:5555"
DEFAULT_TASK_NAME="sim_transfer_cube_scripted"

# 解析命令行参数
SERVER_URL=${1:-$DEFAULT_SERVER_URL}
TASK_NAME=${2:-$DEFAULT_TASK_NAME}

echo "=========================================="
echo "🤖 Franka ACT推理控制系统测试"
echo "=========================================="
echo "📡 服务器地址: $SERVER_URL"
echo "🎯 任务名称: $TASK_NAME"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo "❌ ROS环境未设置，正在设置..."
    source /opt/ros/noetic/setup.bash
    cd /home/jason/ws/catkin_ws
    source devel/setup.bash
    echo "✅ ROS环境设置完成"
else
    echo "✅ ROS环境已设置"
fi

# 检查必要的包
echo "🔍 检查依赖包..."
python3 -c "import agentlace; print('✅ AgentLace可用')" || {
    echo "❌ AgentLace不可用，请检查aloha_client环境"
    exit 1
}

python3 -c "import cv2; print('✅ OpenCV可用')" || {
    echo "❌ OpenCV不可用"
    exit 1
}

echo "✅ 所有依赖检查通过"

# 测试AgentLace服务器连接
echo "🔗 测试AgentLace服务器连接..."
cd /home/jason/ws/catkin_ws/src/frk_act_unified/communication
timeout 10s ./client_setup.sh || {
    echo "❌ AgentLace服务器连接失败"
    echo "请确认服务器 $SERVER_URL 正在运行"
    exit 1
}
echo "✅ AgentLace服务器连接正常"

# 启动仿真环境
echo "🚀 启动Franka ACT推理仿真环境..."
echo "📌 Launch参数:"
echo "   server_url: $SERVER_URL"
echo "   task_name: $TASK_NAME"
echo ""

# 在当前终端启动launch文件
echo "🎯 启动ACT推理控制器..."
echo "📝 按 Ctrl+C 停止仿真"
echo ""

# 设置AgentLace环境
PROJECT_ROOT="/home/jason/ws/catkin_ws/src/frk_act_unified"
export PYTHONPATH="${PYTHONPATH}:${PROJECT_ROOT}:${PROJECT_ROOT}/communication/agentlace"

cd /home/jason/ws/catkin_ws
source devel/setup.bash
roslaunch frk_act_unified franka_act_inference.launch server_url:=$SERVER_URL task_name:=$TASK_NAME
