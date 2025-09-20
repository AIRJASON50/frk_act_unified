#!/bin/bash
# ACT分布式推理客户端配置脚本
# 基于robot_config.yaml的客户端配置，支持仿真和真机

set -e

echo "========================================="
echo "ACT分布式推理客户端配置"
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
    print(f'export ACT_PROTOCOL=\"{network[\"protocol\"]}\"')
    print(f'export ACT_PROTOCOL_VERSION=\"{network[\"protocol_version\"]}\"')
    
    # 机器人配置
    robot = config['robot']
    print(f'export ROBOT_IP=\"{robot[\"robot_ip\"]}\"')
    print(f'export ROBOT_TYPE=\"{robot[\"robot_type\"]}\"')
    print(f'export GRIPPER_TYPE=\"{robot[\"gripper_type\"]}\"')
    print(f'export CONTROL_FREQ=\"{robot[\"control_freq\"]}\"')
    
    # 相机配置
    camera = config['camera']
    print(f'export IMAGE_HEIGHT=\"{camera[\"image_height\"]}\"')
    print(f'export IMAGE_WIDTH=\"{camera[\"image_width\"]}\"')
    print(f'export JPEG_QUALITY=\"{camera[\"jpeg_quality\"]}\"')
    
    # 客户端配置
    client = config['client']
    print(f'export USE_SIMULATION=\"{str(client[\"use_simulation\"]).lower()}\"')
    print(f'export GAZEBO_WORLD=\"{client[\"gazebo_world\"]}\"')
    print(f'export ROS_MASTER_PORT=\"{client[\"ros_master_port\"]}\"')
    
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

echo "🔧 客户端网络配置:"
echo "   服务器地址: $ACT_SERVER_IP:$ACT_SERVER_PORT"
echo "   通信协议: $ACT_PROTOCOL v$ACT_PROTOCOL_VERSION"
echo ""
echo "🤖 机器人配置:"
echo "   机器人IP: $ROBOT_IP"
echo "   机器人类型: $ROBOT_TYPE"
echo "   夹爪类型: $GRIPPER_TYPE"
echo "   控制频率: ${CONTROL_FREQ}Hz"
echo "   使用仿真: $USE_SIMULATION"
echo ""

# 检查ROS环境
ROS_DISTRO=${1:-noetic}
echo "🔧 检查ROS环境..."

if ! command -v roscore &> /dev/null; then
    echo "❌ ROS未安装，请先安装ROS $ROS_DISTRO"
    echo "参考: http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
fi

echo "✅ ROS环境: $(rosversion -d)"

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

# 客户端依赖安装
echo "📦 安装客户端Python依赖..."
pip install --upgrade pip

# 基础科学计算包
pip install numpy==1.24.3 scipy matplotlib
pip install opencv-python pillow>=8.0.0 imageio

# AgentLace客户端依赖
pip install pyzmq>=24.0.0 msgpack>=1.0.0 cloudpickle>=2.0.0 lz4

# 配置文件和数据处理
pip install pyyaml>=6.0 h5py
pip install tqdm ipython

# ROS-Python桥接
echo "🔗 安装ROS-Python桥接依赖..."
pip install rospkg catkin_pkg

echo "🧪 验证客户端环境..."
python3 -c "
import numpy as np
print(f'✓ NumPy版本: {np.__version__}')

import cv2
print(f'✓ OpenCV版本: {cv2.__version__}')

import yaml
print('✓ YAML支持正常')

import zmq
print(f'✓ ZeroMQ版本: {zmq.zmq_version()}')
print('✓ AgentLace客户端依赖就绪')

try:
    import rospy
    print('✓ ROS-Python桥接正常')
except:
    print('⚠️ ROS-Python桥接可能有问题')
"

# 创建客户端环境激活脚本
echo "📄 创建客户端环境脚本..."
cat > activate_client_env.sh << 'EOF'
#!/bin/bash
# 客户端环境激活脚本

# 激活ROS环境
source /opt/ros/noetic/setup.bash

# 激活conda环境
source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate aloha

# 读取配置文件
if [[ -f "robot_config.yaml" ]]; then
    # 解析配置并设置环境变量
    python3 -c "
import yaml
with open('robot_config.yaml', 'r') as f:
    config = yaml.safe_load(f)

network = config['network']
robot = config['robot']
camera = config['camera']

print(f'export ACT_SERVER_IP=\"{network[\"server_ip\"]}\"')
print(f'export ACT_SERVER_PORT=\"{network[\"server_port\"]}\"')
print(f'export ROBOT_IP=\"{robot[\"robot_ip\"]}\"')
print(f'export CONTROL_FREQ=\"{robot[\"control_freq\"]}\"')
print(f'export IMAGE_HEIGHT=\"{camera[\"image_height\"]}\"')
print(f'export IMAGE_WIDTH=\"{camera[\"image_width\"]}\"')
    " > temp_config.sh
    source temp_config.sh
    rm -f temp_config.sh
fi

# 设置环境变量
export FRANKA_ACT_ROOT=$(pwd)
export PYTHONPATH=$FRANKA_ACT_ROOT:$FRANKA_ACT_ROOT/communication:$PYTHONPATH

# 设置ROS变量
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

echo "🤖 客户端环境已激活"
echo "   - ROS发行版: $(rosversion -d)"
echo "   - Conda环境: aloha"
echo "   - 服务器地址: $ACT_SERVER_IP:$ACT_SERVER_PORT"
echo "   - 项目根目录: $FRANKA_ACT_ROOT"
EOF

chmod +x activate_client_env.sh

# 创建客户端启动脚本
echo "🚀 配置客户端启动脚本..."
cat > start_act_client.sh << 'EOF'
#!/bin/bash
# Franka ACT 分布式推理客户端启动脚本（Franka控制机）
# 基于AgentLace协议的分布式架构

set -e

# 参数设置
SERVER_IP=${1:-"10.16.49.124"}  # AgentLace推理服务器IP
ROBOT_IP=${2:-"172.16.0.2"}    # 机器人IP  
MODE=${3:-"sim"}               # sim 或 real
TASK=${4:-"franka_pick_place"}  # 任务名称

echo "========================================="
echo "🚀 启动 Franka ACT 分布式推理客户端..."
echo "AgentLace服务器IP: $SERVER_IP:5555"
echo "机器人IP: $ROBOT_IP"
echo "运行模式: $MODE"
echo "任务名称: $TASK"
echo "========================================="

# 激活环境
source ./activate_client_env.sh

# 添加AgentLace到Python路径
export PYTHONPATH="${PYTHONPATH}:${FRANKA_ACT_ROOT}:${FRANKA_ACT_ROOT}/communication/agentlace"

# 启动ROS核心
echo "[1/4] 启动 ROS Master..."
if ! pgrep -x "rosmaster" > /dev/null; then
    roscore &
    sleep 3
    echo "ROS Master started"
else
    echo "ROS Master already running"
fi

echo "[2/4] 启动 Franka 系统..."
if [ "$MODE" = "sim" ]; then
    echo "启动 Franka Gazebo 仿真..."
    
    # 启动Gazebo仿真
    roslaunch franka_gazebo panda.launch \
        gazebo:=true \
        headless:=false \
        use_gripper:=true \
        controller:=cartesian_impedance_example_controller \
        rviz:=true &
    
    sleep 15  # 等待Gazebo完全启动
    echo "Franka Gazebo仿真启动完成"
    
elif [ "$MODE" = "real" ]; then
    echo "启动 Franka 真机控制..."
    
    # 启动真机控制器
    roslaunch franka_control franka_control.launch \
        robot_ip:=$ROBOT_IP \
        load_gripper:=true \
        robot:=panda &
    
    sleep 8  # 等待真机连接
    echo "Franka真机控制启动完成"
    
else
    echo "错误：不支持的模式 $MODE，请使用 'sim' 或 'real'"
    exit 1
fi

echo "[3/4] 检查AgentLace连接..."
# 测试AgentLace服务器连接
echo "🔗 测试与AgentLace推理服务器的连接..."
python3 -c "
import sys
import socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    result = sock.connect_ex(('$SERVER_IP', 5555))
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
    echo "❌ AgentLace服务器不可达，请检查服务器是否启动"
    echo "   启动命令: cd communication && ./start_inference_server.sh"
    exit 1
}

echo "[4/4] 启动 ACT 分布式推理客户端..."
# 启动AgentLace客户端进行分布式推理
echo "🎯 启动分布式ACT推理客户端..."
python3 ../test_act_client.py \
    --server_host $SERVER_IP \
    --server_port 5555 \
    --mode $MODE \
    --robot_ip $ROBOT_IP &

CLIENT_PID=$!

echo "========================================="
echo "✅ Franka ACT 分布式推理客户端启动完成！"
echo ""
echo "🔧 系统状态："
echo "- ROS Master: $(pgrep -x rosmaster | head -1 || echo 'Not running')"
echo "- AgentLace Client PID: $CLIENT_PID"
echo "- 推理服务器: $SERVER_IP:5555"
echo "- 运行模式: $MODE"
echo ""
echo "📊 监控命令："
echo "- ROS topics: rostopic list"
echo "- 客户端日志: tail -f /tmp/act_client.log"
echo "- 网络连接: netstat -an | grep 5555"
echo "- 系统状态: ps aux | grep test_act_client"
echo ""
echo "🎯 测试命令："
echo "- 手动测试: python3 ../test_act_client.py --server_host $SERVER_IP"
echo "- 性能测试: python3 ../robot_server/test_server.py"
echo ""
echo "🛑 停止客户端: Ctrl+C 或 kill $CLIENT_PID"
echo "========================================="

# 等待用户中断
trap 'echo "🛑 正在停止AgentLace客户端..."; kill $CLIENT_PID 2>/dev/null || true; exit 0' INT

# 保持脚本运行
echo "🔄 客户端运行中，按 Ctrl+C 停止..."
wait $CLIENT_PID

EOF

chmod +x start_act_client.sh

echo "✅ ACT分布式推理客户端配置完成！"
echo ""
echo -e "${GREEN}🚀 使用方法：${NC}"
echo "1. 激活环境: source ./activate_client_env.sh"
echo "2. 启动仿真客户端: ./start_act_client.sh sim"
echo "3. 启动真机客户端: ./start_act_client.sh real"
echo ""
echo -e "${BLUE}📋 系统信息：${NC}"
echo "   - 服务器地址: $ACT_SERVER_IP:$ACT_SERVER_PORT"
echo "   - 机器人IP: $ROBOT_IP"
echo "   - 协议: $ACT_PROTOCOL v$ACT_PROTOCOL_VERSION"
echo "   - 图像尺寸: ${IMAGE_HEIGHT}×${IMAGE_WIDTH}"
echo ""
echo -e "${YELLOW}⚠️ 注意事项：${NC}"
echo "   - 确保服务器端已启动 (./start_inference_server.sh)"
echo "   - 仿真模式需要Gazebo和franka_ros包"
echo "   - 真机模式需要正确的机器人IP和网络连接"
echo ""

