#!/bin/bash
# deploy/client_setup.sh
# Franka ACT 客户端环境配置脚本
# 适用于机器人控制端，需要ROS环境

set -e

echo "========================================="
echo "Franka ACT 客户端环境配置"
echo "========================================="

# 检查ROS环境
if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
    echo "❌ ROS Noetic未安装，请先安装ROS Noetic"
    echo "参考: http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
fi

# 检查conda环境
if ! command -v conda &> /dev/null; then
    echo "❌ Conda未安装，请先安装Miniconda或Anaconda"
    exit 1
fi

# 激活或创建客户端环境
echo "🔧 配置客户端Python环境..."
source $HOME/miniconda3/etc/profile.d/conda.sh
if ! conda env list | grep -q "franka_client"; then
    echo "创建franka_client conda环境..."
    conda create -n franka_client python=3.9 -y
fi

conda activate franka_client

# 客户端依赖安装
echo "📦 安装客户端Python依赖..."
pip install --upgrade pip

# 基础科学计算包（轻量版）
pip install numpy==1.24.3 scipy
pip install gymnasium==1.1.1

# OpenCV和图像处理（无CUDA版本）
pip install opencv-python-headless==4.8.0.76
pip install pillow>=8.0.0 imageio

# Web通信
pip install flask==2.3.2 requests==2.31.0
pip install websockets

# 配置文件
pip install pyyaml>=6.0

# ROS Python支持
pip install rospy-builder catkin-pkg rospkg
pip install sensor-msgs geometry-msgs std-msgs

# 机器人相关
pip install modern-robotics
pip install scipy

echo "🤖 配置ROS环境..."
source /opt/ros/noetic/setup.bash

# 检查franka_ros是否存在
if [ ! -d "franka_ros" ]; then
    echo "⚠️  franka_ros子模块未找到，初始化git子模块..."
    git submodule update --init --recursive
fi

echo "🧪 验证客户端环境..."
python3 -c "
try:
    import numpy as np
    print(f'✓ NumPy版本: {np.__version__}')
    
    import cv2
    print(f'✓ OpenCV版本: {cv2.__version__}')
    
    import gymnasium as gym
    print('✓ Gymnasium导入成功')
    
    import yaml, flask, requests
    print('✓ 配置和通信库导入成功')
    
    # 测试ROS Python包（可能失败，但不影响基本功能）
    try:
        import rospy, sensor_msgs, geometry_msgs
        print('✓ ROS Python包导入成功')
    except ImportError as e:
        print(f'⚠️  ROS Python包导入失败: {e}')
        print('   （可能需要在ROS环境中运行）')
        
except Exception as e:
    print(f'❌ 环境验证失败: {e}')
"

# 创建客户端环境激活脚本
echo "📄 创建客户端环境脚本..."
cat > activate_client_env.sh << 'EOF'
#!/bin/bash
# 客户端环境激活脚本

# 激活ROS环境
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_IP=$(hostname -I | awk '{print $1}')

# 激活conda环境
source $HOME/miniconda3/etc/profile.d/conda.sh
conda activate franka_client

# 设置项目路径
export FRANKA_ACT_ROOT=$(pwd)
export PYTHONPATH=$FRANKA_ACT_ROOT:$FRANKA_ACT_ROOT/robot_env:$PYTHONPATH

# Franka相关环境变量
export ROBOT_IP=${ROBOT_IP:-"172.16.0.2"}
export SERVER_IP=${SERVER_IP:-"10.16.49.124"}

echo "🤖 客户端环境已激活"
echo "   - ROS环境: Noetic"
echo "   - Conda环境: franka_client"
echo "   - ROS_MASTER_URI: $ROS_MASTER_URI"
echo "   - ROS_IP: $ROS_IP"
echo "   - 机器人IP: $ROBOT_IP"
echo "   - 服务器IP: $SERVER_IP"
echo "   - 项目根目录: $FRANKA_ACT_ROOT"
EOF

chmod +x activate_client_env.sh

# 创建客户端启动脚本
echo "🚀 配置客户端启动脚本..."
cat > start_franka_client.sh << 'EOF'
#!/bin/bash
# 客户端启动脚本

set -e

# 参数设置
SERVER_IP=${1:-"10.16.49.124"}  # 训练服务器IP
ROBOT_IP=${2:-"172.16.0.2"}    # 机器人IP  
MODE=${3:-"sim"}               # sim 或 real
TASK=${4:-"franka_cube_transfer"}  # 任务名称

echo "========================================="
echo "启动 Franka ACT 客户端"
echo "训练服务器IP: $SERVER_IP"
echo "机器人IP: $ROBOT_IP"
echo "运行模式: $MODE"
echo "任务名称: $TASK"
echo "========================================="

# 激活环境
source ./activate_client_env.sh

# 设置环境变量
export SERVER_IP=$SERVER_IP
export ROBOT_IP=$ROBOT_IP

# 启动ROS master（如果未运行）
if ! pgrep -f "rosmaster" > /dev/null; then
    echo "🚀 启动ROS Master..."
    roscore &
    sleep 3
fi

# 根据模式启动不同组件
if [ "$MODE" = "sim" ]; then
    echo "🎮 启动Gazebo仿真..."
    # 启动Franka Gazebo仿真
    roslaunch franka_gazebo panda.launch &
    sleep 10
    
    # 启动仿真控制器
    roslaunch franka_example_controllers cartesian_impedance_example_controller.launch &
    sleep 5
    
elif [ "$MODE" = "real" ]; then
    echo "🤖 连接真实机器人..."
    # 启动真实机器人控制
    roslaunch franka_control franka_control.launch robot_ip:=$ROBOT_IP &
    sleep 5
    
    # 启动控制器
    roslaunch franka_example_controllers cartesian_impedance_example_controller.launch &
    sleep 5
fi

# 启动Flask HTTP服务器
echo "🌐 启动Flask服务器..."
cd robot_servers
python3 franka_server.py --robot_ip $ROBOT_IP --mode $MODE &
FLASK_PID=$!
cd ..
sleep 3

# 启动ACT客户端
echo "🧠 启动ACT训练客户端..."
python3 train_act_distributed.py \
    --mode client \
    --server_ip $SERVER_IP \
    --server_port 5555 \
    --robot_ip $ROBOT_IP \
    --task $TASK \
    --config configs/act_config.yaml \
    2>&1 | tee logs/client_$(date +%Y%m%d_%H%M%S).log

# 清理进程
echo "🧹 清理进程..."
kill $FLASK_PID 2>/dev/null || true
pkill -f "gazebo" 2>/dev/null || true
pkill -f "roslaunch" 2>/dev/null || true

EOF

chmod +x start_franka_client.sh

echo "✅ 客户端环境配置完成！"
echo ""
echo "使用方法："
echo "1. 激活环境: source ./activate_client_env.sh"
echo "2. 启动客户端: ./start_franka_client.sh [服务器IP] [机器人IP] [模式] [任务]"
echo "3. 示例: ./start_franka_client.sh 10.16.49.124 172.16.0.2 sim franka_cube_transfer"
echo ""
