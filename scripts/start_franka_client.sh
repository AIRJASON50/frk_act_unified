#!/bin/bash
# scripts/start_franka_client.sh
# Franka ACT 客户端启动脚本（Franka控制机）
# 参考计划文档 lines 277-328

set -e

# 参数设置
SERVER_IP=${1:-"10.16.49.124"}  # 训练服务器IP
ROBOT_IP=${2:-"172.16.0.2"}    # 机器人IP  
MODE=${3:-"sim"}               # sim 或 real
TASK=${4:-"franka_cube_transfer"}  # 任务名称

echo "========================================="
echo "启动 Franka ACT 客户端..."
echo "训练服务器IP: $SERVER_IP"
echo "机器人IP: $ROBOT_IP"
echo "运行模式: $MODE"
echo "任务名称: $TASK"
echo "========================================="

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "设置ROS环境变量..."
    source /opt/ros/noetic/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_IP=$(hostname -I | awk '{print $1}')
fi

# 检查conda环境
if [ -z "$CONDA_DEFAULT_ENV" ] || [ "$CONDA_DEFAULT_ENV" != "aloha" ]; then
    echo "激活aloha conda环境..."
    source ~/anaconda3/etc/profile.d/conda.sh
    conda activate aloha
fi

# 设置项目路径
PROJECT_ROOT="/home/wujielin/CascadeProjects/projects/ws/frk_act_unified"
cd $PROJECT_ROOT

echo "[1/4] 启动 ROS Master..."
# 检查roscore是否已运行
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

echo "[3/4] 启动 SERL Flask HTTP 服务器..."
# 启动Flask机器人服务器（后台运行）
python robot_servers/franka_server.py \
    --robot_ip $ROBOT_IP \
    --gripper_type Franka \
    --port 5000 &

FLASK_PID=$!
sleep 5

# 检查Flask服务器是否正常启动
if curl -s "http://127.0.0.1:5000/status" > /dev/null; then
    echo "Flask服务器启动成功"
else
    echo "警告：Flask服务器可能未正常启动"
fi

echo "[4/4] 启动 ACT 客户端训练进程..."
# 启动ACT分布式训练客户端
python train_act_distributed.py \
    --client \
    --server_ip $SERVER_IP \
    --server_port 5555 \
    --flask_url "http://127.0.0.1:5000" \
    --config configs/act_config.yaml &

CLIENT_PID=$!

echo "========================================="
echo "✅ Franka ACT 客户端启动完成！"
echo ""
echo "进程状态："
echo "- ROS Master: $(pgrep -x rosmaster | head -1)"
echo "- Flask Server PID: $FLASK_PID"
echo "- ACT Client PID: $CLIENT_PID"
echo ""
echo "监控命令："
echo "- ROS topics: rostopic list"
echo "- Flask状态: curl http://127.0.0.1:5000/status"
echo "- 查看日志: tail -f /tmp/franka_act_client.log"
echo ""
echo "停止客户端: Ctrl+C 或 kill $CLIENT_PID"
echo "========================================="

# 等待用户中断
trap 'echo "正在停止所有进程..."; kill $FLASK_PID $CLIENT_PID; exit 0' INT

# 保持脚本运行
wait $CLIENT_PID
