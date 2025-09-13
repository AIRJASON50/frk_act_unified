#!/bin/bash
# scripts/setup_project.sh
# Franka ACT 项目初始化脚本
# 参考计划文档 lines 252-273

set -e

echo "========================================="
echo "Franka ACT 统一架构项目初始化"
echo "========================================="

# 设置项目根路径
PROJECT_ROOT="/home/wujielin/CascadeProjects/projects/ws/frk_act_unified"
cd $PROJECT_ROOT

echo "[1/6] 检查项目结构..."
# 创建必要的目录结构
mkdir -p checkpoints
mkdir -p logs  
mkdir -p results
mkdir -p data
mkdir -p tests

echo "✓ 目录结构检查完成"

echo "[2/6] 初始化 Git 仓库和子模块..."
# 检查Git仓库状态
if [ ! -d ".git" ]; then
    echo "初始化Git仓库..."
    git init
    git add .gitmodules
    git commit -m "Initial commit with franka_ros submodule"
fi

# 初始化和更新Git子模块
echo "更新 franka_ros 子模块..."
git submodule update --init --recursive

echo "✓ Git仓库和子模块初始化完成"

echo "[3/6] 检查conda环境..."
# 检查aloha环境是否存在
if conda env list | grep -q "aloha"; then
    echo "✓ aloha conda环境已存在"
    source ~/anaconda3/etc/profile.d/conda.sh
    conda activate aloha
else
    echo "错误：aloha conda环境不存在，请先创建环境"
    echo "创建命令: conda create -n aloha python=3.8"
    exit 1
fi

echo "[4/6] 安装Python依赖..."
# 安装项目依赖
pip install -r requirements.txt

# 安装AgentLace（开发模式）
echo "安装AgentLace分布式框架..."
cd agentlace
pip install -e .
cd ..

echo "✓ Python依赖安装完成"

echo "[5/6] 设置ROS环境..."
# 检查ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "✓ ROS Noetic已安装"
    source /opt/ros/noetic/setup.bash
else
    echo "警告：未检测到ROS Noetic，请确保已正确安装"
fi

# 设置franka_ros环境变量
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:${PROJECT_ROOT}/franka_ros"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${PROJECT_ROOT}/franka_ros"

echo "✓ ROS环境设置完成"

echo "[6/6] 验证安装..."
# 验证关键组件
echo "验证Python导入..."
python -c "
try:
    import torch
    import numpy as np
    import gym
    import flask
    import yaml
    print('✓ 核心Python包导入成功')
except ImportError as e:
    print(f'✗ Python包导入失败: {e}')
    exit(1)
"

echo "验证项目模块..."
python -c "
import sys
sys.path.append('.')
try:
    from robot_env.franka_act_env import FrankaACTEnv
    from act_algo.policy import ACTPolicy
    print('✓ 项目模块导入成功')
except ImportError as e:
    print(f'✗ 项目模块导入失败: {e}')
    exit(1)
"

# 创建环境设置脚本
echo "创建环境激活脚本..."
cat > activate_env.sh << 'EOF'
#!/bin/bash
# 激活Franka ACT项目环境

# 激活conda环境
source ~/anaconda3/etc/profile.d/conda.sh
conda activate aloha

# 设置ROS环境
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

# 设置项目路径
export FRANKA_ACT_ROOT="/home/wujielin/CascadeProjects/projects/ws/frk_act_unified"
export PYTHONPATH="${PYTHONPATH}:${FRANKA_ACT_ROOT}"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:${FRANKA_ACT_ROOT}/franka_ros"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${FRANKA_ACT_ROOT}/franka_ros"

cd $FRANKA_ACT_ROOT

echo "✅ Franka ACT环境已激活"
echo "项目路径: $FRANKA_ACT_ROOT"
echo "Python环境: $(which python)"
echo "ROS版本: $ROS_DISTRO"
EOF

chmod +x activate_env.sh

# 设置脚本执行权限
chmod +x scripts/*.sh

echo "========================================="
echo "✅ Franka ACT 项目初始化完成！"
echo ""
echo "项目结构："
echo "- 原版ACT算法: act_algo/"
echo "- SERL Flask服务器: robot_servers/"
echo "- AgentLace分布式框架: agentlace/"
echo "- Franka环境适配器: robot_env/"
echo "- franka_ros子模块: franka_ros/"
echo "- 训练脚本: train_act_distributed.py"
echo "- 启动脚本: scripts/"
echo ""
echo "使用方法："
echo "1. 激活环境: source activate_env.sh"
echo "2. 启动服务器: ./scripts/start_act_server.sh"
echo "3. 启动客户端: ./scripts/start_franka_client.sh [server_ip]"
echo ""
echo "配置文件："
echo "- ACT参数: configs/act_config.yaml"
echo "- 机器人配置: configs/robot_config.yaml"
echo ""
echo "日志和输出："
echo "- 训练日志: logs/"
echo "- 模型检查点: checkpoints/"
echo "- 实验结果: results/"
echo "========================================="
