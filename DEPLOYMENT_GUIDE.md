# Franka ACT 分布式部署指南

## 📋 系统架构

```
┌─────────────────┐         ┌─────────────────┐
│   服务器端       │         │   客户端         │
│  (GPU训练服务器) │  <───>  │ (机器人控制端)   │
│                 │         │                 │
│ • 模型训练       │         │ • 机器人控制     │
│ • 模型推理       │         │ • 数据采集       │
│ • 网络通信       │         │ • Flask HTTP     │
│ • 数据存储       │         │ • ROS接口        │
└─────────────────┘         └─────────────────┘
        ↑                           ↑
    AgentLace                   FrankaACTEnv
     (Port 5555)               (HTTP + ROS)
```

## 🖥️ 服务器端部署

### 硬件要求
- **GPU**: NVIDIA RTX 4090 或更高
- **内存**: 32GB+ RAM
- **存储**: 100GB+ 可用空间
- **网络**: 千兆以太网

### 环境配置
```bash
# 1. 克隆项目
cd /path/to/projects
git clone [项目地址]
cd frk_act_unified

# 2. 运行服务器端配置
chmod +x deploy/server_setup.sh
./deploy/server_setup.sh

# 3. 激活服务器环境
source ./activate_server_env.sh

# 4. 验证环境
python3 -c "import torch; print(f'GPU数量: {torch.cuda.device_count()}')"
```

### 启动服务器
```bash
# 基础启动（默认端口5555，使用所有GPU）
./start_training_server.sh

# 自定义配置
./start_training_server.sh 5555 "0,1" models/my_model

# 参数说明：
# $1: 端口号 (默认5555)
# $2: GPU设备 (默认"0,1,2,3")
# $3: 模型保存目录 (默认models/franka_act)
```

## 💻 客户端部署

### 硬件要求
- **CPU**: Intel i5 或更高
- **内存**: 16GB+ RAM  
- **网络**: 千兆以太网连接服务器
- **机器人**: Franka Emika Panda (仿真或真实)

### 环境配置
```bash
# 1. 安装ROS Noetic (Ubuntu 20.04)
sudo apt update
sudo apt install ros-noetic-desktop-full

# 2. 克隆项目到客户端
cd /path/to/client
git clone [项目地址]
cd frk_act_unified

# 3. 运行客户端配置
chmod +x deploy/client_setup.sh
./deploy/client_setup.sh

# 4. 激活客户端环境
source ./activate_client_env.sh

# 5. 验证环境
python3 -c "import cv2, numpy; print('客户端环境正常')"
```

### 启动客户端
```bash
# 仿真模式启动
./start_franka_client.sh 10.16.49.124 172.16.0.2 sim

# 真实机器人模式
./start_franka_client.sh 10.16.49.124 172.16.0.2 real

# 参数说明：
# $1: 服务器IP (训练服务器地址)
# $2: 机器人IP (仿真默认172.16.0.2)
# $3: 模式 (sim/real)
# $4: 任务名称 (可选)
```

## 🔧 环境差异说明

### 服务器端环境 (`aloha`)
- **PyTorch**: 2.0.0 + CUDA 11.8
- **无ROS**: 纯Python环境
- **GPU支持**: 多GPU训练
- **依赖**: 深度学习 + Web框架

### 客户端环境 (`franka_client`)  
- **轻量级**: 无PyTorch GPU版本
- **ROS集成**: ROS Noetic + Python
- **OpenCV**: CPU版本，避免GPU冲突
- **依赖**: 机器人控制 + 通信

## 🚦 启动流程

### 1. 服务器端启动
```bash
# 激活环境
source ./activate_server_env.sh

# 启动训练服务器
./start_training_server.sh
# 看到: "🚀 训练服务器启动在端口 5555"
```

### 2. 客户端启动
```bash
# 激活环境  
source ./activate_client_env.sh

# 启动客户端（替换为实际服务器IP）
./start_franka_client.sh 10.16.49.124 172.16.0.2 sim
# 看到: "🤖 客户端连接到服务器 10.16.49.124:5555"
```

## 📊 监控和日志

### 服务器端监控
```bash
# 查看训练日志
tail -f logs/server_*.log

# 监控GPU使用
nvidia-smi -l 1

# 监控网络端口
netstat -tlnp | grep 5555
```

### 客户端监控
```bash
# 查看客户端日志
tail -f logs/client_*.log

# 监控ROS节点
rosnode list
rostopic list

# 检查Flask服务器
curl http://localhost:5000/robot_state
```

## 🐛 故障排除

### 网络连接问题
```bash
# 测试服务器连通性
ping 10.16.49.124
telnet 10.16.49.124 5555

# 检查防火墙
sudo ufw status
sudo ufw allow 5555/tcp
```

### ROS环境问题
```bash
# 重新source ROS
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

# 检查ROS master
roscore &
rosnode list
```

### 依赖问题
```bash
# 服务器端重新安装
conda activate aloha
pip install --force-reinstall torch torchvision

# 客户端重新安装
conda activate franka_client  
pip install --force-reinstall opencv-python-headless
```

## 📈 性能调优

### GPU内存优化
```bash
# 限制GPU内存使用
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512

# 使用指定GPU
export CUDA_VISIBLE_DEVICES=0,1
```

### 网络优化
```bash
# 调整网络缓冲区
echo 'net.core.rmem_max = 268435456' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 268435456' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## 🔄 更新和维护

### 更新项目代码
```bash
# 服务器端更新
git pull origin main
git submodule update --recursive

# 客户端更新  
git pull origin main
git submodule update --recursive
```

### 清理和重置
```bash
# 清理日志
rm -f logs/*.log

# 重置模型
rm -rf models/franka_act/*

# 重启所有服务
pkill -f "train_act_distributed"
pkill -f "franka_server"
```
