# Franka ACT 统一分布式训练系统

基于 SERL 架构 + 原版 ACT 算法 + AgentLace 分布式框架的 Franka 机器人模仿学习系统

## 🎯 项目概述

将原有的 `frk_act_server` 和 `frk_act_client` 分离架构重构为统一代码库，实现基于 ACT (Action Chunking with Transformers) 模仿学习的分布式训练系统。

### 核心特性
- **🔧 统一代码库**：消除 server-client 重复代码，基于 SERL 成熟架构
- **🌐 分布式部署**：支持训练服务器（GPU）与 Franka 控制机分离部署
- **🤖 ACT 集成**：集成原版 ACT 模仿学习算法
- **🎮 仿真支持**：基于 Franka 官方 Gazebo 仿真器
- **📡 实时通信**：Flask HTTP + AgentLace 分布式通信框架

## 🏗️ 系统架构

```
🖥️  GPU 训练服务器          │  📱 Franka 控制机（客户端）
┌─────────────────────┐     │  ┌─────────────────────────┐
│  ACT 训练服务器      │     │  │   ACT 环境适配器        │
│  - 原版 ACT 算法     │◄────┼──┤  - franka_act_env.py   │
│  - 模仿学习训练      │Agent│  │  - HTTP 客户端模式      │
│  - 模型参数分发      │Lace │  │  - 观测：qpos+images   │
└─────────────────────┘     │  │  - 动作：7D关节+1D夹爪  │
         ▲                  │  └─────────────────────────┘
         │ ZeroMQ           │             │ HTTP API
         │ (模型同步)        │             ▼
         │                  │  ┌─────────────────────────┐
         │                  │  │   SERL Flask 服务器     │
         │                  │  │  - Python ↔ ROS 桥接   │
         │                  │  │  - 机器人状态查询       │
         │                  │  │  - 控制命令执行         │
         │                  │  └─────────────────────────┘
                           │             │ ROS Topics
                           │             ▼
                           │  ┌─────────────────────────┐
                           │  │     franka_ros          │
                           │  │  - franka_gazebo        │
                           │  │  - franka_control       │
                           │  │  - franka_gripper       │
                           │  └─────────────────────────┘
```

## 📁 项目结构

```
frk_act_unified/
├── act_algo/                    # 原版 ACT 算法
│   ├── policy.py               # ACT策略网络
│   ├── detr/                   # DETR网络实现
│   ├── imitate_episodes.py     # 模仿学习训练
│   └── utils.py                # ACT工具函数
├── robot_servers/               # SERL Flask服务器
│   └── franka_server.py        # HTTP-ROS桥接服务器
├── robot_env/                   # 机器人环境
│   └── franka_act_env.py       # ACT专用环境接口
├── agentlace/                   # 分布式训练框架
│   ├── trainer.py              # TrainerServer/Client
│   └── data/                   # 数据传输协议
├── franka_ros/                  # franka_ros（Git子模块）
│   ├── franka_gazebo/          # Gazebo仿真
│   ├── franka_control/         # 机器人控制
│   └── franka_gripper/         # 夹爪控制
├── scripts/                     # 启动脚本
│   ├── start_franka_client.sh  # 客户端启动
│   ├── start_act_server.sh     # 服务器启动
│   └── setup_project.sh        # 项目初始化
├── configs/                     # 配置文件
│   ├── act_config.yaml         # ACT超参数
│   └── robot_config.yaml       # 机器人配置
├── train_act_distributed.py    # 主训练脚本
├── requirements.txt             # Python依赖
└── README.md                    # 项目文档
```

## 🚀 快速开始

### 1. 环境准备

```bash
# 克隆项目（如果从Git）
git clone <repository_url>
cd frk_act_unified

# 运行项目初始化
chmod +x scripts/setup_project.sh
./scripts/setup_project.sh

# 激活环境
source activate_env.sh
```

### 2. 分布式训练部署

#### 服务器端（GPU训练机）
```bash
# 启动 ACT 训练服务器
./scripts/start_act_server.sh [服务器IP] [端口] [任务名] [GPU_ID]

# 示例
./scripts/start_act_server.sh 0.0.0.0 5555 franka_cube_transfer 0
```

#### 客户端（Franka控制机）
```bash
# 启动 Franka 客户端（仿真模式）
./scripts/start_franka_client.sh [服务器IP] [机器人IP] [模式] [任务名]

# 示例 - 仿真模式
./scripts/start_franka_client.sh 10.16.49.124 172.16.0.2 sim franka_cube_transfer

# 示例 - 真机模式
./scripts/start_franka_client.sh 10.16.49.124 172.16.0.2 real franka_cube_transfer
```

### 3. 单机测试模式

```bash
# 终端1：启动服务器
python train_act_distributed.py --server --config configs/act_config.yaml

# 终端2：启动客户端
python train_act_distributed.py --client --server_ip 127.0.0.1 --config configs/act_config.yaml
```

## ⚙️ 配置说明

### ACT 超参数配置 (`configs/act_config.yaml`)
```yaml
act_params:
  hidden_dim: 512          # 模型隐藏层维度
  chunk_size: 100          # 动作序列长度
  kl_weight: 10.0          # KL散度权重
  batch_size: 8            # 批次大小
  num_epochs: 2000         # 训练轮数
  lr: 1e-5                 # 学习率

model_params:
  backbone: "resnet18"     # 视觉编码器
  camera_names: ["cam_high", "cam_low"]  # 相机列表

training_params:
  temporal_agg: true       # 时序聚合
  camera_input: true       # 相机输入
```

### 机器人配置 (`configs/robot_config.yaml`)
```yaml
robot:
  arm_id: "panda"         # 机器人ID
  dof: 7                  # 自由度
  gripper_range: [0.0, 0.08]  # 夹爪范围

control:
  frequency: 1000         # 控制频率(Hz)
  mode: "position"        # 控制模式
```

## 🤖 Franka 机器人规格

### 机械结构
- **自由度**：7DOF机械臂 + 2DOF平行夹爪
- **关节命名**：panda_joint1 到 panda_joint7
- **夹爪范围**：0.0-0.08m

### 控制接口
- **位置控制**：PositionJointInterface
- **速度控制**：VelocityJointInterface  
- **力矩控制**：EffortJointInterface
- **笛卡尔控制**：FrankaPoseCartesianInterface

### 观测空间
- **关节状态**：qpos(7) + qvel(7)
- **相机数据**：多相机图像支持
- **末端执行器**：位姿和夹爪状态

## 📊 训练监控

### 日志文件
- **服务器日志**：`logs/act_server_*.log`
- **客户端日志**：`/tmp/franka_act_client.log`

### 监控命令
```bash
# 查看训练进程
ps aux | grep train_act_distributed

# 监控GPU使用
watch -n 1 nvidia-smi

# 查看ROS话题
rostopic list

# 检查Flask服务器
curl http://127.0.0.1:5000/status
```

### 模型检查点
- **保存路径**：`checkpoints/act_epoch_*.pth`
- **保存频率**：每100个epoch

## 🛠️ 开发指南

### 添加新任务
1. 在 `configs/` 中创建任务配置文件
2. 在 `robot_env/` 中实现任务特定的环境
3. 更新启动脚本中的任务列表

### 自定义相机配置
```python
# 在 configs/act_config.yaml 中
model_params:
  camera_names: ["cam_high", "cam_low", "cam_wrist"]
```

### 调试模式
```bash
# 启用详细日志
export PYTHONPATH="${PYTHONPATH}:."
python train_act_distributed.py --server --config configs/act_config.yaml --verbose
```

## 🔧 故障排除

### 常见问题

1. **ROS连接失败**
   ```bash
   source /opt/ros/noetic/setup.bash
   export ROS_MASTER_URI=http://localhost:11311
   ```

2. **Flask服务器无响应**
   ```bash
   curl http://127.0.0.1:5000/status
   pkill -f franka_server.py
   ```

3. **GPU内存不足**
   - 减小 `batch_size` 参数
   - 使用 `torch.cuda.empty_cache()`

4. **网络连接问题**
   - 检查防火墙设置
   - 确认IP地址和端口正确

### 日志分析
```bash
# 查看详细错误信息
tail -f logs/act_server_*.log | grep ERROR

# 检查网络连接
netstat -tlnp | grep 5555
```

## 📚 参考文档

### 技术文档
- **控制接口规格**：`docs/frkdocs/Franka Control Interface Specifications.markdown`
- **franka_ros文档**：`docs/frkdocs/franka_ros Documentation.markdown`
- **libfranka文档**：`docs/frkdocs/libfranka Documentation.markdown`

### 相关论文
- [ACT: Action Chunking with Transformers](https://arxiv.org/abs/2304.13705)
- [SERL: A Software Suite for Sample-Efficient Robotic Reinforcement Learning](https://serl-robot.github.io/)

## 🤝 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/new-feature`)
3. 提交更改 (`git commit -am 'Add new feature'`)
4. 推送到分支 (`git push origin feature/new-feature`)
5. 创建 Pull Request

## 📄 许可证

本项目基于 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 🙏 致谢

- [Franka Emika](https://www.franka.de/) - Franka 机器人平台
- [SERL Team](https://serl-robot.github.io/) - SERL 架构和工具
- [ACT Authors](https://arxiv.org/abs/2304.13705) - ACT 算法
- [AgentLace](https://github.com/youliangtan/agentlace) - 分布式训练框架

---

**版本**：v1.0  
**更新时间**：2025-01-13  
**维护者**：Franka ACT 开发团队
