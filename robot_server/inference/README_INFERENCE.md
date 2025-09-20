# ACT分布式推理系统

基于训练好的Franka单臂ACT模型的分布式推理架构，使用agentlace协议实现客户端-服务器通信。

## 🏗️ 系统架构

```
┌─────────────────┐    agentlace     ┌─────────────────┐
│   客户端        │    Protocol      │   推理服务器    │
│   (仿真环境)    │ ◄─────────────► │   (GPU服务器)   │
└─────────────────┘                  └─────────────────┘
      │                                       │
      ├─ 机器人状态 (8维)                     ├─ ACT模型 (83.91M参数)
      ├─ 相机图像 (480×640×3)                ├─ CUDA推理加速
      └─ 实时控制 (50Hz)                     └─ 动作序列 (100步×8维)
```

## 📁 目录结构

```
robot_server/
├── inference/                    # 推理模块
│   └── act_inference.py         # ACT推理引擎
├── communication/               # 通信模块  
│   ├── act_server.py           # agentlace推理服务器
│   └── act_client.py           # 客户端示例
├── start_inference_server.sh  # 服务器启动脚本
└── README_INFERENCE.md        # 本文档

../communication/
├── act_protocol.py            # 通信协议定义
├── agentlace/                # agentlace通信库
└── robot_config.yaml        # 配置文件
```

## 🚀 快速开始

### 1. 启动推理服务器

```bash
# 进入服务器目录
cd robot_server/

# 启动服务器（默认端口5555）
./start_inference_server.sh

# 指定端口启动
./start_inference_server.sh --port 5556

# 测试模式
./start_inference_server.sh --test
```

### 2. 客户端连接测试

```bash
# 测试客户端连接
cd robot_server/communication/
python act_client.py
```

### 3. 验证推理功能

```bash
# 测试推理引擎
python inference/act_inference.py
```

## 🔧 配置说明

### 网络配置
- **服务器IP**: 10.16.49.124 (局域网)
- **默认端口**: 5555 (agentlace)
- **通信协议**: ZMQ REQ-REP
- **网络延迟**: <111ms (局域网)

### 模型配置
- **模型类型**: ACT Transformer
- **参数数量**: 83.91M
- **输入维度**: qpos(8) + image(480×640×3)
- **输出维度**: actions(100×8)
- **推理时间**: ~10ms (CUDA)

### 数据格式
- **状态**: 8维 (7关节 + 1夹爪)
- **图像**: 480×640×3 RGB
- **动作**: 100步×8维连续动作
- **频率**: 50Hz控制，0.5Hz推理

## 🌐 通信协议

### 推理接口 (`act_predict`)

**请求格式**:
```json
{
  "robot_state": {
    "qpos": [8个关节位置],
    "qvel": [8个关节速度], 
    "timestamp": 时间戳
  },
  "camera_data": {
    "image": "base64编码的JPEG图像",
    "timestamp": 时间戳
  },
  "request_id": "请求ID"
}
```

**响应格式**:
```json
{
  "actions": [[100步×8维动作序列]],
  "success": true,
  "latency_ms": 延迟时间,
  "request_id": "请求ID"
}
```

### 状态接口 (`server_status`)

查询服务器状态和性能统计。

## 🧪 测试验证

### 推理引擎测试
```bash
python inference/act_inference.py
```

预期输出：
```
✅ 推理成功!
   输入: qpos(8,), image(480, 640, 3)
   输出: actions(100, 8)
   耗时: ~10ms
   动作范围: [-0.5, 0.5]
```

### 通信测试
```bash
# 终端1: 启动服务器
./start_inference_server.sh

# 终端2: 测试客户端
python communication/act_client.py
```

## 📊 性能指标

### 推理性能
- **模型加载**: ~3秒
- **推理延迟**: 10-15ms (CUDA)
- **内存使用**: ~2GB GPU内存
- **吞吐量**: ~100 FPS

### 网络性能
- **往返延迟**: 46-111ms (局域网)
- **数据压缩**: JPEG ~50-100KB/帧
- **总体延迟**: <150ms (满足2秒缓冲要求)

## 🔍 故障排除

### 常见问题

1. **模型加载失败**
   ```bash
   # 检查模型文件
   ls -la /home/wujielin/CascadeProjects/data/act_training/checkpoints/franka_pick_place/
   
   # 验证环境
   conda activate aloha
   python -c "import torch; print(torch.cuda.is_available())"
   ```

2. **网络连接失败**
   ```bash
   # 检查端口
   netstat -an | grep 5555
   
   # 测试连接
   telnet 10.16.49.124 5555
   ```

3. **推理错误**
   ```bash
   # 检查输入格式
   python communication/act_client.py
   ```

### 日志调试
- 服务器日志：终端直接输出
- 详细错误：Python traceback
- 性能统计：`server_status`接口

## 🎯 使用示例

### Python客户端集成
```python
from act_client import ACTClient
import numpy as np

# 连接服务器
client = ACTClient("10.16.49.124", 5555)

# 推理循环
for step in range(1000):
    # 获取机器人状态和图像
    qpos = get_robot_state()  # (8,)
    image = get_camera_image()  # (480, 640, 3)
    
    # 请求动作序列
    actions = client.predict(qpos, image)  # (100, 8)
    
    # 执行动作
    for action in actions:
        robot.step(action)
        time.sleep(0.02)  # 50Hz
```

## 📈 扩展功能

1. **多客户端支持**: agentlace天然支持多客户端
2. **负载均衡**: 可部署多个推理服务器
3. **模型热更新**: 支持不停机更新模型
4. **监控面板**: 基于状态接口的可视化监控

---

**🎉 分布式ACT推理系统已就绪！**

*版本: v1.0 | 更新: 2025-09-20*
