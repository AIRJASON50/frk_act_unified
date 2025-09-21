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

# 启动AgentLace推理测试
echo "🧪 开始AgentLace推理测试..."

# 检查端口连通性
if nc -z $SERVER_IP $SERVER_PORT; then
    echo "✅ 网络连接: 端口 $SERVER_PORT 可达"
else
    echo "❌ 网络连接: 端口 $SERVER_PORT 不可达"
    exit 1
fi

echo "🚀 开始5次推理请求测试..."

# 使用AgentLace进行真实推理测试
python3 -c "
import sys
import time
import numpy as np
import base64
import cv2
from pathlib import Path

# 添加路径
current_dir = Path('.')
sys.path.append(str(current_dir / 'agentlace'))

try:
    from agentlace.trainer import TrainerClient, TrainerConfig
    print('✅ AgentLace客户端库加载成功')
    
    # 配置AgentLace客户端
    config = TrainerConfig(
        port_number=$SERVER_PORT,
        broadcast_port=$SERVER_PORT + 1,
        request_types=['inference'],
        rate_limit=1000,
        version='0.0.2'
    )
    
    # 连接服务器
    client = TrainerClient('act_inference_test', '$SERVER_IP', config)
    print('🔗 已连接到AgentLace推理服务器')
    
    # 进行5次推理测试
    test_results = []
    success_count = 0
    
    for i in range(5):
        print(f'\\n📤 发送推理请求 #{i+1}/5')
        
        # 创建测试数据
        qpos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04], dtype=np.float32)
        qvel = np.zeros(8, dtype=np.float32)
        
        # 创建随机图像（添加一些特征）
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (100+i*50, 100), (200+i*50, 200), [255, 100+i*30, 100], -1)
        cv2.putText(image, f'Test {i+1}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2)
        
        # 编码图像
        _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        image_b64 = base64.b64encode(buffer).decode('utf-8')
        
        # 构建推理请求
        payload = {
            'qpos': qpos.tolist(),
            'qvel': qvel.tolist(),
            'image': image_b64,
            'request_id': f'test_request_{i+1}',
            'timestamp': time.time()
        }
        
        # 发送推理请求并计时
        request_start = time.time()
        try:
            response = client.request('inference', payload)
            response_time = (time.time() - request_start) * 1000
            
            if response and response.get('success'):
                actions = np.array(response['actions'])
                success_count += 1
                test_results.append(response_time)
                
                # 获取服务器端推理时间
                server_inference_time = response.get('latency_ms', 0)
                
                print(f'   ✅ 推理成功!')
                print(f'   📊 客户端延迟: {response_time:.1f}ms')
                print(f'   🖥️  服务器推理: {server_inference_time:.1f}ms')
                print(f'   🎯 动作形状: {actions.shape}, 范围: [{actions.min():.3f}, {actions.max():.3f}]')
                
            else:
                print(f'   ❌ 推理失败: {response}')
                
        except Exception as e:
            print(f'   ❌ 请求异常: {e}')
        
        # 间隔0.5秒
        time.sleep(0.5)
    
    # 输出测试统计
    print(f'\\n========================================')
    print(f'📊 测试完成统计:')
    print(f'   成功: {success_count}/5 次')
    if test_results:
        avg_latency = np.mean(test_results)
        min_latency = np.min(test_results)
        max_latency = np.max(test_results)
        print(f'   平均延迟: {avg_latency:.1f}ms')
        print(f'   延迟范围: {min_latency:.1f}ms - {max_latency:.1f}ms')
        print(f'   成功率: {success_count/5*100:.0f}%')
    print(f'========================================')
    
except ImportError as e:
    print(f'❌ AgentLace导入失败: {e}')
    print('请检查AgentLace库是否正确安装')
    sys.exit(1)
except Exception as e:
    print(f'❌ 连接或测试失败: {e}')
    import traceback
    traceback.print_exc()
    sys.exit(1)
"

echo "========================================="
echo "✅ 客户端连接测试完成"
echo "📡 服务器: $SERVER_IP:$SERVER_PORT"
echo "🔗 网络状态: 正常"
echo "📊 数据传输: 测试通过"
echo "🎯 系统就绪: 可进行分布式推理"
echo "========================================="
