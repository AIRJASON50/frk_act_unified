#!/usr/bin/env python3
"""
ACT分布式推理服务器AgentLace延迟测试
验证AgentLace协议是否正常部署并测量网络延迟
"""

import sys
import time
import numpy as np
import base64
import cv2
import statistics
from pathlib import Path
from typing import List, Dict, Any

# 添加路径
current_dir = Path(__file__).parent
communication_dir = current_dir.parent / "communication"
sys.path.append(str(communication_dir))
sys.path.append(str(communication_dir / "agentlace"))

# 导入AgentLace
try:
    from agentlace.trainer import TrainerClient, TrainerConfig
    print("✅ AgentLace导入成功")
except ImportError as e:
    print(f"❌ AgentLace导入失败: {e}")
    print("请运行: pip install -e communication/agentlace")
    sys.exit(1)

def test_agentlace_connection():
    """测试AgentLace协议连接"""
    print("🔗 测试AgentLace协议连接...")
    
    try:
        # 创建配置
        config = TrainerConfig(
            port_number=5555,
            broadcast_port=5556,
            request_types=["inference", "server_status"],
            rate_limit=1000,
            version="0.0.2"
        )
        
        # 创建客户端 (使用本地环回地址进行单机测试)
        client = TrainerClient("test_client", "127.0.0.1", config)
        print("✅ AgentLace客户端创建成功")
        
        # 测试多次连接延迟
        latencies = []
        for i in range(5):
            start_time = time.time()
            response = client.request("server_status", {})
            latency = (time.time() - start_time) * 1000
            latencies.append(latency)
            
            if response and response.get("success"):
                print(f"   测试 {i+1}/5: {latency:.2f}ms")
            else:
                print(f"   测试 {i+1}/5: 失败")
                return False, []
            
            time.sleep(0.1)  # 短暂间隔
        
        # 计算延迟统计
        avg_latency = statistics.mean(latencies)
        min_latency = min(latencies)
        max_latency = max(latencies)
        
        print(f"✅ AgentLace连接测试成功:")
        print(f"   平均延迟: {avg_latency:.2f}ms")
        print(f"   最小延迟: {min_latency:.2f}ms")
        print(f"   最大延迟: {max_latency:.2f}ms")
        print(f"   延迟抖动: {max_latency - min_latency:.2f}ms")
        
        return True, latencies
            
    except Exception as e:
        print(f"❌ AgentLace连接测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False, []

def test_inference_latency():
    """测试推理请求延迟"""
    print("🎯 测试推理延迟性能...")
    
    try:
        # 创建客户端
        config = TrainerConfig(
            port_number=5555,
            broadcast_port=5556,
            request_types=["inference", "server_status"],
            rate_limit=1000,
            version="0.0.2"
        )
        client = TrainerClient("inference_test_client", "127.0.0.1", config)
        
        # 测试不同大小的图像延迟
        image_sizes = [
            (240, 320, "小图像"),
            (480, 640, "标准图像"),
        ]
        
        for height, width, desc in image_sizes:
            print(f"\n📊 {desc} ({height}×{width}) 延迟测试:")
            
            latencies = []
            for i in range(3):
                # 创建测试数据
                qpos = np.random.randn(8).astype(np.float32) * 0.1
                image = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
                
                # 编码图像为JPEG
                _, buffer = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                image_b64 = base64.b64encode(buffer).decode('utf-8')
                
                # 构建推理请求
                payload = {
                    "qpos": qpos.tolist(),
                    "qvel": [0.0] * 8,
                    "image": image_b64,
                    "timestamp": time.time(),
                    "request_id": f"latency_test_{i}"
                }
                
                # 测量延迟
                start_time = time.time()
                response = client.request("inference", payload)
                network_latency = (time.time() - start_time) * 1000
                latencies.append(network_latency)
                
                if response and response.get("success"):
                    print(f"   测试 {i+1}/3: 网络{network_latency:.1f}ms + 推理{response.get('inference_ms', 0):.1f}ms = 总计{response.get('latency_ms', 0):.1f}ms")
                    print(f"            图像大小: {len(image_b64)} bytes")
                else:
                    print(f"   测试 {i+1}/3: 失败 - {response}")
                    return False
                
                time.sleep(0.2)
            
            # 延迟统计
            avg_latency = statistics.mean(latencies)
            print(f"   平均网络延迟: {avg_latency:.2f}ms")
        
        return True
            
    except Exception as e:
        print(f"❌ 延迟测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主测试函数"""
    print("🚀 ACT分布式推理服务器 AgentLace 测试")
    print("=" * 60)
    
    print("📋 测试内容:")
    print("   1. AgentLace协议连接测试")
    print("   2. 网络延迟性能测试")
    print("   3. 推理功能延迟测试")
    print("")
    print("⚠️ 请确保服务器已启动:")
    print("   cd robot_server && ./start_inference_server.sh")
    print("")
    
    input("按回车键开始测试...")
    print("")
    
    # 测试AgentLace连接
    print("🔗 第一阶段: AgentLace协议测试")
    connection_ok, latencies = test_agentlace_connection()
    print("")
    
    if connection_ok:
        # 测试推理延迟
        print("⚡ 第二阶段: 推理延迟测试")
        inference_ok = test_inference_latency()
        print("")
        
        # 延迟分析
        if latencies:
            avg_latency = statistics.mean(latencies)
            print("📊 延迟分析报告:")
            print(f"   AgentLace平均延迟: {avg_latency:.2f}ms")
            
            if avg_latency < 20:
                print("   ✅ 延迟优秀 (<20ms) - 适合实时控制")
            elif avg_latency < 50:
                print("   ✅ 延迟良好 (<50ms) - 适合ACT批量预测")
            else:
                print("   ⚠️ 延迟较高 (>50ms) - 可能影响实时性")
        
        # 总结
        print("")
        print("=" * 60)
        print("📋 测试结果总结:")
        print(f"   AgentLace连接: {'✅ 成功' if connection_ok else '❌ 失败'}")
        print(f"   推理功能: {'✅ 成功' if inference_ok else '❌ 失败'}")
        
        if connection_ok and inference_ok:
            print("")
            print("🎉 所有测试通过！AgentLace推理服务器工作正常！")
            print("💡 系统已准备就绪，可以开始客户端开发")
        else:
            print("")
            print("⚠️ 部分测试失败，请检查服务器配置")
    else:
        print("❌ AgentLace连接失败，请检查:")
        print("   1. 服务器是否已启动 (./start_inference_server.sh)")
        print("   2. 端口5555是否可用 (netstat -an | grep 5555)")
        print("   3. 防火墙设置")
        print("   4. AgentLace依赖是否正确安装")

if __name__ == "__main__":
    main()
