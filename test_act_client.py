#!/usr/bin/env python3
"""
ACT分布式推理客户端测试脚本
"""

import sys
import time
import numpy as np
import base64
import cv2
from pathlib import Path

# 添加路径
current_dir = Path(__file__).parent
communication_dir = current_dir / "communication"
sys.path.append(str(communication_dir))
sys.path.append(str(communication_dir / "agentlace"))

try:
    from agentlace.trainer import TrainerClient, TrainerConfig
    print("✅ AgentLace客户端导入成功")
except ImportError as e:
    print(f"❌ AgentLace导入失败: {e}")
    sys.exit(1)

class ACTTestClient:
    """ACT测试客户端"""
    
    def __init__(self, server_host="127.0.0.1", server_port=5555):
        self.server_host = server_host
        self.server_port = server_port
        self.client = None
        
    def connect(self):
        """连接服务器"""
        try:
            config = TrainerConfig(
                port_number=self.server_port,
                broadcast_port=self.server_port + 1,
                request_types=["inference", "server_status"],
                rate_limit=1000,
                version="0.0.2"
            )
            self.client = TrainerClient("act_test_client", self.server_host, config)
            print("✅ 已连接到ACT推理服务器")
            return True
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def create_test_data(self):
        """创建测试数据"""
        # 模拟Franka机器人状态 (7关节+1夹爪)
        qpos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04], dtype=np.float32)
        qvel = np.zeros(8, dtype=np.float32)
        
        # 创建模拟图像 (640x480)
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # 添加一些有意义的图案
        cv2.rectangle(image, (200, 150), (300, 250), [255, 100, 100], -1)
        cv2.circle(image, (400, 300), 50, [100, 255, 100], -1)
        cv2.putText(image, "ACT Test", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2)
        
        return qpos, qvel, image
    
    def run_inference_test(self, num_tests=3):
        """运行推理测试"""
        print(f"🎯 开始ACT推理测试 ({num_tests}次)")
        
        for i in range(num_tests):
            print(f"\n📨 测试 {i+1}/{num_tests}:")
            
            # 创建测试数据
            qpos, qvel, image = self.create_test_data()
            
            # 编码图像
            _, buffer = cv2.imencode('.jpg', image)
            image_b64 = base64.b64encode(buffer).decode('utf-8')
            
            # 构建请求
            payload = {
                "qpos": qpos.tolist(),
                "qvel": qvel.tolist(), 
                "image": image_b64,
                "request_id": f"test_{i}"
            }
            
            # 发送请求
            start_time = time.time()
            response = self.client.request("inference", payload)
            latency = (time.time() - start_time) * 1000
            
            if response and response.get("success"):
                actions = np.array(response["actions"])
                print(f"   ✅ 成功! 延迟: {latency:.1f}ms")
                print(f"   动作序列: {actions.shape}, 范围: [{actions.min():.3f}, {actions.max():.3f}]")
                print(f"   推理时间: {response.get('inference_ms', 0):.1f}ms")
            else:
                print(f"   ❌ 失败: {response}")
            
            time.sleep(0.5)

def main():
    print("🚀 ACT分布式推理客户端测试")
    print("="*50)
    
    client = ACTTestClient()
    
    if client.connect():
        client.run_inference_test(5)
        print("\n🎉 测试完成!")
    else:
        print("❌ 无法连接服务器，请先启动服务器")

if __name__ == "__main__":
    main()
