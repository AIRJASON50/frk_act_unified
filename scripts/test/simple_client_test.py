#!/usr/bin/env python3
"""
简化版客户端测试Demo
测试与服务器的通信和数据传输
"""

import socket
import json
import time
import numpy as np
import threading
from typing import Dict, Any

class SimpleTrainingClient:
    """简化版训练客户端"""
    
    def __init__(self, server_ip='10.16.49.124', server_port=5555):
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.connected = False
        
    def connect(self):
        """连接到服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_ip, self.server_port))
            self.connected = True
            print(f"✓ 连接到服务器: {self.server_ip}:{self.server_port}")
            return True
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.socket:
            self.socket.close()
            self.connected = False
            print("📡 已断开连接")
    
    def send_request(self, request):
        """发送请求并接收响应"""
        if not self.connected:
            return None
            
        try:
            # 发送请求
            request_str = json.dumps(request)
            self.socket.send(request_str.encode('utf-8'))
            
            # 接收响应
            response_data = self.socket.recv(4096).decode('utf-8')
            response = json.loads(response_data.strip())
            
            return response
        except Exception as e:
            print(f"❌ 通信错误: {e}")
            return None
    
    def test_ping(self):
        """测试心跳连接"""
        request = {
            'type': 'ping',
            'timestamp': time.time()
        }
        
        start_time = time.time()
        response = self.send_request(request)
        end_time = time.time()
        
        if response and response['type'] == 'pong':
            latency = (end_time - start_time) * 1000  # ms
            print(f"✓ 心跳正常 - 延迟: {latency:.1f}ms")
            return True
        else:
            print("❌ 心跳失败")
            return False
    
    def test_prediction(self):
        """测试模型推理"""
        # 模拟Franka机器人观测数据
        fake_qpos = np.random.randn(7).tolist()  # 7D关节位置
        
        request = {
            'type': 'predict',
            'data': {
                'qpos': fake_qpos,
                'timestamp': time.time()
            }
        }
        
        start_time = time.time()
        response = self.send_request(request)
        end_time = time.time()
        
        if response and response['type'] == 'prediction':
            total_latency = (end_time - start_time) * 1000
            inference_time = response['inference_time'] * 1000
            network_latency = total_latency - inference_time
            
            print(f"✓ 推理成功:")
            print(f"  - 总延迟: {total_latency:.1f}ms")
            print(f"  - 推理时间: {inference_time:.1f}ms") 
            print(f"  - 网络延迟: {network_latency:.1f}ms")
            print(f"  - 动作维度: {response['chunk_size']}x{response['action_dim']}")
            return True
        else:
            print(f"❌ 推理失败: {response}")
            return False
    
    def run_continuous_test(self, duration=30):
        """连续测试"""
        print(f"🔄 开始连续测试 {duration}秒...")
        
        start_time = time.time()
        success_count = 0
        total_count = 0
        latencies = []
        
        while time.time() - start_time < duration:
            total_count += 1
            
            # 测试推理
            test_start = time.time()
            if self.test_prediction():
                success_count += 1
                latency = (time.time() - test_start) * 1000
                latencies.append(latency)
            
            # 等待一段时间再次测试
            time.sleep(0.1)  # 10Hz频率
        
        # 统计结果
        success_rate = success_count / total_count * 100
        avg_latency = np.mean(latencies) if latencies else 0
        max_latency = np.max(latencies) if latencies else 0
        min_latency = np.min(latencies) if latencies else 0
        
        print(f"\n📊 测试结果:")
        print(f"  - 成功率: {success_rate:.1f}% ({success_count}/{total_count})")
        print(f"  - 平均延迟: {avg_latency:.1f}ms")
        print(f"  - 延迟范围: {min_latency:.1f}ms - {max_latency:.1f}ms")

def main():
    print("========================================")
    print("Franka ACT 简化版客户端测试")
    print("========================================")
    
    # 服务器配置
    server_ip = input("服务器IP (默认10.16.49.124): ").strip()
    if not server_ip:
        server_ip = "10.16.49.124"
    
    # 创建客户端
    client = SimpleTrainingClient(server_ip=server_ip, server_port=5555)
    
    # 连接服务器
    if not client.connect():
        print("❌ 无法连接到服务器，请检查:")
        print("  1. 服务器是否已启动")
        print("  2. IP地址是否正确")
        print("  3. 网络连接是否正常")
        return
    
    try:
        # 基础测试
        print("\n🧪 基础连接测试...")
        if client.test_ping():
            print("✓ 网络连接正常")
        
        print("\n🧠 推理功能测试...")
        if client.test_prediction():
            print("✓ 推理功能正常")
        
        # 询问是否进行连续测试
        print("\n是否进行连续测试？(y/n): ", end="")
        if input().lower().startswith('y'):
            client.run_continuous_test(duration=30)
    
    except KeyboardInterrupt:
        print("\n🛑 测试中断")
    
    finally:
        client.disconnect()

if __name__ == '__main__':
    main()
