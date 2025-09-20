#!/usr/bin/env python3
"""
简化版服务器端测试Demo
测试模型加载、GPU推理和网络通信
"""

import torch
import torch.nn as nn
import socket
import threading
import json
import time
import numpy as np
from typing import Dict, Any

class SimpleACTModel(nn.Module):
    """简化版ACT模型，用于测试"""
    def __init__(self, obs_dim=7, action_dim=8, chunk_size=100, hidden_dim=256):
        super().__init__()
        self.chunk_size = chunk_size
        
        # 简化的编码器
        self.encoder = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )
        
        # 简化的解码器
        self.decoder = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim * chunk_size)
        )
        
    def forward(self, obs):
        # obs: [batch_size, obs_dim]
        encoded = self.encoder(obs)
        decoded = self.decoder(encoded)
        # 重塑为动作序列: [batch_size, chunk_size, action_dim]
        actions = decoded.view(-1, self.chunk_size, 8)
        return actions

class SimpleTrainingServer:
    """简化版训练服务器"""
    
    def __init__(self, port=5555, device='cuda'):
        self.port = port
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.model = SimpleACTModel().to(self.device)
        self.running = False
        
        print(f"🖥️  初始化服务器 - 设备: {self.device}")
        
    def start_server(self):
        """启动服务器"""
        self.running = True
        
        # 创建socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', self.port))
        server_socket.listen(5)
        
        print(f"🚀 服务器启动在端口 {self.port}")
        
        while self.running:
            try:
                client_socket, addr = server_socket.accept()
                print(f"📡 客户端连接: {addr}")
                
                # 启动客户端处理线程
                client_thread = threading.Thread(
                    target=self.handle_client, 
                    args=(client_socket, addr)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                print(f"❌ 服务器错误: {e}")
                break
                
        server_socket.close()
        
    def handle_client(self, client_socket, addr):
        """处理客户端请求"""
        try:
            while self.running:
                # 接收数据
                data = client_socket.recv(4096).decode('utf-8')
                if not data:
                    break
                    
                # 解析请求
                request = json.loads(data)
                
                if request['type'] == 'predict':
                    # 执行推理
                    response = self.predict(request['data'])
                    
                elif request['type'] == 'ping':
                    # 心跳检测
                    response = {'type': 'pong', 'timestamp': time.time()}
                    
                else:
                    response = {'type': 'error', 'message': 'Unknown request type'}
                
                # 发送响应
                response_str = json.dumps(response) + '\n'
                client_socket.send(response_str.encode('utf-8'))
                
        except Exception as e:
            print(f"❌ 客户端处理错误 {addr}: {e}")
        finally:
            client_socket.close()
            print(f"📡 客户端断开: {addr}")
    
    def predict(self, obs_data):
        """模型推理"""
        try:
            # 转换观测数据
            obs = torch.tensor(obs_data['qpos'], dtype=torch.float32).unsqueeze(0).to(self.device)
            
            # 模型推理
            with torch.no_grad():
                start_time = time.time()
                actions = self.model(obs)
                inference_time = time.time() - start_time
            
            # 转换结果
            actions_np = actions.cpu().numpy()[0]  # [chunk_size, action_dim]
            
            return {
                'type': 'prediction',
                'actions': actions_np.tolist(),
                'inference_time': inference_time,
                'chunk_size': actions_np.shape[0],
                'action_dim': actions_np.shape[1]
            }
            
        except Exception as e:
            return {
                'type': 'error',
                'message': f'推理错误: {str(e)}'
            }

def main():
    print("========================================")
    print("Franka ACT 简化版服务器测试")
    print("========================================")
    
    # 检查GPU
    if torch.cuda.is_available():
        print(f"✓ GPU可用: {torch.cuda.get_device_name(0)}")
        device = 'cuda'
    else:
        print("⚠️  使用CPU运行")
        device = 'cpu'
    
    # 启动服务器
    server = SimpleTrainingServer(port=5555, device=device)
    
    try:
        server.start_server()
    except KeyboardInterrupt:
        print("\n🛑 服务器停止")
        server.running = False

if __name__ == '__main__':
    main()
