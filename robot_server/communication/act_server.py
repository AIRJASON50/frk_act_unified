#!/usr/bin/env python3
"""
ACT分布式推理服务器 - 基于AgentLace协议
充分利用RTX 4090 GPU性能，提供高效的ACT模型推理服务
"""

import sys
import os
import time
import argparse
import logging
import traceback
import base64
import numpy as np
import cv2
from pathlib import Path
from typing import Dict, List, Any, Optional

# 添加路径
current_dir = Path(__file__).parent
robot_server_dir = current_dir.parent
communication_dir = robot_server_dir.parent / "communication"

sys.path.append(str(robot_server_dir))
sys.path.append(str(communication_dir))
sys.path.append(str(communication_dir / "agentlace"))

# 导入AgentLace
from agentlace.trainer import TrainerServer, TrainerConfig

# 导入推理引擎
from inference.act_inference import ACTInferenceEngine

class ACTDistributedServer:
    """基于AgentLace的ACT分布式推理服务器"""
    
    def __init__(self, port: int = 5555, model_path: Optional[str] = None, log_level: str = "INFO"):
        self.port = port
        self.model_path = model_path
        self.server = None
        self.inference_engine = None
        
        # 统计信息
        self.start_time = time.time()
        self.request_count = 0
        self.error_count = 0
        
        # 设置日志
        logging.basicConfig(level=getattr(logging, log_level.upper()))
        self.logger = logging.getLogger(__name__)
        
        print(f"🤖 ACT分布式推理服务器初始化")
        print(f"   端口: {port}")
        print(f"   模型路径: {model_path or '默认路径'}")
    
    def start(self) -> bool:
        """启动推理服务器"""
        try:
            print("🚀 启动ACT分布式推理服务器...")
            
            # 1. 初始化推理引擎
            print("🤖 初始化ACT推理引擎...")
            self.inference_engine = ACTInferenceEngine(self.model_path, device="cuda")
            
            if not self.inference_engine.load_model():
                print("❌ ACT模型加载失败")
                return False
            
            # 2. 创建AgentLace配置
            config = TrainerConfig(
                port_number=self.port,
                broadcast_port=self.port + 1,
                request_types=["inference", "server_status"],
                rate_limit=1000,
                version="0.0.2"
            )
            
            # 3. 创建AgentLace服务器
            print("🌐 创建AgentLace服务器...")
            self.server = TrainerServer(
                config=config,
                request_callback=self._handle_request
            )
            
            print("✅ ACT分布式推理服务器启动成功!")
            print(f"   服务地址: 10.16.49.124:{self.port}")
            print(f"   协议版本: AgentLace v{config.version}")
            print("🎯 等待客户端连接...")
            
            # 4. 启动服务器（阻塞模式）
            self.server.start()
            
            # 保持服务器运行
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("🛑 收到停止信号...")
                self.stop()
                
            return True
            
        except Exception as e:
            print(f"❌ 服务器启动失败: {e}")
            traceback.print_exc()
            return False
    
    def _handle_request(self, request_type: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        """处理AgentLace请求"""
        if request_type == "inference":
            return self._handle_inference_request(payload)
        elif request_type == "server_status":
            return self._handle_status_request(payload)
        else:
            return {"success": False, "error": f"Unknown request type: {request_type}"}
    
    def _handle_inference_request(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """处理推理请求"""
        start_time = time.time()
        self.request_count += 1
        request_id = payload.get("request_id", f"req_{self.request_count}")
        
        try:
            # 解析输入数据
            qpos = np.array(payload["qpos"], dtype=np.float32)
            qvel = np.array(payload.get("qvel", [0.0] * 8), dtype=np.float32)
            
            # 解码图像
            image_b64 = payload["image"]
            image_bytes = base64.b64decode(image_b64)
            image_array = np.frombuffer(image_bytes, dtype=np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            print(f"📨 收到推理请求 #{self.request_count}")
            print(f"   qpos: {qpos.shape}, 范围: [{qpos.min():.3f}, {qpos.max():.3f}]")
            print(f"   image: {image.shape}")
            
            # 执行推理
            actions, inference_time = self.inference_engine.predict(qpos, image)
            
            # 计算总延迟
            total_latency = (time.time() - start_time) * 1000
            
            print(f"✅ 推理完成")
            print(f"   推理耗时: {inference_time:.2f}ms")
            print(f"   总体延迟: {total_latency:.2f}ms")
            print(f"   动作范围: [{actions.min():.3f}, {actions.max():.3f}]")
            
            return {
                "success": True,
                "actions": actions.tolist(),
                "latency_ms": total_latency,
                "inference_ms": inference_time,
                "request_id": request_id
            }
            
        except Exception as e:
            self.error_count += 1
            error_msg = f"推理失败: {str(e)}"
            print(f"❌ {error_msg}")
            traceback.print_exc()
            
            return {
                "success": False,
                "error_msg": error_msg,
                "latency_ms": (time.time() - start_time) * 1000,
                "request_id": request_id
            }
    
    def _handle_status_request(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """处理状态查询请求"""
        try:
            uptime = time.time() - self.start_time
            engine_stats = self.inference_engine.get_stats() if self.inference_engine else {}
            
            status = {
                "success": True,
                "status": "running",
                "uptime_seconds": uptime,
                "request_count": self.request_count,
                "error_count": self.error_count,
                "success_rate": (self.request_count - self.error_count) / max(self.request_count, 1),
                "engine_stats": engine_stats
            }
            
            print(f"📊 状态查询: {self.request_count}次请求, {self.error_count}次错误")
            return status
            
        except Exception as e:
            return {"success": False, "error": str(e)}

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="ACT分布式推理服务器")
    parser.add_argument("--port", type=int, default=5555,
                       help="服务端口 (默认: 5555)")
    parser.add_argument("--model_path", type=str, default=None,
                       help="模型路径 (默认: 自动查找)")
    parser.add_argument("--log_level", type=str, default="INFO",
                       choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                       help="日志级别")
    
    args = parser.parse_args()
    
    # 创建并启动服务器
    server = ACTDistributedServer(
        port=args.port,
        model_path=args.model_path,
        log_level=args.log_level
    )
    
    try:
        server.start()
    except KeyboardInterrupt:
        print(f"\n🛑 收到中断信号，正在停止服务器...")
    except Exception as e:
        print(f"❌ 服务器运行错误: {e}")
        traceback.print_exc()
    finally:
        print("👋 服务器已停止")

if __name__ == "__main__":
    main()
