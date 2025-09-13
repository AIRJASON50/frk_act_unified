"""
Franka ACT Environment Adapter
基于 SERL Flask HTTP 通信的 ACT 环境适配器

参考计划文档 lines 336-361：
- 使用 HTTP 客户端连接 Flask 服务器
- 适配 ACT 观测空间（qpos + images）和动作空间（7D关节 + 1D夹爪）
- 复用 SERL 机器人状态查询和控制接口
"""

import gymnasium as gym
import numpy as np
import requests
import json
import base64
import cv2
from gymnasium import spaces
from typing import Dict, Any, Optional, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FrankaACTEnv(gym.Env):
    """Franka ACT 环境，使用 SERL Flask HTTP 通信"""
    
    def __init__(self, 
                 server_url: str = "http://127.0.0.1:5000",
                 camera_names: list = ["cam_high", "cam_low"],
                 max_episode_steps: int = 500,
                 control_frequency: int = 10):
        """
        初始化 Franka ACT 环境
        
        Args:
            server_url: Flask 服务器地址
            camera_names: 相机名称列表
            max_episode_steps: 最大episode步数
            control_frequency: 控制频率(Hz)
        """
        super().__init__()
        
        self.server_url = server_url.rstrip('/')
        self.camera_names = camera_names
        self.max_episode_steps = max_episode_steps
        self.control_frequency = control_frequency
        self.current_step = 0
        
        # ACT 观测空间定义（参考计划 lines 349-361）
        self.observation_space = spaces.Dict({
            'qpos': spaces.Box(low=-np.inf, high=np.inf, shape=(7,), dtype=np.float32),  # 关节位置
            'qvel': spaces.Box(low=-np.inf, high=np.inf, shape=(7,), dtype=np.float32),  # 关节速度
            'images': spaces.Dict({
                name: spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8)
                for name in camera_names
            })
        })
        
        # ACT 动作空间定义（根据Franka规格：7D关节 + 1D夹爪）
        # 参考franka_ros文档中的关节限制和夹爪规格
        joint_limits_low = np.array([
            -2.8973,  # joint1: ±166°
            -1.7628,  # joint2: ±101° 
            -2.8973,  # joint3: ±166°
            -3.0718,  # joint4: ±176°
            -2.8973,  # joint5: ±166°
            -0.0175,  # joint6: ±1°
            -2.8973   # joint7: ±166°
        ])
        joint_limits_high = np.array([
            2.8973,   # joint1: ±166°
            1.7628,   # joint2: ±101°
            2.8973,   # joint3: ±166°
            -0.0698,  # joint4: ±176°
            2.8973,   # joint5: ±166°
            3.7525,   # joint6: ±215°
            2.8973    # joint7: ±166°
        ])
        # 夹爪限制：0.0-0.08m（根据Franka技术规格）
        gripper_limits = np.array([0.0, 0.08])
        
        self.action_space = spaces.Box(
            low=np.concatenate([joint_limits_low, [gripper_limits[0]]]),
            high=np.concatenate([joint_limits_high, [gripper_limits[1]]]),
            dtype=np.float32
        )
        
        self._last_observation = None
        
        logger.info(f"FrankaACTEnv initialized with server: {server_url}")
        logger.info(f"Camera names: {camera_names}")
        
    def reset(self) -> Dict[str, Any]:
        """重置环境"""
        self.current_step = 0
        
        try:
            # 调用 Flask 服务器重置接口
            response = requests.post(f"{self.server_url}/reset", timeout=10)
            response.raise_for_status()
            
            # 获取初始观测
            observation = self._get_observation()
            self._last_observation = observation
            
            logger.info("Environment reset successfully")
            return observation
            
        except requests.RequestException as e:
            logger.error(f"Failed to reset environment: {e}")
            # 返回默认观测
            return self._get_default_observation()
    
    def step(self, action: np.ndarray) -> Tuple[Dict[str, Any], float, bool, Dict[str, Any]]:
        """执行动作"""
        self.current_step += 1
        
        try:
            # 分离关节动作和夹爪动作
            joint_action = action[:7].tolist()
            gripper_action = float(action[7])
            
            # 发送动作到 Flask 服务器（符合franka_ros接口）
            action_data = {
                'joint_positions': joint_action,  # 7个关节位置（panda_joint1-7）
                'gripper_position': gripper_action,  # 夹爪开合度（0-0.08m）
                'control_mode': 'position',  # 位置控制模式
                'arm_id': 'panda'  # 机器人ID（默认panda）
            }
            
            response = requests.post(
                f"{self.server_url}/step",
                json=action_data,
                timeout=5
            )
            response.raise_for_status()
            
            # 获取新观测
            observation = self._get_observation()
            self._last_observation = observation
            
            # 计算奖励（ACT 使用模仿学习，奖励通常为0）
            reward = 0.0
            
            # 检查episode是否结束
            done = self.current_step >= self.max_episode_steps
            
            info = {
                'step': self.current_step,
                'max_steps': self.max_episode_steps
            }
            
            return observation, reward, done, info
            
        except requests.RequestException as e:
            logger.error(f"Failed to execute step: {e}")
            # 返回上一个观测
            observation = self._last_observation or self._get_default_observation()
            return observation, 0.0, True, {'error': str(e)}
    
    def _get_observation(self) -> Dict[str, Any]:
        """获取当前观测"""
        try:
            # 获取机器人状态
            state_response = requests.get(f"{self.server_url}/get_state", timeout=5)
            state_response.raise_for_status()
            state_data = state_response.json()
            
            # 获取相机图像
            images = {}
            for camera_name in self.camera_names:
                try:
                    img_response = requests.get(
                        f"{self.server_url}/get_image/{camera_name}",
                        timeout=5
                    )
                    img_response.raise_for_status()
                    img_data = img_response.json()
                    
                    # 解码 base64 图像
                    img_bytes = base64.b64decode(img_data['image'])
                    img_array = np.frombuffer(img_bytes, dtype=np.uint8)
                    image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    # 确保图像尺寸正确
                    if image is not None:
                        image = cv2.resize(image, (640, 480))
                        images[camera_name] = image
                    else:
                        # 使用默认图像
                        images[camera_name] = np.zeros((480, 640, 3), dtype=np.uint8)
                        
                except Exception as e:
                    logger.warning(f"Failed to get image from {camera_name}: {e}")
                    images[camera_name] = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # 构建符合ACT格式的观测（基于Franka RobotState）
            observation = {
                'qpos': np.array(state_data.get('joint_positions', [0.0] * 7), dtype=np.float32),  # panda_joint1-7位置
                'qvel': np.array(state_data.get('joint_velocities', [0.0] * 7), dtype=np.float32),  # panda_joint1-7速度
                'images': images,  # 多相机图像数据
                # 可选：添加Franka特有的状态信息
                'ee_pose': np.array(state_data.get('ee_pose', np.eye(4).flatten()), dtype=np.float32),  # 末端执行器位姿 O_T_EE
                'gripper_width': np.array([state_data.get('gripper_width', 0.0)], dtype=np.float32)  # 夹爪开合度
            }
            
            return observation
            
        except requests.RequestException as e:
            logger.error(f"Failed to get observation: {e}")
            return self._get_default_observation()
    
    def _get_default_observation(self) -> Dict[str, Any]:
        """获取默认观测（用于错误处理）"""
        return {
            'qpos': np.zeros(7, dtype=np.float32),
            'qvel': np.zeros(7, dtype=np.float32),
            'images': {
                name: np.zeros((480, 640, 3), dtype=np.uint8)
                for name in self.camera_names
            },
            'ee_pose': np.eye(4, dtype=np.float32).flatten(),
            'gripper_width': np.array([0.0], dtype=np.float32)
        }
    
    def close(self):
        """关闭环境"""
        try:
            requests.post(f"{self.server_url}/close", timeout=5)
            logger.info("Environment closed")
        except requests.RequestException as e:
            logger.error(f"Failed to close environment: {e}")
    
    def get_server_status(self) -> Dict[str, Any]:
        """获取服务器状态"""
        try:
            response = requests.get(f"{self.server_url}/status", timeout=5)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            logger.error(f"Failed to get server status: {e}")
            return {'status': 'unknown', 'error': str(e)}


# 环境工厂函数
def make_franka_act_env(server_url: str = "http://127.0.0.1:5000", **kwargs) -> FrankaACTEnv:
    """创建 Franka ACT 环境"""
    return FrankaACTEnv(server_url=server_url, **kwargs)


# 测试函数
def test_environment():
    """测试环境连接"""
    env = FrankaACTEnv()
    
    print("Testing Franka ACT Environment...")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    
    try:
        # 测试服务器状态
        status = env.get_server_status()
        print(f"Server status: {status}")
        
        # 测试重置
        obs = env.reset()
        print(f"Reset successful, observation keys: {obs.keys()}")
        
        # 测试动作
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        print(f"Step successful, reward: {reward}, done: {done}")
        
        env.close()
        print("Test completed successfully!")
        
    except Exception as e:
        print(f"Test failed: {e}")


if __name__ == "__main__":
    test_environment()
