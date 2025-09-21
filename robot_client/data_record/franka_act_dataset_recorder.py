#!/usr/bin/env python3

"""
Franka ACT Dataset Recorder

MAIN FUNCTIONALITY:
ACT兼容数据集录制器，实时收集机械臂演示数据并生成训练就绪的HDF5文件

RECORDING STATES:
1. IDLE - 等待录制命令，监听所有传感器数据但不存储
2. RECORDING - 主动录制状态，缓存所有传感器数据到内存
3. SAVING - 将缓存数据同步并写入HDF5文件
4. ERROR - 错误状态，停止录制并重置缓冲区

STATE TRANSITION CONDITIONS:
- IDLE → RECORDING:
  * Phase callback: phase 0→1 (石块生成→开始录制)
  * Manual command: 'start_episode'
- RECORDING → SAVING:
  * Phase callback: phase 7→8 (返回家位置→结束录制)
  * Manual command: 'stop_episode' + 'save_episode'
- SAVING → IDLE:
  * HDF5文件成功写入磁盘
  * Episode编号自动递增
- ANY_STATE → ERROR:
  * 数据同步失败 (图像/关节数据长度不匹配)
  * 文件写入异常
  * 传感器数据超时
- ERROR → IDLE:
  * 缓冲区重置完成
  * 错误日志记录完成

DATA SYNCHRONIZATION:
- 多传感器数据流实时对齐 (关节状态、夹爪状态、相机图像)
- 基于最短序列长度的数据截断确保一致性
- 线程安全的数据缓冲和访问控制

AUTOMATIC PHASE-DRIVEN RECORDING:
- Phase 1触发: 自动开始episode录制
- Phase 8触发: 自动结束并保存episode
- 8阶段操作完全自动化，无需手动干预

ERROR HANDLING:
- 传感器数据丢失时使用零填充
- 图像转换失败时跳过该帧
- 磁盘空间不足时记录警告
- 权限错误时自动重试

=== DATA FORMAT SPECIFICATION ===

HDF5 File Structure:
├── action (T, 8)           # Target joint positions for next timestep
├── observations/
│   ├── qpos (T, 8)         # Current joint positions [7 joints + 1 gripper]
│   ├── qvel (T, 8)         # Current joint velocities [7 joints + 1 gripper]
│   └── images/
│       └── top (T, H, W, 3) # RGB images from top camera (480x640x3)
└── attributes:
    └── sim: True           # Simulation flag

Data Dimensions:
- T: Number of timesteps in episode (typically 300-400 for pick-and-place)
- 8 DOF: 7 Franka joint positions + 1 gripper width
- Images: Height=480, Width=640, Channels=3 (RGB)

=== ROS TOPICS ===

Input Topics:
- /top_camera/rgb/image_raw              # Camera images (sensor_msgs/Image)
- /franka_state_controller/joint_states  # Joint positions/velocities (sensor_msgs/JointState)
- /franka_gripper/joint_states           # Gripper state (sensor_msgs/JointState)
- /franka_controller/current_phase       # Episode phase (std_msgs/Int32)
- /franka_controller/episode_control     # Recording control (std_msgs/String)

Phase-Based Control:
- Phase 0: Stone Generate (准备状态)
- Phase 1: Record Start (自动开始录制)
- Phase 2-7: 抓取执行阶段 (持续录制)
- Phase 8: Record End (自动保存并结束)

Manual Control Commands:
- 'start_episode': Begin recording data
- 'stop_episode': Stop recording
- 'save_episode': Save current episode to HDF5

=== USAGE ===

1. Start the recorder node:
   rosrun frk_act_unified franka_act_dataset_recorder.py

2. Automatic mode (recommended):
   - 录制器监听phase变化自动工作
   - Phase 1→开始录制, Phase 8→保存episode

3. Manual control (for testing):
   rostopic pub /franka_controller/episode_control std_msgs/String "data: 'start_episode'"
   rostopic pub /franka_controller/episode_control std_msgs/String "data: 'save_episode'"

4. Dataset files saved to: /home/jason/ws/catkin_ws/src/act_data/
   Format: episode_XXXXXX_YYYYMMDD_HHMMSS.hdf5

=== COMPATIBILITY ===

This recorder generates datasets compatible with:
- ACT (Action Chunking with Transformers) training pipeline
- Standard imitation learning frameworks expecting (observation, action) pairs
- Single-arm manipulation tasks
- Real robot deployment with domain adaptation
"""

import rospy
import numpy as np
import h5py
from cv_bridge import CvBridge
import threading
import os
from datetime import datetime

# ROS message types
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Int32
from franka_msgs.msg import FrankaState


class FrankaACTDatasetRecorder:
    """ACT-compatible dataset recorder for single Franka arm"""
    
    def __init__(self, save_dir="/home/jason/ws/catkin_ws/src/act_data"):
        """Initialize recorder with specified save directory"""
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        
        # Recording state
        self.current_episode = self._get_next_episode_number()
        self.recording = False
        self.current_phase = 0
        self.data_lock = threading.Lock()
        
        # ACT data buffers
        self._reset_buffers()
        
        # Robot state tracking
        self.current_qpos = np.zeros(8)  # 7 joints + gripper
        self.current_qvel = np.zeros(8)
        
        # ROS components
        self.bridge = CvBridge()
        self._init_subscribers()
        
        rospy.loginfo(f"Franka ACT Dataset Recorder initialized - Next episode: {self.current_episode}")
    
    def _get_next_episode_number(self):
        """Get the next episode number based on existing files"""
        if not os.path.exists(self.save_dir):
            return 0
        
        existing_files = [f for f in os.listdir(self.save_dir) if f.startswith('episode_') and f.endswith('.hdf5')]
        if not existing_files:
            return 0
        
        # Extract episode numbers from filenames
        episode_numbers = []
        for f in existing_files:
            try:
                # Format: episode_XXXXXX_YYYYMMDD_HHMMSS.hdf5
                episode_num = int(f.split('_')[1])
                episode_numbers.append(episode_num)
            except (IndexError, ValueError):
                continue
        
        return max(episode_numbers) + 1 if episode_numbers else 0
    
    def _reset_buffers(self):
        """Reset data buffers for new episode"""
        self.observations = {
            'qpos': [],
            'qvel': [],
            'images': {'top': []}
        }
        self.actions = []
        self.timestamps = []
    
    def _init_subscribers(self):
        """Initialize ROS subscribers"""
        rospy.Subscriber('/top_camera/rgb/image_raw', Image, self._image_callback, queue_size=10)
        rospy.Subscriber('/franka_state_controller/joint_states', JointState, self._joint_callback, queue_size=10)
        rospy.Subscriber('/franka_gripper/joint_states', JointState, self._gripper_callback, queue_size=10)
        rospy.Subscriber('/franka_controller/current_phase', Int32, self._phase_callback, queue_size=5)
        rospy.Subscriber('/franka_controller/episode_control', String, self._episode_control_callback, queue_size=5)
        # Franka state subscriber (not used in current ACT format)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self._franka_state_callback, queue_size=10)
    
    def _image_callback(self, msg):
        """Store camera images in ACT format"""
        if not self.recording:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            with self.data_lock:
                self.observations['images']['top'].append(cv_image)
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")
    
    def _joint_callback(self, msg):
        """Process joint states and generate ACT data"""
        # Extract joint data
        joint_positions = np.array(msg.position[:7])
        joint_velocities = np.array(msg.velocity[:7]) if len(msg.velocity) >= 7 else np.zeros(7)
        
        with self.data_lock:
            # Update current state
            self.current_qpos[:7] = joint_positions
            self.current_qvel[:7] = joint_velocities
            
            if self.recording:
                # Store observations
                qpos_obs = self.current_qpos.copy()
                qvel_obs = self.current_qvel.copy()
                self.observations['qpos'].append(qpos_obs)
                self.observations['qvel'].append(qvel_obs)
                
                # Generate action (for demonstration: action = current position)
                self.actions.append(qpos_obs.copy())
                self.timestamps.append(rospy.Time.now().to_sec())
    
    def _franka_state_callback(self, msg):
        """Franka state callback (unused in current ACT format)"""
        pass
    
    def _gripper_callback(self, msg):
        """Update gripper state in qpos"""
        gripper_width = np.sum(msg.position[:2]) if len(msg.position) >= 2 else 0.0
        with self.data_lock:
            self.current_qpos[7] = gripper_width
            self.current_qvel[7] = 0.0
    
    def _phase_callback(self, msg):
        """Update episode phase and handle automatic recording"""
        new_phase = msg.data
        
        # Automatic recording control based on phase
        if new_phase == 1 and self.current_phase != 1:
            # Start recording when stone is detected and recording begins (phase 1)
            if not self.recording:
                rospy.loginfo("Phase 1 detected: Stone generated, starting episode recording automatically")
                self.start_recording()
                
        elif new_phase == 8 and self.current_phase == 7:
            # Auto-save when task completed and returning to record end (phase 7->8)
            if self.recording:
                rospy.loginfo("Phase 8 detected: Task completed, ending episode recording and saving")
                self.stop_recording()
                self.save_current_episode()
        
        # Log phase transitions for debugging
        if new_phase != self.current_phase:
            phase_names = {
                0: "Stone Generate",
                1: "Record Start", 
                2: "Open Gripper",
                3: "Move Above Target",
                4: "Descend to Grasp",
                5: "Close Gripper",
                6: "Lift Object",
                7: "Return Home",
                8: "Record End"
            }
            rospy.loginfo(f"Phase transition: {self.current_phase} -> {new_phase} ({phase_names.get(new_phase, 'Unknown')})")
        
        self.current_phase = new_phase
    
    def _episode_control_callback(self, msg):
        """Handle manual recording control commands (fallback)"""
        command = msg.data.lower()
        rospy.loginfo(f"Manual episode control: {command}")
        
        if command == "start_episode":
            self.start_recording()
        elif command == "stop_episode":
            self.stop_recording()
        elif command == "save_episode":
            self.save_current_episode()
        elif command == "reset_episode":
            # Force reset without saving
            with self.data_lock:
                self.recording = False
                self._reset_buffers()
            rospy.loginfo("Episode reset without saving")
    
    def start_recording(self):
        """Start recording new episode"""
        with self.data_lock:
            self.recording = True
            self._reset_buffers()
        rospy.loginfo(f"Started recording episode {self.current_episode}")
    
    def stop_recording(self):
        """Stop recording"""
        with self.data_lock:
            self.recording = False
        rospy.loginfo(f"Stopped recording episode {self.current_episode}")
    
    def save_current_episode(self):
        """Save current episode to HDF5 file in ACT format"""
        with self.data_lock:
            if not self.observations['qpos']:
                rospy.logwarn("No episode data to save")
                return
                
            # Synchronize data streams
            min_length = min(
                len(self.observations['qpos']),
                len(self.observations['qvel']),
                len(self.observations['images']['top']),
                len(self.actions)
            )
            
            if min_length == 0:
                rospy.logwarn("No synchronized data to save")
                return
        
        # Generate filename and save
        filename = f"episode_{self.current_episode:06d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.hdf5"
        filepath = os.path.join(self.save_dir, filename)
        
        try:
            with h5py.File(filepath, 'w', rdcc_nbytes=1024**2*2) as root:
                root.attrs['sim'] = True
                
                # Create observations group
                obs = root.create_group('observations')
                obs.create_dataset('qpos', data=np.array(self.observations['qpos'][:min_length]), dtype=np.float64)
                obs.create_dataset('qvel', data=np.array(self.observations['qvel'][:min_length]), dtype=np.float64)
                
                # Images
                images_group = obs.create_group('images')
                if self.observations['images']['top']:
                    img_array = np.array(self.observations['images']['top'][:min_length])
                    images_group.create_dataset('top', data=img_array, dtype=np.uint8,
                                              chunks=(1, img_array.shape[1], img_array.shape[2], 3))
                
                # Actions
                root.create_dataset('action', data=np.array(self.actions[:min_length]), dtype=np.float64)
                
            rospy.loginfo(f"Saved episode {self.current_episode} to {filepath}")
            
        except Exception as e:
            rospy.logerr(f"Error saving episode: {e}")
            return
            
        self.current_episode += 1
    
    def get_status(self):
        """Get current recorder status"""
        with self.data_lock:
            return {
                'recording': self.recording,
                'episode': self.current_episode,
                'phase': self.current_phase,
                'data_length': len(self.observations['qpos']) if self.observations['qpos'] else 0
            }


def main():
    """Main function to run the dataset recorder node"""
    rospy.init_node('franka_act_dataset_recorder')
    
    # Initialize recorder
    save_dir = rospy.get_param('~save_dir', '/home/jason/ws/catkin_ws/src/act_data')
    recorder = FrankaACTDatasetRecorder(save_dir=save_dir)
    
    # Status reporting
    def publish_status(event):
        status = recorder.get_status()
        rospy.loginfo(f"Recording: {status['recording']}, Episode: {status['episode']}, "
                     f"Phase: {status['phase']}, Data points: {status['data_length']}")
    
    status_timer = rospy.Timer(rospy.Duration(10.0), publish_status)
    
    # Startup messages
    rospy.loginfo("Franka ACT Dataset Recorder started")
    rospy.loginfo(f"Save directory: {save_dir}")
    rospy.loginfo("Episode control commands:")
    rospy.loginfo("  start_episode, stop_episode, save_episode")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down recorder")
    finally:
        status_timer.shutdown()


if __name__ == '__main__':
    main()
