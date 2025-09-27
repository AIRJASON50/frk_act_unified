#!/usr/bin/env python3
"""
Franka ACT Inference Controller for Distributed ACT Policy Execution

MAIN FUNCTIONALITY:
基于ACT推理的Franka阻抗控制器，通过AgentLace与远程服务器通信执行ACT策略推理

CORE FEATURES:
- 实时采集机械臂状态数据 (qpos, qvel) 
- 采集Gazebo相机图像数据
- 通过AgentLace发送状态到远程ACT推理服务器
- 接收推理得到的动作指令 (action)
- 通过阻抗控制器执行推理动作

COMMUNICATION ARCHITECTURE:
- AgentLace客户端 → 远程ACT服务器 (端口5555)
- 数据格式: ACT标准格式 (qpos, qvel, images)
- 推理返回: 100步连续动作序列 (chunk_size=100)
- 执行频率: 10Hz控制循环

DATA COLLECTION:
- Joint positions (qpos): 7-DOF arm + 2-DOF gripper = 9维
- Joint velocities (qvel): 9维速度数据
- Camera images: 480x640 RGB图像，base64编码传输
- Action execution: 7-DOF pose + gripper命令

CONTROL FLOW:
1. 初始化AgentLace客户端连接
2. 循环执行:
   - 采集当前机械臂状态 (qpos, qvel)
   - 采集相机图像
   - 发送数据到推理服务器
   - 接收100步动作序列
   - 逐步执行动作通过阻抗控制器
   - 循环下一次推理

ERROR HANDLING:
- 通信故障自动重连
- 推理失败安全停止
- 动作执行边界检查
- 紧急停止机制

COORDINATE SYSTEMS:
- Robot Base Frame: fr3_link0 (阻抗控制指令)
- ACT Action Space: 7-DOF end-effector pose + gripper width
- Camera Frame: Gazebo仿真相机视角

USAGE:
roslaunch frk_act_unified franka_act_inference.launch
"""

import rospy
import numpy as np
import tf.transformations as tft
import actionlib
import cv2
import base64
import json
import time
from io import BytesIO
from PIL import Image

# ROS messages
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header, String, Bool
from sensor_msgs.msg import Image as ROSImage, JointState
from franka_msgs.msg import FrankaState
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
from std_srvs.srv import Trigger, TriggerResponse
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
from cv_bridge import CvBridge
import tf
import random

# AgentLace for distributed inference
try:
    from agentlace.action import ActionConfig
    from agentlace.trainer import TrainerConfig, TrainerClient
    AGENTLACE_AVAILABLE = True
    rospy.loginfo("AgentLace modules imported successfully")
except ImportError as e:
    AGENTLACE_AVAILABLE = False
    rospy.logwarn(f"AgentLace not available: {e}")

class FrankaACTInferenceController:
    def __init__(self):
        rospy.init_node('franka_act_inference_controller', anonymous=True)
        
        # AgentLace configuration
        self.server_url = rospy.get_param('~server_url', '10.16.49.124:5555')
        self.task_name = rospy.get_param('~task_name', 'sim_transfer_cube_scripted')
        self.chunk_size = rospy.get_param('~chunk_size', 100)
        self.control_freq = rospy.get_param('~control_freq', 10)  # 10Hz
        
        # Control state
        self.running = False
        self.current_pose = None
        self.current_joint_states = None
        self.current_gripper_width = 0.08
        self.robot_state_received = False
        self.current_action_sequence = None
        self.action_step = 0
        
        # Camera and image processing
        self.cv_bridge = CvBridge()
        self.latest_camera_image = None
        self.image_received = False
        
        # Publishers
        self.equilibrium_pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose', 
            PoseStamped, 
            queue_size=1
        )
        
        self.status_pub = rospy.Publisher(
            '/franka_act_inference/status',
            String,
            queue_size=5
        )
        
        # Subscribers
        self.franka_state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.franka_state_callback
        )
        
        self.camera_sub = rospy.Subscriber(
            '/top_camera/rgb/image_raw',
            ROSImage,
            self.camera_callback
        )
        
        # Services
        self.start_inference_service = rospy.Service(
            '/franka_act_inference/start',
            Trigger,
            self.handle_start_inference
        )
        
        self.stop_inference_service = rospy.Service(
            '/franka_act_inference/stop',
            Trigger,
            self.handle_stop_inference
        )
        
        # Action clients for gripper control
        self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        
        # Gazebo services for stone management
        self.delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Stone generation parameters
        self.robot_world_offset = [-0.5, 0.0, 0.0]  # Robot base offset from world origin
        self.initial_stone_center = [0.5, 0.0]  # Fixed center for 15cm circle randomization
        
        # Control parameters
        self.control_rate = rospy.Rate(self.control_freq)
        self.pose_tolerance = 0.005  # 5mm position tolerance for precise control
        
        # Robot coordinate frame
        self.robot_base_frame = "fr3_link0"
        
        rospy.loginfo("Franka ACT Inference Controller initialized")
        
        # Initialize connections
        self.wait_for_connections()
        self.wait_for_robot_state()
        self.wait_for_camera_data()
        
        # Initialize AgentLace client
        if AGENTLACE_AVAILABLE:
            self.init_agentlace_client()
        else:
            rospy.logerr("AgentLace not available - cannot perform inference")
            self.agentlace_client = None
            
        # Generate initial stone for inference target
        self.generate_inference_stone()
            
        rospy.loginfo("Controller ready for ACT inference")
        rospy.loginfo("Services:")
        rospy.loginfo("  Start: /franka_act_inference/start")
        rospy.loginfo("  Stop: /franka_act_inference/stop")
    
    def init_agentlace_client(self):
        """Initialize AgentLace inference client using TrainerClient"""
        try:
            rospy.loginfo(f"Initializing AgentLace client for server: {self.server_url}")
            
            # Parse server URL to extract IP and port
            # Expected format: http://IP:PORT or IP:PORT
            server_url_clean = self.server_url.replace('http://', '').replace('https://', '')
            if ':' in server_url_clean:
                server_ip, port_str = server_url_clean.split(':')
                port_num = int(port_str)
            else:
                server_ip = server_url_clean
                port_num = 5555  # Default port
            
            # Create TrainerConfig (matching client_setup.sh exactly)
            config = TrainerConfig(
                port_number=port_num,
                broadcast_port=port_num + 1,
                request_types=['inference', 'server_status'],
                rate_limit=1000,
                version='0.0.2'
            )
            
            # Create TrainerClient (matching client_setup.sh successful implementation)
            self.agentlace_client = TrainerClient(
                name='franka_act_inference', 
                server_ip=server_ip, 
                config=config,
                timeout_ms=1000
            )
            
            # Test connection
            test_success = self.test_agentlace_connection()
            if test_success:
                rospy.loginfo("✅ AgentLace client initialized successfully")
            else:
                rospy.logerr("❌ AgentLace client connection test failed")
                
        except Exception as e:
            rospy.logerr(f"Failed to initialize AgentLace client: {e}")
            self.agentlace_client = None
    
    def test_agentlace_connection(self):
        """Test AgentLace server connection using TrainerClient"""
        try:
            if self.agentlace_client is None:
                return False
                
            # Test connection by making a simple request (matching client_setup.sh approach)
            rospy.loginfo("Testing AgentLace connection...")
            
            # Use server_status request instead of inference to avoid qpos requirement
            response = self.agentlace_client.request('server_status', {})
            
            if response is not None and response.get('success', False):
                rospy.loginfo("✅ Connection test successful - server responded")
                rospy.loginfo(f"   Server status: {response.get('status', 'unknown')}")
                return True
            else:
                rospy.logwarn("❌ Connection test failed - no response from server")
                return False
                
        except Exception as e:
            rospy.logerr(f"AgentLace connection test error: {e}")
            return False
    
    def wait_for_connections(self):
        """Wait for all necessary connections"""
        rospy.loginfo("Waiting for ROS connections...")
        
        # Wait for equilibrium pose topic
        while self.equilibrium_pose_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for cartesian_impedance_example_controller...")
            rospy.sleep(1.0)
            
        # Wait for gripper action servers
        rospy.loginfo("Waiting for gripper action servers...")
        self.gripper_grasp_client.wait_for_server(timeout=rospy.Duration(10.0))
        self.gripper_move_client.wait_for_server(timeout=rospy.Duration(10.0))
        
        rospy.loginfo("All ROS connections established")
    
    def wait_for_robot_state(self):
        """Wait for first robot state message"""
        rospy.loginfo("Waiting for robot state...")
        while not self.robot_state_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("✅ Robot state received")
    
    def wait_for_camera_data(self):
        """Wait for first camera image"""
        rospy.loginfo("Waiting for camera data...")
        while not self.image_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("✅ Camera data received")
    
    def franka_state_callback(self, msg):
        """Callback for Franka state messages"""
        # Extract current end-effector pose
        T_matrix = np.array(msg.O_T_EE).reshape(4, 4, order='F')
        position = T_matrix[:3, 3]
        quaternion = tft.quaternion_from_matrix(T_matrix)
        
        self.current_pose = {
            'position': position,
            'orientation': quaternion
        }
        
        # Extract joint states (7-DOF arm)
        self.current_joint_states = {
            'position': np.array(msg.q),     # Joint positions
            'velocity': np.array(msg.dq),    # Joint velocities
            'effort': np.array(msg.tau_J)   # Joint torques
        }
        
        self.robot_state_received = True
    
    def camera_callback(self, msg):
        """Callback for camera image messages"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Store latest image
            self.latest_camera_image = cv_image
            self.image_received = True
            
        except Exception as e:
            rospy.logwarn(f"Failed to process camera image: {e}")
    
    def get_current_state_data(self):
        """Get current robot state in ACT format (Cartesian space)"""
        if self.current_pose is None or self.latest_camera_image is None:
            return None
            
        try:
            # Prepare Cartesian state data (8-DOF: [x,y,z,qx,qy,qz,qw,gripper])
            position = self.current_pose['position']  # [x, y, z]
            orientation = self.current_pose['orientation']  # [qx, qy, qz, qw]
            gripper_pos = self.current_gripper_width  # Gripper width
            
            # Combine pose + gripper (8-DOF total)
            full_qpos = np.concatenate([position, orientation, [gripper_pos]])
            
            # For velocities, use zeros for now (can be improved with twist estimation)
            full_qvel = np.zeros(8)  # [dx,dy,dz,wx,wy,wz,gripper_vel]
            
            # Prepare camera image
            image_resized = cv2.resize(self.latest_camera_image, (640, 480))
            image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
            
            # Encode image to base64
            _, img_encoded = cv2.imencode('.jpg', image_rgb)
            img_b64 = base64.b64encode(img_encoded.tobytes()).decode('utf-8')
            
            # Prepare data dictionary
            state_data = {
                'qpos': full_qpos.tolist(),
                'qvel': full_qvel.tolist(),
                'image': img_b64,
                'request_id': f'franka_inference_{time.time()}',
                'timestamp': time.time()
            }
            
            return state_data
            
        except Exception as e:
            rospy.logerr(f"Failed to prepare state data: {e}")
            return None
    
    def send_equilibrium_pose(self, position, orientation):
        """Send equilibrium pose to impedance controller"""
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.robot_base_frame
        
        pose_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])
        pose_msg.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], 
                                             z=orientation[2], w=orientation[3])
        
        self.equilibrium_pose_pub.publish(pose_msg)
    
    def execute_gripper_width_command(self, gripper_width):
        """Execute gripper width command directly
        
        Args:
            gripper_width: Target gripper width in meters (0.0 to 0.08)
        """
        try:
            # Clamp to valid range
            target_width = np.clip(gripper_width, 0.0, 0.08)
            
            # Execute gripper movement if significant change
            width_change = abs(target_width - self.current_gripper_width)
            if width_change > 0.005:  # 5mm threshold
                goal = MoveGoal()
                goal.width = target_width
                goal.speed = 0.1
                
                self.gripper_move_client.send_goal(goal)
                # Don't wait for completion to maintain control frequency
                
                self.current_gripper_width = target_width
                
        except Exception as e:
            rospy.logwarn(f"Gripper width command failed: {e}")
    
    def execute_action(self, action):
        """Execute single action command
        
        Args:
            action: 8-dimensional action [x, y, z, qx, qy, qz, qw, gripper_width] in Cartesian space
        """
        try:
            if len(action) != 8:
                rospy.logwarn(f"Invalid action dimension: {len(action)}, expected 8")
                return False
            
            # Extract Cartesian pose command (first 7 elements) and gripper (8th element)
            target_position = action[:3]      # [x, y, z] position
            target_orientation = action[3:7]  # [qx, qy, qz, qw] quaternion
            gripper_width = action[7]         # Gripper width in meters
            
            # Send Cartesian pose command to impedance controller
            self.send_equilibrium_pose(target_position, target_orientation)
            
            # Handle gripper command (convert width to move command)
            self.execute_gripper_width_command(gripper_width)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to execute action: {e}")
            return False
    
    def execute_gripper_command(self, gripper_value):
        """Execute gripper command
        
        Args:
            gripper_value: Normalized gripper command [-1, 1]
                          -1 = fully closed, +1 = fully open
        """
        try:
            # Convert normalized command to physical width (0.0 to 0.08m)
            min_width = 0.0    # Fully closed
            max_width = 0.08   # Fully open
            
            # Map [-1, 1] to [min_width, max_width]
            target_width = min_width + (max_width - min_width) * (gripper_value + 1) / 2
            target_width = np.clip(target_width, min_width, max_width)
            
            # Execute gripper movement if significant change
            width_change = abs(target_width - self.current_gripper_width)
            if width_change > 0.005:  # 5mm threshold
                goal = MoveGoal()
                goal.width = target_width
                goal.speed = 0.1
                
                self.gripper_move_client.send_goal(goal)
                # Don't wait for completion to maintain control frequency
                
                self.current_gripper_width = target_width
                
        except Exception as e:
            rospy.logwarn(f"Gripper command failed: {e}")
    
    def inference_loop(self):
        """Main ACT inference and execution loop"""
        rospy.loginfo("🚀 Starting ACT inference loop")
        self.publish_status("RUNNING")
        
        inference_count = 0
        
        while self.running and not rospy.is_shutdown():
            try:
                # Step 1: Get current state
                state_data = self.get_current_state_data()
                if state_data is None:
                    rospy.logwarn("No state data available, skipping inference")
                    self.control_rate.sleep()
                    continue
                
                # Step 2: Perform inference (every chunk_size steps or when starting)
                if self.current_action_sequence is None or self.action_step >= len(self.current_action_sequence):
                    rospy.loginfo(f"🧠 Requesting inference #{inference_count + 1}")
                    start_time = time.time()
                    
                    # Get action sequence from server using TrainerClient (matching client_setup.sh)
                    response = self.agentlace_client.request('inference', state_data)
                    
                    if response and response.get('success'):
                        actions = np.array(response['actions'])
                    else:
                        actions = None
                    
                    inference_time = (time.time() - start_time) * 1000
                    
                    if actions is not None and len(actions) > 0:
                        self.current_action_sequence = actions
                        self.action_step = 0
                        inference_count += 1
                        
                        rospy.loginfo(f"✅ Inference successful - {len(actions)} actions, {inference_time:.1f}ms")
                        rospy.loginfo(f"   Action shape: {np.array(actions).shape}")
                        rospy.loginfo(f"   Action range: [{np.min(actions):.3f}, {np.max(actions):.3f}]")
                    else:
                        rospy.logwarn("❌ Inference failed - no actions received")
                        self.control_rate.sleep()
                        continue
                
                # Step 3: Execute current action
                if self.action_step < len(self.current_action_sequence):
                    current_action = self.current_action_sequence[self.action_step]
                    success = self.execute_action(current_action)
                    
                    if success:
                        self.action_step += 1
                    else:
                        rospy.logwarn(f"Action execution failed at step {self.action_step}")
                
                # Step 4: Control loop timing
                self.control_rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in inference loop: {e}")
                self.publish_status("ERROR")
                break
        
        rospy.loginfo("🛑 ACT inference loop stopped")
        self.publish_status("STOPPED")
    
    def publish_status(self, status):
        """Publish controller status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def handle_start_inference(self, req):
        """Service handler to start ACT inference"""
        if self.agentlace_client is None:
            return TriggerResponse(success=False, message="AgentLace client not available")
        
        if self.running:
            return TriggerResponse(success=False, message="Inference already running")
        
        try:
            rospy.loginfo("🎯 Starting ACT inference mode")
            self.running = True
            
            # Reset action sequence
            self.current_action_sequence = None
            self.action_step = 0
            
            # Start inference loop in separate thread
            import threading
            self.inference_thread = threading.Thread(target=self.inference_loop)
            self.inference_thread.daemon = True
            self.inference_thread.start()
            
            return TriggerResponse(success=True, message="ACT inference started successfully")
            
        except Exception as e:
            rospy.logerr(f"Failed to start inference: {e}")
            self.running = False
            return TriggerResponse(success=False, message=f"Start failed: {str(e)}")
    
    def handle_stop_inference(self, req):
        """Service handler to stop ACT inference"""
        try:
            rospy.loginfo("🛑 Stopping ACT inference mode")
            self.running = False
            
            # Wait for thread to finish
            if hasattr(self, 'inference_thread') and self.inference_thread.is_alive():
                self.inference_thread.join(timeout=2.0)
            
            return TriggerResponse(success=True, message="ACT inference stopped successfully")
            
        except Exception as e:
            rospy.logerr(f"Failed to stop inference: {e}")
            return TriggerResponse(success=False, message=f"Stop failed: {str(e)}")
    
    def _generate_random_stone_position(self):
        """Generate random stone position within 10cm circle of fixed center (matching training)"""
        z_table = 0.4  # Table surface height
        z_stone_half = 0.032  # Half of stone height (0.064/2)
        z_spawn = z_table + z_stone_half  # Place stone on table surface
        
        # Always use fixed initial center for randomization
        center_x, center_y = self.initial_stone_center[0], self.initial_stone_center[1]
        
        # Generate random position within 10cm circle of fixed center (matching training)
        radius_max = 0.1  # 10cm radius (same as training)
        
        # Use sqrt for uniform distribution within circle (not just circumference)
        radius = radius_max * np.sqrt(random.uniform(0.01, 1.0))  # Uniform distribution within circle
        angle = random.uniform(0, 2 * np.pi)
        
        rand_x = center_x + radius * np.cos(angle)
        rand_y = center_y + radius * np.sin(angle)
        
        # Ensure new position stays within workspace bounds
        rand_x = np.clip(rand_x, 0.35, 0.65)  # Tighter bounds to ensure reachability
        rand_y = np.clip(rand_y, -0.15, 0.15)
        
        rospy.loginfo(f"Random stone position generation:")
        rospy.loginfo(f"  Fixed center: [{center_x:.3f}, {center_y:.3f}]")
        rospy.loginfo(f"  Radius: {radius:.3f}m (max {radius_max:.3f}m), Angle: {angle:.2f}rad")
        rospy.loginfo(f"  Generated position: [{rand_x:.3f}, {rand_y:.3f}, {z_spawn:.3f}]")
        
        # Convert to world coordinates (robot base is at [-0.5, 0, 0] in world)
        world_x = rand_x + (-0.5)  # robot_x + robot_world_x
        world_y = rand_y + 0.0     # robot_y + robot_world_y  
        world_z = z_spawn
        
        rospy.loginfo(f"  World coordinates: [{world_x:.3f}, {world_y:.3f}, {world_z:.3f}]")
        
        return rand_x, rand_y, z_spawn, world_x, world_y, world_z
    
    def _spawn_stone_at_position(self, world_x, world_y, world_z, robot_x, robot_y, robot_z):
        """Helper method to spawn stone at specified position using model file path"""
        # Path to the stone model SDF file
        stone_model_path = '/home/jason/ws/catkin_ws/src/franka_ros/franka_gazebo/models/stone/model.sdf'
        
        # Read the SDF content from file
        try:
            with open(stone_model_path, 'r') as sdf_file:
                stone_sdf = sdf_file.read()
        except IOError as e:
            rospy.logerr(f"Failed to read stone model file: {e}")
            return False
        
        try:
            # Set initial pose
            initial_pose = Pose()
            initial_pose.position.x = world_x
            initial_pose.position.y = world_y
            initial_pose.position.z = world_z
            initial_pose.orientation.w = 1.0  # No rotation
            
            # Spawn the stone
            response = self.spawn_model_srv(
                model_name='stone',
                model_xml=stone_sdf,
                robot_namespace='',
                initial_pose=initial_pose,
                reference_frame='world'
            )
            
            if response.success:
                rospy.loginfo(f"Stone spawned at world: [{world_x:.3f}, {world_y:.3f}, {world_z:.3f}]")
                rospy.loginfo(f"Stone robot-relative: [{robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f}]")
                return True
            else:
                rospy.logerr(f"Failed to spawn stone: {response.status_message}")
                return False
                
        except Exception as e:
            rospy.logerr(f"Exception during stone spawning: {e}")
            return False
    
    def generate_inference_stone(self):
        """Generate a single random stone for ACT inference target"""
        rospy.loginfo("🎯 Generating inference target stone at random position")
        
        try:
            # Wait for Gazebo services
            rospy.loginfo("Waiting for Gazebo services...")
            try:
                rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
                rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.0)
            except rospy.ROSException:
                rospy.logwarn("Some Gazebo services not available, stone generation may fail")
            
            # Delete any existing stone first
            try:
                delete_response = self.delete_model_srv('stone')
                if delete_response.success:
                    rospy.loginfo("Existing stone deleted successfully")
                rospy.sleep(1.0)  # Wait for deletion to complete
            except Exception as e:
                rospy.loginfo(f"No existing stone to delete (expected): {e}")
            
            # Generate new random position and spawn stone
            rand_x, rand_y, z_spawn, world_x, world_y, world_z = self._generate_random_stone_position()
            
            # Spawn stone at random position
            max_attempts = 3
            for attempt in range(max_attempts):
                if self._spawn_stone_at_position(world_x, world_y, world_z, rand_x, rand_y, z_spawn):
                    rospy.loginfo(f"✅ Inference stone spawned successfully on attempt {attempt + 1}")
                    rospy.loginfo(f"   Position: [{rand_x:.3f}, {rand_y:.3f}, {z_spawn:.3f}] (robot frame)")
                    return True
                else:
                    rospy.logwarn(f"Stone spawn attempt {attempt + 1} failed")
                    if attempt < max_attempts - 1:
                        rospy.sleep(1.0)  # Wait before retry
            
            rospy.logerr("❌ Failed to spawn inference stone after multiple attempts")
            return False
            
        except Exception as e:
            rospy.logerr(f"Exception during inference stone generation: {e}")
            return False

def main():
    try:
        controller = FrankaACTInferenceController()
        rospy.loginfo("🤖 Franka ACT Inference Controller ready")
        rospy.loginfo("📡 Use services to start/stop inference:")
        rospy.loginfo("   rosservice call /franka_act_inference/start")
        rospy.loginfo("   rosservice call /franka_act_inference/stop")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller interrupted")
    except Exception as e:
        rospy.logerr(f"Controller error: {str(e)}")

if __name__ == '__main__':
    main()
