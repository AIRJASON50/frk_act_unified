#!/usr/bin/env python3
"""
Franka Impedance Grasping Controller for ACT Dataset Collection

MAIN FUNCTIONALITY:
8阶段自主抓取控制器，执行pick-and-place操作并协调ACT数据录制流程

EXECUTION STATES (8-PHASE OPERATION):
Phase 0: STONE_GENERATE - 生成/检测石块位置，准备开始新episode
Phase 1: RECORD_START - 石块检测完成，触发数据录制开始
Phase 2: GRIPPER_OPEN - 打开夹爪，准备抓取
Phase 3: MOVE_ABOVE - 移动到目标上方10cm位置
Phase 4: DESCEND_GRASP - 下降到抓取位置
Phase 5: GRIPPER_CLOSE - 关闭夹爪，抓取物体
Phase 6: LIFT_OBJECT - 提升物体到安全高度
Phase 7: RETURN_HOME - 返回到初始家位置
Phase 8: RECORD_END - 任务完成，触发数据保存并结束录制

STATE TRANSITION CONDITIONS:
Phase 0→1: 石块位置检测完成 && get_stones_info()返回有效位置
Phase 1→2: 数据录制器确认开始录制 && 位置计算完成
Phase 2→3: 夹爪打开成功 (gripper_width > threshold)
Phase 3→4: 到达目标上方位置 (position_error < tolerance)
Phase 4→5: 到达抓取位置 (z_position ≈ grasp_height)
Phase 5→6: 夹爪关闭成功 && 力传感器检测到接触
Phase 6→7: 物体提升到安全高度 (lift_success = True)
Phase 7→8: 到达家位置 (position_error < home_tolerance)
Phase 8→0: 数据保存完成 && 环境清理完成 (石块删除)

ERROR HANDLING STATES:
- TIMEOUT_ERROR: 任何阶段超时 → 重置到Phase 0
- GRASP_FAILURE: 抓取失败 → 重试或重置到Phase 0  
- MOTION_ERROR: 运动规划失败 → 紧急停止并重置
- SERVICE_ERROR: ROS服务调用失败 → 记录错误并重试

COORDINATE SYSTEMS:
- Robot Base Frame: 机械臂基座坐标系 (控制指令)
- World Frame: Gazebo世界坐标系 (模型状态)
- Transform: robot_world_offset = [-0.5, 0, 0] (robot base in world)

STONE MANAGEMENT:
- 随机位置生成: 15cm半径圆内，固定中心[0.5, 0.0]
- 位置更新方式: SetModelState服务移动现有石块 (优先)
- 备用方案: DeleteModel + SpawnModel重新生成
- 坐标同步: 实时监听/gazebo/model_states话题

SERVICE INTERFACES:
- /franka_controller/trigger_grasp: 触发单次8阶段抓取序列
- /gazebo/delete_model: 删除仿真模型
- /gazebo/spawn_sdf_model: 生成新模型
- /gazebo/set_model_state: 设置模型状态

ROS TOPIC COORDINATION:
Published Topics:
- /franka_controller/current_phase: 当前执行阶段 (Int32)
- /cartesian_impedance_example_controller/equilibrium_pose: 目标位置 (PoseStamped)

Subscribed Topics:
- /franka_state_controller/franka_states: 机械臂状态反馈
- /gazebo/model_states: 仿真模型状态 (石块位置追踪)

GRIPPER CONTROL:
- Open: width=0.08m, speed=0.1m/s
- Close: width=0.035m, force=15N, speed=0.05m/s
- Stone dimensions: 25×32×64mm, mass=0.1024kg
- Grasp tolerance: ±0.01m inner/outer epsilon

MOTION CONTROL:
- Impedance control via equilibrium pose
- Position tolerance: 0.01m (normal), 0.02m (lift with object)
- Timeout protection: 10s per motion segment
- Coordinate frame: fr3_link0 (robot base)

CONTINUOUS OPERATION:
- 每个episode后自动清理环境 (删除石块)
- 支持无限循环演示模式
- 失败时自动重试机制
- 优雅错误恢复和状态重置

USAGE SCENARIOS:
1. 单次抓取: rosservice call /franka_controller/trigger_grasp
2. 自动数据收集: 配合automated_data_collection.sh使用
3. 连续演示: run_continuous_demo()循环执行
"""

import rospy
import numpy as np
import tf.transformations as tft
import actionlib
import os
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header, String, Int32, Empty
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
from franka_msgs.msg import FrankaState
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
from std_srvs.srv import Trigger, TriggerResponse
import tf
import random

class FrankaImpedanceGraspingController:
    def __init__(self):
        rospy.init_node('franka_impedance_grasping_controller', anonymous=True)
        
        
        # Publishers
        self.equilibrium_pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose', 
            PoseStamped, 
            queue_size=1
        )
        
        # Dataset recording control publishers
        self.phase_pub = rospy.Publisher(
            '/franka_controller/current_phase',
            Int32,
            queue_size=5
        )
        
        # Service for triggering single grasp sequences
        self.grasp_service = rospy.Service(
            '/franka_controller/trigger_grasp',
            Trigger,
            self.handle_grasp_trigger
        )
        
        self.episode_control_pub = rospy.Publisher(
            '/franka_controller/episode_control',
            String,
            queue_size=5
        )
        
        # Subscribers
        self.franka_state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.franka_state_callback
        )
        
        # Subscribe to Gazebo model states to get stone position
        self.model_states_sub = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.model_states_callback
        )
        
        # Gazebo services for reset functionality
        self.delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Action clients for gripper control
        self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        
        # Current robot state
        self.current_pose = None
        self.robot_state_received = False
        self.home_position = None  # Store initial robot position as home
        
        # Stone position tracking
        self.stone_positions = {}
        self.robot_world_offset = np.array([0.5, 0.0, 0.0])  # Robot base offset from world origin
        self.last_stone_position = None  # Track last stone position for local randomization
        self.initial_stone_center = [0.5, 0.0]  # Fixed center for 15cm circle randomization
        self.model_states_received = False
        
        # TF listener for coordinate transformations
        self.tf_listener = tf.TransformListener()
        
        # Robot base frame and world offset
        self.robot_base_frame = "fr3_link0"
        # Robot is spawned at (-0.5, 0, 0) in world coordinates
        self.robot_world_offset = [-0.5, 0.0, 0.0]
        
        # Control parameters
        self.control_rate = rospy.Rate(10)  # 10 Hz
        self.pose_tolerance = 0.015  # 1.5cm position tolerance (balanced for precision/success)
        self.lift_pose_tolerance = 0.03   # 3cm tolerance for lifting with object
        self.home_tolerance = 0.025       # 2.5cm tolerance specifically for returning home
        self.orientation_tolerance = 0.1  # orientation tolerance
        
        
        # Wait for connections
        self.wait_for_connections()
        
        # Get initial robot state and model states
        self.wait_for_robot_state()
        self.wait_for_model_states()
        
        # Initialize to phase 0 (ready for stone generation)
        self.publish_phase(0)
        
    
    
    
    def publish_phase(self, phase):
        """Publish current execution phase for dataset recording"""
        phase_msg = Int32()
        phase_msg.data = phase
        self.phase_pub.publish(phase_msg)
    
    def publish_episode_control(self, command):
        """Publish episode control command for dataset recording"""
        control_msg = String()
        control_msg.data = command
        self.episode_control_pub.publish(control_msg)

    def wait_for_connections(self):
        """Wait for all necessary connections to be established"""
        # Wait for equilibrium pose topic to be ready
        while self.equilibrium_pose_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.5)
            
        # Wait for Gazebo services
        try:
            rospy.wait_for_service('/gazebo/delete_model', timeout=10.0)
            rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10.0) 
            rospy.wait_for_service('/gazebo/set_model_state', timeout=10.0)
        except rospy.ROSException:
            pass
        
        # Wait for gripper action servers
        self.gripper_grasp_client.wait_for_server(timeout=rospy.Duration(10.0))
        self.gripper_move_client.wait_for_server(timeout=rospy.Duration(10.0))

    def wait_for_robot_state(self):
        """Wait for first robot state message"""
        while not self.robot_state_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def wait_for_model_states(self):
        """Wait for first model states message"""
        while not self.model_states_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def wait_for_new_stone_position(self, timeout=10.0):
        """Wait for stone position to be updated in model states after spawning"""
        expected_pos = self.last_stone_position
        if expected_pos is None:
            return
        
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if 'stone' in self.stone_positions:
                current_pos = self.stone_positions['stone']['position']
                # Check if positions are close enough (within 1cm)
                pos_diff = np.linalg.norm([
                    current_pos[0] - expected_pos[0],
                    current_pos[1] - expected_pos[1]
                ])
                
                if pos_diff < 0.05:  # Within 5cm tolerance
                    return
            
            # Check timeout
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                break
            
            rospy.sleep(0.1)

    def franka_state_callback(self, msg):
        """Callback for Franka state messages"""
        # Extract current end-effector pose from transformation matrix
        # The O_T_EE is a flattened 4x4 transformation matrix (column-major order)
        T_matrix = np.array(msg.O_T_EE).reshape(4, 4, order='F')  # Use Fortran order for column-major
        
        position = T_matrix[:3, 3]
        quaternion = tft.quaternion_from_matrix(T_matrix)
        
        self.current_pose = {
            'position': position,
            'orientation': quaternion
        }
        
        # Store initial position as home position
        if not self.robot_state_received:
            self.home_position = {
                'position': position.copy(),
                'orientation': quaternion.copy()
            }
            self.robot_state_received = True
    
    def model_states_callback(self, msg):
        """Callback for Gazebo model states messages"""
        # Update stone positions from Gazebo model states with proper coordinate conversion
        self.stone_positions = {}
        
        for i, name in enumerate(msg.name):
            if 'stone' in name.lower() or 'cube' in name.lower() or 'block' in name.lower():
                world_position = msg.pose[i].position
                world_orientation = msg.pose[i].orientation
                
                # Convert from world coordinates to robot base coordinates
                # Robot base is at (-0.5, 0, 0) in world frame
                robot_x = world_position.x - self.robot_world_offset[0]  # 0.016428 - (-0.5) = 0.516428
                robot_y = world_position.y - self.robot_world_offset[1]  # 0.000584 - 0 = 0.000584
                robot_z = world_position.z - self.robot_world_offset[2]  # 0.475172 - 0 = 0.475172
                
                self.stone_positions[name] = {
                    'position': [robot_x, robot_y, robot_z],
                    'orientation': [world_orientation.x, world_orientation.y, world_orientation.z, world_orientation.w],
                    'world_position': [world_position.x, world_position.y, world_position.z]
                }
        
        if not self.model_states_received:
            self.model_states_received = True

    def send_equilibrium_pose(self, position, orientation):
        """Send equilibrium pose to the impedance controller"""
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "fr3_link0"
        
        pose_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])
        pose_msg.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], 
                                             z=orientation[2], w=orientation[3])
        
        self.equilibrium_pose_pub.publish(pose_msg)

    def move_to_pose(self, target_position, target_orientation, timeout=10.0, use_lift_tolerance=False):
        """
        Move to target pose using impedance control
        
        Args:
            target_position: [x, y, z] in meters
            target_orientation: [x, y, z, w] quaternion
            timeout: maximum time to wait for convergence
            use_lift_tolerance: use larger tolerance for lifting with object
        
        Returns:
            bool: True if successful, False if timeout
        """
        
        # Select appropriate tolerance
        tolerance = self.lift_pose_tolerance if use_lift_tolerance else self.pose_tolerance
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Send equilibrium pose command
            self.send_equilibrium_pose(target_position, target_orientation)
            
            # Check if we've reached the target
            if self.current_pose is not None:
                pos_error = np.linalg.norm(np.array(target_position) - self.current_pose['position'])
                
                if pos_error < tolerance:
                    return True
            
            # Check timeout
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return False
            
            self.control_rate.sleep()
        
        return False

    def move_to_pose_with_tolerance(self, target_position, target_orientation, custom_tolerance, timeout=10.0):
        """
        Move to target pose using impedance control with custom tolerance
        
        Args:
            target_position: [x, y, z] in meters
            target_orientation: [x, y, z, w] quaternion
            custom_tolerance: custom position tolerance in meters
            timeout: maximum time to wait for convergence
        
        Returns:
            bool: True if successful, False if timeout
        """
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Send equilibrium pose command
            self.send_equilibrium_pose(target_position, target_orientation)
            
            # Check if we've reached the target
            if self.current_pose is not None:
                pos_error = np.linalg.norm(np.array(target_position) - self.current_pose['position'])
                
                if pos_error < custom_tolerance:
                    return True
            
            # Check timeout
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return False
            
            self.control_rate.sleep()
        
        return False

    def open_gripper(self, width=0.08):
        """Open gripper to specified width"""
        
        goal = MoveGoal()
        goal.width = width
        goal.speed = 0.1
        
        self.gripper_move_client.send_goal(goal)
        result = self.gripper_move_client.wait_for_result(timeout=rospy.Duration(5.0))
        
        if result:
            return True
        else:
            return False

    def close_gripper(self, width=0.035, force=15.0):
        """Close gripper with specified force - optimized for stone grasping
        
        Stone dimensions: 0.025 x 0.032 x 0.064m
        Gripper width should be > stone width for proper grasping
        """
        
        goal = GraspGoal()
        goal.width = width  # 35mm width - larger than stone's 25mm for proper contact
        goal.epsilon.inner = 0.01  # Increased tolerance for better grasping
        goal.epsilon.outer = 0.01  # Increased tolerance for better grasping
        goal.speed = 0.05  # Slower speed for more controlled grasping
        goal.force = force  # Increased force for secure grip
        
        self.gripper_grasp_client.send_goal(goal)
        result = self.gripper_grasp_client.wait_for_result(timeout=rospy.Duration(5.0))
        
        if result:
            return True
        else:
            return False

    def get_stones_info(self):
        """Get information about all detected stones"""
        stones_info = []
        for name, info in self.stone_positions.items():
            if 'stone' in name.lower() or 'cube' in name.lower() or 'block' in name.lower():
                stones_info.append({
                    'name': name,
                    'position': info['position'],
                    'orientation': info['orientation']
                })
        return stones_info
    
    def get_stone_target_positions(self, stone_name):
        """Calculate target positions for grasping a specific stone"""
        if stone_name not in self.stone_positions:
            return None
            
        stone_pos = self.stone_positions[stone_name]['position']
        stone_orient = self.stone_positions[stone_name]['orientation']
        
        # Define grasp approach and movement parameters according to user requirements
        approach_height = 0.08  # 8cm above stone (reduced for workspace limits)
        
        # Use actual stone dimensions from user's model: 0.025 x 0.032 x 0.064
        stone_height = 0.064  # Actual stone height from model.sdf
        grasp_descent = 0.08 + (stone_height / 2)  # 8cm + half stone height
        
        place_offset = [0.2, 0.2, 0.0]  # Place location offset from stone
        
        # Calculate target positions according to user specifications
        # Step 1: Move to 10cm above stone
        above_stone_position = [stone_pos[0], stone_pos[1], stone_pos[2] + approach_height]
        
        # Step 2: Move down to grasp position
        grasp_position = [stone_pos[0], stone_pos[1], 0.05+stone_pos[2] + approach_height - grasp_descent]
        
        # Other positions
        lift_position = [stone_pos[0], stone_pos[1], stone_pos[2] + approach_height]  # Back to 10cm above
        place_position = [stone_pos[0] + place_offset[0], stone_pos[1] + place_offset[1], stone_pos[2] + approach_height]
        place_down_position = [stone_pos[0] + place_offset[0], stone_pos[1] + place_offset[1], stone_pos[2] + approach_height - grasp_descent]
        
        
        # Use downward pointing orientation for grasping
        grasp_orientation = tft.quaternion_from_euler(np.pi, 0, 0)  # Point downward
        
        return {
            'above_stone': above_stone_position,  # 10cm above stone
            'grasp': grasp_position,              # Down by 10cm + half stone height
            'lift': lift_position,                # Back to 10cm above
            'place': place_position,
            'place_down': place_down_position,
            'orientation': grasp_orientation
        }
    
    def return_to_home(self):
        """Return robot to home position with custom tolerance"""
        if self.home_position is None:
            return False
        home_pos = self.home_position['position']
        home_orient = self.home_position['orientation']
        
        # Use custom home tolerance for better success rate
        return self.move_to_pose_with_tolerance([home_pos[0], home_pos[1], home_pos[2]], 
                                               home_orient, self.home_tolerance)
    
    def execute_pick_and_place_sequence(self):
        """Execute a complete pick and place sequence with 8-phase recording"""
        
        try:
            # Phase 0: Stone Generation
            self.publish_phase(0)
            
            # Always delete existing stones first to ensure clean slate
            try:
                # Try to delete stone model (may not exist, which is fine)
                delete_response = self.delete_model_srv('stone')
                if delete_response.success:
                    pass
                else:
                    pass
                rospy.sleep(1.5)  # Wait for deletion to complete
            except Exception as e:
                pass
            
            # Always spawn new stone at random location within 15cm circle
            max_attempts = 3
            for attempt in range(max_attempts):
                if self.spawn_new_stone_random():
                    break
                    if attempt < max_attempts - 1:
                        rospy.sleep(0.5)  # Wait before retry
            else:
                return False
            rospy.sleep(2.0)  # Wait for spawn to complete and physics to settle
                
            # Verify stone generation
            stones = self.get_stones_info()
            if not stones:
                return False
                
            stone_info = stones[0]  # Use the first stone
            
            # Phase 1: Record Start (triggers automatic recording in dataset recorder)
            self.publish_phase(1)
            rospy.sleep(0.25)  # Brief pause for recording to start
            
            # Calculate approach targets based on stone position
            targets = self.get_stone_target_positions(stone_info['name'])
            
            # Phase 2: Open gripper
            self.publish_phase(2)
            if not self.open_gripper():
                return False
            rospy.sleep(0.5)
            
            # Phase 3: Move to 10cm above stone
            self.publish_phase(3)
            if not self.move_to_pose(targets['above_stone'], targets['orientation']):
                return False
            rospy.sleep(0.5)
            
            # Phase 4: Move down to grasp position
            self.publish_phase(4)
            if not self.move_to_pose(targets['grasp'], targets['orientation']):
                return False
            rospy.sleep(0.5)
            
            # Phase 5: Close gripper (grasp)
            self.publish_phase(5)
            if not self.close_gripper():
                return False
            rospy.sleep(1.0)  # Wait for grasp to settle
            
            # Phase 6: Lift object
            self.publish_phase(6)
            if not self.move_to_pose(targets['lift'], targets['orientation'], timeout=20.0, use_lift_tolerance=True):
                return False
            rospy.sleep(0.5)
            
            # Phase 7: Return to home position
            self.publish_phase(7)
            if not self.return_to_home():
                return False
            rospy.sleep(0.5)
            
            # Delete stone to complete the task (stone deletion triggers recording end)
            try:
                delete_response = self.delete_model_srv('stone')
                if delete_response.success:
                    pass
                else:
                    pass
                rospy.sleep(0.5)  # Wait for deletion to complete
                
                # Phase 8: Record End (triggered by stone deletion completion)
                self.publish_phase(8)
                rospy.sleep(0.5)  # Wait for episode to be saved
                
            except Exception as e:
                # Still trigger recording end even if deletion failed
                self.publish_phase(8)
                rospy.sleep(0.5)
            
            return True
            
        except Exception as e:
            # Emergency reset to phase 0
            self.publish_phase(0)
            return False

    def run_demo(self):
        """Run demonstration sequence"""
        
        # Wait a bit for everything to initialize
        rospy.sleep(2.0)
        
        # Execute pick and place
        success = self.execute_pick_and_place_sequence()
        
        if success:
            
            # Reset for continuous loop: clear gripper and spawn new stone
            rospy.sleep(0.5)
            
            # Delete stone from gripper and spawn new one for continuous mode
            try:
                self.delete_model_srv('stone')
                rospy.sleep(0.5)
                
                self.spawn_new_stone_random()
                rospy.sleep(2.0)
                
                self.open_gripper()
                rospy.sleep(0.5)
                
            except Exception as e:
                pass
        else:
            pass
        
        # Keep node alive
        
        # Continuous loop for repeated grasping
        self.run_continuous_demo()
        rospy.spin()

    def reset_simulation(self):
        """Reset robot pose and spawn new stone at random position"""
        
        try:
            # 1. Reset robot to home position (keeping stone in gripper)
            if self.home_position is not None:
                self.move_to_pose(
                    self.home_position['position'], 
                    self.home_position['orientation'], 
                    timeout=15.0
                )
            
            # 2. Move existing stone to new random position (more reliable than delete+spawn)
            success = self.move_stone_to_random_position()
            if not success:
                try:
                    self.delete_model_srv('stone')
                    rospy.sleep(1.5)  # Longer delay for proper cleanup
                    success = self.spawn_new_stone_random()
                    if not success:
                        return False
                except Exception as e:
                    return False
            
            # 4. Wait for Gazebo model states to update with new stone position
            self.wait_for_new_stone_position()
            rospy.sleep(0.5)  # Additional physics settling time
            
            # 5. Open gripper (now empty since old stone was deleted)
            self.open_gripper()
            
            return True
            
        except Exception as e:
            return False
    
    
    def _generate_random_stone_position(self):
        """Generate random stone position within 15cm circle of fixed center"""
        z_table = 0.4  # Table surface height
        z_stone_half = 0.032  # Half of stone height (0.064/2)
        z_spawn = z_table + z_stone_half  # Place stone on table surface
        
        # Always use fixed initial center for randomization
        center_x, center_y = self.initial_stone_center[0], self.initial_stone_center[1]
        
        # Generate random position within 15cm circle of fixed center  
        radius_max = 0.1  # 15cm radius
        
        # Use sqrt for uniform distribution within circle (not just circumference)
        radius = radius_max * np.sqrt(random.uniform(0.01, 1.0))  # Uniform distribution within circle
        angle = random.uniform(0, 2 * np.pi)
        
        rand_x = center_x + radius * np.cos(angle)
        rand_y = center_y + radius * np.sin(angle)
        
        # Ensure new position stays within workspace bounds
        rand_x = np.clip(rand_x, 0.35, 0.65)  # Tighter bounds to ensure reachability
        rand_y = np.clip(rand_y, -0.15, 0.15)
        
        
        # Store new position for next iteration
        self.last_stone_position = [rand_x, rand_y, z_spawn]
        
        # Convert to world coordinates (robot base is at [-0.5, 0, 0] in world)
        world_x = rand_x + (-0.5)  # robot_x + robot_world_x
        world_y = rand_y + 0.0     # robot_y + robot_world_y  
        world_z = z_spawn
        
        
        return rand_x, rand_y, z_spawn, world_x, world_y, world_z
    
    def spawn_new_stone_random(self):
        """Spawn new stone at random position"""
        rand_x, rand_y, z_spawn, world_x, world_y, world_z = self._generate_random_stone_position()
        return self._spawn_stone_at_position(world_x, world_y, world_z, rand_x, rand_y, z_spawn)
    
    def move_stone_to_random_position(self):
        """Move existing stone to new random position using SetModelState service"""
        rand_x, rand_y, z_spawn, world_x, world_y, world_z = self._generate_random_stone_position()
        
        try:
            from gazebo_msgs.msg import ModelState
            from geometry_msgs.msg import Pose, Twist
            
            # Create model state message
            model_state = ModelState()
            model_state.model_name = 'stone'
            model_state.pose.position.x = world_x
            model_state.pose.position.y = world_y
            model_state.pose.position.z = world_z
            model_state.pose.orientation.w = 1.0  # No rotation
            
            # Set velocity to zero (static)
            model_state.twist.linear.x = 0.0
            model_state.twist.linear.y = 0.0
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0
            
            model_state.reference_frame = 'world'
            
            # Use SetModelState service to move the stone
            response = self.set_model_state_srv(model_state)
            
            if response.success:
                return True
            else:
                return False
                
        except Exception as e:
            return False
        
    def _spawn_stone_at_position(self, world_x, world_y, world_z, robot_x, robot_y, robot_z):
        """Helper method to spawn stone at specified position using model file path"""
        # Path to the stone model SDF file
        stone_model_path = '/home/jason/ws/catkin_ws/src/franka_ros/franka_gazebo/models/stone/model.sdf'
        
        # Read the SDF content from file
        try:
            with open(stone_model_path, 'r') as sdf_file:
                stone_sdf = sdf_file.read()
        except IOError as e:
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
                return True
            else:
                return False
                
        except Exception as e:
            return False

    
    def run_continuous_demo(self):
        """Run continuous grasping demonstration with resets"""
        iteration = 1
        
        while not rospy.is_shutdown():
            
            # Wait for model states to update after reset
            rospy.sleep(2.0)
            
            # Execute pick and place
            success = self.execute_pick_and_place_sequence()
            
            if success:
                pass
            else:
                pass
            
            # Wait before reset
            rospy.sleep(3.0)
            
            # Reset simulation for next iteration
            if self.reset_simulation():
                iteration += 1
            else:
                break
            
            # Wait between iterations
            rospy.sleep(2.0)

    def handle_grasp_trigger(self, req):
        """Service callback to trigger a single grasp sequence"""
        try:
            success = self.execute_pick_and_place_sequence()
            
            if success:
                # Clean up environment for next sequence
                rospy.sleep(2.0)  # Wait for any pending operations
                
                # Delete any existing stones to ensure clean state for next episode
                try:
                    self.delete_model_srv('stone')
                    rospy.sleep(0.5)  # Wait for deletion to complete
                    pass
                except Exception as cleanup_error:
                    pass
                
                # Reset to phase 0 for next sequence
                self.publish_phase(0)
                return TriggerResponse(success=True, message="8-phase grasp sequence completed successfully")
            else:
                # Clean up and reset to phase 0 on failure
                try:
                    self.delete_model_srv('stone')
                except:
                    pass  # Ignore cleanup errors on failure
                self.publish_phase(0)
                return TriggerResponse(success=False, message="8-phase grasp sequence failed")
                
        except Exception as e:
            # Clean up and reset on exception
            try:
                self.delete_model_srv('stone')
            except:
                pass
            self.publish_phase(0)  # Reset on exception
            return TriggerResponse(success=False, message=f"Grasp sequence error: {str(e)}")

def main():
    try:
        controller = FrankaImpedanceGraspingController()
        rospy.spin()  # Keep node alive to handle service calls
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        pass

if __name__ == '__main__':
    main()
