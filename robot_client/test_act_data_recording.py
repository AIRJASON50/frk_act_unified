#!/usr/bin/env python3
"""
ACT数据录制测试脚本

测试修复后的数据录制器是否能够正确生成ACT格式的数据。

Usage:
    # 启动Gazebo和机器人控制器后运行
    python test_act_data_recording.py
"""

import rospy
import time
import subprocess
import sys
from pathlib import Path

def test_data_recording():
    """测试ACT数据录制功能"""
    
    print("🧪 ACT数据录制测试")
    print("=" * 60)
    
    # 检查ROS环境
    try:
        rospy.init_node('act_data_test', anonymous=True)
        print("✅ ROS environment initialized")
    except Exception as e:
        print(f"❌ ROS initialization failed: {e}")
        return False
    
    # 检查必要的话题
    print("\n📡 Checking required ROS topics...")
    required_topics = [
        '/top_camera/rgb/image_raw',
        '/franka_state_controller/franka_states', 
        '/franka_gripper/joint_states'
    ]
    
    available_topics = rospy.get_published_topics()
    available_topic_names = [topic[0] for topic in available_topics]
    
    missing_topics = []
    for topic in required_topics:
        if topic in available_topic_names:
            print(f"   ✅ {topic}")
        else:
            print(f"   ❌ {topic} (missing)")
            missing_topics.append(topic)
    
    if missing_topics:
        print(f"\n❌ Missing topics: {missing_topics}")
        print("Please ensure Gazebo simulation and robot controllers are running")
        return False
    
    print("✅ All required topics available")
    
    # 测试数据录制器
    print("\n🎬 Testing data recorder...")
    
    # 创建测试数据目录
    test_dir = Path("/tmp/act_test_data")
    test_dir.mkdir(exist_ok=True)
    
    print(f"📁 Test data directory: {test_dir}")
    
    # 导入录制器
    try:
        from data_record.franka_act_dataset_recorder import FrankaACTDatasetRecorder
        print("✅ Data recorder imported successfully")
    except Exception as e:
        print(f"❌ Failed to import data recorder: {e}")
        return False
    
    # 创建录制器实例
    try:
        recorder = FrankaACTDatasetRecorder(save_dir=str(test_dir))
        print("✅ Data recorder created")
    except Exception as e:
        print(f"❌ Failed to create data recorder: {e}")
        return False
    
    # 等待数据流
    print("\n⏳ Waiting for sensor data...")
    start_time = time.time()
    timeout = 10  # 10秒超时
    
    while time.time() - start_time < timeout:
        status = recorder.get_status()
        if status['data_length'] > 0:
            print("✅ Sensor data received")
            break
        time.sleep(0.5)
    else:
        print("❌ Timeout waiting for sensor data")
        return False
    
    # 测试短时间录制
    print("\n🔴 Starting test recording (5 seconds)...")
    recorder.start_recording()
    
    # 录制5秒
    for i in range(5):
        print(f"   Recording... {i+1}/5")
        time.sleep(1)
    
    recorder.stop_recording()
    print("🛑 Recording stopped")
    
    # 保存数据
    print("\n💾 Saving test episode...")
    recorder.save_current_episode()
    
    # 验证生成的数据
    hdf5_files = list(test_dir.glob("*.hdf5"))
    if hdf5_files:
        test_file = hdf5_files[0]
        print(f"✅ Test episode saved: {test_file.name}")
        
        # 简单验证生成的数据格式
        print("\n🔍 Checking generated data...")
        try:
            import h5py
            with h5py.File(test_file, 'r') as f:
                qpos_shape = f['/observations/qpos'].shape
                action_shape = f['/action'].shape
                images_shape = f['/observations/images/top'].shape
                
                print(f"   📊 Data shapes: qpos{qpos_shape}, actions{action_shape}, images{images_shape}")
                
                if qpos_shape[1] == 8 and action_shape[1] == 8 and qpos_shape[0] == action_shape[0]:
                    print("🎉 Test passed! ACT data recording works correctly!")
                    return True
                else:
                    print("⚠️  Data format issues detected")
                    return False
        except Exception as e:
            print(f"❌ Data check failed: {e}")
            return False
    else:
        print("❌ No test file generated")
        return False

def main():
    print("🚀 Starting ACT Data Recording Test")
    print("Please ensure:")
    print("  1. Gazebo simulation is running")
    print("  2. Robot controllers are active") 
    print("  3. Camera topics are publishing")
    print("\nPress Enter to continue or Ctrl+C to cancel...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\n❌ Test cancelled by user")
        sys.exit(0)
    
    success = test_data_recording()
    
    if success:
        print("\n🎉 All tests passed!")
        print("Your ACT data recording system is working correctly!")
        sys.exit(0)
    else:
        print("\n❌ Tests failed!")
        print("Please check the error messages above and fix the issues.")
        sys.exit(1)

if __name__ == "__main__":
    main()
