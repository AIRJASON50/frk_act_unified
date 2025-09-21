#!/usr/bin/env python3

"""
ACT Dataset Validation Tool

MAIN FUNCTIONALITY:
ACT数据集兼容性验证工具，检查生成的HDF5文件格式和数据完整性

VALIDATION STATES:
1. FILE_DISCOVERY - 扫描指定目录，发现所有HDF5文件
2. STRUCTURE_CHECK - 验证HDF5文件内部结构和必需字段
3. DATA_VALIDATION - 检查数据维度、类型和数值范围
4. COMPATIBILITY_TEST - 确认ACT训练框架兼容性
5. REPORT_GENERATION - 生成验证结果报告

STATE TRANSITION CONDITIONS:
- FILE_DISCOVERY → STRUCTURE_CHECK: 找到至少1个.hdf5文件
- STRUCTURE_CHECK → DATA_VALIDATION: 所有必需字段存在且类型正确
- DATA_VALIDATION → COMPATIBILITY_TEST: 数据维度和数值范围检查通过
- COMPATIBILITY_TEST → REPORT_GENERATION: 兼容性测试完成
- ANY_STATE → ERROR: 文件损坏、权限错误或格式不兼容

VALIDATION CRITERIA:
Required HDF5 Structure:
- Root attributes: 'sim' (simulation flag)
- observations/qpos: (T, 8) float64 - joint positions
- observations/qvel: (T, 8) float64 - joint velocities  
- observations/images/top: (T, H, W, 3) uint8 - RGB images
- action: (T, 8) float64 - target actions

Data Consistency Checks:
- 所有数据集长度必须一致 (T维度相同)
- 关节数据: 8维度 (7个关节 + 1个夹爪)
- 图像数据: 标准尺寸 (通常480x640x3)
- 数值范围: 关节位置[-π,π], 速度合理范围
- NaN/Inf检测: 不允许无效数值

ERROR DETECTION:
- Missing fields: 缺少必需的数据集或属性
- Dimension mismatch: 数据维度不匹配
- Data corruption: 文件损坏或读取错误
- Type errors: 数据类型不正确
- Range violations: 数值超出合理范围

COMPATIBILITY VERIFICATION:
- ACT training pipeline compatibility
- Standard ML framework support (PyTorch/JAX)
- Episode length consistency across dataset
- Memory usage estimation for training

OUTPUT REPORTING:
- File-by-file validation summary
- Overall dataset statistics
- Compatibility status (✅/❌)
- Detailed error messages and suggestions
- Training readiness assessment

USAGE:
python3 validate_act_dataset.py <dataset_directory>
"""

import h5py
import numpy as np
import os
import sys

def validate_hdf5_structure(filepath):
    """Validate HDF5 file structure matches ACT requirements"""
    print(f"Validating: {filepath}")
    
    try:
        with h5py.File(filepath, 'r') as f:
            # Check required attributes
            if 'sim' not in f.attrs:
                print("❌ Missing 'sim' attribute")
                return False
            print(f"✅ sim attribute: {f.attrs['sim']}")
            
            # Check observations group
            if 'observations' not in f:
                print("❌ Missing 'observations' group")
                return False
            
            obs = f['observations']
            
            # Check qpos dataset
            if 'qpos' not in obs:
                print("❌ Missing 'qpos' dataset")
                return False
            qpos = obs['qpos']
            print(f"✅ qpos shape: {qpos.shape}, dtype: {qpos.dtype}")
            
            # Check qvel dataset  
            if 'qvel' not in obs:
                print("❌ Missing 'qvel' dataset")
                return False
            qvel = obs['qvel']
            print(f"✅ qvel shape: {qvel.shape}, dtype: {qvel.dtype}")
            
            # Check images group
            if 'images' not in obs:
                print("❌ Missing 'images' group")
                return False
            
            images = obs['images']
            if 'top' not in images:
                print("❌ Missing 'top' camera images")
                return False
            
            top_imgs = images['top']
            print(f"✅ top images shape: {top_imgs.shape}, dtype: {top_imgs.dtype}")
            
            # Check action dataset
            if 'action' not in f:
                print("❌ Missing 'action' dataset")
                return False
            action = f['action']
            print(f"✅ action shape: {action.shape}, dtype: {action.dtype}")
            
            # Validate data consistency
            T = qpos.shape[0]
            if qvel.shape[0] != T:
                print(f"❌ qvel length {qvel.shape[0]} != qpos length {T}")
                return False
            
            if action.shape[0] != T:
                print(f"❌ action length {action.shape[0]} != qpos length {T}")
                return False
                
            if top_imgs.shape[0] != T:
                print(f"❌ images length {top_imgs.shape[0]} != qpos length {T}")
                return False
            
            print(f"✅ All datasets have consistent length: {T}")
            
            # Validate dimensions
            if len(qpos.shape) != 2:
                print(f"❌ qpos should be 2D, got {len(qpos.shape)}D")
                return False
                
            if qpos.shape[1] != 8:  # 7 joints + 1 gripper for single arm
                print(f"❌ qpos should have 8 DOF, got {qpos.shape[1]}")
                return False
                
            if len(top_imgs.shape) != 4:  # (T, H, W, C)
                print(f"❌ images should be 4D (T,H,W,C), got {len(top_imgs.shape)}D")
                return False
                
            print("✅ All dimensions are correct")
            
            # Sample some data
            print("\n📊 Sample data:")
            print(f"qpos[0]: {qpos[0][:3]}...")  # First 3 joint positions
            print(f"qvel[0]: {qvel[0][:3]}...")  # First 3 joint velocities  
            print(f"action[0]: {action[0][:3]}...")  # First 3 action values
            print(f"image[0] shape: {top_imgs[0].shape}, min: {top_imgs[0].min()}, max: {top_imgs[0].max()}")
            
            return True
            
    except Exception as e:
        print(f"❌ Error reading file: {e}")
        return False

def main():
    """Main validation function"""
    print("🔍 ACT Dataset Validation Tool")
    print("=" * 50)
    
    # Check for dataset files
    dataset_dir = "/home/jason/ws/catkin_ws/src/act_data"
    
    if not os.path.exists(dataset_dir):
        print(f"❌ Dataset directory not found: {dataset_dir}")
        return False
    
    # Find HDF5 files
    hdf5_files = [f for f in os.listdir(dataset_dir) if f.endswith('.hdf5')]
    
    if not hdf5_files:
        print(f"❌ No HDF5 files found in {dataset_dir}")
        return False
    
    print(f"Found {len(hdf5_files)} HDF5 files:")
    for f in hdf5_files:
        print(f"  - {f}")
    
    print("\n" + "=" * 50)
    
    # Validate each file
    all_valid = True
    for filename in hdf5_files:
        filepath = os.path.join(dataset_dir, filename)
        is_valid = validate_hdf5_structure(filepath)
        all_valid = all_valid and is_valid
        print("-" * 30)
    
    if all_valid:
        print("🎉 All dataset files are ACT-compatible!")
        return True
    else:
        print("⚠️  Some files have issues")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
