#!/bin/bash

#===============================================================================
# Automated ACT Dataset Collection Script
#
# MAIN FUNCTIONALITY:
# 自动化ACT数据集收集的主控脚本，协调整个数据录制流程
#
# EXECUTION STATES:
# 1. ENVIRONMENT_VALIDATION - 验证ROS环境和依赖
# 2. ROSCORE_STARTUP - 启动/检查roscore
# 3. SIMULATION_LAUNCH - 启动Gazebo仿真环境
# 4. RECORDER_STARTUP - 启动数据集录制器节点
# 5. EPISODE_RECORDING - 执行N个episode的数据录制
# 6. PROCESS_CLEANUP - 清理所有进程
# 7. DATA_VALIDATION - 验证生成的HDF5文件
# 8. REPORT_GENERATION - 生成收集报告
#
# STATE TRANSITION CONDITIONS:
# - ENVIRONMENT_VALIDATION → ROSCORE_STARTUP: 环境检查通过
# - ROSCORE_STARTUP → SIMULATION_LAUNCH: roscore运行确认
# - SIMULATION_LAUNCH → RECORDER_STARTUP: 仿真初始化完成(10s延迟)
# - RECORDER_STARTUP → EPISODE_RECORDING: 录制器节点启动确认
# - EPISODE_RECORDING → PROCESS_CLEANUP: 所有episode完成或错误
# - PROCESS_CLEANUP → DATA_VALIDATION: 进程清理完成
# - DATA_VALIDATION → REPORT_GENERATION: 数据验证完成
# - REPORT_GENERATION → EXIT: 报告生成完成
#
# PROCESS COORDINATION:
# - 通过ROS服务调用触发单个episode录制 (/franka_controller/trigger_grasp)
# - 使用PID跟踪和管理子进程生命周期
# - 错误时自动清理所有相关进程
# - 支持超时保护和优雅降级
#
# ERROR HANDLING:
# - 各阶段失败时执行cleanup()函数
# - 进程启动失败时记录日志并退出
# - 超时检测和自动恢复机制
#
# USAGE:
# ./automated_data_collection.sh [episodes_count]
# Default episodes: 5
#===============================================================================

set -e  # Exit on any error

# Configuration
WORKSPACE_DIR="/home/jason/ws/catkin_ws"
DATASET_DIR="$WORKSPACE_DIR/src/act_data"
NUM_EPISODES=${1:-5}  # Default to 5 episodes if not specified
GRASP_TIMEOUT=60      # Maximum time to wait for grasp completion
SETUP_DELAY=10         # Delay for systems to initialize

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${BLUE}[$(date '+%H:%M:%S')]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[$(date '+%H:%M:%S')] $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}[$(date '+%H:%M:%S')] $1${NC}"
}

log_error() {
    echo -e "${RED}[$(date '+%H:%M:%S')] $1${NC}"
}

# Cleanup function
cleanup() {
    log " Cleaning up processes..."
    
    # Kill ROS processes gracefully
    if [[ $SIMULATION_PID ]]; then
        log "Stopping simulation (PID: $SIMULATION_PID)..."
        kill -TERM $SIMULATION_PID 2>/dev/null || true
        wait $SIMULATION_PID 2>/dev/null || true
    fi
    
    if [[ $RECORDER_PID ]]; then
        log "Stopping dataset recorder (PID: $RECORDER_PID)..."
        kill -TERM $RECORDER_PID 2>/dev/null || true
        wait $RECORDER_PID 2>/dev/null || true
    fi
    
    # Kill any remaining ROS processes
    pkill -f "franka_grasping_record" 2>/dev/null || true
    pkill -f "franka_act_dataset_recorder" 2>/dev/null || true
    pkill -f "roslaunch" 2>/dev/null || true
    
    log "Cleanup completed"
}

# Set trap for cleanup on script exit
trap cleanup EXIT INT TERM

# Validate environment
validate_environment() {
    log " Validating environment..."
    
    # Check if in catkin workspace
    if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
        log_error "Catkin workspace not found at $WORKSPACE_DIR"
        exit 1
    fi
    
    # Check if launch file exists
    if [[ ! -f "$WORKSPACE_DIR/src/frk_act_unified/robot_client/launch/franka_grasping_record.launch" ]]; then
        log_error "Launch file not found: franka_grasping_record.launch"
        exit 1
    fi
    
    # Check if dataset recorder exists
    if [[ ! -f "$WORKSPACE_DIR/src/frk_act_unified/robot_client/data_record/franka_act_dataset_recorder.py" ]]; then
        log_error "Dataset recorder not found: franka_act_dataset_recorder.py"
        exit 1
    fi
    
    # Create dataset directory
    mkdir -p "$DATASET_DIR"
    
    log_success "Environment validation completed"
}

# Start ROS core if needed
start_roscore() {
    if ! pgrep -f "roscore" > /dev/null; then
        log " Starting roscore..."
        cd "$WORKSPACE_DIR"
        source devel/setup.bash
        roscore &
        ROSCORE_PID=$!
        sleep 3
        log_success "roscore started (PID: $ROSCORE_PID)"
    else
        log " roscore already running"
    fi
}

# Start simulation
start_simulation() {
    log " Starting Franka grasping simulation..."
    cd "$WORKSPACE_DIR"
    source devel/setup.bash
    
    # Launch simulation in background
    roslaunch frk_act_unified franka_grasping_record.launch > /tmp/simulation.log 2>&1 &
    SIMULATION_PID=$!
    
    log "Waiting for simulation to initialize..."
    sleep $SETUP_DELAY
    
    # Check if simulation is still running
    if ! kill -0 $SIMULATION_PID 2>/dev/null; then
        log_error "Simulation failed to start. Check /tmp/simulation.log"
        exit 1
    fi
    
    log_success "Simulation started successfully (PID: $SIMULATION_PID)"
}

# Start dataset recorder
start_recorder() {
    log " Starting ACT dataset recorder..."
    cd "$WORKSPACE_DIR"
    source devel/setup.bash
    
    # Start recorder in background
    python3 src/frk_act_unified/robot_client/data_record/franka_act_dataset_recorder.py > /tmp/recorder.log 2>&1 &
    RECORDER_PID=$!
    
    log "Waiting for recorder to initialize..."
    sleep 5
    
    # Check if recorder is still running
    if ! kill -0 $RECORDER_PID 2>/dev/null; then
        log_error "Dataset recorder failed to start. Check /tmp/recorder.log"
        exit 1
    fi
    
    log_success "Dataset recorder started successfully (PID: $RECORDER_PID)"
}

# Record episodes automatically
record_episodes() {
    log " Starting automated grasp-based episode recording..."
    log "Target episodes: $NUM_EPISODES"
    log "Recording triggered by grasp phases automatically"
    
    for ((i=1; i<=NUM_EPISODES; i++)); do
        log " Triggering grasp sequence $i/$NUM_EPISODES..."
        
        # Trigger a grasp sequence (this will automatically record via phase callbacks)
        rosservice call /franka_controller/trigger_grasp "{}" >/dev/null 2>&1
        
        # Wait for grasp sequence to complete (monitor phase or timeout)
        log "  Waiting for grasp sequence completion (max ${GRASP_TIMEOUT}s)..."
        
        # Monitor completion with timeout
        start_time=$(date +%s)
        completed=false
        
        while [[ $(($(date +%s) - start_time)) -lt $GRASP_TIMEOUT ]]; do
            # Check if we have a new dataset file (indicating completion)
            current_files=$(ls -1 $DATASET_DIR/*.hdf5 2>/dev/null | wc -l)
            if [[ $current_files -ge $i ]]; then
                completed=true
                break
            fi
            sleep 2
        done
        
        if [[ $completed == true ]]; then
            log_success "Grasp sequence $i completed and episode saved automatically"
        else
            log_error "Grasp sequence $i timed out after ${GRASP_TIMEOUT}s"
            # Try to reset if stuck
            rostopic pub -1 /franka_controller/episode_control std_msgs/String "data: 'reset_episode'" >/dev/null 2>&1
        fi
        
        # Brief pause between episodes
        if [[ $i -lt $NUM_EPISODES ]]; then
            log "  Pausing before next sequence..."
            sleep 5
        fi
    done
    
    log_success "All $NUM_EPISODES episodes recorded!"
}

# Validate collected data
validate_data() {
    log "🔍 Validating collected dataset..."
    
    cd "$WORKSPACE_DIR"
    
    # Run validation script
    if python3 src/frk_act_unified/scripts/validate_act_dataset.py > /tmp/validation.log 2>&1; then
        log_success "Dataset validation completed successfully"
        
        # Show validation results
        log "📊 Dataset Validation Report:"
        echo "----------------------------------------"
        cat /tmp/validation.log
        echo "----------------------------------------"
    else
        log_error "Dataset validation failed. Check /tmp/validation.log"
        cat /tmp/validation.log
    fi
}

# Generate summary report
generate_report() {
    log "📈 Generating collection summary report..."
    
    # Count HDF5 files
    local hdf5_count=$(find "$DATASET_DIR" -name "*.hdf5" | wc -l)
    local total_size=$(du -sh "$DATASET_DIR" | cut -f1)
    
    echo ""
    echo "============================================="
    echo "🎯 ACT Dataset Collection Summary Report"
    echo "============================================="
    echo "📅 Collection Date: $(date)"
    echo "🎬 Target Episodes: $NUM_EPISODES"
    echo "📁 Dataset Directory: $DATASET_DIR"
    echo "📊 HDF5 Files Created: $hdf5_count"
    echo "💾 Total Dataset Size: $total_size"
    echo ""
    
    if [[ $hdf5_count -eq $NUM_EPISODES ]]; then
        echo "✅ SUCCESS: All episodes collected successfully!"
    elif [[ $hdf5_count -gt 0 ]]; then
        echo "⚠️  PARTIAL: $hdf5_count/$NUM_EPISODES episodes collected"
    else
        echo "❌ FAILURE: No episodes collected"
    fi
    
    echo ""
    echo "📋 Latest Dataset Files:"
    find "$DATASET_DIR" -name "*.hdf5" -exec ls -lh {} \; | tail -5
    echo "============================================="
}

# Main execution
main() {
    echo "🚀 Starting Automated ACT Dataset Collection"
    echo "=============================================="
    
    validate_environment
    start_roscore
    start_simulation
    start_recorder
    record_episodes
    
    log "🛑 Stopping processes for data validation..."
    cleanup
    
    # Brief pause before validation
    sleep 5
    
    validate_data
    generate_report
    
    log_success "🎉 Automated data collection completed!"
}

# Execute main function
main "$@"
