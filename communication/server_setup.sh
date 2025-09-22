#!/bin/bash
# AgentLace 推理服务器启动脚本
# 功能：开启服务器的AgentLace网络端口，等待客户端连接

set -e

# 固定网络配置 - 基于当前机器网络状况
SERVER_IP="10.16.49.124"        # 主网卡IP，局域网可访问
SERVER_PORT="5555"              # AgentLace固定端口

# 可选参数
MODEL_PATH=${1:-"/home/wujielin/CascadeProjects/data/act_training/checkpoints/franka_pick_place"}
MODEL_SELECTION=${2:-"auto"}  # auto, best, latest, 或具体文件名

# 智能GPU选择逻辑：优先使用GPU 2、3，选择占用最小的
echo "🔍 检查GPU使用情况..."
GPU_INFO=$(nvidia-smi --query-gpu=index,memory.used --format=csv,noheader,nounits)
echo "$GPU_INFO"

# 获取GPU数量
GPU_COUNT=$(echo "$GPU_INFO" | wc -l)
echo "检测到 $GPU_COUNT 个GPU"

# 解析GPU 2和3的内存使用情况
GPU2_MEM=$(echo "$GPU_INFO" | awk 'NR==3 {print $2}' | tr -d ',')
GPU3_MEM=$(echo "$GPU_INFO" | awk 'NR==4 {print $2}' | tr -d ',')

if [[ $GPU_COUNT -ge 4 ]]; then
    # 如果有4个GPU，比较GPU 2和3的内存使用
    echo "GPU 2内存使用: ${GPU2_MEM}MB, GPU 3内存使用: ${GPU3_MEM}MB"
    
    if [[ $GPU2_MEM -le $GPU3_MEM ]]; then
        GPU_ID=2
        echo "✅ 选择GPU 2 (内存占用较少: ${GPU2_MEM}MB)"
    else
        GPU_ID=3
        echo "✅ 选择GPU 3 (内存占用较少: ${GPU3_MEM}MB)"
    fi
elif [[ $GPU_COUNT -ge 3 ]]; then
    # 如果只有3个GPU，使用GPU 2  
    GPU_ID=2
    echo "✅ 选择GPU 2 (内存占用: ${GPU2_MEM}MB)"
else
    # 如果GPU数量不足，使用GPU 0
    GPU_ID=0
    echo "⚠️  GPU数量不足，选择GPU 0"
fi

# 显示可用模型信息
echo "📋 检查可用模型..."
if [ -d "$MODEL_PATH" ]; then
    echo "模型目录: $MODEL_PATH"
    
    # 检查特殊模型文件
    if [ -f "$MODEL_PATH/policy_best.ckpt" ]; then
        BEST_SIZE=$(ls -lh "$MODEL_PATH/policy_best.ckpt" | awk '{print $5}')
        echo "   ✅ policy_best.ckpt (最佳验证损失) - $BEST_SIZE"
    else
        echo "   ❌ policy_best.ckpt (未找到)"
    fi
    
    if [ -f "$MODEL_PATH/policy_last.ckpt" ]; then
        LAST_SIZE=$(ls -lh "$MODEL_PATH/policy_last.ckpt" | awk '{print $5}')
        echo "   ✅ policy_last.ckpt (最终训练) - $LAST_SIZE"
    else
        echo "   ❌ policy_last.ckpt (未找到)"
    fi
    
    # 统计epoch模型
    EPOCH_COUNT=$(ls "$MODEL_PATH"/policy_epoch_*.ckpt 2>/dev/null | wc -l)
    if [ "$EPOCH_COUNT" -gt 0 ]; then
        FIRST_EPOCH=$(ls "$MODEL_PATH"/policy_epoch_*.ckpt 2>/dev/null | head -1 | grep -o 'epoch_[0-9]*' | cut -d'_' -f2)
        LAST_EPOCH=$(ls "$MODEL_PATH"/policy_epoch_*.ckpt 2>/dev/null | tail -1 | grep -o 'epoch_[0-9]*' | cut -d'_' -f2)
        echo "   📊 Epoch模型: $EPOCH_COUNT个 (Epoch $FIRST_EPOCH - $LAST_EPOCH)"
    else
        echo "   ❌ 未找到Epoch模型"
    fi
else
    echo "⚠️  模型路径不是目录: $MODEL_PATH"
fi

echo "========================================="
echo "🚀 启动 AgentLace 推理服务器"
echo "监听地址: $SERVER_IP:$SERVER_PORT"
echo "模型路径: $MODEL_PATH"
echo "模型选择: $MODEL_SELECTION (auto=优先最佳模型)"
echo "GPU设备: $GPU_ID"
echo "========================================="

# 设置GPU设备
export CUDA_VISIBLE_DEVICES=$GPU_ID

# 设置项目路径
PROJECT_ROOT="/home/wujielin/CascadeProjects/projects/ws/frk_act_unified"
export PYTHONPATH="${PYTHONPATH}:${PROJECT_ROOT}:${PROJECT_ROOT}/communication/agentlace"

# 检查AgentLace依赖
python3 -c "
try:
    import zmq
    print('✅ ZeroMQ可用')
except ImportError:
    print('❌ ZeroMQ未安装，请运行: pip install pyzmq')
    exit(1)
" || exit 1
# 创建日志目录
mkdir -p logs

# 启动AgentLace推理服务器
echo "🌐 启动AgentLace推理服务器..."
# 根据模型选择参数设置具体模型路径
if [ "$MODEL_SELECTION" != "auto" ]; then
    if [ "$MODEL_SELECTION" = "best" ]; then
        SPECIFIC_MODEL="$MODEL_PATH/policy_best.ckpt"
        echo "🎯 手动选择最佳模型: policy_best.ckpt"
    elif [ "$MODEL_SELECTION" = "latest" ]; then
        LATEST_EPOCH=$(ls "$MODEL_PATH"/policy_epoch_*.ckpt 2>/dev/null | tail -1)
        if [ -n "$LATEST_EPOCH" ]; then
            SPECIFIC_MODEL="$LATEST_EPOCH"
            EPOCH_NUM=$(echo "$LATEST_EPOCH" | grep -o 'epoch_[0-9]*' | cut -d'_' -f2)
            echo "🎯 手动选择最新Epoch模型: policy_epoch_${EPOCH_NUM}_seed_0.ckpt"
        else
            echo "❌ 未找到Epoch模型，使用默认路径"
            SPECIFIC_MODEL="$MODEL_PATH"
        fi
    elif [ -f "$MODEL_PATH/$MODEL_SELECTION" ]; then
        SPECIFIC_MODEL="$MODEL_PATH/$MODEL_SELECTION"
        echo "🎯 手动选择指定模型: $MODEL_SELECTION"
    else
        echo "⚠️  指定模型文件不存在: $MODEL_SELECTION，使用默认"
        SPECIFIC_MODEL="$MODEL_PATH"
    fi
else
    echo "🤖 使用自动模型选择 (优先policy_best.ckpt)"
    SPECIFIC_MODEL="$MODEL_PATH"
fi

echo "📁 最终模型路径: $SPECIFIC_MODEL"
echo ""

cd ../robot_server/communication/
python3 act_server.py \
    --port $SERVER_PORT \
    --model_path "$SPECIFIC_MODEL" \
    2>&1 | tee ../../logs/agentlace_server_$(date +%Y%m%d_%H%M%S).log &

SERVER_PID=$!

# 等待服务器启动
echo "⏳ 等待服务器完全启动..."
sleep 8

# 测试服务器状态
echo "🧪 服务器状态测试..."
python3 -c "
import socket
import time
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    result = sock.connect_ex(('$SERVER_IP', $SERVER_PORT))
    sock.close()
    if result == 0:
        print('✅ 网络连接: 正常')
        print(f'📡 服务地址: $SERVER_IP:$SERVER_PORT')
        print('🎯 服务器就绪，等待客户端连接')
    else:
        print('❌ 网络连接: 失败')
        exit(1)
except Exception as e:
    print(f'❌ 测试失败: {e}')
    exit(1)
" || {
    echo "❌ 服务器启动失败"
    kill $SERVER_PID 2>/dev/null
    exit 1
}

echo "========================================="
echo "✅ AgentLace推理服务器运行中"
echo "📊 状态: 已启动并监听"
echo "🌐 地址: $SERVER_IP:$SERVER_PORT" 
echo "🖥️  GPU: $GPU_ID (自动选择)"
echo "========================================="

# 等待用户中断
trap 'echo "🛑 正在停止AgentLace服务器..."; kill $SERVER_PID 2>/dev/null; exit 0' INT

# 监控服务器状态和连接
echo "🔄 开始监控服务器，等待客户端连接..."
echo "   客户端连接后将自动进行推理测试"
echo "   按 Ctrl+C 停止服务器"
echo ""

# 初始化计数器
last_request_count=0
last_success_count=0
start_time=$(date +%s)
client_connected=false

# 监控循环
while kill -0 $SERVER_PID 2>/dev/null; do
    sleep 3  # 更频繁检查
    current_time=$(date +%s)
    uptime=$((current_time - start_time))
    
    # 检查网络连接
    connections=$(netstat -an | grep ":$SERVER_PORT" | grep ESTABLISHED | wc -l)
    
    # 从日志中提取推理请求统计
    log_file=$(ls ../logs/agentlace_server_*.log 2>/dev/null | tail -1)
    if [[ -f "$log_file" ]]; then
        current_requests=$(grep -c "收到推理请求" "$log_file" 2>/dev/null || echo 0)
        current_success=$(grep -c "推理完成" "$log_file" 2>/dev/null || echo 0)
        current_errors=$(grep -c "推理失败" "$log_file" 2>/dev/null || echo 0)
        
        new_requests=$((current_requests - last_request_count))
        new_success=$((current_success - last_success_count))
        
        # 实时显示推理请求
        if [[ $new_requests -gt 0 ]]; then
            echo "$(date '+%H:%M:%S') 📨 收到推理请求 #$current_requests"
            
            # 显示最新的输入数据信息
            latest_qpos=$(grep "qpos原始" "$log_file" | tail -1 | grep -o "范围\[[^\]]*\]")
            latest_image=$(grep "image原始" "$log_file" | tail -1 | grep -o "shape([^)]*)")
            if [[ -n "$latest_qpos" && -n "$latest_image" ]]; then
                echo "$(date '+%H:%M:%S')    📊 输入数据: qpos$latest_qpos, $latest_image"
            fi
            
            last_request_count=$current_requests
        fi
        
        # 实时显示推理完成
        if [[ $new_success -gt 0 ]]; then
            # 提取最新的推理时间和动作信息
            latest_total_time=$(grep "总耗时:" "$log_file" | tail -1 | grep -o "[0-9]\+\.[0-9]\+ms")
            latest_model_time=$(grep "模型推理耗时:" "$log_file" | tail -1 | grep -o "[0-9]\+\.[0-9]\+ms")
            latest_action_range=$(grep "动作范围:" "$log_file" | tail -1 | grep -o "\[[^\]]*\]")
            latest_joint_range=$(grep "关节动作范围:" "$log_file" | tail -1 | grep -o "\[[^\]]*\]")
            latest_gripper_range=$(grep "夹爪动作范围:" "$log_file" | tail -1 | grep -o "\[[^\]]*\]")
            
            echo "$(date '+%H:%M:%S') ✅ 推理完成 #$current_success"
            echo "$(date '+%H:%M:%S')    ⏱️  性能: 总计${latest_total_time}, 模型${latest_model_time}"
            if [[ -n "$latest_action_range" ]]; then
                echo "$(date '+%H:%M:%S')    🎯 动作: 全部$latest_action_range, 关节$latest_joint_range, 夹爪$latest_gripper_range"
            fi
            
            last_success_count=$current_success
            
            # 每5个请求输出统计
            if [[ $((current_success % 5)) -eq 0 ]] && [[ $current_success -gt 0 ]]; then
                success_rate=$(echo "scale=1; ($current_success * 100) / ($current_requests)" | bc -l 2>/dev/null || echo "100.0")
                echo "$(date '+%H:%M:%S') 📊 推理统计: 总请求 $current_requests, 成功 $current_success, 错误 $current_errors, 成功率 ${success_rate}%"
            fi
        fi
        
        # 显示错误信息
        if [[ $current_errors -gt 0 ]]; then
            latest_error=$(grep "推理失败" "$log_file" | tail -1)
            if [[ -n "$latest_error" ]]; then
                echo "$(date '+%H:%M:%S') ❌ 检测到推理错误，请查看日志: $log_file"
            fi
        fi
    fi
    
    # 检测客户端连接
    if [[ $connections -gt 0 ]] && [[ "$client_connected" == "false" ]]; then
        echo "$(date '+%H:%M:%S') 🔗 客户端已连接！等待推理请求..."
        client_connected=true
    elif [[ $connections -eq 0 ]] && [[ "$client_connected" == "true" ]]; then
        echo "$(date '+%H:%M:%S') 🔌 客户端已断开连接"
        client_connected=false
    fi
    
    # 每30秒显示详细状态
    if [[ $((uptime % 30)) -eq 0 ]] && [[ $uptime -gt 0 ]]; then
        # 计算平均推理时间
        if [[ $current_success -gt 0 ]] && [[ -f "$log_file" ]]; then
            avg_time=$(grep "总耗时:" "$log_file" | tail -10 | grep -o "[0-9]\+\.[0-9]\+" | awk '{sum+=$1} END {if(NR>0) print sum/NR; else print 0}' | head -c 6)
            echo "$(date '+%H:%M:%S') 🟢 服务器运行 ${uptime}s | 连接: $connections | 推理: $current_success/${current_requests} | 平均延迟: ${avg_time}ms"
        else
            echo "$(date '+%H:%M:%S') 🟢 服务器运行 ${uptime}s | 连接: $connections | 等待推理请求..."
        fi
    fi
done

echo "🛑 AgentLace推理服务器已停止"
