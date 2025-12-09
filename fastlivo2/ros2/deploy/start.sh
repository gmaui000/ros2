#!/bin/bash

source /opt/ros/humble/setup.bash
source /root/livox2/install/setup.bash
source /root/fastlivo2/install/setup.bash

SHELL_FOLDER=$(cd "$(dirname "$0")"; pwd)

# 日志目录
LOG_DIR="$SHELL_FOLDER/logs"
mkdir -p "$LOG_DIR"

# ==================== 可扩展的多级启动配置 ====================
# 定义所有可用的节点及其启动命令
declare -A ALL_NODES
ALL_NODES["foxglove"]="ros2 launch foxglove_bridge foxglove_bridge_launch.xml max_qos_depth:=1 send_buffer_limit:=2000000 debug:=true"
ALL_NODES["livox"]="ros2 launch livox_ros_driver msg_MID360_launch.py"
ALL_NODES["mvs"]="ros2 launch mvs_ros_driver mvs_camera_trigger.py"
ALL_NODES["fast_livo"]="ros2 launch fast_livo mapping_mid360.launch.py"
ALL_NODES["supervisor"]="ros2 launch supervisor supervisor.launch.py"
# ALL_NODES["supervisor"]="ros2 launch supervisor supervisor.launch.py livox_enabled:=true mvs_enabled:=true fast_livo_enabled:=true"

# 定义启动级别配置（级别 -> 节点列表，用空格分隔）
# 级别说明：
#   0: 什么都不做，只睡眠
#   1: 只启动 supervisor
#   2: 只启动 foxglove
#   3: 启动 foxglove + mvs
declare -A LEVEL_CONFIG
LEVEL_CONFIG["0"]=""  # 空列表，不启动任何节点
LEVEL_CONFIG["1"]="supervisor"
LEVEL_CONFIG["2"]="foxglove"
LEVEL_CONFIG["3"]="foxglove livox mvs fast_livo"

# 保存PID
declare -A PIDS

# ==================== 启动逻辑 ====================
# 解析 AUTO_START 环境变量
AUTO_START="${AUTO_START:-0}"  # 默认值为0（不启动）

echo "AUTO_START level: ${AUTO_START}"

# 确定要启动的节点列表
if [[ -n "${LEVEL_CONFIG[$AUTO_START]}" ]]; then
    # 使用预定义的级别配置
    NODES_TO_START="${LEVEL_CONFIG[$AUTO_START]}"
    echo "Using predefined level ${AUTO_START} configuration"
elif [[ "$AUTO_START" =~ ^[0-9]+$ ]]; then
    # 数字级别但未预定义，视为级别0
    echo "Level ${AUTO_START} not predefined, treating as level 0 (no nodes)"
    NODES_TO_START=""
fi

# 检查是否没有任何节点要启动
if [ -z "$NODES_TO_START" ]; then
    echo "No nodes to start. Entering sleep loop..."
    while true; do
        sleep 60
    done
    exit 0
fi

# 验证节点是否存在
VALID_NODES=""
for node in $NODES_TO_START; do
    if [[ -n "${ALL_NODES[$node]}" ]]; then
        VALID_NODES="$VALID_NODES $node"
    else
        echo "Warning: Node '$node' is not defined in ALL_NODES, skipping."
    fi
done

if [ -z "$VALID_NODES" ]; then
    echo "No valid nodes to start. Entering sleep loop..."
    while true; do
        sleep 60
    done
    exit 0
fi

echo "Starting nodes:${VALID_NODES}"

sleep 5
# ==================== 节点管理函数 ====================
# 启动节点，输出到终端 + 日志文件
start_node() {
    local name=$1
    local cmd=$2
    local logfile="$LOG_DIR/$name.log"

    echo "Starting $name ..."
    # 使用 nohup + tee 保留日志在终端可见，同时写入日志
    # 每次启动前备份旧日志
    if [ -f "$logfile" ]; then
        mv "$logfile" "$logfile.old"
    fi

    nohup bash -c "$cmd" 2>&1 | tee -a "$logfile" &
    PIDS[$name]=$!
    echo "$name PID: ${PIDS[$name]}"
}

# 监控节点，异常自动重启
monitor_nodes() {
    while true; do
        for name in "${!PIDS[@]}"; do
            pid=${PIDS[$name]}
            if ! kill -0 "$pid" 2>/dev/null; then
                echo "[$(date)] $name crashed or exited. Restarting..."
                start_node "$name" "${ALL_NODES[$name]}"
            fi
        done
        sleep 5
    done
}

# ==================== 启动节点 ====================
for node in $VALID_NODES; do
    start_node "$node" "${ALL_NODES[$node]}"
    sleep 1
done

# ==================== 进入监控循环 ====================
monitor_nodes
