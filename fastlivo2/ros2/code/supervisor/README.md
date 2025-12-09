# ROS2 Supervisor

一个可配置的 ROS2 监督节点，用于管理多个 ROS2 节点和进程。

## 功能特性

- **节点管理**：通过 YAML 配置文件定义和管理节点
- **自动重启**：节点失败时自动重启（可配置重试次数）
- **健康检查**：基础健康检查支持
- **录制控制**：集成 rosbag 录制，支持时长控制
- **安全认证**：可选的令牌认证
- **状态发布**：定期发布节点状态
- **日志管理**：自动日志轮转和文件输出
- **参数覆盖**：启动时通过参数覆盖节点使能状态

## 快速开始

### 安装
```bash
cd /path/to/workspace
colcon build --packages-select supervisor
source install/setup.bash
```

### 启动 Supervisor
```bash
# 使用 launch 文件
ros2 launch supervisor supervisor.launch.py

# 直接运行
ros2 run supervisor supervisor
```

## 配置

### 配置文件 (config/nodes.yaml)
```yaml
nodes:
  fast_livo:
    enabled: true
    launch_command: ["ros2", "launch", "fast_livo", "mapping_mid360.launch.py"]
    description: "FAST-LIVO SLAM 节点"
    restart_on_failure: true
    
  foxglove:
    enabled: true
    launch_command: ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]
    args: ["address:=localhost", "max_qos_depth:=1"]
    description: "Foxglove 可视化桥接"
    restart_on_failure: true

recording:
  default_topics: ["/livox/imu", "/livox/lidar"]
  default_duration: 300
  storage_path: "/root/fastlivo2/dataset/rosbags"
```

## 参数覆盖

启动时可通过参数覆盖配置文件的节点使能状态：

```bash
# 启用 fast_livo（覆盖配置文件的 false 设置）
ros2 launch supervisor supervisor.launch.py fast_livo_enabled:=true

# 禁用 foxglove（覆盖配置文件的 true 设置）
ros2 launch supervisor supervisor.launch.py foxglove_enabled:=false

# 同时控制多个节点
ros2 launch supervisor supervisor.launch.py \
  fast_livo_enabled:=true \
  livox_enabled:=true \
  mvs_enabled:=false
```

**参数值**：`''`（使用配置）、`'true'`/`'1'`/`'yes'`/`'on'`（启用）、`'false'`/`'0'`/`'no'`/`'off'`（禁用）

## 命令控制

向 `/supervisor/command` 主题发送 JSON 命令控制 supervisor：

### 基本语法
```bash
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"命令名\", \"payload\": {...}}"}'
```

### 常用命令示例

#### 启动/停止节点
```bash
# 启动 mvs 节点
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"start_node\", \"payload\": {\"node\": \"mvs\"}}"}'

# 停止 mvs 节点
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"stop_node\", \"payload\": {\"node\": \"mvs\"}}"}'

# 启动所有节点
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"start_all\"}"}'

# 停止所有节点
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"stop_all\"}"}'

# 重启节点
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"restart_node\", \"payload\": {\"node\": \"foxglove\"}}"}'
```

#### 录制控制
```bash
# 开始录制（默认使用日期时间格式文件名）
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"start_record\", \"payload\": {\"topics\": [\"/livox/imu\"], \"duration\": 60}}"}'

# 自定义文件名
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"start_record\", \"payload\": {\"name\": \"my_recording\", \"topics\": [\"/livox/imu\"], \"duration\": 120}}"}'

# 停止录制
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"stop_record\"}"}'
```

#### 状态查询
```bash
# 获取状态
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"get_status\"}"}'
```

### 完整命令列表
- `start_node` - 启动特定节点
- `stop_node` - 停止特定节点
- `start_all` - 启动所有节点
- `stop_all` - 停止所有节点
- `restart_node` - 重启特定节点
- `start_record` - 开始录制
- `stop_record` - 停止录制
- `get_status` - 获取状态

## 状态监控

### 查看状态
```bash
# 查看原始 JSON 状态
ros2 topic echo /supervisor/status

# 使用 jq 格式化查看
ros2 topic echo -n 1 /supervisor/status std_msgs/String | grep 'data:' | cut -d':' -f2- | jq '.'
```

### 状态内容
状态消息包含：
- 主机名和时间戳
- 所有节点状态（运行/停止、PID、运行时间）
- 录制状态（是否活动、bag 名称、剩余时间）
- Supervisor 信息（配置文件、节点数量）

## 故障排除

### 常见问题
1. **命令未生效**：
   ```bash
   # 检查 supervisor 是否运行
   ros2 node list | grep supervisor
   
   # 查看 supervisor 日志
   ros2 topic echo /rosout | grep supervisor
   ```

2. **JSON 格式错误**：
   ```bash
   # 验证 JSON 格式
   echo '{"command": "start_node", "payload": {"node": "mvs"}}' | python3 -m json.tool
   ```

3. **节点未找到**：
   - 确保节点在 `config/nodes.yaml` 中正确定义
   - 检查节点名称拼写

### 录制文件名
- **默认格式**：`rosbag_YYYYMMDD_HHMMSS`（如 `rosbag_20251209_154401`）
- **自定义**：通过命令的 `name` 字段指定

## 安全配置

启用令牌认证：
```yaml
security:
  require_auth_token: true
  allowed_tokens:
    - "your_secret_token_1"
    - "your_secret_token_2"
```

带令牌的命令：
```bash
ros2 topic pub -1 /supervisor/command std_msgs/String '{"data": "{\"command\": \"start_node\", \"payload\": {\"node\": \"mvs\"}, \"auth_token\": \"your_secret_token\"}"}'
```

## 日志

日志存储在 `~/.ros/supervisor_logs/` 目录，按节点和时间戳命名。

## 扩展

### 添加新节点
1. 在 `config/nodes.yaml` 中添加节点配置
2. 确保启动命令正确
3. 重启 supervisor 或发送 `start_node` 命令

### 健康检查
在 `supervisor_node.py` 的 `_health_check()` 方法中实现自定义健康检查逻辑。

## 许可证

Apache 2.0
