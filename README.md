# ROS2 项目集合

![ROS](https://img.shields.io/badge/ROS-Noetic|Humble-blue)
![Docker](https://img.shields.io/badge/Docker-✓-blue)
![License](https://img.shields.io/badge/License-MIT-green)

本仓库包含多个ROS/ROS2相关项目的Docker化部署方案。

## 项目列表

### 1. FAST-LIVO2

- 🚀 激光-视觉-惯性里程计系统
- 📦 Docker化ROS1 Noetic环境
- [详细文档](./fastlivo2/readme.md)

### 2. ReTerminal ROS2

- 🖥️ ARM64架构ROS2容器
- 🔄 x86_64交叉编译支持
- [详细文档](./reterminal/readme.md)

### 3. Foxglove

- 📊 ROS2可视化工具套件
- 🖼️ 数据分析和可视化
- [详细文档](./foxglove/readme.md)

## 快速开始

```bash
# 克隆仓库
git clone https://github.com/your-repo/ros2.git
cd ros2

# 构建指定项目 (示例:fastlivo2)
cd fastlivo2
./build_docker_images.sh -t build
```

## 项目结构

```shell
ros2/
├── fastlivo2/      # FAST-LIVO2项目
├── reterminal/     # ReTerminal ROS2容器  
├── foxglove/       # Foxglove可视化工具
└── README.md       # 本文件
```

## 许可证

MIT License © 2025
