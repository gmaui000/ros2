# FAST-LIVO2 Docker 部署

![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![Docker](https://img.shields.io/badge/Docker-✓-blue)
![License](https://img.shields.io/badge/License-MIT-green)

FAST-LIVO2 是基于ROS的激光-视觉-惯性里程计系统。本仓库提供Docker化部署方案。

## 功能特性

- ✅ 激光-视觉-惯性多传感器融合
- ✅ 实时SLAM建图
- ✅ Docker容器化部署
- ✅ 支持图形化界面(RViz)
- ✅ 支持GPU加速

## 快速开始

### 前提条件

- Docker 20.04+
- NVIDIA Docker运行时(可选)
- X11服务(用于图形界面)

### 构建与运行

```bash
# 构建Docker镜像
./build_docker_images.sh -t build

# 推送镜像到仓库(可选)
./build_docker_images.sh -t push

# 允许Docker访问X11服务
xhost +local:root

# 启动容器
cd deploy
docker-compose up -d
```

## 详细指南

### 1. 构建选项

| 参数 | 说明 |
|------|------|
| `-t build` | 构建Docker镜像 |
| `-t push` | 推送镜像到仓库 |
| `--no-cache` | 不使用缓存构建 |

### 2. 图形界面支持

确保主机已安装X11服务，并运行：

```bash
xhost +local:root
```

### 3. 访问容器

```bash
docker exec -it fastlivo2 bash
```

在容器内启动RViz:

```bash
rviz
```

## 项目结构

```shell
fastlivo2/
├── deploy/            # 部署配置
│   ├── docker-compose.yml
│   └── start.sh
├── dockerfile         # Docker构建文件
└── src/               # 项目源码
```

## 参考

- 原项目: [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)
- ROS文档: [ros.org](https://www.ros.org/)

## 许可证

MIT License
