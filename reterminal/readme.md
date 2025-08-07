# ReTerminal ROS2 容器

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Docker](https://img.shields.io/badge/Docker-✓-blue)
![Multi-arch](https://img.shields.io/badge/ARM64-✓-success)
![License](https://img.shields.io/badge/License-MIT-green)

在x86_64主机上构建ARM64架构的ROS2 Docker容器，用于ReTerminal设备。

## 功能特性

- ✅ 跨架构构建 (x86_64 → ARM64)
- ✅ ROS2 Humble 完整环境
- ✅ QEMU模拟支持
- ✅ Docker多阶段构建
- ✅ 自动化构建脚本

## 快速开始

### 前提条件

- x86_64 Linux主机
- Docker 20.04+
- QEMU用户态模拟

```bash
sudo apt update
sudo apt install qemu qemu-user-static
```

### 构建镜像

```bash
# 启用QEMU多架构支持
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# 构建Docker镜像
./build_docker_images.sh -t build

# 推送镜像到仓库(可选)
./build_docker_images.sh -t push
```

## 详细指南

### 1. 构建选项

| 参数 | 说明 |
|------|------|
| `-t build` | 构建Docker镜像 |
| `-t push` | 推送镜像到仓库 |
| `--no-cache` | 不使用缓存构建 |

### 2. 安全提示

⚠️ **重要安全提醒** ⚠️

- 不要在代码库中存储凭证
- 使用Docker secrets或环境变量管理密码
- 定期轮换访问凭证

### 3. 项目结构

```shell
reterminal/
├── deploy/            # 部署配置
│   └── docker-compose.yml
├── dockerfile         # Docker构建文件
└── build_docker_images.sh # 构建脚本
```

## 参考资源

- ROS2文档: [docs.ros.org](https://docs.ros.org/)
- QEMU多架构支持: [github.com/multiarch](https://github.com/multiarch)
- Docker多架构构建: [docs.docker.com](https://docs.docker.com/build/building/multi-platform/)

## 许可证

MIT License
