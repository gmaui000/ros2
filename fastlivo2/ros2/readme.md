# FAST-LIVO2 Docker 部署

## 快速开始

### 前提条件

- Docker 22.04+
- NVIDIA Docker运行时(可选)
- X11服务(用于图形界面)

### 构建与运行

```bash
# 构建Docker镜像
./build_docker_images.sh -t build

# 推送镜像到仓库(可选)
./build_docker_images.sh -t push

# 允许Docker访问X11服务
xhost +SI:localuser:root

# 启动容器
cd deploy
docker-compose up -d
```

## 详细指南

### 1. 图形界面支持

确保主机已安装X11服务，并运行：

```bash
xhost +SI:localuser:root
```

### 2. 访问容器

```bash
docker exec -it fastlivo2-ros2 bash
```

### 3. FAST-LIVO2 复现

```bash
colcon build --symlink-install --continue-on-error
source install/setup.bash

ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True
```

```bash
rosbags-convert --src dataset/Bright_Screen_Wall.bag --dst dataset/Bright_Screen_Wall
ros2 bag play -p dataset/Bright_Screen_Wall.bag
```

## 参考

- ros2原项目: [xiayip/FAST-LIVO2](https://github.com/xiayip/FAST-LIVO2)
- ROS2文档: [docs.ros.org](https://docs.ros.org/en/humble/)

## 许可证

MIT License
