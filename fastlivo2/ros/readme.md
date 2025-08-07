# FAST-LIVO2 Docker 部署

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
docker exec -it fastlivo2-ros bash
```

### 3. FAST-LIVO2 复现

```bash
catkin_make
source devel/setup.bash
roslaunch fast_livo mapping_avia.launch
```

```bash
source devel/setup.bash
rosbag play dataset/Bright_Screen_Wall.bag
```

```bash
source devel/setup.bash
rostopic list

root@caobing-Lenovo-Legion-R9000K2021H:~/fastlivo2# rostopic list
/LIVO2/imu_propagate
/Laser_map
/aft_mapped_to_init
/clicked_point
/cloud_effected
/cloud_registered
/cloud_visual_sub_map_before
/dyn_obj
/dyn_obj_dbg_hist
/dyn_obj_removed
/fsm_node/visualization/exp_traj
/initialpose
/left_camera/image
/left_camera/image/compressed
/livox/imu
/livox/lidar
/mavros/vision_pose/pose
/move_base_simple/goal
/path
/planes
/planner_normal
/planner_normal_array
/republish/compressed/parameter_descriptions
/republish/compressed/parameter_updates
/rgb_img
/rgb_img/compressed
/rgb_img/compressed/parameter_descriptions
/rgb_img/compressed/parameter_updates
/rgb_img/compressedDepth
/rgb_img/compressedDepth/parameter_descriptions
/rgb_img/compressedDepth/parameter_updates
/rgb_img/mouse_click
/rgb_img/theora
/rgb_img/theora/parameter_descriptions
/rgb_img/theora/parameter_updates
/rosout
/rosout_agg
/tf
/tf_static
/visualization_marker
/voxels

```

#### 在容器内启动rviz

```bash
rviz
```

## 参考

- 原项目: [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)
- ROS文档: [ros.org](https://www.ros.org/)

## 许可证

MIT License
