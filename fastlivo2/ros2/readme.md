# FAST-LIVO2 Docker 部署

## 参考

- FAST-LIVO2 ros2源码： [xiayip/FAST-LIVO2](https://github.com/xiayip/FAST-LIVO2)
- FAST-LIO ros2源码：[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- ROS2文档: [docs.ros.org](https://docs.ros.org/en/humble/)

### 前提条件

- Docker 22.04+
- NVIDIA Docker运行时(可选)
- X11服务(用于图形界面)

### 构建与运行

```bash
cd fastlivo2/ros2
mkdir src
git clone https://github.com/xiayip/fast-livo2-rpg_vikit.git
git clone https://github.com/xiayip/FAST-LIVO2
git clone https://github.com/Ericsii/FAST_LIO_ROS2

# patch for FAST-LIVO2
~~~
# launch/mapping_avia.launch.py
             remappings=[
-                ("in",  "/left_camera/image"),
+                ("in/compressed",  "/left_camera/image/compressed"),

grep -ril --exclude='*.md' 'livox_ros_driver2' . | xargs sed -i 's/livox_ros_driver2/livox_ros_driver/gI'
~~~

# patch for FAST_LIO_ROS2
~~~
grep -ril --exclude='*.md' 'livox_ros_driver2' . | xargs sed -i 's/livox_ros_driver2/livox_ros_driver/gI'
~~~

cd ..
bash build_docker_images.sh -t build

# (可选)
bash build_docker_images.sh -t push

# 允许Docker访问X11服务
xhost +SI:localuser:root

# 启动容器
cd deploy
docker-compose up -d

docker exec -it fastlivo2-ros2 bash
```

### FAST-LIVO2 复现

```bash
colcon build --symlink-install --continue-on-error
source install/setup.bash

ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True
```

```bash
# 先将ros1 bag 转换为ros2 bag
rosbags-convert --src dataset/Retail_Street.bag --dst dataset/Retail_Street
# 将ros1 bag 转换为mcap
rosbags-convert --src dataset/Retail_Street.bag --dst dataset/Retail_Street_mcap --dst-storage mcap
```

### 之前已经对livox_ros_driver2统一调整为livox_ros_driver，故而已废弃
<!-- ```diff
--- a/Retail_Street.bag (ROS1)
+++ b/Retail_Street/metadata.yaml (ROS2)
@@ -1,3 +1,3 @@
 rosbag2_bagfile_information:
-  custom_data: null
+  custom_data: {}
@@ -10,7 +10,7 @@
     topic_metadata:
       name: /livox/lidar
       offered_qos_profiles: ''
-      type: livox_ros_driver/msg/CustomMsg
+      type: livox_ros_driver2/msg/CustomMsg
       type_description_hash: RIHS01_94041b4794f52c1d81def2989107fc898a62dacb7a39d5dbe80d4b55e538bf6d
   - message_count: 1355
     topic_metadata:
``` -->

```bash
rosbags-convert --src dataset/Bright_Screen_Wall.bag --dst dataset/Bright_Screen_Wall
ros2 bag play -p dataset/Bright_Screen_Wall
```

[rosbag2 doc](https://github.com/ros2/rosbag2/blob/rolling/README.md#convert)

```bash
# ros2 bag与mcap的转换
ros2 bag convert -i Retail_Street_mcap.mcap -o ros2.yaml
ros2 bag convert -i Retail_Street.db3 -o mcap.yaml
```

```yaml
# ros2.yaml
output_bags:
  - storage_id: sqlite3
    uri: Retail_Street_ros2
# mcap.yaml
output_bags:
  - storage_id: mcap
    uri: Retail_Street_mcap
```

```bash
#lichtblick
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## 许可证

MIT License
