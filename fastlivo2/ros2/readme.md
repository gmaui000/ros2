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
# colcon build --symlink-install --continue-on-error --packages-select livox2pc
source install/setup.bash

ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True

ros2 launch fast_livo mapping_cowa.launch.py use_rviz:=True

ros2 launch fast_lio mapping_cowa.launch.py
```

```bash
# 查看话题详情：
ros2 topic echo /rgb_img --once

# 查看tf 
ros2 run tf2_ros tf2_echo imu main/lidar

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
# dataset
rosbags-convert --src Bright_Screen_Wall.bag --dst Bright_Screen_Wall
ros2 bag play Bright_Screen_Wall

# dataset/14081/
rosbags-convert --src 14081.bag --dst 14081_ros2
ros2 bag play 14081_ros2 --qos-profile-overrides-path qos_config.yaml --start-offset 125
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
ros2 launch foxglove_bridge foxglove_bridge_launch.xml debug:=true
```

```bash
ros2 launch  mvs_ros_driver mvs_camera_trigger.py

ros2 launch livox_ros_driver msg_MID360_launch.py

ros2 launch livox2pc livox2std_launch.py

```

# run on RK3588

```bash
adb shell

```

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ca-certificates curl gnupg lsb-release

sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/docker.gpg] https://mirrors.aliyun.com/docker-ce/linux/ubuntu focal stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

sudo systemctl start docker
sudo systemctl enable docker
sudo systemctl status docker

sudo usermod -aG docker $USER
newgrp docker

sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<-'EOF'
{
  "registry-mirrors": [
    "https://hub-mirror.c.163.com",
    "https://mirror.aliyuncs.com",
    "https://docker.mirrors.ustc.edu.cn"
  ]
}
EOF

sudo systemctl restart docker
docker run --rm hello-world
docker run -it --rm ubuntu:22.04

docker run -d -p 80:80 --name nginx-test nginx:alpine

```


# 标定

```bash
ros2 launch  mvs_ros_driver mvs_camera_trigger.py

# 12 x 9 黑白格，单个格子宽度5cm
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.050 image:=/left_camera/image

docker cp fastlivo2-ros2:/tmp/calibrationdata.tar.gz .

#ros2 topic pub -r 10 /camera_info sensor_msgs/msg/CameraInfo "{height: 512, width: 640, k: [639.1123,0,337.3092,0,638.7531,250.5352,0,0,1], d: [-0.061406,0.06985,0.001111,0.001739,0], r: [1,0,0,0,1,0,0,0,1], p: [630.8092,0,337.8753,0,0,632.3165,250.3365,0,0,0,1,0], distortion_model: 'plumb_bob'}"

ros2 topic pub -r 10 /camera_info sensor_msgs/msg/CameraInfo "{header: {frame_id: '/left_camera/image'}, height: 512, width: 640, distortion_model: 'plumb_bob', d: [-0.064706, 0.086212, 0.000207, 0.000288, 0.0], k: [639.470092, 0.0, 332.613927, 0.0, 638.908881, 247.858794, 0.0, 0.0, 1.0], r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], p: [631.488159, 0.0, 332.371498, 0.0, 0.0, 632.378540, 247.341559, 0.0, 0.0, 0.0, 1.0, 0.0]}" 


```

# 自启动

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml debug:=true
ros2 launch  mvs_ros_driver mvs_camera_trigger.py
ros2 launch livox_ros_driver msg_MID360_launch.py
ros2 launch livox2pc livox2std_launch.py

ros2 launch fast_livo mapping_mid360.launch.py use_rviz:=True image_convert:=True


```

## 许可证

MIT License
