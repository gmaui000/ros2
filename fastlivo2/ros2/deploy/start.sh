#!/bin/bash
SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)

sed -e

sleep 10

ros2 launch foxglove_bridge foxglove_bridge_launch.xml debug:=true &
sleep 1

ros2 launch livox_ros_driver msg_MID360_launch.py &
sleep 1

ros2 launch  mvs_ros_driver mvs_camera_trigger.py &
sleep 1

ros2 launch livox2pc livox2std_launch.py &
sleep 1

ros2 launch fast_livo mapping_mid360.launch.py &

while true; do sleep 10; done
