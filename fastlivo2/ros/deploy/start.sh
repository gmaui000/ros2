#!/bin/bash
SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)

sed -e

echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

source /opt/ros/noetic/setup.bash && roscore &

while true; do sleep 10; done
