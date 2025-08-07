#!/bin/bash
SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)

sed -e

echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
echo "source /root/livox2/install/setup.bash" >> /root/.bashrc

while true; do sleep 10; done
