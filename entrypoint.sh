#!/usr/bin/env bash
set -e

# ROS 2 Jazzy 환경 불러오기
source /opt/ros/jazzy/setup.bash

# 작업공간을 빌드한 경우 자동으로 환경 불러오기
if [ -f "/home/jinsoo/ros2_ws/install/setup.bash" ]; then
    source /home/jinsoo/ros2_ws/install/setup.bash
fi

cd /home/jinsoo/ros2_ws

exec "$@"
