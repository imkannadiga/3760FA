#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/iron/setup.bash

# Run both nodes
ros2 launch slam_toolbox online_async_launch.py &
ros2 launch nav2_bringup navigation_launch.py
