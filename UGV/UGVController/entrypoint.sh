#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/iron/setup.bash
source /home/ros2_ws/install/setup.bash

# Run both nodes
ros2 run ugv_client ugv_client_new &
ros2 run ugv_client map_server_client