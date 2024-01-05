#!/bin/bash

echo "[${ROBOT_NAME}] - Starting ${WEBOTS_CONTROLLER_URL} ..."

source /opt/ros/humble/setup.bash

colcon build --packages-select webots_ros2
source install/local_setup.bash
ros2 launch webots_ros2 robot_launch.py