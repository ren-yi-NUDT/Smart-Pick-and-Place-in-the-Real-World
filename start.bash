#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# 终端1: ROS bringup (灵巧手8000, 机械臂8010)
gnome-terminal --title="ROS Bringup" -- bash -c "
cd '$PROJECT_ROOT/dependence/smart_pick_and_place_ws'
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch pkg_launch bringup.launch
exec bash"

# 终端2: Twin IK 服务 (端口8020)
gnome-terminal --title="Twin IK" -- bash -ic "
cd '$PROJECT_ROOT/dependence/twin_inference'
source /opt/ros/noetic/setup.bash
conda activate anygrasp
python3 twin.py
exec bash"

echo "Started ROS Bringup + Twin IK in separate terminals."
