#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# 终端1: ROS bringup (灵巧手8000, 机械臂8010)
gnome-terminal --title="ROS Bringup" -- bash -c "
cd '$PROJECT_ROOT/dependence/smart_pick_and_place_ws'
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch '$PROJECT_ROOT/dependence/smart_pick_and_place_ws/src/pkg_launch/launch/bringup.launch'
exec bash"

# 终端2: Twin IK 服务 (端口8020)
gnome-terminal --title="Twin IK" -- bash -ic "
cd '$PROJECT_ROOT/dependence/twin_inference'
source /opt/ros/noetic/setup.bash
conda activate anygrasp
python3 twin.py
exec bash"

# 终端3: Azure Kinect 录像守护 (ping 机械臂, 开机即录)
gnome-terminal --title="Azure Recorder" -- bash -ic "
cd '$PROJECT_ROOT'
source ~/anaconda3/etc/profile.d/conda.sh
conda activate anygrasp
python3 tools/azure_recorder.py
exec bash"

# 终端4: Robotiq 85 夹爪服务 (TCP :8001 -> Modbus RTU ttyUSB0)
gnome-terminal --title="Gripper Server" -- bash -ic "
cd '$PROJECT_ROOT'
source ~/anaconda3/etc/profile.d/conda.sh
conda activate anygrasp
python3 dependence/gripper-programming/server.py --serial /dev/ttyUSB0 --slave 1
exec bash"

echo "Started ROS Bringup + Twin IK + Azure Recorder + Gripper Server in separate terminals."
