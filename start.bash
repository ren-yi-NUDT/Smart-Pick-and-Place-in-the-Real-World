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

# 终端2: Twin IK 服务 - 左臂 (端口8020)
gnome-terminal --title="Twin IK (left)" -- bash -ic "
cd '$PROJECT_ROOT/dependence/twin_inference'
source /opt/ros/noetic/setup.bash
conda activate anygrasp
python3 twin.py --side left
exec bash"

# 终端3: Twin IK 服务 - 右臂 (端口8021)
gnome-terminal --title="Twin IK (right)" -- bash -ic "
cd '$PROJECT_ROOT/dependence/twin_inference'
source /opt/ros/noetic/setup.bash
conda activate anygrasp
python3 twin.py --side right
exec bash"

# 后台进程: Robotiq 夹爪 socket server (端口 8001)
# 先清理可能残留的旧进程，避免端口/串口冲突
if pkill -f 'gripper-programming/server.py' 2>/dev/null; then
    echo "  Killed stale gripper server, restarting ..."
    sleep 1
fi
nohup bash -c "
cd '$PROJECT_ROOT/dependence/gripper-programming'
source /home/zz/anaconda3/etc/profile.d/conda.sh
conda activate anygrasp
python3 server.py" > "$PROJECT_ROOT/dependence/gripper-programming/server.log" 2>&1 &
echo "  Robotiq gripper server PID=$! (log: dependence/gripper-programming/server.log)"

echo "Started ROS Bringup + Twin IK (left:8020 + right:8021) + Robotiq gripper server."
