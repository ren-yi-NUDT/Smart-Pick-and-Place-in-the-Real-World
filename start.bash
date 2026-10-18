#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# 单窗口多标签启动（避免同时弹 5 个窗口 + 同时抢资源导致死机）
# 标签1: ROS bringup (机械臂8010)
gnome-terminal --title="ROS Bringup" --tab -- bash -c "
cd '$PROJECT_ROOT/dependence/smart_pick_and_place_ws'
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch '$PROJECT_ROOT/dependence/smart_pick_and_place_ws/src/pkg_launch/launch/bringup.launch'
exec bash"
sleep 5   # 等 catkin_make/roslaunch 起来再启动其它服务

# 标签2: Twin IK 服务 (端口8020)
gnome-terminal --title="Twin IK" --tab -- bash -ic "
cd '$PROJECT_ROOT/dependence/twin_inference'
source /opt/ros/noetic/setup.bash
conda activate anygrasp
python3 twin.py
exec bash"
sleep 3

# 标签3: Azure Kinect 录像、检测和只读本地视觉服务 (:8792)
gnome-terminal --title="Azure Recorder" --tab -- bash -ic "
cd '$PROJECT_ROOT'
source ~/anaconda3/etc/profile.d/conda.sh
conda activate anygrasp
python3 tools/azure_recorder.py --host 127.0.0.1 --port 8792
exec bash"
sleep 3

# 标签4: Robotiq 85 夹爪服务 (TCP :8001 -> Modbus RTU)
# 串口用 by-id 路径: ttyUSB 编号会漂移, FT231X DU0E613N 是当前装机的那个适配器
gnome-terminal --title="Gripper Server" --tab -- bash -ic "
cd '$PROJECT_ROOT'
source ~/anaconda3/etc/profile.d/conda.sh
conda activate anygrasp
python3 dependence/gripper-programming/server.py --serial /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E613N-if00-port0 --slave 1
exec bash"
sleep 3

# 标签5: Open-Jev 推理服务 (端口8791) —— run_nl.py 解析自然语言指令的后端
gnome-terminal --title="Open-Jev" --tab -- bash -c "
cd '$PROJECT_ROOT/Open-Jev'
source .venv/bin/activate
python -m jev.server --checkpoint models/Open-Jev-2B/package/checkpoint --max-length 4096
exec bash"

echo "Started ROS Bringup + Twin IK + Azure Recorder + Gripper Server + Open-Jev in one window (5 tabs)."
echo "现在可以: conda activate anygrasp && python run_nl.py   # 直接打中文下指令"
