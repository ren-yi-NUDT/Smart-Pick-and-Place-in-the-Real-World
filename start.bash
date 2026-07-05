#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# 清理上次残留的进程
# 注意：进程 cmdline 里只有 "python3 server.py"，不含目录路径，所以 pkill 模式要按实际命令行匹配
for pattern in 'python3 server.py' 'python3 anygrasp_server.py' 'python3 twin.py'; do
    PIDS=$(pgrep -f "$pattern" || true)
    if [ -n "$PIDS" ]; then
        echo "  Killing stale processes matching '$pattern': $PIDS"
        kill $PIDS 2>/dev/null || true
    fi
done
sleep 1

# 检查 xfce4-terminal
if ! command -v xfce4-terminal >/dev/null 2>&1; then
    echo "ERROR: xfce4-terminal not installed." >&2
    echo "       Install with: sudo apt install xfce4-terminal" >&2
    exit 1
fi

# xfce4-terminal 单次调用多 tab（不像 gnome-terminal 3.36 那样 -- 之后吞掉所有 --tab 参数）
# 每个 --tab 引入一个 tab spec，--working-directory / --command 都作用于"最后一个 --tab"
xfce4-terminal \
    --tab --title="ROS Bringup" \
    --working-directory="$PROJECT_ROOT/dependence/smart_pick_and_place_ws" \
    --command="bash -c 'catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && source devel/setup.bash && roslaunch pkg_launch bringup.launch; exec bash'" \
    --tab --title="Twin IK (left)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && python3 twin.py --side left; exec bash'" \
    --tab --title="Twin IK (right)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && python3 twin.py --side right; exec bash'" \
    --tab --title="Gripper Server (:8001)" \
    --working-directory="$PROJECT_ROOT/dependence/gripper-programming" \
    --command="bash -ic 'source /home/zz/anaconda3/etc/profile.d/conda.sh && conda activate anygrasp && python3 server.py; exec bash'" \
    --tab --title="AnyGrasp Server (:8030)" \
    --working-directory="$PROJECT_ROOT/dependence/anygrasp_server" \
    --command="bash -ic 'source /home/zz/anaconda3/etc/profile.d/conda.sh && conda activate anygrasp && export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && python3 anygrasp_server.py; exec bash'"

echo
echo "Started xfce4-terminal window with 5 tabs:"
echo "  Tab 1: ROS Bringup         (灵巧手 8000, 机械臂 8010)"
echo "  Tab 2: Twin IK left        (端口 8020)"
echo "  Tab 3: Twin IK right       (端口 8021)"
echo "  Tab 4: Gripper Server      (端口 8001)"
echo "  Tab 5: AnyGrasp Server     (端口 8030)"
