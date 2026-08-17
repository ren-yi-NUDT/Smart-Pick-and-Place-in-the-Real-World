#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# 清理上次残留的仿真进程（roscore / rosmaster / sim_server）
for pattern in 'roscore' 'rosmaster' 'sim_server.py'; do
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

# 虚拟仿真仅需 roscore + PyBullet SimServer（端口 8031），无需真实硬件 / Twin IK / AnyGrasp
# 注意：sim_server.py 依赖 ROS master（rospy.init_node），故必须先起 roscore 再等它就绪。
# Tab 2 用 `until rostopic list` 轮询等待 master 上线，避免启动竞态。
xfce4-terminal \
    --tab --title="roscore" \
    --command="bash -c 'source /opt/ros/noetic/setup.bash && roscore; exec bash'" \
    --tab --title="PyBullet SimServer (:8031)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && until rostopic list >/dev/null 2>&1; do sleep 1; done && python3 sim_server.py --port 8031; exec bash'"

echo
echo "Started xfce4-terminal window with 2 tabs:"
echo "  Tab 1: roscore              (ROS master, port 11311)"
echo "  Tab 2: PyBullet SimServer   (port 8031, GUI)"
echo
echo "仿真模式下运行 skill：SIM_MODE=1 python run_skill.py ..."
echo "（无 GUI 环境可改用 --novis：python3 sim_server.py --novis --port 8031）"
