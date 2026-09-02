#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# 清理上次残留的仿真进程
for pattern in 'roscore' 'rosmaster' 'sim_server.py' 'twin.py' 'anygrasp_server.py'; do
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

# 虚拟仿真完整流水线：roscore + PyBullet SimServer(:8031) +
# Twin IK left(:8032) + Twin IK right(:8033) + AnyGrasp(:8030)
# sim_server.py / twin.py 依赖 ROS master（rospy.init_node），故先起 roscore 再等它就绪。
# 仿真 twin 用 8032 避让真机 twin 的 8020/8021，二者可同时运行不冲突。
# 各 Tab 用 `until rostopic list` 轮询等待 master 上线，避免启动竞态。
xfce4-terminal \
    --tab --title="roscore" \
    --command="bash -c 'source /opt/ros/noetic/setup.bash && roscore; exec bash'" \
    --tab --title="PyBullet SimServer (:8031)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && until rostopic list >/dev/null 2>&1; do sleep 1; done && python3 sim_server.py --port 8031 ${SIM_SCENE:+--scene "$SIM_SCENE"}; exec bash'" \
    --tab --title="Twin IK left (:8032)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && until rostopic list >/dev/null 2>&1; do sleep 1; done && python3 twin.py --side left --port 8032 --novis; exec bash'" \
    --tab --title="Twin IK right (:8033)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && until rostopic list >/dev/null 2>&1; do sleep 1; done && python3 twin.py --side right --port 8033 --novis; exec bash'" \
    --tab --title="AnyGrasp (:8030)" \
    --command="bash -ic 'conda activate anygrasp && export PATH=$PATH:/sbin:/usr/sbin && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && python3 $PROJECT_ROOT/dependence/anygrasp_server/anygrasp_server.py --port 8030; exec bash'"

echo
echo "Started xfce4-terminal window with 5 tabs:"
echo "  Tab 1: roscore              (ROS master, port 11311)"
echo "  Tab 2: PyBullet SimServer   (port 8031, GUI)"
echo "  Tab 3: Twin IK left         (port 8032, headless)"
echo "  Tab 4: Twin IK right        (port 8033, headless)"
echo "  Tab 5: AnyGrasp              (port 8030)"
echo
echo "仿真模式下运行 skill：SIM_MODE=1 python run_skill.py ..."
echo "（无 GUI 环境可改用 --novis：python3 sim_server.py --novis --port 8031）"
