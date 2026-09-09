#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

# Use stable USB serial identities instead of ttyUSB numbers, which can change
# when the adapters are moved between physical USB ports.
RIGHT_GRIPPER_SERIAL="/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E613N-if00-port0"
LEFT_GRIPPER_SERIAL="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00T5A5-if00-port0"

for serial_dev in "$RIGHT_GRIPPER_SERIAL" "$LEFT_GRIPPER_SERIAL"; do
    if [ ! -e "$serial_dev" ]; then
        echo "ERROR: gripper serial device not found: $serial_dev" >&2
        echo "       Check /dev/serial/by-id/ and reconnect the USB adapters." >&2
        exit 1
    fi
done

# 清理本项目上次残留的进程。
# 不能只匹配 "python3 server.py"：手动启动时常见的
# ".../python3 -u server.py" 会漏掉，导致第二个实例绑定端口失败。
# 这里同时校验进程 cwd，只清理本项目对应目录中的进程，避免误杀其他项目。
kill_stale_script() {
    local script_name="$1"
    local work_dir="$2"
    local pid cwd

    while read -r pid; do
        [ -z "$pid" ] && continue
        [ "$pid" = "$$" ] && continue
        cwd=$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)
        if [ "$cwd" = "$work_dir" ]; then
            echo "  Killing stale $script_name (pid $pid, cwd $cwd)"
            kill "$pid" 2>/dev/null || true
        fi
    done < <(pgrep -f "$script_name" || true)
}

kill_stale_script 'server.py' "$PROJECT_ROOT/dependence/gripper-programming"
kill_stale_script 'anygrasp_server.py' "$PROJECT_ROOT/dependence/anygrasp_server"
kill_stale_script 'twin.py' "$PROJECT_ROOT/dependence/twin_inference"
BRIDGE_DIR="$PROJECT_ROOT/dependence/smart_pick_and_place_ws/src/rm_65_pkg/src"
kill_stale_script 'arm_75_bringup.py' "$BRIDGE_DIR"
kill_stale_script 'arm_75_bringup_right.py' "$BRIDGE_DIR"

# 等待被清理的监听 socket 释放；若端口仍被其他程序占用，明确报错而不是
# 打开一组必然启动失败的终端标签。
for port in 8001 8002 8010 8011 8020 8021 8030; do
    for _ in $(seq 1 20); do
        if ! ss -ltn | grep -q ":$port "; then
            break
        fi
        sleep 0.25
    done
    if ss -ltn | grep -q ":$port "; then
        echo "ERROR: port $port is still occupied by a process outside this project." >&2
        ss -ltnp | grep ":$port " >&2 || true
        exit 1
    fi
done

# 检查 xfce4-terminal
if ! command -v xfce4-terminal >/dev/null 2>&1; then
    echo "ERROR: xfce4-terminal not installed." >&2
    echo "       Install with: sudo apt install xfce4-terminal" >&2
    exit 1
fi

# xfce4-terminal 单次调用多 tab（不像 gnome-terminal 3.36 那样 -- 之后吞掉所有 --tab 参数）
# 每个 --tab 引入一个 tab spec，--working-directory / --command 都作用于"最后一个 --tab"
#
# Gripper 串口分配（按 USB 设备序列号绑定）：
#   右臂夹爪 → FT231X ...DU0E613N, slave=9 (端口 8001)
#   左臂夹爪 → FT232R ...BG00T5A5, slave=1 (端口 8002)
# 注意：两条臂的 Robotiq 85 Modbus 地址不同（出厂或现场配置导致），不要互换。
xfce4-terminal \
    --tab --title="ROS Bringup" \
    --working-directory="$PROJECT_ROOT/dependence/smart_pick_and_place_ws" \
    --command="bash -c 'catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && source devel/setup.bash && roslaunch pkg_launch bringup.launch; exec bash'" \
    --tab --title="Arm bridge L (:8010)" \
    --working-directory="$PROJECT_ROOT" \
    --command="bash -ic 'tools/start_arm_bridge.sh left; exec bash'" \
    --tab --title="Arm bridge R (:8011)" \
    --working-directory="$PROJECT_ROOT" \
    --command="bash -ic 'tools/start_arm_bridge.sh right; exec bash'" \
    --tab --title="Twin IK (left)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && python3 twin.py --side left; exec bash'" \
    --tab --title="Twin IK (right)" \
    --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
    --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && python3 twin.py --side right; exec bash'" \
    --tab --title="Gripper R (:8001)" \
    --working-directory="$PROJECT_ROOT/dependence/gripper-programming" \
    --command="bash -ic 'source /home/zz/anaconda3/etc/profile.d/conda.sh && conda activate anygrasp && python3 server.py --serial $RIGHT_GRIPPER_SERIAL --slave 9 --port 8001 --src /right_gripper/movement_control; exec bash'" \
    --tab --title="Gripper L (:8002)" \
    --working-directory="$PROJECT_ROOT/dependence/gripper-programming" \
    --command="bash -ic 'source /home/zz/anaconda3/etc/profile.d/conda.sh && conda activate anygrasp && python3 server.py --serial $LEFT_GRIPPER_SERIAL --slave 1 --port 8002 --src /left_gripper/movement_control; exec bash'" \
    --tab --title="AnyGrasp Server (:8030)" \
    --working-directory="$PROJECT_ROOT/dependence/anygrasp_server" \
    --command="bash -ic 'source /home/zz/anaconda3/etc/profile.d/conda.sh && conda activate anygrasp && export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && python3 anygrasp_server.py; exec bash'"

echo
echo "Started xfce4-terminal window with 8 tabs:"
echo "  Tab 1: ROS Bringup         (双臂可视化与相机)"
echo "  Tab 2: Arm bridge left     (端口 8010；等待 ROS Master 后启动)"
echo "  Tab 3: Arm bridge right    (端口 8011；等待 ROS Master 后启动)"
echo "  Tab 4: Twin IK left        (端口 8020)"
echo "  Tab 5: Twin IK right       (端口 8021)"
echo "  Tab 6: Gripper Right       (端口 8001, FT231X ...DU0E613N, slave 9)"
echo "  Tab 7: Gripper Left        (端口 8002, FT232R ...BG00T5A5, slave 1)"
echo "  Tab 8: AnyGrasp Server     (端口 8030)"
echo
echo "若 gripper server 启动失败，请检查稳定设备路径：ls -l /dev/serial/by-id/"
