#!/bin/bash
set -euo pipefail

# Start one hardware bridge independently from bringup.launch.
# The ROS launch file owns visualization/cameras; this script owns 8010/8011.

SIDE="${1:-}"
BRIDGE_DIR="$(cd "$(dirname "$0")/../dependence/smart_pick_and_place_ws/src/rm_65_pkg/src" && pwd)"
PYTHON_BIN="/home/zz/anaconda3/envs/anygrasp/bin/python3"

case "$SIDE" in
    left)
        BRIDGE_SCRIPT="arm_75_bringup.py"
        BRIDGE_PORT=8010
        BRIDGE_NODE_PATTERN="arm_65_bringup_"
        BRINGUP_NODE="/left_arm/robot_state_publisher"
        ;;
    right)
        BRIDGE_SCRIPT="arm_75_bringup_right.py"
        BRIDGE_PORT=8011
        BRIDGE_NODE_PATTERN="arm_65_bringup_right_"
        BRINGUP_NODE="/right_arm/robot_state_publisher"
        ;;
    *)
        echo "Usage: $0 {left|right}" >&2
        exit 2
        ;;
esac

source /opt/ros/noetic/setup.bash

echo "Waiting for ROS bringup node ${BRINGUP_NODE} ..."
for _ in $(seq 1 180); do
    if rosnode list 2>/dev/null | grep -Fxq "$BRINGUP_NODE"; then
        break
    fi
    sleep 1
done

if ! rosnode list 2>/dev/null | grep -Fxq "$BRINGUP_NODE"; then
    echo "ERROR: ROS bringup node ${BRINGUP_NODE} is not ready after 180 seconds." >&2
    echo "Check the ROS Bringup tab and its launch log." >&2
    exit 1
fi

bridge_pid="$(ss -ltnp 2>/dev/null \
    | sed -n "s/.*:${BRIDGE_PORT}[[:space:]].*pid=\([0-9][0-9]*\).*/\1/p" \
    | head -n 1)"
if [ -n "$bridge_pid" ] && [ -r "/proc/${bridge_pid}/cmdline" ]; then
    bridge_cmd="$(tr '\0' ' ' < "/proc/${bridge_pid}/cmdline")"
    if [[ "$bridge_cmd" == *"$BRIDGE_SCRIPT"* ]]; then
        if rosnode list 2>/dev/null | grep -q "$BRIDGE_NODE_PATTERN"; then
            echo "${SIDE} arm bridge is already registered in the current ROS Master (PID ${bridge_pid}); reusing it."
            exit 0
        fi

        # The same bridge script is listening, but its ROS registration belongs
        # to an old master.  Replace only this exact bridge process; never kill
        # an unrelated service that happens to use the port.
        echo "Found stale ${SIDE} arm bridge (PID ${bridge_pid}); stopping it before re-registering."
        kill "$bridge_pid" 2>/dev/null || true
        for _ in $(seq 1 10); do
            if ! kill -0 "$bridge_pid" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        if kill -0 "$bridge_pid" 2>/dev/null; then
            echo "ERROR: stale ${SIDE} arm bridge did not stop cleanly (PID ${bridge_pid})." >&2
            exit 1
        fi
    else
        echo "ERROR: TCP port ${BRIDGE_PORT} is already in use by another process." >&2
        echo "Inspect the owner with: ss -ltnp | grep :${BRIDGE_PORT}" >&2
        exit 1
    fi
fi

echo "Starting ${SIDE} arm bridge on TCP :${BRIDGE_PORT}"
cd "$BRIDGE_DIR"
exec "$PYTHON_BIN" "$BRIDGE_SCRIPT"
