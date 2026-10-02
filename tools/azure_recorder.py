#!/usr/bin/env python3
"""Azure Kinect 录像守护: ping 机械臂 IP, 通则录, 断则停。"""
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.azure_camera import AzureCamera

def _arm_ip():
    import json
    cfg = Path(__file__).resolve().parent.parent / "robot_config.json"
    return json.load(open(cfg))["arm_ip"]

ARM_IP = _arm_ip()
PING_INTERVAL = 2.0
PING_COUNT = 2
GRACE = 3  # 连续 N 次失败才算断


def arm_alive():
    try:
        r = subprocess.run(
            ["ping", "-c", str(PING_COUNT), "-W", "1", ARM_IP],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def main():
    cam = None
    fail_streak = 0
    print(f"[azure_recorder] watching {ARM_IP} ...")
    while True:
        alive = arm_alive()
        if alive and cam is None:
            print(f"[azure_recorder] arm up -> start recording")
            try:
                cam = AzureCamera(record=True)
                cam.start()
            except Exception as e:
                print(f"[azure_recorder] camera start failed: {e}")
                cam = None
        elif not alive and cam is not None:
            fail_streak += 1
            if fail_streak >= GRACE:
                print(f"[azure_recorder] arm down -> stop recording")
                try:
                    cam.stop()
                except Exception as e:
                    print(f"[azure_recorder] stop error: {e}")
                cam = None
                fail_streak = 0
        else:
            fail_streak = 0
        time.sleep(PING_INTERVAL)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
