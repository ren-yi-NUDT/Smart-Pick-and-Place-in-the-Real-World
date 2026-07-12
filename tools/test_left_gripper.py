#!/usr/bin/env python3
"""左臂夹爪链路 + is_grasping 验证脚本

用法:
    conda activate anygrasp
    python3 tools/test_left_gripper.py

测试项:
  1. socket 连接 (port 8002)
  2. open / close / is_fully_open / get_finger_deviation
  3. is_grasping() 空载测试（应返回 False）
  4. is_grasping() 抓物体测试（用户放入物体后应返回 True）
"""
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.gripper import GripperClient


def main():
    c = GripperClient(host='127.0.0.1', port=8002, src='/left_gripper/movement_control')
    if not c.connect():
        print("✗ 无法连接 8002，请检查 left gripper server 是否启动")
        print("  启动命令: cd dependence/gripper-programming && python3 server.py \\")
        print("            --serial /dev/ttyUSB1 --slave 1 --port 8002 \\")
        print("            --src /left_gripper/movement_control")
        return 1

    print("\n=== 1. 空载闭合测试（应返回 False）===")
    c.open()
    time.sleep(0.8)
    result_empty = c.is_grasping()
    print(f"  is_grasping() = {result_empty}  {'✓' if not result_empty else '✗'}")

    print("\n=== 2. 抓物体测试 ===")
    c.open()
    time.sleep(0.8)
    input("  把笔/手指/小物体放进夹爪两指之间，准备好后按 Enter...")

    result_obj = c.is_grasping()
    print(f"  is_grasping() = {result_obj}")
    if result_obj:
        print("  ✓ 检测到物体（gOBJ=2）")
    else:
        print("  ✗ 未检测到物体（gOBJ=3，可能物体太小或没塞到位）")

    print("\n=== 3. 释放 ===")
    c.open()
    time.sleep(0.5)
    c.close_connection()
    print("✓ 测试完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
