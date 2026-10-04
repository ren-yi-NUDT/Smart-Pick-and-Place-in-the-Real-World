#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hold() 必须把夹爪停在探到的开度上, 不能合到机械限位。

背景 (2026-10-04 实测): soft close 探到橙子后停在 pos=221/255 (开度 74mm), 紧接着的
close(force=40) 又把它推到 255 —— 那不是夹紧, 是往里挤, 果子被挤出去。现场看到的是
"夹爪一端先碰到果子, 然后往里收, 结果没夹起来"。

hold() 的做法是把 get_state 读到的客户端值原样发回去, 只加力。这条依赖
pos_to_client / client_to_pos 互为逆运算: 发回去的开度必须还是原来那个。
"""

import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "dependence", "gripper-programming"))

# server.py 在模块级 import pymodbus (真机 Modbus 驱动), 但下面这两个映射函数是纯函数,
# 不需要串口。塞个空壳把模块导进来就行 —— 跑测试的 base 环境没装 pymodbus (真机服务
# 跑在 anygrasp 环境里)。
for _n in ("pymodbus", "pymodbus.client", "pymodbus.client.sync", "pymodbus.exceptions"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
sys.modules["pymodbus.client.sync"].ModbusSerialClient = object
sys.modules["pymodbus.exceptions"].ModbusIOException = Exception

from server import client_to_pos, pos_to_client      # noqa: E402


def test_client_pos_round_trip_is_identity():
    """hold() 把读到的客户端值发回去, 开度不能漂。"""
    for pos in range(256):
        assert client_to_pos(pos_to_client(pos)) == pos, f"pos={pos} 漂了"


def test_close_is_the_mechanical_limit():
    """对照: close() 发的是 [0,0] -> pos 255, 这就是"挤到底"的来源。"""
    assert client_to_pos([0, 0]) == 255
    assert client_to_pos([1000, 1000]) == 0
