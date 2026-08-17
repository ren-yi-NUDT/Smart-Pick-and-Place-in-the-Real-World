import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.sim_utils import (
    deg2rad_list, rad2deg_list, projection_bounds,
    depth_buffer_to_mm, map_gripper_value, parse_mimic_joints,
)


def test_deg2rad_list():
    out = deg2rad_list([0.0, 180.0, 90.0])
    assert np.allclose(out, [0.0, np.pi, np.pi / 2], atol=1e-9)


def test_rad2deg_list():
    out = rad2deg_list([0.0, np.pi, np.pi / 2])
    assert np.allclose(out, [0.0, 180.0, 90.0], atol=1e-6)


def test_projection_bounds_left_camera():
    fx = fy = 392.268
    cx, cy = 325.468, 242.282
    left, right, bottom, top = projection_bounds(fx, fy, cx, cy, 640, 480, near=0.01)
    assert left < 0 < right
    assert bottom < 0 < top
    assert np.isclose(left, -cx * 0.01 / fx)
    assert np.isclose(top, (480 - cy) * 0.01 / fy)


def test_depth_buffer_to_mm_roundtrip():
    near, far = 0.01, 3.0
    buf = np.array([0.0, 0.5, 1.0], dtype=np.float32)  # 0=near, 1=far
    mm = depth_buffer_to_mm(buf, near, far)
    assert mm.dtype == np.uint16
    assert mm[0] == 10  # near=0.01m -> 10mm
    assert mm[2] == 3000  # far=3m -> 3000mm


def test_map_gripper_value():
    assert map_gripper_value(0, close=0.0, open=0.8) == 0.0
    assert np.isclose(map_gripper_value(1000, close=0.0, open=0.8), 0.8)
    assert np.isclose(map_gripper_value(500, close=0.0, open=0.8), 0.4)


def test_parse_mimic_joints():
    urdf = """<?xml version="1.0"?>
<robot name="t">
  <joint name="L_finger_joint" type="revolute"><parent link="b"/><child link="f"/></joint>
  <joint name="L_left_inner_knuckle_joint" type="revolute">
    <parent link="b"/><child link="k"/>
    <mimic joint="L_finger_joint" multiplier="1" offset="0"/>
  </joint>
  <joint name="L_left_inner_finger_joint" type="revolute">
    <parent link="k"/><child link="f2"/>
    <mimic joint="L_finger_joint" multiplier="-1" offset="0"/>
  </joint>
  <joint name="L_right_outer_knuckle_joint" type="revolute">
    <parent link="b"/><child link="rk"/>
    <mimic joint="L_finger_joint" offset="0"/>
  </joint>
</robot>"""
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(urdf)
        path = f.name
    mapping = parse_mimic_joints(path)
    os.unlink(path)
    assert mapping["L_left_inner_knuckle_joint"] == ("L_finger_joint", 1.0)
    assert mapping["L_left_inner_finger_joint"] == ("L_finger_joint", -1.0)
    assert mapping["L_right_outer_knuckle_joint"] == ("L_finger_joint", 1.0)
    assert "L_finger_joint" not in mapping


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"ALL {len(fns)} TESTS PASSED")
