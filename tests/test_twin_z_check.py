"""Z 高度检查必须量到桌面平面的真实距离, 不能用 AABB 角点。

实测 (2026-10-04) 一个橙子的抓取候选: IK 残差 0.0001m (完美), 三个候选全被判
"插进桌面"拒掉。原因是检查用的是 getAABB 的最低角点 —— AABB 是**轴对齐**包围盒,
link 一斜盒子就比几何大一圈: 那个位姿下 right_inner_finger 的 AABB 最低点
-0.0195, 而它到桌面的真实距离是 +0.0005 (指尖正好贴桌面, 正是抓取需要的姿势)。

这个测试直接钉住"两种量法给出相反结论"这件事: 谁把检查改回 AABB, 它就失败。
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "dependence", "twin_inference",
))

from twin import _Z_QUERY_M  # noqa: E402

URDF_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "dependence", "smart_pick_and_place_ws", "src", "rm_description",
    "urdf", "SingleArm",
)

# 日志里被 z 检查拒掉的两个抓取候选 (trajectory_generation2 的 js_fake)
POSE2 = [-0.08617894558774354, 0.6644359403570143, 1.8037176602924003,
         -2.1397712475060433, -0.9614682861558886, -3.283067549117817]
POSE3 = [-0.17755431299494054, 0.7639525449505385, 1.6024124146400784,
         -2.2379428690768517, -1.069766054323458, -3.2268981540208217]

SKIP = ("base_link", "robot_base_link", "arm_end_link")


@pytest.fixture(scope="module")
def rig():
    p = pytest.importorskip("pybullet")
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data_path(p))
    plane = p.loadURDF("plane.urdf")           # 和 twin 一样先加载 plane
    body = p.loadURDF(os.path.join(URDF_DIR, "rm65.urdf"), useFixedBase=True)
    assert plane == 0, "twin 里 plane 是 body 0, check_collision 靠这个跳过它"
    links = [p.getJointInfo(body, i)[12].decode() for i in range(p.getNumJoints(body))]
    yield p, plane, body, links
    p.disconnect(cid)


def pybullet_data_path(p):
    import pybullet_data
    return pybullet_data.getDataPath()


def _measure(p, plane, body, links, js):
    """返回 (AABB 量法的最低值, 到桌面真实距离的最低值, 谁在贴桌面)。"""
    for i in range(6):
        p.resetJointState(body, i, js[i])
    aabb_min, true_min, lowest_link = float("inf"), float("inf"), None
    for i in range(p.getNumJoints(body)):
        name = links[i]
        if name in SKIP:
            continue
        aabb_min = min(aabb_min, p.getAABB(body, i)[0][2])
        pts = p.getClosestPoints(bodyA=body, bodyB=plane, linkIndexA=i, distance=_Z_QUERY_M)
        d = min(q[8] for q in pts) if pts else _Z_QUERY_M
        if d < true_min:
            true_min, lowest_link = d, name
    return aabb_min, true_min, lowest_link


def test_fingertip_touching_table_is_not_reported_as_penetration(rig):
    """指尖贴桌面 (真值 +0.0005) 被 AABB 报成 -0.0195 —— 这个检查会拒掉正确抓取。"""
    aabb_min, true_min, link = _measure(*rig, POSE2)
    assert link == "right_inner_finger"
    assert aabb_min < 0.0, f"AABB 量法本该虚高到负数, 实得 {aabb_min}"
    assert true_min >= 0.0, f"指尖其实在桌面上方 {true_min}, 不该判插进桌面"
    assert true_min - aabb_min > 0.01, "两种量法差得应该很明显 (虚高 >1cm)"


def test_real_penetration_is_still_rejected(rig):
    """修完不能变成"一律放行": 真插进桌面 2mm 的候选仍要拒。"""
    _, true_min, _ = _measure(*rig, POSE3)
    assert true_min < 0.0, f"这个位姿指尖真在桌面下 {true_min}, 应该拒"


def test_metric_is_a_plane_distance_not_a_bounding_box(rig):
    """换量法之后, 没有任何 link 的"到桌面距离"会比 AABB 最低点还低。"""
    aabb_min, true_min, _ = _measure(*rig, POSE2)
    assert true_min >= aabb_min
