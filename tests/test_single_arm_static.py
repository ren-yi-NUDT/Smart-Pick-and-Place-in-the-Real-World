import ast
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_single_arm_config_and_startup():
    config = json.loads((ROOT / "robot_config.json").read_text())
    assert config["arm_model"] == "RM65"
    assert config["arm_ip"] == "192.168.1.18"
    assert config["arm_port"] == 8080
    assert config["arm_dof"] == 6
    assert config["rm65_poses_calibrated"] is True

    sim_config = json.loads((ROOT / "dependence/smart_pick_and_place_ws/src/rm_description/urdf/SingleArm/robot_config.json").read_text())
    assert list(sim_config) == ["rm65"]
    assert len(sim_config["rm65"]["joint_names"]) == 6
    assert "base_link" not in sim_config["rm65"]["link_names"]

    launch = ET.parse(ROOT / "dependence/smart_pick_and_place_ws/src/pkg_launch/launch/bringup.launch")
    nodes = [node.attrib["type"] for node in launch.findall("node")]
    assert nodes.count("arm_75_bringup.py") == 1
    assert nodes.count("mount_camera.py") == 1
    assert "rm65.urdf" in (ROOT / "dependence/smart_pick_and_place_ws/src/pkg_launch/launch/bringup.launch").read_text()
    assert "ROBOT_BACKEND" not in (ROOT / "core/backends/__init__.py").read_text()
    assert not (ROOT / "core/backends/igrape").exists()


RM65_JOINT_LIMITS_DEG = [(-178, 178), (-130, 130), (-135, 135), (-178, 178), (-128, 128), (-360, 360)]

TOP_LEVEL_POSES = [
    "get_ready_to_handover_1st", "get_ready_to_handover_2nd", "handover_pose",
    "throw_to_trash_pose", "look_over_what_in_user_hand_pose",
    "desk_pose_1", "desk_pose_2", "desk_pose_3",
]


def test_config_poses_within_rm65_limits():
    """所有预定义位姿必须在真机行程内, 否则 rm_movej 被控制器拒绝 (tag 1), 臂不动。"""
    config = json.loads((ROOT / "robot_config.json").read_text())
    poses = dict(config["default_traj_js"])
    poses.update({k: config[k] for k in TOP_LEVEL_POSES if k in config})

    offenders = {}
    for name, pose in poses.items():
        if "J1" not in pose:
            continue
        bad = [f"J{i+1}={pose[f'J{i+1}']:.1f} (限 {lo}..{hi})"
               for i, (lo, hi) in enumerate(RM65_JOINT_LIMITS_DEG)
               if not lo <= pose[f"J{i+1}"] <= hi]
        if bad:
            offenders[name] = bad

    assert not offenders, f"位姿超出 RM65 关节行程: {offenders}"


def test_recorded_poses_are_rm65_six_joint():
    poses = json.loads((ROOT / "recorded_poses.json").read_text())
    assert all(len(pose["joint_angles_deg"]) == 6 for pose in poses.values())
    import sys
    sys.path.insert(0, str(ROOT))
    from core.arm import ArmClient

    arm = ArmClient()
    arm.start_cmd()
    try:
        arm.add_js_cmd({f"J{i}": 0 for i in range(1, 8)})
    except ValueError:
        pass
    else:
        raise AssertionError("seven-joint command was accepted")
    assert len(arm._cmds) == 1


def test_teach_record_routing_and_limits(tmp_path, monkeypatch):
    """teach_record 必须把 grasp* 写进 default_traj_js, 其余写顶层, 并报出超限关节。"""
    import sys
    sys.path.insert(0, str(ROOT))
    from tools import teach_record

    monkeypatch.setattr(teach_record, "CONFIG_PATH", str(tmp_path / "robot_config.json"))
    config = {"default_traj_js": {}, "arm_model": "RM65"}
    (tmp_path / "robot_config.json").write_text(json.dumps(config))

    teach_record.write_pose(config, "grasp1", [1.0, 2.0, 3.0, 4.0, -100.0, 6.0])
    teach_record.write_pose(config, "desk_pose_1", [1.0, 2.0, 3.0, 4.0, -100.0, 6.0])

    written = json.loads((tmp_path / "robot_config.json").read_text())
    assert written["default_traj_js"]["grasp1"]["J5"] == -100.0
    assert "grasp1" not in written and written["desk_pose_1"]["J6"] == 6.0

    lo, hi = [-178, -130, -135, -178, -128, -360], [178, 130, 135, 178, 128, 360]
    assert teach_record.check_limits([0, 0, 0, 0, -100.0, 0], lo, hi) == []
    assert teach_record.check_limits([0, 0, 0, 0, 172.3, 0], lo, hi) == ["J5=172.3 (限 -128..128)"]


def test_yolo_confidence_is_not_hardcoded_in_skill_paths():
    """YOLO 阈值只能来自 core/config.py。

    这些数字散落在各 skill 里时, 抓取侧被改到 0.05 而放置侧还是 0.25 没人发现 ——
    0.25 正好卡在实测盘子分数 (0.096~0.29) 的中间, 表现成"放置时灵时不灵"。
    """
    from core.config import YOLO_CONF_GRASP, YOLO_CONF_CONTAINER
    assert 0 < YOLO_CONF_GRASP <= YOLO_CONF_CONTAINER < 1

    for rel in ["skills/pick_and_place.py", "skills/grasp.py",
                "skills/place.py", "skills/fetch_from_user.py"]:
        tree = ast.parse((ROOT / rel).read_text())
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if getattr(node.func, "attr", None) not in ("detect_objects", "detect_container"):
                continue
            confs = [kw.value for kw in node.keywords if kw.arg == "conf"]
            assert confs, f"{rel}:{node.lineno} detect_objects 没传 conf"
            for v in confs:
                assert isinstance(v, ast.Name), (
                    f"{rel}:{node.lineno} 硬编码了 YOLO 阈值, 应改用 core.config 的常量")
