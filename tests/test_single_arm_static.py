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
    assert nodes.count("inspire_hand_bringup.py") == 1
    assert nodes.count("mount_camera.py") == 1
    assert "rm65.urdf" in (ROOT / "dependence/smart_pick_and_place_ws/src/pkg_launch/launch/bringup.launch").read_text()
    assert "ROBOT_BACKEND" not in (ROOT / "core/backends/__init__.py").read_text()
    assert not (ROOT / "core/backends/igrape").exists()


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
