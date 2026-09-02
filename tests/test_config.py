import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.config import Config


def test_canonical_robot_config_contains_all_robot_models():
    config = Config()

    assert set(config.get_robot_model("left_gripper")) == {"left_arm"}
    assert set(config.get_robot_model("right_gripper")) == {"right_arm"}
    assert set(config.get_robot_model("dual_arm")) == {"left_arm", "right_arm"}


def test_robot_model_is_returned_as_a_copy():
    config = Config()
    model = config.get_robot_model("left_gripper")
    model["left_arm"]["joint_names"].clear()

    assert len(config.get_robot_model("left_gripper")["left_arm"]["joint_names"]) == 7


def test_canonical_robot_config_is_valid_json():
    with open(config_path(), encoding="utf-8") as config_file:
        data = json.load(config_file)

    assert "arms" in data
    assert "shared" in data
    assert "robot_models" in data


def config_path():
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "robot_config.json",
    )
