"""
Joint format conversion between Skill-DB and Igrape-bot3.

Skill-DB:  {"J1": deg, ..., "J7": deg}  — 7-DOF, degrees
Igrape:    {"21": rad, ..., "27": rad}   — motor IDs, radians
"""

import math

# Right arm mapping (single-arm mode)
J_NAME_TO_MOTOR_ID = {
    "J1": 21,  # shoulder_pitch_r
    "J2": 22,  # shoulder_roll_r
    "J3": 23,  # shoulder_yaw_r
    "J4": 24,  # elbow_pitch_r
    "J5": 25,  # elbow_yaw_r
    "J6": 26,  # wrist_pitch_r
    "J7": 27,  # wrist_roll_r
}

MOTOR_ID_TO_J_NAME = {v: k for k, v in J_NAME_TO_MOTOR_ID.items()}

J_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]


def skill_to_igrape(joint_dict_deg: dict) -> dict:
    """{"J1": 30.0, ...} -> {"21": 0.5236, ...}"""
    return {
        str(J_NAME_TO_MOTOR_ID[k]): math.radians(v)
        for k, v in joint_dict_deg.items()
        if k in J_NAME_TO_MOTOR_ID
    }


def igrape_to_skill(motor_dict_rad: dict) -> dict:
    """{"21": 0.5236, ...} -> {"J1": 30.0, ...}"""
    result = {}
    for str_id, val_rad in motor_dict_rad.items():
        j_name = MOTOR_ID_TO_J_NAME.get(int(str_id))
        if j_name:
            result[j_name] = math.degrees(float(val_rad))
    return result


def skill_traj_to_igrape(traj_deg) -> list:
    """Convert trajectory from Skill-DB format to Igrape format.

    Accepts either:
      - list of lists: [[J1_deg, J2_deg, ..., J7_deg], ...]
      - list of dicts: [{"J1": deg, ...}, ...]
    Returns:
      - list of dicts: [{"21": rad, "22": rad, ...}, ...]  (str keys)
    """
    result = []
    for wp in traj_deg:
        if isinstance(wp, dict):
            d = {}
            for j_name, motor_id in J_NAME_TO_MOTOR_ID.items():
                if j_name in wp:
                    d[str(motor_id)] = math.radians(wp[j_name])
            result.append(d)
        else:
            d = {}
            for i, j_name in enumerate(J_NAMES):
                d[str(J_NAME_TO_MOTOR_ID[j_name])] = math.radians(wp[i])
            result.append(d)
    return result


def igrape_traj_to_skill(traj_motor: list) -> list:
    """[{21: rad, ...}, ...] -> [[deg_J1, ..., deg_J7], ...]"""
    result = []
    for wp in traj_motor:
        row = []
        for j_name in J_NAMES:
            motor_id = J_NAME_TO_MOTOR_ID[j_name]
            row.append(math.degrees(wp.get(motor_id, 0.0)))
        result.append(row)
    return result


def motor_traj_to_j_rad(traj_motor: list) -> list:
    """[{21: rad, ...}, ...] -> [[rad_J1, ..., rad_J7], ...]  (keeps radians)

    Used by IgrapeTwinClient to return trajectories in the format skills expect:
    radians in J1-J7 order. Skills then convert rad→deg before sending to arm.
    """
    result = []
    for wp in traj_motor:
        row = []
        for j_name in J_NAMES:
            motor_id = J_NAME_TO_MOTOR_ID[j_name]
            row.append(wp.get(motor_id, 0.0))
        result.append(row)
    return result
