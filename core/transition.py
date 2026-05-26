"""
Arm transition adjacency checker.

Defines which poses can directly transition to which other poses.
If a direct transition is not allowed, the arm goes to home first.

Config format (in robot_config.json per arm):
  "transition_adjacency": "free"                     -- all transitions allowed
  "transition_adjacency": { "pose_a": ["pose_b"] }   -- explicit allowed pairs

Implicit rules:
  - home -> anything: always allowed
  - anything -> home: always allowed
  - same pose -> same pose: always allowed
"""

def is_transition_allowed(from_pose, to_pose, adjacency):
    if adjacency == "free":
        return True
    if from_pose == to_pose or from_pose == "home" or to_pose == "home":
        return True
    return to_pose in adjacency.get(from_pose, [])


def plan_transition_sequence(poses, adjacency):
    """Insert 'home' between consecutive poses where direct transition is not allowed."""
    if adjacency == "free" or len(poses) <= 1:
        return list(poses)
    result = [poses[0]]
    for i in range(1, len(poses)):
        if not is_transition_allowed(poses[i - 1], poses[i], adjacency):
            result.append("home")
        result.append(poses[i])
    return result
