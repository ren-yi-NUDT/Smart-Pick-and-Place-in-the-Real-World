# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robotic pick-and-place system controlled via JSON input. The system integrates:
- **JSON Input**: stdin-based command input for specifying grasp/place tasks
- **Object Detection**: YOLO-World for detecting graspable objects and containers
- **Grasp Detection**: AnyGrasp SDK for generating grasp poses
- **Motion Planning**: PyBullet-based twin simulation for IK solving and trajectory generation
- **Robot Control**: ROS-based control of a 7-DOF arm and Inspire dexterous hand
- **Vision**: Intel RealSense camera for RGB-D capture

## Running the System

The system requires 3 concurrent processes started via `start.bash`:

```bash
./start.bash  # Launches all 3 scripts in separate terminals
```

Or run individually:
1. **start1.bash**: ROS bringup - builds and launches ROS nodes (robot, camera, hand)
2. **start2.bash**: Twin inference server - PyBullet simulation for IK/trajectory (port 8020)
3. **start3.bash**: Main planner - the core pick-and-place pipeline

**Conda Environment**: `anygrasp` (Python 3.9)

**cuDNN Library Path** (required for AnyGrasp):
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
```

**Hardware IPs**:
- Robot arm: 192.168.1.19
- Dexterous hand: 192.168.11.210

## JSON Input Format

The planner reads JSON commands from stdin. The system accepts any YOLO-World compatible class names.

**Format:**
```json
{
    "object": "orange",        // Required: object to grasp (supports comma-separated classes)
    "container": "pink plate", // Required: container to place (or special mode, see below)
    "direction": "left"        // Optional: spatial hint (not yet implemented)
}
```

**Special Container Modes:**
- `"person"` → Handover mode: moves to predefined handover pose and releases object
- `"trash"`, `"垃圾桶"`, `"garbage"`, `"bin"` → Trash mode: moves to trash pose and drops object
- `"desk"`, `"桌子"`, `"table"` → Desk mode: randomly places on one of 3 predefined desk poses
- Any other value → Normal placement: uses vision to detect container position

**Examples:**
```bash
# Basic usage
echo '{"object": "orange", "container": "pink plate"}' | python3 planner.py

# Multi-class detection (detects apple OR fruit)
echo '{"object": "apple,fruit", "container": "bowl"}' | python3 planner.py

# Handover to person
echo '{"object": "bottle", "container": "person"}' | python3 planner.py

# Throw in trash
echo '{"object": "wrapper", "container": "trash"}' | python3 planner.py
```

**Notes:**
- `object` and `container` values are passed directly to YOLO-World
- Use class names that YOLO-World can recognize for best results
- No predefined vocabulary - any valid class name is accepted

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         planner.py                              │
│  (Main orchestrator: JSON → detect → grasp → place)             │
└─────────────────────────────────────────────────────────────────┘
         │              │              │              │
         ▼              ▼              ▼              ▼
   json_input.py       camera.py    armcontroller.py
   (stdin JSON)       (RealSense)    (Socket→ROS)
         │              │              │
         │              │              ▼
         │              │     Socket Servers (8010/8000)
         │              │              │
         │              ▼              ▼
         │     anygrasp_sdk     transformation.py   ROS Nodes
         │     (grasp poses)    (TF transforms)    (start1.bash)
         │              │              │
         └──────────────┴──────────────┘
                        │
                        ▼
              twin_inference/twin.py
              (PyBullet IK solver, port 8020)
```

## Pipeline Flow

1. **Input**: Read JSON command from stdin
2. **Grasp Phase**:
   - Move to grasp approach pose (grasp1/2/3)
   - Capture RGB-D image
   - Run AnyGrasp to generate grasp candidates
   - Filter grasps using YOLO-World detection of target object
   - Transform grasp poses to world frame
   - Generate collision-free trajectory via twin inference
   - Execute grasp trajectory and close hand
3. **Placement Phase**:
   - If special container (person/trash/desk): use predefined poses
   - Otherwise: detect container with YOLO-World, compute placement position
   - Generate and execute placement trajectory
   - Open hand to release object

## Key Files

| File | Purpose |
|------|---------|
| `planner.py` | Main pipeline: JSON input → grasp → place |
| `json_input.py` | JSON command parsing from stdin |
| `armcontroller.py` | Robot arm control via socket (port 8010) |
| `camera.py` | RealSense RGB-D capture |
| `transformation.py` | ROS TF utilities for coordinate transforms |
| `robot_config.json` | Default joint positions, link names |
| `twin_inference/twin.py` | PyBullet-based IK solver and trajectory generator |

## Socket Communication

The system uses socket-based IPC:
- **Port 8000**: Hand control (open/close gripper)
- **Port 8010**: Arm control (joint space commands)
- **Port 8020**: Twin inference (IK, trajectory generation)

**Message Formats:**

Hand/Arm control (ports 8000/8010):
```json
{"src": "/left_hand/movement_control", "type": "set", "cmd": [0, 0, 0, 460, 0, 0]}
{"src": "/right_arm/movement_control", "cmd": [{"type": "start"}, {"type": "js", "act": {...}, "speed": 20}, {"type": "end"}]}
```

Twin inference (port 8020):
- Request: plain JSON
- Response: 4-byte big-endian length prefix + JSON data

**Joint Angles**: All joint angles in trajectories use **degrees**, not radians.

## Supported Service Types (Twin Inference)

- `reachability_check`: Check if pose is reachable
- `collision_check`: Check for collisions
- `IK_calculation`: Inverse kinematics
- `trajectory_generation`: Single-target linear trajectory
- `trajectory_generation2`: Multi-target linear trajectory

## Predefined Poses (robot_config.json)

| Pose Name | Purpose |
|-----------|---------|
| `grasp1`, `grasp2`, `grasp3` | Grasp approach positions (cycled during search) |
| `place1`, `place2` | Post-placement return positions |
| `handover_pose` | Position for handing object to person |
| `throw_to_trash_pose` | Position for dropping into trash |
| `desk_pose_1/2/3` | Random desk placement positions |

## Detection Classes

The system uses YOLO-World for open-vocabulary object detection. Any class name that YOLO-World can recognize is supported.

**Common examples:**
- Objects: orange, apple, lemon, pear, bottle, cup, mug, banana, carrot, etc.
- Containers: bowl, plate, box, basket, tray, cup, etc.

**Multi-class syntax:** Use comma to specify multiple classes (OR logic):
- `"object": "apple,orange,fruit"` → detects any of these
- `"container": "bowl,plate"` → detects either bowl or plate

## Dependencies

- ROS (tested with Melodic/Noetic)
- PyBullet (for twin simulation)
- YOLO-World (object detection)
- AnyGrasp SDK (grasp detection, requires license)
- pyrealsense2 (Intel RealSense)
- CUDA/cuDNN (GPU acceleration)

## Model Paths (configured in planner.py)

- YOLO-World: `/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/yolo_world/yolov8x-worldv2.pt`
- AnyGrasp: `/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/anygrasp_sdk/checkpoint_detection.tar`
