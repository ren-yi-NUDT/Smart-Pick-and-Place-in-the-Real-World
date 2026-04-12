# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robotic pick-and-place system for a university innovation project ("基于虚实结合双重推理架构的桌面级智能机械臂平台"). It uses a dual-inference architecture combining real-world robot control with PyBullet-based simulation for safe trajectory planning.

Core capabilities:
- **Pick and Place**: Vision-driven grasp an object and place it into a container
- **Handover**: Deliver an object to a person or receive an object from a person
- **Trash/Desk Placement**: Throw items into trash or place on desk using predefined poses
- **Look Around**: Scan the workspace and analyze scene with GLM-4.5V VLM
- **Pose Recording**: Record and replay robot arm poses and action sequences

## Running the System

The system requires 3 concurrent processes started via `start.bash`:

```bash
./start.bash  # Launches all 3 scripts in separate gnome-terminals
```

Or run individually:
1. **start1.bash**: ROS bringup — builds workspace and launches ROS nodes (robot driver, camera, hand)
2. **start2.bash**: Twin inference server — PyBullet simulation for IK/trajectory (port 8020)
3. **start3.bash**: Main planner — reads stdin JSON and executes pick-and-place pipeline

**Conda Environment**: `anygrasp` (Python 3.9)

**cuDNN Library Path** (required for AnyGrasp):
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
```

**Hardware IPs**:
- Robot arm: 192.168.1.19
- Dexterous hand: 192.168.11.210

## Architecture

```
                         planner.py / fetch_from_user.py
                    (Main orchestrator: JSON -> detect -> grasp -> place)
                    ┌──────────┬──────────┬──────────────┬──────────────┐
                    │          │          │              │              │
              json_input.py  camera.py  armcontroller.py  look_around.py
              (stdin JSON)  (RealSense)  (Socket->ROS)   (GLM-4.5V scene analysis)
                    │          │          │
                    │          │          │
                    │          ▼          ▼
                    │   anygrasp_sdk   Socket Servers (8010/8000)
                    │   (grasp poses)       │
                    │          │            │
                    │          ▼            ▼
                    │   transformation.py  ROS Nodes (start1.bash)
                    │   (TF transforms)
                    │          │
                    └──────────┴──────────────────┐
                               │                  │
                               ▼                  │
                    twin_inference/twin.py        │
                    (PyBullet IK server, port 8020)
```

## JSON Input Format

The planner reads JSON commands from stdin. Object and container names are passed directly to YOLO-World (any valid class name).

**Format:**
```json
{
    "object": "orange",
    "container": "pink plate",
    "direction": "left"
}
```

- `object` (required): object to grasp, supports comma-separated classes (OR logic)
- `container` (required): container to place, or a special mode keyword
- `direction` (optional): spatial hint (not yet implemented)

**Special Container Modes:**
| Container Value | Mode | Behavior |
|---|---|---|
| `"person"` | Handover | Smooth trajectory to handover pose, open hand |
| `"trash"`, `"垃圾桶"`, `"garbage"`, `"bin"` | Trash | Move to trash pose, drop object |
| `"desk"`, `"桌子"`, `"table"` | Desk | Random selection from 3 predefined desk poses |

**Examples:**
```bash
echo '{"object": "orange", "container": "pink plate"}' | python3 planner.py
echo '{"object": "apple,fruit", "container": "bowl"}' | python3 planner.py
echo '{"object": "bottle", "container": "person"}' | python3 planner.py
echo '{"object": "wrapper", "container": "trash"}' | python3 planner.py
echo '{"container": "pink plate"}' | python3 fetch_from_user.py  # receive from user
```

## Key Files

### Core Pipeline
| File | Purpose |
|---|---|
| `planner.py` | Main pipeline: JSON input -> detect -> grasp -> place |
| `fetch_from_user.py` | Reverse pipeline: receive item from user -> place into container |
| `json_input.py` | JSON command parsing from stdin |
| `armcontroller.py` | Robot arm control via socket (port 8010) with 4-byte length prefix |
| `camera.py` | RealSense RGB-D capture (640x480, 30fps, auto white balance off) |
| `transformation.py` | ROS TF utilities for coordinate transforms |
| `utils.py` | Camera projection, coordinate transforms, 3D visualization |
| `robot_config.json` | Predefined joint positions, link names, camera intrinsics |

### Auxiliary Modules
| File | Purpose |
|---|---|
| `look_around.py` | Scan workspace from grasp positions, GLM-4.5V scene analysis |
| `capture_at_handover.py` | Move to handover-look pose, capture image, GLM-4.5V recognition |
| `arm_pose_record_and_execute.py` | Record/play robot poses and action sequences |
| `get_current_pose.py` | Read current arm joint state from hardware |

### Twin Inference System (`twin_inference/`)
| File | Purpose |
|---|---|
| `twin.py` | PyBullet socket server (port 8020), IK solving and trajectory generation |
| `robot.py` | Robot model: `ErdaijiRobot` class, `Arm`/`Hand`/`Gripper`/`Head` structs |
| `sim_world.py` | PyBullet physics simulation environment |
| `utils.py` | Transformation matrices, SLERP, visualization helpers |
| `p_utils.py` | PyBullet joint/link/collision utilities |

### ROS Workspace (`smart_pick_and_place_ws/`)
| File | Purpose |
|---|---|
| `src/rm_65_pkg/src/arm_75_bringup.py` | Robot arm ROS bringup node |
| `src/rm_65_pkg/src/mount_camera.py` | Camera mounting/calibration node |
| `src/rm_65_pkg/src/inspire_hand_bringup.py` | Inspire dexterous hand ROS node |
| `src/rm_65_pkg/src/hand_controller_modbus.py` | Hand control via Modbus protocol |
| `src/rm_description/urdf/SingleArm/` | URDF models and robot config for simulation |

## Socket Communication

| Port | Service | Protocol | Message Format |
|---|---|---|---|
| 8000 | Hand control | TCP | JSON: `{"src": "/left_hand/movement_control", "type": "set"/"get", "cmd": [...]}` |
| 8010 | Arm control | TCP | 4-byte BE length prefix + JSON: `{"srv": "/right_arm/movement_control", "cmd": [{"type": "start"}, {"type": "js", "act": {...}, "speed": N, "block": bool}, {"type": "end"}]}` |
| 8020 | Twin inference | TCP | Request: plain JSON; Response: 4-byte BE length prefix + JSON |

**Important**: Joint angles in `robot_config.json` and arm commands use **degrees**. Twin inference trajectories are returned in **radians** and converted to degrees in `planner.py` (divide by pi * 180).

## Twin Inference Service Types

| Type | Description | Response |
|---|---|---|
| `reachability_check` | Check if target pose is reachable + collision check | `is_reached`, `delta_xyz`, `delta_rpy`, `is_collided` |
| `collision_check` | Same as reachability check | Same as above |
| `IK_calculation` | Same as reachability check | Same as above |
| `trajectory_generation` | Single-target linear trajectory with collision checking | `trajectory` (rad), `trajectory_ee`, `infos` |
| `trajectory_generation2` | Multi-target linear trajectory with Z-height safety check | Same as above + `is_z_safe`, `unsafe_links` |

## Predefined Poses (robot_config.json)

| Pose Name | Purpose |
|---|---|
| `grasp1`, `grasp2`, `grasp3`, `grasp4` | Grasp observation positions (cycled during search) |
| `place1`, `place2` | Post-placement return positions |
| `handover_pose` | Position for handing object to person |
| `get_ready_to_handover_1st`, `get_ready_to_handover_2nd` | Intermediate waypoints for smooth handover trajectory |
| `throw_to_trash_pose` | Position for dropping into trash |
| `desk_pose_1/2/3` | Random desk placement positions |
| `look_over_what_in_user_hand_pose` | Camera position for viewing user's hand |

## Detection and Grasping

- **Object Detection**: YOLO-World (yolov8x-worldv2.pt) — open-vocabulary, any class name
- **Grasp Detection**: AnyGrasp SDK — generates top-50 grasp candidates, filtered by YOLO detection bounding box with 20px margin
- **Multi-class**: Comma-separated values use OR logic (e.g., `"apple,orange,fruit"`)

## Hand Gesture Presets (arm_pose_record_and_execute.py)

6-value array: `[pinky, ring, middle, index, thumb, thumb_abduct]` (0=bent, 1000=extended)

| Gesture | Values |
|---|---|
| open | `[1000, 1000, 1000, 1000, 1000, 500]` |
| close | `[0, 0, 0, 0, 0, 0]` |
| peace | `[0, 0, 1000, 1000, 0, 0]` |
| thumbs_up | `[0, 0, 0, 0, 1000, 800]` |
| grab | `[50, 50, 50, 100, 100, 0]` |

## Dependencies

- ROS Noetic (robot control, TF)
- PyBullet (physics simulation, IK)
- YOLO-World / Ultralytics (open-vocabulary object detection)
- AnyGrasp SDK (grasp pose generation, requires license)
- pyrealsense2 (Intel RealSense D435)
- CUDA/cuDNN (GPU acceleration)
- Robotic_Arm SDK (RM65 arm direct control, used in pose recorder)
- scipy, numpy, open3d, PIL, matplotlib

## Model Paths (configured in planner.py)

- YOLO-World: `/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/yolo_world/yolov8x-worldv2.pt`
- AnyGrasp: `/home/zz/ros_proj/erdaiji_ws/src/anygrasp_ros/src/anygrasp_sdk/checkpoint_detection.tar`
- URDF (twin): `smart_pick_and_place_ws/src/rm_description/urdf/SingleArm/easy_single_arm_bullet.urdf`

## Pipeline Flow (planner.py)

1. Read JSON command from stdin (`json_input.py`)
2. **Grasp Phase** — cycle through grasp1-4 positions:
   - Move to grasp approach pose
   - Capture RGB-D image
   - Run AnyGrasp for grasp candidates
   - Filter grasps using YOLO-World detection of target object (bounding box overlap with 20px margin)
   - Transform grasp poses camera->world frame
   - Generate collision-free trajectory via twin inference
   - Execute trajectory, close hand, verify grasp success (finger position delta check)
3. **Placement Phase**:
   - Person mode: smooth interpolated trajectory through 2 waypoints to handover pose
   - Trash mode: move to `throw_to_trash_pose`, open hand
   - Desk mode: randomly select from `desk_pose_1/2/3`, open hand
   - Normal mode: detect container with YOLO-World, compute 3D position from depth, generate trajectory via twin inference
4. Open hand to release, return to grasp1

## Pipeline Flow (fetch_from_user.py)

1. Read JSON (only `container` field needed)
2. Move to `handover_pose`, open hand
3. Wait for user to place item (1s initial + 3s retry)
4. Close hand, verify grasp (finger position delta)
5. Execute placement (same modes as planner.py)
