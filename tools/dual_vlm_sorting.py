#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VLM-planned dual-arm sorting with resource and collision interlocks.

The VLM only returns a semantic scene inventory (objects, groups, and
destination containers).  It never returns robot poses or motion commands.
Depth + calibrated camera transforms provide geometry; AnyGrasp and Twin
provide grasp/reachability plans; the executor applies conservative safety
rules before commanding either real or simulated hardware.

Default workflow:
  1. Move the arms one at a time to configured observation poses.
  2. Capture and save both RGB-D observations.
  3. Ask the configured VLM to inventory objects and containers.
  4. Convert boxes + depth to calibrated 3-D points and deduplicate views.
  5. Build semantic arm queues from the VLM; grasp geometry is deferred to
     execution so every pick uses that arm's own fresh camera.
  6. Execute with a home-synchronized pipeline: one arm runs the single-arm
     visual grasp and returns home, the other arm then does the same, and
     each arm places using the successful runtime grasp pose.

Safety defaults intentionally require explicit ``--execute`` and, for real
hardware, ``--real-confirm``.  A plan is saved before execution.  On a
post-grasp failure the run stops and the abort cleanup opens connected
grippers, as required by the operator's termination policy.

Examples:
  SIM_MODE=1 python -m tools.dual_vlm_sorting --plan-only
  SIM_MODE=1 python -m tools.dual_vlm_sorting --execute --yes
  python -m tools.dual_vlm_sorting --execute --real-confirm
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation as R
from termcolor import cprint

from core.transforms import pixel_to_camera_point2
from skills.grasp import GraspSkill


PROJECT_ROOT = Path(__file__).resolve().parents[1]
# The deployed sorting task is fruit/vegetable separation: fruit goes to the
# pink plate and vegetables go to the blue plate.  Keep the generic colour
# grouping config available through --task-config when needed.
DEFAULT_TASK_CONFIG = PROJECT_ROOT / "configs" / "dual_vlm_sorting_real_fruit_vegetable.json"


class SortingError(RuntimeError):
    """Expected planning or execution failure."""


def _finite_box(box, width, height, format_hint="normalized_1000"):
    """Convert a VLM box to pixel coordinates.

    The scene-inventory prompt requires 0..1000 coordinates.  Do not infer
    that format from whether a coordinate happens to be below the image
    width/height: e.g. ``[514, 362, 554, 420]`` is a valid normalized box
    whose y coordinates are still below a 480-pixel image height.  An
    explicit pixel hint remains supported for adapters using pixel boxes.
    """
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return None
    try:
        values = [float(x) for x in box]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(x) for x in values):
        return None
    x1, y1, x2, y2 = values
    if format_hint == "pixels":
        pass
    elif max(values) <= 1.0:
        x1, x2 = x1 * width, x2 * width
        y1, y2 = y1 * height, y2 * height
    elif max(values) <= 1000.0:
        x1, x2 = x1 * width / 1000.0, x2 * width / 1000.0
        y1, y2 = y1 * height / 1000.0, y2 * height / 1000.0
    x1 = max(0.0, min(float(width), x1))
    x2 = max(0.0, min(float(width), x2))
    y1 = max(0.0, min(float(height), y1))
    y2 = max(0.0, min(float(height), y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def _box_center(box):
    return (0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3]))


def _label_key(label):
    """Normalize a VLM label for conservative cross-view matching."""
    value = str(label or "").strip().lower()
    for char in (" ", "\t", "\n", "-", "_", "/", "／", "、", ",", "，"):
        value = value.replace(char, "")
    return value


def _labels_overlap(first, second):
    """Return whether two VLM labels share a stable semantic token."""
    first_key = _label_key(first)
    second_key = _label_key(second)
    return bool(
        first_key
        and second_key
        and (
            first_key == second_key
            or first_key in second_key
            or second_key in first_key
        )
    )


def _colors_overlap(first, second):
    """Match simple VLM color descriptors across the two cameras."""
    first_key = _label_key(first)
    second_key = _label_key(second)
    return bool(
        first_key
        and second_key
        and (
            first_key == second_key
            or first_key in second_key
            or second_key in first_key
        )
    )


def _semantic_view_weight(label):
    """Down-weight explicitly uncertain VLM labels during group voting."""
    text = str(label or "")
    uncertain_markers = ("或", "疑似", "可能", "不确定", "部分", "看不清")
    return 0.35 if any(marker in text for marker in uncertain_markers) else 1.0


def _depth_for_box(depth, box, min_depth_m, max_depth_m):
    """Robustly estimate the nearest valid depth in the inner box region."""
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = box
    # Avoid platform edges and neighboring objects.
    ix1 = int(max(0, x1 + 0.25 * (x2 - x1)))
    ix2 = int(min(width, x2 - 0.25 * (x2 - x1)))
    iy1 = int(max(0, y1 + 0.25 * (y2 - y1)))
    iy2 = int(min(height, y2 - 0.25 * (y2 - y1)))
    crop = depth[iy1:iy2, ix1:ix2]
    valid = crop[(crop > 0)]
    if not len(valid):
        return None
    # RealSense and SimCamera both expose uint16 millimetres.
    values_m = valid.astype(np.float64) * 1e-3
    values_m = values_m[(values_m >= min_depth_m) & (values_m <= max_depth_m)]
    if not len(values_m):
        return None
    # The nearer quartile is more likely to be the object surface than the
    # supporting platform when a VLM box contains a little background.
    return float(np.percentile(values_m, 25.0))




def _as_point4(point):
    return np.array([float(point[0]), float(point[1]), float(point[2]), 1.0])


def _pose7(matrix):
    p = np.asarray(matrix[:3, 3], dtype=float)
    q = R.from_matrix(np.asarray(matrix[:3, :3], dtype=float)).as_quat()
    return [float(x) for x in (*p, *q)]


@dataclass
class Observation:
    side: str
    rgb: np.ndarray
    depth: np.ndarray
    T_base_to_cam: np.ndarray
    objects: list[dict] = field(default_factory=list)
    destinations: list[dict] = field(default_factory=list)
    source_surfaces: list[dict] = field(default_factory=list)
    raw_grasps: list[dict] = field(default_factory=list)
    # A unique id is required once one camera contributes several views.
    # Keeping it on the observation prevents boxes from one image being used
    # with the RGB-D data of another image during grasp generation.
    view_id: str = ""
    pose_name: str = ""
    capture_timestamp: str = ""
    capture_monotonic_s: float = 0.0


@dataclass
class SceneObject:
    object_id: str
    label: str
    group: str
    point_left: np.ndarray
    confidence: float
    views: list[dict] = field(default_factory=list)
    candidates: dict = field(default_factory=dict)
    source_anchor_left: Optional[np.ndarray] = None
    source_resource_id: Optional[str] = None


@dataclass
class Destination:
    destination_id: str
    role: str
    label: str
    point_left: np.ndarray
    confidence: float
    views: list[dict] = field(default_factory=list)


def _load_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


class VLMScenePlanner:
    """VLM adapter.  It intentionally has no arm-control dependency."""

    def __init__(self, helper: GraspSkill, task_config: dict):
        self.helper = helper
        self.task_config = task_config

    def _prompt(self, side, width, height):
        groups = self.task_config.get("groups", {})
        destinations = self.task_config.get("destinations", {})
        return (
            "你是双臂机器人分拣任务的场景清单模块。只做视觉识别和语义分类，"
            "不要输出机器人位姿、关节角、抓取点或运动指令。\n"
            f"当前相机：{side}；图像尺寸：{width}x{height}。\n"
            f"允许的物体分组及规则：{json.dumps(groups, ensure_ascii=False)}\n"
            f"允许的目标容器角色：{json.dumps(destinations, ensure_ascii=False)}\n\n"
            "分组描述中的示例仅用于解释类别，不代表现场一定存在这些物体；"
            "必须依据当前图像外观判断，不能为了匹配示例而猜测具体名称。\n"
            "请识别图中所有需要搬运的物体，以及所有可能的目标平台/盒子/容器。"
            "重要：平台、盒子、容器、桌面和背景板不是待搬运物体，绝对不要放入 objects；"
            "只有能被夹爪拿起并搬运的独立小物体才放入 objects。"
            "每个物体给出唯一临时 id、自然语言 label、group（必须是允许的分组 key）、"
            "颜色或分类依据、0到1置信度和一个只包住该物体的框。"
            "即使多个物体彼此相邻或接触，也必须按独立物体逐个列出，"
            "绝对不要合并成‘球组’、‘彩色球’或一个大框；每个球只能有一个框。"
            "优先依据可见颜色完成分组，不要因为物体较小就写 unknown。"
            "每个容器给出 role（必须是允许的目标角色 key）、label、置信度和框。"
            "同时识别承载待搬运物体的桌面、平台、盒子或容器，给出 source_surfaces "
            "列表；物体如能判断来源，在 source_surface_id 中引用对应 id。"
            "看不清或无法分类的物体不要猜，group 写 unknown。\n"
            "只返回 JSON，不要 Markdown："
            '{"objects":[{"id":"obj_1","label":"...","group":"cool",'
            '"color":"...","confidence":0.9,"source_surface_id":"src_1",'
            '"box":[x1,y1,x2,y2]}],'
            '"destinations":[{"id":"dst_1","role":"cool_destination",'
            '"label":"...","confidence":0.9,"box":[x1,y1,x2,y2]}],'
            '"source_surfaces":[{"id":"src_1","label":"...",'
            '"confidence":0.9,"box":[x1,y1,x2,y2]}]}。'
            "坐标 box 使用 0到1000 归一化坐标。"
        )

    def detect(self, observation: Observation):
        response = self.helper.vlm.analyze(
            observation.rgb,
            prompt=self._prompt(
                observation.side,
                observation.rgb.shape[1],
                observation.rgb.shape[0],
            ),
            max_tokens=2048,
            temperature=0.1,
            thinking_type="disabled",
        )
        data = self.helper.vlm._parse_json_object(response)
        if not isinstance(data, dict):
            raise SortingError(f"{observation.side} VLM 未返回 JSON 对象")
        return data


class ResourceScheduler:
    """Resource locks plus a conservative disjoint-lane collision policy."""

    def __init__(self, task_config):
        safety = task_config.get("safety", {})
        self.lanes = task_config.get("arm_lanes", {
            "left": {"y_min": -10.0, "y_max": -0.02},
            "right": {"y_min": 0.02, "y_max": 10.0},
        })
        self.allow_parallel = bool(safety.get("allow_parallel", True))
        self.allow_pipeline_parallel = bool(
            safety.get("allow_pipeline_parallel", False)
        )
        self.allow_shared_source_cross_lane = bool(
            safety.get("allow_shared_source_cross_lane", False)
        )
        # A destination-to-arm declaration is stronger than a guessed lane
        # from a point coordinate.  It is required before allowing one arm
        # to place while the other arm picks from a shared source.
        self.pipeline_destination_lanes = {
            str(key): str(value)
            for key, value in task_config.get(
                "pipeline_destination_lanes", {}
            ).items()
        }
        self._lock = threading.Lock()

    def destination_lane(self, destination_id, destination_role=None):
        """Return the declared arm lane for a visual destination.

        Role keys are preferred because destination ids are generated from
        each camera observation and are not stable across runs.
        """
        if destination_role is not None:
            lane = self.pipeline_destination_lanes.get(str(destination_role))
            if lane in ("left", "right"):
                return lane
        lane = self.pipeline_destination_lanes.get(str(destination_id))
        return lane if lane in ("left", "right") else None

    def lane_for(self, point):
        point = np.asarray(point, dtype=float)
        matches = [
            side for side, limits in self.lanes.items()
            if self._point_in_lane(point, limits)
        ]
        return matches[0] if len(matches) == 1 else None

    @staticmethod
    def _point_in_lane(point, limits):
        """Support legacy y-lanes and explicit x/y work-cell lanes."""
        axis = str(limits.get("axis", "y")).lower()
        index = {"x": 0, "y": 1}.get(axis)
        if index is None:
            return False
        value = float(point[index])
        return (
            float(limits.get(f"{axis}_min", -math.inf)) <= value <=
            float(limits.get(f"{axis}_max", math.inf))
        )

    def can_parallel(self, first, second):
        if not self.allow_parallel:
            return False
        if first["arm"] == second["arm"]:
            return False
        resources_a = {first["source_id"], first["destination_id"]}
        resources_b = {second["source_id"], second["destination_id"]}
        if resources_a & resources_b:
            return False
        # Only disjoint configured work-cell lanes may run concurrently.  If
        # a point lies in a shared/dead band, serialize it instead of guessing.
        return (
            first.get("lane") is not None
            and second.get("lane") is not None
            and first["lane"] != second["lane"]
        )

    def batch(self, actions):
        """Greedy batches of at most one action per arm."""
        remaining = list(actions)
        batches = []
        while remaining:
            first = remaining.pop(0)
            batch = [first]
            for index, candidate in enumerate(remaining):
                if self.can_parallel(first, candidate):
                    batch.append(candidate)
                    remaining.pop(index)
                    break
            batches.append(batch)
        return batches



class DualVlmSorter:
    def __init__(self, config_path, task_path, log_root):
        self.task_config_path = Path(task_path)
        self.task = _load_json(self.task_config_path)
        self.config_path = config_path
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.run_dir = Path(log_root) / f"dual_vlm_sort_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.helper = GraspSkill(config_path=config_path, save_path=str(self.run_dir))
        self.scheduler = ResourceScheduler(self.task)
        self.observations = {}
        self.objects = []
        self.destinations = []
        self.plan = None
        self._planning_started_at = None

    @property
    def safety(self):
        return self.task.get("safety", {})

    def _assigned_arm_for_group(self, group):
        """Return the arm assigned by task policy for one semantic group."""
        assignments = self.task.get("group_arm_assignments", {})
        side = assignments.get(group) if isinstance(assignments, dict) else None
        if side in ("left", "right"):
            return side
        return None

    def _calib_right_to_left(self):
        calibration = self.helper.config.shared.get("calibration", {})
        matrix = calibration.get("T_right_to_left")
        source = "stored_matrix"

        # The simulator can restore an historical arm layout without
        # overwriting the real-robot hand-eye/base calibration.
        if self.helper.config.sim_mode:
            matrix = self.task.get("sim_calibration", {}).get(
                "T_right_to_left", matrix
            )
            source = "sim_task_calibration"
        if matrix is None:
            raise SortingError("缺少 shared.calibration.T_right_to_left")
        matrix = np.asarray(matrix, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise SortingError("T_right_to_left 不是有效的 4x4 矩阵")
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise SortingError("T_right_to_left 的旋转部分不是正交矩阵")
        if np.linalg.det(rotation) <= 0:
            raise SortingError("T_right_to_left 的旋转部分不是合法右手系")
        self._calibration_source = source
        return matrix

    def _point_to_left(self, side, point_base):
        if side == "left":
            return np.asarray(point_base, dtype=float)
        return (self._calib_right_to_left() @ _as_point4(point_base))[:3]

    def _save_annotated(self, observation, detections):
        image = Image.fromarray(observation.rgb.copy())
        draw = ImageDraw.Draw(image)
        for item in detections:
            box = item["box"]
            label = f"{item.get('id', '?')}:{item.get('group', item.get('role', '?'))}"
            color = "red" if "group" in item else "yellow"
            draw.rectangle(tuple(int(x) for x in box), outline=color, width=3)
            draw.text((int(box[0]), max(0, int(box[1]) - 14)), label, fill=color)
        suffix = observation.view_id or observation.side
        image.save(self.run_dir / f"{suffix}_scene_annotated.png")

    def _observation_pose_specs(self, side):
        """Expand named poses and small, configured joint-space view offsets.

        A view offset is deliberately relative to a recorded safe pose.  It
        is useful for eye-in-hand cameras whose target is clipped at an image
        edge, while avoiding hard-coded absolute robot poses in this task.
        """
        pose_config = self.task.get("observation_poses", {})
        value = pose_config.get(side) if isinstance(pose_config, dict) else None
        if isinstance(value, str):
            value = [value]
        if not isinstance(value, (list, tuple)):
            raise SortingError(f"observation_poses 必须配置 {side} 的一个或多个位姿")
        specs = []
        max_offset = float(self.safety.get("max_observation_offset_deg", 12.0))
        for index, entry in enumerate(value, 1):
            if isinstance(entry, str):
                name = entry.strip()
                tag = ""
                offsets = {}
            elif isinstance(entry, dict):
                name = str(entry.get("pose", entry.get("name", ""))).strip()
                tag = str(entry.get("tag", "")).strip()
                offsets = entry.get("joint_offsets_deg", {})
            else:
                name, tag, offsets = "", "", {}
            if not name:
                raise SortingError(f"observation_poses[{side}][{index}] 缺少 pose/name")
            if isinstance(offsets, (list, tuple)):
                if len(offsets) != 7:
                    raise SortingError(
                        f"观测位姿 {side}/{name} 的 joint_offsets_deg 必须有 7 个值"
                    )
                offsets = {f"J{i}": value for i, value in enumerate(offsets, 1)}
            if not isinstance(offsets, dict):
                raise SortingError(f"观测位姿 {side}/{name} 的 joint_offsets_deg 无效")
            checked = {}
            for joint, raw_offset in offsets.items():
                joint = str(joint).upper()
                if joint not in {f"J{i}" for i in range(1, 8)}:
                    raise SortingError(f"观测位姿偏移包含无效关节: {joint}")
                try:
                    numeric = float(raw_offset)
                except (TypeError, ValueError) as exc:
                    raise SortingError(f"观测位姿偏移 {joint} 不是数字") from exc
                if not math.isfinite(numeric) or abs(numeric) > max_offset:
                    raise SortingError(
                        f"观测位姿 {side}/{name} 的 {joint} 偏移超过安全上限 "
                        f"{max_offset:.1f}°"
                    )
                checked[joint] = numeric
            display = name
            if tag:
                display = f"{name}_{tag}"
            elif checked:
                display = f"{name}_offset{index}"
            specs.append({
                "name": name,
                "display_name": display,
                "tag": tag,
                "joint_offsets_deg": checked,
            })
        if not specs:
            raise SortingError(f"observation_poses[{side}] 不能为空")
        return specs

    def _observation_pose(self, side, spec):
        """Return a recorded observation pose plus its validated offset."""
        pose = self.helper.config.get_pose(spec["name"], side=side)
        if not isinstance(pose, dict):
            raise SortingError(f"{side} 缺少观测位姿 {spec['name']}")
        result = dict(pose)
        for joint, offset in spec["joint_offsets_deg"].items():
            if joint not in result:
                raise SortingError(f"{side}/{spec['name']} 缺少关节 {joint}")
            result[joint] = float(result[joint]) + float(offset)
        return result

    @staticmethod
    def _capture_stem(side, view_index, pose_name):
        safe_pose = "".join(
            char if char.isalnum() or char in ("-", "_") else "_"
            for char in str(pose_name)
        )
        return f"{side}_view_{view_index:02d}_{safe_pose}"

    def _capture_observations(self, move_to_observation=True):
        pose_specs = {
            side: self._observation_pose_specs(side)
            for side in ("left", "right")
        }
        if move_to_observation:
            self._home_both()
        planner = VLMScenePlanner(self.helper, self.task)
        manifest = {
            "created_at": datetime.now().isoformat(),
            "capture_mode": "look_around_multi_view",
            "observations": [],
        }
        for side in ("left", "right"):
            side_moved = False
            try:
                for view_index, pose_spec in enumerate(pose_specs[side], 1):
                    pose_name = pose_spec["display_name"]
                    pose = self._observation_pose(side, pose_spec)
                    if move_to_observation:
                        if not self._move_named(side, pose, f"observation:{pose_name}"):
                            raise SortingError(f"{side} 到观测位姿 {pose_name} 失败")
                        side_moved = True
                    # Settle at every new pose.  There is deliberately no
                    # motion between frames belonging to the same pose.
                    time.sleep(float(self.safety.get("observation_settle_s", 1.0)))
                    self.helper.save_current_transformation(side)
                    T_base_to_cam, _ = self.helper._get_side_transforms(side)
                    captured_at = datetime.now().isoformat(timespec="milliseconds")
                    captured_monotonic = time.monotonic()
                    rgb, depth = self.helper.get_camera_obs(side)
                    stem = self._capture_stem(side, view_index, pose_name)
                    rgb_path = self.run_dir / f"{stem}_rgb.png"
                    depth_path = self.run_dir / f"{stem}_depth.png"
                    # RealSenseCapture/SimCamera also save their raw frames;
                    # these deterministic names make the multi-view set
                    # easy to inspect and keep a stable manifest reference.
                    Image.fromarray(np.asarray(rgb)).save(rgb_path)
                    Image.fromarray(np.asarray(depth)).save(depth_path)
                    view_id = stem
                    obs = Observation(
                        side,
                        rgb,
                        depth,
                        np.asarray(T_base_to_cam, dtype=float),
                        view_id=view_id,
                        pose_name=pose_name,
                        capture_timestamp=captured_at,
                        capture_monotonic_s=captured_monotonic,
                    )
                    raw = planner.detect(obs)
                    with open(
                        self.run_dir / f"{stem}_vlm_inventory.json",
                        "w", encoding="utf-8"
                    ) as stream:
                        json.dump(raw, stream, ensure_ascii=False, indent=2)
                    obs_data = self._parse_observation(obs, raw)
                    obs.objects = obs_data[0]
                    obs.destinations = obs_data[1]
                    obs.source_surfaces = obs_data[2]
                    self._save_annotated(
                        obs, obs.objects + obs.destinations + obs.source_surfaces
                    )
                    self.observations[view_id] = obs
                    manifest["observations"].append({
                        "view_id": view_id,
                        "side": side,
                        "pose_name": pose_name,
                        "base_pose_name": pose_spec["name"],
                        "joint_offsets_deg": pose_spec["joint_offsets_deg"],
                        "view_index": view_index,
                        "captured_at": captured_at,
                        "capture_monotonic_s": captured_monotonic,
                        "rgb_path": str(rgb_path),
                        "depth_path": str(depth_path),
                        "vlm_inventory_path": str(
                            self.run_dir / f"{stem}_vlm_inventory.json"
                        ),
                        "annotated_path": str(
                            self.run_dir / f"{stem}_scene_annotated.png"
                        ),
                        "object_count": len(obs.objects),
                        "destination_count": len(obs.destinations),
                    })
                    cprint(
                        f"[{side}] 视角 {view_index}/{len(pose_specs[side])} "
                        f"{pose_name}: 物体 {len(obs.objects)}，"
                        f"目标容器 {len(obs.destinations)}",
                        "cyan",
                    )
            finally:
                if move_to_observation and side_moved:
                    if not self._move_named(
                        side,
                        self.helper.config.get_pose("home", side=side),
                        "home",
                    ):
                        raise SortingError(f"{side} 观测后回 home 失败")
        with open(self.run_dir / "capture_manifest.json", "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2)
        self._merge_scene()

    def _parse_observation(self, observation, data):
        min_conf = float(self.safety.get("min_vlm_confidence", 0.35))
        min_depth = float(self.safety.get("min_depth_m", 0.10))
        max_depth = float(self.safety.get("max_depth_m", 3.0))
        groups = self.task.get("groups", {})
        destinations = self.task.get("destinations", {})
        parsed_objects, parsed_destinations = [], []
        raw_objects = data.get("objects", [])
        raw_destinations = data.get("destinations", data.get("containers", []))

        def point_from(item):
            box = _finite_box(
                item.get("box"),
                observation.rgb.shape[1],
                observation.rgb.shape[0],
                format_hint=str(item.get("box_format", "normalized_1000")),
            )
            if box is None:
                return None, None
            depth_m = _depth_for_box(observation.depth, box, min_depth, max_depth)
            if depth_m is None:
                return box, None
            u, v = _box_center(box)
            cam_point = pixel_to_camera_point2(
                np.asarray([[u, v]], dtype=float), depth_m,
                cam_type=observation.side,
                intrinsics=self.helper.config.get_camera_intrinsics(observation.side),
            )[0]
            base_point = (observation.T_base_to_cam @ _as_point4(cam_point))[:3]
            return box, self._point_to_left(observation.side, base_point)

        for index, item in enumerate(raw_objects if isinstance(raw_objects, list) else []):
            if not isinstance(item, dict):
                continue
            group = str(item.get("group", "unknown")).strip()
            if group not in groups:
                continue
            confidence = float(item.get("confidence", 0.0))
            if confidence < min_conf:
                continue
            box, point = point_from(item)
            if box is None or point is None:
                cprint(f"[{observation.side}] 丢弃无有效深度的物体 {index + 1}", "yellow")
                continue
            parsed_objects.append({
                "id": str(item.get("id", f"{observation.side}_obj_{index + 1}")),
                "label": str(item.get("label", "object")),
                "color": str(item.get("color", "")),
                "group": group,
                "confidence": confidence,
                "source_surface_id": str(item.get("source_surface_id", "")).strip(),
                "box": box,
                "point_left": point.tolist(),
                "side": observation.side,
                "observation_id": observation.view_id,
            })

        for index, item in enumerate(raw_destinations if isinstance(raw_destinations, list) else []):
            if not isinstance(item, dict):
                continue
            role = str(item.get("role", "")).strip()
            if role not in destinations:
                continue
            confidence = float(item.get("confidence", 0.0))
            if confidence < min_conf:
                continue
            box, point = point_from(item)
            if box is None or point is None:
                continue
            parsed_destinations.append({
                "id": str(item.get("id", f"{observation.side}_dst_{index + 1}")),
                "role": role,
                "label": str(item.get("label", role)),
                "confidence": confidence,
                "box": box,
                "point_left": point.tolist(),
                "side": observation.side,
                "observation_id": observation.view_id,
            })

        # A VLM can occasionally describe a target platform/container twice:
        # once correctly under ``destinations`` and once incorrectly under
        # ``objects``. Never allow such a box to become a grasp target. This
        # is a visual consistency check only; it does not use simulator state.
        filtered_objects = []
        for item in parsed_objects:
            object_box = item["box"]
            ox1, oy1, ox2, oy2 = object_box
            object_area = max(ox2 - ox1, 0.0) * max(oy2 - oy1, 0.0)
            object_center = ((ox1 + ox2) / 2.0, (oy1 + oy2) / 2.0)
            overlaps_destination = False
            for destination in parsed_destinations:
                dx1, dy1, dx2, dy2 = destination["box"]
                ix1, iy1 = max(ox1, dx1), max(oy1, dy1)
                ix2, iy2 = min(ox2, dx2), min(oy2, dy2)
                intersection = max(ix2 - ix1, 0.0) * max(iy2 - iy1, 0.0)
                center_inside = (
                    dx1 <= object_center[0] <= dx2
                    and dy1 <= object_center[1] <= dy2
                )
                destination_overlap_threshold = float(self.safety.get(
                    "destination_object_overlap_threshold", 0.30
                ))
                if (
                    object_area > 0.0
                    and center_inside
                    and intersection / object_area >= destination_overlap_threshold
                ):
                    overlaps_destination = True
                    break
            if overlaps_destination:
                cprint(
                    f"[{observation.side}] 丢弃与目标平台/容器重叠的待抓框: "
                    f"{item.get('label', 'object')}",
                    "yellow",
                )
                continue
            filtered_objects.append(item)
        parsed_objects = filtered_objects

        raw_surfaces = data.get("source_surfaces", data.get("surfaces", []))
        parsed_surfaces, surface_points = [], {}
        for index, item in enumerate(raw_surfaces if isinstance(raw_surfaces, list) else []):
            if not isinstance(item, dict):
                continue
            confidence = float(item.get("confidence", 0.0))
            if confidence < min_conf:
                continue
            box, point = point_from(item)
            if box is None or point is None:
                continue
            surface_id = str(item.get("id", f"{observation.side}_src_{index + 1}"))
            surface = {
                "id": surface_id,
                "label": str(item.get("label", "source surface")),
                "confidence": confidence,
                "box": box,
                "point_left": point.tolist(),
                "side": observation.side,
            }
            parsed_surfaces.append(surface)
            surface_points[surface_id] = surface["point_left"]

        # VLM ids are local to one image. Keep only the calibrated source
        # anchor on each object; the global merge below creates physical locks.
        # If one image contains several surfaces, use the largest declared
        # surface as the active work surface. Small desk/background regions
        # can otherwise be hallucinated as additional graspable objects (for
        # example a monitor stand being labelled as a vegetable). This is a
        # geometry/scene rule and intentionally does not depend on object
        # names or priority labels.
        surface_area = {
            surface["id"]: max(surface["box"][2] - surface["box"][0], 0.0)
            * max(surface["box"][3] - surface["box"][1], 0.0)
            for surface in parsed_surfaces
        }
        dominant_surface_id = max(
            surface_area, key=surface_area.get
        ) if surface_area else None
        filtered_surface_objects = []
        for item in parsed_objects:
            source_id = str(item.get("source_surface_id", "")).strip()
            if (
                dominant_surface_id is not None
                and source_id
                and source_id in surface_area
                and source_id != dominant_surface_id
            ):
                cprint(
                    f"[{observation.side}] 丢弃非主工作表面上的待抓物体: "
                    f"{item.get('label', 'object')}",
                    "yellow",
                )
                continue
            if source_id in surface_points:
                item["source_anchor_left"] = list(surface_points[source_id])
            item.pop("source_surface_id", None)
            filtered_surface_objects.append(item)
        return filtered_surface_objects, parsed_destinations, parsed_surfaces

    def _merge_scene(self):
        # Objects from one image must never be merged solely because they are
        # close: adjacent objects on the same tray can legitimately be only a
        # few centimetres apart.  Across calibrated cameras, however, depth
        # noise and VLM box-centre error can be larger than that spacing, so
        # match only against an object that has not already been seen from the
        # current camera and use the wider cross-view gate.
        same_view_dedup = float(self.safety.get("dedup_distance_m", 0.03))
        cross_view_dedup = float(self.safety.get(
            "cross_view_merge_distance_m", 0.12
        ))
        # RGB-D box centres from two poses of the same fixed camera can move
        # more than the normal geometric gate because of occlusion and depth
        # quantisation.  Use a slightly wider, unique geometry fallback only when
        # the object has not already been observed in the current view.  This
        # keeps two same-class objects in one image separate while allowing a
        # single object to be tracked across adjacent camera poses.
        same_camera_geometry_distance = float(self.safety.get(
            "same_camera_label_merge_distance_m", 0.08
        ))
        label_merge_distance = float(self.safety.get(
            "cross_view_label_merge_distance_m", 0.35
        ))
        configured_scene_sides = self.task.get("scene_inventory_sides")
        if isinstance(configured_scene_sides, (list, tuple, set)):
            scene_sides = {
                str(side).strip() for side in configured_scene_sides
                if str(side).strip()
            }
        else:
            scene_sides = {"left", "right"}
        if not scene_sides:
            raise SortingError("scene_inventory_sides 不能为空")
        self.objects, self.destinations = [], []
        for obs in self.observations.values():
            # Some eye-in-hand views are intentionally auxiliary: they are
            # captured for grasp generation/diagnostics but must not create
            # new scene objects or containers when their depth calibration is
            # not trusted for global inventory. The real fruit/vegetable
            # task uses the left camera as its global inventory source.
            if obs.side not in scene_sides:
                continue
            for item in obs.objects:
                point = np.asarray(item["point_left"], dtype=float)
                observation_id = item.get("observation_id", obs.view_id)
                same_view_matches = [
                    obj for obj in self.objects
                    if obj.group == item["group"]
                    and any(view.get("side") == item.get("side")
                            and view.get("observation_id") == observation_id
                            and view.get("id") == item.get("id")
                            for view in obj.views)
                    and np.linalg.norm(obj.point_left - point) <= same_view_dedup
                ]
                multi_view_matches = [
                    obj for obj in self.objects
                    if obj.group == item["group"]
                    and any(
                        view.get("side") == item.get("side")
                        and view.get("observation_id") != observation_id
                        for view in obj.views
                    )
                    and np.linalg.norm(obj.point_left - point) <= float(
                        self.safety.get("multi_view_merge_distance_m", 0.08)
                    )
                ]
                # A fixed-camera VLM can change both the label and the
                # semantic group for the same object when it is partly
                # occluded (for example: red onion -> tomato -> purple ball).
                # If exactly one prior track from this camera is geometrically
                # close, use it as the identity even when the current semantic
                # answer is inconsistent.  Requiring a unique match prevents
                # two nearby objects from being merged opportunistically.
                same_camera_geometry_matches = [
                    obj for obj in self.objects
                    if any(
                        view.get("side") == item.get("side")
                        and view.get("observation_id") != observation_id
                        for view in obj.views
                    )
                    and np.linalg.norm(obj.point_left - point)
                    <= same_camera_geometry_distance
                ]
                cross_view_matches = [
                    obj for obj in self.objects
                    if obj.group == item["group"]
                    and all(view.get("side") != item.get("side")
                            for view in obj.views)
                    and np.linalg.norm(obj.point_left - point) <= cross_view_dedup
                ]
                cross_camera_geometry_matches = [
                    obj for obj in self.objects
                    if all(view.get("side") != item.get("side")
                           for view in obj.views)
                    and np.linalg.norm(obj.point_left - point) <= cross_view_dedup
                    and any(
                        _labels_overlap(view.get("label", ""), item["label"])
                        for view in obj.views
                    )
                ]
                # If the right camera's depth/TF is temporarily biased, a
                # calibrated 3-D gate cannot associate its view with the
                # left-camera track.  A unique color is still a useful visual
                # identity cue (yellow lemon vs purple onion); never use it
                # when several same-colored objects make the association
                # ambiguous.
                cross_camera_color_matches = [
                    obj for obj in self.objects
                    if all(view.get("side") != item.get("side")
                           for view in obj.views)
                    and any(
                        _colors_overlap(view.get("color", ""), item.get("color", ""))
                        for view in obj.views
                    )
                ]
                match = None
                if same_view_matches:
                    match = min(
                        same_view_matches,
                        key=lambda obj: np.linalg.norm(obj.point_left - point),
                    )
                    match_method = "same_view_geometry"
                elif len(multi_view_matches) == 1:
                    match = multi_view_matches[0]
                    match_method = "same_camera_multi_view"
                elif len(same_camera_geometry_matches) == 1:
                    match = same_camera_geometry_matches[0]
                    match_method = "same_camera_geometry_fallback"
                    cprint(
                        f"[merge] {item['label']}: 同相机几何轨迹辅助去重，"
                        f"忽略本视角语义变化",
                        "yellow",
                    )
                elif cross_view_matches:
                    match = min(
                        cross_view_matches,
                        key=lambda obj: np.linalg.norm(obj.point_left - point),
                    )
                    match_method = "cross_view_geometry"
                elif len(cross_camera_geometry_matches) == 1:
                    match = cross_camera_geometry_matches[0]
                    match_method = "cross_camera_geometry_fallback"
                    cprint(
                        f"[merge] {item['label']}: 跨相机几何去重，"
                        f"忽略单视角语义差异",
                        "yellow",
                    )
                elif len(cross_camera_color_matches) == 1:
                    match = cross_camera_color_matches[0]
                    match_method = "cross_camera_color_fallback"
                    cprint(
                        f"[merge] {item['label']}: 跨相机颜色辅助去重，"
                        "保留已有几何锚点",
                        "yellow",
                    )
                else:
                    # Camera extrinsic/depth bias can exceed the normal
                    # geometric gate.  If both views contain exactly one
                    # object with the same normalized label and group, use a
                    # wider gate.  This prevents the common case where the
                    # two cameras independently report the same cucumber,
                    # while avoiding blind label-only merging of repeated
                    # objects.
                    label_matches = [
                        obj for obj in self.objects
                        if obj.group == item["group"]
                        and all(view.get("side") != item.get("side")
                                for view in obj.views)
                        and _label_key(obj.label) == _label_key(item["label"])
                        and np.linalg.norm(obj.point_left - point) <= label_merge_distance
                    ]
                    if len(label_matches) == 1:
                        match = label_matches[0]
                        match_method = "cross_view_unique_label"
                view = dict(item)
                if match is not None:
                    view["merge_method"] = match_method
                if match is None:
                    self.objects.append(SceneObject(
                        object_id=f"object_{len(self.objects) + 1}",
                        label=item["label"], group=item["group"],
                        point_left=point, confidence=item["confidence"],
                        views=[view],
                    ))
                else:
                    match.views.append(view)
                    if item["confidence"] > match.confidence:
                        match.label = item["label"]
                        match.confidence = item["confidence"]
                    if match_method in (
                        "same_camera_multi_view",
                        "same_camera_geometry_fallback",
                        "cross_camera_geometry_fallback",
                        "cross_camera_color_fallback",
                    ):
                        # A stationary object can be called “tomato” in one
                        # view and “pepper/tomato” in another.  Once geometry
                        # associates the views, use the modal semantic group
                        # and label instead of allowing a single noisy VLM
                        # response to create a second physical object.
                        group_stats = {}
                        for view_item in match.views:
                            group = str(view_item.get("group", "unknown"))
                            stats = group_stats.setdefault(
                                group, {"weight": 0.0, "count": 0, "max": 0.0}
                            )
                            confidence = float(view_item.get("confidence", 0.0))
                            weight = confidence * _semantic_view_weight(
                                view_item.get("label", "")
                            )
                            stats["weight"] += weight
                            stats["count"] += 1
                            stats["max"] = max(stats["max"], confidence)
                        match.group = max(
                            group_stats,
                            key=lambda group: (
                                group_stats[group]["weight"],
                                group_stats[group]["count"],
                                group_stats[group]["max"],
                            ),
                        )
                        # Resolve an explicitly named, unambiguous food item
                        # when a partially occluded view gives it the wrong
                        # broad category (e.g. "洋葱或水果" vs "洋葱").
                        # This is semantic normalization, not a task-priority
                        # rule, and only applies when the label itself names
                        # the item in both Chinese and English forms.
                        fused_labels = [
                            str(view_item.get("label", "")).lower()
                            for view_item in match.views
                        ]
                        if any(
                            "洋葱" in label or "onion" in label
                            for label in fused_labels
                        ):
                            match.group = "vegetable"
                        # Do not resolve a semantic tie by first-seen order.
                        # A partially occluded lemon was previously reported
                        # as "banana" (0.85) and then "lemon" (0.92), which
                        # made the stale first label survive the fusion.
                        # Weight each label by confidence, then use count and
                        # peak confidence as deterministic tie breakers.
                        label_stats = {}
                        for view_item in match.views:
                            label_key = _label_key(
                                view_item.get("label", "object")
                            )
                            stats = label_stats.setdefault(
                                label_key,
                                {
                                    "sum": 0.0,
                                    "count": 0,
                                    "max": 0.0,
                                },
                            )
                            confidence = float(
                                view_item.get("confidence", 0.0)
                            )
                            stats["sum"] += confidence
                            stats["count"] += 1
                            stats["max"] = max(stats["max"], confidence)
                        modal_label_key = max(
                            label_stats,
                            key=lambda key: (
                                label_stats[key]["sum"],
                                label_stats[key]["count"],
                                label_stats[key]["max"],
                            ),
                        )
                        label_candidates = [
                            view_item for view_item in match.views
                            if _label_key(view_item.get("label", "object"))
                            == modal_label_key
                        ]
                        best_label = max(
                            label_candidates,
                            key=lambda view_item: float(
                                view_item.get("confidence", 0.0)
                            ),
                        )
                        match.label = best_label.get("label", match.label)
                    # Fuse independent RGB-D estimates when the cross-view
                    # geometry is trusted.  A color-only association is
                    # deliberately semantic: keep the existing anchor
                    # because averaging an invalid right-camera depth would
                    # corrupt the local left-camera grasp plan.
                    if match_method != "cross_camera_color_fallback":
                        match.point_left = np.mean(
                            [np.asarray(view_item["point_left"], dtype=float)
                             for view_item in match.views],
                            axis=0,
                        )
            for item in obs.destinations:
                point = np.asarray(item["point_left"], dtype=float)
                same_role = [
                    dst for dst in self.destinations
                    if dst.role == item["role"]
                ]
                match = next(
                    (
                        dst for dst in same_role
                        if np.linalg.norm(dst.point_left - point)
                        <= cross_view_dedup
                    ),
                    None,
                )
                role_cardinality = int(self.safety.get(
                    "max_destinations_per_role", 0
                ))
                if match is None and role_cardinality == 1 and same_role:
                    # One receiving plate is declared for each semantic role.
                    # Edge-clipped eye-in-hand views can otherwise create
                    # phantom duplicate plates with inconsistent 3-D centres.
                    match = max(
                        same_role,
                        key=lambda dst: self._destination_view_score(dst.views),
                    )
                view = dict(item)
                if match is None:
                    self.destinations.append(Destination(
                        destination_id=f"destination_{len(self.destinations) + 1}",
                        role=item["role"], label=item["label"],
                        point_left=point, confidence=item["confidence"],
                        views=[view],
                    ))
                else:
                    match.views.append(view)
                    if role_cardinality == 1:
                        best_view = max(
                            match.views,
                            key=self._destination_item_score,
                        )
                        match.point_left = np.asarray(
                            best_view["point_left"], dtype=float
                        )
                        match.confidence = float(
                            best_view.get("confidence", match.confidence)
                        )
                        match.label = best_view.get("label", match.label)

        max_objects = int(self.safety.get("max_objects", 32))
        if not self.objects or len(self.objects) > max_objects:
            raise SortingError(f"识别到的物体数量无效: {len(self.objects)}")
        if self.safety.get("require_visual_destinations", True) and not self.destinations:
            raise SortingError("没有识别到有效目标容器，拒绝执行")
        missing_roles = sorted({
            self.task["groups"][obj.group]["destination_role"]
            for obj in self.objects
            if obj.group in self.task.get("groups", {})
        } - {dst.role for dst in self.destinations})
        if missing_roles:
            raise SortingError(
                "识别到物体但缺少对应目标容器，拒绝执行: "
                + ", ".join(missing_roles)
            )
        cprint(
            f"[merge] 跨视角融合后物体数: {len(self.objects)}，"
            f"目标容器数: {len(self.destinations)}",
            "cyan",
        )
        self._save_scene_inventory()

    @staticmethod
    def _destination_item_score(item):
        """Rank a destination observation, not a movable object."""
        box = item.get("box", [])
        try:
            area = max(float(box[2]) - float(box[0]), 0.0) * max(
                float(box[3]) - float(box[1]), 0.0
            )
        except (IndexError, TypeError, ValueError):
            area = 0.0
        return float(item.get("confidence", 0.0)), area

    @classmethod
    def _destination_view_score(cls, views):
        return max(
            (cls._destination_item_score(view) for view in views),
            default=(0.0, 0.0),
        )

    def _save_scene_inventory(self):
        """Save a human-readable scene-to-object mapping before planning.

        Internal ids are useful for locks, but operators need to know which
        real item an id refers to and which visual views support it.
        """
        inventory = {
            "created_at": datetime.now().isoformat(),
            "objects": [
                {
                    "object_id": obj.object_id,
                    "object_name": obj.label,
                    "group": obj.group,
                    "confidence": float(obj.confidence),
                    "point_left_m": np.asarray(obj.point_left, dtype=float).tolist(),
                    "source_resource_id": obj.source_resource_id,
                    "views": [
                        {
                            "side": view.get("side"),
                            "observation_id": view.get("observation_id"),
                            "label": view.get("label"),
                            "color": view.get("color"),
                            "point_left_m": view.get("point_left"),
                            "merge_method": view.get("merge_method"),
                        }
                        for view in obj.views
                    ],
                }
                for obj in self.objects
            ],
            "destinations": [
                {
                    "destination_id": dst.destination_id,
                    "destination_name": dst.label,
                    "role": dst.role,
                    "confidence": float(dst.confidence),
                    "point_left_m": np.asarray(dst.point_left, dtype=float).tolist(),
                }
                for dst in self.destinations
            ],
        }
        with open(self.run_dir / "scene_inventory.json", "w", encoding="utf-8") as stream:
            json.dump(inventory, stream, ensure_ascii=False, indent=2)

    def _assign_source_resources(self):
        """Assign one lock to objects on the same physical source surface.

        A camera-local VLM id is not a safe resource id. If the VLM gives a
        source surface box, its calibrated 3-D anchor is used. Otherwise
        object anchors are conservatively clustered in XY. False merging only
        serializes work; false splitting could permit an unsafe concurrent pick.
        """
        threshold = float(self.safety.get("source_cluster_distance_m", 0.35))
        clusters = []
        for obj in self.objects:
            anchors = [
                view.get("source_anchor_left")
                for view in obj.views
                if view.get("source_anchor_left") is not None
            ]
            anchor = np.asarray(anchors[0] if anchors else obj.point_left, dtype=float)
            obj.source_anchor_left = anchor
            match = None
            for cluster in clusters:
                if np.linalg.norm(anchor[:2] - cluster["center"][:2]) <= threshold:
                    match = cluster
                    break
            if match is None:
                match = {"id": f"source_{len(clusters) + 1}", "center": anchor.copy()}
                clusters.append(match)
            match["points"] = match.get("points", []) + [anchor]
            match["center"] = np.mean(match["points"], axis=0)
            obj.source_resource_id = match["id"]

    def _preflight_real(self):
        if self.helper.config.sim_mode:
            return
        from tools.pose_record import _check_arm_health, _connect_arm
        for side in ("left", "right"):
            arm, _ = _connect_arm(side)
            if arm is None:
                raise SortingError(f"{side} 真机 SDK 连接失败")
            try:
                healthy, reason = _check_arm_health(arm)
                if not healthy:
                    raise SortingError(f"{side} 真机健康检查失败: {reason}")
            finally:
                try:
                    arm.rm_delete_robot_arm()
                except Exception:
                    pass

    def _home_both(self):
        for side in ("left", "right"):
            pose = self.helper.config.get_pose("home", side=side)
            if not self._move_named(side, pose, "home"):
                raise SortingError(f"{side} 回 home 失败")

    def _move_named(self, side, pose, label, helper=None):
        if not isinstance(pose, dict):
            return False
        helper = helper or self.helper
        cprint(f"[sort] {side} -> {label}", "cyan")
        if label == "home_after_grasp":
            speed = self.safety.get("holding_home_speed", 6)
        elif label.startswith("home"):
            speed = self.safety.get("home_speed", 12)
        else:
            speed = self.safety.get("observation_speed", 10)
        return bool(helper.arm_for(side).move_to_named_pose(
            pose,
            speed=int(speed),
        ))

    def _camera_base_to_arm(self, camera_side, arm_side):
        """Return the base-frame transform from camera arm to target arm.

        The fruit/vegetable task deliberately plans a grasp only from the
        camera mounted on the same arm.  Make that invariant explicit here:
        local grasp poses must not depend on the inter-arm calibration.  The
        cross-arm transform remains available to the global inventory merge,
        where it is needed only to deduplicate the two camera observations.
        """
        if camera_side == arm_side:
            return np.eye(4)
        T_right_to_left = self._calib_right_to_left()
        camera_base_to_left = (
            np.eye(4) if camera_side == "left" else T_right_to_left
        )
        left_to_arm = (
            np.eye(4) if arm_side == "left" else np.linalg.inv(T_right_to_left)
        )
        return left_to_arm @ camera_base_to_left

    def _destination_for(self, group, used, object_point=None):
        role = self.task["groups"][group]["destination_role"]
        choices = [dst for dst in self.destinations if dst.role == role]
        if not choices:
            raise SortingError(f"没有分组 {group} 对应的目标容器")
        point = None if object_point is None else np.asarray(object_point, dtype=float)
        # A role usually describes one receiving plate. Reusing it is valid
        # for multiple objects; do not rotate a later object onto a remote
        # duplicate container merely because the first one is already used.
        # If several same-role containers really exist, choose the nearest one
        # for the current object, then prefer higher visual confidence.
        return min(
            choices,
            key=lambda dst: (
                float(np.linalg.norm(dst.point_left - point))
                if point is not None else 0.0,
                -float(dst.confidence),
                used.get(dst.destination_id, 0),
            ),
        )

    def _target_in_arm(self, side, point_left):
        if side == "left":
            return np.asarray(point_left, dtype=float)
        return (np.linalg.inv(self._calib_right_to_left()) @ _as_point4(point_left))[:3]

    def _destination_target_in_arm(self, side, destination):
        """Return a destination point in the executing arm's own frame.

        The initial VLM inventory is fused for counting and semantic
        assignment, but eye-in-hand cameras can disagree in metric position
        when the inter-arm transform is only a coarse/manual relationship.
        Prefer a destination observation made by the executing arm itself;
        its parsed ``point_left`` is converted back only when the arm is the
        right arm, recovering the original right-base measurement.
        """
        own_views = [
            view for view in getattr(destination, "views", [])
            if view.get("side") == side and view.get("point_left") is not None
        ]
        if own_views:
            best_view = max(own_views, key=self._destination_item_score)
            return self._target_in_arm(
                side, np.asarray(best_view["point_left"], dtype=float)
            )
        return self._target_in_arm(side, destination.point_left)

    def _plan_place(self, side, candidate, destination, helper=None):
        planner_helper = helper or self.helper
        target = self._destination_target_in_arm(side, destination).copy()
        target[2] += float(self.safety.get("place_clearance_m", 0.05))
        # Execution is home-synchronized: after grasping, the arm is parked
        # at home before the cached placement trajectory is replayed.  Twin
        # must therefore validate the same start state, rather than the last
        # joint state of the grasp trajectory.
        home_pose = planner_helper.config.get_pose("home", side=side)
        current_js = self._pose_to_radians(home_pose)
        hand = np.asarray(candidate["pose"], dtype=float).copy()
        T_hand_to_end = np.asarray(candidate["T_hand_to_end"], dtype=float)
        def hand_pose_for_target(rotation, hand_point):
            """Place the grasping hand/held object at the plate target.

            VLM/RGB-D destination centres are tool/held-object points. Twin
            models the arm-end link, so the fixed hand->end transform is
            applied only when forming the pose sent to Twin.
            """
            trial = hand.copy()
            trial[:3, :3] = rotation
            trial[:3, 3] = np.asarray(hand_point, dtype=float)
            return trial
        sim_suction = planner_helper.config.sim_mode and planner_helper.config.shared.get(
            "sim_suction", {}
        ).get("enabled", False)
        # The object does not need to keep the pickup yaw when it is dropped
        # into a plate.  Keeping that yaw can make an otherwise reachable
        # home->plate motion fail Twin because the wrist joints are needlessly
        # constrained.  Rotate around the hand approach axis only: this keeps
        # the gripper's contact direction unchanged while giving Twin a few
        # equivalent wrist configurations.  The first variant preserves the
        # pickup orientation for callers that do care about it.
        place_rotations = [hand[:3, :3].copy()]
        yaw_variants_deg = self.safety.get(
            "place_yaw_variants_deg", [90.0, -90.0, 180.0]
        )
        for angle_deg in yaw_variants_deg:
            try:
                angle_rad = float(angle_deg) * np.pi / 180.0
            except (TypeError, ValueError):
                continue
            rotation = hand[:3, :3] @ R.from_euler("z", angle_rad).as_matrix()
            if not any(np.allclose(rotation, old) for old in place_rotations):
                place_rotations.append(rotation)
        if sim_suction:
            # A suction attachment transports the object independently of
            # the pickup wrist orientation. Also try the existing broad set
            # of equivalent orientations in simulation.
            for axis in ("x", "y", "z"):
                for angle in (np.pi / 2.0, -np.pi / 2.0, np.pi):
                    rotation = hand[:3, :3] @ R.from_euler(axis, angle).as_matrix()
                    if not any(np.allclose(rotation, old) for old in place_rotations):
                        place_rotations.append(rotation)
        last_info = None
        configured_approach = float(self.safety.get("place_approach_m", 0.12))
        approach_offsets = []
        for approach_m in (configured_approach, 0.08, 0.05):
            if approach_m >= 0.0 and not any(
                abs(approach_m - old) < 1e-6 for old in approach_offsets
            ):
                approach_offsets.append(approach_m)
        for place_rotation in place_rotations:
            for approach_m in approach_offsets:
                trial = hand_pose_for_target(place_rotation, target)
                pre_target = target.copy()
                pre_target[2] += approach_m
                pre = hand_pose_for_target(place_rotation, pre_target)
                pre_end = pre @ T_hand_to_end
                place_end = trial @ T_hand_to_end
                cnfg = {
                    "target_pose": [_pose7(pre_end), _pose7(place_end)],
                    "current_js": current_js,
                    "xyz_threshold": float(self.safety.get(
                        "place_xyz_threshold_m", 0.02
                    )),
                    "rpy_threshold": float(self.safety.get(
                        "place_rpy_threshold_rad", 0.15
                    )),
                    "struct": planner_helper.config.get_arm_config(side).get(
                        "twin_struct", f"{side}_arm"
                    ),
                }
                if sim_suction:
                    suction_cfg = planner_helper.config.shared.get("sim_suction", {})
                    cnfg["sim_suction"] = True
                    cnfg["xyz_threshold"] = float(
                        suction_cfg.get("twin_xyz_threshold_m", 0.08)
                    )
                    cnfg["rpy_threshold"] = float(
                        suction_cfg.get("twin_rpy_threshold_rad", 0.15)
                    )
                rsp = self._twin_generate(side, cnfg, helper=planner_helper)
                last_info = rsp.get("info")
                if rsp.get("value") and rsp.get("info", {}).get("trajectory"):
                    return (np.asarray(rsp["info"]["trajectory"], dtype=float) * 180.0 / np.pi).tolist()
        # Some targets are reachable at the final placement pose but not at
        # the extra raised waypoint (for example near a workspace boundary).
        # Twin's trajectory_generation2 accepts a single target pose and will
        # still validate the complete home->target interpolation, so use that
        # as a bounded fallback instead of rejecting a valid placement solely
        # because the approach waypoint was unreachable.
        # VLM/depth points identify the destination container, not a
        # millimetre-accurate drop point.  If the centre is close to an IK
        # boundary, sample a few nearby points in the same plate plane.  The
        # offsets are arm-local XY offsets and remain bounded by config.
        target_offsets = self.safety.get(
            "place_target_offsets_m",
            [[0.0, 0.0], [0.02, 0.0], [-0.02, 0.0], [0.0, 0.02], [0.0, -0.02]],
        )
        parsed_offsets = []
        for offset in target_offsets:
            try:
                if len(offset) != 2:
                    continue
                dx, dy = float(offset[0]), float(offset[1])
            except (TypeError, ValueError):
                continue
            if not any(abs(dx - ox) < 1e-6 and abs(dy - oy) < 1e-6
                       for ox, oy in parsed_offsets):
                parsed_offsets.append((dx, dy))
        for dx, dy in parsed_offsets:
            sampled_target = target.copy()
            sampled_target[:2] += (dx, dy)
            for place_rotation in place_rotations:
                trial = hand_pose_for_target(place_rotation, sampled_target)
                place_end = trial @ T_hand_to_end
                cnfg = {
                    "target_pose": [_pose7(place_end)],
                    "current_js": current_js,
                    "xyz_threshold": float(self.safety.get(
                        "place_xyz_threshold_m", 0.02
                    )),
                    "rpy_threshold": float(self.safety.get(
                        "place_rpy_threshold_rad", 0.15
                    )),
                    "struct": planner_helper.config.get_arm_config(side).get(
                        "twin_struct", f"{side}_arm"
                    ),
                }
                if sim_suction:
                    suction_cfg = planner_helper.config.shared.get("sim_suction", {})
                    cnfg["sim_suction"] = True
                    cnfg["xyz_threshold"] = float(
                        suction_cfg.get("twin_xyz_threshold_m", 0.08)
                    )
                    cnfg["rpy_threshold"] = float(
                        suction_cfg.get("twin_rpy_threshold_rad", 0.15)
                    )
                rsp = self._twin_generate(side, cnfg, helper=planner_helper)
                last_info = rsp.get("info")
                if rsp.get("value") and rsp.get("info", {}).get("trajectory"):
                    if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                        cprint(
                            f"[place-plan] {side} 放置点使用盘内偏移 "
                            f"dx={dx:.3f}m dy={dy:.3f}m",
                            "cyan",
                        )
                    return (np.asarray(rsp["info"]["trajectory"], dtype=float) * 180.0 / np.pi).tolist()
        raise SortingError(f"{side} 放置轨迹不可达: {last_info}")

    @staticmethod
    def _pose_to_radians(pose):
        """Convert a named joint pose (the config format is degrees) to Twin radians."""
        if not isinstance(pose, dict) or not pose:
            raise SortingError("home 位姿不是有效的关节角字典")
        try:
            return [float(value) * np.pi / 180.0 for value in pose.values()]
        except (TypeError, ValueError) as exc:
            raise SortingError("home 位姿包含无效的关节角") from exc

    def _make_plan(self):
        """Create only the semantic task queues supplied by the VLM.

        Grasp geometry is deliberately not planned here. Each action is
        executed later through ``GraspSkill.visual_grasp`` with the assigned
        arm's own camera, AnyGrasp model, intrinsics, TF and Twin service.
        This prevents stale multi-view candidates from blocking an otherwise
        valid single-arm grasp.
        """
        self._planning_started_at = time.monotonic()
        self._assign_source_resources()
        self._save_scene_inventory()

        used_destinations = {}
        actions = []
        for obj in self.objects:
            destination = self._destination_for(
                obj.group, used_destinations, object_point=obj.point_left
            )
            used_destinations[destination.destination_id] = (
                used_destinations.get(destination.destination_id, 0) + 1
            )

            side = self._assigned_arm_for_group(obj.group)
            if side not in ("left", "right"):
                raise SortingError(
                    f"{obj.object_id}[{obj.label}/{obj.group}] 没有配置负责机械臂"
                )
            destination_lane = self.scheduler.destination_lane(
                destination.destination_id, destination.role
            )
            if (
                self.safety.get("enforce_arm_lanes", False)
                and destination_lane is not None
                and destination_lane != side
            ):
                raise SortingError(
                    f"{obj.object_id}[{obj.label}/{obj.group}] 的目标容器"
                    f" {destination.label} 未分配给 {side} 臂"
                )

            action = {
                "action_id": f"action_{len(actions) + 1}",
                "object_id": obj.object_id,
                "object_name": obj.label,
                "object_label": obj.label,
                "group": obj.group,
                "arm": side,
                "arm_task": f"{side}臂视觉抓取{obj.label}，并放入{destination.label}",
                "lane": self.scheduler.lane_for(obj.point_left),
                "source_id": obj.source_resource_id,
                "destination_id": destination.destination_id,
                "destination_name": destination.label,
                "destination_role": destination.role,
                "source_point_left": obj.point_left.tolist(),
                "destination_point_left": destination.point_left.tolist(),
                "grasp_mode": "runtime_single_arm_visual",
                "grasp_camera": side,
                "grasp_score": None,
                "grasp_anchor_source": None,
                "grasp_seed_camera": None,
                "grasp_trajectory_deg": None,
                "place_trajectory_deg": None,
                # Runtime-only destination model used after this task's
                # visual grasp succeeds. It is filtered from plan.json.
                "_destination": destination,
            }
            actions.append(action)

        self.plan = {
            "type": "dual_vlm_sorting_plan",
            "created_at": datetime.now().isoformat(),
            "task_config": str(self.task_config_path),
            "sim_mode": bool(self.helper.config.sim_mode),
            "object_count": len(actions),
            "execution_mode": "interleaved_single_arm_visual_home_pipeline",
            "actions": [self._public_action(action) for action in actions],
            "arm_tasks": [
                {
                    "action_id": action["action_id"],
                    "object_id": action["object_id"],
                    "object_name": action["object_name"],
                    "arm": action["arm"],
                    "task": action["arm_task"],
                    "destination": action["destination_id"],
                    "destination_name": action["destination_name"],
                    "destination_role": action["destination_role"],
                }
                for action in actions
            ],
            "execution_batches": [
                [action["action_id"] for action in batch]
                for batch in self.scheduler.batch(actions)
            ],
            "pipeline_arm_queues": {
                side: [
                    action["action_id"]
                    for action in actions
                    if action["arm"] == side
                ]
                for side in ("left", "right")
            },
            "execution_sequence": self._interleaved_execution_sequence(actions),
        }
        self._planned_actions = actions
        self._save_plan()
        return actions

    def _planning_check(self):
        """Stop bounded planning before an unbounded candidate search grows."""
        if self._planning_started_at is None:
            return
        limit = float(self.safety.get("max_planning_time_s", 90.0))
        if limit > 0 and time.monotonic() - self._planning_started_at > limit:
            raise SortingError(f"Twin规划超过时间上限（{limit:.0f} 秒）")

    def _twin_generate(self, side, config, helper=None):
        """Call Twin with a bounded socket timeout and recoverable reconnect."""
        self._planning_check()
        planner_helper = helper or self.helper
        twin = planner_helper.twin_for(side)
        sock = getattr(twin, "sock", None)
        timeout = float(self.safety.get("twin_request_timeout_s", 10.0))
        old_timeout = sock.gettimeout() if sock is not None else None
        if sock is not None and timeout > 0:
            sock.settimeout(timeout)
        try:
            rsp = twin.generate_trajectory2(config)
            if (
                not bool(rsp.get("value"))
                and self.safety.get("enable_twin_generation3_fallback", False)
            ):
                cprint(
                    f"[{side}/place] generation2 未通过，尝试 Twin robust generation3",
                    "yellow",
                )
                rsp = twin.call_service("trajectory_generation3", config)
            return rsp
        except socket.timeout as exc:
            twin.close()
            # TwinClient is cached by GraspSkill; remove the dead client so a
            # later safe retry gets a fresh TCP connection.
            cached = getattr(planner_helper, "_twins", None)
            if isinstance(cached, dict):
                cached.pop(side, None)
            raise SortingError(
                f"{side} Twin请求超时（{timeout:.1f} 秒）"
            ) from exc
        finally:
            if sock is not None and getattr(twin, "sock", None) is sock:
                sock.settimeout(old_timeout)

    def _plan_grasp_candidate_bounded(
        self, candidate, side, obs_pose, helper=None
    ):
        """Run the grasp planner under this task's Twin limits.

        Planning normally uses the sorter helper. Runtime physical retries
        happen after that helper's sockets are intentionally closed, so they
        must use the execution worker's freshly connected Twin client.
        """
        self._planning_check()
        planner_helper = helper or self.helper
        twin = planner_helper.twin_for(side)
        # This is a task-level allowance for non-contact pre-grasp waypoints;
        # the final physical contact is still checked by Twin and the
        # gripper sensor.  Keep it on the single candidate, not as extra
        # candidate combinations.
        candidate["grasp_xyz_threshold_m"] = float(
            self.safety.get("grasp_xyz_threshold_m", 0.03)
        )
        candidate["grasp_rpy_threshold_rad"] = float(
            self.safety.get("grasp_rpy_threshold_rad", 0.05)
        )
        candidate["twin_generation3_fallback"] = bool(
            self.safety.get("enable_twin_generation3_fallback", False)
        )
        sock = getattr(twin, "sock", None)
        timeout = float(self.safety.get("twin_request_timeout_s", 10.0))
        old_timeout = sock.gettimeout() if sock is not None else None
        if sock is not None and timeout > 0:
            sock.settimeout(timeout)
        try:
            if planner_helper.grasp_pipeline._plan_grasp_candidate(candidate, side, obs_pose):
                return True

            # Keep one logical Twin candidate per object/arm, but repair an
            # orientation branch inside that candidate.  AnyGrasp can return
            # a valid contact axis whose wrist yaw is a poor IK seed; testing
            # two bounded local-yaw variants avoids treating that as a
            # workspace failure without reintroducing a combinatorial
            # candidate list.
            variants = self.safety.get(
                "grasp_plan_pose_variants_deg",
                self.safety.get("grasp_retry_pose_variants", [
                    {"yaw_deg": 90.0}, {"yaw_deg": -90.0}
                ]),
            )
            if not isinstance(variants, list):
                variants = []
            original_pose = np.asarray(candidate["pose"], dtype=float).copy()
            for spec in variants:
                if isinstance(spec, dict):
                    try:
                        yaw_deg = float(spec.get("yaw_deg", 0.0))
                    except (TypeError, ValueError):
                        continue
                else:
                    try:
                        yaw_deg = float(spec)
                    except (TypeError, ValueError):
                        continue
                if abs(yaw_deg) < 1e-9:
                    continue
                trial = dict(candidate)
                trial["pose"] = original_pose.copy()
                trial["pose"][:3, :3] = original_pose[:3, :3] @ R.from_euler(
                    "z", yaw_deg * np.pi / 180.0
                ).as_matrix()
                if planner_helper.grasp_pipeline._plan_grasp_candidate(trial, side, obs_pose):
                    candidate.update(trial)
                    candidate["planning_pose_variant_yaw_deg"] = yaw_deg
                    cprint(
                        f"[grasp-plan] {side} 候选姿态使用局部 yaw {yaw_deg:.0f}° 修复",
                        "cyan",
                    )
                    return True
            return False
        except socket.timeout as exc:
            twin.close()
            cached = getattr(planner_helper, "_twins", None)
            if isinstance(cached, dict):
                cached.pop(side, None)
            raise SortingError(
                f"{side} Twin抓取规划请求超时（{timeout:.1f} 秒）"
            ) from exc
        finally:
            if sock is not None and getattr(twin, "sock", None) is sock:
                sock.settimeout(old_timeout)

    @staticmethod
    def _public_action(action):
        return {key: value for key, value in action.items() if not key.startswith("_")}

    def _save_plan(self):
        with open(self.run_dir / "plan.json", "w", encoding="utf-8") as stream:
            json.dump(self.plan, stream, ensure_ascii=False, indent=2)

    def _candidate_for_action(self, action, candidate=None):
        candidate = candidate if candidate is not None else action["_candidate"]
        if self.helper.config.sim_mode and candidate.get("suction_mode"):
            # Bind the simulator's proximity suction to this action's
            # selected object, rather than to a noisy intermediate AnyGrasp
            # projection cached inside the candidate.
            candidate = dict(candidate)
            candidate["depth_anchor_left"] = list(action["source_point_left"])
        return candidate

    def _execute_grasp_with_retries(self, action, worker_helper):
        """Try paired grasp/place candidates until contact is confirmed.

        ``skills.base`` performs the hardware-level contact check and opens
        the gripper/repositions to the observation pose after a failed
        candidate.  This method adds the sorting-level retry policy and
        updates the place trajectory together with the candidate that won.
        """
        side = action["arm"]
        options = action.get("_grasp_options") or [{
            "candidate": action["_candidate"],
            "place_trajectory": action["place_trajectory_deg"],
        }]
        failures = []
        safety = getattr(self, "safety", {})
        max_attempts = max(
            len(options),
            max(1, int(safety.get("max_grasp_retry_candidates", 3))),
        )
        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            if attempt <= len(options):
                option = options[attempt - 1]
            else:
                # Keep the normal planning budget at one Twin candidate per
                # arm. Only a confirmed physical miss unlocks a bounded
                # runtime alternative, which is then checked by Twin before
                # it can be sent to the arm.
                option = self._runtime_grasp_option(
                    action, worker_helper, options[0]["candidate"],
                    attempt - len(options),
                )
                if option is None:
                    break
            candidate = option["candidate"]
            if attempt > 1:
                cprint(
                    f"[sort] {side} 重试抓取 {action['action_id']}："
                    f"换第 {attempt} 个候选位姿",
                    "yellow",
                )
            grasp_ok = worker_helper.grasp_pipeline._execute_scored_grasp(
                self._candidate_for_action(action, candidate),
                side,
                action["_observation_pose"],
                hold_after_grasp=True,
            )
            if grasp_ok:
                action["_candidate"] = candidate
                action["place_trajectory_deg"] = option["place_trajectory"]
                action["grasp_score"] = float(candidate.get("composite_score", 0.0))
                action["grasp_anchor_source"] = candidate.get(
                    "depth_anchor_source", "view_anchor"
                )
                action["grasp_seed_camera"] = candidate.get("camera_side")
                action["grasp_trajectory_deg"] = (
                    np.asarray(candidate["trajectory"], dtype=float)
                    * 180.0 / np.pi
                ).tolist()
                if attempt > 1:
                    cprint(
                        f"[sort] {side} 第 {attempt} 个候选位姿抓取成功 "
                        f"{action['action_id']}",
                        "green",
                    )
                return True
            failures.append(f"候选{attempt}(AnyGrasp index={candidate.get('index')})")
            cprint(
                f"[sort] {side} 候选{attempt}未确认夹到物体，准备恢复后重试",
                "yellow",
            )
        raise SortingError(
            f"{action['action_id']} 抓取失败：已尝试 "
            + ", ".join(failures)
        )

    def _runtime_grasp_option(
        self, action, worker_helper, base_candidate, variant_index
    ):
        """Create and Twin-check one fallback pose after a physical miss."""
        variants = self.safety.get(
            "grasp_retry_pose_variants",
            [
                {"yaw_deg": 90.0, "offset_m": [0.0, 0.0]},
                {"yaw_deg": -90.0, "offset_m": [0.0, 0.0]},
            ],
        )
        if not isinstance(variants, list) or not (
            0 < variant_index <= len(variants)
        ):
            return None
        spec = variants[variant_index - 1]
        if not isinstance(spec, dict):
            return None
        try:
            yaw_deg = float(spec.get("yaw_deg", 0.0))
            offset = spec.get("offset_m", [0.0, 0.0])
            if len(offset) != 2:
                return None
            dx, dy = float(offset[0]), float(offset[1])
        except (TypeError, ValueError):
            return None

        candidate = dict(base_candidate)
        candidate["pose"] = np.asarray(
            base_candidate["pose"], dtype=float
        ).copy()
        candidate["pose"][:3, :3] = candidate["pose"][:3, :3] @ R.from_euler(
            "z", yaw_deg * np.pi / 180.0
        ).as_matrix()
        candidate["pose"][:3, 3] += np.asarray([dx, dy, 0.0], dtype=float)
        candidate["runtime_retry_variant"] = variant_index
        candidate["runtime_retry_yaw_deg"] = yaw_deg
        candidate["runtime_retry_offset_m"] = [dx, dy]
        candidate.pop("trajectory", None)
        candidate["twin_reachable"] = 0.0
        try:
            self._plan_grasp_candidate_bounded(
                candidate, action["arm"], action["_observation_pose"],
                helper=worker_helper,
            )
            if not candidate.get("twin_reachable"):
                return None
            place_trajectory = self._plan_place(
                action["arm"], candidate, action["_destination"],
                helper=worker_helper,
            )
        except SortingError as exc:
            cprint(
                f"[sort] {action['arm']} 备用抓取姿态 {variant_index} Twin失败: {exc}",
                "yellow",
            )
            return None
        cprint(
            f"[sort] {action['arm']} 已生成备用抓取姿态 {variant_index} "
            f"(yaw={yaw_deg:.0f}°, dx={dx:.3f}m, dy={dy:.3f}m)",
            "cyan",
        )
        return {"candidate": candidate, "place_trajectory": place_trajectory}

    def _record_runtime_grasp(self, action, worker_helper):
        """Attach the successful single-arm visual grasp to an action."""
        candidate = getattr(
            worker_helper, "_last_successful_grasp_candidate", None
        )
        if not isinstance(candidate, dict) or not candidate.get("trajectory"):
            raise SortingError(
                f"{action['action_id']} 视觉抓取成功但没有返回有效抓取候选"
            )
        action["_candidate"] = candidate
        action["grasp_score"] = float(candidate.get("composite_score", 0.0))
        action["grasp_anchor_source"] = candidate.get(
            "depth_anchor_source", "runtime_single_arm_camera"
        )
        action["grasp_seed_camera"] = candidate.get(
            "camera_side", action["arm"]
        )
        action["grasp_camera"] = action["grasp_seed_camera"]
        action["grasp_trajectory_deg"] = (
            np.asarray(candidate["trajectory"], dtype=float)
            * 180.0 / np.pi
        ).tolist()
        return candidate

    def _execute_pick_phase(
        self, action, worker_helper, source_lock, abort_event=None
    ):
        side = action["arm"]
        if abort_event is not None and abort_event.is_set():
            raise SortingError(f"{action['action_id']} 因任务中止而取消")
        # The source lock covers the complete approach/grasp section.  It is
        # deliberately released immediately after grasp confirmation so the
        # other arm can pick while this arm is travelling to its destination.
        with source_lock:
            if abort_event is not None and abort_event.is_set():
                raise SortingError(f"{action['action_id']} 因任务中止而取消")
            cprint(
                f"[sort] {side} 开始抓取 {action['action_id']}（锁定 {action['source_id']}）",
                "cyan",
            )
            # Re-run the atomic single-arm visual grasp at execution time.
            # It captures this arm's own RGB-D frame, runs YOLO/AnyGrasp/Twin,
            # checks the gripper, and leaves the object held at the grasp pose.
            max_attempts = max(
                1, int(self.safety.get("max_visual_grasp_attempts", 3))
            )
            for attempt in range(1, max_attempts + 1):
                if attempt > 1:
                    cprint(
                        f"[sort] {side} 重新感知并尝试抓取 "
                        f"{action['action_id']}（第 {attempt}/{max_attempts} 次）",
                        "yellow",
                    )
                # Each attempt is a complete fresh single-arm visual grasp;
                # never reuse a pose from a failed attempt.
                if worker_helper.visual_grasp(
                    action["object_name"],
                    side=side,
                    location=action.get("grasp_location", "desk_front"),
                    hold_after_grasp=True,
                    use_vlm_grounding=True,
                ):
                    self._record_runtime_grasp(action, worker_helper)
                    cprint(
                        f"[sort] {side} 已确认抓取 {action['action_id']}，释放 {action['source_id']}",
                        "green",
                    )
                    return
            raise SortingError(
                f"{action['action_id']} 单臂视觉抓取失败，"
                f"已重新感知尝试 {max_attempts} 次"
            )

    def _execute_place_phase(
        self, action, worker_helper, destination_lock
    ):
        side = action["arm"]
        # From the first successful grasp onward, do not open the gripper on
        # any error.  The destination lock protects the receiving platform.
        with destination_lock:
            cprint(
                f"[sort] {side} 开始放置 {action['action_id']}（锁定 {action['destination_id']}）",
                "cyan",
            )
            arm = worker_helper.arm_for(side)
            if not arm.execute_trajectory(
                action["place_trajectory_deg"],
                speed=int(self.safety.get("place_speed", 12)),
            ):
                raise SortingError(
                    f"{action['action_id']} 放置轨迹执行失败；物体仍由夹爪保持"
                )
            hand = worker_helper.gripper_for(side)
            hand.open()
            time.sleep(0.4)
            if not hand.is_fully_open():
                raise SortingError(f"{action['action_id']} 松夹爪未确认；停止回 home")
            if not self._move_named(
                side,
                worker_helper.config.get_pose("home", side=side),
                "home",
                helper=worker_helper,
            ):
                raise SortingError(f"{action['action_id']} 放置后回 home 失败")
            cprint(f"[sort] {side} 已完成放置 {action['action_id']}", "green")

    @staticmethod
    def _interleaved_execution_sequence(actions):
        """Describe the home-synchronized order used by the executor."""
        queues = {
            side: [action for action in actions if action["arm"] == side]
            for side in ("left", "right")
        }
        indices = {"left": 0, "right": 0}
        sequence = []
        first_side = "left" if queues["left"] else "right"
        second_side = "right" if first_side == "left" else "left"

        while (
            indices[first_side] < len(queues[first_side])
            or indices[second_side] < len(queues[second_side])
        ):
            first = (
                queues[first_side][indices[first_side]]
                if indices[first_side] < len(queues[first_side])
                else None
            )
            second = (
                queues[second_side][indices[second_side]]
                if indices[second_side] < len(queues[second_side])
                else None
            )
            if first is not None:
                sequence.append({"phase": "pick_home", "action_id": first["action_id"]})
                indices[first_side] += 1
            if second is not None:
                sequence.append({"phase": "pick_home", "action_id": second["action_id"]})
                indices[second_side] += 1
            if first is not None:
                sequence.append({"phase": "place", "action_id": first["action_id"]})
            if second is not None:
                sequence.append({"phase": "place", "action_id": second["action_id"]})
        return sequence

    def _execute_pick_then_home(self, action, worker_helper):
        """Grasp an item and park the holding arm at home before handoff."""
        side = action["arm"]
        hand = worker_helper.gripper_for(side)
        scoring = worker_helper.config.get_grasp_scoring(side)
        home_pose = worker_helper.config.get_pose("home", side=side)
        max_regrasp = max(
            0, int(self.safety.get("max_post_home_regrasp_attempts", 2))
        )

        # A failed post-home check is a grasp failure, not a reason to reuse
        # the old candidate. Re-enter the complete single-arm visual grasp
        # path so the object can be localized again after it has moved.
        for regrasp_attempt in range(max_regrasp + 1):
            self._execute_pick_phase(
                action,
                worker_helper,
                source_lock=threading.Lock(),
            )
            if not self._move_named(
                side,
                home_pose,
                "home_after_grasp",
                helper=worker_helper,
            ):
                raise SortingError(
                    f"{action['action_id']} 抓取后回 home 失败；物体仍由夹爪保持"
                )
            if hand.is_grasping(
                force=int(scoring.get("gripper_close_force", 20))
            ):
                break
            if regrasp_attempt >= max_regrasp:
                raise SortingError(
                    f"{action['action_id']} 回 home 后未确认仍持有物体，"
                    f"已重新视觉抓取 {max_regrasp} 次"
                )
            cprint(
                f"[sort] {side} 回 home 持物复核失败，重新视觉抓取 "
                f"{action['action_id']}（第 {regrasp_attempt + 1}/{max_regrasp} 次）",
                "yellow",
            )
        # Placement is planned only after this arm has returned home. This
        # uses the exact candidate produced by the just-finished single-arm
        # visual grasp, then blocks the other arm until this planning step is
        # complete as part of the home barrier.
        self._planning_started_at = time.monotonic()
        action["place_trajectory_deg"] = self._plan_place(
            side, action["_candidate"], action["_destination"],
            helper=worker_helper,
        )
        if getattr(self, "plan", None) is not None:
            self.plan["actions"] = [
                self._public_action(item)
                for item in getattr(self, "_planned_actions", [action])
            ]
            self._save_plan()
        cprint(f"[sort] {side} 已抓取并回到 home: {action['action_id']}", "green")

    def _execute_interleaved_home_pipeline(self, actions, worker_helpers):
        """Run per-action visual picks as pick/home, pick/home, place, place.

        Each pick is delegated to the atomic single-arm visual grasp skill.
        The second grasp is not started until the first arm is parked at home.
        The first placement is not started until the second arm is parked at
        home. This keeps both arms synchronized without overlapping their
        source-to-home or home-to-destination motions.
        """
        queues = {
            side: [action for action in actions if action["arm"] == side]
            for side in ("left", "right")
        }
        indices = {"left": 0, "right": 0}
        first_side = "left" if queues["left"] else "right"
        second_side = "right" if first_side == "left" else "left"

        cprint(
            "[sort] 启用 home 同步流水线：抓取→回 home→另一臂抓取→回 home→依次放置",
            "cyan",
        )
        while (
            indices[first_side] < len(queues[first_side])
            or indices[second_side] < len(queues[second_side])
        ):
            first = (
                queues[first_side][indices[first_side]]
                if indices[first_side] < len(queues[first_side])
                else None
            )
            second = (
                queues[second_side][indices[second_side]]
                if indices[second_side] < len(queues[second_side])
                else None
            )
            if first is not None:
                indices[first_side] += 1
                self._execute_pick_then_home(first, worker_helpers[first_side])
            if second is not None:
                indices[second_side] += 1
                self._execute_pick_then_home(second, worker_helpers[second_side])
            if first is not None:
                self._execute_place_phase(
                    first,
                    worker_helpers[first_side],
                    destination_lock=threading.Lock(),
                )
            if second is not None:
                self._execute_place_phase(
                    second,
                    worker_helpers[second_side],
                    destination_lock=threading.Lock(),
                )

    def _finish_pipeline(self, worker_helpers):
        """Put both arms and grippers into the declared completed state."""
        home_failures = []
        for side in ("left", "right"):
            if not self._move_named(
                side,
                worker_helpers[side].config.get_pose("home", side=side),
                "home",
                helper=worker_helpers[side],
            ):
                home_failures.append(side)
        if home_failures:
            raise SortingError(
                "流水线完成收尾时回 home 失败: " + ", ".join(home_failures)
            )

        for side in ("left", "right"):
            hand = worker_helpers[side].gripper_for(side)
            hand.open()
            time.sleep(0.4)
            if not hand.is_fully_open():
                raise SortingError(f"流水线完成收尾时 {side} 夹爪未确认完全打开")
        cprint("[sort] 流水线完成：双臂已回 home，左右夹爪均已打开", "green")

    def _open_grippers_on_abort(self, worker_helpers):
        """Open both grippers when execution aborts, without masking the error."""
        failures = []
        for side in ("left", "right"):
            try:
                # Connect lazily as well: the arm that was not reached yet
                # may not have created its worker gripper client.
                hand = worker_helpers[side].gripper_for(side)
                hand.open()
                time.sleep(0.4)
                if not hand.is_fully_open():
                    failures.append(f"{side} 未确认完全打开")
            except Exception as exc:  # noqa: BLE001
                failures.append(f"{side}: {exc}")
        if failures:
            cprint(
                "[sort] 异常终止时打开夹爪失败: " + "; ".join(failures),
                "red",
            )
        else:
            cprint("[sort] 异常终止收尾：左右夹爪均已打开", "yellow")

    @staticmethod
    def _close_helper_clients(helper):
        """Close idle planning sockets before another helper takes the service.

        The real arm/gripper servers process one TCP client at a time.  The
        observation/planning helper therefore must not keep its idle sockets
        open while execution helpers connect, otherwise the latter can sit in
        the listen backlog until the arm request times out.
        """
        clients = []
        for name in ("_arms", "_hands", "_twins"):
            clients.extend(getattr(helper, name, {}).values())
        perception = getattr(helper, "_perception", None)
        if perception is not None:
            anygrasp = getattr(perception, "_anygrasp_client", None)
            if anygrasp is not None:
                clients.append(anygrasp)
        cameras = getattr(helper, "_cameras", {})
        if isinstance(cameras, dict):
            clients.extend(cameras.values())
        camera = getattr(helper, "_camera", None)
        if camera is not None:
            clients.append(camera)

        seen = set()
        for client in clients:
            if client is None or id(client) in seen:
                continue
            seen.add(id(client))
            # GripperClient.close() is a physical close command. Prefer its
            # transport-only close_connection() when both methods exist.
            close = getattr(client, "close_connection", None)
            if close is None:
                close = getattr(client, "close", None)
            if close is not None:
                try:
                    close()
                except Exception as exc:  # noqa: BLE001
                    cprint(f"[sort] 关闭服务连接失败: {exc}", "yellow")

    def execute(self, actions):
        # Every run starts from a known pair state.  Execution is deliberately
        # synchronized at home between the two arms; this is the only stage
        # policy used for real and simulated hardware.
        try:
            self._home_both()
        finally:
            # Planning has opened persistent sockets on the main helper.
            # Release them even if the home move itself fails, before any
            # execution helper attempts to connect to single-client services.
            self._close_helper_clients(self.helper)
        worker_helpers = {
            side: GraspSkill(config_path=self.config_path, save_path=str(self.run_dir))
            for side in ("left", "right")
        }
        completed = False
        try:
            self._execute_interleaved_home_pipeline(actions, worker_helpers)
            self._finish_pipeline(worker_helpers)
            completed = True
        finally:
            if not completed:
                self._open_grippers_on_abort(worker_helpers)
            self._close_helper_clients(worker_helpers["left"])
            self._close_helper_clients(worker_helpers["right"])

    def run(self, execute=False, yes=False, move_to_observation=True):
        self._preflight_real()
        if not yes:
            answer = input(
                "确认工作区无人员、障碍物、未登记物体，且两夹爪可安全操作？输入 yes 继续："
            ).strip().lower()
            if answer != "yes":
                raise SortingError("操作者取消")
        self._capture_observations(move_to_observation=move_to_observation)
        actions = self._make_plan()
        cprint(f"[sort] 计划已保存: {self.run_dir / 'plan.json'}", "green")
        cprint(f"[sort] 物体数: {len(actions)}; 执行批次: {self.plan['execution_batches']}", "cyan")
        if not execute:
            cprint("[sort] plan-only：未执行抓取/放置", "yellow")
            return self.plan
        if not self.helper.config.sim_mode and not os.environ.get("DUAL_SORT_REAL_CONFIRM"):
            raise SortingError(
                "真机执行还需要环境变量 DUAL_SORT_REAL_CONFIRM=1，"
                "并使用 --real-confirm 显式确认"
            )
        self.execute(actions)
        cprint("[sort] 双臂分拣完成", "green")
        return self.plan


def main():
    parser = argparse.ArgumentParser(description="VLM 双臂视觉分拣")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "robot_config.json"))
    parser.add_argument("--task-config", default=str(DEFAULT_TASK_CONFIG))
    parser.add_argument("--log-root", default=str(PROJECT_ROOT / "log"))
    parser.add_argument("--execute", action="store_true", help="执行已规划的抓取/放置流程")
    parser.add_argument("--plan-only", action="store_true", help="只拍照并规划，不抓取")
    parser.add_argument("--sim", action="store_true", help="强制使用仿真后端")
    parser.add_argument("--real-confirm", action="store_true", help="允许真机执行")
    parser.add_argument("--yes", action="store_true", help="跳过安全确认提示")
    parser.add_argument("--no-observation-motion", action="store_true",
                        help="假设两臂已经在配置的观测位姿")
    args = parser.parse_args()
    if args.sim:
        os.environ["SIM_MODE"] = "1"
    if args.real_confirm:
        os.environ["DUAL_SORT_REAL_CONFIRM"] = "1"
    if args.execute and args.plan_only:
        parser.error("--execute 与 --plan-only 不能同时使用")
    try:
        sorter = DualVlmSorter(args.config, args.task_config, args.log_root)
        result = sorter.run(
            execute=args.execute,
            yes=args.yes,
            move_to_observation=not args.no_observation_motion,
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:  # noqa: BLE001
        cprint(f"[sort] 安全停止: {type(exc).__name__}: {exc}", "red")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
