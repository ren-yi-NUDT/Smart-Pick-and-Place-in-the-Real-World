import copy
import socket
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core.transforms import pixel_to_camera_point2


@register_skill("place")
class PlaceSkill(Skill):
    """Vision-based placement: detect container, compute 3D position, execute."""

    def run(self, **kwargs):
        container = kwargs.get("container")
        if container is None:
            cprint("place skill: missing 'container' parameter", "red")
            return False

        side = kwargs.get("side", "left")
        if side not in ("left", "right"):
            cprint(f"place skill: unsupported arm side '{side}'", "red")
            return False

        location = kwargs.get("location", "desk_front")
        use_vlm_grounding = bool(kwargs.get("use_vlm_grounding", True))
        object_name = kwargs.get("object") or kwargs.get("object_name")
        object_size_m = kwargs.get("object_size_m", kwargs.get("object_footprint_m"))
        observation_keys = self._observation_pose_keys(side, location)
        if not observation_keys:
            cprint(f"[place] no observation pose configured for {side} arm", "red")
            return False

        placement_succeeded = False
        for key in observation_keys:
            if not self.control_arm(pose_type=key, speed=30, side=side):
                cprint(f"[place] failed to reach {side}-arm observation pose {key}", "yellow")
                continue
            self.rgb, self.depth = self.get_camera_obs(side=side)
            self.save_current_transformation(side=side)
            placing_pos_world = self._get_placing_position(
                container,
                self.rgb,
                side=side,
                use_vlm_grounding=use_vlm_grounding,
                object_size_m=object_size_m,
            )
            if not len(placing_pos_world):
                continue

            # Synchronize the physical starting state with Twin before
            # planning the placement.  The target is already expressed in the
            # arm base frame, so moving from the observation pose to home does
            # not invalidate the detected container position.
            placement_start_key = key
            if self.config.shared.get("place_home_sync", True):
                if not self.control_arm(pose_type="home", speed=30, side=side):
                    cprint(
                        f"[place] failed to synchronize {side} arm at home before placement",
                        "yellow",
                    )
                    continue
                placement_start_key = "home"

            # Twin must start from the pose that the arm is actually in.
            check = self._execute_placement(
                placing_pos_world,
                initial_js_key=placement_start_key,
                side=side,
                container_name=container,
                object_name=object_name,
                object_size_m=object_size_m,
            )
            if check:
                placement_succeeded = True
                break
            # A failed attempt may already have reached a pre-place pose while
            # still holding the object. Do not move to another observation
            # pose and retry blindly.
            cprint(
                f"[place] {side} placement attempt failed; stopping retries while object state is uncertain",
                "red",
            )
            break

        safe_pose = "grasp1" if self.config.get_pose("grasp1", side=side) else "home"
        returned_to_safe_pose = self.control_arm(
            pose_type=safe_pose, speed=30, side=side
        )
        if not placement_succeeded:
            cprint(
                f"[place] failed to complete placement of '{container}' with {side} arm: "
                "motion, release, or post-release verification failed",
                "red",
            )
            return False
        if not returned_to_safe_pose:
            cprint("[place] placement succeeded but safe-pose return failed", "red")
            return False
        return True

    def _observation_pose_keys(self, side, location):
        """Return observation poses from the selected arm's own configuration."""
        arm_config = self.config.get_arm_config(side)
        default_traj_js = arm_config.get("default_traj_js", {})
        if side == "left":
            return [key for key in default_traj_js if "grasp" in key]

        # The right arm has location-based observation poses rather than the
        # left arm's grasp1..grasp4 poses.
        if location in default_traj_js:
            return [location]
        if "desk_front" in default_traj_js:
            return ["desk_front"]
        return list(default_traj_js)[:1]

    def _vlm_container_detection_attempts(self, class_name, image):
        """Build VLM-grounded container detection attempts.

        The VLM is only a fallback for placement perception.  If it returns a
        precise instance box, that box is tried first; otherwise its expanded
        prompts are sent back through YOLO-World.  Neither result directly
        commands the robot: AnyGrasp still has to provide a valid pose inside
        the selected box.
        """
        try:
            grounding = self.vlm.ground_object(image, class_name)
        except Exception as exc:
            cprint(f"[place] VLM grounding failed for '{class_name}': {exc}", "yellow")
            return []

        prompts = grounding.get("prompts", []) if isinstance(grounding, dict) else []
        prompts = [str(item).strip() for item in prompts if str(item).strip()]
        if not prompts:
            prompts = [class_name]
        box = grounding.get("box") if isinstance(grounding, dict) else None
        cprint(
            f"[place] VLM fallback for '{class_name}': prompts={prompts} box={box}",
            "cyan",
        )

        attempts = []
        valid_box = False
        if isinstance(box, (list, tuple)) and len(box) == 4:
            try:
                box = [float(value) for value in box]
                if (
                    all(np.isfinite(value) for value in box)
                    and box[2] > box[0]
                    and box[3] > box[1]
                ):
                    # Keep the VLM box as a detection-like record so the
                    # existing bbox/depth/AnyGrasp path remains unchanged.
                    attempts.append(("VLM target box", [box + [1.0, 0.0]]))
                    valid_box = True
            except (TypeError, ValueError):
                pass

        # A valid VLM box already fulfils semantic grounding. Do not run
        # YOLOWorld again on the same frame: besides being redundant, its
        # CLIP branch can fail on mixed CUDA/CPU installations.
        if not valid_box:
            try:
                detections = self.perception.detect_objects(image, prompts, conf=0.25)
            except Exception as exc:
                cprint(f"[place] VLM prompt detection failed: {exc}", "yellow")
                detections = []
            if detections:
                attempts.append(("VLM prompts", detections))
        return attempts

    def _object_footprint_m(self, object_size_m=None):
        """Return the held object's conservative XY footprint in metres."""
        value = object_size_m
        if value is None:
            value = self.config.shared.get("place_object_footprint_m", [0.06, 0.06])
        try:
            if np.isscalar(value):
                size = float(value)
                footprint = (size, size)
            else:
                values = [float(item) for item in value]
                if len(values) == 1:
                    footprint = (values[0], values[0])
                else:
                    footprint = (values[0], values[1])
            if not all(np.isfinite(item) and item > 0.0 for item in footprint):
                raise ValueError
            return footprint
        except (TypeError, ValueError, IndexError):
            cprint(
                f"[place] invalid object footprint {value!r}; using 0.06m x 0.06m",
                "yellow",
            )
            return (0.06, 0.06)

    def _segment_container_interior(self, image, box):
        """Estimate a container's visible interior mask from a detector box.

        There is no segmentation service in the current runtime.  GrabCut is
        therefore used as a local, bounded segmentation step, with an inner
        ellipse/rectangle fallback for low-texture bowls and plates.  The
        returned mask is deliberately conservative: it describes where the
        centre of the held object may be placed, not the whole container.
        """
        try:
            import cv2
        except ImportError:
            cv2 = None
        height, width = image.shape[:2]
        x1, y1, x2, y2 = [int(value) for value in box]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(width, x2), min(height, y2)
        if x2 <= x1 or y2 <= y1:
            return np.zeros((height, width), dtype=np.uint8)

        mask = np.zeros((height, width), dtype=np.uint8)
        if cv2 is not None and x2 - x1 >= 12 and y2 - y1 >= 12:
            try:
                gc_mask = np.full((height, width), cv2.GC_BGD, dtype=np.uint8)
                gc_mask[y1:y2, x1:x2] = cv2.GC_PR_BGD
                margin_x = max(2, int(0.18 * (x2 - x1)))
                margin_y = max(2, int(0.18 * (y2 - y1)))
                ix1, ix2 = x1 + margin_x, x2 - margin_x
                iy1, iy2 = y1 + margin_y, y2 - margin_y
                if ix2 > ix1 and iy2 > iy1:
                    gc_mask[iy1:iy2, ix1:ix2] = cv2.GC_PR_FGD
                bgd_model = np.zeros((1, 65), dtype=np.float64)
                fgd_model = np.zeros((1, 65), dtype=np.float64)
                cv2.grabCut(
                    image, gc_mask, (x1, y1, x2 - x1, y2 - y1),
                    bgd_model, fgd_model, 2, cv2.GC_INIT_WITH_MASK,
                )
                mask = np.where(
                    (gc_mask == cv2.GC_FGD) | (gc_mask == cv2.GC_PR_FGD), 255, 0
                ).astype(np.uint8)
                kernel = np.ones((5, 5), dtype=np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            except Exception as exc:
                cprint(f"[place] container GrabCut failed; using geometric mask: {exc}", "yellow")

        # GrabCut can classify a uniformly coloured plate as background.  A
        # geometric interior is safer than rejecting a valid target entirely.
        box_area = float((x2 - x1) * (y2 - y1))
        if int(np.count_nonzero(mask)) < max(20, 0.05 * box_area):
            mask = np.zeros((height, width), dtype=np.uint8)
            if cv2 is not None:
                center = ((x1 + x2) // 2, (y1 + y2) // 2)
                axes = (max(2, int(0.40 * (x2 - x1))), max(2, int(0.40 * (y2 - y1))))
                cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)
            else:
                mx, my = int(0.10 * (x2 - x1)), int(0.10 * (y2 - y1))
                mask[y1 + my:y2 - my, x1 + mx:x2 - mx] = 255
        return mask

    def _safe_placement_region(self, image, box, depth_m, object_size_m=None):
        """Shrink the visible interior by the projected held-object radius."""
        mask = self._segment_container_interior(image, box)
        footprint_x, footprint_y = self._object_footprint_m(object_size_m)
        intrinsics = self.config.get_camera_intrinsics(self._placement_side)
        fx = float(intrinsics.get("fx", intrinsics.get("Fx", 600.0)))
        fy = float(intrinsics.get("fy", intrinsics.get("Fy", fx)))
        projected_radius = max(fx * footprint_x, fy * footprint_y) / max(depth_m, 1e-3) / 2.0
        extra_px = float(self.config.shared.get("place_region_margin_px", 4.0))
        erosion_px = int(np.ceil(projected_radius + extra_px))
        safe = mask
        try:
            import cv2
            if erosion_px > 0:
                kernel_size = min(2 * erosion_px + 1, 101)
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
                )
                safe = cv2.erode(mask, kernel)
        except ImportError:
            pass
        if np.count_nonzero(safe) < int(self.config.shared.get("place_min_safe_pixels", 25)):
            cprint(
                f"[place] safe region too small after {erosion_px}px object-margin erosion; "
                "using a smaller conservative erosion",
                "yellow",
            )
            try:
                import cv2
                fallback_px = max(1, min(erosion_px // 2, 12))
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (2 * fallback_px + 1, 2 * fallback_px + 1)
                )
                safe = cv2.erode(mask, kernel)
            except ImportError:
                safe = mask
        return safe

    def _depth_at_pixel_m(self, x, y, fallback=None):
        """Use a small valid-depth patch at a sampled image pixel."""
        height, width = self.depth.shape[:2]
        radius = int(self.config.shared.get("place_depth_patch_px", 4))
        x, y = int(round(x)), int(round(y))
        x1, x2 = max(0, x - radius), min(width, x + radius + 1)
        y1, y2 = max(0, y - radius), min(height, y + radius + 1)
        values = self.depth[y1:y2, x1:x2]
        values = values[values > 0].astype(np.float64) * 1e-3
        min_depth_m = float(self.config.shared.get("place_min_depth_m", 0.10))
        max_depth_m = float(self.config.shared.get("place_max_depth_m", 3.0))
        values = values[(values >= min_depth_m) & (values <= max_depth_m)]
        return float(np.percentile(values, 50)) if len(values) else fallback

    def _sample_safe_pixels(self, safe_mask, box, max_candidates=8):
        """Sample well-separated points, prioritising the largest clearance."""
        try:
            import cv2
            distance = cv2.distanceTransform((safe_mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
        except ImportError:
            distance = safe_mask.astype(np.float32)
        ys, xs = np.where(safe_mask > 0)
        if not len(xs):
            return []
        order = np.argsort(distance[ys, xs])[::-1]
        min_spacing = float(self.config.shared.get("place_candidate_spacing_px", 12.0))
        selected = []
        for index in order:
            point = (int(xs[index]), int(ys[index]))
            if all(np.hypot(point[0] - old[0], point[1] - old[1]) >= min_spacing for old in selected):
                selected.append(point)
            if len(selected) >= int(max_candidates):
                break
        # Keep the image-box centre when it is safe, even if a distance peak
        # is slightly off-centre due to segmentation noise.
        center = (int(round((box[0] + box[2]) / 2)), int(round((box[1] + box[3]) / 2)))
        if 0 <= center[1] < safe_mask.shape[0] and 0 <= center[0] < safe_mask.shape[1] and safe_mask[center[1], center[0]]:
            selected.append(center)
        unique = []
        for point in selected:
            if point not in unique:
                unique.append(point)
        return unique[:int(max_candidates)]

    def _placing_position_from_detections(
        self, class_name, detections, side="left", source="YOLO", object_size_m=None
    ):
        """Build a safe placement region and several world-frame SE(3) targets."""
        self._placement_rotation = None
        self._placement_candidate_targets_world = []
        try:
            det = detections[0][:4]
            if det[1] >= 400 and det[3] <= 480 and len(detections) > 1:
                det = detections[1][:4]
            x1, y1, x2, y2 = [int(c) for c in det]
            H, W = self.depth.shape[:2]
            x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
            if x2 <= x1 or y2 <= y1:
                return None
            mean_depth_m = self._robust_container_depth_m(x1, y1, x2, y2)
            if mean_depth_m is None:
                cprint(f"[place] {source}: no valid depth inside {class_name}", "yellow")
                return None

            self._placement_side = side
            self._placement_last_box = (x1, y1, x2, y2)
            self._placement_last_source = source
            best_grasp = self._select_best_container_grasp(
                self.rgb, self.depth, (x1, y1, x2, y2), side=side
            )
            if best_grasp is not None:
                self._placement_rotation = best_grasp["rotation"]
                cprint(
                    f"[place] {source}: AnyGrasp container orientation index={best_grasp['index']} "
                    f"score={best_grasp['score']:.4f}", "cyan",
                )
            else:
                # Position planning can still be useful when AnyGrasp has no
                # point inside a thin/occluded container. Preserve the last
                # grasp orientation if available; otherwise fail safely.
                previous = getattr(self, "_last_successful_grasp_candidate", None)
                if isinstance(previous, dict) and np.asarray(previous.get("pose", [])).shape == (4, 4):
                    self._placement_rotation = np.asarray(previous["pose"][:3, :3], dtype=float)
                    cprint("[place] AnyGrasp has no container pose; reusing current grasp orientation", "yellow")
                else:
                    cprint(f"[place] {source}: no valid orientation for {class_name}", "yellow")
                    return None

            safe_mask = self._safe_placement_region(
                self.rgb, (x1, y1, x2, y2), mean_depth_m, object_size_m
            )
            pixels = self._sample_safe_pixels(safe_mask, (x1, y1, x2, y2))
            if not pixels:
                cprint(f"[place] {source}: no safe interior pixel for {class_name}", "yellow")
                return None
            self._placement_region_mask = safe_mask
            self._placement_region_depth_m = mean_depth_m
            for px, py in pixels:
                point_depth_m = self._depth_at_pixel_m(px, py, fallback=mean_depth_m)
                point_cam = pixel_to_camera_point2(
                    np.array([px, py]).reshape(-1, 2), point_depth_m,
                    cam_type=side, intrinsics=self.config.get_camera_intrinsics(side),
                ).flatten()
                target = self._transform_pose_to_world(point_cam, side=side)
                self._placement_candidate_targets_world.append(target)
            cprint(
                f"[place] {source}: interior/safe region generated "
                f"{len(self._placement_candidate_targets_world)} SE(3) candidates "
                f"(object footprint={self._object_footprint_m(object_size_m)})",
                "cyan",
            )
            return self._placement_candidate_targets_world[0]
        except Exception as exc:
            cprint(f"[place] {source} container pose failed: {exc}", "yellow")
            self._placement_candidate_targets_world = []
            return None

    def _robust_container_depth_m(self, x1, y1, x2, y2):
        """Estimate container depth from the inner box region.

        A detector/VLM box often includes the table around a plate.  Avoiding
        the boundary and using the nearer valid quartile follows the sorting
        pipeline and is more stable than taking a median over the whole box.
        """
        height, width = self.depth.shape[:2]
        ix1 = int(max(0, x1 + 0.25 * (x2 - x1)))
        ix2 = int(min(width, x2 - 0.25 * (x2 - x1)))
        iy1 = int(max(0, y1 + 0.25 * (y2 - y1)))
        iy2 = int(min(height, y2 - 0.25 * (y2 - y1)))
        if ix2 <= ix1 or iy2 <= iy1:
            return None

        values_m = self.depth[iy1:iy2, ix1:ix2]
        values_m = values_m[values_m > 0].astype(np.float64) * 1e-3
        min_depth_m = float(self.config.shared.get("place_min_depth_m", 0.10))
        max_depth_m = float(self.config.shared.get("place_max_depth_m", 3.0))
        values_m = values_m[
            (values_m >= min_depth_m) & (values_m <= max_depth_m)
        ]
        if not len(values_m):
            return None
        return float(np.percentile(values_m, 25.0))

    def _get_placing_position(
        self, class_name, image, side="left", use_vlm_grounding=True, object_size_m=None
    ):
        """Detect a container and return the best safe target.

        VLM grounding is deliberately first in this placement pipeline: its
        box defines the semantic target, while local perception/AnyGrasp only
        supplies geometry and orientation. YOLO remains a bounded fallback
        for deployments without a VLM token or when the VLM box is unusable.
        """
        if use_vlm_grounding:
            for source, detections in self._vlm_container_detection_attempts(
                class_name, image
            ):
                pose = self._placing_position_from_detections(
                    class_name, detections, side=side, source=source,
                    object_size_m=object_size_m,
                )
                if pose is not None:
                    return pose

        try:
            detections = self.perception.detect_objects(
                image, [class_name], conf=0.25
            )
        except Exception as exc:
            cprint(f"[place] direct container detection failed: {exc}", "yellow")
            detections = []
        if detections:
            pose = self._placing_position_from_detections(
                class_name, detections, side=side, source="YOLO",
                object_size_m=object_size_m,
            )
            if pose is not None:
                return pose

        cprint(f"[place] no usable container pose for '{class_name}'", "yellow")
        return []

    def _transform_pose_to_world(self, pose_cam_point, side="left"):
        T_cam_point = np.eye(4)
        T_cam_point[:3, 3] = pose_cam_point.flatten()
        T_base_to_cam, _ = self._get_side_transforms(side)
        target = T_base_to_cam @ T_cam_point
        placement_rotation = getattr(self, "_placement_rotation", None)
        if placement_rotation is not None:
            target[:3, :3] = placement_rotation
        return target

    @staticmethod
    def _pose7(matrix):
        """Convert a homogeneous pose to Twin's [xyz, quaternion] format."""
        pos = np.asarray(matrix[:3, 3], dtype=float)
        quat = R.from_matrix(np.asarray(matrix[:3, :3], dtype=float)).as_quat()
        return [float(value) for value in (*pos, *quat)]

    @staticmethod
    def _vertical_gripper_rotation(rotation):
        """Make the gripper approach axis point straight down.

        The project uses the hand's local +Z axis as its approach axis. Keep
        the original AnyGrasp orientation's horizontal heading by projecting
        its local X axis onto the horizontal plane, then construct the nearest
        proper rotation whose local Z axis is the base-frame down direction.
        """
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("invalid placement rotation")

        down = np.array([0.0, 0.0, -1.0])
        x_axis = rotation[:, 0].copy()
        x_axis -= down * float(np.dot(x_axis, down))
        if np.linalg.norm(x_axis) < 1e-8:
            # A degenerate source heading is rare, but the original Y axis
            # provides a deterministic horizontal fallback.
            x_axis = rotation[:, 1].copy()
            x_axis -= down * float(np.dot(x_axis, down))
        if np.linalg.norm(x_axis) < 1e-8:
            x_axis = np.array([1.0, 0.0, 0.0])
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(down, x_axis)
        y_axis /= max(np.linalg.norm(y_axis), 1e-8)
        return np.column_stack((x_axis, y_axis, down))

    def _twin_generate(self, cnfg, side="left"):
        """Call Twin with the bounded/recoverable request logic used by sort."""
        twin = self.twin_for(side)
        sock = getattr(twin, "sock", None)
        timeout = float(self.config.shared.get("twin_request_timeout_s", 30.0))
        old_timeout = sock.gettimeout() if sock is not None else None
        if sock is not None and timeout > 0:
            sock.settimeout(timeout)

        try:
            rsp = twin.generate_trajectory2(cnfg)
            # Keep the same robust fallback as dual_vlm_sorting.  The service
            # accepts the same placement payload for generation3.
            if (
                not bool(rsp.get("value"))
                and self.config.shared.get("enable_twin_generation3_fallback", True)
            ):
                cprint(
                    "[place] trajectory_generation2 failed; trying generation3",
                    "yellow",
                )
                rsp = twin.call_service("trajectory_generation3", cnfg)
            return rsp
        except socket.timeout as exc:
            cprint(f"[place] Twin request timed out after {timeout:.1f}s", "red")
            twin.close()
            self._twins.pop(side, None)
            raise RuntimeError("Twin request timed out") from exc
        except Exception as exc:
            cprint(f"[place] Twin request failed: {exc}", "yellow")
            twin.close()
            self._twins.pop(side, None)
            return {"value": False, "info": str(exc)}
        finally:
            if sock is not None and getattr(twin, "sock", None) is sock:
                sock.settimeout(old_timeout)

    def _try_placement_candidate(
        self, target_hand, rotation, current_js, approach_m=None,
        side="left", hand_to_end=None
    ):
        """Generate one sort-style Twin placement candidate."""
        target_hand = np.asarray(target_hand, dtype=float)
        final_hand = target_hand.copy()
        final_hand[:3, :3] = rotation
        if hand_to_end is None:
            _, hand_to_end = self._get_side_transforms(side)

        if approach_m is None:
            target_pose = [self._pose7(final_hand @ hand_to_end)]
        else:
            pre_hand = final_hand.copy()
            pre_hand[2, 3] += float(approach_m)
            target_pose = [
                self._pose7(pre_hand @ hand_to_end),
                self._pose7(final_hand @ hand_to_end),
            ]

        cnfg = {
            "target_pose": target_pose,
            "current_js": current_js,
            "xyz_threshold": float(
                self.config.shared.get("place_xyz_threshold_m", 0.02)
            ),
            "rpy_threshold": float(
                self.config.shared.get("place_rpy_threshold_rad", 0.15)
            ),
            "struct": self.config.get_arm_config(side).get(
                "twin_struct", f"{side}_arm"
            ),
        }
        if (
            self.config.sim_mode
            and self.config.shared.get("sim_suction", {}).get("enabled", False)
        ):
            suction_cfg = self.config.shared.get("sim_suction", {})
            cnfg["sim_suction"] = True
            cnfg["xyz_threshold"] = float(
                suction_cfg.get("twin_xyz_threshold_m", 0.08)
            )
            cnfg["rpy_threshold"] = float(
                suction_cfg.get("twin_rpy_threshold_rad", 0.15)
            )
        rsp = self._twin_generate(cnfg, side=side)
        if not isinstance(rsp, dict) or not rsp.get("value"):
            return None
        trajectory = rsp.get("info", {}).get("trajectory")
        if not trajectory:
            return None
        return np.asarray(copy.deepcopy(trajectory), dtype=float) / np.pi * 180.0

    def _reobserve_container_target(self, container_name, side, object_size_m=None):
        """Reacquire the container immediately before the final descent."""
        if not container_name:
            return None
        try:
            self.rgb, self.depth = self.get_camera_obs(side=side)
            self.save_current_transformation(side=side)
            target = self._get_placing_position(
                container_name, self.rgb, side=side,
                use_vlm_grounding=True, object_size_m=object_size_m,
            )
            if isinstance(target, np.ndarray) and target.shape == (4, 4):
                cprint("[place] near-place visual correction acquired a fresh target", "cyan")
                return target
        except Exception as exc:
            cprint(f"[place] near-place visual correction failed: {exc}", "yellow")
        return None

    def _placement_execution_offset_base_m(self, side):
        """Return the calibrated final placement offset in the arm base frame.

        Placement uses a separate empirical correction from grasp: X is
        shifted by -30 mm by default. A placement-specific shared override is
        supported without changing the grasp correction.
        """
        offset = self.config.shared.get("place_execution_offset_base_m")
        if offset is None:
            offset = self.config.get_grasp_scoring(side).get(
                "place_execution_offset_base_m", [-0.030, 0.0, 0.0]
            )
        try:
            values = np.asarray(offset, dtype=float).reshape(-1)
            if values.size != 3 or not np.all(np.isfinite(values)):
                raise ValueError
            return values
        except (TypeError, ValueError):
            cprint(
                f"[place] invalid placement calibration offset {offset!r}; using [-0.03, 0, 0]m",
                "yellow",
            )
            return np.array([-0.030, 0.0, 0.0], dtype=float)

    @staticmethod
    def _valid_box(box, width, height):
        try:
            values = [float(value) for value in box[:4]]
            x1, y1, x2, y2 = values
            if not all(np.isfinite(value) for value in values):
                return None
            x1, x2 = max(0.0, min(width, x1)), max(0.0, min(width, x2))
            y1, y2 = max(0.0, min(height, y1)), max(0.0, min(height, y2))
            return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None
        except (TypeError, ValueError, IndexError):
            return None

    def _grounding_box(self, image, class_name):
        """Return a VLM pixel box without making it a motion command."""
        try:
            result = self.vlm.ground_object(image, class_name)
            box = result.get("box") if isinstance(result, dict) else None
            return box if isinstance(box, (list, tuple)) and len(box) == 4 else None
        except Exception as exc:
            cprint(f"[place] post-release VLM grounding failed for '{class_name}': {exc}", "yellow")
            return None

    def _verify_object_in_container(self, object_name, container_name, side="left"):
        """Verify semantic object/container overlap after release.

        A positive result requires the released object's detected centre to be
        inside a conservative inner container region. If the object detector
        is unavailable, the method can only perform a degraded container
        observation check; that case is logged and controlled by config.
        """
        if not container_name:
            return False
        try:
            image, depth = self.get_camera_obs(side=side)
            self.rgb, self.depth = image, depth
            height, width = image.shape[:2]
            container_box = None
            try:
                detections = self.perception.detect_objects(image, [container_name], conf=0.20)
            except Exception as exc:
                cprint(f"[place] post-release container detection failed: {exc}", "yellow")
                detections = []
            if detections:
                container_box = self._valid_box(detections[0], width, height)
            if container_box is None:
                container_box = self._valid_box(self._grounding_box(image, container_name), width, height)
            if container_box is None:
                cprint("[place] post-release verification failed: container not visible", "red")
                return False

            if not object_name:
                cprint(
                    "[place] object name not supplied; only gripper/container release was verified "
                    "(pass object:'orange' for semantic verification)",
                    "yellow",
                )
                return not self.config.shared.get("place_require_object_verification", False)

            object_box = None
            try:
                object_detections = self.perception.detect_objects(image, [object_name], conf=0.15)
            except Exception as exc:
                cprint(f"[place] post-release object detection failed: {exc}", "yellow")
                object_detections = []
            if object_detections:
                object_box = self._valid_box(object_detections[0], width, height)
            if object_box is None:
                object_box = self._valid_box(self._grounding_box(image, object_name), width, height)
            if object_box is None:
                cprint(f"[place] post-release verification failed: '{object_name}' not detected", "red")
                return False

            x1, y1, x2, y2 = container_box
            ox = (object_box[0] + object_box[2]) / 2.0
            oy = (object_box[1] + object_box[3]) / 2.0
            margin_x = 0.10 * (x2 - x1)
            margin_y = 0.10 * (y2 - y1)
            inside_inner_box = x1 + margin_x <= ox <= x2 - margin_x and y1 + margin_y <= oy <= y2 - margin_y
            try:
                depth_m = self._robust_container_depth_m(int(x1), int(y1), int(x2), int(y2))
                region = self._segment_container_interior(image, container_box)
                pixel_inside_region = bool(
                    0 <= int(round(oy)) < region.shape[0]
                    and 0 <= int(round(ox)) < region.shape[1]
                    and region[int(round(oy)), int(round(ox))] > 0
                )
            except Exception:
                depth_m, pixel_inside_region = None, False
            verified = inside_inner_box and (pixel_inside_region or depth_m is not None)
            cprint(
                f"[place] post-release verification: object='{object_name}' "
                f"container='{container_name}' inside={verified} "
                f"center=({ox:.1f},{oy:.1f})",
                "green" if verified else "red",
            )
            return verified
        except Exception as exc:
            cprint(f"[place] post-release verification crashed: {exc}", "red")
            return False

    def _execute_placement(
        self, placement_pos_world, initial_js_key="grasp1", side="left",
        container_name=None, object_name=None, object_size_m=None
    ):
        """Plan, visually correct, release, and verify one placement.

        Twin is queried before every physical motion. The first motion only
        reaches a pre-place pose; a fresh RGB-D/VLM observation then corrects
        XYZ before the final descent. This prevents a stale camera transform
        or a small arm/object motion from turning a valid plan into a miss.
        """
        initial_target = np.asarray(placement_pos_world, dtype=float).copy()
        targets = getattr(self, "_placement_candidate_targets_world", None)
        if not targets:
            targets = [initial_target]
        placement_offset = self._placement_execution_offset_base_m(side)
        adjusted_targets = []
        for candidate in targets:
            adjusted = np.asarray(candidate, dtype=float).copy()
            adjusted[:3, 3] += placement_offset
            adjusted_targets.append(adjusted)
        targets = adjusted_targets
        cprint(
            f"[place] applying calibrated placement offset in {side} base frame: "
            f"{placement_offset.round(4).tolist()}m",
            "cyan",
        )

        start_pose = self.config.get_pose(initial_js_key, side=side)
        if not isinstance(start_pose, dict):
            cprint(f"[place] missing {side}-arm Twin start pose: {initial_js_key}", "red")
            return False
        current_js = [float(value) * np.pi / 180.0 for value in start_pose.values()]
        _, hand_to_end = self._get_side_transforms(side)

        clearance_m = float(self.config.shared.get("place_clearance_m", 0.05))
        approach_offsets = []
        configured_approach = float(
            self.config.shared.get("place_approach_m", 0.12)
        )
        for approach_m in (configured_approach, 0.08, 0.05):
            if approach_m >= 0.0 and not any(
                abs(approach_m - old) < 1e-6 for old in approach_offsets
            ):
                approach_offsets.append(approach_m)

        def rotations_for(target):
            # The first orientation is the AnyGrasp orientation. Yaw variants
            # are equivalent placement orientations and help Twin escape a
            # wrist-limit branch without imposing verticality by default.
            placement_rotation = target[:3, :3].copy()
            if self.config.shared.get("place_vertical_gripper", False):
                placement_rotation = self._vertical_gripper_rotation(placement_rotation)
                cprint(
                    f"[place] enforcing vertical gripper: down_axis="
                    f"{placement_rotation[:, 2].round(4).tolist()}", "cyan",
                )
            rotations = [placement_rotation]
            for angle_deg in self.config.shared.get(
                "place_yaw_variants_deg", [90.0, -90.0, 180.0]
            ):
                try:
                    rotation = placement_rotation @ R.from_euler(
                        "z", float(angle_deg) * np.pi / 180.0
                    ).as_matrix()
                except (TypeError, ValueError):
                    continue
                if not any(np.allclose(rotation, old) for old in rotations):
                    rotations.append(rotation)
            return rotations

        # Stage 1: reach a Twin-approved pre-place pose. Stage 2: reobserve
        # the container at close range and plan only the final descent.
        for target_index, raw_target in enumerate(targets):
            target = np.asarray(raw_target, dtype=float).copy()
            target[2, 3] += clearance_m
            rotations = rotations_for(target)
            for rotation in rotations:
                for approach_m in approach_offsets:
                    pre_target = target.copy()
                    pre_target[2, 3] += approach_m
                    pre_trajectory = self._try_placement_candidate(
                        pre_target, rotation, current_js, approach_m=None,
                        side=side, hand_to_end=hand_to_end
                    )
                    if pre_trajectory is None:
                        continue
                    if not self.control_arm(
                        trajectory=pre_trajectory, speed=20, side=side
                    ):
                        cprint(f"[place] {side}-arm pre-place trajectory execution failed", "red")
                        return False

                    corrected = self._reobserve_container_target(
                        container_name, side, object_size_m=object_size_m
                    )
                    if corrected is not None:
                        corrected = np.asarray(corrected, dtype=float).copy()
                        corrected[:3, 3] += placement_offset
                        reference = target[:3, 3].copy()
                        reference[2] -= clearance_m
                        correction_delta_m = float(
                            np.linalg.norm(np.asarray(corrected[:3, 3]) - reference)
                        )
                        max_correction_m = float(
                            self.config.shared.get(
                                "place_visual_correction_max_m", 0.08
                            )
                        )
                        if max_correction_m > 0.0 and correction_delta_m > max_correction_m:
                            cprint(
                                f"[place] rejecting visual correction {correction_delta_m:.3f}m "
                                f"> limit {max_correction_m:.3f}m; retaining original target",
                                "yellow",
                            )
                            corrected = None
                    final_target = target if corrected is None else np.asarray(corrected, dtype=float).copy()
                    final_target[2, 3] += clearance_m
                    if corrected is not None:
                        cprint(
                            f"[place] candidate {target_index}: applying fresh XYZ correction "
                            f"delta={(final_target[:3, 3] - target[:3, 3]).round(4).tolist()}",
                            "cyan",
                        )
                    pre_current_js = [
                        float(value) * np.pi / 180.0 for value in pre_trajectory[-1]
                    ]
                    final_trajectory = self._try_placement_candidate(
                        final_target, rotation, pre_current_js, approach_m=None,
                        side=side, hand_to_end=hand_to_end
                    )
                    if final_trajectory is None:
                        cprint("[place] fresh final target failed Twin; object remains held", "red")
                        return False
                    if not self.control_arm(
                        trajectory=final_trajectory, speed=20, side=side
                    ):
                        cprint(f"[place] {side}-arm final placement trajectory execution failed", "red")
                        return False
                    return self._release_and_verify(
                        side=side, object_name=object_name, container_name=container_name
                    )

        # If the pre-place waypoint is unreachable, try bounded XY offsets in
        # the same plate plane, matching sort's final-pose fallback.
        configured_offsets = self.config.shared.get(
            "place_target_offsets_m",
            [[0.0, 0.0], [0.02, 0.0], [-0.02, 0.0], [0.0, 0.02], [0.0, -0.02]],
        )
        offsets = []
        for offset in configured_offsets:
            try:
                if len(offset) != 2:
                    continue
                dx, dy = float(offset[0]), float(offset[1])
            except (TypeError, ValueError):
                continue
            if not any(abs(dx - ox) < 1e-6 and abs(dy - oy) < 1e-6 for ox, oy in offsets):
                offsets.append((dx, dy))

        for raw_target in targets:
            base_target = np.asarray(raw_target, dtype=float).copy()
            base_target[2, 3] += clearance_m
            for dx, dy in offsets:
                sampled_target = base_target.copy()
                sampled_target[0, 3] += dx
                sampled_target[1, 3] += dy
                for rotation in rotations_for(sampled_target):
                    trajectory = self._try_placement_candidate(
                        sampled_target, rotation, current_js, approach_m=None,
                        side=side, hand_to_end=hand_to_end
                    )
                    if trajectory is None:
                        continue
                    if not self.control_arm(trajectory=trajectory, speed=20, side=side):
                        cprint(f"[place] {side}-arm placement trajectory execution failed", "red")
                        return False
                    return self._release_and_verify(
                        side=side, object_name=object_name, container_name=container_name
                    )

        return False

    def _release_and_verify(self, side="left", object_name=None, container_name=None):
        """Open the gripper, verify opening, then verify object/container result."""
        time.sleep(0.5)
        response = self.control_hand(cmd_type="open", side=side)
        if not isinstance(response, dict) or response.get("value") is False:
            cprint(f"[place] {side} gripper open command failed", "red")
            return False
        time.sleep(0.4)
        try:
            if not self.gripper_for(side).is_fully_open():
                cprint(f"[place] {side} gripper did not confirm fully open", "red")
                return False
        except Exception as exc:
            cprint(f"[place] gripper release verification failed: {exc}", "red")
            return False
        return self._verify_object_in_container(object_name, container_name, side=side)
