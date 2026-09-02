"""
Perception wrappers for YOLO-World object detection and AnyGrasp grasp
detection.

Usage:
    from core.perception import Perception
    perc = Perception(
        yolo_model_path="/path/to/yolov8x-worldv2.pt",
        anygrasp_checkpoint="/path/to/checkpoint_detection.tar",
    )
    detections = perc.detect_objects(rgb, class_names=["orange"])
    grasps = perc.detect_grasps(rgb, depth)
    filtered = perc.filter_grasps_by_detection(grasps, detections, rgb, class_name="orange")
"""

import os

import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from PIL import Image
from termcolor import cprint
from ultralytics import YOLOWorld

from core.transforms import graspcam2pixel


def _import_anygrasp():
    """Lazy import -- anygrasp may not be installed in every environment."""
    from anygrasp_sdk.grasp_detection.anygrasp_get_poses import anygrasp_get_poses
    return anygrasp_get_poses


class Perception:
    """Unified perception front-end combining YOLO-World and AnyGrasp."""

    def __init__(
        self,
        yolo_model_path: str = "",
        anygrasp_checkpoint: str = "",
        save_path: str = "",
        anygrasp_host: str = "127.0.0.1",
        anygrasp_port: int = 8030,
        camera_intrinsics: dict = None,
    ):
        self.save_path = save_path
        self.checkpoint_path = anygrasp_checkpoint
        self.anygrasp_host = anygrasp_host
        self.anygrasp_port = anygrasp_port
        self.camera_intrinsics = dict(camera_intrinsics or {})

        # Load YOLO-World
        if yolo_model_path:
            self.yolo_model = YOLOWorld(yolo_model_path)
        else:
            self.yolo_model = None

        # AnyGrasp runs in a long-running server process; client is lazy.
        self._anygrasp_client = None

    # ------------------------------------------------------------------
    # YOLO-World
    # ------------------------------------------------------------------
    def detect_objects(self, image, class_names, conf=0.2):
        """Run YOLO-World on *image* for the given *class_names*.

        Parameters
        ----------
        image : np.ndarray or PIL.Image
            RGB image.
        class_names : str or list[str]
            Category names (comma-separated string or list).

        Returns
        -------
        list[list[float]]
            Detection boxes as ``[[x1, y1, x2, y2, conf, cls], ...]``.
        """
        if isinstance(class_names, str):
            class_names = [c.strip() for c in class_names.split(",")]

        self.yolo_model.set_classes(class_names)

        if isinstance(image, np.ndarray):
            pil_image = Image.fromarray(image)
        else:
            pil_image = image

        results = self.yolo_model.predict(source=pil_image, conf=conf)
        if len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
            return []
        return results[0].boxes.data.tolist()

    # ------------------------------------------------------------------
    # AnyGrasp
    # ------------------------------------------------------------------
    def detect_grasps(
        self, rgb, depth, model: str = None, side: str = None,
        intrinsics: dict = None, depth_scale: float = None,
    ):
        """Run AnyGrasp on an RGB-D pair by calling the long-running server.

        Returns
        -------
        list[dict]
            Each dict has ``"trans"``, ``"score"`` and
            ``"rotation_matrix"``; the server may also return the official
            gripper dimensions ``"width"``, ``"height"`` and ``"depth"``.
        """
        if model is None:
            model = f"rs_{side or 'right'}"
        try:
            if (
                self._anygrasp_client is None
                or getattr(self._anygrasp_client, "sock", None) is None
            ):
                from core.anygrasp_client import AnyGraspClient
                self._anygrasp_client = AnyGraspClient(
                    self.anygrasp_host, self.anygrasp_port
                )
                if not self._anygrasp_client.connect():
                    raise ConnectionError("AnyGrasp client connection failed")
            return self._anygrasp_client.detect_grasps(
                rgb, depth, model=model, intrinsics=intrinsics,
                depth_scale=depth_scale,
            )
        except Exception as e:
            cprint(f"[Perception] AnyGrasp server call failed: {e}", "red")
            client = self._anygrasp_client
            self._anygrasp_client = None
            if client is not None:
                client.close()
            return []

    # ------------------------------------------------------------------
    # Combined filtering  (from Planner.filtering_pose)
    # ------------------------------------------------------------------
    def filter_grasps_by_detection(
        self,
        anygrasp_pose,
        image,
        class_name: str = "",
        return_label: bool = False,
        vis: bool = True,
        side: str = "right",
        intrinsics: dict = None,
        target_box: list = None,
    ):
        """Filter grasp candidates by YOLO-World detection bounding boxes.

        Only grasps whose projected pixel lies strictly inside a detection
        box are kept; the detection box is not expanded.  ``target_box``
        (VLM-grounded) is used only when YOLO-World returns no detection.

        Parameters
        ----------
        anygrasp_pose : list[dict]
            Output of :meth:`detect_grasps`.
        image : np.ndarray
            RGB image (used for projection and optional visualisation).
        class_name : str or list[str]
            Comma-separated class names or a list of YOLO-World prompts.
        return_label : bool
            If ``True``, attach ``"label"`` to each returned grasp dict.
        vis : bool
            Save a visualisation PNG when ``True``.

        Returns
        -------
        list[dict]
            Filtered grasp poses.
        """
        if isinstance(class_name, str):
            class_name_list = [cls.strip() for cls in class_name.split(",") if cls.strip()]
        else:
            class_name_list = [str(cls).strip() for cls in class_name if str(cls).strip()]

        # YOLO-World first; the VLM box is only a fallback when YOLO finds
        # nothing (e.g. YOLO misses the object but the VLM grounded it).
        detections = self.detect_objects(image, class_name_list, conf=0.2)
        vlm_box = None
        if not len(detections) and target_box is not None:
            try:
                candidate_box = [float(value) for value in target_box]
                if len(candidate_box) == 4:
                    x1, y1, x2, y2 = candidate_box
                    if x2 > x1 and y2 > y1:
                        vlm_box = candidate_box
            except (TypeError, ValueError):
                vlm_box = None

        grasp_points, grasp_pose_cam = graspcam2pixel(
            anygrasp_pose, cam_type=side, intrinsics=intrinsics
        )
        valid_indices = set()
        final_grasps = []
        valid_boxes = []
        ans = False

        boxes = [detections[0][:4]] if len(detections) else (
            [vlm_box] if vlm_box is not None else []
        )
        if boxes:
            for det in boxes:
                x1, y1, x2, y2 = det
                valid_boxes.append((x1, y1, x2, y2))
                for i, grasp_p in enumerate(grasp_points):
                    if (
                        grasp_p[0] > x1
                        and grasp_p[0] < x2
                        and grasp_p[1] > y1
                        and grasp_p[1] < y2
                    ):
                        valid_indices.add(i)

            if len(valid_indices):
                sorted_indices = sorted(list(valid_indices))
                for i in sorted_indices:
                    g_pose = grasp_pose_cam[i]
                    if return_label:
                        if isinstance(g_pose, dict):
                            g_pose["label"] = class_name_list
                    final_grasps.append(g_pose)
                cprint(
                    f"*********** Class name {class_name_list} ****************** "
                    f"Grasp pose number: {len(final_grasps)} ******************",
                    "red",
                )
                ans = True
            else:
                cprint(
                    f"Found objects ({class_name_list}) but NO grasp points inside them.",
                    "yellow",
                )
        else:
            cprint(f"No object detected for class: {class_name_list}", "yellow")

        if vis:
            self._visualise_filter(
                image, valid_boxes, grasp_points, valid_indices,
                final_grasps, class_name_list,
                grasp_pose_cam=grasp_pose_cam, intrinsics=intrinsics,
            )

        return final_grasps if ans else []

    # ------------------------------------------------------------------
    # Placement detection  (from Planner.get_placing_position)
    # ------------------------------------------------------------------
    def detect_placement_position(
        self,
        class_name: str,
        image,
        depth,
        cam_type: str = "right",
        vis: bool = False,
    ):
        """Detect the 3-D world-frame position of a container for placement.

        Parameters
        ----------
        class_name : str
            Container class name for YOLO-World.
        image : np.ndarray
            RGB image.
        depth : np.ndarray
            Depth image (uint16, mm).
        cam_type : str
            ``"right"`` or ``"left"``.

        Returns
        -------
        list  or  np.ndarray (4, 4)
            Placement pose in world frame, or ``[]`` on failure.
        """
        from core.transforms import pixel_to_camera_point2

        detections = self.detect_objects(image, [class_name], conf=0.25)

        try:
            if len(detections):
                det = detections[0][:4]
                if det[1] >= 400 and det[3] <= 480 and len(detections) > 1:
                    det = detections[1][:4]
                x1, y1, x2, y2 = [int(coord) for coord in det]

                H, W = depth.shape
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(W, x2)
                y2 = min(H, y2)

                depth_sub_image_mm = depth[y1:y2, x1:x2]
                valid_depths_mm = depth_sub_image_mm[depth_sub_image_mm > 0]

                if len(valid_depths_mm) > 0:
                    mean_depth_mm = np.median(valid_depths_mm)
                else:
                    print("Warning: No valid depth values found in the bounding box.")
                    return []

                mean_depth_m = mean_depth_mm * 1e-3

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_cam_point = pixel_to_camera_point2(
                    np.array([center_x, center_y]).reshape(-1, 2),
                    mean_depth_m,
                    cam_type=cam_type,
                    intrinsics=self.camera_intrinsics.get(cam_type),
                )
                center_cam_point = center_cam_point.flatten()

                # Return raw camera-frame point; caller transforms to world
                return center_cam_point

        except Exception as e:
            cprint(f"[Perception] Placement detection failed: {e}", "red")

        return []

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _normalise_gripper_dimension(value, default):
        """Return a positive dimension in metres.

        AnyGrasp's ``Grasp`` stores dimensions in metres.  The millimetre
        fallback keeps the visualiser tolerant of older/custom server
        responses while retaining the SDK's native convention.
        """
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = float(default)
        if not np.isfinite(value) or value <= 0:
            value = float(default)
        if value > 1.0:
            value *= 1e-3
        return value

    @classmethod
    def _official_gripper_components(cls, grasp):
        """Build the same four cuboids as AnyGrasp's official renderer.

        ``graspnetAPI.utils.utils.plot_gripper_pro_max`` creates two fingers,
        a bottom bar and a tail in the grasp-local frame, then applies the
        grasp rotation and translation.  Returning cuboid vertices here lets
        the RGB logger use that exact geometry without importing Open3D in the
        normal robot process.
        """
        center = np.asarray(grasp["trans"], dtype=float).reshape(3)
        rotation = np.asarray(grasp["rotation_matrix"], dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(center)) or not np.all(np.isfinite(rotation)):
            raise ValueError("grasp pose contains non-finite values")

        width = cls._normalise_gripper_dimension(
            grasp.get("width", grasp.get("gripper_width")), 0.05
        )
        depth = cls._normalise_gripper_dimension(grasp.get("depth"), 0.02)

        # These constants intentionally match plot_gripper_pro_max().
        height = 0.004
        finger_width = 0.004
        tail_length = 0.04
        depth_base = 0.02

        def cuboid(length, breadth, thickness, offset):
            vertices = np.array([
                [0, 0, 0], [length, 0, 0], [0, 0, thickness],
                [length, 0, thickness], [0, breadth, 0],
                [length, breadth, 0], [0, breadth, thickness],
                [length, breadth, thickness],
            ], dtype=float)
            vertices += np.asarray(offset, dtype=float)
            return np.dot(rotation, vertices.T).T + center

        components = [
            cuboid(
                depth + depth_base + finger_width,
                finger_width,
                height,
                (-depth_base - finger_width, -width / 2 - finger_width, -height / 2),
            ),
            cuboid(
                depth + depth_base + finger_width,
                finger_width,
                height,
                (-depth_base - finger_width, width / 2, -height / 2),
            ),
            cuboid(
                finger_width,
                width,
                height,
                (-finger_width - depth_base, -width / 2, -height / 2),
            ),
            cuboid(
                tail_length,
                finger_width,
                height,
                (-tail_length - finger_width - depth_base, -finger_width / 2, -height / 2),
            ),
        ]
        return components, center, rotation, width

    @staticmethod
    def _project_camera_points(points, intrinsics):
        """Project camera-frame points using the same pinhole model as grasp points."""
        if not isinstance(intrinsics, dict):
            raise ValueError("camera intrinsics are required for gripper rendering")
        fx = float(intrinsics["fx"])
        fy = float(intrinsics["fy"])
        cx = float(intrinsics["cx"])
        cy = float(intrinsics["cy"])
        points = np.asarray(points, dtype=float)
        valid = np.all(np.isfinite(points), axis=1) & (points[:, 2] > 1e-6)
        pixels = np.full((len(points), 2), np.nan, dtype=float)
        pixels[valid, 0] = fx * points[valid, 0] / points[valid, 2] + cx
        pixels[valid, 1] = fy * points[valid, 1] / points[valid, 2] + cy
        return pixels, valid

    @classmethod
    def _draw_gripper(cls, ax, grasp, intrinsics, label):
        """Draw an AnyGrasp gripper pose on a Matplotlib RGB image."""
        # The official renderer maps low score to blue and high score to red.
        score = float(grasp.get("score", 0.0))
        score = min(1.0, max(0.0, score))
        color = (score, 0.05, 1.0 - score)
        components, center, rotation, width = cls._official_gripper_components(grasp)
        edges = (
            (0, 1), (1, 3), (3, 2), (2, 0),
            (4, 5), (5, 7), (7, 6), (6, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        faces = (
            (0, 1, 3, 2), (4, 5, 7, 6),
            (0, 1, 5, 4), (2, 3, 7, 6),
            (0, 2, 6, 4), (1, 3, 7, 5),
        )

        for vertices in components:
            pixels, valid = cls._project_camera_points(vertices, intrinsics)
            for face in faces:
                face_pixels = pixels[list(face)]
                if np.all(valid[list(face)]):
                    ax.fill(
                        face_pixels[:, 0], face_pixels[:, 1],
                        color=color, alpha=0.12, linewidth=0,
                    )
            for start, end in edges:
                if valid[start] and valid[end]:
                    ax.plot(
                        pixels[[start, end], 0], pixels[[start, end], 1],
                        color=color, linewidth=1.4, alpha=0.95,
                    )

        center_pixel, center_valid = cls._project_camera_points(
            center.reshape(1, 3), intrinsics
        )
        if center_valid[0]:
            u, v = center_pixel[0]
            # Local axes make the orientation explicit, while the gripper
            # outline conveys the opening direction and approach depth.
            axis_length = 0.035
            axis_points = np.vstack([
                center,
                center + rotation[:, 0] * axis_length,
                center + rotation[:, 1] * axis_length,
                center + rotation[:, 2] * axis_length,
            ])
            axis_pixels, axis_valid = cls._project_camera_points(axis_points, intrinsics)
            for axis_index, axis_color in enumerate(("r", "g", "b"), start=1):
                if axis_valid[0] and axis_valid[axis_index]:
                    ax.plot(
                        axis_pixels[[0, axis_index], 0],
                        axis_pixels[[0, axis_index], 1],
                        color=axis_color, linewidth=0.9, alpha=0.8,
                    )
            ax.text(
                u + 4, v - 4,
                f"G{label} {score:.2f} {width * 1000:.0f}mm",
                color=color, fontsize=6, weight="bold",
                bbox={"facecolor": "black", "alpha": 0.35, "pad": 1},
            )

    def _visualise_filter(
        self, image, valid_boxes, grasp_points, valid_indices,
        final_grasps, class_name_list, grasp_pose_cam=None, intrinsics=None,
    ):
        """Save detections, grasp points and projected AnyGrasp grippers."""
        plt.figure(figsize=(10, 8))
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plt.imshow(image)
        ax = plt.gca()

        for box in valid_boxes:
            x1, y1, x2, y2 = box
            ax.add_patch(plt.Rectangle(
                (x1, y1), x2 - x1, y2 - y1,
                fill=False, color="red", linewidth=2,
            ))
            ax.text(x1, y1 - 5, str(class_name_list), color="red", fontsize=10, weight="bold")

        # Render only grasps that survived the YOLO/VLM box filter.  This
        # mirrors the old green-star selection and avoids covering the whole
        # image with the 50 unfiltered AnyGrasp candidates.
        if grasp_pose_cam is not None and intrinsics is not None:
            for idx in sorted(valid_indices or []):
                if idx >= len(grasp_pose_cam):
                    continue
                try:
                    self._draw_gripper(ax, grasp_pose_cam[idx], intrinsics, idx)
                except (KeyError, TypeError, ValueError, IndexError) as exc:
                    cprint(f"[Perception] skip invalid gripper visualisation {idx}: {exc}", "yellow")

        if final_grasps:
            for idx in valid_indices:
                plt.plot(grasp_points[idx][0], grasp_points[idx][1], "g*", markersize=8)
        else:
            if len(grasp_points) > 0:
                plt.plot(grasp_points[:, 0], grasp_points[:, 1], "b.", markersize=6, alpha=0.5)

        plt.title(f"{class_name_list}: {len(valid_boxes)} objects, {len(final_grasps)} grasps")
        plt.axis("off")
        save_filename = f"filtered_rgb_{timestamp}_{class_name_list}.png"
        if self.save_path:
            plt.savefig(os.path.join(self.save_path, save_filename))
        plt.close()
