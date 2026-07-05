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
    ):
        self.save_path = save_path
        self.checkpoint_path = anygrasp_checkpoint
        self.anygrasp_host = anygrasp_host
        self.anygrasp_port = anygrasp_port

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
    def detect_grasps(self, rgb, depth, model: str = "rs_right"):
        """Run AnyGrasp on an RGB-D pair by calling the long-running server.

        Returns
        -------
        list[dict]
            Each dict has keys ``"trans"``, ``"score"``, ``"rotation_matrix"``.
        """
        if self._anygrasp_client is None:
            from core.anygrasp_client import AnyGraspClient
            self._anygrasp_client = AnyGraspClient(self.anygrasp_host, self.anygrasp_port)
            self._anygrasp_client.connect()
        try:
            return self._anygrasp_client.detect_grasps(rgb, depth, model=model)
        except Exception as e:
            cprint(f"[Perception] AnyGrasp server call failed: {e}", "red")
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
    ):
        """Filter grasp candidates by YOLO-World detection bounding boxes.

        Only grasps whose projected pixel lies inside (with a 20-px margin)
        a detection box are kept.

        Parameters
        ----------
        anygrasp_pose : list[dict]
            Output of :meth:`detect_grasps`.
        image : np.ndarray
            RGB image (used for projection and optional visualisation).
        class_name : str
            Comma-separated class names (same format as YOLO-World).
        return_label : bool
            If ``True``, attach ``"label"`` to each returned grasp dict.
        vis : bool
            Save a visualisation PNG when ``True``.

        Returns
        -------
        list[dict]
            Filtered grasp poses.
        """
        class_name_list = [cls for cls in class_name.split(",")]
        detections = self.detect_objects(image, class_name_list, conf=0.2)

        grasp_points, grasp_pose_cam = graspcam2pixel(anygrasp_pose)
        valid_indices = set()
        final_grasps = []
        valid_boxes = []
        ans = False

        if len(detections):
            det = detections[0][:4]
            x1, y1, x2, y2 = det
            valid_boxes.append((x1, y1, x2, y2))
            for i, grasp_p in enumerate(grasp_points):
                if (
                    grasp_p[0] > x1 - 20
                    and grasp_p[0] < x2 + 20
                    and grasp_p[1] > y1 - 20
                    and grasp_p[1] < y2 + 20
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
    def _visualise_filter(
        self, image, valid_boxes, grasp_points, valid_indices,
        final_grasps, class_name_list,
    ):
        """Save a visualisation image showing detections and valid grasps."""
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
