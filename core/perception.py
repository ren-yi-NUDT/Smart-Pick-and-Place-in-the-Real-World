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

import cv2
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from PIL import Image
from termcolor import cprint
from ultralytics import YOLOWorld

from core.abc import BasePerception
from core.config import YOLO_CONF_GRASP, YOLO_CONF_CONTAINER
from core.transforms import graspcam2pixel


# 色相 -> 颜色名 (上界, 单位度)。换灯/换房间后要跟着 CAMERA_WHITE_BALANCE 重标。
# 0/360 两侧都是 pink: 实测 3500K 下粉盘子 354°、橙子 14°, 中间只隔 20°, 所以
# pink/orange 的界线定在 10° 而不是 15° (15° 会把橙子判成粉)。
_HUE_BINS_DEG = [
    (10, "pink"), (45, "orange"), (70, "yellow"), (170, "green"),
    (200, "cyan"), (270, "blue"), (330, "purple"), (360, "pink"),
]


def dominant_color_name(rgb, box, inset: float = 0.3):
    """框内主色的英文名；近中性时按明度退回 white/gray/black。

    颜色必须自己从像素算, 不能问 YOLO-World 的颜色词 —— 它的颜色绑定是从语言
    先验猜的, 跟像素无关。实测同一个蓝盘子: 问 "pink plate" 得 0.621, 问
    "blue plate" 只有 0.165。取框中心 inset 区域避开桌面背景。
    """
    x1, y1, x2, y2 = [int(v) for v in box[:4]]
    w, h = x2 - x1, y2 - y1
    x1, y1 = x1 + int(w * inset), y1 + int(h * inset)
    x2, y2 = x2 - int(w * inset), y2 - int(h * inset)
    if x2 <= x1 or y2 <= y1:
        return None

    crop = np.ascontiguousarray(rgb[y1:y2, x1:x2])
    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV).reshape(-1, 3).astype(float)
    hue = float(np.median(hsv[:, 0])) * 2.0   # OpenCV H 0-179 -> 0-360 度
    sat = float(np.median(hsv[:, 1])) / 255.0
    val = float(np.median(hsv[:, 2])) / 255.0

    if sat < 0.12:                            # 近中性, 色相是噪声
        return "white" if val > 0.70 else ("gray" if val > 0.30 else "black")
    return next(name for hi, name in _HUE_BINS_DEG if hue < hi)


def _import_anygrasp():
    """Lazy import -- anygrasp may not be installed in every environment."""
    from anygrasp_sdk.grasp_detection.anygrasp_get_poses import anygrasp_get_poses
    return anygrasp_get_poses


class Perception(BasePerception):
    """Unified perception front-end combining YOLO-World and AnyGrasp."""

    def __init__(
        self,
        yolo_model_path: str = "",
        anygrasp_checkpoint: str = "",
        save_path: str = "",
    ):
        self.save_path = save_path
        self.checkpoint_path = anygrasp_checkpoint

        # Load YOLO-World
        if yolo_model_path:
            self.yolo_model = YOLOWorld(yolo_model_path)
        else:
            self.yolo_model = None

        # AnyGrasp is loaded lazily (heavy)
        self._anygrasp_fn = None

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

    def detect_container(self, image, class_name, conf=YOLO_CONF_CONTAINER):
        """检测容器。*class_name* 可以带颜色词 (``"pink plate"``)。

        带颜色词时只把名词交给 YOLO, 再用框内像素的颜色挑 —— YOLO-World 的颜色
        词是语言先验, 不是从图里看出来的, 用它等于随机。找不到匹配颜色时返回空
        并告警, 而不是退回"随便给一个": 把东西放进错的盘子比放不进去更糟。
        """
        words = class_name.split()
        if len(words) < 2:
            return self.detect_objects(image, [class_name], conf=conf)

        want, noun = words[0], words[-1]
        detections = self.detect_objects(image, [noun], conf=conf)
        if not detections:
            return []

        hits = [d for d in detections if dominant_color_name(image, d) == want]
        if hits:
            return hits

        seen = {dominant_color_name(image, d) for d in detections}
        cprint(f"[Perception] 要 {want} {noun}, 但只看到 {seen}; 颜色对不上, 不猜", "yellow")
        return []

    # ------------------------------------------------------------------
    # AnyGrasp
    # ------------------------------------------------------------------
    def detect_grasps(self, rgb, depth, model: str = "rs_right"):
        """Run AnyGrasp on an RGB-D pair.

        Returns
        -------
        list[dict]
            Each dict has keys ``"trans"``, ``"score"``, ``"rotation_matrix"``.
        """
        if self._anygrasp_fn is None:
            self._anygrasp_fn = _import_anygrasp()
        try:
            anygrasp_pose, point_cloud = self._anygrasp_fn(
                self.checkpoint_path, rgb, depth, model=model,
            )
            self.point_cloud = point_cloud
            return anygrasp_pose
        except Exception as e:
            cprint(f"[Perception] AnyGrasp failed: {e}", "red")
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
        detections = self.detect_objects(image, class_name_list, conf=YOLO_CONF_GRASP)

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

        detections = self.detect_objects(image, [class_name], conf=YOLO_CONF_CONTAINER)

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
