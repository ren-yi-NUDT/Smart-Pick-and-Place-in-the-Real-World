"""Continuous Azure detection snapshots without robot/world-coordinate assumptions."""
import math
import threading
import time
import uuid


_IOU_MATCH = 0.3


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1, ix2, iy2 = max(ax1, bx1), max(ay1, by1), min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    return inter / (area_a + area_b - inter) if area_a + area_b - inter else 0.0


def validate_snapshot(snapshot, max_age=2.0, now=None):
    """Validate a local snapshot and reject stale or unsafe metadata."""
    if not isinstance(snapshot, dict):
        raise ValueError("snapshot must be an object")
    required = ("session_id", "frame_id", "captured_at", "width", "height", "objects")
    if any(key not in snapshot for key in required):
        raise ValueError("snapshot is missing metadata")
    now = time.monotonic() if now is None else now
    if not isinstance(snapshot["session_id"], str) or not snapshot["session_id"]:
        raise ValueError("invalid session_id")
    if not isinstance(snapshot["frame_id"], int) or isinstance(snapshot["frame_id"], bool):
        raise ValueError("invalid frame_id")
    try:
        age = now - float(snapshot["captured_at"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid snapshot timestamp") from exc
    if not math.isfinite(age) or age < 0 or age > max_age:
        raise ValueError("stale snapshot")
    if (not isinstance(snapshot["width"], int) or isinstance(snapshot["width"], bool)
            or not isinstance(snapshot["height"], int) or isinstance(snapshot["height"], bool)
            or snapshot["width"] <= 0 or snapshot["height"] <= 0):
        raise ValueError("invalid dimensions")
    if not isinstance(snapshot["objects"], list):
        raise ValueError("objects must be a list")
    target_ids = set()
    for obj in snapshot["objects"]:
        if not isinstance(obj, dict) or len(obj.get("bbox", ())) != 4:
            raise ValueError("invalid bbox")
        try:
            bbox = [float(value) for value in obj["bbox"]]
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid bbox") from exc
        if (not all(math.isfinite(value) for value in bbox)
                or bbox[0] > bbox[2] or bbox[1] > bbox[3]):
            raise ValueError("invalid bbox")
        if not isinstance(obj.get("target_id"), str) or not obj["target_id"]:
            raise ValueError("invalid target_id")
        if obj["target_id"] in target_ids:
            raise ValueError("duplicate target_id")
        target_ids.add(obj["target_id"])
        try:
            confidence = float(obj.get("confidence", -1))
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid confidence") from exc
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("invalid confidence")
    return snapshot


class VisionOwner:
    """Own capture/detection state; consumers only receive immutable copies."""

    def __init__(self, camera, detector, session_id=None):
        self.camera = camera
        self.detector = detector
        self.session_id = session_id or uuid.uuid4().hex
        self._previous = []
        self._previous_session = None
        self._latest = None
        self._lock = threading.Lock()

    def update(self):
        packet = self.camera.get_snapshot()
        image = packet["bgr"]
        # YOLO/Perception consumes RGB; camera packets remain BGR for recording/API.
        import cv2
        raw = self.detector(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        boxes = []
        for d in raw or []:
            try:
                if len(d) < 6:
                    continue
                bbox = [float(value) for value in d[:4]]
                confidence = float(d[4])
                class_id = int(d[5])
            except (TypeError, ValueError, OverflowError):
                continue
            if (not all(math.isfinite(value) for value in bbox)
                    or bbox[0] > bbox[2] or bbox[1] > bbox[3]
                    or not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0):
                continue
            boxes.append({
                "bbox": bbox, "confidence": confidence, "class_id": class_id,
                "confirmed": False,
            })
        if self._previous_session != packet["session_id"]:
            self._previous = []
            self._previous_session = packet["session_id"]

        # A pair is safe only when it is the sole qualifying same-class pair on
        # both sides; ambiguous overlap must never inherit an identity.
        candidates = {}
        reverse = {}
        for current_index, obj in enumerate(boxes):
            for old_index, old in enumerate(self._previous):
                if obj["class_id"] != old["class_id"]:
                    continue
                score = _iou(obj["bbox"], old["bbox"])
                if score >= _IOU_MATCH:
                    candidates.setdefault(current_index, []).append((old_index, score))
                    reverse.setdefault(old_index, []).append(current_index)
        for current_index, matches in candidates.items():
            if len(matches) != 1:
                continue
            old_index, _ = matches[0]
            if len(reverse[old_index]) == 1:
                boxes[current_index]["target_id"] = self._previous[old_index]["target_id"]

        camera_session = str(packet["session_id"])
        for index, obj in enumerate(boxes):
            obj.setdefault("target_id", f"{self.session_id}:{camera_session}:"
                           f"{packet['frame_id']}:{index}")
        frame = {
            "session_id": packet["session_id"],
            "frame_id": int(packet["frame_id"]),
            "captured_at": float(packet["captured_at"]),
            "width": int(image.shape[1]), "height": int(image.shape[0]),
            "objects": boxes,
        }
        with self._lock:
            self._previous = [dict(obj, session_id=packet["session_id"])
                              for obj in boxes]
            self._latest = _copy_snapshot(frame)
        return _copy_snapshot(frame)

    def snapshot(self, max_age=2.0):
        with self._lock:
            if self._latest is None:
                raise RuntimeError("no Azure vision snapshot")
            frame = _copy_snapshot(self._latest)
        validate_snapshot(frame, max_age=max_age)
        return frame


def _copy_snapshot(snapshot):
    return {
        **snapshot,
        "objects": [dict(obj, bbox=list(obj["bbox"])) for obj in snapshot["objects"]],
    }


def get_snapshot(owner, max_age=2.0):
    """Read a snapshot from an already-running owner; never opens hardware."""
    return owner.snapshot(max_age=max_age)
