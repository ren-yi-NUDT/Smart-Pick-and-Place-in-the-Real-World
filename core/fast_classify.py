"""YOLO 框 + jev 快速物体类别判断。

流程：Azure 相机帧 → YOLO-World 用宽松上位词类名出框 → 每个框的 crop 交给
jev (POST 127.0.0.1:8791/v1/systemone) 在细粒度候选类别中做 choice 判断。
YOLO 负责定位（快），jev 负责精细分类（无生成、~0.5s）。
"""
import json
import time
import urllib.request
from urllib.error import URLError

JEV_URL = "http://127.0.0.1:8791/v1/systemone"

# YOLO 上位词 -> jev 候选细类。定位用粗类，分类用细类。
COARSE_CLASSES = ["fruit", "bottle", "cup", "toy", "snack"]
# 预览/分类用水果细类，用户后续自行增删
FINE_CANDIDATES = [
    "apple", "orange", "banana", "peach", "pear", "grape",
    "kiwi fruit", "lemon", "strawberry", "unknown object",
]
CONF_THRESHOLD = 0.4  # jev 置信度低于此值的结果直接丢弃


def jev_classify(crop_description, candidates=FINE_CANDIDATES, timeout=10.0):
    """让 jev 从 candidates 中选一个类别。返回 (类别, 概率, confidence)。"""
    request = {
        "state": f"A camera view cropped around one object. Observed: {crop_description}",
        "questions": {"class": {
            "type": "choice",
            "instructions": "Which object category best matches what is observed in the cropped view?",
            "criteria": {name: "" for name in candidates},
        }},
    }
    data = json.dumps(request).encode()
    req = urllib.request.Request(
        JEV_URL, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        answer = json.loads(resp.read())["answers"]["class"]
    return answer["choice"], answer["probabilities"][answer["choice"]], answer["confidence"]


def _crop_hint(frame_bgr, box, bgr=True):
    """从 crop 提取廉价视觉提示（颜色直方图主色 + 长宽比），喂给 jev 的 state。"""
    import cv2
    import numpy as np
    x1, y1, x2, y2 = [int(v) for v in box[:4]]
    crop = frame_bgr[max(y1, 0):y2, max(x1, 0):x2]
    if crop.size == 0:
        return "an object of unknown appearance"
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV if bgr else cv2.COLOR_RGB2HSV)
    mask = hsv[:, :, 1] > 60
    if mask.any():
        hue = int(np.median(hsv[:, :, 0][mask]))
        color = ["red", "orange", "yellow", "green", "cyan", "blue", "purple",
                 "pink", "red"][min(hue * 9 // 180, 8)]
    else:
        color = "gray or white"
    h, w = crop.shape[:2]
    shape = "tall and narrow" if h > 1.3 * w else ("flat" if w > 1.3 * h else "roughly round")
    return f"a {color} object, {shape}"


def classify_frame(frame_bgr, perception, coarse=COARSE_CLASSES, bgr=True):
    """一帧 → [{box, coarse, class, prob, confidence}, ...]。

    perception: core.perception.Perception 实例（YOLO-World 已加载）。
    bgr: 输入帧通道序（OpenCV 默认 BGR；ROS 图像为 RGB 时传 bgr=False）。
    """
    detections = perception.detect_objects(frame_bgr, coarse, conf=0.15)
    results = []
    for box in detections:
        coarse_name = coarse[int(box[5])] if isinstance(coarse[0], str) else box[5]
        hint = _crop_hint(frame_bgr, box, bgr=bgr)
        try:
            cls, prob, conf = jev_classify(hint, candidates)
        except (URLError, TimeoutError, KeyError) as e:
            # jev 不可用时退回 YOLO 粗类，不阻塞抓取流水线
            print(f"[fast_classify] jev fallback: {e}")
            cls, prob, conf = coarse_name, float(box[4]), float(box[4])
        if conf < CONF_THRESHOLD:
            continue
        results.append({"box": [round(v, 1) for v in box[:4]],
                        "coarse": coarse_name, "class": cls,
                        "prob": round(prob, 3), "confidence": round(conf, 3)})
    return results


def identify_scene_objects(perception, rgb):
    """一帧场景 → 实际存在的物体类别列表（去重，YOLO+jev）。"""
    return [r["class"] for r in classify_frame(rgb, perception, bgr=False)]


def main():
    from core.azure_camera import AzureCamera
    from core.perception import Perception
    from core.config import DEFAULT_YOLO_MODEL

    camera = AzureCamera()
    camera.start()
    try:
        time.sleep(1.0)
        frame = camera.get_frame()
        perception = Perception(yolo_model_path=DEFAULT_YOLO_MODEL)
        start = time.time()
        results = classify_frame(frame, perception)
        print(json.dumps(results, ensure_ascii=False, indent=2))
        print(f"[fast_classify] {len(results)} objects in {time.time()-start:.2f}s")
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
