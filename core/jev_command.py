"""自然语言指令 → YOLO 粗框 → jev 选目标 → grasp dry-run。

链路：用户说"把水果拿给我" → YOLO 出框 → 一次 jev choice 选哪个框是目标
→ 目标框 crop 再 jev 细分类 → 生成 grasp(object=细类) 调用（dry-run 只打印，
不真抓）。

用法：
    python3 -m core.jev_command "把苹果拿给我"          # dry-run
    python3 -m core.jev_command "把苹果拿给我" --exec   # 实机执行（暂未启用）
"""
import json
import sys
import time

from .fast_classify import COARSE_CLASSES, FINE_CANDIDATES, jev_classify, _crop_hint

# jev 选框阶段的 YOLO 候选：比 COARSE_CLASSES 更全（含具体水果），提高目标召回
SELECT_CLASSES = ["fruit", "apple", "orange", "banana", "toy", "bottle",
                  "cup", "snack", "vegetable"]
SELECT_CONF = 0.35  # 逐框 yes/no 判定的最低置信度

# 指令中文词 -> 英文目标词。jev 候选必须纯英文（中文内嵌会破坏打分）。
_GLOSS = {
    "苹果": "apple", "橙子": "orange", "橘子": "orange", "香蕉": "banana",
    "桃子": "peach", "梨": "pear", "葡萄": "grape", "柠檬": "lemon",
    "草莓": "strawberry", "猕猴桃": "kiwi fruit", "水果": "fruit",
    "瓶子": "bottle", "瓶": "bottle", "杯子": "cup", "杯": "cup",
    "玩具": "toy", "零食": "snack", "蔬菜": "vegetable",
}


def _english_target(instruction):
    """把指令归一成英文目标词：先查对照表，再取词级候选里能映射的词。"""
    for zh, en in _GLOSS.items():
        if zh in instruction:
            return en
    for w in _extract_target_words(instruction):
        for zh, en in _GLOSS.items():
            if zh in w:
                return en
    return ""


def _extract_target_words(instruction):
    """从指令提取可能的目标名词，用于动态扩充 YOLO 候选。

    jev 逐词判断"这个词是用户想要的东西吗"，取 conf 达标的词。
    """
    words = [w.strip("，。！？、的把给那个这个我要拿") for w in instruction.split()
             if len(w.strip("，。！？、的把给那个这个我要拿")) >= 2]
    # 中文无分词：整句 + 去掉请求动词后的短语都作为候选
    phrases = [instruction.strip("，。！？ ")]
    for verb in ["给我", "拿给我", "我要", "拿走", "抓起", "拿起", "把"]:
        for p in phrases[:]:
            if verb in p:
                phrases.append(p.replace(verb, " ").strip())
    candidates = list(dict.fromkeys(words + phrases))
    if not candidates:
        return []
    state = f'User instruction to a robot: "{instruction}". Which of these is the object the user wants to grab?'
    try:
        choice, prob, conf = jev_classify(state, candidates=candidates)
    except Exception as e:
        print(f"[jev_command] word extraction failed: {e}")
        return []
    print(f"[jev_command] target word: '{choice}' (conf {conf:.2f})")
    return [choice] if conf >= SELECT_CONF else []


def _boxes_to_state(box, hint, index, coarse=""):
    x1, y1, x2, y2 = [int(v) for v in box[:4]]
    w, h = x2 - x1, y2 - y1
    cx, cy = x1 + w // 2, y1 + h // 2
    pos = f"{'left' if cx < 640 else ('middle' if cx < 1280 else 'right')} of view, {'upper' if cy < 540 else 'lower'} half"
    return (f"object {index} (detector says: {coarse}): position {pos}, "
            f"size {w}x{h} px, appearance: {hint}")


def select_target_box(frame, perception, instruction):
    """YOLO 粗框 + 逐框 jev yes/no 打分选出符合指令的目标。

    YOLO 候选默认 SELECT_CLASSES；指令里出现新目标词时动态加进候选重检。
    返回 (框索引, 检测列表)；无框或全部低置信时返回 (None, 检测列表)。
    """
    classes = list(SELECT_CLASSES)
    target = _english_target(instruction)
    detections = []
    for attempt in range(2):  # 第二轮 = 加了指令关键词后重检
        detections = perception.detect_objects(frame, classes, conf=0.1)
        if detections:
            break
        if attempt == 0:
            extra = _extract_target_words(instruction)
            if not extra:
                break
            print(f"[jev_command] adding instruction words to YOLO classes: {extra}")
            classes = list(SELECT_CLASSES) + [e for e in extra if e not in SELECT_CLASSES]
        # 无框且无新词 -> 跳出
    if not detections:
        return None, [], classes

    gloss = f' ({target})' if target else ''
    best_idx, best_conf = None, 0.0
    for i, box in enumerate(detections):
        ci = int(box[5])
        coarse = classes[ci] if ci < len(classes) else str(box[5])
        hint = _crop_hint(frame, box, bgr=False)
        state = f'A camera sees {coarse}: {hint}. User request: "{instruction}"{gloss}.'
        try:
            # 候选必须纯英文：中文指令内嵌进候选会破坏 jev 打分（实测 yes 侧 conf 归零）
            yes_phrase = f"yes, this is the {target} the user asked for" if target \
                else "yes, this is what the user asked for"
            choice, prob, conf = jev_classify(
                state, candidates=[yes_phrase,
                                   "no, this is not what the user asked for"])
        except Exception as e:
            print(f"[jev_command] box{i} jev failed: {e}")
            continue
        yes = choice.startswith("yes")
        score = conf if yes else -conf
        print(f"[jev_command] box{i} ({coarse}, {hint}): "
              f"{'YES' if yes else 'no'} conf {conf:.2f}")
        if yes and conf > best_conf:
            best_conf, best_idx = conf, i

    if best_idx is None or best_conf < SELECT_CONF:
        return None, [], classes
    return best_idx, detections, classes


def command_to_grasp(instruction, perception, frame, dry_run=True):
    """指令 → grasp 调用参数。dry_run=True 时只打印，不发实机命令。"""
    t0 = time.time()
    idx, dets, classes = select_target_box(frame, perception, instruction)
    if idx is None:
        print(f"[jev_command] no confident target for: {instruction}")
        return None
    box = dets[idx]
    ci = int(box[5])
    coarse = classes[ci] if ci < len(classes) else str(box[5])
    # YOLO 命中具体细类（apple/orange/banana）时直接用，否则退回 jev 细分类
    if coarse in FINE_CANDIDATES and coarse != "unknown object":
        obj_cls, prob, conf = coarse, float(box[4]), float(box[4])
        print(f"[jev_command] using YOLO fine class '{obj_cls}' (conf {conf:.2f})")
    else:
        hint = _crop_hint(frame, box, bgr=False)
        obj_cls, prob, conf = jev_classify(hint, FINE_CANDIDATES)
    elapsed = time.time() - t0
    print(f"[jev_command] target box {idx} @ {box[:4]} -> fine class '{obj_cls}' "
          f"(prob {prob:.2f}, conf {conf:.2f}, {elapsed:.1f}s total)")

    cmd = {"object": obj_cls, "container": "person"}
    if dry_run:
        print(f"[jev_command] DRY-RUN would call: "
              f"echo '{json.dumps(cmd, ensure_ascii=False)}' | python run_skill.py grasp")
        return cmd
    # 实机执行：导入 grasp skill 跑 run()（需要 ROS/Twin 全链路在线）
    from skills.grasp import GraspSkill
    skill = GraspSkill()
    ok = skill.run(**cmd)
    print(f"[jev_command] grasp result: {ok}")
    return cmd


def main():
    import argparse
    import numpy as np
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("instruction", help="自然语言指令，如 '把水果拿给我'")
    parser.add_argument("--exec", dest="do_exec", action="store_true",
                        help="实机执行（默认 dry-run）")
    parser.add_argument("--preview", action="store_true", help="保存标注图到 /tmp")
    args = parser.parse_args()

    from core.azure_camera import AzureCamera
    from core.perception import Perception
    from core.config import DEFAULT_YOLO_MODEL

    cam = AzureCamera()
    cam.start()
    try:
        time.sleep(1)
        frame = cam.get_frame(max_age=2.0)
        perception = Perception(yolo_model_path=DEFAULT_YOLO_MODEL)
        command_to_grasp(args.instruction, perception, frame, dry_run=not args.do_exec)
        if args.preview:
            import cv2
            idx, dets, _ = select_target_box(frame, perception, args.instruction)
            vis = frame.copy()
            for i, box in enumerate(dets):
                x1, y1, x2, y2 = [int(v) for v in box[:4]]
                color = (0, 255, 0) if i == idx else (128, 128, 128)
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                cv2.putText(vis, f"box{i}", (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            path = "/tmp/jev_command_preview.png"
            cv2.imwrite(path, vis)
            print(f"[jev_command] preview saved: {path}")
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
