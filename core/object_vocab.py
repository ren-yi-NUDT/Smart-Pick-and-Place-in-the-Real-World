"""中英物体词表：单一来源 + 运行期自动加名。

统一了原来三处各管各的词表：
- core/scene.py 的 SCAN_VOCAB（YOLO 扫描词，现场标定过的）
- run_nl.py 的 _OBJECT_CANDIDATES（jev 单选题候选）
- run_nl.py / core/jev_command.py 的 _GLOSS（中文兜底）

object_vocab.json 两节：
- "scan": YOLO 建图用的英文词（必须 YOLO-World 认得出，进这张表的都经过真机验证）
- "gloss": 中文名 -> 英文名（指令解析兜底；英文名不一定进 scan）

动态加名 add()：run_nl 从场景地图学到 "中文片段 -> 已检测物体名" 时落盘，
并原位扩展已加载的 scene.SCAN_VOCAB（长驻 run_nl 进程立即生效，不用重启）。
"""
import json
import os
import sys

_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "object_vocab.json")


def load():
    """返回 (scan 词表, gloss 字典)；文件缺失/损坏时给空表，不让启动挂掉。"""
    try:
        with open(_PATH) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return [], {}
    return list(data.get("scan", [])), dict(data.get("gloss", {}))


def add(zh, en):
    """学一个新词并落盘。en 来自场景地图 = YOLO 已验证，顺手进 scan。"""
    scan, gloss = load()
    if gloss.get(zh) == en:
        return False
    gloss[zh] = en
    if en not in scan:
        scan.append(en)
    with open(_PATH, "w") as f:
        json.dump({"scan": sorted(scan), "gloss": gloss},
                  f, ensure_ascii=False, indent=1, sort_keys=True)
    scene = sys.modules.get("core.scene")
    if scene is not None and en not in scene.SCAN_VOCAB:
        scene.SCAN_VOCAB.append(en)
    return True
