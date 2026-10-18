#!/usr/bin/env python3
"""自然语言指令 -> jev 解析 -> 直接执行 skill。交互式 REPL。

    python run_nl.py                        # 交互模式，直接打中文
    python run_nl.py "把黄瓜放进粉色盘子"      # 单次执行
    python run_nl.py --dry "把橙子递给我"     # 只解析，不动机械臂

分工：**jev 抽目标物体**（实测 conf 0.83~0.92，中文名词抽得很准），
**容器/动作走关键词** —— jev 对"递给我"这类只有 0.24~0.44 置信度，而关键词是确定的。
jev 服务没起时物体也退回关键词表。

词表单源在 object_vocab.json（见 core/object_vocab.py）：scan 节是 YOLO 建图词，
gloss 节是中文兜底。指令里出现词表外的物体名时，向 jev 问"桌上哪个已检测物体
匹配它"，置信度够就自动落盘学习（auto-add），下次直接认得。

需要 Open-Jev 在 127.0.0.1:8791 上跑着（见 Open-Jev/README.md）：
    cd Open-Jev && source .venv/bin/activate
    python -m jev.server --checkpoint models/Open-Jev-2B/package/checkpoint --max-length 4096
"""
import argparse
import json
import os
import re
import sys
import time
import urllib.request

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
os.chdir(PROJECT_ROOT)
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "dependence"))

from core import object_vocab  # noqa: E402  (单源词表: object_vocab.json)

JEV_URL = "http://127.0.0.1:8791/v1/systemone"

# 容器/动作关键词。顺序有意义：先匹配的先赢。
_CONTAINER_KEYWORDS = [
    (("垃圾桶", "垃圾", "扔", "丢掉", "扔了"), "trash"),
    (("桌上", "桌子", "桌面", "放下", "放桌"), "desk"),
    (("盘子", "碗", "碟子", "碟"), "plate"),
    (("给我", "递给我", "拿给我", "交给我", "拿过来", "递过来", "递来"), "person"),
]

_COLOR = {"粉": "pink", "红": "red", "蓝": "blue", "白": "white", "绿": "green",
          "黄": "yellow", "黑": "black", "灰": "gray", "紫": "purple"}
_NOUN = {"盘子": "plate", "碗": "bowl", "碟子": "plate", "碟": "plate"}

# 学习模式抠未知中文片段时剔除的虚词/动作词（贪心去除后剩下的 CJK 连串当物体名）
_STOPWORDS = [
    "垃圾桶", "垃圾", "丢掉", "放到", "放进", "放在", "扔到", "扔进", "扔了", "扔",
    "桌上", "桌子", "桌面", "放下", "放桌", "递给", "拿给", "交给我", "拿过来",
    "递过来", "递来", "给我", "盘子", "碟子", "全部", "一起", "还有", "里面",
    "里", "上", "去", "中", "把", "个", "只", "和", "都", "的", "色",
    "蓝", "粉", "红", "白", "绿", "黄", "黑", "灰", "紫",
]

_LOOK_WORDS = ("看看", "看一下", "环视", "看看桌上", "有什么", "扫描")
_FETCH_WORDS = ("接过来", "从我这拿", "递给我的手上", "拿走我手里")

_SEQ_CONNECTORS = re.compile(r"然后|接着|再|最后|之后")
_ACTION_HINTS = ("放", "抓", "递", "扔", "接", "拿", "看看", "扫描")


def split_sequence(text):
    """一条指令 -> 动作序列（段的列表）。

    "然后/再/接着/最后/之后"是动作边界；逗号只在**两边都是完整动作句**时才拆
    （"把苹果放进盘子，把香蕉递给我"拆两步，"把苹果，柠檬放进盘子"保持一步）。
    """
    def _split_commas(parts):
        out = []
        for p in parts:
            subs = [s for s in re.split(r"[，,。;；]", p) if s.strip()]
            if len(subs) > 1 and all(any(w in s for w in _ACTION_HINTS) for s in subs):
                out.extend(subs)
            else:
                out.append(p)
        return out

    return _split_commas(_SEQ_CONNECTORS.split(text))

_SCENE_MAP_TTL = 600  # 与 pick_and_place 的地图过期时间一致


def _unknown_fragment(instruction, gloss):
    """没匹配上词表时抠出可能是物体名的中文片段（贪心，认不准总比卡死强）。"""
    text = instruction
    for w in sorted(set(list(gloss) + _STOPWORDS), key=len, reverse=True):
        text = text.replace(w, " ")
    runs = re.findall(r"[一-鿿]{2,}", text)
    return runs[0] if runs else None


def _jev(questions, state, timeout=15.0):
    """问 jev，返回 {qid: (choice, confidence)}；服务不可用时抛异常。"""
    req = urllib.request.Request(
        JEV_URL, data=json.dumps({"state": state, "questions": questions}).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        answers = json.loads(resp.read())["answers"]
    return {k: (v["choice"], float(v["confidence"])) for k, v in answers.items()}


def load_scene_names(path="./log/scene.json", ttl=_SCENE_MAP_TTL):
    """读场景地图里的物体名；过期/缺失/损坏返回空表（学习模式跳过）。"""
    try:
        if time.time() - os.path.getmtime(path) > ttl:
            return []
        with open(path) as f:
            return sorted({o["name"] for o in json.load(f)["objects"]})
    except (OSError, ValueError, KeyError):
        return []


def parse(instruction, ask=_jev, scene_names=None):
    """一句中文 -> {'skill':..., 'object':..., 'container':...}。

    ask 可注入，便于测试时替掉 jev 服务。
    scene_names: 场景地图里 YOLO 已检测的物体名；给了才可能触发自动加名。
    """
    scan, gloss = object_vocab.load()
    vocab_en = sorted(set(scan) | set(gloss.values()))

    out = {"skill": "pick_and_place", "object": None, "container": None}

    if any(w in instruction for w in _LOOK_WORDS):
        out["skill"] = "look_around"
        return out
    if any(w in instruction for w in _FETCH_WORDS):
        out["skill"] = "fetch_from_user"

    # --- 容器：关键词优先，颜色词一起带上（"粉色盘子" -> "pink plate"） ---
    for words, container in _CONTAINER_KEYWORDS:
        hit = next((w for w in words if w in instruction), None)
        if hit is None:
            continue
        if container == "plate":
            # "粉色盘子" -> "pink plate"：颜色词要紧贴在容器名词前面才算
            head = instruction[:instruction.find(hit)][-3:]
            color = next((c for c in _COLOR if c in head), None)
            noun = next((_NOUN[n] for n in _NOUN if n in instruction), "plate")
            out["container"] = f"{_COLOR[color]} {noun}" if color else noun
        else:
            out["container"] = container
        break

    # --- 物体：指令里出现多个已知名词时全部收下（"苹果，柠檬和橙子" -> 逐个抓），
    #     单个物体才走 jev 抽取。jev 那一问是单选题，表达不了"全部"。 ---
    hits = []
    for zh in sorted(gloss, key=len, reverse=True):
        i = instruction.find(zh)
        if i < 0:
            continue
        if any(s < i + len(zh) and i < e for s, e, _ in hits):
            continue  # "瓶子"已匹配, 跳过子串"瓶"
        hits.append((i, i + len(zh), gloss[zh]))
    if len(hits) > 1:
        out["objects"] = [en for _, _, en in sorted(hits)]
        return out

    # --- 单物体：jev 抽，抽不出再退关键词 ---
    try:
        ans = ask({"object": {"type": "choice",
                              "instructions": "Which object does the user want the robot to act on?",
                              "criteria": {c: "" for c in vocab_en + ["none"]}}},
                  f'User instruction to a tabletop robot arm: "{instruction}".')
        choice, conf = ans["object"]
        out["jev_conf"] = conf
        if choice != "none" and conf >= 0.3:
            out["object"] = choice
    except Exception as e:
        out["jev_error"] = str(e)

    if out["object"] is None:
        out["object"] = next((en for zh, en in gloss.items() if zh in instruction), None)

    # --- 自动加名：词表和 jev 都没认出的物体，问 jev"它对应桌上哪个已检测物体"，
    #     置信度够就落盘（中文名 -> 英文名，英文名来自场景地图 = YOLO 已验证）。 ---
    if out["object"] is None and out["container"] and scene_names:
        frag = _unknown_fragment(instruction, gloss)
        if frag:
            try:
                cands = sorted(set(scene_names) | set(vocab_en))
                ans = ask({"object": {"type": "choice",
                                      "instructions": (f'The user said "{frag}", which is not in the '
                                                       "known vocabulary. Which object matches it?"),
                                      "criteria": {c: "" for c in cands}}},
                          f'User instruction to a tabletop robot arm: "{instruction}".')
                choice, conf = ans["object"]
                if choice != "none" and conf >= 0.3:
                    out["object"] = choice
                    out["learned_from"] = frag
                    object_vocab.add(frag, choice)
            except Exception as e:
                out["learn_error"] = str(e)

    if out["object"] is None and out["container"] is None:
        out["skill"] = "look_around"
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("instruction", nargs="*", help="自然语言指令；不给则进交互模式")
    ap.add_argument("--dry", action="store_true", help="只解析，不执行")
    ap.add_argument("--config", default="./robot_config.json")
    ap.add_argument("--save-path", default="./log")
    args = ap.parse_args()

    import skills
    from skills.base import get_skill
    from termcolor import cprint

    _cache = {}

    def skill_for(name):
        if name not in _cache:
            _cache[name] = get_skill(name)(config_path=args.config, save_path=args.save_path)
        return _cache[name]

    def execute(plan):
        """执行单个解析出的动作, 返回 True/False。"""
        skill, obj, container = plan["skill"], plan["object"], plan["container"]
        try:
            if skill == "look_around":
                ok = skill_for("look_around").run()
            elif skill == "fetch_from_user":
                ok = skill_for("fetch_from_user").run(container=container or "person")
            elif plan.get("objects"):
                # 多物体: 逐个抓, 容器共用
                failed = [o for o in plan["objects"]
                          if not skill_for("pick_and_place").run(object=o, container=container)]
                cprint((f"全部成功: {', '.join(plan['objects'])}" if not failed
                        else f"失败: {', '.join(failed)}"),
                       "green" if not failed else "red")
                return not failed
            elif container and obj:
                ok = skill_for("pick_and_place").run(object=obj, container=container)
            elif container:
                # object 还是 None（学习失败/没开地图）: 不能进 pick_and_place,
                # 它会回退读 stdin 把 REPL 卡死
                cprint("没听出要抓什么 —— 先让我看看桌上有什么（环视建图）再试", "red")
                return False
            else:
                ok = skill_for("grasp").run(object=obj)
        except Exception as e:
            cprint(f"执行出错: {e}", "red")
            return False
        cprint("成功" if ok else "失败", "green" if ok else "red")
        return bool(ok)

    def run_one(text):
        text = text.strip()
        if not text:
            return
        segs = split_sequence(text)
        if len(segs) > 1:
            cprint(f"动作序列 ({len(segs)} 步): {segs}", "cyan")
        for i, seg in enumerate(segs):
            plan = parse(seg, scene_names=load_scene_names())
            cprint(f"jev 解析[{i+1}/{len(segs)}]: {plan}", "cyan")
            if args.dry:
                continue
            # 某步失败即中止: 物体状态已变, 继续做后面的步骤只会错上加错
            if not execute(plan):
                if i + 1 < len(segs):
                    cprint(f"序列中止, 剩余 {len(segs) - i - 1} 步未执行", "red")
                return

    if args.instruction:
        for text in args.instruction:  # 每个引号参数算一条指令
            run_one(text)
        return

    cprint("输入中文指令，回车执行。exit / Ctrl-D 退出。", "yellow")
    while True:
        try:
            line = input("jev> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if line.strip() in ("exit", "quit", "q", "退出"):
            break
        run_one(line)


if __name__ == "__main__":
    main()
