#!/usr/bin/env python3
"""自然语言指令 -> jev 解析 -> 直接执行 skill。交互式 REPL。

    python run_nl.py                        # 交互模式，直接打中文
    python run_nl.py "把黄瓜放进粉色盘子"      # 单次执行
    python run_nl.py --dry "把橙子递给我"     # 只解析，不动机械臂

分工：**jev 抽目标物体**（实测 conf 0.83~0.92，中文名词抽得很准），
**容器/动作走关键词** —— jev 对"递给我"这类只有 0.24~0.44 置信度，而关键词是确定的。
jev 服务没起时物体也退回关键词表。

需要 Open-Jev 在 127.0.0.1:8791 上跑着（见 Open-Jev/README.md）：
    cd Open-Jev && source .venv/bin/activate
    python -m jev.server --checkpoint models/Open-Jev-2B/package/checkpoint --max-length 4096
"""
import argparse
import json
import os
import sys
import urllib.request

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
os.chdir(PROJECT_ROOT)
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "dependence"))

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

# jev 抽物体用的候选。加 "none" 让它能说"没听出要抓什么"。
_OBJECT_CANDIDATES = ["cucumber", "orange", "apple", "banana", "peach", "pear", "grape",
                      "kiwi fruit", "lemon", "strawberry", "bottle", "cup", "toy",
                      "snack", "vegetable", "fruit", "none"]

# jev 不可用时的兜底词表（与 core/jev_command.py 的 _GLOSS 一致）
_GLOSS = {"黄瓜": "cucumber", "苹果": "apple", "橙子": "orange", "橘子": "orange",
          "香蕉": "banana", "桃子": "peach", "梨": "pear", "葡萄": "grape",
          "柠檬": "lemon", "草莓": "strawberry", "猕猴桃": "kiwi fruit",
          "水果": "fruit", "瓶子": "bottle", "瓶": "bottle", "杯子": "cup", "杯": "cup",
          "玩具": "toy", "零食": "snack", "蔬菜": "vegetable"}

_LOOK_WORDS = ("看看", "看一下", "环视", "看看桌上", "有什么", "扫描")
_FETCH_WORDS = ("接过来", "从我这拿", "递给我的手上", "拿走我手里")


def _jev(questions, state, timeout=15.0):
    """问 jev，返回 {qid: (choice, confidence)}；服务不可用时抛异常。"""
    req = urllib.request.Request(
        JEV_URL, data=json.dumps({"state": state, "questions": questions}).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        answers = json.loads(resp.read())["answers"]
    return {k: (v["choice"], float(v["confidence"])) for k, v in answers.items()}


def parse(instruction, ask=_jev):
    """一句中文 -> {'skill':..., 'object':..., 'container':...}。

    ask 可注入，便于测试时替掉 jev 服务。
    """
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

    # --- 物体：jev 抽，抽不出再退关键词 ---
    try:
        ans = ask({"object": {"type": "choice",
                              "instructions": "Which object does the user want the robot to act on?",
                              "criteria": {c: "" for c in _OBJECT_CANDIDATES}}},
                  f'User instruction to a tabletop robot arm: "{instruction}".')
        choice, conf = ans["object"]
        out["jev_conf"] = conf
        if choice != "none" and conf >= 0.3:
            out["object"] = choice
    except Exception as e:
        out["jev_error"] = str(e)

    if out["object"] is None:
        out["object"] = next((en for zh, en in _GLOSS.items() if zh in instruction), None)

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

    def run_one(text):
        text = text.strip()
        if not text:
            return
        plan = parse(text)
        cprint(f"jev 解析: {plan}", "cyan")
        if args.dry:
            return
        skill, obj, container = plan["skill"], plan["object"], plan["container"]
        try:
            if skill == "look_around":
                ok = skill_for("look_around").run()
            elif skill == "fetch_from_user":
                ok = skill_for("fetch_from_user").run(container=container or "person")
            elif container:
                ok = skill_for("pick_and_place").run(object=obj, container=container)
            else:
                ok = skill_for("grasp").run(object=obj)
        except Exception as e:
            cprint(f"执行出错: {e}", "red")
            return
        cprint("成功" if ok else "失败", "green" if ok else "red")

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
