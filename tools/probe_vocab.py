#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
词表重探
========
core/scene.py 的 SCAN_VOCAB 是**现场标定**出来的: 每个词要同时满足"YOLO-World
认得出"和"用户下指令时会这么说"。换场景、换光照、换东西之后这张表就过期了 ——
而它过期的方式很安静: 某个词不再命中, 地图里就少一样东西, 后面按名字查就查不到。

这个工具在**当前视角**下把候选词逐个跑一遍 YOLO-World, 打出置信度、框和框内颜色,
用来重建那张表。逐词跑比一次全给慢十几倍, 但探词要的正是"这个词单独认不认得出",
所以只能逐词。

用法:
    conda activate anygrasp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib

    python3 tools/probe_vocab.py plum pitaya "dragon fruit" cucumber   # 探这几个词
    python3 tools/probe_vocab.py --image log/xxx.png lemon peach       # 用现成的图, 不连相机
    python3 tools/probe_vocab.py --all                                 # 探 SCAN_VOCAB 里的全部

判读: 置信度 < 0.05 基本等于"认不出" (实测目标常常只有 0.012~0.05, 框却可能是准的);
同一个框被好几个词命中说明这些词互相混淆 (实测 apple 对桃子给 0.968、对黄瓜给 0.423)。
"""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from core import scene as scene_map                       # noqa: E402
from core.backends import create_camera, create_perception  # noqa: E402
from core.perception import dominant_color_name           # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="逐个词探 YOLO-World 的命中情况")
    ap.add_argument("words", nargs="*", help="要探的词 (不给就配合 --all)")
    ap.add_argument("--all", action="store_true", help="探 SCAN_VOCAB 里的全部词")
    ap.add_argument("--image", help="用这张图, 不连相机")
    ap.add_argument("--conf", type=float, default=0.01, help="探测阈值, 默认 0.01 (要看到低分命中)")
    args = ap.parse_args()

    words = scene_map.SCAN_VOCAB if args.all else args.words
    if not words:
        ap.error("给几个词, 或者 --all")

    if args.image:
        import numpy as np
        from PIL import Image
        rgb = np.array(Image.open(args.image).convert("RGB"))
        print(f"用图 {args.image} {rgb.shape}")
    else:
        rgb, _ = create_camera(save_path=os.path.join(ROOT, "log")).get_rgbd()
        print(f"相机取帧 {rgb.shape}  (记得确认光照: 平均亮度 >{40} 才有意义)")

    perception = create_perception()
    for word in words:
        dets = perception.detect_objects(rgb, [word], conf=args.conf)
        if not dets:
            print(f"  {word:16s} —")
            continue
        for d in dets:
            x1, y1, x2, y2, conf, _ = d
            color = dominant_color_name(rgb, d)
            print(f"  {word:16s} conf={conf:.3f} {color or '?':8s} "
                  f"框=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f}) "
                  f"{x2 - x1:.0f}x{y2 - y1:.0f}px")


if __name__ == "__main__":
    main()
