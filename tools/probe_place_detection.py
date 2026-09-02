#!/usr/bin/env python3
"""Offline YOLO-World probe on a saved camera image. No arm motion.

Usage:
    python tools/probe_place_detection.py [image_path] [prompt1,prompt2,...]
Defaults to the latest rgb_*.png in log/ and a built-in prompt list.
"""
import glob
import os
import sys
sys.path.insert(0, '.')
import numpy as np
from PIL import Image
from core.perception import Perception
from core.config import DEFAULT_YOLO_MODEL

if len(sys.argv) > 1:
    IMG = sys.argv[1]
else:
    cands = sorted(glob.glob('log/rgb_*.png'), key=os.path.getmtime)
    if not cands:
        raise SystemExit('no image given and no log/rgb_*.png found')
    IMG = cands[-1]
PROMPTS = (sys.argv[2].split(',') if len(sys.argv) > 2
           else ['pink plate', 'plate', 'bowl', 'dish', 'pink bowl'])

img = np.array(Image.open(IMG).convert('RGB'))
print(f'image: {IMG} shape: {img.shape}')

p = Perception(yolo_model_path=DEFAULT_YOLO_MODEL)
for prompt in PROMPTS:
    for conf in (0.25, 0.15):
        dets = p.detect_objects(img, [prompt], conf=conf)
        pretty = [[round(v, 1) for v in d[:5]] for d in dets]
        print(f'{prompt!r:14} conf={conf}: {pretty}')
