#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 A4 横版打印棋盘格: 9x6 内角点, 25mm 方格 (250x175mm)。

打印时缩放必须选 100%/实际大小, 打完后用尺子量方格边长核对
(偏差 >0.2mm 就把实测值传给 handeye_calibrate.py --square-mm)。
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages

SQ = 25 / 1000            # 25mm, 单位米 (figsize 用英寸, 这里统一米)
COLS, ROWS = 10, 7        # 方格数 -> 9x6 内角点
A4W, A4H = 297 / 1000, 210 / 1000

fig = plt.figure(figsize=(A4W / 0.0254, A4H / 0.0254))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, A4W)
ax.set_ylim(0, A4H)
ax.axis("off")

x0, y0 = (A4W - COLS * SQ) / 2, (A4H - ROWS * SQ) / 2
ax.add_patch(Rectangle((x0, y0), COLS * SQ, ROWS * SQ, facecolor="white", edgecolor="black"))
for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:
            ax.add_patch(Rectangle((x0 + c * SQ, y0 + r * SQ), SQ, SQ, facecolor="black"))

with PdfPages("/home/zz/Desktop/chessboard_9x6_25mm.pdf") as pdf:
    pdf.savefig(fig, dpi=300)
print("saved: /home/zz/Desktop/chessboard_9x6_25mm.pdf  (打印选 100% 缩放)")
