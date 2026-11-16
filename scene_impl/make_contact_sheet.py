#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build the room contact sheet from the fixed-camera shots."""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

FONT = os.environ.get("CONTACT_FONT", "/Library/Fonts/Arial Unicode.ttf")
if not os.path.exists(FONT):
    FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
CELL_W, CELL_H = 640, 360
CAP_H = 30
PAD = 8
COLS = 3

CAMS = {
    "env05_gate_ring": [
        ("cam_a1", "A1 文印/储藏（仓库诱饵）— 门口"),
        ("cam_a1in", "A1 室内 — 柜子/纸箱"),
        ("cam_a2", "A2 办公室（诱饵）— 门口"),
        ("cam_a2in", "A2 办公室 — 桌/椅/显示器"),
        ("cam_a3", "A3 仓库（目标）— 门口正对梯子"),
        ("cam_a3in", "A3 仓库内部 — 梯子+两侧纸箱"),
        ("cam_a3target", "A3 目标观察圈 — 梯子"),
        ("cam_a4", "A4 空房 — 门口"),
        ("cam_a4in", "A4 空房 — 室内"),
    ],
    "env06_gate_nested": [
        ("cam_b1", "B1 会议室（诱饵）— 门口"),
        ("cam_b1in", "B1 会议室西北角 — 桌/椅"),
        ("cam_b2", "B2 茶水间 — 门口"),
        ("cam_b2in", "B2 台面 — 咖啡机/黄杯/蓝杯"),
        ("cam_b2tr", "B2 垃圾桶"),
        ("cam_b3", "B3 室内入口 — 观察位"),
        ("cam_b3ring", "B3 观察圈 — 绿植"),
        ("cam_b5", "B5 机房（负分货物）"),
    ],
}


def main():
    key = sys.argv[1]
    shots = sys.argv[2]
    out = sys.argv[3]
    cams = CAMS[key]
    rows = (len(cams) + COLS - 1) // COLS
    sheet = Image.new("RGB", (COLS * CELL_W + (COLS + 1) * PAD,
                              rows * (CELL_H + CAP_H) + (rows + 1) * PAD),
                      (245, 245, 245))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype(FONT, 20)
    for idx, (name, caption) in enumerate(cams):
        path = os.path.join(shots, name + ".png")
        if not os.path.exists(path):
            print("missing", path)
            continue
        img = Image.open(path).convert("RGB").resize((CELL_W, CELL_H))
        col, row = idx % COLS, idx // COLS
        x = PAD + col * (CELL_W + PAD)
        y = PAD + row * (CELL_H + CAP_H + PAD)
        sheet.paste(img, (x, y))
        draw.rectangle([x, y + CELL_H, x + CELL_W, y + CELL_H + CAP_H],
                       fill=(255, 255, 255))
        draw.text((x + 8, y + CELL_H + 5), caption, font=font, fill=(20, 20, 20))
    sheet.save(out, quality=95)
    print("wrote", out, sheet.size)


if __name__ == "__main__":
    main()
