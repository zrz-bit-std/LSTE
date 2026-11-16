#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Annotate the near-orthographic Gazebo top view with room/doors/target."""
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

FONT = os.environ.get("ANNOTATE_FONT", "/Library/Fonts/Arial Unicode.ttf")
if not os.path.exists(FONT):
    FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
IMG_W, IMG_H = 1600, 1200
PANEL_W = 760
COVER_W = 28.0

SCENES = {
    "env05": dict(
        cam=(11.0, -4.0),
        spawn=(1.2, 2.6, -90.0),
        rooms=[
            ("A1 文印/储藏（仓库诱饵）", 5.90, -0.70, "S=0.73", (0.93, 0.94, 0.96)),
            ("A2 办公室（诱饵）", 15.10, -0.70, "S=-0.45", (0.99, 0.80, 0.80)),
            ("A3 仓库（目标）", 5.90, -6.90, "S=0.18", (0.99, 0.90, 0.62)),
            ("A4 空房", 15.10, -6.90, "S=0.00", (0.93, 0.94, 0.96)),
        ],
        corridor=[("环廊 2.4 m", 11.0, 2.85), ("环廊 2.4 m", 11.0, -10.75)],
        doors=[
            ("D_a1", 5.40, 1.60, "H"), ("D_a2", 14.60, 1.60, "H"),
            ("D_a3", 5.40, -9.60, "H"), ("D_a4", 14.60, -9.60, "H"),
        ],
        target=(5.40, -5.80, "target ladder (5.4, -5.8) 正对 D_a3"),
        negatives=[(0.25, -1.60), (21.72, -1.60), (11.00, 3.70), (2.00, -11.70)],
        legend=[
            "黑 = 2.5 m 墙 / 实体",
            "绿 = 2.3 m 门洞（4 个）",
            "0.7 m 门套（房间内侧，走廊齐平）",
            "红 × = 负分物（消防栓 / 锥桶）",
            "★ = 目标 ladder（A3 仓库）",
            "A1 仓库诱饵：纸箱/文件柜",
            "A2 办公室用品负分（monitor/chair/desk）",
            "A3 仓库：梯子正对 D_a3，纸箱分列两侧",
            "A3 货架靠 NE/SW 墙，不遮挡门口-梯子视线",
            "A4 空房",
            "蓝三角 = spawn（P1）",
        ],
    ),
    "env06": dict(
        cam=(13.0, -2.7),
        spawn=(25.1, -4.1, 180.0),
        rooms=[
            ("B1 会议室（诱饵）", 4.00, 0.40, "S=0.97", (0.99, 0.80, 0.80)),
            ("B2 茶水间", 15.50, 0.40, "S=0.73", (0.99, 0.90, 0.62)),
            ("B3 内间办公室", 20.80, 2.30, "S=0.73", (0.99, 0.90, 0.62)),
            ("B4 储物间（负分）", 22.00, -6.80, "S=-0.69", (0.99, 0.80, 0.80)),
            ("B5 机房（负分）", 10.00, -6.80, "S=-0.69", (0.99, 0.80, 0.80)),
        ],
        corridor=[("主走廊 3.0 m", 13.0, -4.10)],
        doors=[
            ("D_b1", 4.00, -2.60, "H"), ("D_b2", 14.50, -2.60, "H"),
            ("D_b5", 9.50, -5.60, "H"), ("D_b4", 20.50, -5.60, "H"),
            ("D_b3", 18.50, 1.50, "V"),
        ],
        target=(22.62, 0.60, "target plant (22.62, 0.60, z=0.75)"),
        negatives=[(11.90, -2.825), (22.60, -5.375), (6.40, -2.85),
                   (8.20, -8.10), (9.90, -8.10), (11.90, -8.30),
                   (20.20, -7.70), (22.60, -7.80), (23.10, -7.80)],
        legend=[
            "黑 = 2.5 m 墙 / 实体",
            "绿 = 2.3 m 门洞（5 个）",
            "0.7 m 门套（房间内侧，走廊齐平）",
            "红 × = 负分物 / 负分货物",
            "★ = 目标 plant",
            "B1 桌+椅整体靠西北墙，留出探索空间",
            "B3 桌椅东移；plant 在桌面，桌面/东南角补正向 office 物品",
            "B4 / B5 货物为负分（file cabinet / box）→ 跳过",
            "蓝三角 = spawn（P1）",
        ],
    ),
    "env04": dict(
        cam=(10.75, -3.25),
        cover=30.0,
        spawn=(3.40, 0.00, 0.0),
        rooms=[
            ("P1 西侧房间（spawn）", 4.50, 0.00, "spawn", (0.93, 0.94, 0.96)),
            ("P2 北侧房间", 7.00, 4.50, "file boxes", (0.99, 0.90, 0.62)),
            ("P5/P6 中部房间", 12.50, -6.00, "0.9 m 西墙", (0.99, 0.80, 0.80)),
            ("P4 东侧目标房", 18.00, -3.50, "yellow cup", (0.99, 0.90, 0.62)),
            ("南侧走廊", 7.00, -11.00, "", (0.93, 0.94, 0.96)),
        ],
        corridor=[("中央走廊", 13.00, -1.00)],
        doors=[
            ("D_p5", 10.84, -6.45, "V", 2.0),
            ("D_p4", 19.15, -10.20, "H", 8.85),
        ],
        target=(20.61, -3.56, "target yellow cup (20.61, -3.56)"),
        negatives=[(20.41, -3.58), (4.61, 1.51), (13.39, 1.54),
                   (10.79, -8.65), (-1.06, -3.39), (-0.85, -12.86)],
        legend=[
            "黑 = 2.5 m 墙 / 实体",
            "绿 = 门洞（P5/P6 0.9 m 西墙 / 目标房南入口）",
            "红 × = 干扰/负分物（blue cup、fire hydrant）",
            "★ = 目标 yellow cup (20.61, -3.56)",
            "P1 = spawn (3.4, 0.0)",
            "P2 = 北侧房间（file boxes）",
            "P5/P6 = 0.9 m 门洞房间",
            "P4 = 东侧目标房（南侧开口）",
            "蓝三角 = spawn（P1）",
        ],
    ),
}


def pixel(scene, x, y):
    cx, cy = scene["cam"]
    s = IMG_W / scene.get("cover", COVER_W)
    return IMG_W / 2 + (x - cx) * s, IMG_H / 2 - (y - cy) * s


def box(draw, xy, text, font, fg=(20, 20, 20), bg=(255, 255, 255, 225),
        pad=8, anchor="mm"):
    x, y = xy
    l, t, r, b = draw.multiline_textbbox((0, 0), text, font=font,
                                        spacing=4, anchor="la")
    w, h = r - l, b - t
    if anchor == "mm":
        x0, y0 = x - w / 2 - pad, y - h / 2 - pad
    else:
        x0, y0 = x - pad, y - pad
    draw.rounded_rectangle([x0, y0, x0 + w + 2 * pad, y0 + h + 2 * pad],
                           radius=8, fill=bg)
    draw.multiline_text((x0 + pad - l, y0 + pad - t), text, font=font,
                        fill=fg, spacing=4)


def main():
    key = sys.argv[1]
    src = sys.argv[2]
    dst = sys.argv[3]
    sc = SCENES[key]
    src_im = Image.open(src).convert("RGBA")
    im = Image.new("RGBA", (IMG_W + PANEL_W, IMG_H), (235, 235, 235, 255))
    im.paste(src_im, (0, 0))
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)

    f_room = ImageFont.truetype(FONT, 27)
    f_sub = ImageFont.truetype(FONT, 20)
    f_small = ImageFont.truetype(FONT, 18)
    f_leg = ImageFont.truetype(FONT, 20)
    f_tgt = ImageFont.truetype(FONT, 21)

    # doors: thick green segment across the opening
    for door in sc["doors"]:
        name, x, y, kind = door[0], door[1], door[2], door[3]
        half = (float(door[4]) / 2.0) if len(door) > 4 else 1.15
        if kind == "H":
            p0 = pixel(sc, x - half, y)
            p1 = pixel(sc, x + half, y)
        else:
            p0 = pixel(sc, x, y - half)
            p1 = pixel(sc, x, y + half)
        d.line([p0, p1], fill=(30, 200, 80, 255), width=7)
        mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        box(d, (mx, my - 26 if kind == "H" else my - 34), name, f_small,
            bg=(255, 255, 255, 215), pad=4)

    # rooms
    for name, x, y, s_env, color in sc["rooms"]:
        px, py = pixel(sc, x, y)
        c = tuple(int(v * 255) for v in color) + (235,)
        box(d, (px, py), f"{name}\n{s_env}", f_room, bg=c, pad=10)

    # corridor
    for name, x, y in sc["corridor"]:
        px, py = pixel(sc, x, y)
        box(d, (px, py), name, f_sub, fg=(40, 40, 40), bg=(255, 255, 255, 200), pad=5)

    # negatives
    for x, y in sc["negatives"]:
        px, py = pixel(sc, x, y)
        r = 12
        d.line([(px - r, py - r), (px + r, py + r)], fill=(220, 30, 30, 255), width=5)
        d.line([(px - r, py + r), (px + r, py - r)], fill=(220, 30, 30, 255), width=5)

    # spawn marker
    sx, sy, syaw = sc["spawn"]
    px, py = pixel(sc, sx, sy)
    ang = math.radians(syaw)
    tip = (px + 16 * math.cos(ang), py - 16 * math.sin(ang))
    left = (px + 10 * math.cos(ang + 2.5), py - 10 * math.sin(ang + 2.5))
    right = (px + 10 * math.cos(ang - 2.5), py - 10 * math.sin(ang - 2.5))
    d.polygon([tip, left, right], fill=(37, 99, 235, 255), outline=(15, 23, 42, 255))
    box(d, (px, py - 34), "spawn", f_small, fg=(30, 58, 138), bg=(255, 255, 255, 225), pad=3)

    # target star
    tx, ty, label = sc["target"]
    px, py = pixel(sc, tx, ty)
    r = 16
    pts = []
    for i in range(10):
        ang = -math.pi / 2 + i * math.pi / 5
        rr = r if i % 2 == 0 else r * 0.45
        pts.append((px + rr * math.cos(ang), py + rr * math.sin(ang)))
    d.polygon(pts, fill=(230, 40, 40, 255), outline=(120, 0, 0, 255))
    box(d, (px, py - 46), label, f_tgt, fg=(140, 0, 0), bg=(255, 255, 255, 225), pad=5)

    # legend in the grey margin (top-left)
    lx, ly = IMG_W + 20, 24
    lines = "\n".join(sc["legend"])
    l, t, r, b = d.multiline_textbbox((0, 0), lines, font=f_leg, spacing=6)
    d.rounded_rectangle([lx, ly, lx + (r - l) + 28, ly + (b - t) + 24],
                        radius=10, fill=(255, 255, 255, 232),
                        outline=(80, 80, 80, 255), width=2)
    d.multiline_text((lx + 14, ly + 12), lines, font=f_leg,
                     fill=(20, 20, 20), spacing=6)

    im = Image.alpha_composite(im, ov).convert("RGB")
    im.save(dst, quality=95)
    print("wrote", dst)


if __name__ == "__main__":
    main()
