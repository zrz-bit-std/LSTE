#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the two LSTE scene design packages (tables + figures + task JSON).

Outputs into /Users/zrz_mac/lste_scene_design/
  tables/*.csv      wall / door / room / object tables
  tables.md         all tables in markdown
  figures/*.png     floor plans
  tasks/*.json      task_parsed JSON in the existing schema
"""
import csv
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

plt.rcParams["font.sans-serif"] = ["PingFang SC", "Heiti SC", "STHeiti", "Arial Unicode MS"]
plt.rcParams["axes.unicode_minus"] = False

ROOT = "/Users/zrz_mac/lste_scene_design"
THICK = 0.15
WALL_H = 2.5
# Door: 2.30 m clear opening through a 0.70 m wall that thickens only
# into the room.  The corridor face stays flush; no local jamb blocks.
DOOR_W = float(os.environ.get("DOOR_W", "2.30"))
DOOR_DEPTH = float(os.environ.get("DOOR_DEPTH", "0.70"))

# ---------------------------------------------------------------- model footprints
# measured from the COLLADA meshes (full node-transform bbox) on bit1010
MODELS = {
    "desk_brown":            (2.40, 0.80, 0.75),
    "office_chair":          (0.76, 0.64, 0.56),
    "monitor_3":             (0.60, 0.10, 0.34),
    "file_cabinet_large":    (1.49, 1.17, 0.68),
    "file_cabinet_small":    (0.60, 0.77, 0.67),
    "bookshelf_large":       (1.66, 0.43, 1.84),
    "cardboard_box":         (0.40, 0.40, 0.40),
    "laptop_pc_1":           (0.35, 0.31, 0.25),
    "blue_bin_cuboid":       (0.60, 0.67, 0.96),
    "first_2015_trash_can":  (0.57, 0.64, 0.74),
    "fire_hydrant":          (0.30, 0.30, 0.75),
    "construction_cone":     (0.35, 0.35, 0.70),
    "coffee_machine":        (0.43, 0.59, 0.38),
    "plant_1":               (0.83, 0.84, 0.55),
    "table_dining":          (0.94, 0.47, 0.67),
    "cup_yellow":            (0.123, 0.082, 0.077),
    "cup_blue":              (0.128, 0.090, 0.110),
    "keyboard":              (0.44, 0.15, 0.02),
    "lamp_table_small":      (0.18, 0.18, 0.23),
    "ladder_75deg_0rails":   (0.95, 1.55, 2.52),
}

# ---------------------------------------------------------------- scenes
SCENES = {}

SCENES["env05_gate_ring"] = dict(
    title="env05_gate_ring",
    subtitle="环形四房 · 门控语义梯度 · 目标=仓库梯子",
    bbox=(0.0, -12.0, 22.0, 4.0),
    spawn=(1.2, 2.6, -1.5708),
    target="ladder_75deg_0rails",
    target_xy=(5.40, -5.80, 0.0),
    target_view=(5.40, -7.50),
    target_room="A3",
    peek_dist=2.0,
    route=[("spawn", 1.2, 2.6), ("D_a1 观察位", 5.4, 2.8), ("D_a2 观察位", 14.6, 2.8),
           ("A2 室内", 16.6, -0.8), ("东廊拐角", 20.8, 2.8), ("东南角", 20.8, -10.8),
           ("D_a4 观察位", 14.6, -10.8), ("D_a3 观察位", 5.4, -10.8),
           ("A3 室内", 5.4, -7.0), ("目标观察圈", 5.40, -7.50)],
    walls=[
        # id, kind, at, start, end, openings
        ("W_out_n",  "H", 4.0,  0.0, 22.0, []),
        ("W_out_s",  "H", -12.0, 0.0, 22.0, []),
        ("W_out_w",  "V", 0.0, -12.0, 4.0, []),
        ("W_out_e",  "V", 22.0, -12.0, 4.0, []),
        ("W_core_n", "H", 1.6,  2.4, 19.6, [(4.7, 6.1), (13.9, 15.3)]),
        ("W_core_s", "H", -9.6, 2.4, 19.6, [(4.7, 6.1), (13.9, 15.3)]),
        ("W_core_w", "V", 2.4, -9.6, 1.6, []),
        ("W_core_e", "V", 19.6, -9.6, 1.6, []),
    ],
    solids=[
        # name, x0, x1, y0, y1  (full-height 2.5 m masonry / duct)
        ("S_cross_v", 9.4, 10.6, -9.6, 1.6),
        ("S_cross_h", 2.4, 19.6, -4.2, -3.0),
    ],
    rooms=[
        # id, name, x0, x1, y0, y1, s_env_door, visible terms, expected
        ("A1", "文印/储藏间（仓库诱饵）", 2.4, 9.4, -3.0, 1.6, 0.50, "cardboard box", "进入→退出"),
        ("A2", "会议室（办公室诱饵）", 10.6, 19.6, -3.0, 1.6, 0.00, "desk+chair+monitor（负分）", "跳过"),
        ("A3", "仓库（目标）", 2.4, 9.4, -9.6, -4.2, 0.86, "cardboard box+shelf", "进入→二次搜索"),
        ("A4", "空房", 10.6, 19.6, -9.6, -4.2, 0.00, "无", "不进入"),
    ],
    doors=[
        # id, room, wall_axis, at, center, width, s_env
        ("D_a1", "A1", "H", 1.6, 5.4, DOOR_W, 0.50),
        ("D_a2", "A2", "H", 1.6, 14.6, DOOR_W, 0.00),
        ("D_a3", "A3", "H", -9.6, 5.4, DOOR_W, 0.86),
        ("D_a4", "A4", "H", -9.6, 14.6, DOOR_W, 0.00),
    ],
    # +1 = thicken toward +axis (room side), -1 = toward -axis.
    door_side={"W_core_n": -1, "W_core_s": 1},
    objects=[
        # id, model, x, y, yaw, z, role, note
        ("a1_cab1", "file_cabinet_large", 3.90, 0.00, 0.0, 0.0, "decoy", "门左，留出中央通道"),
        ("a1_cab2", "file_cabinet_large", 7.20, 0.00, 0.0, 0.0, "decoy", "门右，留出中央通道"),
        ("a1_box1", "cardboard_box", 2.85, -0.90, 0.0, 0.0, "decoy", "西墙边"),
        ("a1_box2", "cardboard_box", 2.85, -0.45, 0.0, 0.0, "decoy", "西墙边"),
        ("a1_box3", "cardboard_box", 8.60, -0.90, 0.3, 0.0, "decoy", "东墙边"),
        ("a2_desk", "desk_brown", 14.60, -2.20, 0.0, 0.0, "env", "贴南墙，消除桌后 frontier 陷阱"),
        ("a2_ch1", "office_chair", 13.60, -0.40, 0.0, 0.0, "env", "门前一排"),
        ("a2_ch2", "office_chair", 14.60, -0.40, 0.0, 0.0, "env", "门前一排"),
        ("a2_ch3", "office_chair", 15.60, -0.40, 0.0, 0.0, "env", "门前一排"),
        ("a2_mon", "monitor_3", 15.40, -2.20, 0.0, 0.75, "env", "桌面，随桌南移"),
        ("a3_target", "ladder_75deg_0rails", 5.40, -5.80, 0.0, 0.0, "target", "A3 门正对面，东西向对齐 D_a3"),
        ("a3_shelf_n", "bookshelf_large", 7.60, -4.45, 0.0, 0.0, "env", "北墙货架，避开门口-梯子视线"),
        ("a3_shelf_w", "bookshelf_large", 2.72, -7.60, 90.0, 0.0, "env", "西墙货架，避开门口-梯子视线"),
        ("a3_box1", "cardboard_box", 4.30, -5.55, 0.0, 0.0, "env", "梯子西侧纸箱，不遮挡"),
        ("a3_box2", "cardboard_box", 4.30, -5.55, 0.0, 0.40, "env", "梯子西侧纸箱堆中层"),
        ("a3_box3", "cardboard_box", 4.30, -6.05, 0.0, 0.0, "env", "梯子西侧纸箱"),
        ("a3_box4", "cardboard_box", 4.30, -6.55, 0.0, 0.0, "env", "梯子西侧纸箱"),
        ("a3_box5", "cardboard_box", 4.30, -6.55, 0.0, 0.40, "env", "梯子西侧纸箱堆中层"),
        ("a3_box6", "cardboard_box", 6.50, -5.55, 0.0, 0.0, "env", "梯子东侧纸箱，不遮挡"),
        ("a3_box7", "cardboard_box", 6.50, -5.55, 0.0, 0.40, "env", "梯子东侧纸箱堆中层"),
        ("a3_box8", "cardboard_box", 6.50, -6.05, 0.0, 0.0, "env", "梯子东侧纸箱"),
        ("a3_box9", "cardboard_box", 6.50, -6.55, 0.0, 0.0, "env", "梯子东侧纸箱"),
        ("a3_box10", "cardboard_box", 6.50, -6.55, 0.0, 0.40, "env", "梯子东侧纸箱堆中层"),
        ("n_hyd1", "fire_hydrant", 0.25, -1.60, 0.0, 0.0, "negative", "西走廊墙边"),
        ("n_hyd2", "fire_hydrant", 21.72, -1.60, 0.0, 0.0, "negative", "东走廊墙边"),
        ("n_cone1", "construction_cone", 11.00, 3.70, 0.0, 0.0, "negative", "北走廊"),
        ("n_cone2", "construction_cone", 2.00, -11.70, 0.0, 0.0, "negative", "南走廊"),
    ],
)

SCENES["env06_gate_nested"] = dict(
    title="env06_gate_nested",
    subtitle="主轴+套间 · 两层门控 · 目标=绿植",
    bbox=(0.0, -8.8, 26.0, 3.4),
    spawn=(25.1, -4.1, 3.1416),
    target="plant_1",
    target_xy=(22.62, 0.60, 0.75),
    target_view=(20.80, 0.90),
    target_room="B3",
    peek_dist=1.5,
    route=[("spawn", 25.1, -4.1), ("D_b2 观察位", 14.5, -4.1),
           ("B2 室内", 14.2, 1.2), ("绕过挡墙", 17.0, 0.4), ("D_b3 观察位", 17.0, 1.5),
           ("B3 室内", 20.0, 1.5), ("绕桌东侧", 23.6, -0.5), ("目标观察圈", 20.8, 0.9)],
    walls=[
        ("W_out_n",  "H", 3.4,  0.0, 26.0, []),
        ("W_out_s",  "H", -8.8, 0.0, 26.0, []),
        ("W_out_w",  "V", 0.0, -8.8, 3.4, []),
        ("W_out_e",  "V", 26.0, -8.8, 3.4, []),
        ("W_corr_n", "H", -2.6, 0.0, 26.0, [(3.3, 4.7), (14.3, 15.7)]),
        ("W_corr_s", "H", -5.6, 0.0, 26.0, [(8.8, 10.2), (19.8, 21.2)]),
        ("W_b1_e", "V", 8.0, -2.6, 3.4, []),
        ("W_b2_w", "V", 12.5, -2.6, 3.4, []),
        ("W_b2_e", "V", 18.5, -2.6, 3.4, [(0.8, 2.2)]),
        ("W_b2_stub", "V", 16.0, -2.6, -0.6, []),
        ("W_b5_w", "V", 7.0, -8.8, -5.6, []),
        ("W_b5_e", "V", 13.0, -8.8, -5.6, []),
        ("W_b4_w", "V", 18.0, -8.8, -5.6, []),
    ],
    solids=[
        ("S_n_gap", 8.0, 12.5, -2.6, 3.4),
        ("S_s_gap1", 0.0, 7.0, -8.8, -5.6),
        ("S_s_gap2", 13.0, 18.0, -8.8, -5.6),
    ],
    rooms=[
        ("B1", "会议室（诱饵）", 0.0, 8.0, -2.6, 3.4, 0.88, "desk+chair+monitor", "进入→退出"),
        ("B2", "茶水间（中转）", 12.5, 18.5, -2.6, 3.4, 0.50, "desk+chair", "进入→拿证据"),
        ("B3", "内间办公室（套间）", 18.5, 26.0, -2.6, 3.4, 0.50, "desk+chair", "进入→二次搜索"),
        ("B4", "储物间", 18.0, 26.0, -8.8, -5.6, -0.70, "file cabinet+cardboard box（负分货物）", "跳过（负分）"),
        ("B5", "机房", 7.0, 13.0, -8.8, -5.6, -0.70, "file cabinet+cardboard box（负分货物）", "跳过（负分）"),
    ],
    doors=[
        ("D_b1", "B1", "H", -2.6, 4.0, DOOR_W, 0.88),
        ("D_b2", "B2", "H", -2.6, 14.5, DOOR_W, 0.50),
        ("D_b5", "B5", "H", -5.6, 9.5, DOOR_W, -0.70),
        ("D_b4", "B4", "H", -5.6, 20.5, DOOR_W, -0.70),
        ("D_b3", "B3", "V", 18.5, 1.5, DOOR_W, 0.50),
    ],
    # +1 = thicken toward +axis (room side), -1 = toward -axis.
    door_side={"W_corr_n": 1, "W_corr_s": -1, "W_b2_e": 1},
    # Narrow the B2 pantry door to 2.00 m.  Measured with StructuralPlaceMap
    # (clearance 0.60, place_clear 0.75): 1.40/1.45 -> 5/5 splits, 1.50 -> 4/5,
    # 2.00 and 2.30 -> 0/5.  So 2.00 m alone does not split B2 from the
    # corridor; keep 1.40 m if the goal is a structural place split.
    door_widths={"D_b2": 2.00},
    objects=[
        ("b1_desk", "desk_brown", 2.10, 2.55, 0.0, 0.0, "env", "B1 西北角，整体贴墙"),
        ("b1_ch1", "office_chair", 1.10, 1.35, 0.0, 0.0, "env", "随桌移到西北"),
        ("b1_ch2", "office_chair", 2.10, 1.35, 0.0, 0.0, "env", "随桌移到西北"),
        ("b1_ch3", "office_chair", 3.10, 1.35, 0.0, 0.0, "env", "随桌移到西北"),
        ("b1_mon", "monitor_3", 2.70, 2.55, 0.0, 0.75, "env", "桌面"),
        ("b5_cab1", "file_cabinet_large", 8.20, -8.10, 180.0, 0.0, "negative", "负分货物，靠南墙"),
        ("b5_cab2", "file_cabinet_large", 9.90, -8.10, 180.0, 0.0, "negative", "负分货物，靠南墙"),
        ("b5_box", "cardboard_box", 11.90, -8.30, 0.0, 0.0, "negative", "负分货物，角落"),
        ("b2_desk", "desk_brown", 13.80, 2.60, 0.0, 0.0, "env", "西北角，正对门可见"),
        ("b2_chair", "office_chair", 13.00, 1.60, 0.0, 0.0, "env", "靠西墙，让出房间中部"),
        ("b2_counter", "desk_brown", 16.60, 2.85, 0.0, 0.0, "env", "北墙，挡墙后，西移避开 0.9 m 墙带"),
        ("b2_coffee", "coffee_machine", 15.90, 2.85, 0.0, 0.75, "env", "北墙台面，随台面西移"),
        ("b2_trash", "first_2015_trash_can", 17.60, -1.45, 0.0, 0.0, "decoy", "异类干扰(垃圾桶)，北移避开房间侧墙"),
        ("b2_cup_y", "cup_yellow", 16.75, 2.80, 0.0, 0.75, "decoy", "颜色消融变体目标"),
        ("b2_cup_b", "cup_blue", 17.15, 2.80, 0.0, 0.75, "decoy", "颜色消融变体负例"),
        ("b3_desk", "desk_brown", 23.40, 0.60, 0.0, 0.0, "env", "再东移 0.6 m，入口更宽"),
        ("b3_chair", "office_chair", 24.30, -0.30, 180.0, 0.0, "env", "桌南侧，观察视角可见"),
        ("b3_shelf", "bookshelf_large", 21.00, 3.05, 0.0, 0.0, "occluder", "北墙，装饰/遮挡"),
        ("b3_target", "plant_1", 22.62, 0.60, 0.0, 0.75, "target", "放在桌面上靠西边缘，z=0.75"),
        ("b3_mon", "monitor_3", 23.90, 0.30, 0.0, 0.75, "env", "桌面显示器，避开绿植遮挡"),
        ("b3_keyboard", "keyboard", 24.20, 0.60, 0.0, 0.75, "env", "桌面键盘，正向 office 物品"),
        ("b3_cup", "cup_yellow", 24.40, 0.80, 0.0, 0.75, "env", "桌面杯子，正向 office 物品"),
        ("b3_trash", "first_2015_trash_can", 23.60, 1.50, 0.0, 0.0, "env", "桌北侧垃圾桶，入口可见"),
        ("b4_cab", "file_cabinet_large", 20.20, -7.70, 180.0, 0.0, "decoy", ""),
        ("b4_box1", "cardboard_box", 22.60, -7.80, 0.0, 0.0, "decoy", ""),
        ("b4_box2", "cardboard_box", 23.10, -7.80, 0.4, 0.0, "decoy", ""),
        ("n_hyd1", "fire_hydrant", 11.90, -2.825, 0.0, 0.0, "negative", "主走廊北墙，完全贴墙"),
        ("n_hyd2", "fire_hydrant", 22.60, -5.375, 0.0, 0.0, "negative", "主走廊南墙，完全贴墙"),
        ("n_cone1", "construction_cone", 6.40, -2.85, 0.0, 0.0, "negative", "主走廊北墙，完全贴墙"),
    ],
)

# ---------------------------------------------------------------- wall boxes
def wall_boxes(scene):
    """Return dicts (id, cx, cy, w, d, yaw_deg) for every solid wall box.

    A wall that contains doors is thickened to ``DOOR_DEPTH`` (0.70 m)
    only toward the room side (``door_side``).  The corridor face stays
    exactly on the old wall face, so the ring/corridor has no bump; the
    room face is one continuous flat plane along the whole segment.  The
    clear opening stays ``DOOR_W`` (2.30 m), so the passage is a
    2.30 x 0.70 m tunnel with the thick wall ends acting as jambs.
    Walls without doors stay the 0.15 m production thickness.
    """
    out = []
    sides = scene.get("door_side", {})

    def segment(wid, kind, center, depth, s0, s1):
        if s1 - s0 <= 1e-6:
            return
        length = s1 - s0
        mid = (s0 + s1) / 2.0
        if kind == "H":
            out.append(dict(id=wid, cx=mid, cy=center, w=length, d=depth, yaw=0.0, kind="H"))
        else:
            out.append(dict(id=wid, cx=center, cy=mid, w=length, d=depth, yaw=90.0, kind="V"))

    overrides = dict(scene.get("door_widths") or {})
    for wid, kind, at, a, b, _openings in scene["walls"]:
        # Derive the clear cuts from the door table so the design tables,
        # figures, and the emitted world always share one door width.
        # ``door_widths`` pins individual doors (metres) on top of DOOR_W.
        cuts = sorted(
            (dc - float(overrides.get(_did, dw)) / 2.0,
             dc + float(overrides.get(_did, dw)) / 2.0)
            for _did, _room, axis, dat, dc, dw, _s in scene["doors"]
            if axis == kind and abs(dat - at) < 1e-6
        )
        start = a
        pieces = []
        for (o0, o1) in cuts:
            if o0 > start:
                pieces.append((start, o0))
            start = max(start, o1)
        if start < b:
            pieces.append((start, b))

        if not cuts:
            for i, (s0, s1) in enumerate(pieces):
                segment(f"{wid}_{i}", kind, at, THICK, s0, s1)
            continue

        side = sides.get(wid)
        if side is None:
            raise ValueError("wall %s has doors but no door_side" % wid)
        corridor_face = at - side * THICK / 2.0
        center = corridor_face + side * DOOR_DEPTH / 2.0
        for i, (s0, s1) in enumerate(pieces):
            segment(f"{wid}_{i}", kind, center, DOOR_DEPTH, s0, s1)

    for name, x0, x1, y0, y1 in scene["solids"]:
        out.append(dict(id=name, cx=(x0 + x1) / 2.0, cy=(y0 + y1) / 2.0,
                        w=x1 - x0, d=y1 - y0, yaw=0.0, kind="BLOCK"))
    return out


def write_tables(scene, outdir):
    rows_w, rows_d, rows_r, rows_o = [], [], [], []
    for b in wall_boxes(scene):
        if b["kind"] == "BLOCK":
            rows_w.append([scene["title"], b["id"], "BLOCK", b["cx"], b["cy"],
                           f'{round(b["w"],3)}x{round(b["d"],3)}', "-", WALL_H, "实体块(全高)"])
        else:
            rows_w.append([scene["title"], b["id"], b["kind"], b["cx"], b["cy"],
                           round(b["w"], 3), THICK, WALL_H, ""])
    overrides = dict(scene.get("door_widths") or {})
    for did, room, axis, at, center, width, s in scene["doors"]:
        rows_d.append([scene["title"], did, room, axis, at, center,
                       float(overrides.get(did, width)), round(s, 2)])
    for rid, name, x0, x1, y0, y1, s, terms, expected in scene["rooms"]:
        rows_r.append([scene["title"], rid, name, x0, x1, y0, y1,
                       round((x1 - x0) * (y1 - y0), 1), round(s, 2), terms, expected])
    for oid, model, x, y, yaw, z, role, note in scene["objects"]:
        w, d, h = MODELS[model]
        rows_o.append([scene["title"], oid, model, x, y, yaw, z, f"{w}x{d}x{h}", role, note])
    for name, rows, header in (
        ("walls", rows_w, ["scene", "id", "orientation", "cx", "cy", "length", "thickness", "height", "note"]),
        ("doors", rows_d, ["scene", "id", "room", "wall_axis", "wall_at", "center", "width", "s_env_door"]),
        ("rooms", rows_r, ["scene", "id", "name", "x0", "x1", "y0", "y1", "area_m2", "s_env_door", "visible_terms", "expected"]),
        ("objects", rows_o, ["scene", "id", "model", "x", "y", "yaw_deg", "z", "size_m", "role", "note"]),
    ):
        path = os.path.join(outdir, f"{scene['title']}_{name}.csv")
        with open(path, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow(header)
            wr.writerows(rows)
    return rows_w, rows_d, rows_r, rows_o


def wide_yaw_placeholder(cx, cy, length, scene):
    return ""


# ---------------------------------------------------------------- figures
ROOM_FILL = {0.0: "#e2e8f0", 0.02: "#cbd5e1", 0.12: "#fde68a",
             0.50: "#fbbf24", 0.88: "#f87171", 1.0: "#dc2626"}


def draw_scene(scene, path):
    x0b, y0b, x1b, y1b = scene["bbox"]
    fig, ax = plt.subplots(figsize=(15, 11))
    # rooms
    for rid, name, x0, x1, y0, y1, s, terms, expected in scene["rooms"]:
        col = ROOM_FILL.get(round(s, 2), "#f1f5f9")
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=col, alpha=0.55, edgecolor="none", zorder=0))
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        ax.text(cx, cy, f"{rid}  {name}\nS_door = {s:.2f}  见: {terms}\n→ {expected}",
                ha="center", va="center", fontsize=8.5, zorder=6, linespacing=1.5,
                bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="#94a3b8", alpha=0.92))
    # walls
    for b in wall_boxes(scene):
        ax.add_patch(Rectangle((b["cx"] - b["w"] / 2, b["cy"] - b["d"] / 2), b["w"], b["d"],
                               angle=b["yaw"], rotation_point="center",
                               facecolor="#334155", edgecolor="none", zorder=3))
    # objects
    for oid, model, x, y, yaw, z, role, note in scene["objects"]:
        w, d, h = MODELS[model]
        color = {"env": "#2563eb", "decoy": "#64748b", "target": "#f59e0b",
                 "negative": "#7c2d12", "hard_negative": "#db2777", "occluder": "#0f766e"}.get(role, "#94a3b8")
        if role == "negative":
            ax.plot(x, y, "X", ms=12, color=color, markeredgecolor="k", zorder=8)
        elif role == "target":
            ax.add_patch(Rectangle((x - w / 2, y - d / 2), w, d, angle=yaw, rotation_point="center",
                                   facecolor="#fde047", edgecolor="k", lw=1.2, zorder=9))
            ax.plot(x, y, "*", ms=20, color="#f59e0b", markeredgecolor="k", zorder=10)
        else:
            ax.add_patch(Rectangle((x - w / 2, y - d / 2), w, d, angle=yaw, rotation_point="center",
                                   facecolor=color, alpha=0.45, edgecolor=color, lw=0.8, zorder=5))
    # doors
    for did, room, axis, at, center, width, s in scene["doors"]:
        if axis == "H":
            ax.plot([center - width / 2, center + width / 2], [at, at], "-", color="#16a34a", lw=4, zorder=6)
            ax.annotate(f"{did}\n{width:.2f}m", (center, at), textcoords="offset points", xytext=(0, 10),
                        ha="center", fontsize=7, color="#15803d", zorder=7)
        else:
            ax.plot([at, at], [center - width / 2, center + width / 2], "-", color="#16a34a", lw=4, zorder=6)
            ax.annotate(f"{did}\n1.4m", (at, center), textcoords="offset points", xytext=(10, 0),
                        ha="left", fontsize=7, color="#15803d", zorder=7)
    # spawn
    sx, sy, syaw = scene["spawn"]
    ax.plot(sx, sy, "o", ms=11, mfc="#0ea5e9", mec="k", zorder=11)
    ax.annotate("pro3 spawn", (sx, sy), textcoords="offset points", xytext=(8, -14), fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlim(x0b - 1.2, x1b + 1.2)
    ax.set_ylim(y0b - 1.2, y1b + 1.2)
    ax.set_xticks(range(int(x0b), int(x1b) + 1, 2))
    ax.set_yticks(range(int(y0b), int(y1b) + 1, 2))
    ax.grid(alpha=0.25)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{scene['title']} — {scene['subtitle']}\n颜色=S_env(门口可见)；★=target；X=负分物；绿线=1.4 m 门洞", fontsize=11)
    plt.tight_layout()
    plt.savefig(path, dpi=130)
    plt.close(fig)


def seg_hits_rect(p, q, rect):
    """2-D segment vs axis-aligned rectangle (slab test)."""
    x0, y0, x1, y1 = rect
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for d, lo, hi, s0 in ((dx, x0, x1, p[0]), (dy, y0, y1, p[1])):
        if abs(d) < 1e-9:
            if s0 < lo or s0 > hi:
                return False
        else:
            a, b = (lo - s0) / d, (hi - s0) / d
            if a > b:
                a, b = b, a
            t0, t1 = max(t0, a), min(t1, b)
            if t0 > t1:
                return False
    return True


def aabb(cx, cy, w, d, yaw_deg):
    """Axis-aligned bounds of a w x d box rotated about its centre."""
    t = math.radians(yaw_deg)
    c, s = abs(math.cos(t)), abs(math.sin(t))
    hw, hd = (w * c + d * s) / 2.0, (w * s + d * c) / 2.0
    return (cx - hw, cy - hd, cx + hw, cy + hd)


def occluder_rects(scene):
    rects = []
    for b in wall_boxes(scene):
        rects.append(aabb(b["cx"], b["cy"], b["w"], b["d"], b["yaw"]))
    for oid, model, x, y, yaw, z, role, note in scene["objects"]:
        w, d, h = MODELS[model]
        # only shelf-height furniture blocks the 0.60 m camera line; the target
        # itself must not count as its own occluder.
        if h >= 1.5 and role != "target":
            x0, y0, x1, y1 = aabb(x, y, w, d, yaw)
            rects.append((x0 - .2, y0 - .2, x1 + .2, y1 + .2))
    return rects


def blocked(p, q, rects):
    for r in rects:
        if seg_hits_rect(p, q, r):
            return True
    return False


def validate(scene, peek_dist):
    rects = occluder_rects(scene)
    env_by_room = {}
    for oid, model, x, y, yaw, z, role, note in scene["objects"]:
        if role in ("env", "decoy", "occluder", "hard_negative"):
            env_by_room.setdefault(oid.split("_")[0], []).append((oid, x, y))
    out = []
    for did, room, axis, at, center, width, s in scene["doors"]:
        if axis == "H":
            room_cy = None
            for r in scene["rooms"]:
                if r[0] == room:
                    room_cy = (r[4] + r[5]) / 2.0
            sign = 1.0 if room_cy < at else -1.0
            peek = (center, at + sign * peek_dist)
        else:
            peek = (at - peek_dist, center)
        # what env objects can be seen through this door?
        rid = room
        seen = []
        for oid, model, x, y, yaw, z, role, note in scene["objects"]:
            if role not in ("env", "decoy"):
                continue
            if not oid.startswith(rid.lower()):
                continue
            if not blocked(peek, (x, y), rects):
                seen.append(model)
        tv = scene.get("target_view")
        tgt = scene["target_xy"][:2]
        out.append(dict(door=did, room=room, peek=peek, seen=sorted(set(seen)), s=s,
                        n_terms=len(set(seen)),
                        target_blocked=blocked(peek, tgt, rects) if tv else None,
                        target_seen_at_view=(not blocked(tv, tgt, rects)) if tv else None))
    return out


def scene_wall_depth(wid, scene):
    for name, sx0, sx1, sy0, sy1 in scene["solids"]:
        if name == wid:
            return sy1 - sy0
    return THICK


# ---------------------------------------------------------------- markdown tables
def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join(["---"] * len(header)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def room_of(scene, x, y):
    for rid, name, x0, x1, y0, y1, s, terms, expected in scene["rooms"]:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return rid
    return "走廊"


def obj_rows3(scene):
    rows = []
    for oid, model, x, y, yaw, z, role, note in scene["objects"]:
        w, d, h = MODELS[model]
        rows.append([oid, model, room_of(scene, x, y), round(x, 3), round(y, 3), round(z, 3),
                     round(yaw, 1), 0.0, 0.0,
                     f"{w}x{d}x{h}", round(z + h, 3),
                     "地板" if z < 0.02 else f"台面 z={z:.3f}", role, note])
    return rows


def wall_rects(scene):
    rows = []
    for b in wall_boxes(scene):
        x0, y0, x1, y1 = aabb(b["cx"], b["cy"], b["w"], b["d"], b["yaw"])
        rows.append([b["id"], b["kind"], round(x0, 3), round(x1, 3), round(y0, 3), round(y1, 3),
                     round(x1 - x0, 3), round(y1 - y0, 3), WALL_H])
    return rows


def route_rows(scene):
    pts = scene["route"]
    rows = []
    total = 0.0
    for i, (name, x, y) in enumerate(pts):
        leg = 0.0
        if i:
            px, py = pts[i - 1][1], pts[i - 1][2]
            leg = math.hypot(x - px, y - py)
            total += leg
        rows.append([i, name, f"({x:.2f}, {y:.2f})", f"{leg:.2f}", f"{total:.2f}"])
    return rows, total


def write_full_spec(scene, path, idx):
    L = []
    L.append(f"## {idx}. {scene['title']} —— {scene['subtitle']}\n")
    L.append(f"运行参数：`clearance:=0.60`，门宽一律 2.30 m（0.90 m 门洞墙厚，P2 样式），栅格 0.10 m，基础墙厚 0.15 m、墙高 2.50 m。\n")

    L.append("### 房间\n")
    L.append(md_table(["id", "名称", "x0", "x1", "y0", "y1", "面积 m²", "S_env(门口)", "门口可见", "期望"],
                      [[r[0], r[1], r[2], r[3], r[4], r[5],
                        f"{(r[3]-r[2])*(r[5]-r[4]):.1f}", f"{r[6]:.2f}", r[7], r[8]]
                       for r in scene["rooms"]]))
    L.append("")

    L.append("### 门洞（含观察位）\n")
    rows = []
    for did, room, axis, at, center, width, s in scene["doors"]:
        if axis == "H":
            sign = 1.0 if at < scene["spawn"][1] else -1.0
            peek = (center, at + sign * scene["peek_dist"])
            wall = f"y={at}（{axis}）"
        else:
            peek = (at - scene["peek_dist"], center)
            wall = f"x={at}（{axis}）"
        rows.append([did, room, wall, f"{center - width/2:.2f} – {center + width/2:.2f}",
                     width, f"({peek[0]:.2f}, {peek[1]:.2f})", f"{s:.2f}"])
    L.append(md_table(["id", "房间", "所在墙", "洞口范围", "宽", "门口观察位", "S_env"], rows))
    L.append("")

    L.append("### 墙体（矩形包围盒，可直接建 box）\n")
    L.append(md_table(["id", "类型", "x0", "x1", "y0", "y1", "dx", "dy", "高"],
                      wall_rects(scene)))
    L.append("")

    L.append("### 物体三维位姿（z 为底面高度，z_top = z + 高）\n")
    L.append(md_table(["id", "模型", "房间", "x", "y", "z", "yaw°", "roll°", "pitch°",
                       "尺寸 w×d×h", "z_top", "支撑面", "作用", "说明"],
                      obj_rows3(scene)))
    L.append("")

    L.append("### 观察点与视线验证\n")
    tv = scene["target_view"]
    tgt = scene["target_xy"]
    L.append(md_table(["项", "值"], [
        ["相机高度", "0.60 m"],
        ["相机内参", "640×360，HFOV 90°，垂直 FOV 58.7°"],
        ["目标", "{} @ ({:.2f}, {:.2f}, z={:.3f})".format(
            scene["target"], tgt[0], tgt[1],
            next(o[5] for o in scene["objects"] if o[6] == "target"))],
        ["观察圈位置", f"({tv[0]:.2f}, {tv[1]:.2f})"],
        ["观察距离", f"{math.hypot(tv[0]-tgt[0], tv[1]-tgt[1]):.2f} m"],
        ["门口 → 目标", "被遮挡（脚本 2D 视线验证）"],
        ["观察圈 → 目标", "无遮挡"],
    ]))
    L.append("")

    rows, total = route_rows(scene)
    L.append("### 期望路径\n")
    L.append(md_table(["#", "节点", "(x, y)", "本段 m", "累计 m"], rows))
    L.append(f"\n总路程 **{total:.1f} m**（0.5 m/s ≈ {total/0.5:.0f} s 纯行驶）\n")

    with open(path, "a", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


def main():
    os.makedirs(os.path.join(ROOT, "tables"), exist_ok=True)
    os.makedirs(os.path.join(ROOT, "figures"), exist_ok=True)
    os.makedirs(os.path.join(ROOT, "tasks"), exist_ok=True)
    md = ["# 生成表（自动产出）", ""]
    for name, scene in SCENES.items():
        rw, rd, rr, ro = write_tables(scene, os.path.join(ROOT, "tables"))
        draw_scene(scene, os.path.join(ROOT, "figures", f"{name}.png"))
        md += [f"## {name}", "", "**墙体（中心线坐标，厚 0.15、高 2.5）**", "",
               md_table(["id", "orient", "cx", "cy", "len", "note"], [[r[1], r[2], r[3], r[4], r[5], r[8]] for r in rw]), "",
               "**门洞（默认 {:.2f} m；door_widths 可逐门覆盖）**".format(DOOR_W), "",
               md_table(["id", "room", "axis", "at", "center", "width", "S_env"],
                        [[r[1], r[2], r[3], r[4], r[5], r[6], r[7]] for r in rd]), "",
               "**房间**", "",
               md_table(["id", "name", "x0", "x1", "y0", "y1", "area", "S_env", "见", "期望"],
                        [[r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]] for r in rr]), "",
               "**物体**", "",
               md_table(["id", "model", "x", "y", "yaw", "z", "size", "role", "note"],
                        [[r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9]] for r in ro]), "",
               "**可达性自检（门口 2D 视线）**", "",
               md_table(["door", "room", "peek(x,y)", "门口可见 env 物体", "S_env", "期望"],
                        [[v["door"], v["room"], f'({v["peek"][0]:.1f}, {v["peek"][1]:.1f})',
                          ", ".join(v["seen"]) or "无", f'{v["s"]:.2f}',
                          "进入" if v["s"] >= 0.4 else "跳过"] for v in validate(scene, 1.2 if name.startswith("env05") else 1.5)]), "",
               "**目标可见性**", "",
               md_table(["门", "目标", "门口能直接看到?", "观察圈能看到?"],
                        [[v["door"], scene["target"], "否(被挡)" if v["target_blocked"] else "是",
                          "是" if v["target_seen_at_view"] else "否"] for v in validate(scene, 1.2 if name.startswith("env05") else 1.5)
                         if v["room"] == scene["target_room"]]), ""]
    with open(os.path.join(ROOT, "tables.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md))
    print("tables + figures written")

    tasks = {
        "ladder.json": {
            "task_parsed": {
                "target": {
                    "name": "ladder",
                    "attributes": ["metal", "folding", "A-frame",
                                   "standing on the floor",
                                   "leaning against a warehouse wall"],
                    "typical_locations": ["warehouses", "storage rooms",
                                          "stockrooms", "maintenance areas"],
                },
                "env": {
                    "env_type_prior": ["warehouse", "storage room", "stockroom"],
                    "layout_prior": ["metal shelving along the walls",
                                     "stacks of cardboard boxes",
                                     "pallets and crates",
                                     "open loading floor"],
                    "related_structures": ["cardboard box", "shelf", "pallet"],
                },
                "obj_related": {
                    "key_objects": ["cardboard box", "shelf", "pallet", "crate"],
                    "search_strategy": "enter the storage room, walk around the partition, and check the shelving aisle beside the boxes",
                    "negative_clues": ["monitor", "computer", "laptop",
                                       "office chair", "desk", "desk lamp",
                                       "coffee machine", "door",
                                       "fire hydrant", "fire extinguisher",
                                       "construction cone", "traffic cone"],
                },
                "target_ctx": {"left": "cardboard box", "right": "shelf"},
            }
        },
        "cup_yellow_variant.json": {
            "task_parsed": {
                "target": {
                    "name": "yellow cup",
                    "attributes": ["yellow", "ceramic", "handle on the side",
                                   "medium-sized drinking cup", "on a pantry counter"],
                    "typical_locations": ["pantry counters", "break rooms",
                                          "near coffee machines"],
                },
                "env": {
                    "env_type_prior": ["office workspace", "shared working area"],
                    "layout_prior": ["counters along the walls", "cabinets and shelves",
                                     "open circulation space"],
                    "related_structures": ["office desk", "chair", "monitor"],
                },
                "obj_related": {
                    "key_objects": ["cup", "coffee machine", "other cups", "cardboard box"],
                    "search_strategy": "search pantry counters near the coffee machine and behind partitions",
                    "negative_clues": ["fire hydrant", "fire extinguisher", "door"],
                },
                "target_ctx": {"left": "coffee machine", "right": "cup"},
            }
        },
        "plant.json": {
            "task_parsed": {
                "target": {
                    "name": "plant",
                    "attributes": ["green", "potted", "tall indoor plant",
                                   "standing on the floor", "next to a desk"],
                    "typical_locations": ["office corners", "next to desks",
                                          "near windows", "reception areas"],
                },
                "env": {
                    "env_type_prior": ["office workspace", "shared working area"],
                    "layout_prior": ["desks along the walls",
                                     "counters and cabinets along the walls",
                                     "open circulation space"],
                    "related_structures": ["office desk", "chair", "monitor"],
                },
                "obj_related": {
                    "key_objects": ["cardboard box", "trash can", "desk", "chair", "monitor"],
                    "search_strategy": "check floor areas behind desks and counters, then round the partition",
                    "negative_clues": ["file cabinet", "cardboard box", "fire hydrant", "fire extinguisher", "door"],
                },
                "target_ctx": {"left": "desk", "right": "chair"},
            }
        },
    }
    for fn, payload in tasks.items():
        with open(os.path.join(ROOT, "tasks", fn), "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
    spec = os.path.join(ROOT, "design_spec.md")
    with open(spec, "w", encoding="utf-8") as f:
        f.write("# LSTE 新场景设计规格（由 build_design.py 生成，权威版）\n\n"
                "所有坐标都是 Gazebo 世界坐标系，单位 m。墙体为轴对齐 box，"
                "物体 z 为底面高度，z_top = z + 模型高度，roll/pitch 恒为 0。\n\n"
                "路径为**节点级参考**（用于估算里程/耗时），不是避障后的实际轨迹。\n\n")
    for i, name in enumerate(SCENES, start=1):
        write_full_spec(SCENES[name], spec, i)
    print("design_spec written")
    print("task json written")


if __name__ == "__main__":
    main()
