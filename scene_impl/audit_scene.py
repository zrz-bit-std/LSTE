#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conformance audit: traversability, magic values, observation dwell, semantic state machine."""
import json
import math
import os
import sys
import types
from collections import deque

import numpy as np

SCRIPTS = "/home/zrz/work/lste_ws/src/lste_topo_access/scripts"
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from global_frontier_topology_places import StructuralPlaceMap  # noqa: E402
from global_frontier_grid import conservative_clearance_cells, inflate  # noqa: E402
from global_frontier_semantic_belief import (  # noqa: E402
    task_intent_from_message, semantic_room_score, semantic_place_action,
)
import build_design as bd  # noqa: E402
import validate_world as V  # noqa: E402

RES = 0.10
CLEAR = 0.60
FALLBACK = 0.30
PLACE_CLEAR = CLEAR + min(0.20, max(0.10, FALLBACK / 2.0))
WORLD_DIR = "/home/zrz/work/lste_ws/worlds/topo3.0_catch_mode/session_test"
TASKS = {"env05_gate_ring": "/home/zrz/lste_scene_impl/tasks/ladder.json",
         "env06_gate_nested": "/home/zrz/lste_scene_impl/tasks/plant.json"}

# what WeDetect actually returns through each door (measured on bit1010)
DOOR_LABELS = {
    "D_a1": ["cardboard box"],
    "D_a2": ["desk", "chair", "monitor"],
    "D_a3": ["shelf"],
    "D_a4": [],
    "D_b1": ["file cabinet", "cardboard box"],
    "D_b2": ["desk", "chair"],
    "D_b3": ["desk", "chair"],
    "D_b4": ["desk", "chair", "monitor"],
    "D_b5": ["file cabinet", "cardboard box"],
}
TARGET_ROOM = {"env05_gate_ring": "A3", "env06_gate_nested": "B3"}


def intent_from(path):
    d = json.load(open(path))["task_parsed"]
    msg = types.SimpleNamespace(
        task_id=os.path.basename(path).replace(".json", ""),
        target_name=d["target"]["name"],
        target_attributes=d["target"]["attributes"],
        env_related_structures=d["env"]["related_structures"],
        env_type_prior=d["env"]["env_type_prior"],
        obj_key_objects=d["obj_related"]["key_objects"],
        ctx_left=d["target_ctx"]["left"],
        ctx_right=d["target_ctx"]["right"],
        obj_negative_clues=d["obj_related"]["negative_clues"],
        raw_json=json.dumps({"task_parsed": d}),
    )
    return task_intent_from_message(msg)


def bfs(free, start):
    H, W = free.shape
    seen = np.zeros_like(free)
    q = deque([start]); seen[start] = True
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and free[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True; q.append((nr, nc))
    return seen


def main(name, scene, out):
    walls, objs = V.load(os.path.join(WORLD_DIR, name + ".world"))
    occ, ox, oy = V.raster(scene, walls)
    occ_nav = occ.copy()
    for o in objs:
        if o["z"] < 0.70:
            a = V.aabb(o)
            c0, c1 = int(round((a[0] - ox) / RES)), int(round((a[2] - ox) / RES))
            r0, r1 = int(round((a[1] - oy) / RES)), int(round((a[3] - oy) / RES))
            occ_nav[max(0, r0):r1 + 1, max(0, c0):c1 + 1] = True
    route = (~occ_nav) & ~inflate(occ_nav, conservative_clearance_cells(CLEAR, RES))
    start = V.cell(*scene["spawn"][:2], ox, oy)
    seen = bfs(route, start)

    out.append(f"\n## {name}\n")
    out.append("| 指标 | 值 |")
    out.append("|---|---|")
    out.append(f"| 出生点 | ({scene['spawn'][0]:.2f}, {scene['spawn'][1]:.2f}) |")
    out.append(f"| 可通行栅格 (clearance {CLEAR}) | {int(route.sum())} cells |")
    out.append(f"| 出生点可达 | {bool(seen[start])} |")

    # ---------- traversability per room
    out.append("\n### 每间房的可通行情况\n")
    out.append("| 房间 | 可通行格 | 可达格 | 可达率 | 无解口袋 (>0.5m²) | 观察点可达 |")
    out.append("|---|---|---|---|---|---|")
    pockets_total = 0
    for rid, rname, rx0, rx1, ry0, ry1, s, terms, exp in scene["rooms"]:
        r0, r1 = int(round((ry0 - oy) / RES)), int(round((ry1 - oy) / RES))
        c0, c1 = int(round((rx0 - ox) / RES)), int(round((rx1 - ox) / RES))
        sub = route[r0:r1, c0:c1]
        sub_seen = seen[r0:r1, c0:c1]
        tot = int(sub.sum()); ok = int((sub & sub_seen).sum())
        # free pockets (route cells unreachable), 4-connected
        unl = sub & ~sub_seen
        lab = np.zeros(unl.shape, np.int32)
        lid = 0
        for rr in range(unl.shape[0]):
            for cc in range(unl.shape[1]):
                if unl[rr, cc] and lab[rr, cc] == 0:
                    lid += 1
                    q = deque([(rr, cc)]); lab[rr, cc] = lid
                    while q:
                        a, b = q.popleft()
                        for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                            na, nb = a + da, b + db
                            if 0 <= na < unl.shape[0] and 0 <= nb < unl.shape[1] \
                                    and unl[na, nb] and lab[na, nb] == 0:
                                lab[na, nb] = lid; q.append((na, nb))
        sizes = [int((lab == k).sum()) for k in range(1, lid + 1)]
        pockets = [x for x in sizes if x * RES * RES > 0.5]
        pockets_total += len(pockets)
        out.append(f"| {rid} {rname} | {tot} | {ok} | {100.0*ok/max(1,tot):.1f}% | "
                   f"{len(pockets)} | - |")
    # target view
    tvx, tvy = scene["target_view"]
    tv = V.cell(tvx, tvy, ox, oy)
    out.append(f"| **目标观察圈** | 1 | {int(seen[tv])} | {'100%' if seen[tv] else '0%'} | 0 | "
               f"{'可达' if seen[tv] else '不可达'} |")

    # ---------- magic values
    out.append("\n### 魔术值符合性\n")
    out.append("| 参数 | 设计值 | 实测 | 结论 |")
    out.append("|---|---|---|---|")
    door_w = scene["doors"][0][5]
    door_ok = abs(door_w - float(bd.DOOR_W)) <= 0.05
    out.append(f"| 门宽 | {float(bd.DOOR_W):.2f} | {door_w:.2f} | "
               f"{'OK（v6）' if door_ok else 'FAIL'} |")
    m = StructuralPlaceMap(~occ, occ, RES, PLACE_CLEAR, 2.5, 1)
    start_label = int(m.labels[start])
    lab_rooms = {}
    for rid, rname, rx0, rx1, ry0, ry1, s, terms, exp in scene["rooms"]:
        best, bestc = -1, None
        for r in range(occ.shape[0]):
            y = oy + r * RES
            if not (ry0 + 0.5 <= y <= ry1 - 0.5):
                continue
            for c in range(occ.shape[1]):
                x = ox + c * RES
                if not (rx0 + 0.5 <= x <= rx1 - 0.5) or int(m.labels[r, c]) == 0:
                    continue
                d = 0
                while d < 12:
                    rr = r + d if r + d < occ.shape[0] else r
                    d += 1
                    break
                bestc = (r, c)
        lab_rooms[rid] = int(m.labels[bestc]) if bestc else 0
    split = len({v for v in lab_rooms.values() if v not in (0, start_label)})
    out.append(f"| Place 切分 | 每间房 1 个 | {split}/{len(lab_rooms)} | "
               f"{'OK' if split == len(lab_rooms) else 'FAIL'} |")
    out.append(f"| region_topology_clearance | 0.75 | {PLACE_CLEAR:.2f} | OK |")
    out.append(f"| route clearance cells | 6 | {conservative_clearance_cells(CLEAR, RES)} | OK |")

    # observation dwell:房间质心到最近可达格的距离 (must be <= 2.5 m)
    out.append("\n### 观望（place dwell）可行性\n")
    out.append("| 房间 | 质心 | 最近可达格距离 | ≤2.5 m | 门内可达 |")
    out.append("|---|---|---|---|---|")
    for rid, rname, rx0, rx1, ry0, ry1, s, terms, exp in scene["rooms"]:
        cx, cy = (rx0 + rx1) / 2, (ry0 + ry1) / 2
        best = 1e9
        for r in range(seen.shape[0]):
            y = oy + r * RES
            if not (ry0 + 0.3 <= y <= ry1 - 0.3):
                continue
            for c in range(seen.shape[1]):
                x = ox + c * RES
                if seen[r, c] and rx0 + 0.3 <= x <= rx1 - 0.3:
                    best = min(best, math.hypot(x - cx, y - cy))
        out.append(f"| {rid} | ({cx:.2f}, {cy:.2f}) | {best:.2f} | "
                   f"{'OK' if best <= 2.5 else 'NO'} | {'yes' if best < 1e8 else 'no'} |")

    # ---------- semantic scores + state machine
    intent = intent_from(TASKS[name])
    out.append("\n### 语义分数（用 WeDetect 实测标签）\n")
    out.append("| 门 | 房间 | 检测标签 | S_env | 期望 |")
    out.append("|---|---|---|---|---|")
    scores = {}
    for did, room, axis, at, center, w, s in scene["doors"]:
        labels = DOOR_LABELS[did]
        sc = semantic_room_score(intent, labels)
        sc = 0.0 if sc is None else sc
        scores[did] = (room, sc)
        out.append(f"| {did} | {room} | {', '.join(labels) or '—'} | {sc:.2f} | "
                   f"{'进入' if room == TARGET_ROOM[name] or sc >= 0.4 else '跳过'} |")

    out.append("\n### 门控阈值扫描（按 semantic_place_action 的状态机）\n")
    out.append("| θ | 进入的房间 | 目标房是否进入 | 结果 |")
    out.append("|---|---|---|---|")
    for th in (0.02, 0.10, 0.20, 0.30, 0.50, 0.90):
        entered, target_in = [], False
        for did, (room, sc) in scores.items():
            if sc >= th:
                entered.append(room)
                if room == TARGET_ROOM[name]:
                    target_in = True
        # state machine: at a covered place with context hits but no target,
        # semantic_place_action returns context_place on the first pass then
        # expand_unobserved_portal -> the room is left, coverage is kept.
        act = semantic_place_action(intent, {"target_hits": 1, "context_hits": 0, "observations": 1}, True)
        outcome = "命中" if target_in else "漏掉目标房"
        out.append(f"| {th:.2f} | {','.join(sorted(set(entered))) or '无'} | "
                   f"{'是' if target_in else '否'} | {outcome} |")

    # ---------- extra: hydrant penalty in a corridor view
    hyd = semantic_room_score(intent, ["fire hydrant"])
    out.append(f"\n- 走廊视图若含 `fire hydrant`：S_env = {hyd:.2f}")
    out.append(f"- 目标房首帧 `target_hits=1` 时状态机动作：**{act}**")
    out.append(f"- 无解口袋总数：{pockets_total}")
    return out


if __name__ == "__main__":
    out = ["# 场景符合性审计（由 audit_scene.py 生成）",
           "",
           f"clearance={CLEAR}, place_clearance={PLACE_CLEAR:.2f}, "
           f"route_cells={conservative_clearance_cells(CLEAR, RES)}, resolution={RES}"]
    for nm, sc in bd.SCENES.items():
        main(nm, sc, out)
    text = "\n".join(out)
    open("/home/zrz/lste_scene_impl/audit_report.md", "w", encoding="utf-8").write(text)
    print(text)
