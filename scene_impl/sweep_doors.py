#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sweep the physical door width so the *existing* Place/portal machinery fires.

No runtime parameter is changed: clearance stays 0.52 (production), so
  route radius  = conservative_clearance_cells(0.52, 0.10) = 6 cells
  place cut    = StructuralPlaceMap(place_clearance = 0.52+0.15 = 0.67)
For each door width W we rasterise the real scene at 0.10 m and ask:
  (a) does the room become its own structural Place?
  (b) is it still reachable from the spawn through the inflated route mask?
"""
import math
import os
import sys

import numpy as np
from collections import deque

SCRIPTS = "/home/zrz/work/lste_ws/src/lste_topo_access/scripts"
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from global_frontier_topology_places import StructuralPlaceMap  # noqa: E402
from global_frontier_grid import conservative_clearance_cells, inflate  # noqa: E402
import build_design as bd  # noqa: E402

RES = 0.10
CLEARANCE = float(os.environ.get("SWEEP_CLEARANCE", "0.60"))
FALLBACK = 0.30            # production launch value
PLACE_CLEAR = CLEARANCE + min(0.20, max(0.10, FALLBACK / 2.0))


def boxes(scene, door_w):
    """Wall boxes with every door opening forced to door_w."""
    out = []
    for wid, kind, at, a, b, _openings in scene["walls"]:
        cuts = []
        for did, room, axis, dat, dcenter, dw, s in scene["doors"]:
            if (axis, dat) == (kind, at) and a <= dcenter <= b:
                cuts.append((dcenter - door_w / 2, dcenter + door_w / 2))
        cuts.sort()
        start = a
        pieces = []
        for c0, c1 in cuts:
            if c0 > start:
                pieces.append((start, c0))
            start = max(start, c1)
        if start < b:
            pieces.append((start, b))
        for p0, p1 in pieces:
            if kind == "H":
                out.append((p0, at - 0.075, p1, at + 0.075))
            else:
                out.append((at - 0.075, p0, at + 0.075, p1))
    for name, x0, x1, y0, y1 in scene["solids"]:
        out.append((x0, y0, x1, y1))
    return out


def raster(scene, rects, margin=1.0):
    x0, y0, x1, y1 = scene["bbox"]
    ox, oy = x0 - margin, y0 - margin
    W = int(round((x1 - x0 + 2 * margin) / RES)) + 1
    H = int(round((y1 - y0 + 2 * margin) / RES)) + 1
    occ = np.zeros((H, W), bool)
    for (a0, b0, a1, b1) in rects:
        c0 = int(round((a0 - ox) / RES))
        c1 = int(round((a1 - ox) / RES))
        r0 = int(round((b0 - oy) / RES))
        r1 = int(round((b1 - oy) / RES))
        occ[max(0, r0):max(0, r1 + 1), max(0, c0):max(0, c1 + 1)] = True
    return occ, ox, oy


def cell(x, y, ox, oy):
    return int(round((y - oy) / RES)), int(round((x - ox) / RES))


def reachable(free, start, goals):
    H, W = free.shape
    seen = np.zeros_like(free)
    q = deque([start])
    seen[start] = True
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and free[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                q.append((nr, nc))
    return [seen[g] for g in goals]


def clearance_map(occ, max_cells=16):
    """Largest inflation radius (cells) for which each cell stays free."""
    clear = np.zeros(occ.shape, np.int16)
    for d in range(1, max_cells + 1):
        inf = inflate(occ, d)
        clear = np.where(~inf, np.int16(d), clear)
    return clear


def room_goal(clear, scene_room, ox, oy, occ):
    rid, name, x0, x1, y0, y1, s, terms, exp = scene_room
    H, W = clear.shape
    best = (-1, None)
    for r in range(H):
        y = oy + r * RES
        if not (y0 + 0.4 <= y <= y1 - 0.4):
            continue
        for c in range(W):
            x = ox + c * RES
            if not (x0 + 0.4 <= x <= x1 - 0.4):
                continue
            if clear[r, c] > best[0]:
                best = (int(clear[r, c]), (r, c))
    return best


def evaluate(scene, door_w):
    rects = boxes(scene, door_w)
    occ, ox, oy = raster(scene, rects)
    known_free = ~occ
    m = StructuralPlaceMap(known_free, occ, RES, PLACE_CLEAR, 2.5, 1)
    clear = clearance_map(occ)
    start = cell(scene["spawn"][0], scene["spawn"][1], ox, oy)
    rooms, goals, splits, room_clear = [], [], 0, []
    start_label = int(m.labels[start])
    for room in scene["rooms"]:
        cl, goal = room_goal(clear, room, ox, oy, occ)
        rooms.append((room[0], goal))
        room_clear.append((room[0], cl * RES))
        goals.append(goal)
        if goal is not None and int(m.labels[goal]) not in (0, start_label):
            splits += 1
    free = known_free & ~inflate(occ, conservative_clearance_cells(CLEARANCE, RES))
    ok = reachable(free, start, goals + [start])
    return dict(occ=occ, labels=m.labels, split=splits, nrooms=len(rooms),
                route_all=all(ok), route_ok=sum(ok), free=free, start=start,
                rooms=rooms, clear=room_clear,
                comps=len([1 for l, d in m.components.items() if d["cells"] > 200]))


def main():
    print("clearance=%.2f place_clearance=%.2f route_cells=%d"
          % (CLEARANCE, PLACE_CLEAR, conservative_clearance_cells(CLEARANCE, RES)))
    widths = [round(1.10 + 0.01 * i, 2) for i in range(36)]
    for name, scene in bd.SCENES.items():
        print("\n=== %s  (%d rooms, %d doors)" % (name, len(scene["rooms"]), len(scene["doors"])))
        print(" W     split  comps  reach  verdict")
        best = []
        for w in widths:
            r = evaluate(scene, w)
            good = (r["split"] == r["nrooms"] and r["route_all"])
            detail = ",".join("%s:%.1f" % (rid, cl) for rid, cl in r["clear"])
            print(" %.2f   %d/%d    %2d    %d/%d   %s  [%s]"
                  % (w, r["split"], r["nrooms"], r["comps"], r["route_ok"], r["nrooms"] + 1,
                     "OK" if good else "-", detail))
            if good:
                best.append(w)
        print(" -> feasible widths:", best if best else "NONE")


if __name__ == "__main__":
    main()
