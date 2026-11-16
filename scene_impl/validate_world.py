#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Self-check the generated worlds on bit1010 (no scipy)."""
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

sys.path.insert(0, "/home/zrz/work/lste_ws/src/lste_topo_access/scripts")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from global_frontier_topology_places import StructuralPlaceMap  # noqa: E402
from global_frontier_grid import conservative_clearance_cells, inflate  # noqa: E402
import build_design as bd  # noqa: E402

RES = 0.10
WALL_H = 2.5
WORLD_DIR = os.environ.get("WORLD_DIR", "/home/zrz/work/lste_ws/worlds/topo3.0_catch_mode/session_test")
CLEARANCE = float(os.environ.get("CHECK_CLEARANCE", "0.60"))
FALLBACK = 0.30
PLACE_CLEAR = CLEARANCE + min(0.20, max(0.10, FALLBACK / 2.0))


def load(path):
    w = ET.parse(path).getroot().find("world")
    walls, objs = [], []
    for m in w.findall("model"):
        for l in m.findall("link"):
            p = [float(v) for v in l.findtext("pose").split()]
            s = [float(v) for v in l.find("collision/geometry/box/size").text.split()]
            walls.append(dict(id=l.get("name"), cx=p[0], cy=p[1], w=s[0], d=s[1], yaw=0.0))
    for inc in w.findall("include"):
        name, uri = inc.findtext("name"), inc.findtext("uri").replace("model://", "")
        if uri not in bd.MODELS:
            continue
        p = [float(v) for v in inc.findtext("pose").split()]
        ww, dd, hh = bd.MODELS[uri]
        objs.append(dict(id=name, model=uri, cx=p[0], cy=p[1], z=p[2], yaw=p[5], w=ww, d=dd, h=hh))
    return walls, objs


def aabb(o):
    c, s = abs(math.cos(o["yaw"])), abs(math.sin(o["yaw"]))
    hw, hd = (o["w"] * c + o["d"] * s) / 2, (o["w"] * s + o["d"] * c) / 2
    return (o["cx"] - hw, o["cy"] - hd, o["cx"] + hw, o["cy"] + hd)


def ov(a, b):
    return min(a[2], b[2]) - max(a[0], b[0]), min(a[3], b[3]) - max(a[1], b[1])


def raster(scene, walls, margin=1.0):
    x0, y0, x1, y1 = scene["bbox"]
    ox, oy = x0 - margin, y0 - margin
    W = int(round((x1 - x0 + 2 * margin) / RES)) + 1
    H = int(round((y1 - y0 + 2 * margin) / RES)) + 1
    occ = np.zeros((H, W), bool)
    for wl in walls:
        a = aabb(wl)
        c0, c1 = int(round((a[0] - ox) / RES)), int(round((a[2] - ox) / RES))
        r0, r1 = int(round((a[1] - oy) / RES)), int(round((a[3] - oy) / RES))
        occ[max(0, r0):max(0, r1 + 1), max(0, c0):max(0, c1 + 1)] = True
    return occ, ox, oy


def cell(x, y, ox, oy):
    return int(round((y - oy) / RES)), int(round((x - ox) / RES))


def bfs(free, start):
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
    return seen


def main():
    ok = True
    for name, scene in bd.SCENES.items():
        path = os.path.join(WORLD_DIR, name + ".world")
        walls, objs = load(path)
        print("\n=== %s : %d wall links, %d objects" % (name, len(walls), len(objs)))

        x0, y0, x1, y1 = scene["bbox"]
        for wl in walls:
            a = aabb(wl)
            if not (x0 - 0.2 <= a[0] and a[2] <= x1 + 0.2 and y0 - 0.2 <= a[1] and a[3] <= y1 + 0.2):
                print("  [FAIL] wall outside bbox:", wl["id"]); ok = False

        for did, room, axis, at, center, w, s in scene["doors"]:
            strip = (center - w / 2, at - 0.25, center + w / 2, at + 0.25) if axis == "H" \
                else (at - 0.25, center - w / 2, at + 0.25, center + w / 2)
            bad = [wl["id"] for wl in walls if (lambda d: d[0] > 0.02 and d[1] > 0.02)(ov(aabb(wl), strip))]
            print("  door %-5s %-3s w=%.2f  blocking walls: %s" % (did, room, w, bad or "none"))
            if bad:
                ok = False

        bad_pairs = 0
        for i in range(len(objs)):
            for j in range(i + 1, len(objs)):
                a, b = objs[i], objs[j]
                dx, dy = ov(aabb(a), aabb(b))
                dz = min(a["z"] + a["h"], b["z"] + b["h"]) - max(a["z"], b["z"])
                if dx > 0.005 and dy > 0.005 and dz > 0.005:
                    print("  [FAIL] 3-D overlap %s <-> %s (dx=%.3f dy=%.3f dz=%.3f)"
                          % (a["id"], b["id"], dx, dy, dz))
                    bad_pairs += 1
        for o in objs:
            for wl in walls:
                dx, dy = ov(aabb(o), aabb(wl))
                if dx > 0.005 and dy > 0.005 and o["z"] < WALL_H:
                    print("  [FAIL] object inside wall %s <-> %s" % (o["id"], wl["id"]))
                    bad_pairs += 1
        print("  object/wall interpenetrations:", bad_pairs)
        if bad_pairs:
            ok = False

        occ, ox, oy = raster(scene, walls)
        # floor-standing furniture is a navigation obstacle; things on a desk
        # (z >= 0.7) are above the robot body and must not block the route.
        occ_nav = occ.copy()
        for o in objs:
            if o["z"] < 0.70:
                a = aabb(o)
                c0, c1 = int(round((a[0] - ox) / RES)), int(round((a[2] - ox) / RES))
                r0, r1 = int(round((a[1] - oy) / RES)), int(round((a[3] - oy) / RES))
                occ_nav[max(0, r0):max(0, r1 + 1), max(0, c0):max(0, c1 + 1)] = True
        free_all = ~occ
        # closure: flood from outside must not reach the interior
        outside = np.zeros_like(free_all)
        q = deque()
        for c in range(free_all.shape[1]):
            for r in (0, free_all.shape[0] - 1):
                if free_all[r, c]:
                    outside[r, c] = True; q.append((r, c))
        for r in range(free_all.shape[0]):
            for c in (0, free_all.shape[1] - 1):
                if free_all[r, c] and not outside[r, c]:
                    outside[r, c] = True; q.append((r, c))
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < free_all.shape[0] and 0 <= nc < free_all.shape[1] \
                        and free_all[nr, nc] and not outside[nr, nc]:
                    outside[nr, nc] = True; q.append((nr, nc))
        sx, sy = scene["spawn"][0], scene["spawn"][1]
        sc = cell(sx, sy, ox, oy)
        if outside[sc]:
            print("  [FAIL] the building leaks: outside reaches the spawn"); ok = False
        else:
            print("  closure: OK (outside cannot reach the interior)")

            # reachability through the route mask (clearance 0.60)
            route = (~occ_nav) & ~inflate(occ_nav, conservative_clearance_cells(CLEARANCE, RES))
            seen = bfs(route, sc)
            miss = []
            for rid, rname, rx0, rx1, ry0, ry1, s, terms, exp in scene["rooms"]:
                if s <= 0.0:
                    # Decoy/negative rooms are observation-only.
                    continue
                cx, cy = (rx0 + rx1) / 2, (ry0 + ry1) / 2
                # best-clearance cell inside the room
                best, bestc = -1, None
                for r in range(free_all.shape[0]):
                    y = oy + r * RES
                    if not (ry0 + 0.5 <= y <= ry1 - 0.5):
                        continue
                    for c in range(free_all.shape[1]):
                        x = ox + c * RES
                        if not (rx0 + 0.5 <= x <= rx1 - 0.5) or not route[r, c]:
                            continue
                        d = min(r, c, free_all.shape[0] - r, free_all.shape[1] - c)
                        if d > best:
                            best, bestc = d, (r, c)
                if bestc is None or not seen[bestc]:
                    miss.append(rid)
            print("  route (clearance %.2f) reaches all rooms:" % CLEARANCE, "YES" if not miss else miss)
            if miss:
                ok = False

            # target observation ring must be reachable and unobstructed
            tvx, tvy = scene["target_view"]
            tv = cell(tvx, tvy, ox, oy)
            if not route[tv]:
                print("  [FAIL] target view %s is not a free route cell" % ((tvx, tvy),)); ok = False
            elif not seen[tv]:
                print("  [FAIL] target view %s not reachable from spawn" % ((tvx, tvy),)); ok = False
            else:
                print("  target view (%.2f, %.2f): free + reachable" % (tvx, tvy))

        # place split
        m = StructuralPlaceMap(free_all, occ, RES, PLACE_CLEAR, 2.5, 1)
        start_label = int(m.labels[sc])
        room_labels = {}
        for rid, rname, rx0, rx1, ry0, ry1, s, terms, exp in scene["rooms"]:
            if s <= 0.0:
                # Negative/zero-score decoy rooms are observation-only; the
                # mission never needs to enter them, so do not fail the route
                # audit on a chair/desk pinch behind a decoy doorway.
                continue
            cx, cy = (rx0 + rx1) / 2, (ry0 + ry1) / 2
            # use the room's most-open cell
            candidates = []
            for r in range(free_all.shape[0]):
                y = oy + r * RES
                if not (ry0 + 0.5 <= y <= ry1 - 0.5):
                    continue
                for c in range(free_all.shape[1]):
                    x = ox + c * RES
                    if not (rx0 + 0.5 <= x <= rx1 - 0.5):
                        continue
                    if int(m.labels[r, c]) > 0:
                        candidates.append((r, c))
            bestc = None
            if candidates:
                bestc = min(
                    candidates,
                    key=lambda rc: ((ox + rc[1] * RES) - cx) ** 2
                    + ((oy + rc[0] * RES) - cy) ** 2,
                )
            room_labels[rid] = int(m.labels[bestc]) if bestc else 0
        split = len({v for v in room_labels.values() if v not in (0, start_label)})
        print("  place split: %d/%d rooms separate (labels %s, corridor=%d)"
              % (split, len(room_labels), room_labels, start_label))
        if split != len(room_labels):
            print("  [WARN] not every room becomes its own Place")

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
