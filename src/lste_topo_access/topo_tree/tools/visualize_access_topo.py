#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize Access-Topo JSON dumps (Updated Color Scheme).

Update (2025-12-26):
- Only visualize *completed* backtrack sessions from backtrack_history (must have "finished").
- Ignore current/in-progress backtrack fields: current_backtrack, backtrack_start_pose, backtrack_path.
- Draw full completed backtrack segments in Amber by reconstructing path on the anchor "prev" tree
  using (start_anchor -> end_anchor). Fallback to sess["path"] if needed.
"""

import argparse
import glob
import json
import math
import os
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.collections import LineCollection

# --- Color Palette Configuration ---
COLOR_ANCHOR_POINTS = "#000000"
COLOR_BRANCH_SEARCH = "#00897B"
COLOR_BRANCH_TRACKBACK = "#FFB300"
COLOR_INTEREST = "#D32F2F"
COLOR_BACKTRACK_START = "#7B1FA2"
COLOR_ROBOT = "#FFEB3B"

COLOR_ANCHOR_PATH = "#B0BEC5"

COLOR_PATH_PASS = "#9EA7B3"
COLOR_PATH_SUS = "#455A64"
COLOR_PATH_LOCKED = "#D32F2F"

COLOR_STATE_SWITCH = "#00BCD4"


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def find_latest_json(tree_dir: str) -> Optional[str]:
    pattern = os.path.join(tree_dir, "**", "access_topo_*.json")
    files = glob.glob(pattern, recursive=True)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def build_anchor_map(nodes: List[dict]) -> Dict[int, dict]:
    return {int(n["id"]): n for n in nodes if "id" in n and "x" in n and "y" in n}


# ---------------------------
# Anchor-tree path utilities
# ---------------------------

def _get_prev_id(anchors: Dict[int, dict], aid: int) -> Optional[int]:
    node = anchors.get(aid)
    if not node:
        return None
    prev = node.get("prev")
    try:
        return int(prev) if prev is not None else None
    except Exception:
        return None


def _trace_to_root(anchors: Dict[int, dict], start_id: int, max_hops: int = 200000) -> List[int]:
    chain = []
    cur = start_id
    while cur is not None and cur in anchors and len(chain) < max_hops:
        chain.append(cur)
        cur = _get_prev_id(anchors, cur)
    return chain


def path_between_anchors_tree(anchors: Dict[int, dict], a: int, b: int) -> List[int]:
    if a not in anchors or b not in anchors:
        return []

    chain_a = _trace_to_root(anchors, a)
    chain_b = _trace_to_root(anchors, b)

    ancestors_a = {nid: i for i, nid in enumerate(chain_a)}
    lca = None
    idx_b = None
    for j, nid in enumerate(chain_b):
        if nid in ancestors_a:
            lca = nid
            idx_b = j
            break
    if lca is None:
        return []

    part_a = chain_a[: ancestors_a[lca] + 1]
    part_b = list(reversed(chain_b[: idx_b + 1]))
    if part_b and part_a[-1] == part_b[0]:
        part_b = part_b[1:]
    return part_a + part_b


def anchor_ids_to_xy(anchors: Dict[int, dict], ids: List[int]) -> List[Tuple[float, float]]:
    return [(anchors[i]["x"], anchors[i]["y"]) for i in ids if i in anchors]


# ---------------------------
# Plotting functions
# ---------------------------

def _state_to_path_color(state):
    try:
        s = int(state)
    except Exception:
        s = 0
    if s == 2:
        return COLOR_PATH_LOCKED
    if s == 1:
        return COLOR_PATH_SUS
    return COLOR_PATH_PASS


def plot_path(ax, anchors):
    segs, cols = [], []
    for n in anchors.values():
        if n.get("prev") is None:
            continue
        p = anchors.get(int(n["prev"]))
        if p is None:
            continue
        segs.append([(p["x"], p["y"]), (n["x"], n["y"])])
        cols.append(_state_to_path_color(n.get("lste_state")))

    if segs:
        ax.add_collection(LineCollection(segs, colors=cols, linewidths=2.6, alpha=0.85))

    ax.scatter(
        [n["x"] for n in anchors.values()],
        [n["y"] for n in anchors.values()],
        s=20,
        c=COLOR_ANCHOR_POINTS,
        zorder=3,
    )


def plot_branches(ax, anchors):
    ray_len = 1.2
    s_segs, t_segs = [], []

    for n in anchors.values():
        x0, y0 = n["x"], n["y"]
        for br in n.get("branches", []):
            h = float(br.get("heading_world", 0.0))
            x1, y1 = x0 + ray_len * math.cos(h), y0 + ray_len * math.sin(h)
            if br.get("status") == "TRACKBACK":
                t_segs.append([(x0, y0), (x1, y1)])
            else:
                s_segs.append([(x0, y0), (x1, y1)])

            ax.scatter([x1], [y1], s=80, marker="*", color=COLOR_INTEREST, zorder=4)

    if s_segs:
        ax.add_collection(LineCollection(s_segs, colors=COLOR_BRANCH_SEARCH, linewidths=2))
    if t_segs:
        ax.add_collection(LineCollection(t_segs, colors=COLOR_BRANCH_TRACKBACK, linewidths=2))


def plot_pose(ax, pose):
    if "x" in pose and "y" in pose:
        ax.scatter([pose["x"]], [pose["y"]], s=120, marker="*", color=COLOR_ROBOT, zorder=6)


def plot_backtrack(ax, data, anchors):
    segs = []
    for sess in data.get("backtrack_history", []):
        if sess.get("finished") is None:
            continue

        sp = sess.get("start_pose", {}) or {}
        if isinstance(sp, dict) and "x" in sp and "y" in sp:
            ax.scatter([sp["x"]], [sp["y"]], s=120, marker="*", color=COLOR_BACKTRACK_START)

        pts = []
        try:
            pts = anchor_ids_to_xy(
                anchors,
                path_between_anchors_tree(
                    anchors, int(sess["start_anchor"]), int(sess["end_anchor"])
                ),
            )
        except Exception:
            pass

        if len(pts) < 2:
            pts = anchor_ids_to_xy(anchors, sess.get("path", []))

        for i in range(len(pts) - 1):
            segs.append([pts[i], pts[i + 1]])

    if segs:
        ax.add_collection(
            LineCollection(segs, colors=COLOR_BRANCH_TRACKBACK, linewidths=3.0, alpha=0.55)
        )


def plot_state_switches(ax, data, anchors):
    for ev in data.get("state_events", []):
        aid = ev.get("anchor_id")
        if aid is not None and int(aid) in anchors:
            n = anchors[int(aid)]
            ax.text(
                n["x"], n["y"], "♻",
                color=COLOR_STATE_SWITCH,
                fontsize=14,
                ha="center",
                va="center",
                zorder=7,
            )


def create_custom_legend(ax):
    items = [
        mlines.Line2D([], [], marker="o", color="white", markerfacecolor=COLOR_ANCHOR_POINTS, label="anchor"),
        mlines.Line2D([], [], color=COLOR_PATH_PASS, lw=3, label="path (PASS)"),
        mlines.Line2D([], [], color=COLOR_PATH_SUS, lw=3, label="path (SUS)"),
        mlines.Line2D([], [], color=COLOR_PATH_LOCKED, lw=3, label="path (LOCKED)"),
        mlines.Line2D([], [], color=COLOR_BRANCH_SEARCH, lw=2, label="branch search"),
        mlines.Line2D([], [], color=COLOR_BRANCH_TRACKBACK, lw=2, label="branch trackback"),
        mlines.Line2D([], [], marker="*", color="white", markerfacecolor=COLOR_INTEREST, label="interest"),
        mlines.Line2D([], [], marker="*", color="white", markerfacecolor=COLOR_BACKTRACK_START, label="backtrack start"),
        mlines.Line2D([], [], marker="*", color="white", markerfacecolor=COLOR_ROBOT, label="robot pose"),
        mlines.Line2D([], [], marker="$♻$", color=COLOR_STATE_SWITCH, label="state switch"),
    ]

    ax.legend(
        handles=items,
        loc="lower left",
        bbox_to_anchor=(1.0, 0.0),   # ⭐ 右下角（轴外）
        bbox_transform=ax.transAxes,
        frameon=True,
        framealpha=0.95,
        fontsize=10,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json")
    parser.add_argument("--tree-dir", default=str(Path(__file__).resolve().parents[1] / "tree"))
    parser.add_argument("--save")
    args = parser.parse_args()

    json_path = args.json or find_latest_json(args.tree_dir)
    if not json_path:
        print("No json found.")
        return

    data = load_json(json_path)
    anchors = build_anchor_map(data.get("nodes", []))

    plt.figure(figsize=(10, 10))
    plt.subplots_adjust(right=0.78)  # ⭐ 给右侧 legend 腾空间
    ax = plt.gca()
    ax.set_aspect("equal")
    ax.set_facecolor("#f5f7fa")

    plot_path(ax, anchors)
    plot_branches(ax, anchors)
    plot_backtrack(ax, data, anchors)
    plot_pose(ax, data.get("pose", {}))
    plot_state_switches(ax, data, anchors)

    create_custom_legend(ax)

    ax.set_title(f"Access-Topo: {os.path.basename(json_path)}")
    ax.grid(True, linestyle="--", alpha=0.6)

    if args.save:
        plt.savefig(args.save, dpi=200)
    else:
        plt.show()


if __name__ == "__main__":
    main()
