#!/usr/bin/env python3
"""Plot measured routes, actual WI dispatches, and root-room priority order."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
import numpy as np

from plot_portal_authentication import read_metrics, ROOT
from plot_world_run import read_scene
sys.path.insert(0, str(ROOT / "scripts/tests/targeted_navigation"))
from audit_place_tsp import audit as tsp_audit
from audit_semantic_room_lifecycle import audit as room_audit

COLORS = ["#0072B2", "#D55E00", "#009E73", "#8B5597", "#9B7100", "#333333"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics", type=Path)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--center", nargs=2, type=float, required=True)
    parser.add_argument("--expected-rooms", type=int, required=True)
    args = parser.parse_args()
    tsp = tsp_audit(args.metrics)
    lifecycle = room_audit(args.metrics, args.center, 2.0, args.expected_rooms)
    events, samples, tree, _ = read_metrics(args.metrics)
    dispatches = []
    for name, data in events:
        if name != "work_item_dispatched":
            continue
        plan = data.get("graph_route_plan") or {}
        if not plan.get("work_item_tour") or plan.get("obligation_kind") != "work_item":
            continue
        item_id = data.get("work_item_id")
        item = next((w for w in data.get("work_items", []) if w.get("id") == item_id), {})
        attempt = next((a for a in item.get("attempts", [])
                        if a.get("id") == item.get("active_attempt_id")), {})
        dispatches.append({
            "index": len(dispatches) + 1, "time_s": data.get("ros_time"),
            "place_id": plan.get("current_place_id"), "work_item_id": item_id,
            "viewpoint_odom": attempt.get("viewpoint_xy"),
            "tour": plan["work_item_tour"], "route_id": data.get("route_id"),
        })
    places = sorted({d["place_id"] for d in dispatches if d["place_id"] is not None})
    colors = {place: COLORS[index % len(COLORS)] for index, place in enumerate(places)}
    fig = plt.figure(figsize=(15, 10), constrained_layout=True, facecolor="white")
    grid = fig.add_gridspec(2, 2, height_ratios=[3.7, 1.6], width_ratios=[1.35, 1])
    ax, side, timeline = fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1]), fig.add_subplot(grid[1, :])
    shapes, _, missing = read_scene(args.world, [Path.home()/".gazebo/models", ROOT/"worlds"])
    for shape in shapes:
        if shape.get("kind") == "wall":
            ax.add_patch(Polygon(shape["xy"], closed=True, facecolor="#39434B", lw=0.3, zorder=1))
    poses = np.asarray([p["pose"][:2] for p in samples], dtype=float).reshape(-1, 2)
    times = np.asarray([p.get("ros_time", float("nan")) for p in samples])
    if len(poses) > 1:
        segments = np.stack([poses[:-1], poses[1:]], axis=1)
        valid = np.isfinite(times[:-1]) & np.isfinite(times[1:]) & ((times[1:]-times[:-1]) <= 2.0)
        ax.add_collection(LineCollection(segments[valid], colors="#8699A8", linewidths=1, zorder=2))
    # Colored overlays are measured sample intervals, never lines between WI goals.
    starts = {}
    for name, data in events:
        room = data.get("active_semantic_room") or {}
        place = room.get("place_id")
        if name == "semantic_room_exploration_started":
            starts[place] = data.get("ros_time", 0)
        elif name == "semantic_room_closed" and place in starts and len(poses) > 1:
            lo, hi = starts.pop(place), data.get("ros_time", 0)
            mask = valid & (times[:-1] >= lo) & (times[1:] <= hi)
            ax.add_collection(LineCollection(segments[mask], colors=colors.get(place, "#333333"), linewidths=2, zorder=3))
    for dispatch in dispatches:
        xy = dispatch["viewpoint_odom"]
        if xy is None:
            continue
        color = colors[dispatch["place_id"]]
        ax.scatter(*xy, color=color, marker="s", edgecolors="white", s=45, zorder=5)
        ax.annotate(str(dispatch["index"]), xy=xy, xytext=(4, 5), textcoords="offset points",
                    color=color, fontsize=8, weight="bold", zorder=6)
    ax.scatter(*args.center, color="black", marker="*", s=110, zorder=7, label="Audited junction")
    if len(poses):
        ax.set_xlim(poses[:, 0].min()-2, poses[:, 0].max()+2)
        ax.set_ylim(poses[:, 1].min()-2, poses[:, 1].max()+2)
    ax.set_aspect("equal", adjustable="box")
    ax.set(xlabel="World / odometry x (m)", ylabel="World / odometry y (m)",
           title="Measured route; squares = dispatched WI viewpoints")
    ax.grid(alpha=0.15)
    ax.legend(loc="best", fontsize=8)
    side.axis("off")
    lines = ["Actual room exploration order (root junction)"]
    rooms = {r["branch_id"]: r for r in lifecycle.get("rooms", [])}
    for rank, branch in enumerate(lifecycle.get("actual_exploration_order", []), 1):
        room = rooms[branch]
        lines.append("%d. B%s / P%s   score %.3f   priority %s" %
                     (rank, branch, room.get("place_id"), room.get("semantic_score", 0), room.get("priority")))
    lines.extend(["", "Actual WI dispatches by Place (retries retained)"])
    for place in places:
        ids = ["%s:W%s" % (d["index"], d["work_item_id"]) for d in dispatches if d["place_id"] == place]
        lines.extend(textwrap.wrap("P%s: %s" % (place, " > ".join(ids)), 57))
    lines.extend(["", "Runtime acceptance checks"])
    lines.extend(("PASS " if ok else "FAIL ") + key.replace("_", " ") for key, ok in tsp["checks"].items())
    lines.append(("PASS " if lifecycle["passed"] else "FAIL ") + "room lifecycle and semantic order")
    lines.extend(["", "Numbers = actual dispatch index, not planned tour edges.",
                  "Grey = all measured motion; colors = room exploration.",
                  "Map crop follows recorded motion; raw log keeps full data."])
    side.text(0, 1, "\n".join(lines), va="top", fontsize=9, linespacing=1.5, family="DejaVu Sans")
    for place in places:
        selected = [d for d in dispatches if d["place_id"] == place]
        timeline.scatter([d["time_s"] for d in selected], [place]*len(selected),
                         color=colors[place], marker="s", s=35, label="P%s" % place)
        for ordinal, d in enumerate(selected):
            timeline.annotate("%s/W%s" % (d["index"], d["work_item_id"]),
                              (d["time_s"], place), xytext=(0, 7 + 11 * (ordinal % 3)), textcoords="offset points", fontsize=7, ha="center")
    timeline.set(xlabel="Simulation time (s)", ylabel="WI owner Place", yticks=places,
                 title="Actual dispatch timeline (index / WorkItem ID)")
    if places:
        timeline.set_ylim(min(places)-0.6, max(places)+max(1.0, 0.6*(max(places)-min(places)+1)))
    timeline.grid(alpha=0.15)
    verdict = "PASS" if tsp["passed"] and lifecycle["passed"] else "INCOMPLETE / FAIL"
    fig.suptitle("Place / TSP execution | %s | %s" % (args.world.stem, verdict), fontsize=16, weight="bold")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160, facecolor="white")
    plt.close(fig)
    provenance = {
        "metrics": str(args.metrics), "metrics_sha256": hashlib.sha256(args.metrics.read_bytes()).hexdigest(),
        "world": str(args.world), "dispatches": dispatches,
        "tsp_checks": tsp["checks"], "lifecycle_checks": lifecycle["checks"],
        "expected_room_order": lifecycle.get("expected_exploration_order"),
        "actual_room_order": lifecycle.get("actual_exploration_order"),
        "missing_geometry_models": missing,
        "transformations": "Raw odometry segments only; gaps >2s disconnected; colored intervals use room start/close events; crop to measured path.",
    }
    args.output.with_suffix(".json").write_text(json.dumps(provenance, indent=2)+"\n")
    print(args.output)


if __name__ == "__main__":
    main()
