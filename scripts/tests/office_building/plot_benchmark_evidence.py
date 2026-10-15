#!/usr/bin/env python3
"""Render a deterministic office floor plan with benchmark robot trajectories."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import yaml


EVENT_RE = re.compile(r"event=(\S+) data=(\{.*\})$")
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MANIFEST = ROOT / "worlds/benchmark/office_building_v1_manifest.yaml"


def read_samples(path: Path) -> list[tuple[float, float]]:
    points = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            match = EVENT_RE.search(line.rstrip())
            if not match or match.group(1) != "sample":
                continue
            try:
                data = json.loads(match.group(2))
            except json.JSONDecodeError:
                continue
            pose = data.get("pose") or []
            if len(pose) >= 2:
                points.append((float(pose[0]), float(pose[1])))
    return points


def render(manifest_path: Path, logs: list[Path], output: Path) -> None:
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    bounds = manifest["building"]["bounds_m"]
    target = manifest["targets"]["primary"]["pose_xy"]
    profiles = manifest.get("robot_profiles", {})

    fig, ax = plt.subplots(figsize=(12, 8), dpi=160)
    colors = ["#1565c0", "#ef6c00", "#2e7d32", "#8e24aa", "#c62828"]
    for room in manifest.get("rooms", []):
        x0, y0, x1, y1 = room["bounds_m"]
        patch = Rectangle(
            (x0, y0), x1 - x0, y1 - y0,
            facecolor="#f5f7fa", edgecolor="#455a64", linewidth=1.0,
        )
        ax.add_patch(patch)
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, room["id"].replace("_", "\n"),
                ha="center", va="center", fontsize=7, color="#263238")

    corridor = manifest.get("main_corridor", {}).get("bounds_m")
    if corridor:
        x0, y0, x1, y1 = corridor
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               facecolor="#e0f2f1", edgecolor="#00796b",
                               linewidth=1.2, alpha=0.8))
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, "main corridor",
                ha="center", va="center", fontsize=8, color="#00695c")

    for index, log in enumerate(logs):
        points = read_samples(log)
        if not points:
            continue
        xs, ys = zip(*points)
        label = log.parent.name
        ax.plot(xs, ys, color=colors[index % len(colors)], linewidth=1.4,
                alpha=0.82, label=label)
        ax.scatter(xs[0], ys[0], color=colors[index % len(colors)], marker="o",
                   s=22, edgecolor="white", linewidth=0.5, zorder=4)
        ax.scatter(xs[-1], ys[-1], color=colors[index % len(colors)], marker="x",
                   s=34, linewidth=1.2, zorder=4)

    primary = profiles.get("primary", {}).get("pose", [])
    if len(primary) >= 2:
        ax.scatter(primary[0], primary[1], marker="*", s=120, color="#212121",
                   edgecolor="white", linewidth=0.7, zorder=5, label="primary start")
    ax.scatter(target[0], target[1], marker="o", s=110, color="#2e7d32",
               edgecolor="white", linewidth=0.8, zorder=5, label="yellow cup")

    ax.set_xlim(bounds[0] - 0.5, bounds[2] + 0.5)
    ax.set_ylim(bounds[1] - 0.5, bounds[3] + 0.5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Office building benchmark: floor plan and robot trajectories")
    ax.grid(True, linewidth=0.35, alpha=0.45)
    ax.legend(loc="upper left", fontsize=7, framealpha=0.9)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path,
                        help="navigation metrics logs")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    logs = [path if path.is_absolute() else ROOT / path for path in args.logs]
    manifest = args.manifest if args.manifest.is_absolute() else ROOT / args.manifest
    output = args.output if args.output.is_absolute() else ROOT / args.output
    missing = [str(path) for path in logs + [manifest] if not path.is_file()]
    if missing:
        raise SystemExit("missing input: %s" % ", ".join(missing))
    render(manifest, logs, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
