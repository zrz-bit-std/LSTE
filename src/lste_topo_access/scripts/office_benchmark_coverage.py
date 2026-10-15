"""Manifest-defined coverage truth for the deterministic office benchmark.

The online SLAM grid is intentionally mutable, whereas benchmark coverage
needs a fixed denominator.  This module constructs that denominator from the
committed office manifest: the robot-centre free floor of enabled rooms and
the main corridor.  It is deliberately independent of ROS so its geometry can
be regression-tested and its result can be reproduced offline.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml


@dataclass(frozen=True)
class TruthPoint:
    """One fixed robot-centre floor sample in physical odom/world metres."""

    x: float
    y: float
    region: str


def _bounds(value: Iterable[float]) -> tuple[float, float, float, float]:
    items = tuple(float(item) for item in value)
    if len(items) != 4 or items[0] >= items[2] or items[1] >= items[3]:
        raise ValueError("bounds must be [min_x, min_y, max_x, max_y]")
    return items


def _inset(bounds: tuple[float, float, float, float], clearance: float):
    x0, y0, x1, y1 = bounds
    inset = max(0.0, float(clearance))
    result = (x0 + inset, y0 + inset, x1 - inset, y1 - inset)
    if result[0] >= result[2] or result[1] >= result[3]:
        raise ValueError("clearance removes all free floor from %r" % (bounds,))
    return result


def _sample_bounds(bounds, resolution: float, region: str):
    """Yield centre samples with deterministic ordering and no edge ambiguity."""
    x0, y0, x1, y1 = bounds
    step = float(resolution)
    if step <= 0.0:
        raise ValueError("sample resolution must be positive")
    x = x0 + step / 2.0
    while x < x1:
        y = y0 + step / 2.0
        while y < y1:
            yield TruthPoint(round(x, 6), round(y, 6), region)
            y += step
        x += step


def load_manifest_truth(manifest_path: Path, level: str):
    """Return the immutable architectural free-floor sampling contract.

    The denominator is *not* inferred from a run's SLAM map.  Furniture is
    still part of the perception task and is counted as known when observed;
    room-boundary clearance removes walls and impossible robot-centre cells.
    This makes the result an architectural free-floor coverage metric, not a
    claim that every mesh triangle is traversable.
    """
    manifest = yaml.safe_load(Path(manifest_path).read_text(encoding="utf-8")) or {}
    level_spec = (manifest.get("levels") or {}).get(level)
    if not isinstance(level_spec, dict):
        raise ValueError("unknown benchmark level %r" % level)
    truth = manifest.get("coverage_truth") or {}
    resolution = float(truth.get("sample_resolution_m", 0.25))
    clearance = float(truth.get("boundary_clearance_m", 0.30))
    if resolution <= 0.0:
        raise ValueError("coverage_truth.sample_resolution_m must be positive")
    if clearance < 0.0:
        raise ValueError("coverage_truth.boundary_clearance_m must be non-negative")

    enabled = {str(item) for item in level_spec.get("enabled_rooms", ())}
    points = []
    for room in manifest.get("rooms") or ():
        room_id = str(room.get("id") or "")
        if room_id in enabled:
            points.extend(_sample_bounds(
                _inset(_bounds(room.get("bounds_m") or ()), clearance),
                resolution,
                room_id,
            ))
    corridor = (manifest.get("main_corridor") or {}).get("bounds_m")
    if corridor:
        points.extend(_sample_bounds(
            _inset(_bounds(corridor), clearance), resolution, "main_corridor"
        ))
    if not points:
        raise ValueError("benchmark truth contains no enabled floor samples")
    return {
        "schema_version": 1,
        "definition": "manifest_architectural_free_floor_map_known_fraction",
        "manifest_path": str(Path(manifest_path)),
        "level": str(level),
        "sample_resolution_m": resolution,
        "boundary_clearance_m": clearance,
        "points": tuple(points),
        "regions": tuple(sorted({point.region for point in points})),
    }


def _world_to_grid(x, y, map_origin, resolution):
    """Map a point in the map frame to an OccupancyGrid row/column."""
    origin_x, origin_y, origin_yaw = map_origin
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    dx, dy = float(x) - origin_x, float(y) - origin_y
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    return int(math.floor(local_x / resolution)), int(math.floor(local_y / resolution))


def evaluate_map_known_fraction(
    truth: dict,
    *,
    occupancy_data,
    width: int,
    height: int,
    resolution: float,
    map_origin: tuple[float, float, float],
    map_from_odom: tuple[float, float, float],
):
    """Project fixed odom/world truth samples through one current SLAM map.

    ``map_from_odom`` is the 2-D transform that maps an odom/world point into
    the map frame.  A cell with any non-negative occupancy value is known;
    unknown and out-of-grid points count against the fixed denominator.
    """
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("invalid OccupancyGrid metadata")
    if len(occupancy_data) < width * height:
        raise ValueError("OccupancyGrid data is shorter than width * height")
    tx, ty, yaw = (float(value) for value in map_from_odom)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    known = 0
    out_of_bounds = 0
    by_region = {region: [0, 0] for region in truth["regions"]}
    for point in truth["points"]:
        map_x = tx + cos_yaw * point.x - sin_yaw * point.y
        map_y = ty + sin_yaw * point.x + cos_yaw * point.y
        column, row = _world_to_grid(map_x, map_y, map_origin, resolution)
        values = by_region[point.region]
        values[1] += 1
        if column < 0 or row < 0 or column >= width or row >= height:
            out_of_bounds += 1
            continue
        if int(occupancy_data[row * width + column]) >= 0:
            known += 1
            values[0] += 1
    total = len(truth["points"])
    return {
        "definition": truth["definition"],
        "status": "measured",
        "level": truth["level"],
        "sample_resolution_m": truth["sample_resolution_m"],
        "boundary_clearance_m": truth["boundary_clearance_m"],
        "truth_sample_count": total,
        "known_sample_count": known,
        "unknown_or_out_of_bounds_sample_count": total - known,
        "out_of_bounds_sample_count": out_of_bounds,
        "building_truth_fraction": round(float(known) / float(total), 6),
        "region_known_fraction": {
            region: round(float(values[0]) / float(values[1]), 6)
            for region, values in sorted(by_region.items())
            if values[1]
        },
    }
