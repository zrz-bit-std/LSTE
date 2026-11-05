#!/usr/bin/env python3
"""Visited-path memory for durable Place identity and return deduplication."""

import math


def _point(value):
    try:
        x = float(value[0])
        y = float(value[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


class FrontierRegionVisitMixin:
    """Record where the robot actually travelled inside one physical Place.

    The live Place mask is a known-free connected component: an open doorway
    connects two rooms, so a destination room is still inside that mask.  This
    history is the complementary fact the mask cannot provide: *has the robot
    actually been there before this doorway observation?*
    """

    def record_visited_pose(
        self, region_id, map_xy=None, odom_xy=None, now=0.0,
        min_spacing=0.12, limit=6000,
    ):
        """Append one measured pose to a Place's visited path."""
        region = self.by_id(region_id)
        if region is None:
            return None
        try:
            now = float(now)
        except (TypeError, ValueError):
            now = 0.0

        def append(key, value):
            point = _point(value)
            if point is None:
                return
            path = region.setdefault(key, [])
            if path:
                last = path[-1]
                try:
                    distance = math.hypot(
                        point[0] - float(last["x"]),
                        point[1] - float(last["y"]),
                    )
                except (KeyError, TypeError, ValueError):
                    distance = float("inf")
                if distance < float(min_spacing):
                    return
            path.append({"x": point[0], "y": point[1], "t": now})
            if len(path) > int(limit):
                del path[: len(path) - int(limit)]

        append("visited_path_map", map_xy)
        append("visited_path_odom", odom_xy)
        region["visited_path_updated_at"] = now
        return region

    def visited_path(self, region_id, frame="odom"):
        """Return the immutable visited points for one Place and frame."""
        region = self.by_id(region_id)
        if region is None:
            return ()
        key = (
            "visited_path_map"
            if str(frame or "odom").strip().lower() == "map"
            else "visited_path_odom"
        )
        return tuple(region.get(key) or ())

    def has_visited_path(self, region_id, frame="odom"):
        return bool(self.visited_path(region_id, frame=frame))

    def destination_was_visited(
        self, region_id, point, before_time=None, radius=0.9, frame="odom",
    ):
        """Return whether ``point`` lies on a path the robot already travelled.

        Points recorded at or after ``before_time`` are excluded.  This is the
        branch-start snapshot: the doorway observation currently in progress
        must not count its own entry path as prior exploration.
        """
        target = _point(point)
        if target is None:
            return False
        try:
            radius = max(0.0, float(radius))
        except (TypeError, ValueError):
            radius = 0.9
        cutoff = None
        if before_time is not None:
            try:
                cutoff = float(before_time)
            except (TypeError, ValueError):
                cutoff = None
        for entry in self.visited_path(region_id, frame=frame):
            try:
                if cutoff is not None and float(entry.get("t", 0.0)) >= cutoff:
                    continue
                if math.hypot(
                    target[0] - float(entry["x"]),
                    target[1] - float(entry["y"]),
                ) <= radius:
                    return True
            except (KeyError, TypeError, ValueError):
                continue
        return False
