"""Geometry-only evidence for promoting a visual target track.

The detector produces bearings, not world points.  This module keeps the
promotion contract in one place so callers cannot accidentally accept a
precise-looking intersection that has no physical observability.
"""

import math

from target_ray_hypothesis import estimate_target_point


# These are observation-contract constants, not controller tuning knobs.  A
# weak visual track must contain enough independent evidence to distinguish a
# static object from a repeated false box while the robot is moving.
MIN_STABLE_RAY_COUNT = 3
MIN_TRANSLATION_BASELINE_M = 0.20


def _ray(value):
    try:
        origin = float(value[0][0]), float(value[0][1])
        direction = float(value[1][0]), float(value[1][1])
    except (IndexError, TypeError, ValueError):
        return None
    length = math.hypot(direction[0], direction[1])
    if not all(math.isfinite(item) for item in origin + direction):
        return None
    if not math.isfinite(length) or length <= 1e-9:
        return None
    return origin, (direction[0] / length, direction[1] / length)


def forward_ray_intersection(first, second):
    """Return the intersection of two non-parallel forward half-rays."""
    first, second = _ray(first), _ray(second)
    if first is None or second is None:
        return None
    (x1, y1), (dx1, dy1) = first
    (x2, y2), (dx2, dy2) = second
    denominator = dx1 * dy2 - dy1 * dx2
    if abs(denominator) <= 1e-6:
        return None
    delta_x, delta_y = x2 - x1, y2 - y1
    along_first = (delta_x * dy2 - delta_y * dx2) / denominator
    along_second = (delta_x * dy1 - delta_y * dx1) / denominator
    if along_first <= 0.0 or along_second <= 0.0:
        return None
    return (
        x1 + along_first * dx1,
        y1 + along_first * dy1,
    )


def supports_static_target(rays):
    """Return whether the rays satisfy the shared estimator contract.

    Keeping this predicate backed by :func:`estimate_target_point` is
    important: a caller must not use the weaker ``pairwise intersection``
    shortcut while the estimator rejects a near-parallel or behind-camera
    configuration. Two rays remain valid diagnostic evidence for backwards
    compatibility; weak-track navigation uses
    :func:`supports_stable_static_target`.
    """
    valid = [ray for ray in (rays or ()) if _ray(ray) is not None]
    return estimate_target_point(valid) is not None


def supports_stable_static_target(rays):
    """Require redundant, translationally observable target geometry.

    A two-ray intersection can be numerically exact while still being wholly
    determined by detector noise. Weak detections therefore need at least
    three source-stamped rays, one physical translation baseline, and two
    independent forward pair intersections. The estimator supplies the
    remaining conditioning, forward-depth, and residual checks.
    """
    valid = [ray for ray in (rays or ()) if _ray(ray) is not None]
    if len(valid) < MIN_STABLE_RAY_COUNT:
        return False
    if estimate_target_point(valid) is None:
        return False
    pair_intersections = 0
    translated_pair = False
    for index, first in enumerate(valid):
        first_ray = _ray(first)
        if first_ray is None:
            continue
        first_origin, first_direction = first_ray
        for second in valid[index + 1:]:
            second_ray = _ray(second)
            if second_ray is None:
                continue
            second_origin, second_direction = second_ray
            baseline = math.hypot(
                first_origin[0] - second_origin[0],
                first_origin[1] - second_origin[1],
            )
            if baseline >= MIN_TRANSLATION_BASELINE_M:
                translated_pair = True
            cross = abs(
                first_direction[0] * second_direction[1]
                - first_direction[1] * second_direction[0]
            )
            if cross <= 1e-3:
                continue
            if forward_ray_intersection(first, second) is not None:
                pair_intersections += 1
    return translated_pair and pair_intersections >= 2
