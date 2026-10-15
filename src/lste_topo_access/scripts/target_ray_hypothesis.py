"""Geometry-only target hypothesis from source-stamped camera rays.

An image detector gives a bearing, not a metric range.  Repeated bearings
from different odometry poses are nevertheless enough to estimate a static
2-D target point.  This module intentionally has no ROS, detector score, map,
or controller dependency, so the estimate can be replayed from a log.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RayObservation:
    origin_xy: tuple
    direction_xy: tuple


@dataclass(frozen=True)
class TargetRayEstimate:
    point_xy: tuple
    residual: float
    ray_count: int
    forward_depths: tuple


def _xy(value):
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def _unit(value):
    point = _xy(value)
    if point is None:
        return None
    length = math.hypot(point[0], point[1])
    if not math.isfinite(length) or length <= 1e-9:
        return None
    return point[0] / length, point[1] / length


def _normal_equation(rays):
    """Build sum(I-dd^T) and sum((I-dd^T)o) for line least squares."""
    a00 = a01 = a11 = b0 = b1 = 0.0
    for origin, direction in rays:
        dx, dy = direction
        # Perpendicular projector for the infinite line through origin.
        p00 = 1.0 - dx * dx
        p01 = -dx * dy
        p11 = 1.0 - dy * dy
        a00 += p00
        a01 += p01
        a11 += p11
        b0 += p00 * origin[0] + p01 * origin[1]
        b1 += p01 * origin[0] + p11 * origin[1]
    return a00, a01, a11, b0, b1


def estimate_target_point(observations, *, min_baseline=0.05):
    """Estimate one static point from at least two non-parallel rays.

    ``min_baseline`` is a numerical observability guard, not a navigation
    parameter.  The returned point is rejected when the rays do not provide
    a well-conditioned forward intersection or when their line residual is
    inconsistent with a small image/odometry error.
    """
    rays = []
    for observation in observations or ():
        if isinstance(observation, RayObservation):
            origin = _xy(observation.origin_xy)
            direction = _unit(observation.direction_xy)
        else:
            try:
                origin = _xy(observation[0])
                direction = _unit(observation[1])
            except (IndexError, TypeError):
                origin = direction = None
        if origin is not None and direction is not None:
            rays.append((origin, direction))
    if len(rays) < 2:
        return None

    # A nearly parallel set has no useful depth observability.  Require one
    # pair with a physical baseline and a non-zero cross product.
    observable = False
    for index, (origin, direction) in enumerate(rays):
        for other_origin, other_direction in rays[index + 1:]:
            baseline = math.hypot(
                origin[0] - other_origin[0], origin[1] - other_origin[1],
            )
            cross = abs(
                direction[0] * other_direction[1]
                - direction[1] * other_direction[0]
            )
            if baseline >= float(min_baseline) and cross > 1e-3:
                observable = True
                break
        if observable:
            break
    if not observable:
        return None

    a00, a01, a11, b0, b1 = _normal_equation(rays)
    determinant = a00 * a11 - a01 * a01
    trace = a00 + a11
    if not math.isfinite(determinant) or determinant <= 1e-6:
        return None
    # A low line residual is not enough to establish depth.  A set of nearly
    # parallel forward rays can intersect at a numerically precise point that
    # moves with the robot (the classic forward-motion triangulation
    # degeneracy).  Normalize the determinant by the squared trace so the
    # observability test is independent of the number of rays.  For two rays,
    # 0.001 corresponds to roughly a 3.6 degree separation; below that angle
    # the range estimate is too ill-conditioned to become navigation state.
    normalized_observability = determinant / max(trace * trace, 1e-12)
    if (
        not math.isfinite(trace)
        or trace <= 1e-9
        or not math.isfinite(normalized_observability)
        or normalized_observability < 0.001
    ):
        return None
    point = (
        (b0 * a11 - a01 * b1) / determinant,
        (a00 * b1 - a01 * b0) / determinant,
    )
    if not all(math.isfinite(value) for value in point):
        return None

    depths = []
    squared_residual = 0.0
    for origin, direction in rays:
        dx, dy = point[0] - origin[0], point[1] - origin[1]
        depth = dx * direction[0] + dy * direction[1]
        perpendicular = dx * direction[1] - dy * direction[0]
        depths.append(depth)
        squared_residual += perpendicular * perpendicular
    if not depths or min(depths) <= 0.0:
        return None
    residual = math.sqrt(squared_residual / float(len(rays)))
    # The bound is deliberately fixed at the geometry layer. A larger error
    # means the detections are not one static target track; keep bearing-only
    # navigation rather than inventing a point between incompatible rays.
    if not math.isfinite(residual) or residual > 0.75:
        return None
    return TargetRayEstimate(
        point_xy=point,
        residual=residual,
        ray_count=len(rays),
        forward_depths=tuple(depths),
    )


__all__ = ["RayObservation", "TargetRayEstimate", "estimate_target_point"]
