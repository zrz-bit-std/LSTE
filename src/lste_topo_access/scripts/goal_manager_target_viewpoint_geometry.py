"""Pure geometry for converting a visual target into a safe view point.

An image ray is not a drivable destination.  When the target ray ledger has a
well-conditioned static-point estimate, this module turns the estimate into a
short approach along the robot-to-target direction and stops on a standoff
circle.  The target itself is therefore used as observation evidence only.

The fallback remains the historical bearing horizon for one-ray or otherwise
unobservable tracks.  That keeps this module an architectural refinement, not
an implicit dependency on simulator truth or a new controller parameter.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class TargetViewpointGeometry:
    """One odometry-frame request before Navfn validates the route."""

    x: float
    y: float
    yaw: float
    distance_from_robot: float
    target_range: Optional[float]
    target_standoff: Optional[float]
    remaining_target_range: Optional[float]
    mode: str


def _finite_xy(value):
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def build_target_viewpoint(manager, navigation_heading: float, requested_distance: float):
    """Build one target observation request in the robot's odometry frame.

    ``target_hypothesis_xy`` is accepted only when the estimator has declared
    at least two rays.  The estimator itself enforces angular observability;
    this second guard keeps old/replayed manager state from becoming a route.
    """
    pose = getattr(manager, "latest_pose", None)
    if pose is None:
        return None
    try:
        robot_x, robot_y = float(pose.x), float(pose.y)
    except (AttributeError, TypeError, ValueError):
        return None
    if not (math.isfinite(robot_x) and math.isfinite(robot_y)):
        return None
    try:
        requested_distance = max(0.0, float(requested_distance))
    except (TypeError, ValueError):
        requested_distance = 0.0
    try:
        fallback_yaw = float(navigation_heading)
    except (TypeError, ValueError):
        fallback_yaw = 0.0
    if not math.isfinite(fallback_yaw):
        fallback_yaw = 0.0

    # A low residual alone does not certify camera/world calibration.  Keep
    # static-point routing opt-in for a separately calibrated experiment; the
    # production path uses the active-parallax + bearing policy below.
    if not getattr(manager, "target_hypothesis_navigation_enabled", False):
        hypothesis = None
    else:
        hypothesis = _finite_xy(getattr(manager, "target_hypothesis_xy", None))
    try:
        ray_count = int(getattr(manager, "target_hypothesis_ray_count", 0) or 0)
    except (TypeError, ValueError):
        ray_count = 0
    if hypothesis is None or ray_count < 2:
        return TargetViewpointGeometry(
            x=robot_x + requested_distance * math.cos(fallback_yaw),
            y=robot_y + requested_distance * math.sin(fallback_yaw),
            yaw=fallback_yaw,
            distance_from_robot=requested_distance,
            target_range=None,
            target_standoff=None,
            remaining_target_range=None,
            mode="bearing_horizon",
        )

    target_x, target_y = hypothesis
    delta_x, delta_y = target_x - robot_x, target_y - robot_y
    target_range = math.hypot(delta_x, delta_y)
    if not math.isfinite(target_range) or target_range <= 1e-6:
        return TargetViewpointGeometry(
            x=robot_x,
            y=robot_y,
            yaw=fallback_yaw,
            distance_from_robot=0.0,
            target_range=0.0,
            target_standoff=None,
            remaining_target_range=0.0,
            mode="safe_standoff",
        )

    # The existing minimum viewpoint distance is derived from the robot's
    # arrival tolerance and footprint.  Reuse it as the observation radius so
    # the target policy adds no independent distance knob.
    try:
        standoff = max(
            float(manager.target_minimum_viewpoint_distance()),
            float(getattr(manager, "target_goal_reached_radius", 0.0)) + 0.05,
        )
    except (AttributeError, TypeError, ValueError):
        standoff = 0.90
    if not math.isfinite(standoff) or standoff <= 0.0:
        standoff = 0.90
    # Navfn may legally return a nearby grid endpoint under its validation
    # tolerance.  Keep that tolerance outside the object-facing side of the
    # observation ring so the global endpoint contract cannot undo the local
    # standoff decision.
    try:
        route_tolerance = max(
            0.0, float(getattr(manager, "target_route_validation_tolerance", 0.0))
        )
    except (TypeError, ValueError):
        route_tolerance = 0.0
    standoff += route_tolerance

    target_direction_x = delta_x / target_range
    target_direction_y = delta_y / target_range
    remaining_range = max(0.0, target_range - standoff)
    step = min(requested_distance, remaining_range)
    if step <= 1e-6:
        # The robot is already on the safe observation circle.  Holding this
        # view is a semantic observation action, not a zero-length Navfn goal.
        return TargetViewpointGeometry(
            x=robot_x,
            y=robot_y,
            yaw=math.atan2(delta_y, delta_x),
            distance_from_robot=0.0,
            target_range=target_range,
            target_standoff=standoff,
            remaining_target_range=target_range,
            mode="safe_standoff",
        )

    goal_x = robot_x + step * target_direction_x
    goal_y = robot_y + step * target_direction_y
    return TargetViewpointGeometry(
        x=goal_x,
        y=goal_y,
        # Face the estimated object at the observation point.  TEB may ignore
        # yaw for a circular base, but the camera-facing intent remains useful
        # to the completion gate and to logs.
        yaw=math.atan2(target_y - goal_y, target_x - goal_x),
        distance_from_robot=step,
        target_range=target_range,
        target_standoff=standoff,
        remaining_target_range=target_range - step,
        mode="static_target_standoff",
    )


__all__ = ["TargetViewpointGeometry", "build_target_viewpoint"]
