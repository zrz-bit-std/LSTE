"""Physical progress contract for an active exploration route.

The SLAM map is a short-lived estimate. Its frontier topology and Navfn
distance can change while the base is stationary, especially after a scan
matching correction. Those values are useful for selection and telemetry,
but they are not evidence that an active controller lease made progress.

This module keeps that distinction explicit and ROS-free so it can be tested
without starting the navigation stack.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PhysicalProgressDecision:
    """One route-progress decision and the evidence that justified it."""

    renew_watchdog: bool
    signal: str


def physical_progress_decision(
    *,
    route_progress=False,
    goal_progress=False,
    odom_detour_progress=False,
    odom_novel_coverage=False,
):
    """Return whether a map snapshot may renew an active route lease.

    ``route_progress`` and ``goal_progress`` are intentionally accepted for
    diagnostics only. They are derived from ``/map`` and may improve solely
    because SLAM moved the coordinate frame or relabelled free space. A
    watchdog is renewed only by physical odometry evidence: a new bounded
    odom excursion or a new odom coverage cell.
    """
    if odom_detour_progress and odom_novel_coverage:
        return PhysicalProgressDecision(True, "odom_detour_and_novel_coverage")
    if odom_detour_progress:
        return PhysicalProgressDecision(True, "odom_detour")
    if odom_novel_coverage:
        return PhysicalProgressDecision(True, "odom_novel_coverage")
    if route_progress or goal_progress:
        return PhysicalProgressDecision(False, "map_progress_without_physical_motion")
    return PhysicalProgressDecision(False, "none")
