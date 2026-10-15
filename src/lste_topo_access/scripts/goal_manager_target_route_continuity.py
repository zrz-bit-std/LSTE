"""Route-shape policy for visual target segments.

This module decides whether a visual target can take over the current
frontier action without an unnecessarily sharp initial manoeuvre.  It does
not publish goals or mutate segment lifecycle state.
"""

import math
from typing import Optional

from goal_manager_target_utils import wrap_angle


class GoalManagerTargetRouteContinuityMixin:
    """Keep target-viewpoint route-shape policy separate from execution."""

    def target_route_entry_heading(self) -> Optional[float]:
        """Return the first meaningful Navfn tangent for the last validation.

        GoalManager never executes this plan.  It uses the first tangent only
        to decide whether two equally reachable visual viewpoints have a
        materially different handoff cost for the still-moving base.
        """
        if (
            not self.target_route_validation_last_plan
            or self.target_route_validation_last_start_heading is None
        ):
            return None
        first = self.target_route_validation_last_plan[0]
        start_x = float(first.pose.position.x)
        start_y = float(first.pose.position.y)
        for pose in self.target_route_validation_last_plan[1:]:
            dx = float(pose.pose.position.x) - start_x
            dy = float(pose.pose.position.y) - start_y
            if math.hypot(dx, dy) >= self.target_route_continuity_entry_lookahead:
                return math.atan2(dy, dx)
        return None

    def target_route_continuity(self, distance: float, desired_distance: float):
        """Score a validated route against the current map-frame yaw."""
        entry_heading = self.target_route_entry_heading()
        current_heading = self.target_route_validation_last_start_heading
        heading_error = None
        if entry_heading is not None and current_heading is not None:
            heading_error = abs(wrap_angle(entry_heading - current_heading))

        # Preserve the longer observation horizon unless the nearer endpoint
        # buys a significant reduction in immediate steering. This prevents a
        # nominally smooth policy from always picking the shortest rung and
        # multiplying terminal/re-observation transitions.
        distance_penalty = 0.0
        ladder_span = max(
            1e-3,
            desired_distance - self.target_minimum_viewpoint_distance(),
        )
        if desired_distance > distance:
            distance_penalty = self.target_route_continuity_distance_tradeoff * (
                (desired_distance - distance) / ladder_span
            )
        score = (heading_error if heading_error is not None else math.pi) + distance_penalty
        tier = "unknown"
        if heading_error is not None:
            tier = (
                "smooth"
                if heading_error <= self.target_route_continuity_smooth_heading
                else "sharp"
            )
        return entry_heading, heading_error, distance_penalty, score, tier

    def should_defer_sharp_target_takeover(
        self,
        source: str,
        entry_heading_error: Optional[float],
    ) -> bool:
        """Keep a healthy frontier action when target lock would force a U-turn.

        This applies only to the first target takeover. Subsequent visual
        segments are terminal-driven, so delaying them here would strand the
        robot at a target observation boundary instead of solving a genuine
        route bend.
        """
        if (
            not self.target_route_continuity_enabled
            or not self.target_route_continuity_defer_sharp_takeover
            or source != "target_follow"
            or entry_heading_error is None
            or entry_heading_error < self.target_route_continuity_takeover_heading
            or self.last_goal is None
            or self.last_goal_source != "global_slam_frontier"
            or self.teb_terminal_goal is not None
        ):
            return False
        distance_to_frontier = self.goal_robot_distance(self.last_goal)
        if distance_to_frontier is None:
            return False
        return distance_to_frontier > self.target_continuous_handoff_distance

    def can_prepare_target_continuous_handoff(self, current_distance: float) -> bool:
        """Return whether fresh evidence can safely prefetch one successor."""
        if not self.target_continuous_handoff_enabled:
            return False
        if self.controller_mode != "teb" or self.target_segment_terminal_ready:
            return False
        if self.target_terminal_reobserve_pending:
            return False
        if self.target_last_heading is None:
            return False
        if self.target_cache_advances >= self.target_cache_max_advances:
            return False
        if not (
            self.target_goal_reached_radius < current_distance
            <= self.target_continuous_handoff_distance
        ):
            return False
        if (
            self.target_observation_epoch
            < self.target_segment_commit_epoch
            + self.target_continuous_handoff_min_new_frames
        ):
            return False
        # A close visual target should terminate and use the explicit
        # close-confirmation contract, not receive another forward segment.
        det = self.target_detection_for_track(self.latest_dets)
        if det is not None and float(det.score) >= self.target_done_min_score:
            if (
                float(det.w) >= self.target_done_min_box_width
                or float(det.h) >= self.target_done_min_box_height
            ):
                return False
        return True
