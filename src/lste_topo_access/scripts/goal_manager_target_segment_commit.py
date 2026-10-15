"""Atomic visual-target segment commitment and its audit events."""

import copy
import math
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped

from goal_manager_viewpoint import select_target_viewpoint


class GoalManagerTargetSegmentCommitMixin:
    """Turn a Navfn-validated visual viewpoint into one stable TEB intent."""

    def _hold_target_for_route_validation(self, now: float, selection) -> None:
        """Retain a healthy action while the current visual ray lacks a route."""
        self.goal_source = (
            "global_slam_frontier"
            if self.last_goal is not None
            and self.last_goal_source == "global_slam_frontier"
            else "target_waiting_navfn_route"
        )
        if now - self.target_route_hold_last_emit < self.target_route_validation_period:
            return
        self.target_route_hold_last_emit = now
        last_requested_goal = selection.last_requested_goal
        self.publish_goal_arbitration(
            "target_route_held",
            reason=(
                "navfn_empty_plan"
                if selection.route_status is False
                else "route_validation_unavailable"
            ),
            goal=(
                None
                if last_requested_goal is None
                else [
                    round(float(last_requested_goal.pose.position.x), 3),
                    round(float(last_requested_goal.pose.position.y), 3),
                ]
            ),
            goal_frame=(
                "map"
                if last_requested_goal is None
                else (last_requested_goal.header.frame_id or "map")
            ),
            fallback_goal=(
                None
                if self.last_goal is None
                else [
                    round(float(self.last_goal.pose.position.x), 3),
                    round(float(self.last_goal.pose.position.y), 3),
                ]
            ),
        )

    def _defer_sharp_target_takeover(self, candidate_goal, entry_heading_error: float) -> None:
        """Save a visual target until the current frontier action terminates."""
        self.target_route_continuity_deferred = True
        self.target_route_continuity_deferred_goal = copy.deepcopy(candidate_goal)
        self.target_execution_state = "TARGET_ROUTE_DEFERRED_FOR_CONTINUITY"
        self.goal_source = "global_slam_frontier"
        self.publish_goal_arbitration(
            "target_takeover_deferred_for_continuity",
            target_track_id=self.target_track_id,
            active_frontier_goal=[
                round(float(self.last_goal.pose.position.x), 3),
                round(float(self.last_goal.pose.position.y), 3),
            ],
            active_frontier_distance=round(
                float(self.goal_robot_distance(self.last_goal) or 0.0), 3
            ),
            deferred_target_goal=[
                round(float(candidate_goal.pose.position.x), 3),
                round(float(candidate_goal.pose.position.y), 3),
            ],
            navfn_entry_heading_error=round(float(entry_heading_error), 4),
            takeover_heading_limit=round(
                float(self.target_route_continuity_takeover_heading), 4
            ),
        )
        rospy.loginfo(
            "GoalManager: defer sharp visual takeover entry_error=%.1fdeg "
            "until frontier terminal",
            math.degrees(entry_heading_error),
        )

    def _record_target_segment_commit(self, now: float, source: str, advance: bool, selection) -> None:
        """Update the segment lifecycle state after a successful selection."""
        self.target_last_goal = selection.goal
        self.target_viewpoint_candidate_id = str(
            getattr(selection, "candidate_id", "") or ""
        )
        self.target_viewpoint_attempt_id = str(
            getattr(selection, "attempt_id", "") or ""
        )
        self.target_last_navigation_heading = selection.navigation_heading
        self.target_execution_state = "TARGET_ROUTE_VALIDATED"
        if advance:
            self.target_cache_advances += 1
        else:
            self.target_cache_advances = 0
            self.target_terminal_blind_advances = 0
        if source == "target_terminal_advance":
            self.target_terminal_blind_advances += 1
        self.target_last_update = now
        self.target_goal_detection_stamp = self.target_last_detection_stamp
        self.target_segment_terminal_ready = False
        self.goal_source = source
        self.target_segment_commit_epoch = int(self.target_observation_epoch)
        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        if approach_transaction is not None:
            approach_transaction.segment_committed(
                self.target_track_id,
                now,
                candidate_id=self.target_viewpoint_candidate_id,
                attempt_id=self.target_viewpoint_attempt_id,
            )

    def _publish_target_segment_commit(self, source: str, advance: bool, selection) -> None:
        """Publish complete decision evidence for a newly committed segment."""
        candidate_goal = selection.goal
        requested_goal = selection.requested_goal or candidate_goal
        endpoint_adjustment = self.pose_distance(requested_goal, candidate_goal)
        self.publish_goal_arbitration(
            "target_segment_committed",
            target_track_id=self.target_track_id,
            target_viewpoint_candidate_id=self.target_viewpoint_candidate_id,
            target_viewpoint_attempt_id=self.target_viewpoint_attempt_id,
            target_viewpoint_map_epoch=getattr(
                self, "target_viewpoint_map_epoch", None
            ),
            advance=bool(advance),
            segment_index=int(self.target_cache_advances),
            approach_strategy=self.target_approach_strategy,
            viewpoint_candidate_index=int(selection.index),
            viewpoint_rejected_distances=list(selection.rejected_distances),
            segment_distance=round(float(selection.distance), 3),
            heading=round(float(self.target_last_heading), 4),
            navigation_heading=(
                None
                if selection.navigation_heading is None
                else round(float(selection.navigation_heading), 4)
            ),
            target_hypothesis_odom=(
                None
                if getattr(self, "target_hypothesis_xy", None) is None
                else [
                    round(float(self.target_hypothesis_xy[0]), 4),
                    round(float(self.target_hypothesis_xy[1]), 4),
                ]
            ),
            target_bearing_odom=[
                round(math.cos(float(self.target_last_heading)), 4),
                round(math.sin(float(self.target_last_heading)), 4),
            ],
            target_observation_origin_odom=(
                None
                if self.target_last_observation_odom is None
                else [
                    round(float(self.target_last_observation_odom[0]), 4),
                    round(float(self.target_last_observation_odom[1]), 4),
                ]
            ),
            navfn_entry_heading=(
                None
                if selection.entry_heading is None
                else round(float(selection.entry_heading), 4)
            ),
            navfn_entry_heading_error=(
                None
                if selection.entry_heading_error is None
                else round(float(selection.entry_heading_error), 4)
            ),
            continuity_distance_penalty=round(
                float(selection.distance_penalty or 0.0), 4
            ),
            continuity_score=round(float(selection.continuity_score or 0.0), 4),
            continuity_tier=selection.continuity_tier,
            # ``goal`` is the actual mission endpoint. The original visual
            # ray remains explicit evidence for diagnosing a tolerance-based
            # Navfn adjustment without misleading downstream log consumers.
            goal=[
                round(float(candidate_goal.pose.position.x), 3),
                round(float(candidate_goal.pose.position.y), 3),
            ],
            goal_frame=(candidate_goal.header.frame_id or "map"),
            requested_visual_goal=[
                round(float(requested_goal.pose.position.x), 3),
                round(float(requested_goal.pose.position.y), 3),
            ],
            navfn_returned_goal=[
                round(float(candidate_goal.pose.position.x), 3),
                round(float(candidate_goal.pose.position.y), 3),
            ],
            navfn_endpoint_adjustment=round(float(endpoint_adjustment or 0.0), 4),
            navfn_endpoint_contract=True,
            viewpoint_geometry_mode=selection.geometry_mode,
            target_range=(
                None
                if selection.target_range is None
                else round(float(selection.target_range), 4)
            ),
            target_standoff=(
                None
                if selection.target_standoff is None
                else round(float(selection.target_standoff), 4)
            ),
            remaining_target_range=(
                None
                if selection.remaining_target_range is None
                else round(float(selection.remaining_target_range), 4)
            ),
            source_image_stamp=(
                None
                if self.target_last_heading_source_stamp is None
                else round(float(self.target_last_heading_source_stamp), 6)
            ),
            projection_tf_mode=(
                "source_stamp"
                if self.target_last_heading_source_stamp
                else "latest_for_unstamped_message"
            ),
            target_approach_transaction=(
                None
                if getattr(self, "target_approach_transaction", None) is None
                else self.target_approach_transaction.snapshot().__dict__
            ),
        )
        if source == "target_continuous_handoff":
            self.publish_goal_arbitration(
                "target_continuous_handoff_prepared",
                target_track_id=self.target_track_id,
                current_segment_epoch=int(self.target_segment_commit_epoch),
                goal=[
                    round(float(candidate_goal.pose.position.x), 3),
                    round(float(candidate_goal.pose.position.y), 3),
                ],
            )

    def commit_target_segment(
        self, now: float, source: str, advance: bool = False
    ) -> Optional[PoseStamped]:
        """Commit one visual-servo horizon as an atomic navigation intent.

        The segment is deliberately created from the latest robot pose only at
        a lifecycle boundary (initial target lock or a terminal action result),
        never from an arbitrary detector callback.  This keeps the world-frame
        goal stable while TEB optimizes and executes the current route.
        """
        if self.latest_pose is None or self.target_last_heading is None:
            return None
        selection = select_target_viewpoint(self, now)
        if selection is None:
            return None
        if selection.goal is None:
            if selection.route_status == "viewpoint_portfolio_exhausted":
                # The target work item, rather than Navfn, owns this terminal
                # boundary.  Completion evidence is evaluated by
                # ``maybe_publish_task_done`` on the same timer tick; do not
                # reinterpret an exhausted ledger as a planner outage.
                self.target_execution_state = "TARGET_VIEWPOINT_PORTFOLIO_EXHAUSTED"
                self.publish_goal_arbitration(
                    "target_viewpoint_portfolio_exhausted",
                    target_track_id=self.target_track_id,
                    completed_segments=int(self.target_completed_segments),
                )
                return None
            if selection.geometry_mode == "safe_standoff":
                # The metric target estimate has reached its observation
                # circle.  A zero-length Navfn action would be interpreted as
                # a controller failure, so keep the camera viewpoint as the
                # active semantic transaction and let fresh detector frames
                # settle completion or explicit target loss.
                self.target_execution_state = "TARGET_SAFE_STANDOFF"
                self.target_observation_hold_until = max(
                    float(getattr(self, "target_observation_hold_until", 0.0)),
                    now + float(getattr(self, "target_observation_hold", 0.0)),
                )
                self.set_navigation_hold(True, "target_safe_standoff")
                self.publish_goal_arbitration(
                    "target_safe_standoff_reached",
                    target_track_id=self.target_track_id,
                    target_range=(
                        None
                        if selection.target_range is None
                        else round(float(selection.target_range), 3)
                    ),
                    target_standoff=(
                        None
                        if selection.target_standoff is None
                        else round(float(selection.target_standoff), 3)
                    ),
                    geometry_mode=selection.geometry_mode,
                )
                rospy.loginfo(
                    "GoalManager: target already at safe observation standoff "
                    "range=%.2fm standoff=%.2fm; holding for evidence",
                    float(selection.target_range or 0.0),
                    float(selection.target_standoff or 0.0),
                )
                return None
            self._hold_target_for_route_validation(now, selection)
            return None
        if self.should_defer_sharp_target_takeover(
            source, selection.entry_heading_error
        ):
            self._defer_sharp_target_takeover(
                selection.goal, selection.entry_heading_error
            )
            return None
        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        ledger = (
            None
            if approach_transaction is None
            else getattr(approach_transaction, "viewpoint_ledger", None)
        )
        if ledger is not None and getattr(selection, "candidate_id", ""):
            candidate = ledger.begin_attempt(selection.candidate_id, now)
            if candidate is None:
                self.publish_goal_arbitration(
                    "target_viewpoint_dispatch_deferred",
                    target_track_id=self.target_track_id,
                    candidate_id=selection.candidate_id,
                    reason="candidate_not_dispatchable",
                )
                return None
            selection.attempt_id = candidate.attempt_id
        self._record_target_segment_commit(now, source, advance, selection)
        self._publish_target_segment_commit(source, advance, selection)
        rospy.loginfo(
            "GoalManager: committed target segment source=%s strategy=%s "
            "candidate=%d advance=%s segment_count=%d distance=%.2f "
            "heading=%.3f goal=(%.2f,%.2f)",
            source,
            self.target_approach_strategy,
            selection.index,
            bool(advance),
            int(
                getattr(
                    getattr(self, "target_approach_transaction", None),
                    "segment_count",
                    self.target_cache_advances,
                )
            ),
            selection.distance,
            self.target_last_heading,
            selection.goal.pose.position.x,
            selection.goal.pose.position.y,
        )
        return self.target_last_goal
