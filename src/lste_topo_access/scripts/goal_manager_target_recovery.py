"""Visual target recovery hints and handoff back to frontier exploration."""

import math
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped


class GoalManagerTargetRecoveryMixin:
    """Keep non-executable visual recovery reasoning outside segment commit."""

    def target_semantic_hint_map(
        self,
        distance: Optional[float] = None,
        use_observation_origin: bool = False,
    ) -> Optional[Tuple[float, float]]:
        """Project a confirmed visual bearing into the stable map frame.

        The result is deliberately *not* a navigation goal. It is supplied to
        the frontier selector only after Navfn rejects the same visual ray, so
        it can prefer a reachable unknown-space boundary on the target side of
        the known obstruction.
        """
        heading = self.target_navigation_heading()
        if heading is None:
            return None
        hypothesis = getattr(self, "target_hypothesis_xy", None)
        if hypothesis is not None and not use_observation_origin:
            target = self.make_goal_pose(
                (float(hypothesis[0]), float(hypothesis[1]), 0.0), heading,
            )
            target_map = self._pose_in_frame(target, "map")
            if target_map is not None:
                return (
                    float(target_map.pose.position.x),
                    float(target_map.pose.position.y),
                )
        origin = self.target_last_observation_odom if use_observation_origin else None
        if origin is None:
            if self.latest_pose is None:
                return None
            origin = (float(self.latest_pose.x), float(self.latest_pose.y))
        if distance is None:
            distance = self.target_segment_distance()
        distance = max(0.2, float(distance))
        candidate = self.make_goal_pose(
            (
                origin[0] + distance * math.cos(heading),
                origin[1] + distance * math.sin(heading),
                0.0,
            ),
            heading,
        )
        candidate_map = self._pose_in_frame(candidate, "map")
        if candidate_map is None:
            return None
        return (
            float(candidate_map.pose.position.x),
            float(candidate_map.pose.position.y),
        )

    def target_pursuit_hint_distance(self) -> float:
        """Return the bounded non-executable depth used for target recovery."""
        return max(
            self.follow_target_step_distance,
            self.follow_target_step_distance * (self.target_cache_max_advances + 1),
        )

    def retain_target_observation_room(
        self, reason="target_viewpoint_insufficient", target_track_id="",
    ):
        """Replan inside the target Place after an insufficient viewpoint.

        Detector absence at one view is not target absence in the room. The
        global explorer receives a durable target-room claim and can select
        only local WorkItems until another view restores the track or an
        explicit multi-view negative episode certifies loss.
        """
        self.target_reinspection_pending = True
        hint = self.target_semantic_hint_map()
        request = getattr(self, "request_global_frontier_replan", None)
        if request is None:
            return 0
        request_id = request(
            str(reason),
            target_track_id=str(target_track_id or self.target_track_id or ""),
            target_room_claim=True,
            target_reinspection=True,
            target_pursuit=bool(hint is not None),
            semantic_hint_map=(
                None
                if hint is None
                else [round(float(hint[0]), 3), round(float(hint[1]), 3)]
            ),
        )
        self.publish_goal_arbitration(
            "target_reinspection_requested",
            replan_request_id=int(request_id or 0),
            target_track_id=str(target_track_id or self.target_track_id or ""),
            reason=str(reason),
            semantic_hint_map=(
                None
                if hint is None
                else [round(float(hint[0]), 3), round(float(hint[1]), 3)]
            ),
        )
        return request_id

    def release_failed_target_route(
        self,
        reason,
        status,
        failure_goal=None,
        target_track_id="",
    ):
        """Release a failed target lease before frontier replanning.

        A controller failure is an execution fact, not a reason to keep the
        old target pose or its Place claim alive. Clear the target transaction
        atomically, then request a route with an explicit target-claim release
        so the next Global Frontier command receives a new route identity.
        """
        claimed_track_id = str(
            target_track_id or getattr(self, "target_track_id", "") or ""
        )
        failure_count = int(getattr(self, "target_failure_count", 0) or 0)
        had_room_claim = bool(
            getattr(self, "target_candidate_room_claim_requested", False)
        )
        clear_memory = getattr(self, "clear_target_memory", None)
        clear_called = callable(clear_memory)
        release_request_id = 0
        if clear_called:
            try:
                release_request_id = int(
                    clear_memory(
                        "target_route_failed",
                        preserve_obligation=False,
                    )
                    or 0
                )
            except TypeError:
                # Keep compatibility with narrow policy fixtures exposing the
                # legacy no-argument clear facade.
                release_request_id = int(clear_memory() or 0)

        # These assignments are intentionally idempotent with
        # clear_target_memory. They also make the ownership boundary explicit
        # for external/legacy GoalManager compositions without the full target
        # completion mixin.
        self.target_blocked = False
        self.target_blocked_goal = None
        self.target_blocked_since = None
        self.target_blocked_reason = ""
        self.target_last_goal = None
        self.target_segment_terminal_ready = False
        self.target_reinspection_pending = False
        self.target_terminal_reobserve_pending = False
        self.target_candidate_room_claim_requested = False
        self.target_viewpoint_candidate_id = ""
        self.target_viewpoint_attempt_id = ""
        self.target_execution_state = "TARGET_ROUTE_FAILED_RELEASED"
        if getattr(self, "last_goal_source", "").startswith("target_"):
            self.last_goal = None
            self.last_goal_source = "waiting_global_slam_frontier"
        if hasattr(self, "teb_terminal_goal"):
            self.teb_terminal_goal = None
        self.goal_source = "waiting_global_slam_frontier"
        self.next_update_time = 0.0

        # A real clear_target_memory() returns the request id from its room
        # claim release. If no claim existed, issue the same explicit release
        # request here so old/legacy target compositions cannot strand the
        # frontier planner behind a stale target transaction.
        if release_request_id <= 0 and (not had_room_claim or not clear_called):
            request = getattr(self, "request_global_frontier_replan", None)
            if callable(request):
                failed_goal = None
                if failure_goal is not None:
                    failed_goal = [
                        round(float(failure_goal.pose.position.x), 3),
                        round(float(failure_goal.pose.position.y), 3),
                    ]
                release_request_id = int(
                    request(
                        "target_route_failed",
                        target_track_id=claimed_track_id,
                        target_room_claim_release=True,
                        target_room_claim=False,
                        target_room_claim_release_reason=str(reason),
                        failed_target_goal=failed_goal,
                        controller_failure=True,
                    )
                    or 0
                )
        self.publish_goal_arbitration(
            "target_route_failed_released",
            reason=str(reason),
            status=str(status),
            target_track_id=claimed_track_id,
            failure_count=failure_count,
            target_room_claim_release=bool(release_request_id > 0),
            replan_request_id=release_request_id,
            controller_lease="released",
            target_ownership="cleared",
        )
        rospy.logwarn(
            "GoalManager: released failed target route to fresh frontier "
            "route track=%s status=%s replan_id=%d",
            claimed_track_id or "-",
            str(status),
            release_request_id,
        )
        return release_request_id

    def release_target_follow_to_frontier(
        self,
        now: float,
        replan_reason: str = "target_segment_complete",
        semantic_hint_map: Optional[Tuple[float, float]] = None,
        target_release_reason: str = "target_track_cleared",
        preserve_obligation: Optional[bool] = None,
    ) -> Optional[PoseStamped]:
        """Release a completed visual segment without leaving a stale goal.

        A frontier saved before the visual approach may now be far behind the
        robot. Request a new map-connected branch instead of resuming that
        stale endpoint. The short replan wait is intentional and observable;
        a long unvalidated reverse route is not.
        """
        # A semantic hint must be applied to a newly selected branch. Reusing
        # a latched frontier here would silently discard the hint and can send
        # the robot back along a route selected before it saw the target.
        if preserve_obligation is None:
            preserve_obligation = bool(
                not getattr(self, "task_done_published", False)
                and (
                    getattr(self, "target_follow_confirmed", False)
                    or int(getattr(self, "target_completed_segments", 0)) > 0
                )
            )
        else:
            preserve_obligation = bool(preserve_obligation)
        frontier = (
            None
            if semantic_hint_map is not None
            else self.fresh_global_frontier_goal(now)
        )
        if preserve_obligation:
            # A cached frontier may belong to another Place. Replan under a
            # target-room claim before releasing visual ownership.
            frontier = None
        previous_target = self.target_last_goal
        self.clear_target_memory(
            target_release_reason,
            preserve_obligation=preserve_obligation,
        )
        if frontier is None:
            if not preserve_obligation:
                self.request_global_frontier_replan(
                    replan_reason,
                    previous_target=(
                        None
                        if previous_target is None
                        else [
                            round(float(previous_target.pose.position.x), 3),
                            round(float(previous_target.pose.position.y), 3),
                        ]
                    ),
                    semantic_hint_map=(
                        None
                        if semantic_hint_map is None
                        else [
                            round(float(semantic_hint_map[0]), 3),
                            round(float(semantic_hint_map[1]), 3),
                        ]
                    ),
                )
            rospy.logwarn(
                "GoalManager: visual segment reached; waiting for fresh frontier replan"
            )
            return None
        self.goal_source = "global_slam_frontier"
        rospy.loginfo(
            "GoalManager: release completed target segment to fresh frontier "
            "goal=(%.2f,%.2f) previous_target=(%.2f,%.2f)",
            frontier.pose.position.x,
            frontier.pose.position.y,
            previous_target.pose.position.x
            if previous_target is not None
            else float("nan"),
            previous_target.pose.position.y
            if previous_target is not None
            else float("nan"),
        )
        return frontier
