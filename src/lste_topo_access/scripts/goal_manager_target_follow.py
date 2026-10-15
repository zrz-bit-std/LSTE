"""Target-follow mission lifecycle for GoalManager.

The visual target route is intentionally a small state machine.  Each method
below owns one lifecycle boundary so target evidence, TEB terminals, and
frontier recovery can be edited independently.
"""

import copy
import math
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

CATCH_TARGET_MODE = "catch_target_mode"
EXPLORE_SUS_C_MODE = "explore_sus_c_mode"


class GoalManagerTargetFollowMixin:
    """Target-follow decision policy used by the Goal Manager host."""

    def target_observation_loss_certified(self) -> bool:
        """Return whether the current confirmed track has explicit loss proof."""
        gate = getattr(self, "target_observation_gate", None)
        if gate is None:
            return False
        return bool(gate.loss_certified(getattr(self, "target_track_id", "")))

    def _release_confirmed_target_after_loss(self, now: float):
        """Release the Place lease only after the observation gate commits loss."""
        if not self.target_observation_loss_certified():
            return None
        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        if approach_transaction is not None and approach_transaction.active:
            approach_transaction.mark_lost(now, "target_loss_certified")
        gate = getattr(self, "target_observation_gate", None)
        snapshot = None if gate is None else gate.snapshot().__dict__
        self.target_execution_state = "TARGET_LOSS_CERTIFIED"
        self.publish_goal_arbitration(
            "target_loss_certified",
            target_track_id=self.target_track_id,
            negative_observations=(
                None if snapshot is None else snapshot["negative_observations"]
            ),
            detector_epoch=(
                None if snapshot is None else snapshot["detector_epoch"]
            ),
            target_observation_gate=snapshot,
        )
        return self.release_target_follow_to_frontier(
            now,
            replan_reason="target_loss_certified",
            target_release_reason="target_loss_certified",
            preserve_obligation=False,
        )

    def _request_target_reinspection(self, now: float):
        """Keep the target Place while acquiring a genuinely new viewpoint."""
        gate = getattr(self, "target_observation_gate", None)
        if gate is None or gate.loss_certified(getattr(self, "target_track_id", "")):
            return None
        self.target_reinspection_pending = True
        self.target_execution_state = "TARGET_REINSPECTION"
        self.set_navigation_hold(False, "target_viewpoint_insufficient")
        return self.release_target_follow_to_frontier(
            now,
            replan_reason="target_viewpoint_insufficient",
            target_release_reason="target_viewpoint_insufficient",
            # An unconfirmed candidate is still a Place-owned evidence
            # obligation. Releasing without preservation erased the only
            # target-room lead after one weak/misaligned detector frame.
            preserve_obligation=True,
        )

    def expire_unconfirmed_target_candidate(self, now: float) -> bool:
        """Release an unconfirmed candidate after its evidence epoch expires.

        Candidate room ownership is an explicit transaction.  It must not stay
        latched merely because the scheduler has fallen back to geometric
        frontier mode before the candidate was confirmed.
        """
        # A target-room reinspection route is already the successor requested
        # by the candidate's durable claim.  Its command carries the new
        # route/graph/lifecycle identity, so clearing the candidate here would
        # immediately request another replan and erase that route before it
        # can be adopted.  Let the matching route terminal or explicit loss
        # event close this information transaction.
        route_active = getattr(self, "target_reinspection_route_active", None)
        if callable(route_active) and route_active():
            return False
        # A parallax viewpoint is an active information action, not a passive
        # detector cache.  Its route can legitimately outlive the candidate
        # evidence timeout while Navfn/TEB is still moving the robot.  Letting
        # this clock clear the candidate publishes a frontier transaction over
        # the live viewpoint and breaks the target terminal contract.
        if (
            getattr(self, "target_parallax_goal", None) is not None
            and not getattr(self, "target_parallax_completed", False)
        ):
            return False
        # The room-search/reacquisition route has the same ownership rule.
        # Its endpoint is stored separately from ``target_last_goal`` because
        # it is an information action rather than a confirmed pursuit segment.
        # Candidate expiry must not clear that route before its terminal.
        if (
            getattr(self, "target_reacquire_goal", None) is not None
            and getattr(self, "target_reacquire_started", None) is not None
        ):
            return False
        if not getattr(self, "target_candidate_room_claim_requested", False):
            return False
        if getattr(self, "target_follow_confirmed", False):
            return False
        if getattr(self, "target_blocked", False):
            return False
        last_seen = getattr(self, "target_candidate_last_seen", None)
        if last_seen is None:
            last_seen = getattr(self, "target_last_seen", None)
        if last_seen is not None and now - float(last_seen) < float(
            getattr(self, "target_follow_candidate_timeout", 0.0)
        ):
            return False
        self.publish_goal_arbitration(
            "target_candidate_expired",
            target_track_id=str(getattr(self, "target_track_id", "") or ""),
            age_seconds=(
                None
                if last_seen is None
                else round(float(now - last_seen), 3)
            ),
            reason="candidate_evidence_expired",
        )
        gate = getattr(self, "target_observation_gate", None)
        if gate is not None and gate.active:
            # A weak target track still has a Place-owned observation
            # obligation. Reinspect the room before allowing an outward
            # Portal; only a multi-view negative episode may release it.
            self.target_reinspection_pending = True
            self.target_execution_state = "TARGET_REINSPECTION"
            self.release_target_follow_to_frontier(
                now,
                replan_reason="target_candidate_reinspection",
                target_release_reason="target_candidate_reinspection",
                preserve_obligation=True,
            )
        else:
            try:
                self.clear_target_memory(preserve_obligation=False)
            except TypeError:
                # Narrow policy fixtures may expose the legacy no-argument
                # facade; their clear operation already means full release.
                self.clear_target_memory()
        return True

    def goal_from_target_follow(self, now: float) -> Optional[PoseStamped]:
        """Resolve one target-follow decision through explicit lifecycle stages."""
        handled, goal = self._resolve_target_follow_preconditions(now)
        if handled:
            return goal
        if self.target_last_goal is None:
            return self._start_confirmed_target_segment(now)
        return self._follow_active_target_segment(now)

    def _complete_reached_target_parallax(self, now: float) -> bool:
        """Close a physical parallax boundary when the action terminal lags."""
        goal = getattr(self, "target_parallax_goal", None)
        if goal is None or getattr(self, "target_parallax_completed", False):
            return False
        if not getattr(self, "target_candidate_viewpoint_diverse", False):
            return False
        try:
            distance = self.goal_robot_distance(goal)
        except (AttributeError, TypeError, ValueError):
            distance = None
        if distance is None:
            return False
        try:
            boundary = max(
                0.30,
                min(0.55, float(getattr(self, "target_goal_reached_radius", 0.5))),
            )
        except (TypeError, ValueError):
            boundary = 0.50
        if float(distance) > boundary:
            return False
        self.target_parallax_goal = None
        self.target_parallax_active_side = ""
        self.target_parallax_completed = True
        self.target_execution_state = "TARGET_PARALLAX_REOBSERVE"
        self.next_update_time = 0.0
        # This local boundary must also become a bridge ownership boundary.
        # Otherwise the persistent controller keeps executing the old
        # parallax action while the semantic layer waits for new evidence.
        handoff = getattr(self, "publish_target_terminal_observation_intent", None)
        if callable(handoff):
            handoff("target_parallax_viewpoint_reached_physical")
        self.publish_goal_arbitration(
            "target_parallax_viewpoint_reached_physical",
            target_track_id=getattr(self, "target_track_id", ""),
            distance=round(float(distance), 3),
            boundary=round(float(boundary), 3),
            evidence="viewpoint_diversity",
            ros_time=round(float(now), 3),
        )
        rospy.loginfo(
            "GoalManager: closed target parallax at physical boundary "
            "distance=%.3fm boundary=%.3fm",
            float(distance),
            float(boundary),
        )
        return True

    def _resolve_target_follow_preconditions(
        self, now: float
    ) -> Tuple[bool, Optional[PoseStamped]]:
        """Handle failed, unconfirmed, and continuity-deferred target tracks."""
        if self.target_blocked:
            self.target_execution_state = "TARGET_BLOCKED"
            self.goal_source = "target_route_blocked"
            if self.global_frontier_enabled:
                frontier = self.fresh_global_frontier_goal(now)
                if frontier is not None:
                    self.goal_source = "global_slam_frontier"
                    return True, frontier
                self.request_global_frontier_replan("target_route_blocked")
            # The bridge cancelled this action. Re-publishing the failed pose
            # under another source would turn a recovery wait into a retry.
            return True, None

        if not self.target_follow_confirmed:
            return self._handle_unconfirmed_target_candidate(now)

        if self.target_route_continuity_deferred:
            # A sharp first visual route is retained as evidence while the
            # healthy frontier action reaches its natural terminal boundary.
            if (
                self.last_goal is not None
                and self.last_goal_source == "global_slam_frontier"
                and self.teb_terminal_goal is None
            ):
                self.goal_source = "global_slam_frontier"
                return True, self.last_goal
            self.target_route_continuity_deferred = False
            self.target_route_continuity_deferred_goal = None
            self.publish_goal_arbitration(
                "target_takeover_continuity_boundary_reached",
                target_track_id=self.target_track_id,
            )
        return False, None

    def _handle_unconfirmed_target_candidate(
        self, now: float
    ) -> Tuple[bool, Optional[PoseStamped]]:
        """Keep a single detector frame from taking over a safe frontier route."""
        parallax = self._target_parallax_observation_goal(now)
        if parallax is not None:
            self.goal_source = "target_parallax"
            return True, parallax
        # A completed information action with an unconfirmed candidate is a
        # reinspection boundary. The ordinary reacquisition helper waits for
        # evidence expiry, but waiting here would strand the target owner while
        # the detector is still seeing the same object. Reuse its bounded,
        # Navfn-validated continuation exactly once.
        if getattr(self, "target_parallax_completed", False):
            reacquire = self.target_reacquisition_goal(now, force=True)
            if reacquire is not None:
                self.goal_source = "target_reacquisition_sweep"
                return True, reacquire
        if getattr(self, "target_candidate_room_claim_requested", False):
            frontier = self.fresh_global_frontier_goal(now)
            if frontier is not None:
                self.goal_source = "target_candidate_room_search"
                return True, frontier
            # A claimed-place replan forms an ownership boundary. Reusing the
            # previous route could immediately drive out of the candidate room.
            self.goal_source = "target_candidate_room_replan"
            return True, None
        if (
            self.last_goal is not None
            and self.last_goal_source == "global_slam_frontier"
        ):
            self.goal_source = self.last_goal_source
            return True, self.last_goal
        self.goal_source = "target_candidate_pending"
        return True, self.goal_from_frontiers_prior()

    def _target_parallax_observation_goal(self, now: float) -> Optional[PoseStamped]:
        """Compile one lateral view that can make target depth observable.

        A confirmed target requires physical viewpoint diversity.  When the
        robot is stationary, repeatedly re-running the detector cannot create
        that evidence, so the semantic layer explicitly asks Navfn for one
        short side-step.  The route is still validated by the same service as
        normal target segments and is bounded to two side choices; it cannot
        become an unbounded pursuit retry.
        """
        # Once a side-step has been validated and published, it is its own
        # action transaction.  The first few metres of that motion may already
        # make the candidate viewpoint diverse; that fact must not let a later
        # detector/timer callback replace the still-active parallax route.
        existing = getattr(self, "target_parallax_goal", None)
        if existing is not None:
            return existing
        if getattr(self, "target_follow_confirmed", False) or getattr(
            self, "target_candidate_viewpoint_diverse", False
        ):
            return None
        candidate_hits = int(getattr(self, "target_candidate_hits", 0))
        confirm_hits = int(getattr(self, "target_follow_confirm_hits", 1))
        if candidate_hits < confirm_hits:
            # A strong direct box is enough to justify one bounded information
            # action, even before the second frame arrives.  The side-step is
            # not target confirmation and cannot own pursuit; it only creates
            # the translational parallax required by the later evidence gate.
            candidate = getattr(self, "target_candidate", None)
            try:
                strong_candidate = (
                    candidate is not None
                    and (
                        float(candidate.score)
                        >= max(0.40, float(self.target_done_min_score))
                        or max(float(candidate.w), float(candidate.h)) >= 0.05
                    )
                )
            except (AttributeError, TypeError, ValueError):
                strong_candidate = False
            if not strong_candidate:
                return None
        if self.latest_pose is None:
            return None
        if int(getattr(self, "target_parallax_attempts", 0)) >= 2:
            return None
        heading = self.target_last_heading
        if heading is None:
            return None
        try:
            # The viewpoint compiler already defines the smallest endpoint
            # outside the target action's arrival radius.  Reusing that
            # contract keeps this information action executable by TEB; a
            # shorter local shortcut can be accepted without producing a new
            # camera viewpoint at all.
            step = float(self.target_minimum_viewpoint_distance())
        except (AttributeError, TypeError, ValueError):
            return None
        failed_sides = set(
            str(side)
            for side in getattr(self, "target_parallax_failed_sides", [])
        )
        # Try the left side first, then the right side if the current costmap
        # rejects it.  The deterministic order is part of the replay contract;
        # a controller failure also closes that side permanently for this
        # target episode.
        for side in (1.0, -1.0):
            side_name = "left" if side > 0.0 else "right"
            if side_name in failed_sides:
                continue
            # A purely perpendicular side-step makes a forward-only base turn
            # in place before it can create any parallax.  Use a diagonal
            # information step instead: it still changes the camera viewpoint
            # on either side of the target ray, while retaining a forward
            # component that TEB can execute through a narrow doorway.
            lateral_heading = float(heading) + side * math.pi * 0.25
            requested_odom = self.make_goal_pose(
                (
                    float(self.latest_pose.x) + step * math.cos(lateral_heading),
                    float(self.latest_pose.y) + step * math.sin(lateral_heading),
                    0.0,
                ),
                float(heading),
            )
            requested_goal = self._pose_in_frame(requested_odom, "map")
            if requested_goal is None:
                continue
            route_status = self.validate_target_route(
                requested_goal, now, force=(self.target_parallax_attempts > 0)
            )
            if route_status is not True:
                if route_status is False:
                    if side_name not in failed_sides:
                        self.target_parallax_failed_sides.append(side_name)
                    self.target_parallax_attempts += 1
                continue
            endpoint = self.target_route_validation_last_endpoint
            if endpoint is None:
                continue
            self.target_parallax_attempts += 1
            self.target_parallax_goal = copy.deepcopy(endpoint)
            self.target_parallax_active_side = side_name
            self.target_execution_state = "TARGET_PARALLAX"
            self.publish_goal_arbitration(
                "target_parallax_viewpoint_selected",
                target_track_id=self.target_track_id,
                side=side_name,
                requested_goal=[
                    round(float(requested_goal.pose.position.x), 3),
                    round(float(requested_goal.pose.position.y), 3),
                ],
                goal=[
                    round(float(endpoint.pose.position.x), 3),
                    round(float(endpoint.pose.position.y), 3),
                ],
                distance=round(float(step), 3),
                attempt=int(self.target_parallax_attempts),
                evidence_basis=(
                    "strong_single_frame"
                    if candidate_hits < confirm_hits
                    else "confirmed_candidate_frames"
                ),
                candidate_hits=candidate_hits,
            )
            rospy.loginfo(
                "GoalManager: selected active target parallax viewpoint "
                "side=%s goal=(%.2f,%.2f) attempt=%d",
                side_name,
                endpoint.pose.position.x,
                endpoint.pose.position.y,
                self.target_parallax_attempts,
            )
            return self.target_parallax_goal
        return None

    def _start_confirmed_target_segment(self, now: float) -> Optional[PoseStamped]:
        """Commit the first validated target segment or retain exploration."""
        candidate = self.commit_target_segment(now, "target_follow")
        if candidate is not None:
            return candidate
        if (
            self.target_route_continuity_deferred
            and self.last_goal is not None
            and self.last_goal_source == "global_slam_frontier"
        ):
            self.goal_source = "global_slam_frontier"
            return self.last_goal
        # A rejected visual ray must not interrupt a map-connected exploration
        # transaction. The detector track remains available for reconsideration
        # once SLAM reveals a connected route.
        frontier = self.fresh_global_frontier_goal(now)
        if frontier is not None:
            self.goal_source = "global_slam_frontier"
            return frontier
        if (
            self.last_goal is not None
            and self.last_goal_source == "global_slam_frontier"
        ):
            self.goal_source = "global_slam_frontier"
            return self.last_goal
        return None

    def _follow_active_target_segment(self, now: float) -> Optional[PoseStamped]:
        """Advance, observe, or release an already committed target segment."""
        self._reset_target_reacquisition()
        current_distance = self._target_segment_distance_from_robot()
        candidate = self._prepare_continuous_target_successor(now, current_distance)
        if candidate is not None:
            return candidate
        if current_distance <= self.target_goal_reached_radius:
            return self._handle_target_segment_terminal(now)
        if self.target_goal_detection_stamp == self.target_last_detection_stamp:
            self.goal_source = "target_follow"
        else:
            self.goal_source = "target_cached"
        return self.target_last_goal

    def _reset_target_reacquisition(self):
        """Clear retry state while a committed segment remains healthy."""
        self.target_reacquire_goal = None
        self.target_reacquire_started = None
        self.target_reacquire_attempts = 0

    def _target_segment_distance_from_robot(self) -> float:
        """Measure remaining distance, or infinity until a base pose is known."""
        if self.latest_pose is None:
            return float("inf")
        measured_distance = self.goal_robot_distance(self.target_last_goal)
        return float("inf") if measured_distance is None else measured_distance

    def _prepare_continuous_target_successor(
        self, now: float, current_distance: float
    ) -> Optional[PoseStamped]:
        """Prepare a fresh successor only inside the existing handoff window."""
        if not self.can_prepare_target_continuous_handoff(current_distance):
            return None
        fresh_frames = self.target_observation_epoch - self.target_segment_commit_epoch
        candidate = self.commit_target_segment(
            now, "target_continuous_handoff", advance=True
        )
        if candidate is not None:
            rospy.loginfo(
                "GoalManager: prepared continuous target successor "
                "distance=%.2fm new_frames=%d",
                current_distance,
                fresh_frames,
            )
        return candidate

    def _handle_target_segment_terminal(self, now: float) -> Optional[PoseStamped]:
        """Resolve the terminal observation boundary of one visual segment."""
        if self.controller_mode == "teb" and not self.target_segment_terminal_ready:
            self.goal_source = "target_waiting_terminal"
            return self.target_last_goal
        if self.target_observation_loss_certified():
            return self._release_confirmed_target_after_loss(now)
        if self._await_required_terminal_observations(now):
            return None

        # Once terminal continuity has begun collecting the configured
        # completion votes, preserve this physical viewpoint until the vote
        # completes or fresh evidence expires. Advancing immediately on the
        # same first eligible frame repeatedly reset terminal readiness and
        # kept the close-hit/hold contract from ever completing.
        if self._hold_close_target_confirmation(now):
            return None

        attempted, candidate = self._try_blind_target_terminal_advance(now)
        if candidate is not None:
            return candidate
        if attempted:
            handled, goal = self._handle_blocked_terminal_target_route(now)
            if handled:
                return goal
            return self._finish_unavailable_terminal_continuation(now)

        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        if (
            approach_transaction is not None
            and approach_transaction.active
            and self.target_follow_confirmed
        ):
            if self.target_tracking_active(now):
                self.target_execution_state = "TARGET_REOBSERVING"
                self.goal_source = "target_waiting_fresh_observation"
                return self.target_last_goal
            gate = getattr(self, "target_observation_gate", None)
            if gate is not None and gate.needs_viewpoint_reinspection:
                return self._request_target_reinspection(now)
            approach_transaction.mark_lost(
                now, "target_evidence_expired"
            )
        frontier = self.release_target_follow_to_frontier(now)
        if frontier is not None:
            return frontier
        self.goal_source = "target_waiting_fresh_observation"
        return self.target_last_goal

    def _await_required_terminal_observations(self, now: float) -> bool:
        """Hold until the post-arrival detector-frame boundary is satisfied."""
        if not (
            self.target_terminal_reobserve_pending
            and self.target_observation_epoch < self.target_terminal_reobserve_min_epoch
            and now < self.target_terminal_reobserve_until
        ):
            return False
        self.target_execution_state = "TARGET_REOBSERVING"
        self.goal_source = "target_reobserving"
        self.set_navigation_hold(True, "await_post_terminal_frames")
        self.publish_goal_arbitration(
            "target_terminal_reobserve",
            observed_epoch=int(self.target_observation_epoch),
            required_epoch=int(self.target_terminal_reobserve_min_epoch),
            remaining_seconds=round(self.target_terminal_reobserve_until - now, 3),
        )
        return True

    def _try_blind_target_terminal_advance(
        self, now: float
    ) -> Tuple[bool, Optional[PoseStamped]]:
        """Advance only when a new target observation supports the next view.

        ``target_cache_max_advances`` used to turn this branch into an
        arbitrary mission boundary. A confirmed target transaction may now
        continue for any number of segments, but a stale bearing alone never
        authorizes another route. The transaction either receives fresh
        evidence or remains at the current viewpoint until the track is
        explicitly lost or the route explicitly fails.
        """
        fresh_evidence = (
            int(getattr(self, "target_observation_epoch", 0))
            > int(getattr(self, "target_terminal_reobserve_epoch", 0))
        )
        if not (
            self.target_last_heading is not None
            and fresh_evidence
            and getattr(
                getattr(self, "target_approach_transaction", None),
                "active",
                True,
            )
        ):
            return False, None
        candidate = self.commit_target_segment(
            now, "target_terminal_advance", advance=True
        )
        if candidate is not None:
            self.target_terminal_reobserve_pending = False
            self.set_navigation_hold(False, "post_terminal_route_validated")
        return True, candidate

    def _handle_blocked_terminal_target_route(
        self, now: float
    ) -> Tuple[bool, Optional[PoseStamped]]:
        """Keep close evidence or request a semantic frontier after Navfn rejection."""
        if self.target_route_validation_last_result != "blocked":
            return False, None
        close_evidence_active = (
            self.target_completed_segments >= 1
            and self.target_close_since is not None
            and self.target_close_last_seen is not None
            and now - self.target_close_last_seen
            <= max(self.target_done_max_detection_age, 2.0)
        )
        if close_evidence_active:
            self.target_execution_state = "TARGET_CLOSE_REOBSERVING"
            self.goal_source = "target_reobserving"
            self.set_navigation_hold(True, "await_close_confirmation")
            self.publish_goal_arbitration(
                "target_close_route_blocked_hold",
                target_track_id=self.target_track_id,
                completed_segments=int(self.target_completed_segments),
                close_hits=int(self.target_close_hits),
                remaining_seconds=round(
                    max(0.0, self.target_terminal_reobserve_until - now), 3
                ),
            )
            return True, None
        semantic_hint = self.target_semantic_hint_map()
        if semantic_hint is None:
            return False, None
        self.target_terminal_reobserve_pending = False
        self.set_navigation_hold(False, "blocked_target_semantic_frontier")
        self.publish_goal_arbitration(
            "target_route_semantic_replan",
            semantic_hint_map=[
                round(float(semantic_hint[0]), 3),
                round(float(semantic_hint[1]), 3),
            ],
        )
        frontier = self.release_target_follow_to_frontier(
            now,
            replan_reason="target_route_blocked_semantic_hint",
            semantic_hint_map=semantic_hint,
        )
        if frontier is not None:
            return True, frontier
        self.goal_source = "target_waiting_semantic_frontier"
        return True, None

    def _finish_unavailable_terminal_continuation(
        self, now: float
    ) -> Optional[PoseStamped]:
        """Hold a confirmed target until evidence or an explicit loss event.

        Frontier exhaustion is a geometric fact about map boundaries. It is not
        evidence that a still-confirmed visual target disappeared, so it cannot
        close this transaction.
        """
        if self.target_observation_loss_certified():
            return self._release_confirmed_target_after_loss(now)
        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        if (
            approach_transaction is not None
            and approach_transaction.active
            and self.target_follow_confirmed
        ):
            if self.target_tracking_active(now):
                self.target_execution_state = "TARGET_TERMINAL_OBSERVING"
                self.goal_source = "target_terminal_observation"
                self.set_navigation_hold(True, "target_terminal_observation")
                self.publish_target_terminal_observation_intent(
                    "navfn_unavailable_after_target_terminal"
                )
                self.publish_goal_arbitration(
                    "target_terminal_observation_waiting",
                    route_validation=self.target_route_validation_last_result,
                    completed_segments=int(self.target_completed_segments),
                )
                return None
            gate = getattr(self, "target_observation_gate", None)
            if gate is not None and gate.needs_viewpoint_reinspection:
                return self._request_target_reinspection(now)
            else:
                approach_transaction.mark_lost(
                    now, "target_evidence_expired"
                )
        if now < self.target_terminal_reobserve_until:
            self.target_terminal_reobserve_pending = True
            self.target_terminal_reobserve_min_epoch = max(
                self.target_terminal_reobserve_min_epoch,
                int(self.target_observation_epoch) + 1,
            )
            self.target_execution_state = "TARGET_REOBSERVING"
            self.goal_source = "target_reobserving"
            self.set_navigation_hold(True, "post_terminal_navfn_reobserve")
            self.publish_goal_arbitration(
                "target_terminal_route_reobserve",
                observed_epoch=int(self.target_observation_epoch),
                required_epoch=int(self.target_terminal_reobserve_min_epoch),
                route_validation=self.target_route_validation_last_result,
                remaining_seconds=round(self.target_terminal_reobserve_until - now, 3),
            )
            return None
        self.target_terminal_reobserve_pending = False
        self.target_execution_state = "TARGET_TERMINAL_OBSERVING"
        self.goal_source = "target_terminal_observation"
        self.set_navigation_hold(True, "target_terminal_observation")
        self.publish_target_terminal_observation_intent(
            "target_terminal_reobserve_expired"
        )
        self.publish_goal_arbitration(
            "target_terminal_observation_waiting",
            route_validation=self.target_route_validation_last_result,
            completed_segments=int(self.target_completed_segments),
        )
        return None

    def _hold_close_target_confirmation(self, now: float) -> bool:
        """Keep the car at the viewpoint while close-range votes are pending."""
        if not self.target_close_confirmation_active(now):
            return False
        self.target_execution_state = "TARGET_CLOSE_REOBSERVING"
        self.goal_source = "target_reobserving"
        self.set_navigation_hold(True, "await_close_confirmation")
        if not self.target_close_wait_reported:
            self.target_close_wait_reported = True
            self.publish_goal_arbitration(
                "target_close_confirmation_waiting",
                target_track_id=self.target_track_id,
                approach_track_id=self.target_approach_track_id,
                completed_segments=int(self.target_completed_segments),
                close_hits=int(self.target_close_hits),
                required_hits=int(self.target_done_min_fresh_hits),
                remaining_seconds=round(
                    self.target_close_confirmation_grace()
                    - (now - self.target_close_last_seen),
                    3,
                ),
            )
        return True

    def _fallback_after_target_loss(self, now: float) -> Optional[PoseStamped]:
        """Retain a briefly lost track, then return the state machine to search."""
        gate = getattr(self, "target_observation_gate", None)
        if gate is not None and gate.needs_viewpoint_reinspection:
            return self._request_target_reinspection(now)
        if self.target_last_seen is not None and (
            now - self.target_last_seen
        ) < self.follow_target_lost_timeout:
            self.goal_source = "target_cached"
            return self.last_goal
        if self.effective_mode == CATCH_TARGET_MODE:
            rospy.loginfo_throttle(
                2.0,
                "GoalManager: target lost >= %.1fs, fallback to explore_sus_c_mode",
                self.follow_target_lost_timeout,
            )
            self.effective_mode = EXPLORE_SUS_C_MODE
            self.pub_access_mode.publish(String(data=self.effective_mode))
            approach_transaction = getattr(
                self, "target_approach_transaction", None
            )
            if approach_transaction is not None:
                approach_transaction.mark_lost(
                    now, "target_evidence_expired"
                )
            self.clear_target_memory()
        return self.goal_from_frontiers_prior()
