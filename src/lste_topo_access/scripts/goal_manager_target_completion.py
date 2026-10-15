#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Visual target completion evidence and bounded viewpoint selection."""

import math
from typing import List

class GoalManagerTargetCompletionMixin:
    def reset_target_close_confirmation(self):
        self.target_close_since = None
        self.target_close_hits = 0
        self.target_close_last_stamp = None
        self.target_close_last_seen = None
        self.target_close_wait_reported = False

    def reset_target_terminal_observation(self):
        """Start a fresh post-terminal completion evidence transaction."""
        self.target_terminal_close_candidate_seen = False
        self.target_terminal_observation_intent_sent = False
        self.target_terminal_observation_pending_transaction = 0
        self.target_terminal_observation_ack_transaction = 0

    def target_terminal_observation_eligible(self, det) -> bool:
        """Bridge a close-box dip with continuity of the same target track.

        A terminal target route is already a validated physical viewpoint.
        Once a post-terminal frame has passed the close-box rule, later
        frames from the same confirmed track may temporarily fall below that
        pixel-size rule because of detector jitter. Treating those frames as
        negative completion evidence creates a false recovery problem: TEB has
        reached the viewpoint and has no safe forward motion left. The
        fallback accepts only the existing track evidence, after a terminal,
        and within the same observation epoch.
        """
        if det is None:
            return False
        candidate_seen = bool(
            getattr(self, "target_terminal_close_candidate_seen", False)
        )
        if not (
            bool(getattr(self, "target_follow_confirmed", False))
            and int(getattr(self, "target_completed_segments", 0) or 0) >= 1
            and bool(getattr(self, "target_segment_terminal_ready", False))
            and str(getattr(self, "target_track_id", "") or "")
            == str(getattr(self, "target_approach_track_id", "") or "")
        ):
            return False
        if not candidate_seen:
            # A validated terminal is already an approved camera viewpoint.
            # When the same confirmed track remains visible there, a detector
            # box just below the pixel close rule must enter the existing
            # terminal-continuity vote instead of reopening the same terminal
            # observation transaction forever. The physical endpoint envelope
            # and fresh-frame vote below remain mandatory.
            distance_fn = getattr(self, "goal_robot_distance", None)
            goal = getattr(self, "target_last_goal", None)
            radius = getattr(self, "target_goal_reached_radius", None)
            if not callable(distance_fn) or goal is None or radius is None:
                return False
            try:
                if float(distance_fn(goal)) > float(radius):
                    return False
            except (TypeError, ValueError):
                return False
        gate = getattr(self, "target_observation_gate", None)
        if gate is not None and gate.loss_certified(
            getattr(self, "target_track_id", "")
        ):
            return False
        try:
            return (
                float(det.score) >= float(self.target_follow_min_score)
                and max(float(det.w), float(det.h))
                >= float(self.target_follow_min_box_size)
                and int(getattr(self, "target_observation_epoch", 0) or 0)
                >= int(getattr(self, "target_terminal_reobserve_min_epoch", 0) or 0)
            )
        except (AttributeError, TypeError, ValueError):
            return False

    def target_close_completion_eligible(self) -> bool:
        """Return whether this target track may collect close completion votes.

        A terminal target segment remains the strongest form of approach
        evidence.  For the normal search task, however, a two-frame confirmed
        direct target that is already close in the camera is also sufficient
        to begin the configured completion transaction.  The vehicle stays at
        that safe viewpoint while the remaining frames arrive.  Context labels
        are deliberately absent from this decision.
        """
        approach_terminal = bool(
            self.target_completed_segments >= 1
            and self.target_track_id
            and self.target_approach_track_id == self.target_track_id
        )
        direct_close_track = bool(
            self.target_follow_confirmed and self.target_track_id
        )
        return approach_terminal or (
            not self.target_done_require_approach_terminal
            and direct_close_track
        )

    def target_viewpoint_portfolio_complete(self, now: float) -> bool:
        """Return whether the current target option ledger is exhausted.

        This is a lifecycle boundary, not task completion evidence. A finite
        set of currently known viewpoints cannot prove that a target was
        reached when target geometry remains uncertain; the caller must either
        have independent close evidence or open a fresh local revalidation
        transaction.
        """
        if not self.target_close_completion_eligible():
            return False
        transaction = getattr(self, "target_approach_transaction", None)
        ledger = (
            None
            if transaction is None
            else getattr(transaction, "viewpoint_ledger", None)
        )
        if ledger is None or not ledger.alternatives_exhausted():
            return False
        if bool(getattr(self, "target_terminal_reobserve_pending", False)) and int(
            getattr(self, "target_observation_epoch", 0) or 0
        ) < int(getattr(self, "target_terminal_reobserve_min_epoch", 0) or 0):
            return False
        latest = getattr(self, "latest_dets", None)
        det = self.target_detection_for_track(latest)
        if latest is None or det is None:
            return False
        try:
            stamp_age = float(now) - float(latest.header.stamp.to_sec())
            return (
                stamp_age >= 0.0
                and stamp_age <= max(
                    float(self.target_done_max_detection_age),
                    float(self.target_follow_candidate_timeout),
                )
                and float(det.score) >= float(self.target_follow_min_score)
                and max(float(det.w), float(det.h))
                >= float(self.target_follow_min_box_size)
            )
        except (AttributeError, TypeError, ValueError):
            return False

    def revalidate_target_viewpoint_portfolio(self, now: float) -> bool:
        """Open one fresh local observation transaction after ledger exhaustion.

        The bounded viewpoint ledger describes only the routes that were known
        when the current target episode was compiled.  Its exhaustion proves
        that those options are closed, not that the semantic target is at the
        last endpoint.  Preserve the target-room obligation, clear the stale
        route, and let the frontier layer discover a new local viewpoint.
        """
        if bool(getattr(self, "target_portfolio_revalidation_requested", False)):
            return False
        self.target_execution_state = "TARGET_PORTFOLIO_REVALIDATION"
        self.publish_goal_arbitration(
            "target_viewpoint_portfolio_requires_revalidation",
            target_track_id=str(getattr(self, "target_track_id", "") or ""),
            completed_segments=int(getattr(self, "target_completed_segments", 0)),
            reason="portfolio_exhausted_without_close_evidence",
        )
        hold = getattr(self, "set_navigation_hold", None)
        if callable(hold):
            hold(False, "target_portfolio_revalidation")
        release = getattr(self, "release_target_follow_to_frontier", None)
        if callable(release):
            release(
                now,
                replan_reason="target_viewpoint_portfolio_revalidation",
                target_release_reason="target_viewpoint_portfolio_revalidation",
                preserve_obligation=True,
            )
        self.target_portfolio_revalidation_requested = True
        return True

    def target_detection_is_close(self, det) -> bool:
        """Apply the single close-range image rule used by hold and completion."""
        if det is None or float(det.score) < self.target_close_score_threshold():
            return False
        width_close = (
            self.target_done_min_box_width > 0.0
            and float(det.w) >= self.target_done_min_box_width
        )
        height_close = (
            self.target_done_min_box_height > 0.0
            and float(det.h) >= self.target_done_min_box_height
        )
        if width_close or height_close:
            return True
        # A small object can remain below a pixel-size threshold even after a
        # well-conditioned multi-view estimate places the robot on its safe
        # observation circle.  This is still direct target evidence: require
        # the current tracked score and a static-point range check, but never
        # drive into the estimated object merely to enlarge its box.
        return self.target_hypothesis_at_safe_standoff()

    def target_hypothesis_at_safe_standoff(self) -> bool:
        """Return whether geometric target evidence has reached the view ring."""
        if not getattr(self, "target_hypothesis_navigation_enabled", False):
            return False
        hypothesis = getattr(self, "target_hypothesis_xy", None)
        pose = getattr(self, "latest_pose", None)
        try:
            ray_count = int(getattr(self, "target_hypothesis_ray_count", 0) or 0)
            target_x, target_y = float(hypothesis[0]), float(hypothesis[1])
            robot_x, robot_y = float(pose.x), float(pose.y)
            target_range = math.hypot(target_x - robot_x, target_y - robot_y)
            standoff = float(self.target_minimum_viewpoint_distance())
        except (AttributeError, IndexError, TypeError, ValueError):
            return False
        if ray_count < 2 or not all(
            math.isfinite(value)
            for value in (target_x, target_y, robot_x, robot_y, target_range, standoff)
        ):
            return False
        # Keep a small geometric envelope for map/odometry updates.  It is
        # derived from the existing arrival radius, not a new task parameter.
        envelope = max(0.10, min(0.25, 0.25 * float(self.target_goal_reached_radius)))
        return target_range <= standoff + envelope

    def target_close_confirmation_grace(self) -> float:
        """Return the bounded fresh-evidence gap allowed at a target endpoint.

        This is intentionally tied to the existing visual observation and
        candidate-confirmation windows, not to context recognition or a
        controller timeout. A detector can miss a monitor or one cup frame
        while the vehicle is stationary, but it must still produce another
        direct target frame within the same window that confirmed the track.
        Keeping those two lifetimes identical prevents a confirmed yellow cup
        from losing room ownership simply because the next slow detector frame
        does not contain the second monitor.
        """
        return max(
            float(self.target_done_max_detection_age),
            float(self.target_observation_hold),
            float(self.target_follow_candidate_timeout),
            float(self.target_follow_confirm_window),
        )

    def target_close_score_threshold(self) -> float:
        """Use direct tracked-target evidence for close-range confirmation.

        Before a visual track is confirmed, retain the stricter close score to
        reject a lone false box.  After a spatially confirmed target track has
        reached a validated approach segment, the track itself is stronger
        evidence than a flickering context pair; use the existing follow
        threshold but retain the multi-frame close-box requirement.
        """
        direct_track_confirmed = (
            self.target_follow_confirmed
            and self.target_completed_segments >= 1
            and bool(self.target_track_id)
            and self.target_approach_track_id == self.target_track_id
        )
        return (
            float(self.target_follow_min_score)
            if direct_track_confirmed
            else float(self.target_done_min_score)
        )

    def target_close_confirmation_active(self, now: float) -> bool:
        """Return whether close direct evidence still owns the viewpoint.

        A completed TEB approach is sufficient evidence, but it is not a
        prerequisite in the normal policy.  A directly observed and
        independently confirmed close target already has a dedicated
        multi-frame completion transaction.  It owns the current viewpoint
        through that transaction so the lower-priority monitor/context state
        cannot send the robot back out of the room.
        """
        return bool(
            self.target_close_completion_eligible()
            and self.target_close_since is not None
            and self.target_close_last_seen is not None
            and float(now) - float(self.target_close_last_seen)
            < self.target_close_confirmation_grace()
        )

    def clear_target_memory(
        self,
        target_release_reason="target_track_cleared",
        preserve_obligation=None,
    ):
        """Reset the active visual route without erasing a confirmed target.

        A detector track is a view-level identity.  After a confirmed target
        reaches a terminal, an empty view proves only that this viewpoint was
        insufficient.  The Place-owned target WorkItem remains unresolved
        until an explicit multi-view negative episode or ``task_done``.
        """
        claimed_track_id = str(self.target_track_id or "")
        gate = getattr(self, "target_observation_gate", None)
        gate_loss_certified = bool(
            gate is not None and gate.loss_certified(claimed_track_id)
        )
        confirmed_target = bool(
            getattr(self, "target_follow_confirmed", False)
            or int(getattr(self, "target_completed_segments", 0)) > 0
        )
        if preserve_obligation is None:
            # The live Goal Manager always owns a gate.  Keep the legacy
            # fallback explicit for narrow callers/tests that do not compose
            # that gate: without an evidence ledger there is no obligation to
            # preserve across a direct clear request.
            preserve_obligation = bool(
                confirmed_target
                and gate is not None
                and not gate_loss_certified
            )
        preserve_obligation = bool(preserve_obligation)
        release_room_claim = bool(
            self.target_candidate_room_claim_requested
            and not preserve_obligation
        )
        release_request_id = 0
        saved = {
            "last_seen": getattr(self, "target_last_seen", None),
            "last_heading": getattr(self, "target_last_heading", None),
            "heading_source_stamp": getattr(
                self, "target_last_heading_source_stamp", None
            ),
            "observation_odom": getattr(
                self, "target_last_observation_odom", None
            ),
            "filtered_cx": getattr(self, "target_filtered_cx", None),
            "filtered_cy": getattr(self, "target_filtered_cy", None),
            "filtered_heading": getattr(self, "target_filtered_heading", None),
            "track_label": getattr(self, "target_track_label", ""),
            "track_id": getattr(self, "target_track_id", ""),
            "hypothesis_xy": getattr(self, "target_hypothesis_xy", None),
            "hypothesis_residual": getattr(self, "target_hypothesis_residual", None),
            "hypothesis_ray_count": getattr(self, "target_hypothesis_ray_count", 0),
            "ray_history": list(getattr(self, "target_ray_history", [])),
        }
        self.target_last_seen = saved["last_seen"] if preserve_obligation else None
        self.target_last_goal = None
        self.target_last_update = 0.0
        self.target_last_heading = saved["last_heading"] if preserve_obligation else None
        self.target_last_navigation_heading = None
        self.target_last_heading_source_stamp = (
            saved["heading_source_stamp"] if preserve_obligation else None
        )
        self.target_last_observation_odom = (
            saved["observation_odom"] if preserve_obligation else None
        )
        self.target_filtered_cx = saved["filtered_cx"] if preserve_obligation else None
        self.target_filtered_cy = saved["filtered_cy"] if preserve_obligation else None
        self.target_filtered_heading = (
            saved["filtered_heading"] if preserve_obligation else None
        )
        self.target_last_detection_stamp = None
        observation_gate = getattr(self, "target_observation_gate", None)
        if observation_gate is not None:
            if preserve_obligation:
                observation_gate.begin_reobserve(
                    claimed_track_id,
                    int(getattr(self, "target_detector_frame_epoch", 0)),
                    getattr(self, "target_last_seen", 0.0) or 0.0,
                )
            else:
                observation_gate.clear("target_memory_cleared")
        self.target_goal_detection_stamp = None
        self.target_segment_terminal_ready = False
        self.target_segment_commit_epoch = 0
        self.target_terminal_reobserve_pending = False
        self.target_terminal_reobserve_epoch = 0
        self.target_terminal_reobserve_min_epoch = 0
        self.target_terminal_reobserve_until = 0.0
        self.target_terminal_observation_intent_sent = False
        self.target_terminal_observation_pending_transaction = 0
        self.target_terminal_observation_ack_transaction = 0
        self.target_track_label = saved["track_label"] if preserve_obligation else ""
        self.target_track_id = saved["track_id"] if preserve_obligation else ""
        self.target_terminal_blind_advances = 0
        self.target_completed_segments = 0
        self.target_approach_track_id = ""
        self.target_cache_advances = 0
        if not preserve_obligation:
            self.target_viewpoint_candidate_id = ""
            self.target_viewpoint_attempt_id = ""
            self.target_viewpoint_map_epoch = None
        self.target_observation_hold_goal = None
        self.target_candidate = None
        self.target_candidate_last_seen = None
        self.target_candidate_source_stamp = None
        self.target_candidate_hits = 0
        self.target_candidate_anchor_cx = None
        self.target_candidate_anchor_cy = None
        self.target_candidate_viewpoint_anchor_odom = None
        self.target_candidate_viewpoint_anchor_yaw = None
        self.target_candidate_viewpoint_translation = 0.0
        self.target_candidate_viewpoint_yaw_delta = 0.0
        self.target_candidate_viewpoint_diverse = False
        self.target_candidate_score_sum = 0.0
        self.target_ray_history = saved["ray_history"] if preserve_obligation else []
        self.target_candidate_room_claim_requested = preserve_obligation
        self.target_reinspection_pending = preserve_obligation
        self.target_follow_confirmed = False
        self.target_terminal_close_candidate_seen = False
        self.target_portfolio_revalidation_requested = False
        self.target_parallax_goal = None
        self.target_parallax_attempts = 0
        self.target_parallax_completed = False
        self.target_parallax_active_side = ""
        self.target_parallax_failed_sides = []
        self.target_direct_close_hold_reported = False
        self.target_execution_state = (
            "TARGET_REINSPECTION" if preserve_obligation else "TARGET_CANDIDATE"
        )
        self.target_observation_epoch = 0
        self.target_blocked = False
        self.target_blocked_goal = None
        self.target_blocked_since = None
        self.target_blocked_reason = ""
        self.target_failure_count = 0
        self.target_hypothesis_xy = (
            saved["hypothesis_xy"] if preserve_obligation else None
        )
        self.target_hypothesis_residual = (
            saved["hypothesis_residual"] if preserve_obligation else None
        )
        self.target_hypothesis_ray_count = (
            saved["hypothesis_ray_count"] if preserve_obligation else 0
        )
        self.target_observation_hold_until = 0.0
        self.target_route_validation_next_time = 0.0
        self.target_route_validation_failures = 0
        self.target_route_validation_last_result = "not_checked"
        self.target_route_validation_last_goal = None
        self.target_route_validation_last_endpoint = None
        self.target_route_validation_last_plan = []
        self.target_route_validation_last_start_heading = None
        self.target_route_continuity_deferred = False
        self.target_route_continuity_deferred_goal = None
        self.target_route_hold_last_emit = 0.0
        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        if approach_transaction is not None:
            if preserve_obligation and approach_transaction.active:
                approach_transaction.mark_lost(
                    getattr(self, "target_last_seen", 0.0) or 0.0,
                    "target_reinspection_required",
                )
            elif approach_transaction.active:
                approach_transaction.clear("target_memory_cleared")
        if release_room_claim:
            release_request_id = self.release_target_room_claim(
                str(target_release_reason or "target_track_cleared"),
                target_track_id=claimed_track_id,
            )
        elif preserve_obligation:
            retain = getattr(self, "retain_target_observation_room", None)
            if retain is not None:
                retain(
                    "target_viewpoint_insufficient",
                    target_track_id=claimed_track_id,
                )
        return int(release_request_id or 0)

    def target_segment_distance(self) -> float:
        """Return a controller-appropriate visual pursuit horizon.

        A monocular detection supplies a bearing, not a trustworthy metric
        range.  The goal must therefore stay within the configured visual
        horizon even for TEB: a long ray can be Navfn-reachable while routing
        around a wall to a place where the object never was.  Fresh detector
        frames can use the native target handoff path, so keeping this short
        no longer requires a terminal stop at every healthy segment boundary.
        """
        distance = self.follow_target_step_distance
        if self.target_approach_strategy != "reachable_viewpoint_ladder":
            return distance
        # RGB detections do not give a metric range, but their normalized box
        # scale is a stable, task-agnostic proxy for approach progress. Once
        # a target is close to the completion scale, shrink the next visual
        # horizon instead of driving the full 1.5 m past it and forcing a
        # stop/reobserve/backtrack cycle.
        det = self.target_detection_for_track(self.latest_dets)
        if det is None:
            return distance
        observed_scale = max(float(det.w), float(det.h))
        completion_scale = max(
            1e-3,
            min(self.target_done_min_box_width, self.target_done_min_box_height),
        )
        near_scale = 0.5 * completion_scale
        if observed_scale <= near_scale:
            return distance
        progress = min(
            1.0,
            (observed_scale - near_scale) / max(1e-3, completion_scale - near_scale),
        )
        minimum = self.target_minimum_viewpoint_distance()
        return max(minimum, distance * (1.0 - 0.40 * progress))

    def target_minimum_viewpoint_distance(self) -> float:
        """Smallest new target endpoint that is outside the arrival radius."""
        return min(
            self.follow_target_step_distance,
            max(self.target_goal_reached_radius + 0.20, 0.90),
        )

    def target_segment_distances(self) -> List[float]:
        """Return bounded map-validation candidates for one visual heading.

        ``legacy_ray`` preserves the former single fixed endpoint. The
        default ladder tries at most three monotonically nearer endpoints. It
        does not use target coordinates, simulator truth, or a controller
        command; every candidate still requires the live Navfn route proof.
        """
        desired = self.target_segment_distance()
        if self.target_approach_strategy == "legacy_ray":
            return [desired]
        minimum = self.target_minimum_viewpoint_distance()
        if desired <= minimum + 1e-3:
            return [minimum]
        distances = [desired]
        for ratio in (0.65, 0.35):
            candidate = minimum + (desired - minimum) * ratio
            if all(abs(candidate - existing) > 0.08 for existing in distances):
                distances.append(candidate)
        if all(abs(minimum - existing) > 0.08 for existing in distances):
            distances.append(minimum)
        return distances
