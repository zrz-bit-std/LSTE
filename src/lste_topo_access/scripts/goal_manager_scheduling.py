#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Periodic goal arbitration and state-to-mode scheduling for GoalManager."""

import math
import copy
from typing import Optional

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped

from goal_manager_modes import (
    CATCH_CTX_MODE,
    CATCH_TARGET_MODE,
    EXPLORE_PASS_MODE,
    EXPLORE_SUS_C_MODE,
    STATE_LOCKED,
    STATE_SUSPICIOUS,
)
from goal_manager_target_utils import wrap_angle

class GoalManagerSchedulingMixin:
    def on_timer(self, _event):
        now = rospy.Time.now().to_sec()
        if self.navigation_hold_active:
            loss_certified = getattr(
                self, "target_observation_loss_certified", None
            )
            if callable(loss_certified) and loss_certified():
                # The semantic release boundary is an explicit detector
                # episode, not the fallback hold deadline. Let the normal
                # target terminal path emit the loss/replan transaction on
                # this same timer tick.
                self.set_navigation_hold(False, "target_loss_certified")
            # A target terminal is a perception barrier, not a fixed dwell.
            # The terminal callback records the current detector epoch and
            # requires two *new* target frames.  Previously this hold was
            # released only at its timeout, while ``compute_goal`` returned
            # early for every held tick.  Fast detectors therefore collected
            # many valid post-arrival images but still parked for the complete
            # fallback window.  Release as soon as the evidence barrier is
            # satisfied; the normal target route validation below still owns
            # whether it is safe to move again.
            post_terminal_frames_ready = (
                self.target_terminal_reobserve_pending
                and self.target_observation_epoch
                >= self.target_terminal_reobserve_min_epoch
            )
            if post_terminal_frames_ready:
                # A close confirmation belongs to the just-arrived target
                # transaction. Do not release its perception barrier merely
                # because the lower two-frame re-observation gate is also
                # satisfied: a failed *next* viewpoint would clear this track
                # and discard its already valid close votes before
                # maybe_publish_task_done() can observe them. Once close
                # evidence becomes stale, maybe_publish_task_done() resets
                # target_close_since and this existing branch resumes the
                # ordinary validated-route or frontier fallback path.
                if self.target_close_since is None:
                    self.set_navigation_hold(False, "post_terminal_frames_ready")
                    self.publish_goal_arbitration(
                        "target_terminal_reobserve_ready",
                        observed_epoch=int(self.target_observation_epoch),
                        required_epoch=int(self.target_terminal_reobserve_min_epoch),
                        remaining_seconds=round(
                            max(0.0, self.target_terminal_reobserve_until - now),
                            3,
                        ),
                    )
            # ``effective_mode`` may legitimately change while the vehicle is
            # approaching a visual target: a context detection can temporarily
            # select CATCH_CTX_MODE before the next target frame arrives.  A
            # terminal re-observation is an owned transaction, so releasing
            # its hold on that unrelated state transition makes this timer
            # clear the hold and ``goal_from_target_follow`` restore it every
            # 0.2 s.  The supervisor maps each restoration to a zero command.
            # Only the evidence barrier above or this bounded timeout may end
            # the transaction; clear_target_memory/new-task paths explicitly
            # cancel it when ownership is genuinely lost.
            elif now >= self.target_observation_hold_until:
                self.set_navigation_hold(False, "observation_window_complete")
        if self.global_goal_source == "fixed":
            # A fixed target must not wait for state, detections, frontier
            # output, or /rbt_pose.  Publishing it at a bounded rate lets late
            # controller subscribers recover while never changing its value.
            if self.last_goal is None or now >= self.next_update_time:
                self.goal_source = "fixed_config"
                self.effective_mode = "fixed_goal_mode"
                self.publish_goal(
                    self.make_goal_pose(self.fixed_goal, self.fixed_goal[2]),
                    force_republish=True,
                )
                self.next_update_time = now + self.fixed_goal_publish_period
            return
        self.maybe_publish_task_done(now)
        if self.task_done_published:
            # Keep the final goal stable after visual completion. The mux has
            # already been told to stop, so recomputing visual rays here could
            # only create confusing post-completion motion.
            return
        if self.latest_pose is None:
            return

        # With online global coverage enabled, wait for the first map-connected
        # frontier rather than moving an arbitrary 10 m from the spawn pose.
        # That prevents the controller from entering a corridor before SLAM has
        # enough scans to judge whether it is useful or safe to explore.
        if self.last_goal is None and self.start_pose is not None:
            if self.global_frontier_enabled:
                goal = self.compute_goal()
                if goal is not None:
                    self.publish_goal(goal)
                    self.next_update_time = now + self.pass_period
                return
            if self.startup_forward_enabled:
                goal = self.build_goal_from_pose(self.start_pose, self.forward_dist)
                if goal:
                    self.goal_source = "startup_forward"
                    self.publish_goal(goal)
                    self.next_update_time = now + self.pass_period
                return
            # Without autonomous frontier dispatch, an arbitrary forward ray
            # would steal control from an impending visual target. Continue to
            # normal arbitration below; it remains idle until another source
            # supplies an explicit goal.

        period = self.state_period()
        if now < self.next_update_time and not self.force_update_due_state():
            return

        goal = self.compute_goal()
        if goal is None:
            return  # 保持原 goal，不发布

        self.publish_goal(goal)
        self.next_update_time = now + period

    def force_update_due_state(self) -> bool:
        if self.latest_state_msg is None:
            return False
        if self.last_goal is None:
            return True
        # 如果 state/subtype 发生变化则强制更新（在 on_state 已经 next_update_time=0）
        return False

    def init_headings(self, pose: Pose2D):
        base = wrap_angle(pose.theta)
        self.headings = [
            base,
            wrap_angle(base + math.pi * 0.5),
            wrap_angle(base + math.pi),
            wrap_angle(base + math.pi * 1.5),
        ]
        self.last_dir_idx = 0

    # -------------------- Goal computation --------------------
    def state_period(self) -> float:
        # Vision-following goals must track at the same cadence as confirmed
        # detections.  The older 2 s state cadence made a moving robot chase a
        # stale image ray.
        if self.effective_mode in (CATCH_TARGET_MODE, CATCH_CTX_MODE):
            return self.follow_goal_publish_period
        if self.global_frontier_enabled:
            return self.global_frontier_period
        if self.current_state == STATE_LOCKED:
            return self.locked_period
        if self.current_state == STATE_SUSPICIOUS:
            if self.current_subtype == "Sus-C":
                return self.sus_c_period
            if self.current_subtype == "Sus-A":
                return self.sus_a_period
            if self.current_subtype == "Sus-B":
                return self.sus_b_period
        return self.pass_period

    def _target_terminal_observation_bridge_pending(self) -> bool:
        """Keep successor planning behind the bridge's release ACK."""
        ready = getattr(self, "target_terminal_observation_bridge_ready", None)
        if callable(ready):
            return not bool(ready())
        pending = int(
            getattr(self, "target_terminal_observation_pending_transaction", 0)
            or 0
        )
        acknowledged = int(
            getattr(self, "target_terminal_observation_ack_transaction", 0)
            or 0
        )
        return pending > 0 and acknowledged < pending

    def compute_goal(self) -> Optional[PoseStamped]:
        # A global SLAM frontier is a map-connected exploration waypoint. It
        # dominates the old local GP/access recovery until a visual target ray
        # is available; visual target following remains higher priority.
        now = rospy.Time.now().to_sec()
        if self._target_terminal_observation_bridge_pending():
            self.goal_source = "target_terminal_observation_waiting_bridge"
            rospy.loginfo_throttle(
                3.0,
                "GoalManager: waiting for bridge release ACK before successor "
                "target transaction",
            )
            return None
        # A visual terminal is not a frontier terminal.  During this bounded
        # re-observation hold, the next detector frames must decide whether to
        # continue, confirm close range, or release the target. Once that
        # bounded hold ends, ``goal_from_target_follow`` owns the pending
        # transition; returning here for the pending flag would deadlock that
        # state machine before it can consume the fresh frames.
        if self.navigation_hold_active:
            self.goal_source = "target_reobserving"
            return None
        if getattr(self, "target_done_policy", "visual_target") == "target_room_entry":
            # The doorway-entry policy measures whether exploration crosses the
            # target-room doorway. Keep detector evidence from taking over
            # control; otherwise a false target box can hold the robot at an
            # unrelated viewpoint and prevent the doorway from being visited.
            return self._compute_room_entry_goal(now)
        # A fresh target ray remains the only reason to suppress global map
        # coverage. Context-only states previously let a chair/monitor pair
        # keep the robot in one corridor indefinitely, even though the target
        # was not visible. Global coverage is therefore the fallback for every
        # non-target state; as soon as target detections resume, the existing
        # target-follow path takes priority again.
        expire_candidate = getattr(
            self, "expire_unconfirmed_target_candidate", None
        )
        if callable(expire_candidate):
            expire_candidate(now)
        loss_certified = getattr(
            self, "target_observation_loss_certified", None
        )
        if callable(loss_certified) and loss_certified():
            # A reinspection may be executed by the global frontier node after
            # the visual target segment has been cleared. The gate still owns
            # the same track, so consume its explicit loss event here instead
            # of waiting for a target-specific terminal that cannot arrive.
            released = self._release_confirmed_target_after_loss(now)
            if released is not None:
                return released
            self.goal_source = "target_loss_certified_waiting_frontier"
            return None
        target_recent = self.target_tracking_active(now)
        candidate_information_active = bool(
            getattr(self, "target_candidate_information_active", lambda _now: False)(now)
        )
        # A committed visual segment is a mission transaction, not a live
        # detector callback. Its ownership must survive an ordinary detector
        # freshness timeout until TEB reports its terminal or Navfn reports a
        # route failure. Otherwise a single detector gap can replace an
        # already validated target path with an unvalidated reacquisition ray.
        target_segment_active = self.target_segment_ownership_active()
        # A target-segment terminal starts a separate observation transaction.
        # Its ownership must likewise survive the ordinary detector freshness
        # timeout: WeDetect can legitimately produce the first post-terminal
        # frame just after that timeout, and falling through to a frontier here
        # would clear the track before ``goal_from_target_follow`` can validate
        # the new ray or release it through the semantic-frontier path.
        target_terminal_pending = bool(
            self.target_terminal_reobserve_pending
            and self.target_last_goal is not None
        )
        # A continuity-deferred lock owns a *frontier* action until its
        # terminal. It is neither a detector freshness timeout nor a blind
        # retry: dropping it here would lose a valid target merely because the
        # healthy route to the next observation boundary took longer than one
        # detector timeout.
        target_takeover_deferred = bool(
            self.target_route_continuity_deferred
            and self.last_goal is not None
            and self.last_goal_source == "global_slam_frontier"
        )
        if (
            target_recent
            or candidate_information_active
            or target_segment_active
            or target_terminal_pending
            or target_takeover_deferred
        ):
            # A validated parallax side-step owns the controller until its
            # matching terminal.  Viewpoint diversity is expected during the
            # side-step and must not be mistaken for permission to release it.
            if (
                getattr(self, "target_parallax_goal", None) is not None
                and not getattr(self, "target_parallax_completed", False)
            ):
                complete_parallax = getattr(
                    self, "_complete_reached_target_parallax", None
                )
                if not callable(complete_parallax) or not complete_parallax(now):
                    self.goal_source = "target_parallax"
                    return self.target_parallax_goal
                self.goal_source = "target_terminal_observation_waiting_bridge"
                return None
            # Never let legacy access-topology recovery preempt a current (or
            # just-lost) visual target ray.  goal_from_target_follow preserves
            # the last short visual-servo goal through a brief detector gap.
            target_goal = self.goal_from_target_follow(now)
            if target_goal is not None:
                return target_goal
            # While the terminal transaction remains pending, ``None`` means
            # that its bounded observation decision is still in progress, not
            # that normal frontier exploration may seize the controller.
            if self.target_terminal_reobserve_pending:
                self.goal_source = "target_reobserving"
                return None
        if not target_recent:
            # With online SLAM enabled, context boxes are inspection evidence,
            # not a second global planner. Keep the map-connected waypoint as
            # the sole exploration commitment unless an actual target ray is
            # confirmed or the caller explicitly opts into context preemption.
            if self.global_frontier_enabled and not self.global_frontier_preempt_context:
                reacquire_goal = self.target_reacquisition_goal(now)
                if reacquire_goal is not None:
                    return reacquire_goal
                global_frontier = self.fresh_global_frontier_goal(now)
                if global_frontier is not None:
                    self.goal_source = "global_slam_frontier"
                    return global_frontier
            # Context detections are deliberately lower priority than an
            # active map-connected route.  The state node can briefly enter
            # Sus-B when a monitor/desk pair is visible; replacing the
            # frontier waypoint at that instant made the controller turn back
            # into the already explored corridor.  Release the commitment only
            # after the robot is close to the current waypoint.  A confirmed
            # target above still preempts this guard immediately.
            # ``last_goal`` is the newest mission intent and may already be a
            # context observation while TEB is still executing a committed
            # frontier action. Use the frontier publication history as the
            # execution boundary so context evidence cannot overwrite a live
            # map route before its terminal result.
            active_frontier_goal = None
            if self.last_goal_source == "global_slam_frontier":
                active_frontier_goal = self.last_goal
            elif (
                self.teb_terminal_goal is None
                and self.teb_frontier_goal_history
            ):
                active_frontier_goal = self.teb_frontier_goal_history[-1]
            if (
                self.global_frontier_enabled
                and not self.global_frontier_preempt_context
                and active_frontier_goal is not None
                and self.effective_mode == CATCH_CTX_MODE
                and self.latest_pose is not None
            ):
                distance_to_frontier = self.goal_robot_distance(active_frontier_goal)
                if distance_to_frontier is None:
                    distance_to_frontier = float("inf")
                release_radius = min(
                    self.global_frontier_update_radius,
                    self.global_frontier_jump_release_radius,
                )
                if distance_to_frontier > release_radius:
                    self.goal_source = "global_slam_frontier"
                    rospy.loginfo_throttle(
                        3.0,
                        "GoalManager: hold frontier during context state "
                        "distance=%.2fm release_radius=%.2fm",
                        distance_to_frontier,
                        release_radius,
                    )
                    return active_frontier_goal
            reacquire_goal = self.target_reacquisition_goal(now)
            if reacquire_goal is not None:
                return reacquire_goal
            # A task-specific pair, for example monitor + monitor for the
            # yellow-cup task, is stronger evidence than an arbitrary map
            # frontier. Do not react to a lone chair, door, or fire hydrant:
            # pick_ctx_pair requires both configured context detections.
            if (
                self.effective_mode == CATCH_CTX_MODE
                and now >= self.ctx_cooldown_until
            ):
                left_ctx, right_ctx = self.pick_ctx_pair()
                if left_ctx is not None and right_ctx is not None:
                    return self.goal_from_ctx_follow(now)
            global_frontier = self.fresh_global_frontier_goal(now)
            if global_frontier is not None:
                self.goal_source = "global_slam_frontier"
                return global_frontier
            if self.global_frontier_enabled:
                # Do not fall back to an arbitrary local ray while online SLAM
                # is starting or unavailable. A new controller then waits for
                # a connected map route instead of moving blindly.
                self.goal_source = "waiting_global_slam_frontier"
                return None

        # Access-topology recovery is valid only while searching.  A recovery
        # waypoint must never replace a visually confirmed target-follow goal.
        if (
            not self.global_frontier_enabled
            and
            self.effective_mode in (EXPLORE_PASS_MODE, EXPLORE_SUS_C_MODE)
            and self.access_mode in (1, 2)
            and self.access_backtrack_goal is not None
        ):
            self.goal_source = "access_backtrack"
            return self.access_backtrack_goal

        # 基于 effective_mode 分发（后续犹豫/降级都在 effective_mode 上动手）
        if self.effective_mode == EXPLORE_PASS_MODE:
            return self.goal_from_frontiers_prior()
        if self.effective_mode == EXPLORE_SUS_C_MODE:
            return self.goal_from_frontiers_prior()
        if self.effective_mode == CATCH_TARGET_MODE:
            return self.goal_from_target_follow(now)
        if self.effective_mode == CATCH_CTX_MODE:
            return self.goal_from_ctx_follow(now)
        # 未知模式：保持现状
        rospy.logwarn_throttle(5.0, "GoalManager: unknown mode=%s", str(self.effective_mode))
        return None

    def _compute_room_entry_goal(self, now: float) -> Optional[PoseStamped]:
        """Promote a reachable target doorway to a task-owned route.

        The doorway policy is a task-level handoff, not just a completion
        mask. Until Navfn can prove the configured doorway inset is connected
        to the current map, frontier exploration remains the owner. Once that
        proof exists, retain the returned endpoint and let the normal target
        route publication carry the current frontier lifecycle identity.
        """
        committed = getattr(self, "target_room_entry_goal", None)
        if committed is not None:
            self.goal_source = "target_room_entry"
            return committed

        if (
            bool(getattr(self, "global_frontier_enabled", False))
            and int(getattr(self, "frontier_terminal_count", 0) or 0) <= 0
        ):
            # Do not preempt the bootstrap WorkItem with a long semantic goal.
            # The first frontier terminal is the proof that the Place has one
            # valid observation and that the graph can safely own the next
            # task-level handoff.
            global_frontier = self.fresh_global_frontier_goal(now)
            if global_frontier is not None:
                self.goal_source = "global_slam_frontier"
                return global_frontier
            self.goal_source = "waiting_global_slam_frontier"
            return None

        if getattr(self, "target_room_entry_retry_blocked", False):
            # A failed task route may have been accepted from an incomplete
            # map. Wait for a real frontier terminal before asking Navfn to
            # prove the doorway again; repeated validation of the same map is
            # not new evidence.
            global_frontier = self.fresh_global_frontier_goal(now)
            if global_frontier is not None:
                self.goal_source = "global_slam_frontier"
                return global_frontier
            self.goal_source = "waiting_global_slam_frontier"
            return None

        try:
            min_x = float(self.target_room_entry_min_x)
            min_y = float(self.target_room_entry_min_y)
            max_x = float(self.target_room_entry_max_x)
            max_y = float(self.target_room_entry_max_y)
            margin = float(self.target_room_entry_margin)
        except (AttributeError, TypeError, ValueError):
            min_x = min_y = max_x = max_y = margin = float("nan")
        inner_min_x = min_x + margin
        inner_min_y = min_y + margin
        inner_max_x = max_x - margin
        inner_max_y = max_y - margin
        if not all(
            math.isfinite(value)
            for value in (
                min_x,
                min_y,
                max_x,
                max_y,
                margin,
                inner_min_x,
                inner_min_y,
                inner_max_x,
                inner_max_y,
            )
        ) or inner_min_x >= inner_max_x or inner_min_y >= inner_max_y:
            requested = None
        else:
            requested = PoseStamped()
            requested.header.stamp = rospy.Time.now()
            requested.header.frame_id = "map"
            requested.pose.position.x = 0.5 * (inner_min_x + inner_max_x)
            requested.pose.position.y = 0.5 * (inner_min_y + inner_max_y)
            requested.pose.orientation.w = 1.0

        if requested is not None:
            route_status = self.validate_target_route(requested, now)
            if route_status is True:
                endpoint = getattr(
                    self, "target_route_validation_last_endpoint", None
                )
                if endpoint is None:
                    endpoint = requested
                self.target_room_entry_goal = copy.deepcopy(endpoint)
                self.goal_source = "target_room_entry"
                self.publish_goal_arbitration(
                    "target_room_entry_route_validated",
                    requested_goal=[
                        round(float(requested.pose.position.x), 3),
                        round(float(requested.pose.position.y), 3),
                    ],
                    navfn_returned_goal=[
                        round(float(endpoint.pose.position.x), 3),
                        round(float(endpoint.pose.position.y), 3),
                    ],
                    goal_frame=(endpoint.header.frame_id or "map"),
                    completion_evidence="target_room_entry",
                )
                rospy.loginfo(
                    "GoalManager: target doorway became Navfn-reachable; "
                    "adopting task route goal=(%.2f,%.2f)",
                    float(endpoint.pose.position.x),
                    float(endpoint.pose.position.y),
                )
                return self.target_room_entry_goal

        global_frontier = self.fresh_global_frontier_goal(now)
        if global_frontier is not None:
            self.goal_source = "global_slam_frontier"
            return global_frontier
        if self.global_frontier_enabled:
            self.goal_source = "waiting_global_slam_frontier"
            return None
        return self.goal_from_frontiers_prior()
