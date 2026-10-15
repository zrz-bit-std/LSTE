#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Goal arbitration diagnostics, route validation, and target recovery helpers."""

import copy
import json
import math
from typing import Optional

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler, quaternion_matrix

from goal_manager_route_validation import validate_target_route as validate_target_route_request
from goal_manager_modes import STATE_LOCKED
from goal_manager_target_utils import labels_compatible, wrap_angle

class GoalManagerGoalArbitrationMixin:
    def publish_goal_arbitration(self, event: str, **fields):
        """Publish mission/execution arbitration decisions for run logs.

        The final-goal topic intentionally carries only executable poses.  A
        separate event stream records why a perception intent was accepted,
        deferred, or rejected, so a stopped vehicle can be diagnosed without
        inferring policy decisions from velocity samples alone.
        """
        try:
            action_generation = int(
                fields.get(
                    "action_generation",
                    0,
                )
                or 0
            )
        except (TypeError, ValueError):
            action_generation = 0
        route_id = int(getattr(self, "global_frontier_route_id", 0) or 0)
        payload = {
            "event": str(event),
            "task_id": str(self.current_task_id),
            "mission_id": str(getattr(self, "current_mission_id", "")),
            "task_version": str(getattr(self, "current_task_version", "")),
            "source": str(self.goal_source),
            "mode": str(self.effective_mode),
            "controller_mode": str(self.controller_mode),
            "target_state": str(self.target_execution_state),
            "target_done_policy": str(
                getattr(self, "target_done_policy", "visual_target")
            ),
            "target_epoch": int(self.target_observation_epoch),
            "target_blocked": bool(self.target_blocked),
            "target_failure_count": int(self.target_failure_count),
            "target_observation_gate": (
                None
                if getattr(self, "target_observation_gate", None) is None
                else self.target_observation_gate.snapshot().__dict__
            ),
            "target_viewpoint_ledger": (
                None
                if getattr(self, "target_approach_transaction", None) is None
                else self.target_approach_transaction.viewpoint_ledger.snapshot().to_dict()
            ),
            "target_reinspection_pending": bool(
                getattr(self, "target_reinspection_pending", False)
            ),
            # Keep the complete route lineage at the mission decision boundary
            # so an arbitration event can be correlated without joining it to
            # a later pose-only publication.
            "route_id": route_id,
            "action_generation": action_generation,
            # GoalManager does not allocate transport generations. Keep the
            # latest Bridge observation separately so a new semantic mission
            # cannot inherit a stale generation during persistent handoff.
            "bridge_action_generation": int(
                getattr(self, "bridge_action_generation", 0) or 0
            ),
            "dedupe_key": (
                [route_id, action_generation]
                if action_generation > 0
                else None
            ),
            "transaction_id": int(getattr(self, "goal_command_id", 0) or 0),
            "lifecycle_transaction_id": int(
                self._canonical_lifecycle_transaction_id()
            ),
            "graph_transaction_id": int(
                getattr(self, "global_frontier_graph_transaction_id", 0) or 0
            ),
            "map_epoch": getattr(self, "global_frontier_map_epoch", None),
            "origin_epoch": getattr(
                self, "global_frontier_origin_epoch", None
            ),
            "projection_epoch": getattr(
                self, "global_frontier_projection_epoch", None
            ),
        }
        payload.update(fields)
        try:
            self.pub_goal_arbitration.publish(
                String(data=json.dumps(payload, sort_keys=True))
            )
        except (TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "GoalManager: arbitration event serialization failed")

    def _pose_in_frame(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """Transform a planar pose without introducing a tf message helper."""
        if pose is None:
            return None
        source_frame = (pose.header.frame_id or "odom").strip().lstrip("/") or "odom"
        target_frame = (target_frame or "map").strip().lstrip("/") or "map"
        if source_frame == target_frame:
            return pose
        transform = self.lookup_transform(target_frame, source_frame, rospy.Time(0))
        if transform is None:
            return None
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        point = matrix[:3, :3].dot(
            np.array([
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            ], dtype=float)
        )
        translation = transform.transform.translation
        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = target_frame
        out.pose.position.x = float(point[0] + translation.x)
        out.pose.position.y = float(point[1] + translation.y)
        out.pose.position.z = float(point[2] + translation.z)
        source_yaw = self.yaw_from_pose(pose) or 0.0
        transform_yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        qx, qy, qz, qw = quaternion_from_euler(
            0.0, 0.0, wrap_angle(source_yaw + transform_yaw)
        )
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        return out

    def pose_distance(self, first: PoseStamped, second: PoseStamped) -> Optional[float]:
        """Measure two mission poses in one frame for lifecycle matching."""
        if first is None or second is None:
            return None
        first_frame = self._frame_name(first.header.frame_id)
        second_frame = self._frame_name(second.header.frame_id)
        comparable = first
        if first_frame != second_frame:
            comparable = self._pose_in_frame(first, second_frame)
            if comparable is None:
                return None
        return math.hypot(
            float(comparable.pose.position.x) - float(second.pose.position.x),
            float(comparable.pose.position.y) - float(second.pose.position.y),
        )

    def validate_target_route(
        self,
        goal: PoseStamped,
        now: float,
        force: bool = False,
    ) -> Optional[bool]:
        """Delegate Navfn validation to the focused route-validation module."""
        return validate_target_route_request(self, goal, now, force=force)
    def target_reacquisition_goal(
        self, now: float, force: bool = False
    ) -> Optional[PoseStamped]:
        """Make one bounded forward continuation after a target is lost.

        The camera saw the target along ``target_last_heading``.  Continuing
        along that bearing is the only evidence-backed way to bring a small,
        distant object back into view.  The previous reverse-heading sweep
        pointed the vehicle away from the detected object and made the normal
        pipeline abandon valid sightings near desks.
        """
        if (
            self.target_reacquire_goal is not None
            and self.target_reacquire_started is not None
        ):
            if now - self.target_reacquire_started < self.target_reacquire_duration:
                self.goal_source = "target_reacquisition_sweep"
                return self.target_reacquire_goal
            rospy.loginfo("GoalManager: target reacquisition sweep expired; resume global exploration")
            self.target_reacquire_goal = None
            self.target_reacquire_started = None
            return None
        if (
            self.target_blocked
            # An active target segment already owns the same visual evidence.
            # A loss-driven sweep must never preempt it merely because one
            # detector interval exceeded the freshness threshold.
            or self.target_last_goal is not None
            or self.target_reacquire_duration <= 0.0
            or self.target_reacquire_attempts >= self.target_reacquire_max_attempts
            or self.target_last_seen is None
            or self.target_last_heading is None
            or self.latest_pose is None
        ):
            return None
        evidence_timeout = self.target_evidence_timeout()
        if not force and now - self.target_last_seen < evidence_timeout:
            return None
        if self.target_reacquire_started is None:
            heading = self.target_last_heading
            x = self.latest_pose.x + self.target_reacquire_distance * math.cos(heading)
            y = self.latest_pose.y + self.target_reacquire_distance * math.sin(heading)
            requested_goal_odom = self.make_goal_pose((x, y, 0.0), heading)
            requested_goal = self._pose_in_frame(requested_goal_odom, "map")
            if requested_goal is None:
                self.goal_source = "target_reacquisition_waiting_route"
                self.publish_goal_arbitration(
                    "target_reacquisition_deferred",
                    reason="tf_unavailable",
                )
                return None
            # Reacquisition is still a target mission, so it must meet the
            # same Navfn endpoint contract as a normal visual segment. This
            # also makes a dynamic map change observable before it is sent to
            # the persistent planner rather than as a later route failure.
            route_status = self.validate_target_route(
                requested_goal, now, force=True
            )
            if (
                route_status is not True
                or self.target_route_validation_last_endpoint is None
            ):
                self.goal_source = "target_reacquisition_waiting_route"
                self.publish_goal_arbitration(
                    "target_reacquisition_deferred",
                    reason=(
                        "navfn_empty_plan"
                        if route_status is False
                        else "route_validation_unavailable"
                    ),
                    requested_goal=[
                        round(float(requested_goal.pose.position.x), 3),
                        round(float(requested_goal.pose.position.y), 3),
                    ],
                    goal_frame=(requested_goal.header.frame_id or "map"),
                )
                return None
            self.target_reacquire_goal = copy.deepcopy(
                self.target_route_validation_last_endpoint
            )
            self.target_reacquire_started = now
            self.target_reacquire_attempts += 1
            endpoint_adjustment = self.pose_distance(
                requested_goal, self.target_reacquire_goal
            )
            self.publish_goal_arbitration(
                "target_reacquisition_route_validated",
                evidence_level=(
                    "confirmed_track"
                    if self.target_follow_confirmed else "direct_candidate"
                ),
                requested_goal=[
                    round(float(requested_goal.pose.position.x), 3),
                    round(float(requested_goal.pose.position.y), 3),
                ],
                navfn_returned_goal=[
                    round(float(self.target_reacquire_goal.pose.position.x), 3),
                    round(float(self.target_reacquire_goal.pose.position.y), 3),
                ],
                navfn_endpoint_adjustment=round(
                    float(endpoint_adjustment or 0.0), 4
                ),
                navfn_endpoint_contract=True,
            )
            rospy.loginfo(
                "GoalManager: target %s; Navfn-validated continuation %d/%d",
                "lost" if self.target_follow_confirmed else "candidate needs reinspection",
                self.target_reacquire_attempts,
                self.target_reacquire_max_attempts,
            )
        if self.target_reacquire_goal is not None:
            self.goal_source = "target_reacquisition_sweep"
            return self.target_reacquire_goal
        return None

    def fresh_global_frontier_goal(self, now: float) -> Optional[PoseStamped]:
        if (
            self.frontier_replan_pending_id > 0
            and self.frontier_replan_ready_id != self.frontier_replan_pending_id
        ):
            return None
        if not self.global_frontier_enabled or self.latest_global_frontier_goal is None:
            return None
        # TEB reports a terminal for the current frontier endpoint before the
        # frontier executive has published its successor.  The cached goal is
        # still present at this point, but reusing it would allocate a new
        # GoalManager transaction for the same route and race the predecessor
        # lease-release ACK.  Only a strictly newer route may clear this
        # boundary.
        if (
            self.teb_terminal_goal is not None
            and self.last_goal_source == "global_slam_frontier"
            and int(getattr(self, "global_frontier_route_id", 0) or 0) > 0
            and int(getattr(self, "last_published_frontier_route_id", 0) or 0) > 0
            and int(getattr(self, "global_frontier_route_id", 0) or 0)
            <= int(getattr(self, "last_published_frontier_route_id", 0) or 0)
        ):
            rospy.loginfo_throttle(
                2.0,
                "GoalManager: holds terminal frontier route=%d until a "
                "strictly newer successor route arrives",
                int(getattr(self, "global_frontier_route_id", 0) or 0),
            )
            return None
        stamp = self.latest_global_frontier_goal.header.stamp
        if self.global_frontier_max_age > 0.0 and stamp:
            age = now - stamp.to_sec()
            if age > self.global_frontier_max_age:
                return None
        return self.latest_global_frontier_goal

    def _complete_target_task(
        self, now: float, det, score_threshold: float, completion_evidence: str
    ):
        """Publish one identity-bearing completion transaction."""
        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        if approach_transaction is not None:
            approach_transaction.mark_completed(now, completion_evidence)
        event = {
            "close_box": "target_close_confirmed",
            "confirmed_track": "target_track_completion_confirmed",
            "candidate_track": "target_candidate_completion_confirmed",
        }.get(completion_evidence, "target_terminal_observation_confirmed")
        fields = {
            "target_track_id": self.target_track_id,
            "approach_track_id": self.target_approach_track_id,
            "score": round(float(det.score), 4),
            "score_threshold": round(float(score_threshold), 4),
            "box": [round(float(det.w), 4), round(float(det.h), 4)],
            "hits": int(self.target_close_hits),
            "hold_seconds": round(
                max(0.0, float(now - self.target_close_since))
                if self.target_close_since is not None
                else 0.0,
                3,
            ),
        }
        self.publish_goal_arbitration(event, **fields)
        self.publish_goal_arbitration(
            "target_task_completed",
            completion_evidence=completion_evidence,
            **fields,
        )
        rospy.loginfo(
            "GoalManager: target completion evidence=%s hits=%d hold=%.1fs, "
            "publish task_done",
            completion_evidence,
            self.target_close_hits,
            max(0.0, now - self.target_close_since)
            if self.target_close_since is not None
            else 0.0,
        )
        self.pub_task_done.publish(Bool(data=True))
        self.task_done_published = True

    def reset_target_room_entry_state(self):
        """Reset the one-way room-entry acceptance state for a new task."""
        self.target_room_entry_seen_outside = False
        self.target_room_entry_inside_since = None
        self.target_room_entry_last_pose = None
        self.target_room_entry_event_reported = False

    def target_room_entry_completion_ready(self, now: float) -> bool:
        """Return whether the robot has crossed the configured target doorway.

        This is deliberately a benchmark acceptance policy, not a source of
        navigation goals. The robot must first be observed outside the
        physical doorway bounds; a later pose inside those bounds is enough
        to prove doorway entry. The default visual-target policy never
        evaluates this predicate.
        """
        if getattr(self, "target_done_policy", "visual_target") != "target_room_entry":
            return False
        if not str(getattr(self, "current_task_id", "") or "").strip():
            return False
        pose = getattr(self, "latest_pose", None)
        if pose is None:
            return False
        try:
            x = float(pose.x)
            y = float(pose.y)
            min_x = float(self.target_room_entry_min_x)
            min_y = float(self.target_room_entry_min_y)
            max_x = float(self.target_room_entry_max_x)
            max_y = float(self.target_room_entry_max_y)
            margin = float(self.target_room_entry_margin)
        except (AttributeError, TypeError, ValueError):
            return False
        values = (x, y, min_x, min_y, max_x, max_y, margin)
        if not all(math.isfinite(value) for value in values):
            return False
        if min_x >= max_x or min_y >= max_y:
            return False
        self.target_room_entry_last_pose = [
            round(x, 4), round(y, 4),
            round(float(getattr(pose, "theta", 0.0)), 4),
        ]
        inside_gate = (
            min_x <= x <= max_x
            and min_y <= y <= max_y
        )
        if not inside_gate:
            self.target_room_entry_seen_outside = True
            self.target_room_entry_inside_since = None
            self.target_room_entry_event_reported = False
            return False
        if (
            getattr(self, "target_room_entry_require_crossing", True)
            and not self.target_room_entry_seen_outside
        ):
            return False
        if self.target_room_entry_inside_since is None:
            self.target_room_entry_inside_since = float(now)
            if not self.target_room_entry_event_reported:
                self.publish_goal_arbitration(
                    "target_room_entry_started",
                    completion_evidence="target_room_entry",
                    pose=list(self.target_room_entry_last_pose),
                    door_id="target_room_corridor",
                    source_region="main_corridor",
                    target_region="target_room",
                    door_bounds=[min_x, min_y, max_x, max_y],
                    inset_margin=margin,
                )
                self.target_room_entry_event_reported = True
        return (
            float(now) - float(self.target_room_entry_inside_since)
            >= float(getattr(self, "target_room_entry_min_hold_time", 0.0))
        )

    def _complete_target_room_entry(self, now: float):
        """Publish task completion at the physical target-doorway boundary."""
        pose = list(getattr(self, "target_room_entry_last_pose", None) or [])
        fields = {
            "completion_evidence": "target_room_entry",
            "target_done_policy": "target_room_entry",
            "pose": pose,
            "door_id": "target_room_corridor",
            "source_region": "main_corridor",
            "target_region": "target_room",
            "door_bounds": [
                float(self.target_room_entry_min_x),
                float(self.target_room_entry_min_y),
                float(self.target_room_entry_max_x),
                float(self.target_room_entry_max_y),
            ],
            "inset_margin": round(float(self.target_room_entry_margin), 3),
            "hold_seconds": round(
                max(
                    0.0,
                    float(now)
                    - float(self.target_room_entry_inside_since or now),
                ),
                3,
            ),
        }
        self.target_execution_state = "TARGET_ROOM_ENTRY_COMPLETED"
        self.publish_goal_arbitration("target_room_entry_confirmed", **fields)
        self.publish_goal_arbitration("target_task_completed", **fields)
        rospy.loginfo(
            "GoalManager: target doorway entry completed at pose=(%.2f, %.2f); "
            "publish task_done",
            float(pose[0]) if pose else 0.0,
            float(pose[1]) if len(pose) > 1 else 0.0,
        )
        self.pub_task_done.publish(Bool(data=True))
        self.task_done_published = True

    def target_context_completion_evidence(self):
        """Return one target plus two distinct configured context detections.

        ``target_context`` is a short perception contract. Detector scores,
        box dimensions, hold time, and navigation state are diagnostics only;
        the detector's published presence is the gate. ``pick_ctx_pair``
        already enforces that identical left/right terms cannot reuse one box.
        """
        task = getattr(self, "latest_task", None)
        detections = getattr(self, "latest_dets", None)
        if task is None or detections is None:
            return None
        task_id = str(getattr(task, "task_id", "") or "").strip()
        current_task_id = str(getattr(self, "current_task_id", "") or "").strip()
        target_name = str(getattr(task, "target_name", "") or "").strip()
        if not task_id or task_id != current_task_id or not target_name:
            return None
        target_hits = [
            detection
            for detection in list(getattr(detections, "target_dets", []) or [])
            if labels_compatible(target_name, getattr(detection, "label", ""))
        ]
        if not target_hits:
            return None
        left, right = self.pick_ctx_pair()
        if left is None or right is None:
            return None
        target = max(target_hits, key=lambda detection: float(detection.score))
        return {
            "target": target,
            "ctx_left": left,
            "ctx_right": right,
            "target_name": target_name,
            "ctx_left_term": str(getattr(task, "ctx_left", "") or ""),
            "ctx_right_term": str(getattr(task, "ctx_right", "") or ""),
        }

    def _complete_target_context(self, now: float, evidence):
        """Publish perception-only completion without creating target motion."""
        target = evidence["target"]
        left = evidence["ctx_left"]
        right = evidence["ctx_right"]
        fields = {
            "completion_evidence": "target_context",
            "target_done_policy": "target_context",
            "target_label": str(getattr(target, "label", "") or ""),
            "target_score": round(float(getattr(target, "score", 0.0)), 4),
            "context_labels": [
                str(getattr(left, "label", "") or ""),
                str(getattr(right, "label", "") or ""),
            ],
            "context_scores": [
                round(float(getattr(left, "score", 0.0)), 4),
                round(float(getattr(right, "score", 0.0)), 4),
            ],
            "target_name": evidence["target_name"],
            "ctx_left_term": evidence["ctx_left_term"],
            "ctx_right_term": evidence["ctx_right_term"],
            "score_gate": "ignored",
            "box_gate": "ignored",
            "navigation_gate": "ignored",
        }
        self.target_execution_state = "TARGET_CONTEXT_COMPLETED"
        self.publish_goal_arbitration("target_context_confirmed", **fields)
        self.publish_goal_arbitration("target_task_completed", **fields)
        rospy.loginfo(
            "GoalManager: target context completed target=%s context=(%s,%s); "
            "publish task_done",
            fields["target_label"],
            fields["context_labels"][0],
            fields["context_labels"][1],
        )
        self.pub_task_done.publish(Bool(data=True))
        self.task_done_published = True

    def maybe_publish_task_done(self, now: float):
        """Close a task from doorway or visual completion evidence.

        Normal completion comes from a close target in fresh detector
        messages, not merely from elapsed state time. An explicit benchmark
        policy may accept a doorway crossing before visual target evidence.
        Counting distinct detector messages prevents
        the 5 Hz Goal Manager timer from treating one stale box as several
        observations. Once complete, the velocity mux stops all commands
        atomically and the evidence event records which policy was used.
        """
        if self.task_done_published:
            return
        if getattr(self, "target_done_policy", "visual_target") == "target_room_entry":
            if self.target_room_entry_completion_ready(now):
                self._complete_target_room_entry(now)
            return
        if getattr(self, "target_done_policy", "visual_target") == "target_context":
            evidence = self.target_context_completion_evidence()
            if evidence is not None:
                self._complete_target_context(now, evidence)
            return
        if self.target_done_require_locked and self.current_state != STATE_LOCKED:
            self.reset_target_close_confirmation()
            return
        # Exhausting the current viewpoint ledger is a revalidation boundary,
        # not completion evidence.  A false or distant track must receive a
        # new local observation transaction; it must never publish task_done
        # merely because its bounded route portfolio ran out.
        portfolio_complete = getattr(
            self, "target_viewpoint_portfolio_complete", None
        )
        require_close_evidence = getattr(
            self, "target_done_require_close_evidence", True
        )
        if (
            require_close_evidence
            and callable(portfolio_complete)
            and portfolio_complete(now)
        ):
            det = self.target_detection_for_track(self.latest_dets)
            if det is None or not self.target_detection_is_close(det):
                revalidate = getattr(
                    self, "revalidate_target_viewpoint_portfolio", None
                )
                if callable(revalidate):
                    revalidate(now)
                return
        if not self.target_close_completion_eligible():
            candidate_track_allowed = (
                not getattr(self, "target_done_require_confirmed_track", True)
                and not getattr(
                    self, "target_done_require_approach_terminal", True
                )
                and bool(self.target_track_id)
            )
            if not candidate_track_allowed:
                # A lone or unconfirmed image must not complete the normal
                # task. Benchmark candidate completion is an explicit,
                # opt-in acceptance policy handled below.
                self.reset_target_close_confirmation()
                return
        det = self.target_detection_for_track(self.latest_dets)
        if self.latest_dets is None:
            self.reset_target_close_confirmation()
            return
        stamp = self.latest_dets.header.stamp
        stamp_sec = stamp.to_sec()
        stamp_key = (
            stamp.secs,
            stamp.nsecs,
            int(getattr(self.latest_dets.header, "seq", 0) or 0),
        )
        fresh_detector_message = self.target_close_last_stamp != stamp_key
        if (
            self.target_done_max_detection_age > 0.0
            and now - stamp_sec > self.target_done_max_detection_age
        ):
            # A slow detector can legitimately leave its last message stale
            # before the next independent frame arrives. It must not create a
            # new close vote, but an already started terminal observation may
            # retain the stationary viewpoint for its bounded grace period.
            if (
                self.target_close_last_seen is None
                or now - self.target_close_last_seen
                > self.target_close_confirmation_grace()
            ):
                self.reset_target_close_confirmation()
            return
        close_score_threshold = self.target_close_score_threshold()
        close_enough = self.target_detection_is_close(det)
        track_completion = (
            not require_close_evidence
            and not close_enough
            and det is not None
            and bool(self.target_track_id)
            and (
                not getattr(self, "target_done_require_confirmed_track", True)
                or bool(self.target_follow_confirmed)
            )
            and float(det.score) >= float(self.target_follow_min_score)
            and max(float(det.w), float(det.h))
            >= float(self.target_follow_min_box_size)
        )
        if track_completion:
            # Benchmark mode may finish at the first valid frame of the
            # current target track. Track identity and fresh-frame gates
            # remain mandatory; policy flags select the remaining evidence.
            if self.target_close_last_stamp != stamp_key:
                self.target_close_last_stamp = stamp_key
                self.target_close_hits += 1
            self.target_close_last_seen = now
            if self.target_close_since is None:
                self.target_close_since = now
                self.publish_goal_arbitration(
                    "target_track_completion_started",
                    target_track_id=self.target_track_id,
                    approach_track_id=self.target_approach_track_id,
                    score=round(float(det.score), 4),
                    score_threshold=round(float(self.target_follow_min_score), 4),
                    box=[round(float(det.w), 4), round(float(det.h), 4)],
                    hits=int(self.target_close_hits),
                    required_hits=int(self.target_done_min_fresh_hits),
                    required_hold_seconds=round(
                        float(self.target_done_min_hold_time), 3
                    ),
                )
                return
            if (
                self.target_close_hits >= self.target_done_min_fresh_hits
                and (now - self.target_close_since)
                >= self.target_done_min_hold_time
            ):
                self._complete_target_task(
                    now,
                    det,
                    float(self.target_follow_min_score),
                    (
                        "confirmed_track"
                        if self.target_follow_confirmed
                        else "candidate_track"
                    ),
                )
            return
        if close_enough and self.target_completed_segments >= 1:
            # This is a post-terminal fact, not a score latch. It allows the
            # same track to bridge a detector-size dip without treating the
            # next frame as proof that the target disappeared.
            self.target_terminal_close_candidate_seen = True
        if not close_enough:
            terminal_observation_eligible = getattr(
                self, "target_terminal_observation_eligible", None
            )
            if (
                fresh_detector_message
                and callable(terminal_observation_eligible)
                and terminal_observation_eligible(det)
            ):
                if self.target_close_last_stamp != stamp_key:
                    self.target_close_last_stamp = stamp_key
                    self.target_close_hits += 1
                self.target_close_last_seen = now
                if self.target_close_since is None:
                    self.target_close_since = now
                    self.publish_goal_arbitration(
                        "target_terminal_observation_started",
                        target_track_id=self.target_track_id,
                        approach_track_id=self.target_approach_track_id,
                        score=round(float(det.score), 4),
                        score_threshold=round(float(close_score_threshold), 4),
                        box=[round(float(det.w), 4), round(float(det.h), 4)],
                        hits=int(self.target_close_hits),
                        required_hits=int(self.target_done_min_fresh_hits),
                        required_hold_seconds=round(
                            float(self.target_done_min_hold_time), 3
                        ),
                    )
                    rospy.loginfo(
                        "GoalManager: terminal target observation started "
                        "score=%.3f box=(%.3f,%.3f) hits=%d/%d",
                        float(det.score), float(det.w), float(det.h),
                        self.target_close_hits, self.target_done_min_fresh_hits,
                    )
                    return
                if (
                    self.target_close_hits >= self.target_done_min_fresh_hits
                    and (now - self.target_close_since)
                    >= self.target_done_min_hold_time
                ):
                    self._complete_target_task(
                        now,
                        det,
                        close_score_threshold,
                        "terminal_track_continuity",
                    )
                return
            # A fresh negative frame is an explicit observation transaction
            # boundary.  Keeping the old close vote through every later
            # negative frame turned a detector dip into a 20 s navigation hold
            # and produced the long zero-velocity stalls seen in the office
            # run.  Stale messages remain harmless; the next fresh frame may
            # start a new close transaction.
            if fresh_detector_message:
                self.reset_target_close_confirmation()
                if self.navigation_hold_active:
                    self.set_navigation_hold(False, "fresh_close_evidence_rejected")
            return
        if self.target_close_last_stamp != stamp_key:
            self.target_close_last_stamp = stamp_key
            self.target_close_hits += 1
        self.target_close_last_seen = now
        if self.target_close_since is None:
            self.target_close_since = now
            self.publish_goal_arbitration(
                "target_close_confirmation_started",
                target_track_id=self.target_track_id,
                approach_track_id=self.target_approach_track_id,
                score=round(float(det.score), 4),
                score_threshold=round(float(close_score_threshold), 4),
                box=[round(float(det.w), 4), round(float(det.h), 4)],
                hits=int(self.target_close_hits),
                required_hits=int(self.target_done_min_fresh_hits),
                required_hold_seconds=round(float(self.target_done_min_hold_time), 3),
            )
            rospy.loginfo(
                "GoalManager: close target confirmation started score=%.3f threshold=%.3f "
                "box=(%.3f,%.3f) hits=%d/%d",
                float(det.score), float(close_score_threshold),
                float(det.w), float(det.h),
                self.target_close_hits, self.target_done_min_fresh_hits,
            )
            return
        if (
            self.target_close_hits >= self.target_done_min_fresh_hits
            and (now - self.target_close_since) >= self.target_done_min_hold_time
        ):
            completion_evidence = "close_box"
            if (
                not getattr(self, "target_done_require_confirmed_track", True)
                and not self.target_follow_confirmed
            ):
                completion_evidence = "candidate_track"
            self._complete_target_task(
                now, det, close_score_threshold, completion_evidence
            )
