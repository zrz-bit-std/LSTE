"""Target-selection and goal-diagnostic helpers for :mod:`lste_goal_manager`.

The GoalManager class is the mission state machine.  This mixin keeps the
small, mostly side-effect-free target utilities in one place so detection
tracking and diagnostic formatting can be edited without navigating the ROS
callback and goal-publication code.
"""

import json
import math
from typing import Any, Optional, Tuple

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import String
from tf.transformations import quaternion_matrix

from target_track_identity import labels_compatible

class GoalManagerTargetUtilsMixin:
    def target_navigation_heading(self) -> Optional[float]:
        """Return a heading to the persistent multi-ray target hypothesis.

        A latest image ray is valid only at its exposure pose.  Once the
        static-ray estimator has a forward intersection, recompute the
        direction from the current odometry pose so each approach segment
        converges on the same physical point instead of repeating a stale
        camera bearing.
        """
        fallback = getattr(self, "target_last_heading", None)
        # The estimator is diagnostic unless the current experiment explicitly
        # enables calibrated point navigation.  Falling back to the latest
        # source-stamped bearing is safer than steering toward an unvalidated
        # intersection produced by a false detector association.
        if not getattr(self, "target_hypothesis_navigation_enabled", False):
            return fallback
        point = getattr(self, "target_hypothesis_xy", None)
        pose = getattr(self, "latest_pose", None)
        if point is None or pose is None:
            return fallback
        try:
            dx = float(point[0]) - float(pose.x)
            dy = float(point[1]) - float(pose.y)
        except (IndexError, TypeError, ValueError, AttributeError):
            return fallback
        if not (math.isfinite(dx) and math.isfinite(dy)):
            return fallback
        if math.hypot(dx, dy) <= 0.20:
            return fallback
        return math.atan2(dy, dx)

    @staticmethod
    def goal_intent_priority(source: str) -> int:
        """Return mission ownership priority used by the TEB action bridge."""
        source = str(source or "").strip().lower()
        if source.startswith("target_"):
            return 2
        if source == "fixed_config":
            return 3
        return 0

    def log_goal_diagnostic(
        self, goal: PoseStamped, previous_goal: Optional[PoseStamped]
    ):
        """Record the state, perception and motion behind one goal decision."""
        pose = self.latest_pose
        if pose is None:
            return
        goal_yaw = self.yaw_from_pose(goal) or 0.0
        robot_goal_dist = self.goal_robot_distance(goal)
        if robot_goal_dist is None:
            robot_goal_dist = float("nan")
        goal_delta = 0.0
        if previous_goal is not None:
            goal_delta = math.hypot(
                goal.pose.position.x - previous_goal.pose.position.x,
                goal.pose.position.y - previous_goal.pose.position.y,
            )
        cmd_v = (
            float(self.latest_cmd_vel.linear.x)
            if self.latest_cmd_vel is not None
            else 0.0
        )
        cmd_w = (
            float(self.latest_cmd_vel.angular.z)
            if self.latest_cmd_vel is not None
            else 0.0
        )
        if self.latest_scores is None:
            score_info = "unavailable"
        else:
            score_info = (
                "total=%.3f,target=%.3f,env=%.3f,ctx=%.3f,detected=%s"
                % (
                    self.latest_scores.s_total,
                    self.latest_scores.s_target,
                    self.latest_scores.s_env,
                    self.latest_scores.s_ctx,
                    self.latest_scores.detected,
                )
            )
        det_info = self.diagnostic_detection_summary()
        rospy.loginfo(
            "GOAL_DIAG mission=%s[%s] task=%s source=%s state=%d subtype=%s mode=%s access=%d "
            "robot=(%.2f,%.2f,%.2f) cmd=(%.2f,%.2f) "
            "goal=(%.2f,%.2f,%.2f) robot_goal_dist=%.2f goal_delta=%.2f "
            "scores={%s} detections={%s} frontiers={%s}",
            getattr(self, "current_mission_id", "") or "-",
            getattr(self, "current_task_version", "") or "-",
            getattr(self, "current_task_id", "") or "-",
            self.goal_source,
            self.current_state,
            self.current_subtype or "-",
            self.effective_mode,
            self.access_mode,
            pose.x,
            pose.y,
            pose.theta,
            cmd_v,
            cmd_w,
            goal.pose.position.x,
            goal.pose.position.y,
            goal_yaw,
            robot_goal_dist,
            goal_delta,
            score_info,
            det_info,
            self.frontier_debug,
        )
        self.pub_goal_diagnostic.publish(
            String(
                data=json.dumps(
                    {
                        "mission_id": str(getattr(self, "current_mission_id", "")),
                        "task_id": str(getattr(self, "current_task_id", "")),
                        "task_version": str(getattr(self, "current_task_version", "")),
                        "source": self.goal_source,
                        "target_state": self.target_execution_state,
                        "target_epoch": int(self.target_observation_epoch),
                        "target_blocked": bool(self.target_blocked),
                        "target_failure_count": int(self.target_failure_count),
                        "target_viewpoint_candidate_id": str(
                            getattr(self, "target_viewpoint_candidate_id", "")
                            or ""
                        ),
                        "target_viewpoint_attempt_id": str(
                            getattr(self, "target_viewpoint_attempt_id", "")
                            or ""
                        ),
                        "target_approach_transaction": (
                            None
                            if getattr(self, "target_approach_transaction", None) is None
                            else self.target_approach_transaction.snapshot().__dict__
                        ),
                        "target_observation_gate": (
                            None
                            if getattr(self, "target_observation_gate", None) is None
                            else self.target_observation_gate.snapshot().__dict__
                        ),
                        "navigation_hold": bool(self.navigation_hold_active),
                        "navigation_hold_remaining": round(
                            max(
                                0.0,
                                self.target_observation_hold_until
                                - rospy.Time.now().to_sec(),
                            ),
                            3,
                        ),
                        "state": int(self.current_state),
                        "subtype": self.current_subtype or "",
                        "mode": self.effective_mode,
                        "access_mode": int(self.access_mode),
                        "robot": [
                            round(float(pose.x), 4),
                            round(float(pose.y), 4),
                            round(float(pose.theta), 4),
                        ],
                        "cmd": [round(cmd_v, 4), round(cmd_w, 4)],
                        "goal": [
                            round(float(goal.pose.position.x), 4),
                            round(float(goal.pose.position.y), 4),
                            round(float(goal_yaw), 4),
                        ],
                        "robot_goal_distance": round(float(robot_goal_dist), 4),
                        "goal_delta": round(float(goal_delta), 4),
                        "scores": None
                        if self.latest_scores is None
                        else {
                            "total": round(float(self.latest_scores.s_total), 4),
                            "target": round(float(self.latest_scores.s_target), 4),
                            "env": round(float(self.latest_scores.s_env), 4),
                            "ctx": round(float(self.latest_scores.s_ctx), 4),
                            "detected": bool(self.latest_scores.detected),
                        },
                        "detections": det_info,
                        "frontier_debug": self.frontier_debug,
                    },
                    sort_keys=True,
                )
            )
        )

    def diagnostic_detection_summary(self) -> str:
        if self.latest_dets is None:
            return "unavailable"
        target_count = len(self.latest_dets.target_dets)
        env_count = len(self.latest_dets.env_dets)
        best = self.pick_best_target()
        if best is None:
            return "target=0,env=%d" % env_count
        return "target=%d,env=%d,best=%s:%.3f@%.3f,%.3f box=%.3fx%.3f" % (
            target_count,
            env_count,
            best.label,
            best.score,
            best.cx,
            best.cy,
            best.w,
            best.h,
        )

    def build_goal_from_pose(self, pose: Pose2D, dist: float) -> PoseStamped:
        x = pose.x + dist * math.cos(pose.theta)
        y = pose.y + dist * math.sin(pose.theta)
        return self.make_goal_pose((x, y, 0.0), pose.theta)

    def pick_best_target(self) -> Optional[Any]:
        return self.best_target_from(self.latest_dets)

    def target_detection_for_track(
        self, message: Optional[Any]
    ) -> Optional[Any]:
        """Keep a locked target identity stable while selecting its best box.

        Once a track exists, a frame containing only another high-scoring
        object is not a valid update.  The old ``best_target_from`` fallback
        silently reassigned the track and caused target goals to jump.
        """
        if message is None or not message.target_dets:
            return None
        if self.target_track_label:
            same_label = [
                detection
                for detection in message.target_dets
                if labels_compatible(
                    self.target_track_label,
                    (detection.label or "").strip().lower(),
                )
            ]
            if same_label:
                return max(same_label, key=lambda detection: float(detection.score))
            return None
        return self.best_target_from(message)

    @staticmethod
    def best_target_from(msg: Optional[Any]) -> Optional[Any]:
        if msg is None or not msg.target_dets:
            return None
        return max(msg.target_dets, key=lambda detection: float(detection.score))

    @staticmethod
    def clone_detection(det: Any) -> Any:
        # Construct the same generated ROS message type without importing the
        # package at module import time.  This keeps the utility mixin usable
        # by dependency-free policy tests and by catkin's normal runtime.
        cloned = type(det)()
        cloned.label = det.label
        cloned.score = float(det.score)
        cloned.cx = float(det.cx)
        cloned.cy = float(det.cy)
        cloned.w = float(det.w)
        cloned.h = float(det.h)
        return cloned

    def target_tracking_active(self, now: float) -> bool:
        if self.target_blocked:
            return False
        if getattr(self, "target_reinspection_pending", False):
            return False
        if self.target_follow_confirmed:
            gate = getattr(self, "target_observation_gate", None)
            if gate is not None and gate.track_id == self.target_track_id:
                # A confirmed track owns its Place through detector gaps. Only
                # the explicit post-terminal negative episode can make it
                # inactive; a wall-clock freshness check is insufficient.
                if gate.needs_viewpoint_reinspection:
                    return False
                return not gate.loss_certified(self.target_track_id)
            transaction = getattr(self, "target_approach_transaction", None)
            if transaction is not None and transaction.active:
                return True
        if self.target_last_seen is None:
            return False
        # A weak candidate remains active only during its explicit observation
        # hold. Once it expires, frontier exploration owns the next decision.
        if not self.target_follow_confirmed and now >= self.target_observation_hold_until:
            return False
        return (now - self.target_last_seen) < self.target_evidence_timeout()

    def target_candidate_information_active(self, now: float) -> bool:
        """Return whether a strong unconfirmed box may request one view action.

        A candidate normally must not preempt a map route.  A strong direct
        observation is the exception: it can own one bounded parallax side-step
        so the robot obtains the missing translational evidence.  This method
        does not confirm the track and never authorizes a pursuit segment.
        """
        if (
            getattr(self, "target_blocked", False)
            or getattr(self, "target_follow_confirmed", False)
            or not getattr(self, "target_candidate_room_claim_requested", False)
        ):
            return False
        last_seen = getattr(self, "target_candidate_last_seen", None)
        candidate = getattr(self, "target_candidate", None)
        if last_seen is None or candidate is None:
            return False
        try:
            if float(now) - float(last_seen) >= float(
                getattr(self, "target_follow_candidate_timeout", 0.0)
            ):
                return False
            return (
                float(candidate.score)
                >= max(0.40, float(self.target_done_min_score))
                or max(float(candidate.w), float(candidate.h)) >= 0.05
            )
        except (AttributeError, TypeError, ValueError):
            return False

    def target_evidence_timeout(self) -> float:
        """Return the direct-evidence expiry for the current target phase."""
        return (
            self.follow_target_lost_timeout
            if self.target_follow_confirmed
            else self.target_follow_candidate_timeout
        )

    def target_segment_ownership_active(self) -> bool:
        """Return whether a validated visual segment still owns motion."""
        return self.target_last_goal is not None and not self.target_blocked

    def pick_ctx_pair(
        self,
    ) -> Tuple[Optional[Any], Optional[Any]]:
        """Return the closest configured left/right context detections."""
        if self.latest_dets is None or self.latest_task is None:
            return None, None
        left_term = (self.latest_task.ctx_left or "").strip().lower()
        right_term = (self.latest_task.ctx_right or "").strip().lower()
        if not left_term or left_term == "none" or not right_term or right_term == "none":
            return None, None
        detections = list(self.latest_dets.env_dets) + list(self.latest_dets.target_dets)
        left_hits = [d for d in detections if left_term in (d.label or "").lower()]
        right_hits = [d for d in detections if right_term in (d.label or "").lower()]
        if not left_hits or not right_hits:
            return None, None
        best_pair = (None, None)
        best_distance = float("inf")
        for left in left_hits:
            for right in right_hits:
                if left is right:
                    continue
                distance = math.hypot(
                    float(left.cx) - float(right.cx),
                    float(left.cy) - float(right.cy),
                )
                if distance < best_distance:
                    best_distance = distance
                    best_pair = (left, right)
        return best_pair if best_pair[0] is not None else (None, None)

    def yaw_from_pose(self, pose: PoseStamped) -> Optional[float]:
        try:
            q = pose.pose.orientation
            matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
            return math.atan2(matrix[1, 0], matrix[0, 0])
        except Exception:
            return None

    def _slerp_yaw(self, previous: float, new: float, alpha: float) -> float:
        """Interpolate yaw across the shortest angular distance."""
        delta = wrap_angle(new - previous)
        return wrap_angle(previous + alpha * delta)


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
