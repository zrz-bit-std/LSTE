#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS input callbacks that update GoalManager mission state."""

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String, UInt8

from lste_msgs.msg import LsteDetections, LsteFrontiers, LsteScores, LsteState, LsteTask
from goal_context import task_version_from_task
from goal_manager_detection import handle_detections, record_detector_frame
from goal_manager_modes import CATCH_CTX_MODE, CATCH_TARGET_MODE, STATE_LOCKED
from lifecycle_manager import EventType


class GoalManagerInputCallbacksMixin:
    """Callbacks for state, perception, task, and operator input topics."""

    @staticmethod
    def _enqueue_or_apply(owner, event_type, message, apply):
        enqueue = getattr(owner, "_enqueue_lifecycle_event", None)
        if callable(enqueue):
            return enqueue(event_type, message)
        return apply(message)

    def set_navigation_hold(self, active: bool, reason: str):
        """Publish mission-level observation hold only on state changes."""
        active = bool(active)
        if active == self.navigation_hold_active:
            return
        self.navigation_hold_active = active
        self.pub_navigation_hold.publish(Bool(data=active))
        rospy.loginfo(
            "GoalManager: navigation_hold=%s reason=%s remaining=%.2fs",
            active,
            reason,
            max(0.0, self.target_observation_hold_until - rospy.Time.now().to_sec()),
        )

    def on_state(self, msg: LsteState):
        return self._enqueue_or_apply(self, EventType.STATE_UPDATED, msg, self.apply_state)

    def apply_state(self, msg: LsteState):
        prev_state = self.current_state
        prev_subtype = self.current_subtype
        self.latest_state_msg = msg
        self.current_state = msg.state
        self.current_subtype = msg.subtype or ""
        now = rospy.Time.now().to_sec()
        if self.current_state == STATE_LOCKED:
            if self.locked_enter_time is None:
                self.locked_enter_time = now
        else:
            self.locked_enter_time = None
            if self.target_done_require_locked:
                self.reset_target_close_confirmation()

        new_base = self.mode_from_state(self.current_state, self.current_subtype)
        # Preserve a recently confirmed target through a transient context-only
        # classification rather than switching to an unrelated context midpoint.
        target_recent = self.target_tracking_active(now)
        if new_base == CATCH_CTX_MODE and target_recent:
            new_base = CATCH_TARGET_MODE
        elif new_base == CATCH_CTX_MODE:
            self.ctx_state_hold_until = max(
                self.ctx_state_hold_until,
                now + self.context_state_hold_time,
            )
        elif (
            not target_recent
            and now < self.ctx_state_hold_until
            and self.base_mode == CATCH_CTX_MODE
        ):
            rospy.loginfo_throttle(
                2.0,
                "GoalManager: hold context mode through transient state=%s "
                "subtype=%s for %.1fs",
                str(self.current_state),
                self.current_subtype or "-",
                max(0.0, self.ctx_state_hold_until - now),
            )
            new_base = CATCH_CTX_MODE
        if new_base != self.base_mode:
            rospy.loginfo(
                "GoalManager: base_mode %s -> %s (state=%s subtype=%s)",
                self.base_mode,
                new_base,
                str(self.current_state),
                self.current_subtype,
            )
        self.base_mode = new_base
        self.effective_mode = new_base

        if (
            self.last_state is None
            or prev_state != msg.state
            or prev_subtype != self.current_subtype
        ):
            self.next_update_time = 0.0
        self.last_state = msg.state

    def on_dets(self, msg: LsteDetections):
        return self._enqueue_or_apply(self, EventType.DETECTIONS_UPDATED, msg, self.apply_dets)

    def apply_dets(self, msg: LsteDetections):
        """Forward one detector frame to the detection evidence state machine."""
        record_detector_frame(self, msg)
        handle_detections(self, msg)

    def on_scores(self, msg: LsteScores):
        return self._enqueue_or_apply(self, EventType.SCORES_UPDATED, msg, self.apply_scores)

    def apply_scores(self, msg: LsteScores):
        self.latest_scores = msg

    def on_task(self, msg: LsteTask):
        return self._enqueue_or_apply(self, EventType.TASK_UPDATED, msg, self.apply_task)

    def apply_task(self, msg: LsteTask):
        task_id = (msg.task_id or "").strip()
        task_version = task_version_from_task(msg)
        mission_id = task_id
        mission_changed = (
            task_id != self.current_task_id
            or task_version != self.current_task_version
        )
        if mission_changed:
            previous = self.current_task_id or "<none>"
            previous_version = self.current_task_version or "<none>"
            self.current_task_id = task_id
            self.current_task_version = task_version
            self.current_mission_id = mission_id
            self.task_done_published = False
            self.reset_target_close_confirmation()
            self.target_completed_segments = 0
            approach_transaction = getattr(
                self, "target_approach_transaction", None
            )
            if approach_transaction is not None:
                approach_transaction.clear("new_task")
            # A task-version change invalidates the previous semantic
            # obligation. Never carry a confirmed target into a new mission.
            self.clear_target_memory(preserve_obligation=False)
            self.target_room_entry_goal = None
            self.target_room_entry_retry_blocked = False
            self.target_room_entry_blocked_route_id = 0
            self.frontier_terminal_count = 0
            reset_room_entry = getattr(self, "reset_target_room_entry_state", None)
            if callable(reset_room_entry):
                reset_room_entry()
            self.set_navigation_hold(False, "new_task")
            # task_done is latched, so consumers need an explicit false for a
            # new task.
            self.pub_task_done.publish(Bool(data=False))
            rospy.loginfo(
                "GoalManager: mission changed task=%s[%s] -> %s[%s]; clear task_done",
                previous,
                previous_version,
                task_id,
                task_version or "<none>",
            )
        else:
            # Keep identity fields populated even for the first empty-id task.
            self.current_task_version = task_version
            self.current_mission_id = mission_id
        self.latest_task = msg

    def on_pose(self, msg: Pose2D):
        return self._enqueue_or_apply(self, EventType.POSE_UPDATED, msg, self.apply_pose)

    def apply_pose(self, msg: Pose2D):
        self.latest_pose = msg
        if self.start_pose is None:
            self.start_pose = msg
            self.init_headings(msg)
        elif self.headings is None:
            self.init_headings(msg)

    def on_cam_info(self, msg: CameraInfo):
        return self._enqueue_or_apply(self, EventType.CAMERA_INFO_UPDATED, msg, self.apply_cam_info)

    def apply_cam_info(self, msg: CameraInfo):
        try:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info = msg
            self.camera_frame = msg.header.frame_id or self.camera_frame
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to load camera info: %s", exc)

    def on_depth(self, msg: Image):
        return self._enqueue_or_apply(self, EventType.DEPTH_UPDATED, msg, self.apply_depth)

    def apply_depth(self, msg: Image):
        self.depth_image = msg
        self.depth_stamp = msg.header.stamp

    def on_frontier(self, msg: Vector3Stamped):
        return self._enqueue_or_apply(self, EventType.FRONTIER_UPDATED, msg, self.apply_frontier)

    def apply_frontier(self, msg: Vector3Stamped):
        self.latest_frontier = msg

    def on_frontiers(self, msg: LsteFrontiers):
        return self._enqueue_or_apply(self, EventType.FRONTIERS_UPDATED, msg, self.apply_frontiers)

    def apply_frontiers(self, msg: LsteFrontiers):
        self.latest_frontiers = msg

    def on_controller_mode(self, msg: String):
        return self._enqueue_or_apply(self, EventType.CONTROLLER_MODE_UPDATED, msg, self.apply_controller_mode)

    def apply_controller_mode(self, msg: String):
        mode = (msg.data or "").strip().lower()
        if mode in ("sappo", "teleop", "teb"):
            if mode != self.controller_mode:
                rospy.loginfo(
                    "GoalManager: controller mode %s -> %s",
                    self.controller_mode,
                    mode,
                )
            self.controller_mode = mode

    def on_scan(self, msg: LaserScan):
        return self._enqueue_or_apply(self, EventType.SCAN_UPDATED, msg, self.apply_scan)

    def apply_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_cmd_vel(self, msg: Twist):
        return self._enqueue_or_apply(self, EventType.CMD_VEL_UPDATED, msg, self.apply_cmd_vel)

    def apply_cmd_vel(self, msg: Twist):
        self.latest_cmd_vel = msg

    def on_access_mode(self, msg: UInt8):
        return self._enqueue_or_apply(self, EventType.ACCESS_MODE_UPDATED, msg, self.apply_access_mode)

    def apply_access_mode(self, msg: UInt8):
        try:
            self.access_mode = int(msg.data)
        except Exception:
            self.access_mode = 0

    def on_access_goal(self, msg: PoseStamped):
        return self._enqueue_or_apply(self, EventType.ACCESS_GOAL_UPDATED, msg, self.apply_access_goal)

    def apply_access_goal(self, msg: PoseStamped):
        self.access_backtrack_goal = msg

    def on_fixed_goal_click(self, msg: PoseStamped):
        return self._enqueue_or_apply(self, EventType.FIXED_GOAL_UPDATED, msg, self.apply_fixed_goal_click)

    def apply_fixed_goal_click(self, msg: PoseStamped):
        """Replace the fixed target from the Gazebo Shift-click UI."""
        previous = self.fixed_goal
        yaw = self.yaw_from_pose(msg)
        self.fixed_goal = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            previous[2] if yaw is None else yaw,
        )
        self.next_update_time = 0.0
        rospy.logwarn(
            "Fixed global goal replaced by click: (%.2f, %.2f) -> (%.2f, %.2f)",
            previous[0],
            previous[1],
            self.fixed_goal[0],
            self.fixed_goal[1],
        )
