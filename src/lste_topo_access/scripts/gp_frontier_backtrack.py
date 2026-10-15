"""Return-to-branch state machine for the legacy GP frontier explorer."""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8


class GpFrontierBacktrackMixin:
    """Select a pending branch, navigate to it, and hand off forced progress."""

    def set_access_mode(self, mode: int):
        """Publish topology priority: forward, return, or forced branch."""
        self.access_mode = mode
        self.access_mode_pub.publish(UInt8(mode))

    def _publish_backtrack_goal(self, node_id: int):
        node = self._get_node(node_id)
        if node is None:
            return
        message = PoseStamped()
        message.header.frame_id = "odom"
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = node["x"]
        message.pose.position.y = node["y"]
        message.pose.orientation.w = 1.0
        self.backtrack_goal_pub.publish(message)

    def _finalize_backtrack_session(self, end_anchor: int = None, status: str = "done"):
        """Store the completed return segment and reset recording state."""
        if end_anchor is not None and self.backtrack_recording and (
            not self.backtrack_path
            or end_anchor == self.backtrack_path[-1] + 1
        ):
            self.backtrack_path.append(end_anchor)
        if (
            self.backtrack_path
            or self.backtrack_start_pose
            or self.backtrack_start_anchor is not None
        ):
            session = {
                "start_pose": (
                    dict(self.backtrack_start_pose)
                    if self.backtrack_start_pose else None
                ),
                "start_anchor": self.backtrack_start_anchor,
                "path": list(self.backtrack_path),
                "end_anchor": end_anchor,
                "status": status,
                "finished": rospy.Time.now().to_sec(),
            }
            if self.current_backtrack is not None:
                session["target"] = {
                    "node": self.current_backtrack[0],
                    "branch": self.current_backtrack[1],
                }
            self.backtrack_history.append(session)
        self.backtrack_recording = False
        self.backtrack_path = []
        self.backtrack_start_pose = None
        self.backtrack_start_anchor = None
        self.no_frontier_start_pose = None
        self.no_frontier_start_anchor = None

    def enter_backtrack(self):
        """Choose a pending branch, or return home when no branch remains."""
        if self.access_mode == 1 and self.current_backtrack is not None:
            return
        if not self.backtrack_stack:
            self.return_home_target = 0
            if self.anchor_nodes:
                self._publish_backtrack_goal(self.return_home_target)
            self.set_access_mode(1)
            self.change_flag = False
            self.change_flag_pub.publish(self.change_flag)
            return

        self.return_home_target = None
        node_id, branch_id, _weight = self.backtrack_stack.pop()
        node = self._get_node(node_id)
        if node is None:
            return
        if self.backtrack_start_anchor is None:
            start_anchor = self.ensure_anchor(force=False)
            self.backtrack_start_anchor = start_anchor
            if self.backtrack_recording:
                self.backtrack_path = []
                if start_anchor is not None:
                    self.backtrack_path.append(start_anchor)
        branches = node["branches"]
        if 0 <= branch_id < len(branches):
            branches[branch_id]["trackback_target"] = True
        self.current_backtrack = (node_id, branch_id)
        self.set_access_mode(1)
        self.change_flag = False
        self.change_flag_pub.publish(self.change_flag)
        self._publish_backtrack_goal(node_id)

    def _remove_from_backtrack_stack(self, node_id: int, branch_id: int):
        """Remove an already-consumed branch from the return candidate stack."""
        self.backtrack_stack = [
            item for item in self.backtrack_stack
            if not (len(item) >= 2 and item[0] == node_id and item[1] == branch_id)
        ]

    @staticmethod
    def _pick_forced_branch(node: dict, fallback_branch_id: int = None) -> int:
        """Prefer the highest-weight pending branch at the arrival anchor."""
        if not node:
            return fallback_branch_id
        pending = [
            (branch.get("weight", 0.0), index)
            for index, branch in enumerate(node.get("branches", []))
            if branch.get("status") == "PENDING"
        ]
        return (
            max(pending, key=lambda item: item[0])[1]
            if pending else fallback_branch_id
        )

    def check_backtrack_progress(self):
        """Advance from return navigation into a forced pending branch."""
        if self.access_mode != 1 or self.pose is None:
            return
        if self.return_home_target is not None:
            target = self._get_node(self.return_home_target)
            if target is not None:
                distance_home = np.hypot(
                    self.pose.x - target["x"], self.pose.y - target["y"],
                )
                if distance_home <= self.backtrack_arrive_dist:
                    end_anchor = self.ensure_anchor(force=True)
                    self._finalize_backtrack_session(end_anchor, "reach_home")
                    self.return_home_target = None
                    self.current_backtrack = None
                    self.set_access_mode(0)
                    self.change_flag = True
                    self.change_flag_pub.publish(self.change_flag)
                    return
            if self.current_backtrack is None:
                return

        node_id, branch_id = self.current_backtrack
        node = self._get_node(node_id)
        if node is None:
            return
        distance = np.hypot(self.pose.x - node["x"], self.pose.y - node["y"])
        if distance > self.backtrack_arrive_dist:
            return
        if self.backtrack_recording:
            end_anchor = self.ensure_anchor(force=True)
            self._finalize_backtrack_session(end_anchor, "reach_interest")
        forced_branch_id = self._pick_forced_branch(node, branch_id)
        if forced_branch_id is not None:
            self._remove_from_backtrack_stack(node_id, forced_branch_id)
            self._start_forced_heading(node_id, forced_branch_id)
