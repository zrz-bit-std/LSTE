"""Forced-forward commitment after returning to an unexplored GP branch."""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from gp_frontier_common import wrap_angle


class GpFrontierForcedBranchMixin:
    """Hold one branch direction until physical progress commits it."""

    def _publish_commit_carrot(self):
        """Publish the fixed forward goal used while a branch is committed."""
        if self.forced_heading_world is None or self.forced_start_xy is None:
            return
        distance = getattr(self, "commit_goal_dist", 5.0)
        heading = self.forced_heading_world
        try:
            if self.headings:
                differences = [
                    abs(wrap_angle(self.forced_heading_world - candidate))
                    for candidate in self.headings
                ]
                heading = self.headings[int(np.argmin(differences))]
        except Exception:
            pass
        x = self.forced_start_xy[0] + distance * np.cos(heading)
        y = self.forced_start_xy[1] + distance * np.sin(heading)
        self.commit_goal = {"x": x, "y": y}
        message = PoseStamped()
        message.header.frame_id = "odom"
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.w = 1.0
        self.backtrack_goal_pub.publish(message)

    def _start_forced_heading(self, node_id: int, branch_id: int):
        node = self._get_node(node_id)
        if node is None:
            return
        branches = node["branches"]
        if not 0 <= branch_id < len(branches):
            return
        self.forced_heading_world = branches[branch_id].get("heading_world")
        self.forced_start_xy = (
            node.get("x", self.pose.x),
            node.get("y", self.pose.y),
        )
        self.forced_branch = (node_id, branch_id)
        self.mode2_start_time = rospy.Time.now().to_sec()
        self.current_backtrack = None
        self.set_access_mode(2)
        self.change_flag = True
        self.change_flag_pub.publish(self.change_flag)
        self._publish_commit_carrot()

    def update_forced_heading_progress(self):
        """Complete the commitment once the base advances far enough."""
        if (
            self.forced_heading_world is None
            or self.forced_start_xy is None
            or self.pose is None
        ):
            return
        dx = self.pose.x - self.forced_start_xy[0]
        dy = self.pose.y - self.forced_start_xy[1]
        progress = (
            dx * math.cos(self.forced_heading_world)
            + dy * math.sin(self.forced_heading_world)
        )
        self._publish_commit_carrot()
        if progress < self.commit_dist:
            return
        if self.forced_branch is not None:
            node = self._get_node(self.forced_branch[0])
            if node is not None:
                branch_id = self.forced_branch[1]
                if 0 <= branch_id < len(node.get("branches", [])):
                    node["branches"][branch_id]["status"] = "DONE"
                    node["branches"][branch_id].pop("trackback_target", None)
        self.forced_heading_world = None
        self.forced_start_xy = None
        self.forced_branch = None
        self.mode2_start_time = None
        self.commit_goal = None
        self.set_access_mode(0)
        self.change_flag = True
        self.change_flag_pub.publish(self.change_flag)
