"""Anchor graph storage and pending-branch admission for GP exploration."""

import numpy as np
import rospy

from gp_frontier_common import angle_diff, wrap_angle


class GpFrontierAnchorGraphMixin:
    """Maintain physical anchors and the unexplored branches attached to them."""

    def ensure_anchor(self, force: bool = False) -> int:
        """Create an anchor at the current pose when its spacing requires it."""
        if self.pose is None:
            return self.anchor_last_id
        current_xy = (self.pose.x, self.pose.y)
        if self.anchor_last_xy is None:
            force = True
        else:
            dx = current_xy[0] - self.anchor_last_xy[0]
            dy = current_xy[1] - self.anchor_last_xy[1]
            if np.hypot(dx, dy) > self.anchor_step_dist:
                force = True
        if not force:
            return self.anchor_last_id

        node_id = len(self.anchor_nodes)
        node = {
            "id": node_id,
            "x": current_xy[0],
            "y": current_xy[1],
            "stamp": rospy.Time.now().to_sec(),
            "prev": self.anchor_last_id,
            "branches": [],
            "lste_state": self.lste_state,
            "lste_subtype": self.lste_subtype,
            "profile": self.current_profile,
            "active_mode": self.active_mode,
        }
        self.anchor_nodes.append(node)
        self.anchor_last_xy = current_xy
        self.anchor_last_id = node_id
        # Keep a continuous anchor trace while a return path is being recorded.
        if self.backtrack_recording and (
            not self.backtrack_path
            or node_id == self.backtrack_path[-1] + 1
        ):
            self.backtrack_path.append(node_id)
        return self.anchor_last_id

    def _get_node(self, node_id: int):
        if node_id is None:
            return None
        if 0 <= node_id < len(self.anchor_nodes):
            return self.anchor_nodes[node_id]
        return None

    def _add_branch_if_new(self, node_id: int, heading_world: float, weight: float):
        """Register a new unexplored branch, merging equivalent evidence.

        ``heading_world`` is expressed in the odom world frame. Only
        ``PENDING`` branches enter the return stack and can later be selected
        as a forced-forward direction.
        """
        node = self._get_node(node_id)
        if node is None:
            return None
        heading_world = wrap_angle(heading_world)
        now_ts = rospy.Time.now().to_sec()

        for branch in node["branches"]:
            if (
                angle_diff(branch.get("heading_world", 0.0), heading_world)
                <= self.cluster_eps_rad
            ):
                branch["weight"] = max(branch.get("weight", 0.0), weight)
                return branch["id"]

        merge_dist = getattr(self, "branch_merge_dist", None)
        if (
            merge_dist is not None
            and node.get("x") is not None
            and node.get("y") is not None
        ):
            for other in self.anchor_nodes:
                if (
                    other is None
                    or other.get("x") is None
                    or other.get("y") is None
                    or other.get("id") == node_id
                ):
                    continue
                if np.hypot(node["x"] - other["x"], node["y"] - other["y"]) > merge_dist:
                    continue
                for branch in other.get("branches", []):
                    if (
                        angle_diff(
                            branch.get("heading_world", 0.0), heading_world,
                        )
                        <= self.cluster_eps_rad
                    ):
                        branch["weight"] = max(branch.get("weight", 0.0), weight)
                        return None

        pending_branches = [
            (index, branch)
            for index, branch in enumerate(node["branches"])
            if branch.get("status") == "PENDING"
        ]
        if len(pending_branches) >= self.max_pending_per_node:
            lowest_index, lowest = min(
                pending_branches,
                key=lambda item: item[1].get("weight", 0.0),
            )
            if weight <= lowest.get("weight", 0.0):
                return None
            node["branches"][lowest_index] = {
                "id": lowest_index,
                "heading_world": heading_world,
                "weight": weight,
                "status": "PENDING",
                "created": now_ts,
            }
            self._remove_from_backtrack_stack(node_id, lowest_index)
            self.backtrack_stack.append((node_id, lowest_index, weight))
            self.backtrack_stack.sort(key=lambda item: (item[0], item[2]))
            return lowest_index

        branch_id = len(node["branches"])
        node["branches"].append({
            "id": branch_id,
            "heading_world": heading_world,
            "weight": weight,
            "status": "PENDING",
            "created": now_ts,
        })
        self.backtrack_stack.append((node_id, branch_id, weight))
        # ``pop`` selects the newest anchor and, within it, the best branch.
        self.backtrack_stack.sort(key=lambda item: (item[0], item[2]))
        return branch_id

    def _nearest_anchor_dist(self, x: float, y: float) -> float:
        """Return the distance to the closest anchor, or ``None`` when empty."""
        if not self.anchor_nodes:
            return None
        return min(
            np.hypot(x - node.get("x", 0.0), y - node.get("y", 0.0))
            for node in self.anchor_nodes
        )
