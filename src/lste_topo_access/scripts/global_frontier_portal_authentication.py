"""Ordered doorway observation transactions for a confirmed junction."""

from dataclasses import dataclass, field
import math


def _xy(value):
    try:
        point = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError):
        return None
    return point if all(math.isfinite(item) for item in point) else None


def _angle_delta(target, source):
    return math.atan2(math.sin(target - source), math.cos(target - source))


@dataclass
class DoorwayAuthentication:
    """One stable GP branch and its enter, inspect, turn, and return record."""

    branch_id: int
    node_id: int
    gate_xy: tuple
    destination_xy: tuple
    required_depth: float = 0.40
    return_radius: float = 0.60
    # Slightly exceed the TEB supervisor's 0.35 rad completion envelope so
    # pose quantization cannot leave authentication waiting after TEB stops.
    turn_tolerance: float = 0.40
    turn_direction: str = ""
    state: str = "pending"
    source_xy: tuple = None
    inside_xy: tuple = None
    initial_yaw: float = None
    target_yaw: float = None
    frontier_count_left: int = 0
    frontier_count_right: int = 0
    chosen_turn_side: str = None
    failure_reason: str = None
    turn_stable_samples: int = 0
    rotation_ready: bool = False
    events: list = field(default_factory=list)
    entry_goal_margin: float = 0.60

    def __post_init__(self):
        self.branch_id = int(self.branch_id)
        self.node_id = int(self.node_id)
        self.gate_xy = _xy(self.gate_xy)
        self.destination_xy = _xy(self.destination_xy)
        self.required_depth = max(0.0, float(self.required_depth))
        self.entry_goal_margin = max(0.0, float(self.entry_goal_margin))
        self.return_radius = max(0.05, float(self.return_radius))
        if self.gate_xy is None or self.destination_xy is None:
            self.fail("invalid_gate_or_destination")

    def _event(self, name, **data):
        event = {"event": name, "branch_id": self.branch_id,
                 "node_id": self.node_id}
        event.update(data)
        self.events.append(event)
        return event

    def begin(self, source_xy):
        if self.state != "pending":
            return False
        self.source_xy = _xy(source_xy)
        if self.source_xy is None:
            return self.fail("source_pose_unavailable")
        self.state = "entering"
        self._event("doorway_entry_started", source_xy=list(self.source_xy))
        return True

    def signed_depth(self, pose_xy):
        pose = _xy(pose_xy)
        if pose is None or self.gate_xy is None or self.destination_xy is None:
            return None
        dx = self.destination_xy[0] - self.gate_xy[0]
        dy = self.destination_xy[1] - self.gate_xy[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return None
        return ((pose[0] - self.gate_xy[0]) * dx
                + (pose[1] - self.gate_xy[1]) * dy) / length

    def route_goal(self):
        """Return XY for entry/return; turning holds XY and commands target yaw."""
        if self.state == "entering":
            dx = self.destination_xy[0] - self.gate_xy[0]
            dy = self.destination_xy[1] - self.gate_xy[1]
            length = math.hypot(dx, dy) or 1.0
            # Leave enough distance for the controller's endpoint tolerance
            # after the full robot footprint has crossed the gate plane.
            goal_depth = self.required_depth + self.entry_goal_margin
            return (self.gate_xy[0] + dx * goal_depth / length,
                    self.gate_xy[1] + dy * goal_depth / length)
        if self.state == "turning":
            return self.inside_xy
        if self.state == "returning":
            return self.source_xy
        return None

    def update_entry(self, pose_xy):
        if self.state != "entering":
            return False
        depth = self.signed_depth(pose_xy)
        if depth is None or depth < self.required_depth:
            return False
        self.inside_xy = _xy(pose_xy)
        self.state = "inside"
        self._event("vehicle_fully_inside", pose_xy=list(self.inside_xy),
                    depth=round(depth, 3))
        return True

    def observe_inside(self, left_frontiers, right_frontiers, current_yaw,
                       frontier_headings):
        """Freeze a turn direction and heading from this completed observation."""
        if self.state != "inside":
            return False
        self.frontier_count_left = max(0, int(left_frontiers))
        self.frontier_count_right = max(0, int(right_frontiers))
        total = self.frontier_count_left + self.frontier_count_right
        if total <= 0:
            return self.request_failure_return("no_frontier_inside_doorway")
        self.chosen_turn_side = (
            "left" if self.frontier_count_left >= self.frontier_count_right
            else "right"
        )
        self.turn_direction = "counterclockwise" if self.chosen_turn_side == "left" else "clockwise"
        self.initial_yaw = float(current_yaw)
        # A doorway observation ends with a U-turn back toward its source
        # node. Frontier counts choose the sweep direction; a small angular
        # bias keeps the equivalent half-turn unambiguous for TEB, and this
        # signed target stays fixed even as the visible frontier set changes.
        sign = 1.0 if self.chosen_turn_side == "left" else -1.0
        self.target_yaw = self.initial_yaw + sign * (math.pi - 0.08)
        self.state = "turning"
        self._event("inside_frontier_observed", left=self.frontier_count_left,
                    right=self.frontier_count_right, total=total,
                    turn_side=self.chosen_turn_side,
                    turn_direction=self.turn_direction,
                    initial_yaw=round(self.initial_yaw, 4),
                    target_yaw=round(self.target_yaw, 4))
        return True

    def update_rotation(self, current_yaw, stable_samples=3):
        """Complete only after a fixed target is held for several pose samples."""
        if self.state != "turning" or self.target_yaw is None:
            return False
        error = abs(_angle_delta(self.target_yaw, float(current_yaw)))
        if error <= self.turn_tolerance:
            self.turn_stable_samples += 1
        else:
            self.turn_stable_samples = 0
        if self.turn_stable_samples < max(1, int(stable_samples)):
            return False
        self.rotation_ready = True
        return True

    def finish_turn(self, current_yaw):
        if self.state != "turning" or not self.rotation_ready:
            return False
        self.state = "returning"
        self._event("room_facing_turn_complete", turn_side=self.chosen_turn_side,
                    turn_direction=self.turn_direction,
                    target_yaw=round(self.target_yaw, 4),
                    final_yaw=round(float(current_yaw), 4))
        return True

    def update_return(self, pose_xy):
        if self.state != "returning":
            return False
        pose = _xy(pose_xy)
        if pose is None or self.source_xy is None:
            return False
        distance = math.hypot(pose[0] - self.source_xy[0],
                              pose[1] - self.source_xy[1])
        if distance > self.return_radius:
            return False
        if self.failure_reason:
            self.state = "failed"
            self._event("returned_to_junction_node_after_failure",
                        pose_xy=list(pose), distance=round(distance, 3),
                        reason=self.failure_reason)
        else:
            self.state = "completed"
            self._event("returned_to_junction_node", pose_xy=list(pose),
                        distance=round(distance, 3))
        return True

    def request_failure_return(self, reason, pose_xy=None):
        """Remember a failed probe, but return to its source before settling it."""
        if self.state in ("completed", "failed"):
            return False
        self.failure_reason = str(reason or "authentication_failed")
        pose = _xy(pose_xy) or self.inside_xy
        if pose is not None and self.source_xy is not None:
            distance = math.hypot(pose[0] - self.source_xy[0],
                                  pose[1] - self.source_xy[1])
            if distance > self.return_radius:
                self.state = "returning"
                self._event("doorway_failure_return_started",
                            reason=self.failure_reason,
                            from_xy=list(pose), source_xy=list(self.source_xy),
                            distance=round(distance, 3))
                return True
        return self.fail(self.failure_reason)

    def fail(self, reason):
        if self.state in ("completed", "failed"):
            return False
        self.state = "failed"
        self.failure_reason = str(reason or "authentication_failed")
        self._event("doorway_authentication_failed", reason=self.failure_reason)
        return False

    def snapshot(self):
        return {
            "branch_id": self.branch_id, "node_id": self.node_id,
            "state": self.state,
            "gate_xy": None if self.gate_xy is None else list(self.gate_xy),
            "destination_xy": None if self.destination_xy is None else list(self.destination_xy),
            "source_xy": None if self.source_xy is None else list(self.source_xy),
            "inside_xy": None if self.inside_xy is None else list(self.inside_xy),
            "frontier_count_left": self.frontier_count_left,
            "frontier_count_right": self.frontier_count_right,
            "chosen_turn_side": self.chosen_turn_side,
            "turn_direction": self.turn_direction,
            "initial_yaw": self.initial_yaw, "target_yaw": self.target_yaw,
            "rotation_ready": self.rotation_ready,
            "failure_reason": self.failure_reason, "events": list(self.events),
        }


class JunctionDoorwayAuthentication:
    """Stable branch ledger and sequential observation period for one node."""

    def __init__(self, node_id, branches=(), settle_seconds=1.0):
        self.node_id = int(node_id)
        self.branches = {int(item.branch_id): item for item in branches}
        self.active_branch_id = None
        self.settle_seconds = max(0.0, float(settle_seconds))
        self.signature_since = 0.0
        self.signature = tuple(sorted(self.branches))
        self.phase = "watching"

    def add(self, branch, now=0.0):
        if int(branch.node_id) != self.node_id:
            raise ValueError("branch belongs to another node")
        branch_id = int(branch.branch_id)
        if branch_id in self.branches:
            return False
        self.branches[branch_id] = branch
        self.signature = tuple(sorted(self.branches))
        self.signature_since = float(now)
        self.phase = "watching"
        return True

    def next_pending(self):
        return next((self.branches[key] for key in sorted(self.branches)
                     if self.branches[key].state == "pending"), None)

    def start_next(self, source_xy, now=0.0, force=False):
        if not force and float(now) - self.signature_since < self.settle_seconds:
            self.phase = "watching"
            return None
        branch = self.next_pending()
        if branch is None:
            self.active_branch_id = None
            self.phase = "complete" if self.all_finished() else "watching"
            return None
        if not branch.begin(source_xy):
            return branch
        self.active_branch_id = branch.branch_id
        self.phase = "authenticating"
        return branch

    def finish_active(self, source_xy, now=0.0):
        branch = self.branches.get(self.active_branch_id)
        if branch is None or branch.state not in ("completed", "failed"):
            return None
        self.active_branch_id = None
        self.phase = "watching"
        return self.start_next(source_xy, now=now)

    def all_finished(self):
        return all(item.state in ("completed", "failed")
                   for item in self.branches.values())

    def snapshot(self):
        return {"node_id": self.node_id, "phase": self.phase,
                "active_branch_id": self.active_branch_id,
                "all_finished": self.all_finished(),
                "branches": [self.branches[key].snapshot()
                             for key in sorted(self.branches)]}


class GlobalFrontierPortalAuthenticationMixin:
    """Runtime adapter joining the authentication ledger to explorer events."""

    def initialize_portal_authentication(self):
        self.junction_portal_authentication = None
        self.portal_authentication_by_node = {}
        self.portal_authentication_history = []
        self.active_portal_authentication_phase = ""

    def begin_junction_portal_authentication(self, node_id, branches, source_xy, now=0.0):
        task = self.portal_authentication_by_node.get(int(node_id))
        if task is None:
            task = JunctionDoorwayAuthentication(node_id, settle_seconds=1.0)
            self.portal_authentication_by_node[int(node_id)] = task
        for branch in branches:
            task.add(branch, now=now)
        self.junction_portal_authentication = task
        branch = task.start_next(source_xy, now=now)
        snapshot = task.snapshot()
        self.publish_status("portal_authentication_observation_period",
                            node_id=int(node_id), phase=task.phase,
                            branch_ids=sorted(task.branches),
                            all_finished=task.all_finished())
        if branch is not None:
            self.publish_status("portal_authentication_branch_started",
                                node_id=int(node_id), branch_id=branch.branch_id,
                                state=branch.state)
        return branch

    def active_portal_authentication(self):
        task = getattr(self, "junction_portal_authentication", None)
        if task is None or task.active_branch_id is None:
            return None
        return task.branches.get(int(task.active_branch_id))

    def finish_portal_authentication_branch(self, now=0.0):
        task = getattr(self, "junction_portal_authentication", None)
        branch = self.active_portal_authentication()
        if task is None or branch is None:
            return None
        result = branch.snapshot()
        self.portal_authentication_history.append(result)
        tree = getattr(self, "gp_exploration_tree", None)
        tree_branch = None if tree is None else tree.branches.get(branch.branch_id)
        if tree_branch is not None:
            tree_branch["authentication_state"] = branch.state
            tree_branch["authentication_failure_reason"] = branch.failure_reason
            tree_branch["authentication_evidence"] = result
            if branch.state == "failed":
                tree.reject_branch_authentication(branch.branch_id,
                                                  branch.failure_reason)
            elif branch.state == "completed":
                tree_branch["state"] = "authenticated"
                tree_branch["authentication_state"] = "completed"
        node = None if tree is None else tree.nodes.get(task.node_id)
        if node is not None and branch.state in ("completed", "failed"):
            tree.current_node_id = task.node_id
        source_xy = (node.get("physical_xy") if node is not None
                     else branch.source_xy)
        next_branch = task.finish_active(source_xy, now=now)
        if next_branch is not None:
            if tree is not None:
                next_tree_branch = tree.branches.get(next_branch.branch_id)
                if next_tree_branch is not None:
                    next_tree_branch["authentication_state"] = "entering"
            self.publish_status(
                "portal_authentication_branch_started",
                node_id=int(task.node_id), branch_id=int(next_branch.branch_id),
                state=next_branch.state,
            )
        self.publish_status("portal_authentication_branch_finished",
                            node_id=int(task.node_id),
                            branch_id=int(branch.branch_id), state=branch.state,
                            failure_reason=branch.failure_reason,
                            next_branch_id=None if next_branch is None
                            else int(next_branch.branch_id),
                            observation_period=task.snapshot())
        return next_branch
