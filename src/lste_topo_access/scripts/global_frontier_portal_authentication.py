"""Ordered doorway observation transactions for a confirmed junction."""

from dataclasses import dataclass, field
import math
import time

from global_frontier_portal_crossing import portal_signed_distance, portal_section_destination


def _xy(value):
    try:
        point = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError):
        return None
    return point if all(math.isfinite(item) for item in point) else None


def _angle_delta(target, source):
    return math.atan2(math.sin(target - source), math.cos(target - source))


def destination_returns_into_source_place(
    node, destination_xy, source_place_id, region_memory=None,
    physical_xy=None, before_time=None,
):
    """Return whether a doorway's inside witness is a previously visited Place.

    The live free-space mask cannot answer this: an open doorway keeps the
    destination room connected to the source room.  The primary evidence is
    the robot's own visited path, filtered to poses recorded *before* this
    branch started.  The durable region identity is retained only as a legacy
    fallback when no visited history is available at all.
    """
    point = _xy(destination_xy)
    try:
        source_place_id = int(source_place_id)
    except (TypeError, ValueError):
        return False
    if point is None or source_place_id <= 0:
        return False
    if region_memory is not None:
        try:
            if region_memory.has_visited_path(source_place_id, frame="odom"):
                return bool(region_memory.destination_was_visited(
                    source_place_id,
                    point,
                    before_time=before_time,
                    radius=0.9,
                    frame="odom",
                ))
        except (AttributeError, TypeError, ValueError):
            pass
        # Compatibility for an old Place record with no pose history.  Do not
        # use the live mask: that is the connectivity fact that over-rejects.
        try:
            _tier, region = region_memory.candidate_tier(
                point[0], point[1], 0.0,
                component=None,
                physical_xy=physical_xy,
            )
        except (AttributeError, TypeError, ValueError):
            region = None
        try:
            if region is not None and int(region.get("id", -1)) == source_place_id:
                return True
        except (TypeError, ValueError):
            pass
    return False


@dataclass
class DoorwayAuthentication:
    """One stable GP branch and its enter, inspect, turn, and return record."""

    branch_id: int
    node_id: int
    gate_xy: tuple
    destination_xy: tuple
    required_depth: float = 0.40
    return_radius: float = 0.75
    return_goal_margin: float = 0.60
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
    return_gate_cleared: bool = False
    events: list = field(default_factory=list)
    entry_goal_margin: float = 0.60
    # Provisional while pending; immutable measured section once entry begins.
    gate_geometry: tuple = None

    def __post_init__(self):
        self.branch_id = int(self.branch_id)
        self.node_id = int(self.node_id)
        self.gate_xy = _xy(self.gate_xy)
        self.destination_xy = _xy(self.destination_xy)
        self.required_depth = max(0.0, float(self.required_depth))
        self.entry_goal_margin = max(0.0, float(self.entry_goal_margin))
        self.return_radius = max(0.05, float(self.return_radius))
        self.return_goal_margin = max(0.0, float(self.return_goal_margin))
        if self.gate_geometry is not None:
            self.refresh_pending_geometry(self.gate_xy, self.destination_xy, *self.gate_geometry)
        if self.gate_xy is None or self.destination_xy is None:
            self.fail("invalid_gate_or_destination")

    def refresh_pending_geometry(self, gate, destination, width, bounds):
        """Follow new SLAM evidence before entry, then freeze one coherent section."""
        if self.state != 'pending':
            return False
        gate, destination = _xy(gate), _xy(destination)
        if gate is None or destination is None:
            return False
        try:
            width=float(width)
            if not math.isfinite(width) or width<=0:width=None
        except (TypeError,ValueError):width=None
        points=None
        if bounds is not None and len(bounds)==2:
            projected=tuple(_xy(point) for point in bounds)
            if all(point is not None for point in projected):points=projected
        self.gate_xy,self.destination_xy=gate,destination
        self.gate_geometry=(width,points)
        return True

    def note_action_running(self, running, now):
        """Record whether the controller has actually taken over this phase.

        The physical-progress window must not judge a doorway while the
        manoeuvre has not started: a route-boundary handoff can hold a freshly
        dispatched route for tens of seconds with the robot stationary, and the
        same manoeuvre still succeeds afterwards (env04 20261112_231045
        branch 15: the turn supervisor stayed PASS_THROUGH for 28 s and the
        doorway was condemned 24 s in).
        """
        key = str(self.state)
        if getattr(self, "_action_phase", None) != key:
            self._action_phase = key
            self._phase_action_running = False
            self._phase_handoff_since = None
        running = bool(running)
        if running and not getattr(self, "_phase_action_running", False):
            self._event("doorway_action_started", phase=key)
        self._phase_action_running = running
        if running:
            self._phase_handoff_since = None
        elif getattr(self, "_phase_handoff_since", None) is None:
            self._phase_handoff_since = float(now)
        return running

    def rearm_physical_progress(self, pose_xy, yaw, now):
        """Restart the motion window without writing a phase verdict."""
        pose = _xy(pose_xy)
        if pose is None:
            return False
        self._progress_pose = pose
        self._progress_yaw = float(yaw)
        self._progress_at = float(now)
        return True

    def reset_for_retry(self, now, cooldown=0.0):
        """Return one attempt to ``pending`` without losing its identity.

        Only the per-attempt state machine is cleared; the measured gate
        section and the durable branch identity stay untouched so a retry is
        the same physical doorway, not a new one.
        """
        self.state = "pending"
        self.failure_reason = None
        self.source_xy = None
        self.inside_xy = None
        self.initial_yaw = None
        self.target_yaw = None
        self.chosen_turn_side = None
        self.turn_direction = ""
        self.turn_stable_samples = 0
        self.rotation_ready = False
        self.frontier_count_left = 0
        self.frontier_count_right = 0
        self.return_gate_cleared = False
        self.return_retry_count = 0
        self.return_recovery_started = None
        self.return_recovery_blocked = False
        self.return_recovery_blocked_at = None
        self.return_recovery_final_blocked = False
        self._progress_phase = None
        self._progress_pose = None
        self._progress_at = 0.0
        self._action_phase = None
        self._phase_action_running = False
        self._phase_handoff_since = None
        self.retry_not_before = float(now) + max(0.0, float(cooldown))
        self._event("doorway_authentication_retry", now=float(now),
                    cooldown=float(cooldown))
        return True

    def _event(self, name, **data):
        event = {"event": name, "branch_id": self.branch_id,
                 "node_id": self.node_id}
        event.update(data)
        self.events.append(event)
        return event

    def _remember_failed_return_fallback(self, pose_xy=None):
        goal = _xy(getattr(self, "last_source_side_return_goal_odom", None))
        if goal is None:
            return
        current = _xy(pose_xy)
        source = _xy(self.source_xy)
        history = list(getattr(self, "failed_source_side_return_goals_odom", ()))
        distance = None if current is None else math.dist(current, goal)
        protected_source = source is not None and math.dist(source, goal) <= self.return_radius
        # A stalled approach does not prove the destination is blocked. Never
        # erase the small source-return disk because a remote controller leg
        # failed; retain it for a new bounded approach.
        if (not protected_source and distance is not None and distance < 0.80
                and not any(math.dist(goal, point) < 0.20 for point in history)):
            history.append(goal)
        self.failed_source_side_return_goals_odom = history[-16:]
        if current is None or distance < 0.80:
            return
        dx, dy = goal[0] - current[0], goal[1] - current[1]
        marker = (current[0] + 0.45 * dx / distance,
                  current[1] + 0.45 * dy / distance)
        obstacles = list(getattr(
            self, "failed_return_approach_obstacles_odom", (),
        ))
        if any(math.hypot(marker[0] - point[0], marker[1] - point[1]) < 0.25
               for point in obstacles if _xy(point) is not None):
            return
        obstacles.append(marker)
        self.failed_return_approach_obstacles_odom = obstacles[-12:]
        self._event(
            "doorway_return_stall_approach_marked",
            pose_xy=list(current), failed_goal_xy=list(goal),
            obstacle_marker_xy=list(marker),
        )

    def _entry_heading(self):
        destination = portal_section_destination(
            self.gate_xy, self.destination_xy,
            None if self.gate_geometry is None else self.gate_geometry[1])
        if destination is None:
            return None
        return math.atan2(destination[1]-self.gate_xy[1],
                          destination[0]-self.gate_xy[0])

    def _physical_motion_progressed(self, pose, yaw):
        reference = getattr(self, "_progress_pose", None)
        if reference is None:
            return False
        if self.state == "returning" and self.source_xy is not None:
            distance = math.dist(pose, self.source_xy)
            return getattr(self, "_return_best_source_distance", distance) - distance >= 0.10
        if self.state == "turning":
            before = abs(_angle_delta(self.target_yaw, self._progress_yaw))
            after = abs(_angle_delta(self.target_yaw, float(yaw)))
            return before - after >= 0.08
        translated = math.hypot(pose[0]-reference[0], pose[1]-reference[1]) >= 0.10
        if translated or self.state != "entering":
            return translated
        # Entering may first require a real half-turn toward the frozen gate.
        # Credit only improvement toward its immutable section normal, using
        # the same angular threshold as the explicit authentication turn.
        # Rotation away or oscillation back to the previous best cannot renew
        # the budget; lease/map changes never supply progress.
        heading = self._entry_heading()
        if heading is None:
            return False
        before = abs(_angle_delta(heading, self._progress_yaw))
        after = abs(_angle_delta(heading, float(yaw)))
        return before - after >= 0.08

    def physical_progress_due(self, pose_xy, yaw, now, timeout):
        """Read-only cooperative deadline check; the owner performs settlement."""
        pose = _xy(pose_xy)
        if (pose is None or self.state not in ("entering", "inside", "turning", "returning")
                or getattr(self, "_progress_phase", None) != self.state
                or getattr(self, "_progress_pose", None) is None
                or getattr(self, "return_recovery_final_blocked", False)):
            return False
        return (not self._physical_motion_progressed(pose, yaw)
                and max(0.0, float(now)-self._progress_at) >= float(timeout))

    def physical_progress_expired(self, pose_xy, yaw, now, timeout):
        """Track phase motion across controller lease renewals and terminals."""
        pose = _xy(pose_xy)
        if pose is None or self.state not in ("entering", "inside", "turning", "returning"):
            return False
        if (self.state == "returning"
                and getattr(self, "return_recovery_final_blocked", False)):
            return False
        now = float(now)
        key = str(self.state)
        reference = getattr(self, "_progress_pose", None)
        if getattr(self, "_progress_phase", None) != key or reference is None:
            self._progress_phase = key
            self._progress_pose = pose
            self._progress_yaw = float(yaw)
            self._progress_at = now
            if key == "returning" and self.source_xy is not None:
                self._return_best_source_distance = math.dist(pose, self.source_xy)
            return False
        progressed = self._physical_motion_progressed(pose, yaw)
        if progressed and key == "returning" and self.source_xy is not None:
            self._return_best_source_distance = math.dist(pose, self.source_xy)
        if progressed:
            if key == "entering" and math.dist(pose, reference) < 0.10:
                self._event("doorway_physical_progress_observed", phase=key,
                            kind="entry_alignment", pose_xy=list(pose),
                            heading_yaw=self._entry_heading(),
                            previous_yaw=self._progress_yaw, yaw=float(yaw))
            self._progress_pose = pose
            self._progress_yaw = float(yaw)
            self._progress_at = now
            return False
        elapsed = max(0.0, now - self._progress_at)
        if elapsed < float(timeout):
            return False
        self._event("doorway_physical_progress_expired", phase=key,
                    elapsed=round(elapsed, 3), timeout=float(timeout),
                    pose_xy=list(pose), source_xy=list(self.source_xy),
                    source_distance=round(math.hypot(pose[0]-self.source_xy[0],
                                                   pose[1]-self.source_xy[1]), 3))
        if key == "returning":
            self._remember_failed_return_fallback(pose)
            # A stalled return is still an outstanding safety obligation.
            # Re-arm a bounded progress window so the route can be retried,
            # but never turn this into a terminal branch failure here.
            self._progress_pose = pose
            self._progress_yaw = float(yaw)
            self._progress_at = now
            retry_count = int(getattr(self, "return_retry_count", 0)) + 1
            self.return_retry_count = retry_count
            self._event("doorway_failure_return_retry",
                        retry_count=retry_count,
                        reason=self.failure_reason or "return_progress_stalled",
                        source_distance=round(math.hypot(
                            pose[0]-self.source_xy[0], pose[1]-self.source_xy[1]), 3))
            if getattr(self, "return_recovery_started", None) is None:
                self.return_recovery_started = float(now)
            recovery_elapsed = max(
                0.0, float(now) - float(self.return_recovery_started))
            if return_recovery_should_block(
                    retry_count, recovery_elapsed, 3,
                    float(getattr(self, "return_recovery_min_seconds", 45.0))):
                if getattr(self, "return_recovery_blocked", False) is not True:
                    # Wall-clock start of the blocked window.  The owner uses
                    # it to settle a return that can never be dispatched
                    # instead of waiting for the 140 s budget, which the 300 s
                    # stationary watchdog can pre-empt (env04 south corridor).
                    self.return_recovery_blocked_at = time.monotonic()
                self.return_recovery_blocked = True
                self._event("doorway_failure_return_recovery_blocked",
                            retry_count=retry_count,
                            recovery_elapsed=round(recovery_elapsed, 1),
                            reason="bounded_source_side_return_retries_exhausted")
            if return_recovery_should_block(
                    retry_count, recovery_elapsed, 6,
                    float(getattr(self, "return_recovery_final_seconds", 140.0))):
                self.return_recovery_final_blocked = True
                self._event("doorway_failure_return_finally_blocked",
                            retry_count=retry_count,
                            recovery_elapsed=round(recovery_elapsed, 1),
                            reason="source_side_recovery_budget_exhausted")
        return True

    def note_return_route_failure(self, reason):
        """Keep the source-return obligation alive and request a fresh fallback."""
        if self.state != "returning":
            return False
        reason = str(reason or "controller_failure")
        self._remember_failed_return_fallback(
            getattr(self, "_progress_pose", None),
        )
        if not self.failure_reason:
            self.failure_reason = "navigation_route_failed:%s" % reason
        failures = int(getattr(self, "return_controller_failure_count", 0)) + 1
        self.return_controller_failure_count = failures
        self.return_recovery_blocked = True
        self.return_recovery_blocked_reported = False
        self._event("doorway_return_controller_failure",
                    reason=reason, retry_count=failures,
                    source_xy=list(self.source_xy or ()))
        if failures >= 4:
            self.return_recovery_final_blocked = True
            self._event("doorway_failure_return_finally_blocked",
                        retry_count=failures,
                        reason="source_side_controller_budget_exhausted")
        return True

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
        return portal_signed_distance(
            pose, self.gate_xy, self.destination_xy,
            None if self.gate_geometry is None else self.gate_geometry[1],
        )

    def route_goal(self):
        """Return XY for entry/return; turning holds XY and commands target yaw."""
        destination = portal_section_destination(
            self.gate_xy, self.destination_xy,
            None if self.gate_geometry is None else self.gate_geometry[1],
        )
        if destination is None:
            return None
        if self.state == "entering":
            dx = destination[0] - self.gate_xy[0]
            dy = destination[1] - self.gate_xy[1]
            length = math.hypot(dx, dy) or 1.0
            # Leave enough distance for the controller's endpoint tolerance
            # after the full robot footprint has crossed the gate plane.
            goal_depth = self.required_depth + self.entry_goal_margin
            return (self.gate_xy[0] + dx * goal_depth / length,
                    self.gate_xy[1] + dy * goal_depth / length)
        if self.state == "turning":
            return self.inside_xy
        if self.state == "returning":
            if self.failure_reason:
                return self.source_xy
            dx = destination[0] - self.gate_xy[0]
            dy = destination[1] - self.gate_xy[1]
            length = math.hypot(dx, dy) or 1.0
            margin = self.return_goal_margin
            # A source pose can be close to the doorway plane (or, after a
            # topology/SLAM correction, slightly on the destination side).
            # Merely offsetting from that pose can leave the goal on the plane;
            # TEB then stops within endpoint tolerance without the robot center
            # actually crossing back. Aim at least this far onto the source
            # side while preserving the original retreat from the source pose.
            source_depth = self.signed_depth(self.source_xy)
            # Once the robot center has crossed back to the source side,
            # finish the return at the actual junction node.  This also covers
            # SLAM placing the node slightly behind the estimated gate plane;
            # repeatedly targeting a margin-offset waypoint would otherwise
            # leave the branch returning forever.
            if self.return_gate_cleared:
                return self.source_xy
            # If SLAM places the estimated gate beyond the junction node, first
            # clear its plane and then explicitly drive back to the source node.
            # Otherwise the plane-clearing waypoint can stop short forever.
            if source_depth is not None and source_depth >= -0.05:
                goal_depth = -margin
            else:
                goal_depth = min(source_depth - margin, -margin)
            return (self.gate_xy[0] + dx * goal_depth / length,
                    self.gate_xy[1] + dy * goal_depth / length)
        return None

    def source_side_depth(self):
        """Signed source-node distance from the directed doorway plane."""
        return self.signed_depth(self.source_xy)

    def source_side_proven(self, margin=0.05):
        depth = self.source_side_depth()
        return depth is not None and depth < -max(0.0, float(margin))

    def reject_completed_portal_candidate(self, reason):
        """Reject a traversable gap whose gate is not beyond the source node."""
        if self.state != "completed":
            return False
        self.state = "failed"
        self.failure_reason = str(reason or "portal_geometry_unproven")
        self._event(
            "portal_candidate_rejected",
            reason=self.failure_reason,
            source_depth=(
                None if self.source_side_depth() is None
                else round(self.source_side_depth(), 3)
            ),
        )
        return True

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
        # A radius-only test can call a robot returned while it is still on
        # the destination side of the doorway plane. Require the robot center
        # to cross back onto the source side before closing this observation.
        source_depth = self.signed_depth(pose)
        node_depth = self.signed_depth(self.source_xy)
        if source_depth is not None and source_depth < -0.05:
            # Failure returns need the same physical gate-crossing proof as
            # successful observations. Being close to the source coordinate
            # while still inside the Portal is not a completed recovery.
            self.return_gate_cleared = True
        if distance > self.return_radius:
            return False
        if self.failure_reason:
            if not self.return_gate_cleared:
                return False
        else:
            if node_depth is None or source_depth is None:
                return False
            if node_depth < -0.05 and source_depth >= -0.05:
                return False
            if node_depth >= -0.05 and not self.return_gate_cleared:
                if source_depth >= -0.05:
                    return False
                self.return_gate_cleared = True
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
            depth = self.signed_depth(pose)
            if (distance > self.return_radius
                    or (not self.return_gate_cleared and (depth is None or depth >= -0.05))):
                self.state = "returning"
                self._event("doorway_failure_return_started",
                            reason=self.failure_reason,
                            from_xy=list(pose), source_xy=list(self.source_xy),
                            distance=round(distance, 3))
                return True
            self._event("doorway_failure_settled_at_source",
                        reason=self.failure_reason, phase=self.state,
                        source_xy=list(self.source_xy), pose_xy=list(pose),
                        distance=round(distance, 3), signed_depth=depth)
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
            "required_depth": self.required_depth,
            "opening_width": None if self.gate_geometry is None else self.gate_geometry[0],
            "physical_bounds": None if self.gate_geometry is None else self.gate_geometry[1],
            "frontier_count_left": self.frontier_count_left,
            "frontier_count_right": self.frontier_count_right,
            "chosen_turn_side": self.chosen_turn_side,
            "turn_direction": self.turn_direction,
            "initial_yaw": self.initial_yaw, "target_yaw": self.target_yaw,
            "rotation_ready": self.rotation_ready,
            "source_side_depth": self.source_side_depth(),
            "source_side_proven": self.source_side_proven(),
            "return_gate_cleared": self.return_gate_cleared,
            "failure_reason": self.failure_reason, "events": list(self.events),
        }


def return_recovery_should_block(retry_count, elapsed, minimum_count, minimum_seconds):
    """A stalled return needs both a retry count and a real recovery window.

    Run 20261103_210032 (L attempt 1) dispatched source-side returns from
    sim 1171.8: the local costmap still carried a transient obstacle blob over
    the source (cost 99 at -18.45,-1.35 while /map was free), so Navfn returned
    an empty plan and TEB commanded zero.  Six retries ~5 s apart exhausted the
    budget at ~30 s; the local costmap cleared at ~1220 and the global one at
    ~1280, so a later attempt would have driven home.  Keep the count bounds
    for reporting but require the physical window before giving up.
    """
    try:
        return (int(retry_count) >= max(1, int(minimum_count))
                and float(elapsed) >= max(0.0, float(minimum_seconds)))
    except (TypeError, ValueError):
        return False


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
        # First observation time with no runnable branch.  ``watching`` used to
        # have no deadline: a node whose branches could never be entered kept
        # the ordinary route retired forever (run 20261031_165652, node 5).
        self.watching_since = None

    def expire_unstartable(self, now, timeout):
        """Conclude an observation period that can never start a branch.

        Returns the branch ids that were concluded.  ``watching`` is only
        allowed to persist while a branch is actually startable; once none has
        been startable for ``timeout`` seconds the remaining branches are
        failed and the task completes, so the caller stops retiring the
        ordinary route and the graph can dispatch again.
        """
        if self.active_branch_id is not None:
            self.watching_since = None
            return []
        if self.all_finished():
            self.watching_since = None
            return []
        now = float(now)
        if self.watching_since is None:
            self.watching_since = now
            return []
        if now - self.watching_since < max(0.0, float(timeout)):
            return []
        expired = []
        for branch in list(self.branches.values()):
            if branch.state == "pending":
                if self.withdraw_pending(branch.branch_id, now=now):
                    expired.append(int(branch.branch_id))
            elif branch.state not in ("completed", "failed"):
                branch.state = "failed"
                branch.failure_reason = "observation_geometry_unavailable"
                branch._event("doorway_authentication_failed",
                              reason="observation_geometry_unavailable")
                expired.append(int(branch.branch_id))
        self.active_branch_id = None
        self.phase = "complete"
        self.watching_since = None
        self.signature_since = now
        self.signature = tuple(sorted(self.branches))
        return expired

    def add(self, branch, now=0.0):
        if int(branch.node_id) != self.node_id:
            raise ValueError("branch belongs to another node")
        branch_id = int(branch.branch_id)
        if branch_id in self.branches:
            return False
        self.branches[branch_id] = branch
        self.signature = tuple(sorted(self.branches))
        self.signature_since = float(now)
        # A late stable arm must be queued even while another arm is being
        # authenticated.  The active arm keeps its lease and phase; the new
        # branch is picked up by start_next after the settle window.  Refusing
        # the add here made a branch that became measurable one update later
        # remain attempts=0 and be pruned as place_id=None when the junction
        # completed (run 20261106_011314, node 2 branch 5).
        if self.active_branch_id is None:
            self.phase = "watching"
        return True

    def withdraw_pending(self, branch_id, now=0.0):
        """Withdraw changed map evidence without cancelling a physical return."""
        branch_id = int(branch_id)
        branch = self.branches.get(branch_id)
        if (branch is None or branch.state != "pending"
                or self.active_branch_id == branch_id):
            return False
        del self.branches[branch_id]
        self.signature = tuple(sorted(self.branches))
        self.signature_since = float(now)
        if self.active_branch_id is None:
            self.phase = "complete" if self.all_finished() else "watching"
        return True

    def next_pending(self, now=None):
        for key in sorted(self.branches):
            branch = self.branches[key]
            if branch.state != "pending":
                continue
            if (now is not None
                    and float(getattr(branch, "retry_not_before", 0.0) or 0.0)
                    > float(now)):
                continue
            return branch
        return None

    def start_next(self, source_xy, now=0.0, force=False):
        if not force and float(now) - self.signature_since < self.settle_seconds:
            self.phase = "watching"
            return None
        branch = self.next_pending(now)
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

    def retry_active(self, source_xy, now=0.0, cooldown=0.0):
        """Return the active attempt to ``pending`` instead of settling it.

        The junction observation period stays open, so a retried doorway is
        re-attempted before any room ranking is committed.
        """
        branch = self.branches.get(self.active_branch_id)
        if branch is None:
            return None
        self.active_branch_id = None
        branch.reset_for_retry(now, cooldown)
        self.signature = tuple(sorted(self.branches))
        self.signature_since = float(now)
        self.phase = "watching"
        self.watching_since = None
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


def resolve_certified_probe_work_items(probe_ledger, work_item_ledger, portal_id, now, reason):
    """Retire source frontier WIs now superseded by a certified Portal.

    Certification transfers ownership of the opening from its generic
    frontier-observation WI to the semantic Portal-entry WI (or suppresses it
    when the room has a negative semantic score). Leaving the old WI unresolved
    makes the current Place look like it still has an unmaterialized obligation.
    """
    try:
        portal_id = int(portal_id)
    except (TypeError, ValueError):
        return []
    if portal_id <= 0 or probe_ledger is None or work_item_ledger is None:
        return []
    try:
        probes = probe_ledger.snapshot()
        items = work_item_ledger.snapshot()
    except (AttributeError, TypeError):
        return []
    item_states = {int(item.get("id", -1)): item.get("state") for item in items}
    resolved = []
    for probe in probes:
        try:
            matches = int(probe.get("portal_id", -1)) == portal_id
            work_item_id = int(probe.get("work_item_id"))
        except (AttributeError, TypeError, ValueError):
            continue
        if not matches or item_states.get(work_item_id) in (None, "resolved"):
            continue
        attempt_id = work_item_ledger.active_attempt_id(work_item_id)
        item = work_item_ledger.resolve(
            work_item_id, now, reason, attempt_id=attempt_id,
        )
        if item is not None and item.get("state") == "resolved":
            resolved.append(work_item_id)
    return resolved


class GlobalFrontierPortalAuthenticationMixin:
    """Runtime adapter joining the authentication ledger to explorer events."""

    # Only execution failures are retryable. A verdict about the doorway
    # itself (nothing inside, not passable, known space on the far side) stays
    # a permanent rejection so a fake doorway is never re-flown.
    AUTH_RETRYABLE_REASONS = (
        "authentication_physical_stall:",
        "authentication_handoff_stall:",
        "return_unrecoverable",
        "zero_information_dead_end",
    )

    def portal_authentication_retry_reason(self, branch, tree_branch=None):
        """Return the reason when one doorway attempt may be retried."""
        reason = str(getattr(branch, "failure_reason", "") or "")
        if not reason:
            return None
        if not any(reason.startswith(prefix)
                   for prefix in self.AUTH_RETRYABLE_REASONS):
            return None
        try:
            limit = int(getattr(self, "portal_authentication_retry_limit", 0) or 0)
        except (TypeError, ValueError):
            limit = 0
        if limit <= 0:
            return None
        attempts = 0
        if isinstance(tree_branch, dict):
            try:
                attempts = int(
                    tree_branch.get("authentication_retry_count", 0) or 0)
            except (TypeError, ValueError):
                attempts = 0
        return reason if attempts < limit else None

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
        if branch.state == "completed" and not branch.source_side_proven():
            branch.reject_completed_portal_candidate(
                "portal_source_side_geometry_unproven",
            )
            self.publish_status(
                "portal_candidate_rejected",
                node_id=int(branch.node_id), branch_id=int(branch.branch_id),
                reason=branch.failure_reason,
                source_depth=branch.source_side_depth(),
            )
        result = branch.snapshot()
        self.portal_authentication_history.append(result)
        tree = getattr(self, "gp_exploration_tree", None)
        tree_branch = None if tree is None else tree.branches.get(branch.branch_id)
        retry_failure = None
        if tree_branch is not None:
            tree_branch["authentication_state"] = branch.state
            tree_branch["authentication_failure_reason"] = branch.failure_reason
            tree_branch["authentication_evidence"] = result
            portal_id = tree_branch.get("canonical_portal_id")
            portal_ledger = getattr(self, "portal_hypothesis_ledger", None)
            portal = None if portal_ledger is None or portal_id is None else portal_ledger.get(portal_id)
            destination_region = None
            if portal is not None and branch.state == "completed":
                portal["gp_authentication_state"] = "completed"
                portal["gp_authentication_node_id"] = int(task.node_id)
                portal["gp_authentication_branch_id"] = int(branch.branch_id)
                portal["gp_authentication_evidence"] = result
                if portal.get("destination_place_id") is None:
                    region_memory = getattr(self, "region_memory", None)
                    map_goal = portal.get("map_destination")
                    if (
                        region_memory is not None
                        and isinstance(map_goal, (list, tuple))
                        and len(map_goal) >= 2
                    ):
                        destination_region, _tier = (
                            region_memory.enter_new_portal_destination(
                                float(map_goal[0]), float(map_goal[1]), now,
                                component=None,
                                entry_portal=portal.get("map_gate"),
                                physical_xy=portal.get("physical_destination"),
                                physical_entry_portal=portal.get("physical_gate"),
                                source_place_id=portal.get("source_place_id"),
                            )
                        )
                        if destination_region is not None:
                            portal["destination_place_id"] = int(
                                destination_region["id"]
                            )
                            destination_region["observed_by_portal_authentication"] = True
                            destination_region["last_reason"] = (
                                "observed_portal_destination_place"
                            )
                            self.publish_status(
                                "portal_destination_place_created",
                                portal_id=int(portal_id),
                                place_id=int(destination_region["id"]),
                                gate=portal.get("map_gate"),
                                inside=portal.get("map_destination"),
                                source_place_id=portal.get("source_place_id"),
                            )
            observation_tree = getattr(self, "gp_observation_tree", None)
            if observation_tree is not None:
                room = observation_tree.finish_room(
                    task.node_id, branch.branch_id,
                    valid=(branch.state == "completed"),
                )
                if room is not None:
                    if destination_region is not None:
                        destination_region["semantic_score"] = room.get(
                            "semantic_score"
                        )
                        destination_region["semantic_priority"] = room.get(
                            "priority"
                        )
                        destination_region["labels"] = list(
                            room.get("labels", [])
                        )
                        room["place_id"] = int(destination_region["id"])
                    tree_branch["room_observation"] = {key: value for key, value in room.items() if key != "frames"}
                    tree_branch["semantic_score"] = room["semantic_score"]
                    tree_branch["room_semantic_score"] = room["semantic_score"]
                    tree_branch["room_priority"] = room.get("priority")
                    tree_branch["room_invalidated"] = room["invalidated"]
                    tree_branch["room_place_id"] = room.get("place_id")
                    tree_branch["room_gp_space_id"] = tree_branch.get("destination_space_id")
                    tree_branch["room_labels"] = list(room.get("labels", []))
            if branch.state == "failed":
                # A manoeuvre that failed to *execute* keeps its doorway
                # identity and is retried a bounded number of times.  A verdict
                # about the doorway itself is rejected immediately.
                retry_failure = self.portal_authentication_retry_reason(
                    branch, tree_branch)
                if retry_failure is None:
                    tree.reject_branch_authentication(branch.branch_id,
                                                      branch.failure_reason)
                else:
                    try:
                        tree_branch["authentication_retry_count"] = int(
                            tree_branch.get("authentication_retry_count", 0)
                            or 0) + 1
                    except (TypeError, ValueError):
                        tree_branch["authentication_retry_count"] = 1
            elif branch.state == "completed":
                tree_branch["state"] = "authenticated"
                tree_branch["authentication_state"] = "completed"
        node = None if tree is None else tree.nodes.get(task.node_id)
        if node is not None and branch.state in ("completed", "failed"):
            tree.current_node_id = task.node_id
        source_xy = (node.get("physical_xy") if node is not None
                     else branch.source_xy)
        if retry_failure is not None:
            next_branch = task.retry_active(
                source_xy, now=now,
                cooldown=float(getattr(
                    self, "portal_authentication_retry_cooldown", 5.0) or 0.0),
            )
            if tree_branch is not None:
                tree_branch["state"] = "retry"
                tree_branch["authentication_state"] = "retry"
                tree_branch["authentication_failure_reason"] = retry_failure
            self.publish_status(
                "portal_authentication_branch_retry",
                node_id=int(task.node_id),
                branch_id=int(branch.branch_id),
                reason=str(retry_failure),
                attempt=int(tree_branch.get("authentication_retry_count", 0) or 0)
                if tree_branch is not None else 0,
                limit=int(getattr(
                    self, "portal_authentication_retry_limit", 2) or 0),
                cooldown_seconds=round(float(getattr(
                    self, "portal_authentication_retry_cooldown", 5.0) or 0.0), 2),
            )
        else:
            next_branch = task.finish_active(source_xy, now=now)
        observation_tree = getattr(self, "gp_observation_tree", None)
        if next_branch is None and task.all_finished() and observation_tree is not None:
            ranked = observation_tree.finish_junction(task.node_id)
            if tree is not None:
                top_entry_created = False
                for room in ranked:
                    item = tree.branches.get(int(room["branch_id"]))
                    if item is not None:
                        item["room_priority"] = room["priority"]
                        item["room_invalidated"] = room["invalidated"]
                        item["room_state"] = room["state"]
                        item["room_observation"] = {key: value for key, value in room.items() if key != "frames"}
                        portal_id = item.get("canonical_portal_id")
                        ledger = getattr(self, "portal_hypothesis_ledger", None)
                        portal = None if ledger is None or portal_id is None else ledger.get(portal_id)
                        if portal is not None:
                            portal["semantic_priority"] = room.get("priority")
                            portal["semantic_score"] = room.get("semantic_score")
                            portal["exploration_suppressed"] = bool(
                                room.get("invalidated") or room.get("exploration_suppressed")
                            )
                            # Establish the observed destination Place now,
                            # before any crossing.  The GP observation already
                            # entered the room and produced a semantic score;
                            # waiting for a later portal transaction leaves the
                            # Place graph with only the source Place and makes
                            # the semantic entry unprojectable.
                            if portal.get("destination_place_id") is None:
                                region_memory = getattr(self, "region_memory", None)
                                map_goal = (
                                    portal.get("map_destination")
                                    or portal.get("physical_destination")
                                )
                                destination_goal = (
                                    _xy(portal.get("physical_destination"))
                                    or _xy(map_goal)
                                )
                                source_place_id = portal.get("source_place_id")
                                if destination_returns_into_source_place(
                                    self,
                                    destination_goal,
                                    source_place_id,
                                    region_memory=region_memory,
                                    physical_xy=portal.get("physical_destination"),
                                    before_time=(
                                        None if item is None
                                        else item.get("first_seen")
                                    ),
                                ):
                                    # The doorway observed by GP leads back into
                                    # the Place the robot already occupies.  A
                                    # new destination Place here is a duplicate
                                    # (P3 == P1 in run 20261105_210329), so keep
                                    # the source identity and retire the room.
                                    try:
                                        source_place_id = int(source_place_id)
                                    except (TypeError, ValueError):
                                        source_place_id = None
                                    portal["destination_place_id"] = source_place_id
                                    portal["self_loop"] = True
                                    portal["exploration_suppressed"] = True
                                    room["exploration_suppressed"] = True
                                    room["place_id"] = source_place_id
                                    self.publish_status(
                                        "portal_destination_self_loop_suppressed",
                                        portal_id=int(portal_id),
                                        source_place_id=source_place_id,
                                        gate=(
                                            portal.get("physical_gate")
                                            or portal.get("map_gate")
                                        ),
                                        destination=(
                                            None if destination_goal is None
                                            else [round(float(v), 3) for v in destination_goal]
                                        ),
                                        semantic_score=room.get("semantic_score"),
                                        priority=room.get("priority"),
                                        reason="doorway_destination_inside_source_place",
                                    )
                                elif (
                                    region_memory is not None
                                    and isinstance(map_goal, (list, tuple))
                                    and len(map_goal) >= 2
                                ):
                                    destination_region, _tier = (
                                        region_memory.enter_new_portal_destination(
                                            float(map_goal[0]),
                                            float(map_goal[1]),
                                            now,
                                            component=None,
                                            entry_portal=(
                                                portal.get("map_gate")
                                                or portal.get("physical_gate")
                                            ),
                                            physical_xy=portal.get("physical_destination"),
                                            physical_entry_portal=portal.get("physical_gate"),
                                            source_place_id=portal.get("source_place_id"),
                                        )
                                    )
                                    if destination_region is not None:
                                        portal["destination_place_id"] = int(
                                            destination_region["id"]
                                        )
                                        destination_region["semantic_score"] = room.get("semantic_score")
                                        destination_region["semantic_priority"] = room.get("priority")
                                        destination_region["labels"] = list(room.get("labels", []))
                                        destination_region["observed_by_portal_authentication"] = True
                                        destination_region["last_reason"] = "observed_portal_destination_place"
                                        room["place_id"] = int(destination_region["id"])
                                        self.publish_status(
                                            "portal_destination_place_created",
                                            portal_id=int(portal_id),
                                            place_id=int(destination_region["id"]),
                                            gate=portal.get("map_gate"),
                                            inside=portal.get("map_destination"),
                                            source_place_id=portal.get("source_place_id"),
                                            semantic_score=room.get("semantic_score"),
                                            priority=room.get("priority"),
                                        )
                            if (
                                not portal["exploration_suppressed"]
                                and not top_entry_created
                            ):
                                # Only the highest-semantic Portal gets an active
                                # entry WorkItem.  Clear the source Place's local
                                # observation WIs first, otherwise they compete
                                # with the doorway entry and recreate the
                                # micro-route churn.
                                top_entry_created = True
                                entry_goal = (
                                    portal.get("physical_gate")
                                    or portal.get("map_gate")
                                    or portal.get("physical_destination")
                                )
                                source_place_id = portal.get("source_place_id")
                                item_ledger = getattr(self, "place_work_items", None)
                                pruned = []
                                if item_ledger is not None:
                                    prune = getattr(
                                        item_ledger,
                                        "prune_frontier_observations",
                                        None,
                                    )
                                    if callable(prune):
                                        pruned = prune(
                                            source_place_id, now,
                                            reason="semantic_portal_entry_selected",
                                        )
                                    defer_entries = getattr(
                                        item_ledger,
                                        "defer_portal_entries",
                                        None,
                                    )
                                    if callable(defer_entries):
                                        deferred_entries = defer_entries(
                                            source_place_id, portal_id, now,
                                            reason="lower_semantic_priority_deferred",
                                        )
                                        if deferred_entries:
                                            self.publish_status(
                                                "portal_lower_entries_deferred",
                                                node_id=int(task.node_id),
                                                portal_id=int(portal_id),
                                                work_item_ids=list(deferred_entries),
                                            )
                                if pruned:
                                    self.publish_status(
                                        "portal_source_work_items_pruned",
                                        node_id=int(task.node_id),
                                        place_id=int(source_place_id),
                                        work_item_ids=list(pruned),
                                        reason="semantic_portal_entry_selected",
                                    )
                                ensure_entry = getattr(
                                    item_ledger, "ensure_portal_entry", None
                                )
                                entry_id = (
                                    None if not callable(ensure_entry) else ensure_entry(
                                        portal_id, source_place_id, entry_goal,
                                        room.get("priority"), room.get("semantic_score"), now,
                                    )
                                )
                                if entry_id is not None:
                                    portal["semantic_entry_work_item_id"] = int(entry_id)
                                    room["work_item_ids"] = [int(entry_id)]
                                    tree_room = observation_tree.junctions.get(int(task.node_id), {}).get("rooms", {}).get(int(room["branch_id"]))
                                    if tree_room is not None:
                                        tree_room["work_item_ids"] = [int(entry_id)]
                                    self.publish_status(
                                        "portal_entry_work_item_created",
                                        node_id=int(task.node_id), branch_id=int(room["branch_id"]),
                                        portal_id=int(portal_id), work_item_id=int(entry_id),
                                        place_id=int(source_place_id), priority=int(room["priority"]),
                                        semantic_score=float(room["semantic_score"]), goal_xy=list(entry_goal),
                                    )
                            elif not portal["exploration_suppressed"]:
                                # Lower-scored rooms remain candidates in the
                                # Portal ledger, but never own an active WI.
                                portal["semantic_entry_work_item_id"] = None
                                room["work_item_ids"] = []
                            entry_ready = bool(
                                portal["exploration_suppressed"]
                                or portal.get("semantic_entry_work_item_id") is not None
                            )
                            if entry_ready:
                                item_ledger = getattr(self, "place_work_items", None)
                                probe_ledger = getattr(self, "portal_probe_ledger", None)
                                reason = (
                                    "negative_semantic_room_priority"
                                    if portal["exploration_suppressed"]
                                    else "portal_entry_work_item_created"
                                )
                                retired = resolve_certified_probe_work_items(
                                    probe_ledger, item_ledger, portal_id, now, reason,
                                )
                                if retired:
                                    room["superseded_work_item_ids"] = retired
                                    self.publish_status(
                                        "portal_probe_work_items_superseded",
                                        node_id=int(task.node_id),
                                        branch_id=int(room["branch_id"]),
                                        portal_id=int(portal_id),
                                        work_item_ids=retired,
                                        reason=reason,
                                    )
                            if portal["exploration_suppressed"]:
                                item_ledger = getattr(self, "place_work_items", None)
                                resolve_entry = getattr(
                                    item_ledger, "resolve_portal_entry", None,
                                )
                                if callable(resolve_entry):
                                    resolve_entry(
                                        portal_id, now,
                                        reason="negative_semantic_room_priority",
                                    )
                                room["work_item_ids"] = []
                            destination_id = portal.get("destination_place_id")
                            if portal["exploration_suppressed"] and destination_id is not None:
                                ledger_items = getattr(self, "place_work_items", None)
                                if ledger_items is not None:
                                    dismissed = ledger_items.close_place(
                                        destination_id, now,
                                        reason="negative_semantic_room_priority",
                                    )
                                    item["room_work_item_ids_dismissed"] = dismissed
            self.publish_status(
                "portal_rooms_prioritized", node_id=int(task.node_id),
                rooms=[{key: value for key, value in room.items() if key != "frames"} for room in ranked],
                observation_tree=observation_tree.snapshot(),
            )
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
        if task.phase == "complete" and task.all_finished():
            # The next snapshot may already belong to a child Place, whose
            # discovery deliberately skips this parent junction. Publish the
            # completed observation transaction here, without waiting for it.
            task.last_report_signature = (
                task.phase, tuple(sorted(task.branches)), task.active_branch_id,
                task.all_finished(),
            )
            self.publish_status("portal_authentication_observation_period",
                node_id=int(task.node_id), phase=task.phase,
                branch_ids=sorted(task.branches), active_branch_id=None,
                all_finished=True)
        return next_branch
