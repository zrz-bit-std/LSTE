"""Persistent lidar-grid and boundary-recovery safety guard for SA-PPO."""

import math
import time

import numpy as np
import rospy

from sappo_grid_boundary import GridBoundaryRecovery
from sappo_safety_primitives import DwaSafetyGuard, LocalGridPlanner, RollingObstacleMap


class GridSafetyGuard(GridBoundaryRecovery, DwaSafetyGuard):
    """Lidar-grid local planner used only for the fixed-goal controller test."""

    def __init__(self):
        super().__init__()
        # The fixed-goal experiment needs to retain the obstacle contour that
        # led to a wall endpoint.  A six-second, six-metre window forgets the
        # U-shaped barrier exactly when the robot reaches its exit, which makes
        # local A* repeatedly choose a different side of the same wall.
        self.obstacle_map = RollingObstacleMap(retention_seconds=90.0, max_range=10.0)
        self.grid_obstacle_radius = max(
            0.30, float(rospy.get_param("~grid_obstacle_radius", 0.48))
        )
        self.grid_edge_clearance = max(
            0.30, float(rospy.get_param("~grid_edge_clearance", 0.42))
        )
        self.grid_planner = LocalGridPlanner(
            half_extent=10.0, obstacle_radius=self.grid_obstacle_radius
        )
        self.boundary_active = False
        self.boundary_heading_world = 0.0
        self.boundary_hit_distance = float("inf")
        self.boundary_start = None
        self.goal_visibility_frames = 0
        # Once a boundary end is detected, keep the selected lateral side for
        # long enough to rotate and obtain a new scan. Re-selecting a side on
        # every control cycle can invert the command while the robot is still
        # turning, which produces an in-place left/right oscillation.
        self.boundary_turn_lock_until = 0.0
        self.boundary_turn_side = 0
        self.boundary_turn_lock_time = max(
            0.5, float(rospy.get_param("~boundary_turn_lock_time", 3.0))
        )
        # A blocked scan is not evidence that another full 90-degree turn is
        # needed.  The lidar can remain blocked while the robot is already
        # looking through the corner, so bound each turn by time/angle and
        # then let the wall-follow controller advance the contour.
        self.boundary_turn_max_duration = max(
            self.boundary_turn_lock_time,
            float(rospy.get_param("~boundary_turn_max_duration", 2.8)),
        )
        self.boundary_turn_max_angle = math.radians(max(
            70.0, float(rospy.get_param("~boundary_turn_max_angle_deg", 112.0))
        ))
        self.boundary_turn_max_attempts = max(
            1, int(rospy.get_param("~boundary_turn_max_attempts", 2))
        )
        self.boundary_turn_active = False
        self.boundary_turn_started_at = 0.0
        self.boundary_turn_start_yaw = 0.0
        self.boundary_turn_requested_angle = 0.0
        self.boundary_turn_attempts = 0
        self.boundary_turn_cooldown_until = 0.0
        self.boundary_reentry_cooldown = max(
            0.0, float(rospy.get_param("~boundary_reentry_cooldown", 1.5))
        )
        self.boundary_goal_cancel_angle = math.radians(max(
            120.0,
            float(rospy.get_param("~boundary_goal_cancel_angle_deg", 165.0)),
        ))
        self.boundary_goal_cancel_distance = max(
            0.45, float(rospy.get_param("~boundary_goal_cancel_distance", 0.75))
        )
        self.boundary_entry_blocked_until = 0.0
        # World-frame target at the moment Bug2 takes ownership.  A target
        # moving behind the robot is expected while tracing a wall; only a
        # genuinely different branch should invalidate this commitment.
        self.boundary_goal_world = None
        self.last_boundary_goal_delta = float("nan")
        self.boundary_entries = 0
        self.boundary_exits = 0
        self.boundary_turns = 0
        # A wall-follow commitment must have a bounded escape plan.  If the
        # distance to the same waypoint has not improved for a while and the
        # robot has already travelled away from the hit point, try the other
        # tangent once, then release ownership so A* can re-evaluate the map.
        self.boundary_progress_timeout = max(
            5.0, float(rospy.get_param("~boundary_progress_timeout", 18.0))
        )
        self.boundary_progress_margin = max(
            0.30, float(rospy.get_param("~boundary_progress_margin", 0.80))
        )
        self.boundary_best_goal_distance = float("inf")
        self.boundary_progress_last_time = 0.0
        self.boundary_side_switches = 0
        self.boundary_stuck_releases = 0
        # Wall-follow feedback.  ``boundary_turn_side`` is the direction of
        # the next turn; this separate sign identifies which side of the car
        # should remain beside the obstacle while driving along its contour.
        self.boundary_wall_side = 0
        self.boundary_wall_distance = max(
            0.45, float(rospy.get_param("~boundary_wall_distance", 0.72))
        )
        self.boundary_wall_max_range = max(
            self.boundary_wall_distance + 0.30,
            float(rospy.get_param("~boundary_wall_max_range", 1.80)),
        )
        self.boundary_wall_kp = max(
            0.05, float(rospy.get_param("~boundary_wall_kp", 0.45))
        )
        self.boundary_wall_heading_kp = max(
            0.0, float(rospy.get_param("~boundary_wall_heading_kp", 0.35))
        )
        self.boundary_wall_max_linear = float(np.clip(
            rospy.get_param("~boundary_wall_max_linear", 0.28), 0.10, 0.55
        ))
        self.boundary_wall_last_distance = float("nan")
        self.boundary_wall_last_front = float("nan")
        self.boundary_wall_last_rear = float("nan")
        self.boundary_wall_last_error = float("nan")
        self.boundary_wall_last_angular = 0.0
        self.boundary_wall_lost_frames = 0
        self.boundary_wall_filter_alpha = min(
            1.0, max(0.05, float(rospy.get_param("~boundary_wall_filter_alpha", 0.28)))
        )
        self.boundary_wall_angular_step = max(
            0.02, float(rospy.get_param("~boundary_wall_angular_step", 0.08))
        )
        self.boundary_wall_angular_deadband = max(
            0.0, float(rospy.get_param("~boundary_wall_angular_deadband", 0.035))
        )
        self.boundary_wall_filtered_distance = float("nan")
        self.boundary_wall_filtered_front = float("nan")
        self.boundary_wall_filtered_rear = float("nan")
        self.boundary_wall_front_missing_frames = 0
        self.boundary_wall_rear_missing_frames = 0
        self.boundary_forward_blocked_frames = 0
        self.boundary_turn_heading_tolerance = math.radians(10.0)
        # Goal-facing rotation uses hysteresis: visual and frontier waypoints
        # can hover near a wall-following turn threshold while the vehicle is
        # turning, so a single threshold would repeatedly hand control back to
        # Bug2 before the vehicle is actually facing the new route.
        self.reorienting_goal = False
        self.reorienting_turn_sign = 0
        self.reorienting_aligned_since = None
        self.reorient_entries = 0
        self.reorient_exits = 0
        # A boundary tangent and a goal-facing turn are mutually exclusive.
        # Keep a small hysteresis band and dwell at the aligned heading so a
        # 10 Hz controller cannot alternate between the two modes when a
        # waypoint sits near the side of the robot.
        self.reorient_enter = math.radians(55.0)
        self.reorient_exit = math.radians(25.0)
        self.reorient_exit_dwell = 0.8
        self.reorient_obstacle_clearance = max(
            0.45, float(rospy.get_param("~reorient_obstacle_clearance", 0.72))
        )
        self.last_point_count = 0
        self.last_forward_clearance = float("nan")
        self.last_waypoint = None
        self.stable_waypoint_world = None
        self.waypoint_hold_radius = max(
            0.20, float(rospy.get_param("~waypoint_hold_radius", 0.40))
        )
        self.waypoint_switch_distance = max(
            self.waypoint_hold_radius,
            float(rospy.get_param("~waypoint_switch_distance", 0.75)),
        )
        self.waypoint_hold_count = 0
        self.waypoint_switch_count = 0
        # A rolling grid can report no path for one or two frames while a scan
        # is integrated.  Keep a deterministic recovery action through that
        # interval instead of invoking the policy/DWA fallback, whose best
        # candidate can alternate between stop and turn commands.
        self.no_path_active = False
        self.no_path_started_at = 0.0
        self.no_path_turn_sign = 0
        self.no_path_entries = 0
        self.no_path_holds = 0
        # Require a valid A* route for a few consecutive frames before leaving
        # a wall contour.  This prevents a single optimistic scan from
        # handing control back to A* and immediately re-entering Bug2.
        self.boundary_astar_clear_frames = 0
        self.boundary_astar_rejoins = 0
        self.last_goal_world = None
        self.forward_clearance_filtered = float("nan")

    @staticmethod
    def _local_to_world(point, pose):
        """Transform a controller-local point into the odom/world frame."""
        x, y, yaw = (float(value) for value in pose)
        local_x, local_y = float(point[0]), float(point[1])
        return np.array([
            x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
        ], dtype=np.float32)

    @staticmethod
    def _world_to_local(point, pose):
        """Transform an odom/world point into the current robot frame."""
        x, y, yaw = (float(value) for value in pose)
        dx, dy = float(point[0]) - x, float(point[1]) - y
        return np.array([
            math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
        ], dtype=np.float32)

    @classmethod
    def _goal_world(cls, local_goal, pose):
        if pose is None:
            return None
        return cls._local_to_world(local_goal, pose)

    def _select_stable_waypoint(self, candidate, pose, points):
        """Keep a safe A* waypoint stable across lidar/grid quantization.

        The rolling grid is rebuilt at 10 Hz.  Without a world-frame hold, a
        straight corridor can alternate between neighbouring y-cells even
        though the physical route has not changed.  Release the held waypoint
        only after it is close, its edge is blocked, or it has fallen behind.
        """
        if candidate is None or pose is None:
            return candidate
        candidate_world = self._local_to_world(candidate, pose)
        if self.stable_waypoint_world is None:
            self.stable_waypoint_world = candidate_world
            self.waypoint_switch_count += 1
            return candidate

        held_local = self._world_to_local(self.stable_waypoint_world, pose)
        held_distance = float(np.hypot(held_local[0], held_local[1]))
        candidate_delta = float(np.hypot(
            candidate_world[0] - self.stable_waypoint_world[0],
            candidate_world[1] - self.stable_waypoint_world[1],
        ))
        held_heading = abs(math.atan2(float(held_local[1]), float(held_local[0])))
        held_clear = self.grid_planner._edge_is_clear(
            np.zeros(2, dtype=np.float32), held_local, points,
            clearance=self.grid_edge_clearance
        )
        release = (
            held_distance <= self.waypoint_hold_radius
            or not held_clear
            or held_local[0] < -0.20
            or held_heading > math.radians(105.0)
        )
        if release:
            self.stable_waypoint_world = candidate_world
            self.waypoint_switch_count += 1
            return candidate

        # A large candidate displacement is logged but does not by itself
        # cancel a still-clear route. This is the same commitment rule used by
        # the global frontier manager and prevents branch flicker at a corner.
        if candidate_delta > self.waypoint_switch_distance:
            self.waypoint_hold_count += 1
        else:
            self.waypoint_hold_count += 1
        return held_local

    def diagnostic(self):
        waypoint = "none" if self.last_waypoint is None else "(%.2f,%.2f)" % (
            self.last_waypoint[0], self.last_waypoint[1]
        )
        clearance = self.last_forward_clearance
        now = time.monotonic()
        lock_remaining = max(0.0, self.boundary_turn_lock_until - now)
        cooldown_remaining = max(0.0, self.boundary_entry_blocked_until - now)
        boundary_goal_delta = float("nan")
        if self.boundary_goal_world is not None:
            # The current pose is not needed here; this field is updated in
            # choose() and is retained for a compact status message.
            boundary_goal_delta = self.last_boundary_goal_delta
        return "grid_points=%d forward_clearance=%.3f waypoint=%s boundary=%s " \
               "side=%d lock=%.2f cooldown=%.2f reorient=%s " \
               "entries=%d exits=%d turns=%d turn_active=%s turn_attempts=%d " \
               "turn_cooldown=%.2f side_switches=%d stuck_releases=%d " \
               "reorient_entries=%d reorient_exits=%d " \
               "wp_holds=%d wp_switches=%d boundary_goal_delta=%.2f " \
               "no_path=%s no_path_entries=%d no_path_holds=%d " \
               "astar_clear=%d astar_rejoins=%d wall_side=%d wall_dist=%.3f " \
               "wall_front=%.3f wall_rear=%.3f wall_error=%.3f wall_angular=%.3f " \
               "wall_filtered=(%.3f,%.3f,%.3f) blocked_frames=%d" % (
            self.last_point_count,
            clearance if math.isfinite(clearance) else float("nan"),
            waypoint,
            self.boundary_active,
            self.boundary_turn_side,
            lock_remaining,
            cooldown_remaining,
            self.reorienting_goal,
            self.boundary_entries,
            self.boundary_exits,
            self.boundary_turns,
            self.boundary_turn_active,
            self.boundary_turn_attempts,
            max(0.0, self.boundary_turn_cooldown_until - now),
            self.boundary_side_switches,
            self.boundary_stuck_releases,
            self.reorient_entries,
            self.reorient_exits,
            self.waypoint_hold_count,
            self.waypoint_switch_count,
            boundary_goal_delta,
            self.no_path_active,
            self.no_path_entries,
            self.no_path_holds,
            self.boundary_astar_clear_frames,
            self.boundary_astar_rejoins,
            self.boundary_wall_side,
            self.boundary_wall_last_distance,
            self.boundary_wall_last_front,
            self.boundary_wall_last_rear,
            self.boundary_wall_last_error,
            self.boundary_wall_last_angular,
            self.boundary_wall_filtered_distance,
            self.boundary_wall_filtered_front,
            self.boundary_wall_filtered_rear,
            self.boundary_forward_blocked_frames,
        )

    @staticmethod
    def _turn_action(error, max_action=0.40):
        """Return a bounded proportional turn in the SA-PPO action domain."""
        return float(np.clip(0.90 * float(error), -max_action, max_action))

    def _reset_no_path_state(self):
        self.no_path_active = False
        self.no_path_started_at = 0.0
        self.no_path_turn_sign = 0

    def _held_waypoint_if_safe(self, pose, points):
        """Reuse the previous world waypoint through a transient A* failure."""
        if self.stable_waypoint_world is None or pose is None:
            return None
        held_local = self._world_to_local(self.stable_waypoint_world, pose)
        distance = float(np.hypot(held_local[0], held_local[1]))
        heading = abs(math.atan2(float(held_local[1]), float(held_local[0])))
        if (
            distance <= self.waypoint_hold_radius
            or held_local[0] < 0.0
            or heading > math.radians(80.0)
            or not self.grid_planner._edge_is_clear(
                np.zeros(2, dtype=np.float32), held_local, points,
                clearance=self.grid_edge_clearance
            )
        ):
            return None
        return held_local

    def _no_path_action(self, local_goal, pose, points, visibility_points):
        """Hold a stable local recovery action while A* catches up.

        The rolling occupancy grid is intentionally conservative.  A scan/map
        update can therefore make a valid corridor disappear for a single
        control cycle.  This state uses the current forward clearance and a
        locked turn sign; it never commands reverse and it only calls Bug2
        when a real near obstacle is present.
        """
        now = time.monotonic()
        if not self.no_path_active:
            self.no_path_active = True
            self.no_path_started_at = now
            self.no_path_entries += 1
            bearing = math.atan2(float(local_goal[1]), float(local_goal[0]))
            self.no_path_turn_sign = 1 if bearing >= 0.0 else -1
        self.no_path_holds += 1

        bearing = math.atan2(float(local_goal[1]), float(local_goal[0]))
        bearing_abs = abs(bearing)
        near_clearance = (
            float(np.min(np.hypot(points[:, 0], points[:, 1])))
            if points.size else float("inf")
        )
        forward_clearance = self.last_forward_clearance

        # A large bearing error is handled as one locked, proportional turn.
        # Unlike the old DWA fallback this cannot change sign because a single
        # noisy grid frame changed the preferred rollout.
        if bearing_abs > math.radians(48.0):
            self.last_reason = "grid_no_path_reorient"
            sign = self.no_path_turn_sign or (1 if bearing >= 0.0 else -1)
            return [0.0, self._turn_action(sign * min(bearing_abs, math.pi / 2.0), 0.34)]

        # If the current scan still shows a close obstacle, let the boundary
        # state choose a verified tangent.  It owns side selection and its
        # lock prevents left/right chatter.
        if near_clearance <= self.reorient_obstacle_clearance:
            boundary_action = self._boundary_action(
                local_goal, pose, points, visibility_points
            )
            if boundary_action is not None:
                return boundary_action

        # Forward clearance is noisy near furniture.  Use a broad hysteresis
        # band and a low-speed continuation instead of dropping to zero at one
        # threshold.  The rollout safety check still rejects a genuinely
        # colliding command before it is published.
        if math.isfinite(forward_clearance) and forward_clearance < 0.62:
            self.last_reason = "grid_no_path_scan_turn"
            sign = self.no_path_turn_sign or 1
            return [0.0, self._turn_action(0.35 * sign, 0.28)]

        speed = 0.24
        if math.isfinite(forward_clearance):
            speed = float(np.clip(
                0.16 + 0.10 * (forward_clearance - 0.75), 0.16, 0.34
            ))
        if bearing_abs > math.radians(12.0):
            speed *= max(0.55, math.cos(bearing_abs))
        angular = 0.0 if bearing_abs < math.radians(4.0) else self._turn_action(
            bearing, 0.22
        )
        self.last_reason = "grid_no_path_hold_forward"
        return [speed, angular]

    def choose(self, raw_action, policy_action, local_goal, scan, scan_param, pose):
        # Update the rolling map before any early turn return.  Previously a
        # goal-facing rotation skipped map updates, so the first post-turn
        # frame used stale obstacle geometry and immediately recreated the
        # previous boundary decision.
        points = self.obstacle_points(scan, scan_param, pose)
        visibility_points = RollingObstacleMap._scan_points(scan, scan_param, max_range=8.0)
        self.last_point_count = int(points.shape[0])
        self.last_waypoint = None
        if pose is not None:
            current_goal_world = self._goal_world(local_goal, pose)
            if self.last_goal_world is not None and current_goal_world is not None:
                goal_delta = float(np.hypot(
                    current_goal_world[0] - self.last_goal_world[0],
                    current_goal_world[1] - self.last_goal_world[1],
                ))
                if goal_delta >= self.boundary_goal_cancel_distance:
                    # A new frontier/visual segment invalidates a held local
                    # waypoint, but not an already selected Bug2 wall side.
                    self.stable_waypoint_world = None
                    self._reset_no_path_state()
            self.last_goal_world = current_goal_world
        forward_points = visibility_points[
            (visibility_points[:, 0] > 0.0)
            & (np.abs(visibility_points[:, 1]) < 0.30)
        ] if visibility_points.size else np.empty((0, 2), dtype=np.float32)
        self.last_forward_clearance = (
            float(np.min(np.hypot(forward_points[:, 0], forward_points[:, 1])))
            if forward_points.size else float("nan")
        )
        if forward_points.size:
            forward_ranges = np.hypot(forward_points[:, 0], forward_points[:, 1])
            raw_min = float(np.min(forward_ranges))
            # One isolated low beam is often a chair leg or scan speckle. Use
            # a robust sector statistic for speed control, while preserving an
            # immediate hard response inside the true footprint buffer.
            robust = float(np.percentile(forward_ranges, 10.0))
            if raw_min <= self.min_safe_clearance:
                robust = raw_min
            if not math.isfinite(self.forward_clearance_filtered):
                self.forward_clearance_filtered = robust
            else:
                self.forward_clearance_filtered += 0.35 * (
                    robust - self.forward_clearance_filtered
                )
            self.last_forward_clearance = self.forward_clearance_filtered
        else:
            self.forward_clearance_filtered = float("nan")

        # Compute the local route before deciding whether to enter a pure
        # goal-facing turn. A valid A* waypoint already encodes a safe bend;
        # using the raw global bearing first was the source of many unnecessary
        # spin-in-place episodes after a frontier handoff.
        waypoint = self.grid_planner.waypoint(points, local_goal)
        if waypoint is not None:
            waypoint = self._select_stable_waypoint(waypoint, pose, points)
            self.last_waypoint = (float(waypoint[0]), float(waypoint[1]))
            if self.reorienting_goal:
                # A newly available collision-free route supersedes a stale
                # pure-turn commitment from a previous no-path scan.
                self.reorienting_goal = False
                self.reorienting_turn_sign = 0
                self.reorienting_aligned_since = None

        # In a clear, nearly straight segment the learned policy should remain
        # the authority. The grid is still used as a collision veto through the
        # rollout check, but rewriting every safe policy action as a waypoint
        # command makes the controller look like a second oscillating planner.
        if (
            waypoint is not None
            and not self.boundary_active
            and not self.reorienting_goal
            and float(raw_action[0]) > 0.08
            and float(policy_action[0]) > 0.04
            and abs(float(policy_action[1])) <= 0.22
            and math.isfinite(self.last_forward_clearance)
            and self.last_forward_clearance >= 0.80
        ):
            # The rolling map is for route planning. Collision prediction must
            # use the current scan; otherwise a wall endpoint observed tens of
            # seconds ago can trigger a false safety intervention after the
            # robot has already passed it.
            policy_rollout = self.rollout(policy_action, local_goal, visibility_points)
            waypoint_heading = math.atan2(float(waypoint[1]), float(waypoint[0]))
            goal_heading = math.atan2(float(local_goal[1]), float(local_goal[0]))
            if (
                policy_rollout is not None
                and abs(waypoint_heading) <= math.radians(28.0)
                and abs(goal_heading) <= math.radians(35.0)
            ):
                self._reset_no_path_state()
                self.last_reason = "policy_safe_open_route"
                return policy_action, False, self.last_reason, policy_rollout[2]

        # A visual-servo or global-frontier update can legitimately put the
        # replacement waypoint behind the robot.  Bug2's wall contour belongs
        # to the local obstacle route, so keep the contour when it is already
        # active.  If a new turn is requested while the robot is close to a
        # wall, enter wall-follow recovery first instead of spinning in place.
        if pose is not None:
            goal_bearing = math.atan2(float(local_goal[1]), float(local_goal[0]))
            bearing_magnitude = abs(goal_bearing)
            now = time.monotonic()
            near_clearance = (
                float(np.min(np.hypot(visibility_points[:, 0], visibility_points[:, 1])))
                if visibility_points.size else float("inf")
            )
            if (
                not self.boundary_active
                and bearing_magnitude >= self.reorient_enter
                and near_clearance <= self.reorient_obstacle_clearance
                and waypoint is None
            ):
                self.reorienting_goal = False
                self.reorienting_turn_sign = 0
                self.reorienting_aligned_since = None
                self.boundary_entry_blocked_until = 0.0
                wall_action = self._boundary_action(
                    local_goal, pose, points, visibility_points
                )
                if wall_action is not None:
                    return wall_action, True, self.last_reason, float("nan")
            # Start the turn before Bug2 claims a wall tangent. This also
            # covers a side-front waypoint when no boundary is active yet.
            if (not self.reorienting_goal and not self.boundary_active
                    and bearing_magnitude >= self.reorient_enter
                    and waypoint is None):
                self.reorienting_goal = True
                self.reorient_entries += 1
                self.reorienting_turn_sign = 1 if goal_bearing > 0.0 else -1
                self.reorienting_aligned_since = None
                self.last_reason = "grid_begin_goal_reorientation"
            if self.reorienting_goal:
                if bearing_magnitude <= self.reorient_exit:
                    if self.reorienting_aligned_since is None:
                        self.reorienting_aligned_since = now
                    elif now - self.reorienting_aligned_since >= self.reorient_exit_dwell:
                        self.reorienting_goal = False
                        self.reorient_exits += 1
                        self.reorienting_turn_sign = 0
                        self.reorienting_aligned_since = None
                        self.boundary_entry_blocked_until = now + self.boundary_reentry_cooldown
                else:
                    self.reorienting_aligned_since = None
            if self.reorienting_goal:
                if near_clearance <= self.reorient_obstacle_clearance:
                    # The route is wall-constrained. Preserve a single
                    # boundary side rather than repeatedly turning toward a
                    # goal that remains occluded from the current pose.
                    self.reorienting_goal = False
                    self.reorienting_turn_sign = 0
                    self.reorienting_aligned_since = None
                    self.boundary_entry_blocked_until = 0.0
                    wall_action = self._boundary_action(
                        local_goal, pose, points, visibility_points
                    )
                    if wall_action is not None:
                        return wall_action, True, self.last_reason, float("nan")
                self.boundary_active = False
                self.boundary_start = None
                self.goal_visibility_frames = 0
                self.boundary_entry_blocked_until = now + self.boundary_reentry_cooldown
                # Slow down near the exit band. A fixed +/-0.5 action can
                # cross both thresholds between observations and immediately
                # hand control to Bug2, recreating the opposite turn.
                turn_magnitude = 0.22 if bearing_magnitude < math.radians(60.0) else 0.5
                self.last_reason = "grid_turn_goal_behind"
                turn_sign = self.reorienting_turn_sign or (1 if goal_bearing > 0.0 else -1)
                return [0.0, self._turn_action(
                    turn_sign * min(bearing_magnitude, math.pi / 2.0),
                    0.34 if turn_magnitude < 0.5 else 0.40,
                )], True, self.last_reason, float("nan")
        # A* is the controller's best local route whenever the rolling lidar
        # map contains a collision-free path. Bug2 is a recovery policy for
        # the genuinely no-path case; letting it run first made it override
        # valid corner/doorway routes and repeatedly turn the camera away from
        # a recently detected target.
        if waypoint is not None:
            action = self.grid_planner.action_to_waypoint(waypoint)
            rollout = self.rollout(action, local_goal, visibility_points)
            if rollout is not None:
                if self.boundary_active:
                    self.boundary_astar_clear_frames += 1
                    if self.boundary_astar_clear_frames < 3:
                        boundary_action = self._boundary_action(
                            local_goal, pose, points, visibility_points
                        )
                        if boundary_action is not None:
                            return boundary_action, True, self.last_reason, float("nan")
                    self.boundary_astar_rejoins += 1
                    self.boundary_exits += 1
                    self._reset_boundary_state(cooldown=True)
                    self.last_reason = "grid_rejoin_astar"
                else:
                    self.boundary_astar_clear_frames = 0
                    self._reset_no_path_state()
                    self.last_reason = "grid_waypoint=(%.2f,%.2f)" % (waypoint[0], waypoint[1])
                return action, True, self.last_reason, rollout[2]

        # If A* briefly loses its target cell, retain the last world waypoint
        # when its edge is still clear.  This is the common scan integration
        # case and is preferable to a full safety re-evaluation.
        held_waypoint = self._held_waypoint_if_safe(pose, points)
        if held_waypoint is not None and not self.boundary_active:
            self.last_waypoint = (float(held_waypoint[0]), float(held_waypoint[1]))
            action = self.grid_planner.action_to_waypoint(held_waypoint)
            rollout = self.rollout(action, local_goal, visibility_points)
            if rollout is not None:
                self.last_reason = "grid_waypoint_hold"
                self._reset_no_path_state()
                return action, True, self.last_reason, rollout[2]

        if waypoint is None:
            # A lidar map cannot represent unobserved space.  While committed
            # to a boundary route, preserve that exploration behavior through
            # temporary no-path intervals; do not surrender to SA-PPO's raw
            # reverse/stop command.
            boundary_action = self._boundary_action(local_goal, pose, points, visibility_points)
            if boundary_action is not None:
                return boundary_action, True, self.last_reason, float("nan")
            action = self._no_path_action(
                local_goal, pose, points, visibility_points
            )
            return action, True, self.last_reason, float("nan")

        boundary_action = self._boundary_action(local_goal, pose, points, visibility_points)
        if boundary_action is not None:
            return boundary_action, True, self.last_reason, float("nan")

        action = self.grid_planner.action_to_waypoint(waypoint)
        rollout = self.rollout(action, local_goal, visibility_points)
        if rollout is None:
            heading = math.atan2(float(waypoint[1]), float(waypoint[0]))
            if abs(heading) > math.radians(3.0):
                # A rotate-only command is safe under rollout(), and gives the
                # next grid update a new viewpoint instead of freezing at the
                # edge of the safety buffer.
                self.last_reason = "grid_turn_before_constrained_edge"
                return [0.0, self._turn_action(
                    math.copysign(min(abs(heading), math.pi / 2.0), heading), 0.34
                )], True, self.last_reason, 0.0
            boundary_action = self._boundary_action(local_goal, pose, points, visibility_points)
            if boundary_action is not None:
                return boundary_action, True, self.last_reason, 0.0
            self.last_reason = "grid_no_safe_forward_edge"
            return [0.0, 0.0], True, self.last_reason, 0.0
        self.last_reason = "grid_waypoint=(%.2f,%.2f)" % (waypoint[0], waypoint[1])
        return action, True, self.last_reason, rollout[2]
