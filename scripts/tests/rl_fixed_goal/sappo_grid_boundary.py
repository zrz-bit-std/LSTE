"""Bug2-style boundary following for :class:`GridSafetyGuard`.

This cooperating base owns one stateful responsibility: entering, following,
and releasing an obstacle boundary after local A* has no safe route.  The
concrete grid guard initializes the shared lidar and waypoint state before
these methods are used.
"""

import math
import time

import numpy as np
import rospy


class GridBoundaryRecovery:
    def _reset_boundary_state(self, cooldown=True):
        """Release Bug2 ownership and clear all route-specific memory."""
        self.boundary_active = False
        self.boundary_start = None
        self.boundary_goal_world = None
        self.last_boundary_goal_delta = float("nan")
        self.boundary_best_goal_distance = float("inf")
        self.boundary_progress_last_time = 0.0
        self.boundary_side_switches = 0
        self.goal_visibility_frames = 0
        self.boundary_turn_lock_until = 0.0
        self.boundary_turn_side = 0
        self.boundary_turn_active = False
        self.boundary_turn_started_at = 0.0
        self.boundary_turn_start_yaw = 0.0
        self.boundary_turn_requested_angle = 0.0
        self.boundary_turn_attempts = 0
        self.boundary_turn_cooldown_until = 0.0
        self.boundary_wall_side = 0
        self.boundary_wall_last_distance = float("nan")
        self.boundary_wall_last_front = float("nan")
        self.boundary_wall_last_rear = float("nan")
        self.boundary_wall_last_error = float("nan")
        self.boundary_wall_last_angular = 0.0
        self.boundary_wall_filtered_distance = float("nan")
        self.boundary_wall_filtered_front = float("nan")
        self.boundary_wall_filtered_rear = float("nan")
        self.boundary_wall_front_missing_frames = 0
        self.boundary_wall_rear_missing_frames = 0
        self.boundary_wall_lost_frames = 0
        self.boundary_forward_blocked_frames = 0
        if cooldown:
            self.boundary_entry_blocked_until = (
                time.monotonic() + self.boundary_reentry_cooldown
            )
        self.reorienting_goal = False
        self.reorienting_turn_sign = 0
        self.reorienting_aligned_since = None
        self.stable_waypoint_world = None
        self.boundary_astar_clear_frames = 0

    def _begin_boundary_turn(self, pose, angle, reason="bug2_boundary_turn"):
        """Start one bounded in-place scan turn.

        ``boundary_turn_lock_until`` is retained for diagnostics and legacy
        callers, but ``boundary_turn_active`` is the authoritative state. A
        turn ends when its requested heading is reached, not merely when a
        wall scan happens to look clear for one frame.
        """
        if pose is None or self.boundary_turn_attempts >= self.boundary_turn_max_attempts:
            return False
        _, _, yaw = (float(value) for value in pose)
        sign = 1.0 if float(angle) >= 0.0 else -1.0
        requested = min(abs(float(angle)), self.boundary_turn_max_angle)
        if requested < math.radians(35.0):
            requested = math.radians(35.0)
        self.boundary_turn_side = 1 if sign > 0.0 else -1
        self.boundary_heading_world = wrap_angle(yaw + sign * requested)
        now = time.monotonic()
        self.boundary_turn_active = True
        self.boundary_turn_started_at = now
        self.boundary_turn_start_yaw = yaw
        self.boundary_turn_requested_angle = requested
        self.boundary_turn_attempts += 1
        self.boundary_turn_lock_until = now + self.boundary_turn_max_duration
        self.boundary_turns += 1
        self.last_reason = reason
        return True

    def _complete_boundary_turn(self, reason):
        """Release the turn lock and give wall following a chance to advance."""
        self.boundary_turn_active = False
        self.boundary_turn_lock_until = 0.0
        self.boundary_turn_cooldown_until = time.monotonic() + 0.9
        self.boundary_forward_blocked_frames = 0
        self.boundary_wall_last_angular = 0.0
        self.last_reason = reason

    def _turn_around_boundary_end(self, local_goal, pose, points):
        """Choose a verified side around a newly encountered boundary end.

        The chosen directions are expressed in the current robot frame.  They
        deliberately exclude a U-turn: boundary following should look through
        an opening beside the wall, not retreat from the unexplored route.
        """
        if pose is None:
            return False
        now = time.monotonic()
        if self.boundary_turn_active or self.boundary_turn_attempts >= self.boundary_turn_max_attempts:
            self.last_reason = "bug2_turn_budget_exhausted"
            return False
        _, _, yaw = (float(value) for value in pose)
        goal_bearing = math.atan2(float(local_goal[1]), float(local_goal[0]))
        # Once Bug2 has selected a boundary side, keep it at corners. Choosing
        # the other side from a transient scan is what creates +/- turn flips.
        if self.boundary_turn_side > 0:
            candidates = (math.pi / 2.0,)
        elif self.boundary_turn_side < 0:
            candidates = (-math.pi / 2.0,)
        else:
            candidates = (-math.pi / 2.0, math.pi / 2.0)
        viable = []
        for angle in candidates:
            direction = np.array([math.cos(angle), math.sin(angle)], dtype=np.float32)
            if self.grid_planner._edge_is_clear(
                    np.zeros(2, dtype=np.float32), direction * 0.90, points,
                    clearance=self.grid_edge_clearance,
            ):
                score = abs(wrap_angle(goal_bearing - angle))
                viable.append((score, angle))
        if not viable:
            return False
        _, angle = min(viable)
        return self._begin_boundary_turn(
            pose, angle, reason="bug2_turn_around_boundary_end"
        )

    def _boundary_progress_watchdog(self, local_goal, pose):
        """Bound Bug2 wall-following when a selected tangent makes no progress."""
        if not self.boundary_active or pose is None or self.boundary_start is None:
            return None
        now = time.monotonic()
        x, y, yaw = (float(value) for value in pose)
        distance = float(np.hypot(local_goal[0], local_goal[1]))
        if not math.isfinite(self.boundary_best_goal_distance):
            self.boundary_best_goal_distance = distance
            self.boundary_progress_last_time = now
            return None
        if distance < self.boundary_best_goal_distance - 0.12:
            self.boundary_best_goal_distance = distance
            self.boundary_progress_last_time = now
            return None

        travelled = math.hypot(
            x - self.boundary_start[0], y - self.boundary_start[1]
        )
        stalled_for = now - self.boundary_progress_last_time
        far_from_best = distance > self.boundary_best_goal_distance + self.boundary_progress_margin
        if stalled_for < self.boundary_progress_timeout or not far_from_best or travelled < 0.80:
            return None

        if self.boundary_side_switches < 1:
            best_before_switch = self.boundary_best_goal_distance
            self.boundary_side_switches += 1
            self.boundary_turn_side = -self.boundary_turn_side or 1
            self.boundary_wall_side = -self.boundary_wall_side or -self.boundary_turn_side
            self.boundary_start = (x, y)
            self.boundary_best_goal_distance = distance
            self.boundary_progress_last_time = now
            self.last_reason = "bug2_switch_side_no_progress"
            rospy.logwarn(
                "RL test Bug2 switched wall side: best_goal=%.2f current=%.2f "
                "travelled=%.2f stalled=%.1fs side=%d",
                best_before_switch,
                distance,
                travelled,
                stalled_for,
                self.boundary_turn_side,
            )
            if self._begin_boundary_turn(
                    pose,
                    self.boundary_turn_side * math.pi / 2.0,
                    reason="bug2_switch_side_no_progress",
            ):
                return [0.0, self._turn_action(
                    self.boundary_turn_side * math.pi / 2.0, 0.40
                )]
            self.last_reason = "bug2_release_no_progress_turn_budget"
            self._reset_boundary_state(cooldown=True)
            return None

        self.boundary_stuck_releases += 1
        self.last_reason = "bug2_release_no_progress"
        rospy.logwarn(
            "RL test Bug2 released after no progress: best_goal=%.2f current=%.2f "
            "travelled=%.2f stalled=%.1fs",
            self.boundary_best_goal_distance,
            distance,
            travelled,
            stalled_for,
        )
        self._reset_boundary_state(cooldown=True)
        return None

    @staticmethod
    def _sector_stat(points, center_angle, half_width, percentile=20.0, max_range=2.5):
        """Return a robust range statistic for one laser angular sector."""
        if points is None or points.size == 0:
            return float("nan")
        angles = np.arctan2(points[:, 1], points[:, 0])
        delta = np.arctan2(
            np.sin(angles - float(center_angle)),
            np.cos(angles - float(center_angle)),
        )
        mask = (np.abs(delta) <= float(half_width))
        if not np.any(mask):
            return float("nan")
        ranges = np.hypot(points[mask, 0], points[mask, 1])
        ranges = ranges[np.isfinite(ranges)]
        # A boundary side is meaningful only while an obstacle is within the
        # local contour window.  Including 6-10 m endpoints in the percentile
        # makes a missing wall look like a large positive distance error and
        # steers the robot toward empty space.
        ranges = ranges[ranges <= float(max_range)]
        if ranges.size == 0:
            return float("nan")
        return float(np.percentile(ranges, percentile))

    def _wall_follow_action(self, pose, visibility_points):
        """Track a selected wall with distance/heading feedback.

        The previous boundary controller kept the heading captured at Bug2
        entry and returned ``[0.35, 0]`` forever.  A small heading error then
        became metres of lateral drift.  This controller uses the current scan
        to regulate the chosen wall side and its tangent, so a doorway/corner
        can be followed without repeatedly reselecting a new goal.
        """
        if pose is None or self.boundary_wall_side == 0:
            return None
        wall_side = 1 if self.boundary_wall_side > 0 else -1
        side_angle = wall_side * math.pi / 2.0
        side_distance = self._sector_stat(
            visibility_points,
            side_angle,
            math.radians(52.0),
            percentile=25.0,
            max_range=self.boundary_wall_max_range,
        )
        front_distance = self._sector_stat(
            visibility_points, wall_side * math.pi / 4.0,
            math.radians(30.0), percentile=35.0,
            max_range=self.boundary_wall_max_range,
        )
        rear_distance = self._sector_stat(
            visibility_points, wall_side * 3.0 * math.pi / 4.0,
            math.radians(30.0), percentile=35.0,
            max_range=self.boundary_wall_max_range,
        )
        self.boundary_wall_last_distance = side_distance
        self.boundary_wall_last_front = front_distance
        self.boundary_wall_last_rear = rear_distance

        def smooth(previous, current):
            if not math.isfinite(current):
                return previous
            if not math.isfinite(previous):
                return current
            alpha = self.boundary_wall_filter_alpha
            return previous + alpha * (current - previous)

        self.boundary_wall_filtered_distance = smooth(
            self.boundary_wall_filtered_distance, side_distance
        )
        if math.isfinite(front_distance):
            self.boundary_wall_front_missing_frames = 0
            self.boundary_wall_filtered_front = smooth(
                self.boundary_wall_filtered_front, front_distance
            )
        else:
            self.boundary_wall_front_missing_frames += 1
            if self.boundary_wall_front_missing_frames >= 2:
                self.boundary_wall_filtered_front = float("nan")
        if math.isfinite(rear_distance):
            self.boundary_wall_rear_missing_frames = 0
            self.boundary_wall_filtered_rear = smooth(
                self.boundary_wall_filtered_rear, rear_distance
            )
        else:
            self.boundary_wall_rear_missing_frames += 1
            if self.boundary_wall_rear_missing_frames >= 2:
                self.boundary_wall_filtered_rear = float("nan")

        # If the wall is briefly out of view at a convex corner, continue in
        # the locked tangent for one scan interval rather than changing sides.
        if not math.isfinite(side_distance):
            self.boundary_wall_lost_frames += 1
            # Do not let a last-seen side range keep steering the car after the
            # contour has disappeared. It is valid only for the current scan.
            self.boundary_wall_filtered_distance = float("nan")
            # A convex corner can hide the wall for a few scans. If it stays
            # absent while the forward sector is open, this is no longer a
            # wall-follow situation; release to A* instead of drifting on a
            # stale tangent.
            if self.boundary_wall_lost_frames >= 3 and (
                    not math.isfinite(self.last_forward_clearance)
                    or self.last_forward_clearance > 0.80
            ):
                self._reset_boundary_state(cooldown=True)
                self.last_reason = "bug2_release_wall_lost"
                return None
            _, _, yaw = (float(value) for value in pose)
            error = wrap_angle(self.boundary_heading_world - yaw)
            angular = self._turn_action(error, 0.30)
            self.boundary_wall_last_error = float("nan")
            self.boundary_wall_last_angular = angular
            return [0.20, angular]

        self.boundary_wall_lost_frames = 0
        side_distance = self.boundary_wall_filtered_distance
        front_distance = self.boundary_wall_filtered_front
        rear_distance = self.boundary_wall_filtered_rear
        if not math.isfinite(side_distance):
            return None
        distance_error = side_distance - self.boundary_wall_distance
        tangent_error = 0.0
        if math.isfinite(front_distance) and math.isfinite(rear_distance):
            # For a left wall, front<rear means the nose points toward the
            # wall and requires a right correction.  Multiplying by the wall
            # sign gives the same convention for a right wall.
            tangent_error = front_distance - rear_distance
        steering = wall_side * (
            self.boundary_wall_kp * distance_error
            + self.boundary_wall_heading_kp * tangent_error
        )
        target_angular = float(np.clip(steering, -0.34, 0.34))
        # A small opposite correction at a corner is usually a percentile/grid
        # fluctuation. Let the previous correction decay through zero; reserve
        # an immediate sign change for a genuinely strong new tangent.
        if (
            self.boundary_wall_last_angular * target_angular < 0.0
            and abs(target_angular) < 0.20
        ):
            target_angular = 0.0
        # Limit wall-feedback changes independently of the policy shaper. This
        # prevents a single percentile jump at a doorway from commanding an
        # immediate opposite turn while still allowing a genuine corner turn.
        angular = float(np.clip(
            target_angular,
            self.boundary_wall_last_angular - self.boundary_wall_angular_step,
            self.boundary_wall_last_angular + self.boundary_wall_angular_step,
        ))
        if abs(angular) < self.boundary_wall_angular_deadband:
            angular = 0.0
        self.boundary_wall_last_error = distance_error
        self.boundary_wall_last_angular = angular

        forward_clearance = self.last_forward_clearance
        if math.isfinite(forward_clearance):
            # Slow down before the footprint buffer, but let the wall feedback
            # continue steering instead of replacing the command with a hard
            # stop on every noisy range sample.
            speed = float(np.clip(
                0.16 + 0.18 * (forward_clearance - 0.55),
                0.12, self.boundary_wall_max_linear,
            ))
        else:
            speed = min(0.25, self.boundary_wall_max_linear)
        if abs(angular) > 0.22:
            speed *= 0.72
        return [speed, angular]

    def _boundary_action(self, local_goal, pose, points, visibility_points=None):
        if pose is None:
            return None
        x, y, yaw = (float(value) for value in pose)
        goal_distance = float(np.hypot(local_goal[0], local_goal[1]))
        direct_goal = np.asarray(local_goal, dtype=np.float32)
        if goal_distance > self.grid_planner.half_extent:
            direct_goal *= self.grid_planner.half_extent / goal_distance

        if self.boundary_active:
            # A goal behind the robot is normal while tracing the obstacle.
            # Compare the world-frame target with the one captured when Bug2
            # entered instead of cancelling on local bearing alone.  The old
            # angle-only rule cancelled a valid wall route at every half-turn,
            # then the goal reorientation state started the same turn again.
            current_goal_world = self._goal_world(local_goal, pose)
            if self.boundary_goal_world is None:
                self.boundary_goal_world = current_goal_world
            if current_goal_world is not None and self.boundary_goal_world is not None:
                self.last_boundary_goal_delta = float(np.hypot(
                    current_goal_world[0] - self.boundary_goal_world[0],
                    current_goal_world[1] - self.boundary_goal_world[1],
                ))
            else:
                self.last_boundary_goal_delta = float("nan")
            goal_bearing = math.atan2(float(local_goal[1]), float(local_goal[0]))
            if (
                math.isfinite(self.last_boundary_goal_delta)
                and self.last_boundary_goal_delta >= self.boundary_goal_cancel_distance
            ):
                # Keep the selected wall side, but re-anchor Bug2 to the new
                # world target and the current hit point. This handles a
                # normal frontier waypoint advance without following the old
                # segment for several metres; it also avoids the old
                # cancel-then-reorient oscillation at the same wall.
                self.boundary_goal_world = current_goal_world
                self.boundary_start = (x, y)
                self.boundary_hit_distance = goal_distance
                self.goal_visibility_frames = 0
                self.last_boundary_goal_delta = 0.0
                self.boundary_best_goal_distance = goal_distance
                self.boundary_progress_last_time = time.monotonic()
                self.boundary_side_switches = 0
                self.last_reason = "bug2_update_boundary_goal"

            watchdog_action = self._boundary_progress_watchdog(local_goal, pose)
            if watchdog_action is not None:
                return watchdog_action
            if not self.boundary_active:
                # The watchdog released an exhausted commitment. Let the
                # caller run A* / no-path selection with the current scan.
                return None

            displacement = (math.hypot(x - self.boundary_start[0], y - self.boundary_start[1])
                            if self.boundary_start is not None else 0.0)
            # Classic Bug2 waits until it is closer to the goal than the hit
            # point.  That condition is unnecessarily conservative for a
            # lidar controller: this U-shaped wall first requires moving away
            # from the goal.  Once the rolling lidar map confirms a direct,
            # footprint-clear segment after genuine exploration, hand control
            # back to A* immediately.
            # Current-frame endpoints are less prone to occlusion ghosts than
            # the rolling map for deciding whether to leave a wall.  Require
            # three consecutive observations before changing modes.
            visibility = points if visibility_points is None else visibility_points
            if self.boundary_wall_side == 0:
                # Recover the wall side if a boundary state was entered by an
                # older checkpoint or a transient no-path branch.
                nearby = visibility[np.hypot(visibility[:, 0], visibility[:, 1]) < 1.6] \
                    if visibility.size else np.empty((0, 2), dtype=np.float32)
                mean_y = float(np.mean(nearby[:, 1])) if nearby.size else 0.0
                self.boundary_wall_side = 1 if mean_y >= 0.0 else -1
            direct_visible = self.grid_planner._edge_is_clear(
                np.zeros(2, dtype=np.float32), direct_goal, visibility,
                clearance=self.grid_edge_clearance,
            )
            self.goal_visibility_frames = self.goal_visibility_frames + 1 if direct_visible else 0
            if displacement >= 0.75 and self.goal_visibility_frames >= 3:
                self.boundary_exits += 1
                self._reset_boundary_state(cooldown=True)
                self.last_reason = "bug2_leave_boundary_direct_route_visible"
                return None
            # A locked outward heading can meet a newly observed obstacle
            # around a corner. Turn for a new scan instead of driving into the
            # safety buffer or handing control back to the reverse-prone policy.
            local_forward = np.array([1.0, 0.0], dtype=np.float32)
            forward_blocked = not self.grid_planner._edge_is_clear(
                np.zeros(2, dtype=np.float32), local_forward * 0.55, visibility,
                clearance=self.grid_edge_clearance,
            )
            if forward_blocked:
                self.boundary_forward_blocked_frames += 1
            else:
                self.boundary_forward_blocked_frames = 0

            # A boundary-end turn is an explicit short maneuver. Finish it by
            # heading or by a bounded time/angle budget, then let the wall
            # controller move. The previous implementation checked only a
            # three-second lock and started another quarter-turn whenever the
            # forward sector was still blocked, which could spin indefinitely.
            now = time.monotonic()
            if self.boundary_turn_active:
                heading_error = wrap_angle(self.boundary_heading_world - yaw)
                turned = abs(wrap_angle(yaw - self.boundary_turn_start_yaw))
                elapsed = now - self.boundary_turn_started_at
                if (
                    abs(heading_error) <= self.boundary_turn_heading_tolerance
                    or turned >= self.boundary_turn_requested_angle - math.radians(4.0)
                ):
                    self._complete_boundary_turn("bug2_boundary_turn_complete")
                elif elapsed >= self.boundary_turn_max_duration or turned >= self.boundary_turn_max_angle:
                    self._complete_boundary_turn("bug2_boundary_turn_budget_exhausted")
                else:
                    self.last_reason = "bug2_boundary_turn_locked"
                    return [0.0, self._turn_action(heading_error, 0.34)]

                # A completed turn gets one scan interval to expose the new
                # tangent. Do not immediately schedule another turn from the
                # same blocked frame.
                wall_action = self._wall_follow_action(pose, visibility)
                if wall_action is not None:
                    return wall_action
                if not self.boundary_active:
                    return None
                self.last_reason = "bug2_boundary_turn_follow_fallback"
                return [0.14, self._turn_action(
                    wrap_angle(self.boundary_heading_world - yaw), 0.24
                )]

            if self.boundary_forward_blocked_frames >= 2:
                if (
                    now >= self.boundary_turn_cooldown_until
                    and self.boundary_turn_attempts < self.boundary_turn_max_attempts
                    and self._turn_around_boundary_end(local_goal, pose, visibility)
                ):
                    error = wrap_angle(self.boundary_heading_world - yaw)
                    return [0.0, self._turn_action(error, 0.40)]
                # The turn budget is exhausted or the last turn just ended.
                # Follow the selected contour at low speed; this keeps the car
                # moving through a narrow doorway instead of issuing another
                # in-place stop/turn pair.
                self.last_reason = "bug2_boundary_blocked_wall_follow"
            wall_action = self._wall_follow_action(pose, visibility)
            if wall_action is not None:
                self.last_reason = "bug2_follow_boundary"
                return wall_action
            if not self.boundary_active:
                # ``_wall_follow_action`` released a stale contour; let the
                # caller immediately try the current A* route.
                return None
            self.last_reason = "bug2_follow_boundary_heading_fallback"
            return [0.24, self._turn_action(
                wrap_angle(self.boundary_heading_world - yaw), 0.28
            )]

        # After leaving a boundary or finishing a goal-facing turn, wait for a
        # fresh scan before re-entering Bug2. This prevents the same obstacle's
        # first partial scan from immediately reversing the previous turn.
        if time.monotonic() < self.boundary_entry_blocked_until:
            self.last_reason = "grid_boundary_entry_cooldown"
            return None
        if points.size == 0:
            return None
        visibility = points if visibility_points is None else visibility_points
        ranges = np.hypot(points[:, 0], points[:, 1])
        # Boundary entry is based on the current scan. The rolling map is used
        # for A*, but old endpoint cells must not make a clear current frame
        # look like a newly blocked wall.
        entry_points = visibility
        entry_ranges = np.hypot(entry_points[:, 0], entry_points[:, 1])
        near = entry_points[entry_ranges < 1.3]
        if near.shape[0] < 4:
            return None
        # In the usual failure case the robot is already moving in a useful
        # free direction, but the direct route to the goal becomes occluded by
        # a wall. Preserve that direction first.  Points are in *robot-local*
        # coordinates, therefore [1, 0] is the current forward edge regardless
        # of the robot's world yaw.  This sends the fixed test south toward the
        # known wall endpoint instead of incorrectly rotating west into it.
        if self.grid_planner._edge_is_clear(
                np.zeros(2, dtype=np.float32), np.array([1.0, 0.0], dtype=np.float32),
                entry_points, clearance=self.grid_edge_clearance,
        ):
            self.boundary_heading_world = yaw
            self.boundary_hit_distance = goal_distance
            self.boundary_start = (x, y)
            self.boundary_goal_world = self._goal_world(local_goal, pose)
            self.last_boundary_goal_delta = 0.0
            self.boundary_active = True
            self.boundary_entries += 1
            self.boundary_best_goal_distance = goal_distance
            self.boundary_progress_last_time = time.monotonic()
            self.boundary_side_switches = 0
            self.goal_visibility_frames = 0
            self.boundary_turn_lock_until = 0.0
            # Deterministic right-hand fallback for a route that was initially
            # clear but later meets a boundary end. Keep this side locked.
            self.boundary_turn_side = -1
            left_distance = self._sector_stat(
                visibility, math.pi / 2.0, math.radians(52.0), percentile=20.0
            )
            right_distance = self._sector_stat(
                visibility, -math.pi / 2.0, math.radians(52.0), percentile=20.0
            )
            if math.isfinite(left_distance) and (
                    not math.isfinite(right_distance)
                    or left_distance + 0.15 < right_distance
            ):
                self.boundary_wall_side = 1
            elif math.isfinite(right_distance) and (
                    not math.isfinite(left_distance)
                    or right_distance + 0.15 < left_distance
            ):
                self.boundary_wall_side = -1
            else:
                nearby = visibility[
                    np.hypot(visibility[:, 0], visibility[:, 1]) < 1.6
                ] if visibility.size else np.empty((0, 2), dtype=np.float32)
                nearby_mean_y = float(np.mean(nearby[:, 1])) if nearby.size else 0.0
                self.boundary_wall_side = 1 if nearby_mean_y >= 0.0 else -1
            self.last_reason = "bug2_enter_boundary_keep_clear_heading"
            return self._boundary_action(local_goal, pose, points, visibility)

        # If forward is genuinely blocked, the weighted normal identifies the
        # wall side. Follow its tangent requiring the least turn from current
        # travel, which is Bug2's deterministic fallback side.
        near_ranges = np.hypot(near[:, 0], near[:, 1])
        normal = -np.sum(near / np.maximum(near_ranges[:, None], 0.15) ** 2, axis=0)
        normal /= max(float(np.hypot(normal[0], normal[1])), 1e-6)
        tangent = np.array([-normal[1], normal[0]], dtype=np.float32)
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float32)
        if float(np.dot(tangent, forward)) < 0.0:
            tangent = -tangent
        self.boundary_heading_world = math.atan2(float(tangent[1]), float(tangent[0]))
        self.boundary_hit_distance = goal_distance
        self.boundary_start = (x, y)
        self.boundary_goal_world = self._goal_world(local_goal, pose)
        self.last_boundary_goal_delta = 0.0
        self.boundary_active = True
        self.boundary_entries += 1
        self.boundary_best_goal_distance = goal_distance
        self.boundary_progress_last_time = time.monotonic()
        self.boundary_side_switches = 0
        self.goal_visibility_frames = 0
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float32)
        cross = float(forward[0] * tangent[1] - forward[1] * tangent[0])
        self.boundary_turn_side = 1 if cross > 0.0 else -1
        nearby = visibility[
            np.hypot(visibility[:, 0], visibility[:, 1]) < 1.6
        ] if visibility.size else np.empty((0, 2), dtype=np.float32)
        mean_y = float(np.mean(nearby[:, 1])) if nearby.size else 0.0
        if abs(mean_y) >= 0.05:
            self.boundary_wall_side = 1 if mean_y > 0.0 else -1
        else:
            # A head-on wall has no signed side in one scan. Keep the side
            # consistent with the chosen tangent until the next corner.
            self.boundary_wall_side = 1 if cross > 0.0 else -1
        self.boundary_turn_lock_until = 0.0
        self.last_reason = "bug2_enter_boundary"
        return self._boundary_action(local_goal, pose, points)
