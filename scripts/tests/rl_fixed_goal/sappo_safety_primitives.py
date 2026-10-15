"""Shared action, lidar-map, and one-step DWA primitives for SA-PPO tests."""

from collections import deque
import heapq
import math
import time

import numpy as np
import rospy


def wrap_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ActionShaper:
    """Bound command changes without delaying a genuine safety stop.

    The policy and the lidar guard both run at 10 Hz, while their decisions can
    change by a full action bound between two scans.  A small action slew limit
    removes the resulting left/right and accelerate/stop chatter.  A hard stop
    is deliberately passed through immediately for a completed goal or an
    unavailable safe rollout.
    """

    def __init__(self):
        self.last = np.zeros(2, dtype=np.float32)
        self.max_linear_step = max(
            0.02, float(rospy.get_param("~max_linear_action_step", 0.12))
        )
        self.max_linear_decel_step = max(
            0.02, float(rospy.get_param("~max_linear_action_decel", 0.06))
        )
        self.max_angular_step = max(
            0.03, float(rospy.get_param("~max_angular_action_step", 0.16))
        )
        self.angular_deadband = max(
            0.0, float(rospy.get_param("~angular_action_deadband", 0.035))
        )
        # Small opposite-sign corrections are usually grid quantization or a
        # noisy wall-distance estimate.  Let the command decay through zero;
        # require a meaningful opposite request before reversing the turn.
        self.angular_sign_switch_threshold = max(
            self.angular_deadband,
            float(rospy.get_param("~angular_sign_switch_threshold", 0.18)),
        )
        self.in_place_turn_threshold = max(
            0.05, float(rospy.get_param("~in_place_turn_angular_threshold", 0.12))
        )

    def apply(self, action, hard_stop=False, immediate_turn_stop=False):
        target = np.asarray(action, dtype=np.float32).reshape(-1)
        if target.size < 2:
            target = np.zeros(2, dtype=np.float32)
        target = np.array([
            float(np.clip(target[0], 0.0, 0.8)),
            float(np.clip(target[1], -0.5, 0.5)),
        ], dtype=np.float32)
        # A boundary turn asks for zero forward speed, but it is not by itself
        # an emergency.  Decelerate through the normal slew limiter so a
        # controller-mode change cannot create a repeated 0.17 -> 0.00 m/s
        # brake pulse.  ``hard_stop`` remains the only instantaneous stop.
        if abs(float(target[1])) < self.angular_deadband:
            target[1] = 0.0
        if (
            abs(float(self.last[1])) > self.angular_deadband
            and float(self.last[1]) * float(target[1]) < 0.0
            and abs(float(target[1])) < self.angular_sign_switch_threshold
        ):
            target[1] = 0.0
        if hard_stop:
            self.last[:] = 0.0
            return [0.0, 0.0]
        delta = target - self.last
        linear_step = self.max_linear_step
        if delta[0] < 0.0:
            linear_step = self.max_linear_decel_step
        delta[0] = float(np.clip(delta[0], -linear_step, self.max_linear_step))
        delta[1] = float(np.clip(delta[1], -self.max_angular_step, self.max_angular_step))
        self.last += delta
        self.last[0] = float(np.clip(self.last[0], 0.0, 0.8))
        self.last[1] = float(np.clip(self.last[1], -0.5, 0.5))
        if abs(float(self.last[1])) < self.angular_deadband:
            self.last[1] = 0.0
        return [float(self.last[0]), float(self.last[1])]


class StabilizedRecovery:
    """Turn once toward a behind-the-robot goal, with hysteresis and dwell."""

    def __init__(self):
        self.active = False
        self.exit_candidate_time = None
        self.enter_error = math.radians(120.0)
        self.exit_error = math.radians(25.0)
        self.exit_dwell = 0.3

    def action(self, local_goal):
        heading_error = math.atan2(float(local_goal[1]), float(local_goal[0]))
        magnitude = abs(heading_error)
        now = rospy.Time.now().to_sec()

        if not self.active and magnitude > self.enter_error:
            self.active = True
            self.exit_candidate_time = None
            rospy.loginfo("RL test recovery entered: heading_error=%.1f deg", math.degrees(heading_error))

        if not self.active:
            return None

        if magnitude < self.exit_error:
            if self.exit_candidate_time is None:
                self.exit_candidate_time = now
            elif now - self.exit_candidate_time >= self.exit_dwell:
                self.active = False
                self.exit_candidate_time = None
                rospy.loginfo("RL test recovery exited: heading_error=%.1f deg", math.degrees(heading_error))
                return None
        else:
            self.exit_candidate_time = None

        # Positive heading error is left; negative is right.
        return [0.0, 0.5 if heading_error > 0.0 else -0.5]


class RollingObstacleMap:
    """Short-lived odom-frame obstacle map built from consecutive 2D scans.

    This is deliberately a controller-local rolling obstacle cache, not a new
    source of navigation goals. It preserves wall geometry while the robot
    turns, so a door opening is not forgotten as soon as it leaves one laser
    frame.
    """

    def __init__(self, retention_seconds=6.0, cell_size=0.12, max_range=8.0):
        self.retention_seconds = retention_seconds
        self.cell_size = cell_size
        self.max_range = max_range
        self.frames = deque()

    @staticmethod
    def _scan_points(scan, scan_param, max_range):
        if scan is None or scan_param is None:
            return np.empty((0, 2), dtype=np.float32)
        values = np.asarray(scan, dtype=np.float32)
        if values.size == 0:
            return np.empty((0, 2), dtype=np.float32)
        angles = float(scan_param[0]) + np.arange(values.size, dtype=np.float32) * float(scan_param[2])
        valid = np.isfinite(values) & (values >= 0.05) & (values <= max_range)
        ranges = values[valid]
        angles = angles[valid]
        # A 512-beam scan has much finer angular resolution than this local
        # controller requires. Downsampling keeps planning deterministic.
        return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))[::4]

    def update(self, scan, scan_param, pose):
        points = self._scan_points(scan, scan_param, self.max_range)
        if points.size == 0 or pose is None:
            return
        x, y, yaw = (float(value) for value in pose)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_points = np.empty_like(points)
        world_points[:, 0] = x + cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
        world_points[:, 1] = y + sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
        now = time.monotonic()
        self.frames.append((now, world_points))
        while self.frames and now - self.frames[0][0] > self.retention_seconds:
            self.frames.popleft()

    def local_points(self, pose):
        if pose is None or not self.frames:
            return np.empty((0, 2), dtype=np.float32)
        world_points = np.concatenate([frame[1] for frame in self.frames], axis=0)
        x, y, yaw = (float(value) for value in pose)
        dx = world_points[:, 0] - x
        dy = world_points[:, 1] - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        points = np.column_stack((cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy))
        nearby = np.hypot(points[:, 0], points[:, 1]) <= self.max_range
        points = points[nearby]
        if points.size == 0:
            return points

        # Occupancy-cell consolidation turns many overlapping scan endpoints
        # into a compact rolling costmap representation.
        cells = np.floor(points / self.cell_size).astype(np.int32)
        _, indices = np.unique(cells, axis=0, return_index=True)
        return points[np.sort(indices)]


class LocalGridPlanner:
    """Small lidar-only occupancy grid with an inflated-obstacle A* search."""

    def __init__(self, half_extent=6.0, cell_size=0.15, obstacle_radius=0.56):
        self.half_extent = half_extent
        self.cell_size = cell_size
        self.obstacle_radius = obstacle_radius
        self.size = int(round(2.0 * half_extent / cell_size)) + 1
        self.center = self.size // 2

    def _cell(self, point):
        return (
            int(round(float(point[0]) / self.cell_size)) + self.center,
            int(round(float(point[1]) / self.cell_size)) + self.center,
        )

    def _point(self, cell):
        return np.array([
            (cell[0] - self.center) * self.cell_size,
            (cell[1] - self.center) * self.cell_size,
        ], dtype=np.float32)

    @staticmethod
    def _edge_is_clear(start, end, points, clearance=0.42):
        """Use the same circular footprint clearance as the velocity guard."""
        if points.size == 0:
            return True
        delta = end - start
        distance = float(np.hypot(delta[0], delta[1]))
        samples = max(1, int(math.ceil(distance / 0.05)))
        for ratio in np.linspace(0.0, 1.0, samples + 1):
            location = start + float(ratio) * delta
            if float(np.min(np.hypot(points[:, 0] - location[0], points[:, 1] - location[1]))) < clearance:
                return False
        return True

    def waypoint(self, points, local_goal):
        """Return a collision-free local waypoint, or None when none is known."""
        if points.size == 0:
            return None
        occupied = set()
        radius = int(math.ceil(self.obstacle_radius / self.cell_size))
        for point in points:
            cx, cy = self._cell(point)
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if dx * dx + dy * dy <= radius * radius:
                        x, y = cx + dx, cy + dy
                        if 0 <= x < self.size and 0 <= y < self.size:
                            occupied.add((x, y))

        start = (self.center, self.center)
        # The grid is local, so pursue a reachable segment when the final goal
        # lies outside it. Replanning on every scan rolls that segment forward.
        goal = np.asarray(local_goal, dtype=np.float32)
        length = float(np.hypot(goal[0], goal[1]))
        if length > self.half_extent - 0.5:
            goal *= (self.half_extent - 0.5) / length
        target = self._cell(goal)
        if start in occupied or target in occupied:
            return None

        frontier = [(0.0, start)]
        cost = {start: 0.0}
        parent = {}
        current = None
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == target:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                neighbour = (current[0] + dx, current[1] + dy)
                if (not (0 <= neighbour[0] < self.size and 0 <= neighbour[1] < self.size)
                        or neighbour in occupied
                        or not self._edge_is_clear(
                            self._point(current), self._point(neighbour), points
                        )):
                    continue
                step = math.hypot(dx, dy)
                candidate = cost[current] + step
                if candidate >= cost.get(neighbour, float("inf")):
                    continue
                cost[neighbour] = candidate
                parent[neighbour] = current
                heuristic = math.hypot(target[0] - neighbour[0], target[1] - neighbour[1])
                heapq.heappush(frontier, (candidate + heuristic, neighbour))
        if current != target:
            return None

        path = [target]
        while path[-1] != start:
            path.append(parent[path[-1]])
        path.reverse()
        lookahead = min(len(path) - 1, max(1, int(1.5 / self.cell_size)))
        cell = path[lookahead]
        return np.array([
            (cell[0] - self.center) * self.cell_size,
            (cell[1] - self.center) * self.cell_size,
        ], dtype=np.float32)

    @staticmethod
    def action_to_waypoint(waypoint):
        distance = float(np.hypot(waypoint[0], waypoint[1]))
        heading = math.atan2(float(waypoint[1]), float(waypoint[0]))
        heading_abs = abs(heading)
        # Turn in place only when the local route is genuinely side-on.  The
        # previous 18-degree hard boundary converted small A* cell changes
        # into alternating stop/turn commands in a straight corridor.
        if heading_abs > math.radians(32.0):
            return [0.0, float(np.clip(0.9 * heading, -0.5, 0.5))]
        # Use a continuous proportional steering law with a small deadband.
        # A +/-0.15 m lateral waypoint correction now produces a small angular
        # command instead of jumping between zero and +/-0.27 action.
        # Keep a small steering deadband around the corridor centreline.  The
        # 15 cm grid can quantize the next cell from left to right while the
        # physical path is still straight; feeding those tiny signs directly
        # into the policy creates visible yaw chatter.
        angular = 0.0 if heading_abs < math.radians(5.0) else float(
            np.clip(0.85 * heading, -0.38, 0.38)
        )
        speed = min(0.55, max(0.10, 0.42 * distance))
        if heading_abs > math.radians(12.0):
            speed *= max(0.45, math.cos(heading_abs))
        return [speed, angular]


class DwaSafetyGuard:
    """Test-only local DWA guard around a proposed SA-PPO action.

    It never changes the final goal. When the policy proposes reverse/zero
    forward motion, has a large heading error, or predicts a collision, this
    guard chooses a collision-free short-horizon (linear, angular) command
    from the same action space. The raw policy command passes through when it
    is usable and safe.
    """

    def __init__(self):
        self.dt = 0.10
        self.steps = 24
        # Actions are scaled by StageWorld before they are sent to Gazebo. The
        # local rollout must use that same physical speed; otherwise it judges
        # a 3 s horizon as twice as long as the robot actually travels.
        self.linear_speed_scale = float(rospy.get_param("~linear_speed_scale", 0.5))
        self.robot_radius = 0.30
        self.clearance_margin = 0.12
        self.min_safe_clearance = self.robot_radius + self.clearance_margin
        self.preferred_clearance = 0.70
        self.heading_recovery = math.radians(20.0)
        self.last_reason = "initializing"
        self.obstacle_map = RollingObstacleMap()

        # Commands are policy-space values; StageWorld scales angular velocity
        # by 1.2 before publishing /cmd_vel.
        self.linear_samples = np.array([0.0, 0.08, 0.16, 0.24, 0.32], dtype=np.float32)
        self.angular_samples = np.linspace(-0.5, 0.5, 25, dtype=np.float32)

    def obstacle_points(self, scan, scan_param, pose):
        self.obstacle_map.update(scan, scan_param, pose)
        return self.obstacle_map.local_points(pose)

    def rollout(self, action, local_goal, points):
        linear = self.linear_speed_scale * float(action[0])
        angular = 1.2 * float(action[1])
        x = y = yaw = 0.0
        min_clearance = float("inf")
        clearance_samples = []

        for _ in range(self.steps):
            x += linear * math.cos(yaw) * self.dt
            y += linear * math.sin(yaw) * self.dt
            yaw = wrap_angle(yaw + angular * self.dt)
            if points.size:
                clearance = float(np.min(np.hypot(points[:, 0] - x, points[:, 1] - y)))
                min_clearance = min(min_clearance, clearance)
                clearance_samples.append(clearance)
                # Rotation in place is allowed to escape a tight position, but
                # forward travel must preserve the configured safety margin.
                if linear > 0.02 and clearance < self.min_safe_clearance:
                    return None

        goal_x, goal_y = float(local_goal[0]), float(local_goal[1])
        goal_distance = math.hypot(goal_x - x, goal_y - y)
        final_heading_error = abs(wrap_angle(math.atan2(goal_y - y, goal_x - x) - yaw))
        if not math.isfinite(min_clearance):
            min_clearance = 5.0
            clearance_samples.append(min_clearance)
        return (
            goal_distance,
            final_heading_error,
            min_clearance,
            clearance_samples[-1],
            float(np.mean(clearance_samples)),
        )

    def choose(self, raw_action, policy_action, local_goal, scan, scan_param, pose):
        points = self.obstacle_points(scan, scan_param, pose)
        heading_error = wrap_angle(math.atan2(float(local_goal[1]), float(local_goal[0])))
        policy_rollout = self.rollout(policy_action, local_goal, points)

        if float(raw_action[0]) > 0.01 and abs(heading_error) <= self.heading_recovery and policy_rollout:
            self.last_reason = "policy_safe"
            return policy_action, False, self.last_reason, policy_rollout[2]

        if float(raw_action[0]) <= 0.01:
            self.last_reason = "policy_reverse_or_stop"
        elif abs(heading_error) > self.heading_recovery:
            self.last_reason = "heading_not_aligned"
        else:
            self.last_reason = "policy_collision_risk"

        best_action = None
        best_score = float("inf")
        best_clearance = 0.0
        for linear in self.linear_samples:
            for angular in self.angular_samples:
                candidate = [float(linear), float(angular)]
                rollout = self.rollout(candidate, local_goal, points)
                if rollout is None:
                    continue
                goal_distance, final_heading_error, clearance, final_clearance, mean_clearance = rollout
                # Prefer progress toward the unchanged goal, then a small final
                # heading error and generous obstacle clearance. This is a local
                # control score, not a planner or a replacement goal generator.
                score = (
                    2.0 * goal_distance
                    + 0.65 * final_heading_error
                    + 0.25 / max(clearance, 0.05)
                    + 0.20 / max(final_clearance, 0.05)
                    + 0.10 / max(mean_clearance, 0.05)
                    + 0.03 * abs(angular)
                )
                if score < best_score:
                    best_score = score
                    best_action = candidate
                    best_clearance = clearance

        if best_action is None:
            self.last_reason = "emergency_stop_no_safe_rollout"
            return [0.0, 0.0], True, self.last_reason, 0.0
        return best_action, True, self.last_reason, best_clearance
