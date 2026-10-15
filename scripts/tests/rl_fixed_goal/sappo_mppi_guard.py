"""Sampling-MPC safety guard for the isolated SA-PPO fixed-goal test."""

import math

import numpy as np
import rospy

from sappo_safety_primitives import DwaSafetyGuard, RollingObstacleMap, wrap_angle


class MppiSafetyGuard(DwaSafetyGuard):
    """Test-only sampling-MPC guard that can turn first and then move.

    DWA evaluates a constant command and can settle at a wall because turning
    initially increases goal-heading error. This guard evaluates complete
    command sequences over the same local laser snapshot, including explicit
    turn-then-drive maneuvers. It remains a controller-side safety override:
    the global goal and all LSTE brain topics stay untouched.
    """

    def __init__(self):
        super().__init__()
        self.steps = 30
        self.sample_count = 120
        self.rng = np.random.default_rng(20260722)
        self.nominal = np.zeros((self.steps, 2), dtype=np.float32)
        # Sampling a fresh sequence every cycle makes the controller oscillate
        # once a wall is close. This state holds one outward heading until the
        # robot has genuinely created space from that wall.
        self.escape_active = False
        self.escape_heading_world = 0.0
        self.escape_start = None
        self.escape_started_at = 0.0
        self.escape_trigger_range = 0.72
        self.escape_release_range = 0.82
        self.escape_distance = 0.70
        self.escape_timeout = 12.0

    @staticmethod
    def _current_scan_points(scan, scan_param):
        """Return current-frame obstacle endpoints, without rolling-map history."""
        return RollingObstacleMap._scan_points(scan, scan_param, max_range=1.5)

    def _begin_escape(self, scan, scan_param, pose, local_goal):
        """Lock a goal-directed wall-following heading from current lidar data."""
        points = self._current_scan_points(scan, scan_param)
        if pose is None or points.size == 0:
            return False

        ranges = np.hypot(points[:, 0], points[:, 1])
        nearby = ranges < 1.2
        if not np.any(nearby):
            return False
        points = points[nearby]
        ranges = ranges[nearby]

        # Endpoint vectors point from robot to obstacles. Their inverse-square
        # sum points toward occupied space, so its negative is an outward
        # normal. It remains the fallback for non-wall-shaped obstacles.
        weights = 1.0 / np.maximum(ranges, 0.15) ** 2
        outward = -np.sum(points * weights[:, None], axis=0)
        outward_norm = float(np.hypot(outward[0], outward[1]))
        if outward_norm < 1e-5:
            return False
        outward /= outward_norm

        x, y, yaw = (float(value) for value in pose)
        direction = outward
        if points.shape[0] >= 8:
            # A wall produces a strongly elongated endpoint cloud. Follow its
            # principal tangent in whichever direction has more goal progress,
            # then retain a small outward component as a clearance buffer.
            centered = points - np.mean(points, axis=0)
            covariance = centered.T @ centered / float(points.shape[0])
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            if float(eigenvalues[1]) > 4.0 * max(float(eigenvalues[0]), 1e-6):
                tangent = eigenvectors[:, 1]
                goal_world = np.array([
                    math.cos(yaw) * float(local_goal[0]) - math.sin(yaw) * float(local_goal[1]),
                    math.sin(yaw) * float(local_goal[0]) + math.cos(yaw) * float(local_goal[1]),
                ], dtype=np.float32)
                if float(np.dot(tangent, goal_world)) < 0.0:
                    tangent = -tangent
                direction = tangent + 0.28 * outward
                direction /= max(float(np.hypot(direction[0], direction[1])), 1e-6)

        self.escape_heading_world = wrap_angle(yaw + math.atan2(direction[1], direction[0]))
        self.escape_start = (x, y)
        self.escape_started_at = rospy.Time.now().to_sec()
        self.escape_active = True
        rospy.logwarn(
            "RL test wall-follow entered: heading=%.1f deg nearest=%.2f m",
            math.degrees(self.escape_heading_world), float(ranges.min()),
        )
        return True

    def _escape_action(self, scan, scan_param, pose):
        """Rotate to the locked safe heading, then drive outward without reversing."""
        if not self.escape_active or pose is None or self.escape_start is None:
            return None

        x, y, yaw = (float(value) for value in pose)
        elapsed = rospy.Time.now().to_sec() - self.escape_started_at
        travelled = math.hypot(x - self.escape_start[0], y - self.escape_start[1])
        current = self._current_scan_points(scan, scan_param)
        current_min = float(np.min(np.hypot(current[:, 0], current[:, 1]))) if current.size else 5.0

        if ((travelled >= self.escape_distance and current_min >= self.escape_release_range)
                or elapsed >= self.escape_timeout):
            self.escape_active = False
            self.escape_start = None
            rospy.loginfo(
                "RL test wall-follow exited: travelled=%.2f m nearest=%.2f m elapsed=%.1f s",
                travelled, current_min, elapsed,
            )
            return None

        heading_error = wrap_angle(self.escape_heading_world - yaw)
        if abs(heading_error) > math.radians(14.0):
            return [0.0, 0.5 if heading_error > 0.0 else -0.5]

        # Continue to favour heading correction while moving. The command stays
        # in the original SA-PPO action domain and never commands reverse.
        return [0.32, float(np.clip(0.9 * heading_error, -0.5, 0.5))]

    def rollout_sequence(self, sequence, local_goal, points):
        x = y = yaw = 0.0
        min_clearance = float("inf")
        clearance_samples = []
        for linear, angular_action in sequence:
            linear_velocity = self.linear_speed_scale * float(linear)
            x += linear_velocity * math.cos(yaw) * self.dt
            y += linear_velocity * math.sin(yaw) * self.dt
            yaw = wrap_angle(yaw + 1.2 * float(angular_action) * self.dt)
            if points.size:
                clearance = float(np.min(np.hypot(points[:, 0] - x, points[:, 1] - y)))
                min_clearance = min(min_clearance, clearance)
                clearance_samples.append(clearance)
                # Never advance into the safety buffer. A zero-linear step is
                # still valid so the guard can rotate away from a nearby wall.
                if linear_velocity > 0.02 and clearance < self.min_safe_clearance:
                    return None

        goal_x, goal_y = float(local_goal[0]), float(local_goal[1])
        goal_distance = math.hypot(goal_x - x, goal_y - y)
        heading_error = abs(wrap_angle(math.atan2(goal_y - y, goal_x - x) - yaw))
        if not math.isfinite(min_clearance):
            min_clearance = 5.0
            clearance_samples.append(min_clearance)
        return (
            goal_distance,
            heading_error,
            min_clearance,
            clearance_samples[-1],
            float(np.mean(clearance_samples)),
        )

    def maneuver_sequences(self):
        """Generate deterministic escape maneuvers plus stochastic MPPI samples."""
        sequences = []
        for direction in (-1.0, 1.0):
            for turn_steps in (0, 4, 8, 12, 16):
                sequence = np.zeros((self.steps, 2), dtype=np.float32)
                sequence[:turn_steps, 1] = direction * 0.5
                # A gentle arc after the turn gives the optimizer trajectories
                # that can leave a wall instead of scoring only spin-in-place.
                sequence[turn_steps:, 0] = 0.20
                sequence[turn_steps:, 1] = direction * 0.12
                sequences.append(sequence)

        # Sample smooth perturbations around the best sequence from the prior
        # control cycle. This is the receding-horizon MPPI exploration term.
        noise = self.rng.normal(
            loc=0.0, scale=np.array([0.10, 0.23], dtype=np.float32),
            size=(self.sample_count, self.steps, 2),
        ).astype(np.float32)
        noise[:, 1:, :] = 0.65 * noise[:, 1:, :] + 0.35 * noise[:, :-1, :]
        samples = np.clip(self.nominal[None, :, :] + noise, ACTION_BOUND[0], ACTION_BOUND[1])
        sequences.extend(samples)
        return sequences

    def choose(self, raw_action, policy_action, local_goal, scan, scan_param, pose):
        escape_action = self._escape_action(scan, scan_param, pose)
        if escape_action is not None:
            self.last_reason = "committed_wall_escape"
            return escape_action, True, self.last_reason, float("nan")

        current_points = self._current_scan_points(scan, scan_param)
        current_min = (float(np.min(np.hypot(current_points[:, 0], current_points[:, 1])))
                       if current_points.size else float("inf"))
        if current_min < self.escape_trigger_range and self._begin_escape(scan, scan_param, pose, local_goal):
            escape_action = self._escape_action(scan, scan_param, pose)
            if escape_action is not None:
                self.last_reason = "committed_wall_escape"
                return escape_action, True, self.last_reason, current_min

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

        best_sequence = None
        best_score = float("inf")
        best_clearance = 0.0
        for sequence in self.maneuver_sequences():
            rollout = self.rollout_sequence(sequence, local_goal, points)
            if rollout is None:
                continue
            goal_distance, final_heading_error, clearance, final_clearance, mean_clearance = rollout
            # Rotating in place can retain a close clearance, but all moving
            # steps have already passed the hard safety-margin constraint.
            clearance_cost = 0.40 / max(clearance, 0.05)
            # The hard constraint uses the worst clearance. For route choice,
            # however, use final and average clearance too: after the robot
            # has reached a wall, every candidate shares the close starting
            # point, so a minimum-only objective cannot reward escaping it.
            preferred_clearance_cost = (
                4.0 * max(0.0, (self.preferred_clearance - final_clearance) / self.preferred_clearance) ** 2
                + 1.5 * max(0.0, (self.preferred_clearance - mean_clearance) / self.preferred_clearance) ** 2
            )
            smoothness = float(np.mean(np.abs(np.diff(sequence[:, 1]))))
            score = (
                2.0 * goal_distance
                + 0.50 * final_heading_error
                + clearance_cost
                + preferred_clearance_cost
                + 0.04 * smoothness
            )
            if score < best_score:
                best_score = score
                best_sequence = sequence
                best_clearance = clearance

        if best_sequence is None:
            self.last_reason = "emergency_stop_no_safe_sequence"
            return [0.0, 0.0], True, self.last_reason, 0.0

        # Receding horizon: retain the selected plan for the next optimization
        # cycle, shifted by one command. The first command alone is published.
        self.nominal[:-1] = best_sequence[1:]
        self.nominal[-1] = best_sequence[-1]
        return best_sequence[0].tolist(), True, self.last_reason, best_clearance
