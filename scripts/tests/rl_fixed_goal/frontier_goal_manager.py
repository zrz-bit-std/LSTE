#!/usr/bin/env python3
"""Choose reachable map frontiers while retaining one fixed final goal."""

import math
from collections import deque
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from actionlib_msgs.msg import GoalID, GoalStatusArray


class FrontierGoalManager:
    def __init__(self):
        self.goal = np.array([rospy.get_param("~goal_x"), rospy.get_param("~goal_y")], dtype=float)
        self.goal_topic = rospy.get_param("~goal_topic", "/rl_fixed_goal_test/final_goal")
        self.pose = None
        self.last_goal = None
        self.pending_goal = None
        self.last_published_goal = None
        self.active_goal = None
        self.active_goal_time = None
        self.latest_map = None
        self.map_processed_for_current_goal = False
        # Once SLAM has observed a clearance-safe path to the actual fixed
        # target, exploration is over.  Keeping this state prevents later map
        # updates from replacing that target with a different frontier.
        self.final_goal_dispatched = False
        self.final_goal_completed = False
        # The fixed-goal publisher republishes its latched value at 1 Hz. Keep
        # a bounded number of full move_base recovery attempts for transient
        # SLAM failures, then make an obstacle-embedded target terminal.
        self.final_goal_unreachable = False
        self.final_goal_failure_count = 0
        self.max_final_goal_failures = max(
            1, int(rospy.get_param("~max_final_goal_failures", 3))
        )
        self.max_subgoal_seconds = float(rospy.get_param("~max_subgoal_seconds", 45.0))
        # A frontier cell can be globally free but still be too close to a
        # newly observed local obstacle for TEB to enter its goal tolerance.
        # Do not hold that stale waypoint for the full action timeout when
        # SLAM has already supplied a materially different reachable frontier.
        self.frontier_stall_seconds = float(rospy.get_param("~frontier_stall_seconds", 6.0))
        self.frontier_stall_progress = float(rospy.get_param("~frontier_stall_progress", 0.10))
        self.frontier_stall_proximity = float(rospy.get_param("~frontier_stall_proximity", 1.0))
        self.frontier_replacement_distance = float(
            rospy.get_param("~frontier_replacement_distance", 0.60)
        )
        self.active_goal_best_distance = None
        self.active_goal_last_progress_time = None
        # Exploration may use a normal doorway, so this is the vehicle radius
        # (0.30 m) plus a modest mapping margin, rather than the larger local
        # planner comfort zone. TEB still provides the final collision check.
        self.frontier_clearance = float(rospy.get_param("~frontier_clearance", 0.42))
        self.publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )
        self.cancel_publisher = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.map_service = rospy.ServiceProxy("/dynamic_map", GetMap)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber("/pro3/wheel_odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.on_final_goal, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.on_move_base_status, queue_size=1)
        # gmapping does not promise a map replay to a late subscriber. Keep the
        # latest candidate and republish it when move_base connects instead of
        # relying on a single startup-time topic delivery.
        rospy.Timer(rospy.Duration(1.0), self.publish_pending)
        rospy.Timer(rospy.Duration(1.0), self.load_cached_map)

    def on_odom(self, message):
        self.pose = np.array([message.pose.pose.position.x, message.pose.pose.position.y])
        if self.active_goal is not None and not self.final_goal_dispatched:
            distance = float(np.linalg.norm(self.pose - self.active_goal))
            if (self.active_goal_best_distance is None
                    or distance <= self.active_goal_best_distance - self.frontier_stall_progress):
                self.active_goal_best_distance = distance
                self.active_goal_last_progress_time = rospy.Time.now()
        # Topic delivery order is not deterministic. If gmapping supplied a
        # cached map before this first pose, finish the deferred plan now.
        if self.latest_map is not None and not self.map_processed_for_current_goal:
            self.on_map(self.latest_map)

    def load_cached_map(self, _event):
        """Recover gmapping's map when a stationary robot emits no new topic."""
        if self.latest_map is not None:
            return
        try:
            self.on_map(self.map_service().map)
            rospy.loginfo("Loaded current SLAM map from /dynamic_map")
        except (rospy.ServiceException, rospy.ROSException) as error:
            rospy.logwarn_throttle(5.0, "Waiting for current SLAM map: %s", error)

    def on_final_goal(self, message):
        """Restart exploration when the LSTE global-goal source changes."""
        new_goal = np.array([message.pose.position.x, message.pose.position.y], dtype=float)
        if np.linalg.norm(new_goal - self.goal) < 0.05:
            # The fixed Goal Manager republishes the current target every second.
            # Do not let that periodic message revive an unreachable target.
            return
        old_goal = self.goal.copy()
        self.goal = new_goal
        self.last_goal = None
        self.pending_goal = None
        self.last_published_goal = None
        self.active_goal = None
        self.active_goal_time = None
        self.active_goal_best_distance = None
        self.active_goal_last_progress_time = None
        self.final_goal_dispatched = False
        self.final_goal_completed = False
        self.final_goal_unreachable = False
        self.final_goal_failure_count = 0
        self.map_processed_for_current_goal = False
        # Stop the old frontier/final action immediately. The next map update
        # will publish a route that belongs to the newly clicked final goal.
        self.cancel_publisher.publish(GoalID())
        rospy.logwarn(
            "Final goal changed by Gazebo click: (%.2f, %.2f) -> (%.2f, %.2f); restarting exploration",
            old_goal[0], old_goal[1], new_goal[0], new_goal[1],
        )
        # A stationary robot causes gmapping to stop publishing changed maps.
        # Reuse the most recent map now, otherwise canceling the old action
        # would leave the vehicle stopped forever while waiting for motion
        # that requires a newly planned action.
        if self.latest_map is not None:
            self.on_map(self.latest_map)

    def on_move_base_status(self, message):
        # Simple-goal action IDs are generated inside move_base, so retain the
        # current frontier while any move_base action is active. It is released
        # only after the action reaches a terminal state; this prevents a map
        # update from repeatedly cancelling an otherwise valid local plan.
        if any(status.status in (0, 1) for status in message.status_list):
            return
        if self.active_goal is not None and message.status_list:
            terminal_statuses = {status.status for status in message.status_list}
            if self.final_goal_dispatched:
                if 3 in terminal_statuses:
                    self.final_goal_completed = True
                    rospy.loginfo("move_base reports the fixed final goal reached")
                else:
                    self.final_goal_failure_count += 1
                    if self.final_goal_failure_count < self.max_final_goal_failures:
                        # Keep the fixed target selected for a bounded retry.
                        # Clearing this marker causes publish_pending() to
                        # issue one new simple-goal action on its next tick.
                        self.last_published_goal = None
                        rospy.logwarn(
                            "Fixed final goal failed after recovery: target=(%.2f,%.2f) "
                            "status=%s; retrying (%d/%d)",
                            self.goal[0], self.goal[1], terminal_statuses,
                            self.final_goal_failure_count, self.max_final_goal_failures,
                        )
                    else:
                        self.final_goal_unreachable = True
                        self.final_goal_dispatched = False
                        self.pending_goal = None
                        self.last_published_goal = None
                        rospy.logerr(
                            "Final goal unreachable after %d failed recovery attempts: "
                            "target=(%.2f,%.2f) status=%s; holding position until "
                            "the target changes",
                            self.final_goal_failure_count, self.goal[0], self.goal[1],
                            terminal_statuses,
                        )
            else:
                rospy.loginfo("Frontier goal completed or failed; selecting the next reachable frontier")
            self.active_goal = None
            self.active_goal_time = None
            self.active_goal_best_distance = None
            self.active_goal_last_progress_time = None

    @staticmethod
    def _inflate(occupied, cells):
        """Return occupied cells enlarged by the robot footprint clearance."""
        inflated = occupied.copy()
        rows, cols = occupied.shape
        for row_offset in range(-cells, cells + 1):
            for col_offset in range(-cells, cells + 1):
                if row_offset * row_offset + col_offset * col_offset > cells * cells:
                    continue
                source_row0 = max(0, -row_offset)
                source_row1 = min(rows, rows - row_offset)
                source_col0 = max(0, -col_offset)
                source_col1 = min(cols, cols - col_offset)
                target_row0 = max(0, row_offset)
                target_row1 = min(rows, rows + row_offset)
                target_col0 = max(0, col_offset)
                target_col1 = min(cols, cols + col_offset)
                inflated[target_row0:target_row1, target_col0:target_col1] |= occupied[
                    source_row0:source_row1, source_col0:source_col1
                ]
        return inflated

    @staticmethod
    def _nearest_free_seed(free, seed_row, seed_col, maximum_cells):
        """Find a known-safe seed when gmapping still marks the car cell unknown."""
        rows, cols = free.shape
        if 0 <= seed_row < rows and 0 <= seed_col < cols and free[seed_row, seed_col]:
            return seed_row, seed_col
        best = None
        for radius in range(1, maximum_cells + 1):
            row0, row1 = max(0, seed_row - radius), min(rows, seed_row + radius + 1)
            col0, col1 = max(0, seed_col - radius), min(cols, seed_col + radius + 1)
            candidates = np.argwhere(free[row0:row1, col0:col1])
            if candidates.size:
                candidates[:, 0] += row0
                candidates[:, 1] += col0
                distances = (candidates[:, 0] - seed_row) ** 2 + (candidates[:, 1] - seed_col) ** 2
                row, col = candidates[np.argmin(distances)]
                return int(row), int(col)
        return best

    def on_map(self, message):
        self.latest_map = message
        if self.pose is None:
            return
        self.map_processed_for_current_goal = True
        data = np.asarray(message.data, dtype=np.int8).reshape(message.info.height, message.info.width)
        known_free = data == 0
        unknown = data == -1
        # Free cells adjacent to unobserved space form exploration frontiers.
        adjacent_unknown = np.zeros_like(unknown, dtype=bool)
        adjacent_unknown[1:] |= unknown[:-1]
        adjacent_unknown[:-1] |= unknown[1:]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        # Keep only the free-space component connected to the robot.  A map
        # frontier on the far side of a wall can be geometrically close to the
        # final goal but is not a valid next navigation target.
        resolution = message.info.resolution
        robot_col = int((self.pose[0] - message.info.origin.position.x) / resolution)
        robot_row = int((self.pose[1] - message.info.origin.position.y) / resolution)
        # Search the complete SLAM map. A local window can declare there is no
        # route when the only doorway lies farther away around a wall.
        row0, row1 = 0, known_free.shape[0]
        col0, col1 = 0, known_free.shape[1]
        occupied = data >= 50
        safe_free = known_free & ~self._inflate(
            occupied, max(1, int(math.ceil(self.frontier_clearance / resolution)) - 1)
        )
        seed_row, seed_col = robot_row, robot_col
        seed = self._nearest_free_seed(
            safe_free, seed_row, seed_col, maximum_cells=max(1, int(0.8 / resolution))
        )
        if seed is None:
            center_value = (
                int(data[robot_row, robot_col])
                if 0 <= robot_row < data.shape[0] and 0 <= robot_col < data.shape[1]
                else None
            )
            rospy.logwarn_throttle(
                2.0,
                "No safe frontier seed: pose=(%.2f,%.2f) cell=(%d,%d) value=%s "
                "known_free=%d safe_free=%d",
                self.pose[0], self.pose[1], robot_row, robot_col, center_value,
                int(np.count_nonzero(known_free)), int(np.count_nonzero(safe_free)),
            )
            return
        seed_row, seed_col = seed
        connected = np.zeros_like(safe_free, dtype=bool)
        path_steps = np.full(safe_free.shape, -1, dtype=np.int32)
        queue = deque([(seed_row, seed_col)])
        connected[seed_row, seed_col] = True
        path_steps[seed_row, seed_col] = 0
        while queue:
            row, col = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = row + dr, col + dc
                if (0 <= nr < safe_free.shape[0] and 0 <= nc < safe_free.shape[1]
                        and safe_free[nr, nc] and not connected[nr, nc]):
                    connected[nr, nc] = True
                    path_steps[nr, nc] = path_steps[row, col] + 1
                    queue.append((nr, nc))

        # The fixed target can be handed to move_base only after it is both
        # observed as free and part of the robot's clearance-safe connected
        # component.  This turns exploration into genuine goal navigation and
        # avoids treating a nearby but wall-separated map cell as reachable.
        goal_col = int((self.goal[0] - message.info.origin.position.x) / resolution)
        goal_row = int((self.goal[1] - message.info.origin.position.y) / resolution)
        goal_local_row, goal_local_col = goal_row, goal_col
        goal_in_local_map = (
            0 <= goal_local_row < safe_free.shape[0]
            and 0 <= goal_local_col < safe_free.shape[1]
        )
        goal_is_safe_and_connected = (
            goal_in_local_map
            and safe_free[goal_local_row, goal_local_col]
            and connected[goal_local_row, goal_local_col]
        )
        if self.final_goal_unreachable:
            # The current clicked target already exhausted move_base recovery.
            # Holding here prevents the publisher's periodic replay from
            # recreating a high-rate recovery loop. A different click resets
            # this state in on_final_goal().
            return
        if goal_is_safe_and_connected:
            if not self.final_goal_dispatched:
                rospy.loginfo(
                    "Final goal now safely connected: target=(%.2f,%.2f), cell=(%d,%d); ending frontier exploration",
                    self.goal[0], self.goal[1], goal_row, goal_col,
                )
                self.final_goal_dispatched = True
                self.pending_goal = self.goal.copy()
                self.last_goal = self.goal.copy()
                # Force one immediate publication even when the last frontier
                # happened to lie very close to the true target.
                self.last_published_goal = None
                # The current move_base action is a frontier waypoint. Clear
                # it so the timer below publishes the final target promptly,
                # which makes move_base preempt the old waypoint.
                self.active_goal = None
                self.active_goal_time = None
                self.active_goal_best_distance = None
                self.active_goal_last_progress_time = None
            return
        if self.final_goal_dispatched:
            return
        frontier = known_free & adjacent_unknown
        rows, cols = np.nonzero(frontier & connected)
        if rows.size == 0:
            # Do not use a point that is merely closer in Euclidean distance:
            # it can sit immediately in front of a wall with the true target
            # on the other side. A new scan/map can still create a real
            # frontier, which will be considered on the next callback.
            rospy.logwarn_throttle(
                2.0,
                "No reachable frontier in the global safe map; "
                "holding position instead of approaching a blocking wall",
            )
            return
        xs = message.info.origin.position.x + (cols + 0.5) * resolution
        ys = message.info.origin.position.y + (rows + 0.5) * resolution
        candidates = np.column_stack((xs, ys))
        # Prefer progress toward the fixed target, using graph path length
        # through the known free component rather than straight-line distance
        # through walls.
        robot_distance = np.linalg.norm(candidates - self.pose, axis=1)
        goal_distance = np.linalg.norm(candidates - self.goal, axis=1)
        path_distance = path_steps[rows, cols].astype(float) * resolution
        valid = robot_distance >= 1.0
        if not np.any(valid):
            rospy.logwarn_throttle(
                2.0,
                "No reachable frontier candidate outside the robot footprint",
            )
            return
        candidates = candidates[valid]
        robot_distance = robot_distance[valid]
        goal_distance = goal_distance[valid]
        path_distance = path_distance[valid]
        scores = goal_distance + path_distance
        selected_index = int(np.argmin(scores))
        candidate = candidates[selected_index]
        rospy.loginfo_throttle(
            2.0,
            "Frontier candidates=%d clearance=%.2f selected=(%.2f,%.2f) "
            "straight_distance=%.2f path_distance=%.2f goal_distance=%.2f",
            len(candidates), self.frontier_clearance, candidate[0], candidate[1],
            float(np.linalg.norm(candidate - self.pose)),
            float(path_distance[selected_index]),
            float(np.linalg.norm(candidate - self.goal)),
        )
        if self.last_goal is not None and np.linalg.norm(candidate - self.last_goal) < 0.6:
            return
        self.last_goal = candidate
        self.pending_goal = candidate

    def publish_pending(self, _event):
        if self.pending_goal is None or self.publisher.get_num_connections() == 0:
            return
        if self.active_goal is not None:
            if self.final_goal_completed:
                return
            elapsed = (rospy.Time.now() - self.active_goal_time).to_sec()
            if self.final_goal_dispatched:
                # A direct final-goal action is never replaced by a frontier
                # timeout.  Its terminal state above determines any retry.
                return
            active_distance = float(np.linalg.norm(self.pose - self.active_goal)) if self.pose is not None else float("inf")
            pending_changed = (
                np.linalg.norm(self.pending_goal - self.active_goal)
                >= self.frontier_replacement_distance
            )
            no_progress_seconds = (
                (rospy.Time.now() - self.active_goal_last_progress_time).to_sec()
                if self.active_goal_last_progress_time is not None else 0.0
            )
            # This is deliberately restricted to the terminal approach to a
            # frontier. A route that temporarily increases its Euclidean
            # distance while going around a wall remains under move_base's
            # ordinary 45-second action timeout.
            if (pending_changed
                    and active_distance <= self.frontier_stall_proximity
                    and no_progress_seconds >= self.frontier_stall_seconds):
                rospy.logwarn(
                    "Frontier subgoal stalled near endpoint: active=(%.2f,%.2f) "
                    "distance=%.2f no_progress=%.1fs replacement=(%.2f,%.2f); "
                    "preempting stale action",
                    self.active_goal[0], self.active_goal[1], active_distance,
                    no_progress_seconds, self.pending_goal[0], self.pending_goal[1],
                )
                # Publishing a new simple goal preempts the old move_base
                # action. Do not send a blanket cancel here: it could cancel
                # the replacement action when actionlib processes messages in
                # a different order.
                self.active_goal = None
                self.active_goal_time = None
                self.active_goal_best_distance = None
                self.active_goal_last_progress_time = None
            elif elapsed < self.max_subgoal_seconds:
                return
            else:
                rospy.logwarn("Frontier goal timed out after %.1f s; trying a new reachable frontier", elapsed)
                self.active_goal = None
                self.active_goal_time = None
                self.active_goal_best_distance = None
                self.active_goal_last_progress_time = None
        if (self.last_published_goal is not None
                and np.linalg.norm(self.pending_goal - self.last_published_goal) < 0.05):
            return
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = self.pending_goal
        goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        self.last_published_goal = self.pending_goal.copy()
        self.active_goal = self.pending_goal.copy()
        self.active_goal_time = rospy.Time.now()
        self.active_goal_best_distance = (
            float(np.linalg.norm(self.pose - self.active_goal)) if self.pose is not None else None
        )
        self.active_goal_last_progress_time = self.active_goal_time
        if self.final_goal_dispatched:
            rospy.loginfo("Published fixed final goal=(%.2f, %.2f)", *self.pending_goal)
        else:
            rospy.loginfo(
                "Frontier subgoal=(%.2f, %.2f), final=(%.2f, %.2f)",
                self.pending_goal[0], self.pending_goal[1], *self.goal,
            )


if __name__ == "__main__":
    rospy.init_node("rl_fixed_goal_frontier_manager")
    FrontierGoalManager()
    rospy.spin()
