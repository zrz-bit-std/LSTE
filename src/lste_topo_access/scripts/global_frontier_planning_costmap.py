"""Costmap freshness and connected-grid validation for frontier planning."""

import math

import numpy as np
import rospy

from clock_provider import now_for


class GlobalFrontierPlanningCostmapMixin:

    @staticmethod
    def _cell_in_bounds(shape, row, col):
        return (
            0 <= int(row) < int(shape[0])
            and 0 <= int(col) < int(shape[1])
        )

    def _costmap_endpoint_margin_cells(self, message):
        """Return the extra lethal-cell margin required around an endpoint.

        Costmap lethal cells already include the robot's inscribed footprint.
        The frontier ``clearance`` additionally reserves TEB's physical
        obstacle distance, so only the remainder is checked here.  The robot
        radius is read from the same global-costmap parameter when available;
        the fixed Pro3 footprint is the source-compatible fallback for pure
        fixtures and startup races.
        """
        resolution = max(1e-6, float(message.info.resolution))
        required = max(0.0, float(getattr(self, "clearance", 0.0)))
        radius = getattr(self, "costmap_inscribed_radius", None)
        if radius is None:
            try:
                radius = float(rospy.get_param(
                    "/move_base/global_costmap/robot_radius", 0.30,
                ))
            except Exception:
                radius = 0.30
            self.costmap_inscribed_radius = max(0.0, float(radius))
        return max(0, int(math.ceil(max(0.0, required - radius) / resolution)))

    def _endpoint_has_lethal_margin(self, data, message, cell):
        """Reject a goal whose required safety disk touches lethal cells."""
        margin_cells = self._costmap_endpoint_margin_cells(message)
        if margin_cells <= 0:
            return True
        row, col = int(cell[0]), int(cell[1])
        radius_squared = margin_cells * margin_cells
        rows, cols = data.shape
        for delta_row in range(-margin_cells, margin_cells + 1):
            for delta_col in range(-margin_cells, margin_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_squared:
                    continue
                neighbour_row = row + delta_row
                neighbour_col = col + delta_col
                if not self._cell_in_bounds(data.shape, neighbour_row, neighbour_col):
                    # A boundary without a known lethal cell is not enough to
                    # reject a candidate; the map/costmap route gate owns
                    # out-of-bounds handling.
                    continue
                if int(data[neighbour_row, neighbour_col]) >= 253:
                    return False
        return True

    def fresh_costmap(self):
        """Return a recent costmap snapshot, or None while it is unavailable."""
        message = self.costmap_msg
        if message is None or message.info.resolution <= 0.0:
            return None
        # Full grids are normally latched and remain unchanged while
        # costmap_2d publishes only incremental updates. Use the shared
        # lifecycle-clock age of the full-grid/update stream rather than the
        # static map stamp.
        age = (
            float("inf")
            if self.costmap_last_receive_wall <= 0.0
            else now_for(self) - self.costmap_last_receive_wall
        )
        if age > self.costmap_max_age:
            if (
                not getattr(self, "_costmap_explicitly_invalid", False)
                and getattr(self, "costmap_stationary", False)
            ):
                rospy.logwarn_throttle(
                    5.0,
                    "Global frontier retaining stationary costmap cache "
                    "age=%.2fs limit=%.2fs",
                    age,
                    self.costmap_max_age,
                )
                return message
            # The benchmark global costmap is a static-map projection whose
            # full grid may be published only once; its update stream can stay
            # quiet while SLAM and the robot continue moving.  This grid is
            # used only for structural candidate filtering.  Exact Navfn
            # validation below remains the executability gate, so retaining a
            # valid stale structure is safer than making exploration
            # permanently costmap-blind after the first quiet interval.
            if bool(getattr(self, "navfn_plan_validation", False)):
                rospy.logwarn_throttle(
                    5.0,
                    "Global frontier retaining structural costmap cache "
                    "age=%.2fs limit=%.2fs; Navfn remains authoritative",
                    age,
                    self.costmap_max_age,
                )
                return message
            rospy.logwarn_throttle(
                5.0,
                "Global frontier costmap stale age=%.2fs limit=%.2fs",
                age,
                self.costmap_max_age,
            )
            return None
        expected = int(message.info.height) * int(message.info.width)
        if expected <= 0 or len(message.data) != expected:
            rospy.logwarn_throttle(
                5.0,
                "Global frontier costmap shape invalid size=%dx%d cells=%d expected=%d",
                message.info.width,
                message.info.height,
                len(message.data),
                expected,
            )
            return None
        return message

    def costmap_steps(self, robot_xy):
        """Build the Navfn-compatible connected mask for the latest costmap.

        Values >= 253 are lethal in costmap_2d. Unknown cells remain
        traversable here because the production NavfnROS configuration allows
        unknown space; the frontier goal itself is still selected from the
        known SLAM-free mask.
        """
        message = self.fresh_costmap()
        if message is None:
            return None
        frame = (message.header.frame_id or "map").strip().lstrip("/") or "map"
        map_frame = (self.map_msg.header.frame_id or "map") if self.map_msg else "map"
        costmap_robot = robot_xy
        if frame != map_frame:
            costmap_robot = self.transform_xy(frame, map_frame, robot_xy[0], robot_xy[1])
            if costmap_robot is None:
                return None
        try:
            data = np.asarray(message.data, dtype=np.int16).reshape(
                int(message.info.height), int(message.info.width)
            )
        except (TypeError, ValueError):
            return None
        free = data < 253
        cell = self.world_cell(message, costmap_robot[0], costmap_robot[1])
        if cell is None:
            return None
        seed = self.nearest_seed(
            free,
            cell[0],
            cell[1],
            max(1, int(math.ceil(0.8 / message.info.resolution)),),
        )
        if seed is None:
            return None
        steps = self.bfs(
            free,
            seed,
            getattr(self, "planning_should_preempt", None),
        )
        if steps is None:
            return None
        return message, data, steps

    def cached_costmap_steps(self, robot_xy, now):
        """Return a throttled costmap connectivity snapshot.

        The global costmap publishes incremental updates while the map node is
        also running a Python BFS. Rebuilding that BFS on every one-second
        timer tick competes with Gazebo and TEB for CPU. A short-lived snapshot
        is sufficient for rejecting an obviously disconnected frontier; TEB's
        rolling local costmap still checks every command against current lidar.
        """
        if (
            self.cached_costmap_validation is not None
            and now - self.cached_costmap_validation_wall
            < self.costmap_validation_period
        ):
            return self.cached_costmap_validation
        validation = self.costmap_steps(robot_xy)
        if validation is not None:
            self.cached_costmap_validation = validation
            self.cached_costmap_validation_wall = now
            return validation
        # Keep a recent valid snapshot through a transient TF/costmap update;
        # ``fresh_costmap`` still expires it through the normal max-age guard.
        if (
            self.cached_costmap_validation is not None
            and now - self.cached_costmap_validation_wall
            <= self.costmap_max_age
        ):
            return self.cached_costmap_validation
        return None

    def candidate_costmap_distance(self, validation, x, y):
        """Return a candidate's costmap path distance, or None if invalid."""
        if validation is None:
            return None
        message, data, steps = validation
        frame = (message.header.frame_id or "map").strip().lstrip("/") or "map"
        map_frame = (self.map_msg.header.frame_id or "map") if self.map_msg else "map"
        target = (x, y)
        if frame != map_frame:
            target = self.transform_xy(frame, map_frame, x, y)
            if target is None:
                return None
        cell = self.world_cell(message, target[0], target[1])
        if cell is None or data[cell] >= 253 or steps[cell] < 0:
            return None
        if not self._endpoint_has_lethal_margin(data, message, cell):
            rospy.loginfo_throttle(
                5.0,
                "Global frontier rejected endpoint near lethal costmap cell "
                "goal=(%.2f,%.2f) margin_cells=%d",
                float(x),
                float(y),
                int(self._costmap_endpoint_margin_cells(message)),
            )
            return None
        return float(steps[cell]) * float(message.info.resolution)
