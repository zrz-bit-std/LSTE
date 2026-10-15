"""Path metrics and straight-corridor classification for the observer."""

import json
import math
import time


class NavigationMetricsPathMixin:
    """Observe planner paths and derive read-only path-quality telemetry."""

    @staticmethod
    def _grid_stats(message):
        values = list(message.data)
        return {
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "free": values.count(0),
            "occupied": sum(1 for value in values if value >= 50),
            "unknown": values.count(-1),
        }

    @staticmethod
    def _path_stats(message):
        poses = message.poses
        length = 0.0
        for previous, current in zip(poses, poses[1:]):
            length += math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
        endpoint = None
        if poses:
            endpoint = [
                round(float(poses[-1].pose.position.x), 3),
                round(float(poses[-1].pose.position.y), 3),
            ]
        # Navfn publishes a path whose first pose is the current planner
        # origin. Its arc length is therefore the best available remaining
        # distance estimate when no synchronized feedback projection exists.
        return {
            "poses": len(poses),
            "length": round(length, 3),
            "remaining_estimate_m": round(length, 3),
            "endpoint": endpoint,
        }

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(float(angle)), math.cos(float(angle)))

    @staticmethod
    def _path_geometry(message):
        """Return the latest path in its native frame for observer-only math."""
        return {
            "frame": (message.header.frame_id or "odom").strip().lstrip("/") or "odom",
            "points": [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in message.poses
            ],
        }

    def _path_straightness_locked(self, geometry, label):
        """Evaluate one TEB path without using it to control the vehicle."""
        state = {
            "is_straight": False,
            "reason": "no_%s" % label,
            "plan_curvature_rad": None,
            "heading_error_rad": None,
            "nearest_path_distance_m": None,
            "frame": None,
        }
        if not geometry or len(geometry.get("points", ())) < 2:
            return state
        frame = geometry["frame"]
        pose = self._pose_xy_in_frame_locked(frame)
        if pose is None:
            state.update(reason="pose_transform_unavailable", frame=frame)
            return state
        points = geometry["points"]
        nearest_index = min(
            range(len(points)),
            key=lambda index: (points[index][0] - pose[0]) ** 2
            + (points[index][1] - pose[1]) ** 2,
        )
        nearest_distance = math.hypot(
            points[nearest_index][0] - pose[0],
            points[nearest_index][1] - pose[1],
        )
        start_index = nearest_index
        while start_index + 1 < len(points) and math.hypot(
            points[start_index + 1][0] - points[start_index][0],
            points[start_index + 1][1] - points[start_index][1],
        ) <= 1e-4:
            start_index += 1
        if start_index + 1 >= len(points):
            state.update(
                reason="path_terminal",
                nearest_path_distance_m=round(nearest_distance, 4),
                frame=frame,
            )
            return state
        first = points[start_index]
        second = points[start_index + 1]
        initial_heading = math.atan2(second[1] - first[1], second[0] - first[0])
        travelled = math.hypot(second[0] - first[0], second[1] - first[1])
        horizon_index = start_index + 1
        while (
            horizon_index + 1 < len(points)
            and travelled < self.straight_path_lookahead
        ):
            previous = points[horizon_index]
            horizon_index += 1
            current = points[horizon_index]
            travelled += math.hypot(current[0] - previous[0], current[1] - previous[1])
        horizon_previous = points[max(start_index, horizon_index - 1)]
        horizon = points[horizon_index]
        horizon_heading = math.atan2(
            horizon[1] - horizon_previous[1],
            horizon[0] - horizon_previous[0],
        )
        curvature = abs(self._normalize_angle(horizon_heading - initial_heading))
        heading_error = abs(self._normalize_angle(initial_heading - pose[2]))
        aligned = (
            curvature <= self.straight_path_max_curvature
            and heading_error <= self.straight_path_max_heading_error
        )
        state.update(
            is_straight=aligned,
            reason=(
                "straight_aligned"
                if aligned
                else "planned_bend"
                if curvature > self.straight_path_max_curvature
                else "heading_alignment"
            ),
            plan_curvature_rad=round(curvature, 4),
            heading_error_rad=round(heading_error, 4),
            nearest_path_distance_m=round(nearest_distance, 4),
            frame=frame,
        )
        return state

    def _straight_path_state_locked(self, now):
        """Classify a settled straight segment in both TEB path horizons.

        Navfn's global route can become straight one cycle before TEB has
        finished a turn in its selected local trajectory. Counting that tail
        as corridor wobble creates a false regression. A command therefore
        qualifies only when the global route *and* the current local TEB path
        are straight and aligned with the base.
        """
        if (
            self.straight_path_state is not None
            and now - self.straight_path_last_eval_wall < self.straight_path_eval_period
        ):
            return self.straight_path_state
        self.straight_path_last_eval_wall = now
        global_state = self._path_straightness_locked(
            self.teb_global_plan_geometry, "teb_global_plan"
        )
        local_state = self._path_straightness_locked(
            self.teb_local_plan_geometry, "teb_local_plan"
        )
        is_straight = (
            bool(global_state["is_straight"])
            and bool(local_state["is_straight"])
        )
        state = {
            "is_straight": is_straight,
            "reason": (
                "straight_aligned"
                if is_straight
                else "global_%s" % global_state["reason"]
                if not global_state["is_straight"]
                else "local_%s" % local_state["reason"]
            ),
            # Preserve the historical global keys for existing log readers.
            "plan_curvature_rad": global_state["plan_curvature_rad"],
            "heading_error_rad": global_state["heading_error_rad"],
            "nearest_path_distance_m": global_state["nearest_path_distance_m"],
            "frame": global_state["frame"],
            "local_plan_curvature_rad": local_state["plan_curvature_rad"],
            "local_heading_error_rad": local_state["heading_error_rad"],
            "local_nearest_path_distance_m": local_state["nearest_path_distance_m"],
            "local_frame": local_state["frame"],
        }
        self.straight_path_state = state
        return state

    def _on_path(self, source, message):
        with self.lock:
            now = time.monotonic()
            stats = self._path_stats(message)
            setattr(self, source, stats)
            timestamp_attr = {
                "navfn_plan_stats": "last_navfn_plan_wall",
                "global_planner_plan_stats": "last_global_planner_plan_wall",
                "teb_global_plan_stats": "last_teb_global_plan_wall",
                "teb_local_plan_stats": "last_teb_local_plan_wall",
            }.get(source)
            if timestamp_attr:
                setattr(self, timestamp_attr, now)
            if source == "teb_global_plan_stats":
                self.teb_global_plan_geometry = self._path_geometry(message)
            elif source == "teb_local_plan_stats":
                self.teb_local_plan_geometry = self._path_geometry(message)
            previous = self.last_plan_log_wall.get(source, 0.0)
            if now - previous >= 1.0:
                self.last_plan_log_wall[source] = now
                self._write("INFO", "planner_path", planner=source, **stats)

    def on_navfn_plan(self, message):
        self._on_path("navfn_plan_stats", message)

    def on_persistent_plan_event(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            self.persistent_plan_received = max(
                self.persistent_plan_received,
                int(payload.get("received", 0) or 0),
            )
            self.persistent_plan_equivalent_retained = max(
                self.persistent_plan_equivalent_retained,
                int(payload.get("equivalent_retained", 0) or 0),
            )
            self.persistent_plan_installed = max(
                self.persistent_plan_installed,
                int(payload.get("installed", 0) or 0),
            )
            self.persistent_plan_last_event = (
                str(payload.get("event", "unknown")).strip() or "unknown"
            )
            try:
                self.persistent_plan_route_version = max(
                    self.persistent_plan_route_version,
                    int(payload.get("route_version", 0) or 0),
                )
            except (TypeError, ValueError):
                pass
            geometry_hash = str(payload.get("geometry_hash", "")).strip()
            if geometry_hash:
                self.persistent_plan_geometry_hash = geometry_hash
            self._write(
                "INFO",
                "persistent_plan_event",
                event_name=self.persistent_plan_last_event,
                received=self.persistent_plan_received,
                equivalent_retained=self.persistent_plan_equivalent_retained,
                installed=self.persistent_plan_installed,
                route_version=self.persistent_plan_route_version,
                geometry_hash=self.persistent_plan_geometry_hash,
                poses=int(payload.get("poses", 0) or 0),
                goal=payload.get("goal"),
            )

    def on_global_planner_plan(self, message):
        self._on_path("global_planner_plan_stats", message)

    def on_teb_global_plan(self, message):
        self._on_path("teb_global_plan_stats", message)

    def on_teb_local_plan(self, message):
        self._on_path("teb_local_plan_stats", message)
