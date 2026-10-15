"""Run setup and low-level value helpers for ``NavigationMetrics``.

These methods are deliberately independent of ROS subscriptions.  Keeping
run-directory creation, startup snapshots, and numeric normalization here
makes the telemetry node's constructor and callback list easier to scan.
"""

import datetime
import json
import math
import os
import shutil
import time
from pathlib import Path

import rospy


class NavigationMetricsRuntimeMixin:
    @staticmethod
    def _resolve_path(value):
        path = Path(str(value)).expanduser()
        if path.is_absolute():
            return path
        return Path(os.environ.get("LSTE_WS", os.getcwd())) / path

    @staticmethod
    def _resolved_startup_params():
        """Read launch parameters after concurrently started nodes register them."""
        names = {
            "controller_mode": "/lste_cmd_vel_mux/initial_mode",
            "teb_max_vel_x": "/move_base/TebLocalPlannerROS/max_vel_x",
            "teb_max_vel_x_backwards": "/move_base/TebLocalPlannerROS/max_vel_x_backwards",
            "teb_max_vel_theta": "/move_base/TebLocalPlannerROS/max_vel_theta",
            "teb_xy_goal_tolerance": "/move_base/TebLocalPlannerROS/xy_goal_tolerance",
            "teb_min_obstacle_dist": "/move_base/TebLocalPlannerROS/min_obstacle_dist",
            "teb_inflation_dist": "/move_base/TebLocalPlannerROS/inflation_dist",
            "teb_homotopy_class_planning": "/move_base/TebLocalPlannerROS/enable_homotopy_class_planning",
            "teb_homotopy_simple_exploration": "/move_base/TebLocalPlannerROS/simple_exploration",
            "teb_homotopy_max_number_classes": "/move_base/TebLocalPlannerROS/max_number_classes",
            "teb_homotopy_viapoints_all_candidates": "/move_base/TebLocalPlannerROS/viapoints_all_candidates",
            "teb_controller_frequency": "/move_base/controller_frequency",
            "persistent_execution": "/move_base/persistent_execution",
            "persistent_frontier_lookahead_enabled": "/lste_teb_goal_bridge/persistent_frontier_lookahead_handoff_enabled",
            "persistent_frontier_lookahead_trigger_distance": "/lste_teb_goal_bridge/persistent_frontier_lookahead_trigger_distance",
            "persistent_frontier_admission_horizon": "/lste_teb_goal_bridge/persistent_frontier_admission_horizon",
            "persistent_frontier_admission_max_costmap_age": "/lste_teb_goal_bridge/persistent_frontier_admission_max_costmap_age",
            "persistent_frontier_curve_handoff_max_heading_deg": "/lste_teb_goal_bridge/persistent_frontier_curve_handoff_max_heading_deg",
            "teb_turn_supervisor_frequency": "/lste_teb_turn_supervisor/command_frequency",
            "teb_turn_supervisor_max_vel_theta": "/lste_teb_turn_supervisor/max_vel_theta",
            "teb_turn_supervisor_acc_lim_theta": "/lste_teb_turn_supervisor/acc_lim_theta",
            "teb_turn_supervisor_yaw_goal_tolerance": "/lste_teb_turn_supervisor/yaw_goal_tolerance",
            "teb_angular_switch_threshold": "/lste_cmd_vel_mux/teb_angular_sign_switch_threshold",
            "teb_angular_deadband": "/lste_cmd_vel_mux/teb_angular_deadband",
            "teb_target_early_handoff_distance": "/lste_teb_goal_bridge/target_early_handoff_distance",
            "teb_target_early_handoff_min_delta": "/lste_teb_goal_bridge/target_early_handoff_min_delta",
            "teb_in_place_replacement_max_delta": "/lste_teb_goal_bridge/in_place_replacement_max_delta",
            "teb_in_place_replacement_min_distance": "/lste_teb_goal_bridge/in_place_replacement_min_distance",
            "teb_in_place_replacement_max_distance": "/lste_teb_goal_bridge/in_place_replacement_max_distance",
            "teb_frontier_replacement_min_delta": "/lste_teb_goal_bridge/frontier_replacement_min_delta",
            "teb_allow_in_place_replacement": "/lste_teb_goal_bridge/allow_in_place_replacement",
            "teb_require_intent": "/lste_teb_goal_bridge/require_intent",
            "frontier_mission_endpoint_only": "/lste_global_frontier/mission_endpoint_only",
            "frontier_turn_execution_mode": "/lste_global_frontier/turn_execution_mode",
            "frontier_legacy_turn_connector_threshold_deg": "/lste_global_frontier/explicit_turn_connector_threshold_deg",
            "target_route_validation": "/lste_goal_manager/target_route_validation",
            "target_route_validation_service": "/lste_goal_manager/target_route_validation_service",
            "target_done_policy": "/lste_goal_manager/target_done_policy",
            "target_room_entry_min_x": "/lste_goal_manager/target_room_entry_min_x",
            "target_room_entry_min_y": "/lste_goal_manager/target_room_entry_min_y",
            "target_room_entry_max_x": "/lste_goal_manager/target_room_entry_max_x",
            "target_room_entry_max_y": "/lste_goal_manager/target_room_entry_max_y",
            "target_room_entry_margin": "/lste_goal_manager/target_room_entry_margin",
            "target_room_entry_min_hold_time": "/lste_goal_manager/target_room_entry_min_hold_time",
            "target_room_entry_require_crossing": "/lste_goal_manager/target_room_entry_require_crossing",
            "failure_evidence_enabled": "/lste_navigation_metrics/failure_evidence_enabled",
            "failure_evidence_sample_period": "/lste_navigation_metrics/failure_evidence_sample_period",
            "failure_evidence_pre_window": "/lste_navigation_metrics/failure_evidence_pre_window",
            "failure_evidence_post_window": "/lste_navigation_metrics/failure_evidence_post_window",
            "failure_evidence_zero_velocity_seconds": "/lste_navigation_metrics/failure_evidence_zero_velocity_seconds",
            "failure_evidence_no_progress_seconds": "/lste_navigation_metrics/failure_evidence_no_progress_seconds",
        }
        values = {key: None for key in names}
        pending = set(names)
        deadline = time.monotonic() + 3.0
        while pending and time.monotonic() < deadline:
            for key in tuple(pending):
                name = names[key]
                if rospy.has_param(name):
                    values[key] = rospy.get_param(name)
                    pending.remove(key)
            if pending:
                time.sleep(0.05)
        return values

    def _run_start_context(self):
        """Capture immutable task and perception settings once per run."""
        task_json = str(rospy.get_param("~task_json", "")).strip()
        task_definition = {
            "task_id": str(rospy.get_param("~task_id", "")).strip(),
            "json_path": task_json or None,
        }
        if task_json:
            try:
                with open(task_json, "r", encoding="utf-8") as stream:
                    parsed = json.load(stream)
                definition = parsed.get("task_parsed", parsed)
                if isinstance(definition, dict):
                    task_definition["definition"] = definition
                else:
                    task_definition["definition_error"] = "task JSON is not an object"
            except (OSError, ValueError, TypeError) as exc:
                task_definition["definition_error"] = str(exc)

        def param(name, default=None):
            return rospy.get_param("~" + name, default)

        return {
            "experiment": {
                "pipeline_config": param("pipeline_config"),
                "pipeline_config_sha256": param("pipeline_config_sha256"),
                "world": param("world"),
                "initial_pose": {
                    "x": param("pro3_spawn_x"),
                    "y": param("pro3_spawn_y"),
                    "z": param("pro3_spawn_z"),
                    "yaw": param("pro3_spawn_yaw"),
                },
                "online_slam_enabled": self._as_bool(param("online_slam_enabled", False)),
                "global_frontier_enabled": self._as_bool(param("global_frontier_enabled", False)),
                "startup_forward_enabled": self._as_bool(param("startup_forward_enabled", False)),
                "global_goal_source": param("global_goal_source"),
                "git_revision": param("git_revision"),
                "git_dirty": self._as_bool(param("git_dirty", False)),
            },
            "task_definition": task_definition,
            "detector": {
                "name": param("detector"),
                "backend": param("detector_backend"),
                "variant": param("detector_variant"),
                "runtime": param("detector_runtime"),
                "score_threshold": param("detector_score_threshold"),
                "nms_iou": param("detector_nms_iou"),
                "min_inference_interval": param("detector_min_inference_interval"),
            },
            "target_thresholds": {
                "done_policy": param("target_done_policy"),
                "room_entry": {
                    "min_x": param("target_room_entry_min_x"),
                    "min_y": param("target_room_entry_min_y"),
                    "max_x": param("target_room_entry_max_x"),
                    "max_y": param("target_room_entry_max_y"),
                    "margin": param("target_room_entry_margin"),
                    "min_hold_time": param("target_room_entry_min_hold_time"),
                    "require_crossing": param("target_room_entry_require_crossing"),
                },
                "follow_min_score": param("target_follow_min_score"),
                "follow_min_box_size": param("target_follow_min_box_size"),
                "follow_confirm_hits": param("target_follow_confirm_hits"),
                "follow_confirm_window": param("target_follow_confirm_window"),
                "done_min_score": param("target_done_min_score"),
                "done_min_box_width": param("target_done_min_box_width"),
                "done_min_box_height": param("target_done_min_box_height"),
                "done_min_fresh_hits": param("target_done_min_fresh_hits"),
                "done_min_hold_time": param("target_done_min_hold_time"),
                "done_require_close_evidence": param("target_done_require_close_evidence"),
                "done_require_confirmed_track": param("target_done_require_confirmed_track"),
                "done_require_approach_terminal": param("target_done_require_approach_terminal"),
            },
        }

    def _create_run_dir(self):
        self.log_root.mkdir(parents=True, exist_ok=True)
        cutoff = time.time() - self.retention_days * 86400.0
        for child in self.log_root.iterdir():
            try:
                if child.is_dir() and not child.is_symlink() and child.stat().st_mtime < cutoff:
                    shutil.rmtree(str(child))
            except OSError as exc:
                rospy.logwarn("Navigation metrics retention cleanup failed for %s: %s", child, exc)
        while True:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            directory = self.log_root / timestamp
            try:
                directory.mkdir()
                return timestamp, directory
            except FileExistsError:
                time.sleep(1.0)

    @staticmethod
    def _finite_min(values):
        finite = [
            float(value)
            for value in values
            if math.isfinite(float(value)) and float(value) > 0.01
        ]
        return min(finite) if finite else float("nan")

    @staticmethod
    def _as_bool(value):
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    @staticmethod
    def _normalize_label(value):
        return " ".join(str(value).strip().lower().replace("_", " ").split())

    @staticmethod
    def _csv_floats(value, count):
        try:
            values = [float(item.strip()) for item in str(value).split(",")]
        except (TypeError, ValueError):
            values = []
        if len(values) != count or not all(math.isfinite(item) for item in values):
            return tuple(0.0 for _ in range(count))
        return tuple(values)

    @staticmethod
    def _quaternion_conjugate(quaternion):
        x, y, z, w = (float(value) for value in quaternion)
        norm = x * x + y * y + z * z + w * w
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return (-x / norm, -y / norm, -z / norm, w / norm)

    @staticmethod
    def _rotate_point(quaternion, point):
        """Apply a quaternion rotation without adding a NumPy dependency."""
        qx, qy, qz, qw = (float(value) for value in quaternion)
        px, py, pz = (float(value) for value in point)
        norm = qx * qx + qy * qy + qz * qz + qw * qw
        if norm <= 1e-12:
            return (px, py, pz)
        qx, qy, qz, qw = (
            qx / math.sqrt(norm),
            qy / math.sqrt(norm),
            qz / math.sqrt(norm),
            qw / math.sqrt(norm),
        )
        tx = 2.0 * (qy * pz - qz * py)
        ty = 2.0 * (qz * px - qx * pz)
        tz = 2.0 * (qx * py - qy * px)
        return (
            px + qw * tx + (qy * tz - qz * ty),
            py + qw * ty + (qz * tx - qx * tz),
            pz + qw * tz + (qx * ty - qy * tx),
        )
