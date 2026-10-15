"""Benchmark-only truth observers kept separate from navigation policy."""

from __future__ import annotations

import math
from pathlib import Path

import rospy
import tf
from tf.transformations import euler_from_quaternion

from office_benchmark_coverage import evaluate_map_known_fraction, load_manifest_truth


class NavigationMetricsBenchmarkTruthMixin:
    """Project the immutable office floor mask into each current SLAM map."""

    def _initialize_benchmark_truth(self):
        self.benchmark_coverage = None
        self._benchmark_truth = None
        self._benchmark_truth_error = None
        manifest_value = str(rospy.get_param("~benchmark_manifest", "")).strip()
        level = str(rospy.get_param("~benchmark_level", "")).strip()
        if not manifest_value or not level:
            return
        try:
            self._benchmark_truth = load_manifest_truth(
                Path(self._resolve_path(manifest_value)), level
            )
            self.benchmark_coverage = {
                "definition": self._benchmark_truth["definition"],
                "status": "awaiting_map_transform",
                "level": level,
                "truth_sample_count": len(self._benchmark_truth["points"]),
            }
        except (OSError, ValueError) as exc:
            self._benchmark_truth_error = str(exc)
            self.benchmark_coverage = {
                "status": "configuration_error",
                "error": self._benchmark_truth_error,
            }

    def _initialize_benchmark_collision_truth(self):
        """Track physical contact edges, never controller safety events."""
        value = str(
            rospy.get_param("~benchmark_collision_truth_enabled", "false")
        ).strip().lower()
        self.benchmark_collision_truth_enabled = value in ("1", "true", "yes", "on")
        self._benchmark_contact_partners = set()
        self._benchmark_contact_messages = 0
        self._benchmark_collision_events = 0
        self.benchmark_collision_truth = {
            "enabled": self.benchmark_collision_truth_enabled,
            "topic": "/lste/benchmark/contact_states",
            "status": (
                "awaiting_contact_sensor"
                if self.benchmark_collision_truth_enabled else "not_configured"
            ),
            "contact_messages": 0,
            "collision_events": None,
        }

    def on_benchmark_contacts(self, message):
        """Count the start of non-ground physical contacts once per episode."""
        if not self.benchmark_collision_truth_enabled:
            return
        with self.lock:
            self._benchmark_contact_messages += 1
            partners = set()
            for state in message.states:
                for name in (state.collision1_name, state.collision2_name):
                    normalized = str(name or "")
                    if not normalized or "ground_plane" in normalized:
                        continue
                    # The sensor is attached to the Pro3 base collision; the
                    # other robot collision name is never an external impact.
                    if normalized.startswith("pro3::"):
                        continue
                    partners.add(normalized)
            new_partners = sorted(partners - self._benchmark_contact_partners)
            self._benchmark_contact_partners = partners
            self._benchmark_collision_events += len(new_partners)
            self.benchmark_collision_truth = {
                "enabled": True,
                "topic": "/lste/benchmark/contact_states",
                "status": "measured",
                "contact_messages": self._benchmark_contact_messages,
                "collision_events": self._benchmark_collision_events,
                "active_partners": sorted(partners),
            }
            for partner in new_partners:
                self._write(
                    "WARN",
                    "benchmark_collision",
                    collision_partner=partner,
                    collision_event_count=self._benchmark_collision_events,
                )

    def _update_benchmark_coverage_locked(self, message):
        if self._benchmark_truth is None:
            return
        map_frame = (message.header.frame_id or "map").strip().lstrip("/") or "map"
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                map_frame, "odom", rospy.Time(0)
            )
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            self.benchmark_coverage["status"] = "awaiting_map_transform"
            return
        origin = message.info.origin
        origin_yaw = euler_from_quaternion((
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        ))[2]
        transform_yaw = euler_from_quaternion(rotation)[2]
        try:
            self.benchmark_coverage = evaluate_map_known_fraction(
                self._benchmark_truth,
                occupancy_data=message.data,
                width=int(message.info.width),
                height=int(message.info.height),
                resolution=float(message.info.resolution),
                map_origin=(origin.position.x, origin.position.y, origin_yaw),
                map_from_odom=(translation[0], translation[1], transform_yaw),
            )
        except ValueError as exc:
            self.benchmark_coverage = {
                "definition": self._benchmark_truth["definition"],
                "status": "invalid_map",
                "error": str(exc),
            }
