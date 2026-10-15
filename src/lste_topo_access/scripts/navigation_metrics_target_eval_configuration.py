"""Configuration metadata and unavailable-evidence reporting for target evaluation."""

import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class NavigationMetricsTargetEvaluationConfigurationMixin:
    """Expose the immutable evaluator contract and bounded warning events."""

    def _target_eval_configuration(self):
        """Return the simulator-only evaluator contract recorded at run start."""
        return {
            "enabled": self.target_eval_enabled,
            "truth_source": "gazebo_evaluation_only",
            "target_model": self.target_eval_target_model,
            "task_id": self.task_id or None,
            "robot_model": self.target_eval_robot_model,
            "world_frame": self.target_eval_world_frame,
            "odom_frame": self.target_eval_odom_frame,
            "camera_frame": self.target_eval_camera_frame or "from_camera_info",
            "camera_info_topic": self.target_eval_camera_info_topic,
            "depth_validation": {
                "enabled": self.target_eval_depth_enabled,
                "topic": self.target_eval_depth_topic or None,
                "transport": "sensor_msgs/CompressedImage compressedDepth",
                "alignment_contract": (
                    "depth pixels are normalized against the CameraInfo image; "
                    "the configured depth topic must be registered/aligned to it"
                ),
                "max_source_age_s": self.target_eval_depth_max_source_age,
                "sample_grid": self.target_eval_depth_sample_grid,
                "min_valid_samples": self.target_eval_depth_min_valid_samples,
                "min_matching_samples": self.target_eval_depth_min_matching_samples,
                "absolute_tolerance_m": self.target_eval_depth_abs_tolerance,
                "relative_tolerance": self.target_eval_depth_relative_tolerance,
                "decoder_available": cv2 is not None and np is not None,
            },
            "detections_topic": self.target_eval_detections_topic,
            "configured_target_labels_metadata": sorted(self.target_eval_labels),
            "candidate_contract": (
                "LsteDetections.target_dets for the matching task_id; "
                "boxes and projected truth are normalized xyxy"
            ),
            "target_center_m": list(self.target_eval_center),
            "target_size_m": list(self.target_eval_size),
            "min_depth_m": self.target_eval_min_depth,
            "max_depth_m": self.target_eval_max_depth,
            "edge_margin_px": self.target_eval_edge_margin,
            "max_gazebo_state_age_s": self.target_eval_max_state_age,
            "exposure_hold_s": self.target_eval_exposure_hold,
            "episode_gap_s": self.target_eval_episode_gap,
            "min_match_iou": self.target_eval_min_match_iou,
            "min_score": self.target_eval_min_score,
            "control_contract": "observer_only_no_publishers_or_control_params",
        }

    def _target_eval_note_unavailable_locked(self, reason, **fields):
        """Count unavailable detector frames without creating a log flood."""
        self.target_eval_unavailable_frames += 1
        now = time.monotonic()
        if (
            reason != self.target_eval_last_unavailable_reason
            or now - self.target_eval_last_unavailable_log_wall >= 2.0
        ):
            self._write(
                "WARN",
                "target_eval_unavailable",
                truth_source="gazebo_evaluation_only",
                reason=reason,
                **fields
            )
            self.target_eval_last_unavailable_reason = reason
            self.target_eval_last_unavailable_log_wall = now
