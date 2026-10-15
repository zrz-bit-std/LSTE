"""Target geometry evaluator configuration and runtime state."""

from collections import deque

import rospy


class NavigationMetricsTargetStateMixin:
    """Keep evaluation-only target state separate from navigation telemetry."""

    def _initialize_target_evaluation_state(self):
        # This is intentionally an evaluator, never a source of navigation
        # inputs. Gazebo has no world -> odom TF in this workspace, so each
        # ModelStates sample records the simultaneous world/odom base poses.
        # It lets logs distinguish "the target was not in the camera frustum"
        # from "the target was in view but the detector did not match it".
        self.target_eval_enabled = self._as_bool(
            rospy.get_param("~target_eval_enabled", False)
        )
        self.target_eval_target_model = str(
            rospy.get_param("~target_eval_gazebo_target_model", "")
        ).strip()
        self.target_eval_robot_model = str(
            rospy.get_param("~target_eval_gazebo_robot_model", "pro3")
        ).strip()
        self.target_eval_world_frame = str(
            rospy.get_param("~target_eval_world_frame", "world")
        ).strip().lstrip("/") or "world"
        self.target_eval_odom_frame = str(
            rospy.get_param("~target_eval_odom_frame", "odom")
        ).strip().lstrip("/") or "odom"
        self.target_eval_camera_frame = str(
            rospy.get_param("~target_eval_camera_frame", "")
        ).strip().lstrip("/")
        self.target_eval_camera_info_topic = str(
            rospy.get_param("~target_eval_camera_info_topic", "/kinect/hd/camera_info")
        ).strip()
        # This is the depth-compressed transport advertised by the Gazebo
        # Kinect in this workspace.  It is sampled solely for evaluation: a
        # Gazebo target projected into the RGB frustum counts as *visible*
        # only when a time-matched depth image contains target-range support
        # at that projection.  None of this data reaches GoalManager, Navfn,
        # TEB, or the actuator mux.
        self.target_eval_depth_enabled = self._as_bool(
            rospy.get_param("~target_eval_depth_enabled", True)
        )
        self.target_eval_depth_topic = str(
            rospy.get_param(
                "~target_eval_depth_topic",
                "/kinect/hd/image_color_rect/compressedDepth",
            )
        ).strip()
        self.target_eval_depth_max_source_age = max(
            0.01,
            float(rospy.get_param("~target_eval_depth_max_source_age_s", 0.25)),
        )
        self.target_eval_depth_sample_grid = min(9, max(
            2, int(rospy.get_param("~target_eval_depth_sample_grid", 5))
        ))
        self.target_eval_depth_min_valid_samples = max(
            1, int(rospy.get_param("~target_eval_depth_min_valid_samples", 3))
        )
        self.target_eval_depth_min_matching_samples = max(
            1, int(rospy.get_param("~target_eval_depth_min_matching_samples", 2))
        )
        self.target_eval_depth_abs_tolerance = max(
            0.0,
            float(rospy.get_param("~target_eval_depth_abs_tolerance_m", 0.12)),
        )
        self.target_eval_depth_relative_tolerance = max(
            0.0,
            float(rospy.get_param("~target_eval_depth_relative_tolerance", 0.05)),
        )
        self.target_eval_detections_topic = str(
            rospy.get_param("~target_eval_detections_topic", "/lste/detections")
        ).strip()
        # Retain configured aliases as run metadata only.  A detection node
        # already separates the current prompt's candidates into
        # ``target_dets`` and ``env_dets``. Re-filtering ``target_dets`` here
        # with a static alias list can silently discard valid detections when
        # a task prompt changes or a shell splits an alias containing spaces.
        self.target_eval_labels = {
            self._normalize_label(value)
            for value in str(
                rospy.get_param("~target_eval_target_labels", "")
            ).split(",")
            if self._normalize_label(value)
        }
        self.target_eval_center = self._csv_floats(
            rospy.get_param("~target_eval_target_center_m", "0,0,0"), 3
        )
        self.target_eval_size = self._csv_floats(
            rospy.get_param("~target_eval_target_size_m", "0.1,0.1,0.1"), 3
        )
        self.target_eval_min_depth = max(
            0.01, float(rospy.get_param("~target_eval_min_depth_m", 0.20))
        )
        self.target_eval_max_depth = max(
            self.target_eval_min_depth,
            float(rospy.get_param("~target_eval_max_depth_m", 12.0)),
        )
        self.target_eval_edge_margin = max(
            0.0, float(rospy.get_param("~target_eval_edge_margin_px", 8.0))
        )
        self.target_eval_max_state_age = max(
            0.05, float(rospy.get_param("~target_eval_max_gazebo_state_age_s", 0.50))
        )
        self.target_eval_exposure_hold = max(
            0.0, float(rospy.get_param("~target_eval_exposure_hold_s", 0.25))
        )
        self.target_eval_episode_gap = max(
            0.05, float(rospy.get_param("~target_eval_episode_gap_s", 0.75))
        )
        self.target_eval_min_match_iou = min(1.0, max(
            0.0, float(rospy.get_param("~target_eval_min_match_iou", 0.10))
        ))
        self.target_eval_min_score = min(1.0, max(
            0.0, float(rospy.get_param("~target_eval_min_score", 0.20))
        ))
        self.target_eval_camera = None
        self.target_eval_depth_frames = deque(maxlen=10)
        self.target_eval_depth_frames_received = 0
        self.target_eval_depth_frames_decoded = 0
        self.target_eval_depth_decode_failures = 0
        self.target_eval_depth_last_decode_reason = ""
        self.target_eval_odom_pose = None
        self.target_eval_states = deque(maxlen=80)
        self.target_eval_last_model_wall = None
        self.target_eval_last_source_stamp = None
        self.target_eval_last_exposed_stamp = None
        self.target_eval_candidate_started_stamp = None
        self.target_eval_active_episode = None
        self.target_eval_completed_episodes = []
        self.target_eval_episode_sequence = 0
        self.target_eval_exposed_frames = 0
        self.target_eval_matched_frames = 0
        # These are deliberately distinct from the established geometric
        # counters above.  A frustum projection only proves line-of-sight in
        # ideal geometry; depth support proves that the rendered camera has a
        # surface at the target's expected range.
        self.target_eval_depth_evaluated_frames = 0
        self.target_eval_depth_visible_frames = 0
        self.target_eval_depth_visible_matched_frames = 0
        self.target_eval_depth_occluded_frames = 0
        self.target_eval_depth_inconsistent_frames = 0
        self.target_eval_depth_unavailable_frames = 0
        self.target_eval_depth_last_visible_stamp = None
        self.target_eval_depth_candidate_started_stamp = None
        self.target_eval_depth_active_episode = None
        self.target_eval_depth_completed_episodes = []
        self.target_eval_depth_episode_sequence = 0
        self.target_eval_evaluated_frames = 0
        # A target-stream candidate without a spatial match is not necessarily
        # a false positive: geometric frustum exposure does not prove that the
        # target is unoccluded in RGB. Keep the literal metric name.
        self.target_eval_unmatched_target_candidate_frames = 0
        self.target_eval_outside_exposure_candidates = 0
        self.target_eval_unavailable_frames = 0
        self.target_eval_tf_failures = 0
        self.target_eval_delivery_latency_total = 0.0
        self.target_eval_delivery_latency_max = 0.0
        self.target_eval_delivery_latency_count = 0
        self.target_eval_first_exposure_stamp = None
        self.target_eval_first_match_stamp = None
        self.target_eval_first_depth_visible_stamp = None
        self.target_eval_first_depth_visible_match_stamp = None
        self.target_eval_last_unavailable_reason = ""
        self.target_eval_last_unavailable_log_wall = 0.0
        self.target_eval_last_frame_log_wall = 0.0
