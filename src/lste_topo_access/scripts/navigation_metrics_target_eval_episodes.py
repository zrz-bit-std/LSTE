"""Exposure-episode lifecycle and run summary for target evaluation."""


class NavigationMetricsTargetEvaluationEpisodesMixin:
    """Maintain held visibility episodes and render evaluator-only metrics."""

    def _target_eval_end_episode_locked(self, reason):
        episode = self.target_eval_active_episode
        if episode is None:
            return
        episode["end_source_stamp"] = self.target_eval_last_exposed_stamp
        episode["end_reason"] = reason
        self.target_eval_completed_episodes.append(episode)
        self._write(
            "INFO",
            "target_exposure_ended",
            truth_source="gazebo_evaluation_only",
            episode_id=episode["id"],
            start_source_stamp=round(episode["start_source_stamp"], 4),
            end_source_stamp=(
                None if episode["end_source_stamp"] is None
                else round(episode["end_source_stamp"], 4)
            ),
            exposed_detector_frames=episode["frames"],
            matched_detector_frames=episode["matches"],
            matched=episode["matches"] > 0,
            end_reason=reason,
        )
        self.target_eval_active_episode = None
        self.target_eval_candidate_started_stamp = None

    def _target_eval_update_exposure_locked(self, source_stamp, exposed):
        """Maintain held frustum-exposure episodes on detector source frames."""
        started = False
        if exposed:
            self.target_eval_last_exposed_stamp = source_stamp
            if self.target_eval_active_episode is None:
                if self.target_eval_candidate_started_stamp is None:
                    self.target_eval_candidate_started_stamp = source_stamp
                elif (
                    source_stamp - self.target_eval_candidate_started_stamp
                    >= self.target_eval_exposure_hold
                ):
                    self.target_eval_episode_sequence += 1
                    self.target_eval_active_episode = {
                        "id": self.target_eval_episode_sequence,
                        "start_source_stamp": self.target_eval_candidate_started_stamp,
                        "frames": 0,
                        "matches": 0,
                        "first_match_source_stamp": None,
                    }
                    started = True
                    if self.target_eval_first_exposure_stamp is None:
                        self.target_eval_first_exposure_stamp = (
                            self.target_eval_candidate_started_stamp
                        )
                    self._write(
                        "INFO",
                        "target_exposure_started",
                        truth_source="gazebo_evaluation_only",
                        episode_id=self.target_eval_episode_sequence,
                        source_stamp=round(self.target_eval_candidate_started_stamp, 4),
                        hold_seconds=self.target_eval_exposure_hold,
                    )
        elif self.target_eval_active_episode is not None:
            last_exposed = self.target_eval_last_exposed_stamp
            if (
                last_exposed is not None
                and source_stamp - last_exposed >= self.target_eval_episode_gap
            ):
                self._target_eval_end_episode_locked("frustum_gap")
        else:
            self.target_eval_candidate_started_stamp = None
        return started

    def _target_eval_end_depth_episode_locked(self, reason):
        """Close one depth-confirmed visible episode without touching legacy data."""
        episode = self.target_eval_depth_active_episode
        if episode is None:
            return
        episode["end_source_stamp"] = self.target_eval_depth_last_visible_stamp
        episode["end_reason"] = reason
        self.target_eval_depth_completed_episodes.append(episode)
        self._write(
            "INFO",
            "target_depth_visible_exposure_ended",
            truth_source="gazebo_depth_evaluation_only",
            episode_id=episode["id"],
            start_source_stamp=round(episode["start_source_stamp"], 4),
            end_source_stamp=(
                None if episode["end_source_stamp"] is None
                else round(episode["end_source_stamp"], 4)
            ),
            depth_visible_detector_frames=episode["frames"],
            depth_visible_matched_detector_frames=episode["matches"],
            matched=episode["matches"] > 0,
            end_reason=reason,
        )
        self.target_eval_depth_active_episode = None
        self.target_eval_depth_candidate_started_stamp = None

    def _target_eval_update_depth_visibility_locked(self, source_stamp, visible):
        """Maintain held depth-confirmed visibility episodes separately.

        Frustum episodes above are part of the old metric contract and remain
        unchanged.  This parallel sequence makes an occluded-in-frustum target
        measurable without redefining historical recall values.
        """
        started = False
        if visible:
            self.target_eval_depth_last_visible_stamp = source_stamp
            if self.target_eval_depth_active_episode is None:
                if self.target_eval_depth_candidate_started_stamp is None:
                    self.target_eval_depth_candidate_started_stamp = source_stamp
                elif (
                    source_stamp - self.target_eval_depth_candidate_started_stamp
                    >= self.target_eval_exposure_hold
                ):
                    self.target_eval_depth_episode_sequence += 1
                    self.target_eval_depth_active_episode = {
                        "id": self.target_eval_depth_episode_sequence,
                        "start_source_stamp": self.target_eval_depth_candidate_started_stamp,
                        "frames": 0,
                        "matches": 0,
                        "first_match_source_stamp": None,
                    }
                    started = True
                    if self.target_eval_first_depth_visible_stamp is None:
                        self.target_eval_first_depth_visible_stamp = (
                            self.target_eval_depth_candidate_started_stamp
                        )
                    self._write(
                        "INFO",
                        "target_depth_visible_exposure_started",
                        truth_source="gazebo_depth_evaluation_only",
                        episode_id=self.target_eval_depth_episode_sequence,
                        source_stamp=round(
                            self.target_eval_depth_candidate_started_stamp, 4
                        ),
                        hold_seconds=self.target_eval_exposure_hold,
                    )
        elif self.target_eval_depth_active_episode is not None:
            last_visible = self.target_eval_depth_last_visible_stamp
            if (
                last_visible is not None
                and source_stamp - last_visible >= self.target_eval_episode_gap
            ):
                self._target_eval_end_depth_episode_locked("depth_visibility_gap")
        else:
            self.target_eval_depth_candidate_started_stamp = None
        return started

    def _target_eval_snapshot_locked(self):
        episodes = list(self.target_eval_completed_episodes)
        if self.target_eval_active_episode is not None:
            episodes.append(self.target_eval_active_episode)
        episode_matches = sum(1 for episode in episodes if episode["matches"] > 0)
        depth_episodes = list(self.target_eval_depth_completed_episodes)
        if self.target_eval_depth_active_episode is not None:
            depth_episodes.append(self.target_eval_depth_active_episode)
        depth_episode_matches = sum(
            1 for episode in depth_episodes if episode["matches"] > 0
        )
        first_exposure = self.target_eval_first_exposure_stamp
        first_match = self.target_eval_first_match_stamp
        first_depth_visible = self.target_eval_first_depth_visible_stamp
        first_depth_visible_match = self.target_eval_first_depth_visible_match_stamp
        return {
            "enabled": self.target_eval_enabled,
            "truth_source": "gazebo_evaluation_only",
            "target_model": self.target_eval_target_model,
            "camera_ready": self.target_eval_camera is not None,
            "model_state_ready": bool(self.target_eval_states),
            "exposure_episodes": len(episodes),
            "matched_exposure_episodes": episode_matches,
            "exposed_detector_frames": self.target_eval_exposed_frames,
            "matched_detector_frames": self.target_eval_matched_frames,
            "frame_geometric_recall": (
                None if self.target_eval_exposed_frames == 0 else round(
                    self.target_eval_matched_frames / self.target_eval_exposed_frames, 4
                )
            ),
            "episode_geometric_recall": (
                None if not episodes else round(episode_matches / len(episodes), 4)
            ),
            "depth_validation_enabled": self.target_eval_depth_enabled,
            "depth_topic": self.target_eval_depth_topic or None,
            "depth_frame_ready": bool(self.target_eval_depth_frames),
            "depth_frames_received": self.target_eval_depth_frames_received,
            "depth_frames_decoded": self.target_eval_depth_frames_decoded,
            "depth_decode_failures": self.target_eval_depth_decode_failures,
            "depth_evaluated_frustum_detector_frames": (
                self.target_eval_depth_evaluated_frames
            ),
            "depth_visible_exposure_episodes": len(depth_episodes),
            "depth_matched_visible_exposure_episodes": depth_episode_matches,
            "depth_visible_detector_frames": self.target_eval_depth_visible_frames,
            "depth_visible_matched_detector_frames": (
                self.target_eval_depth_visible_matched_frames
            ),
            "frame_depth_visible_recall": (
                None if self.target_eval_depth_visible_frames == 0 else round(
                    self.target_eval_depth_visible_matched_frames
                    / self.target_eval_depth_visible_frames,
                    4,
                )
            ),
            "episode_depth_visible_recall": (
                None if not depth_episodes else round(
                    depth_episode_matches / len(depth_episodes), 4
                )
            ),
            "depth_occluded_detector_frames": self.target_eval_depth_occluded_frames,
            "depth_inconsistent_detector_frames": (
                self.target_eval_depth_inconsistent_frames
            ),
            "depth_unavailable_detector_frames": (
                self.target_eval_depth_unavailable_frames
            ),
            "unmatched_target_candidate_frames": (
                self.target_eval_unmatched_target_candidate_frames
            ),
            "target_candidates_outside_exposure": self.target_eval_outside_exposure_candidates,
            "unavailable_detector_frames": self.target_eval_unavailable_frames,
            "tf_failures": self.target_eval_tf_failures,
            "evaluated_detector_frames": self.target_eval_evaluated_frames,
            "detector_delivery_latency_mean_seconds": (
                None if self.target_eval_delivery_latency_count == 0 else round(
                    self.target_eval_delivery_latency_total
                    / self.target_eval_delivery_latency_count,
                    4,
                )
            ),
            "detector_delivery_latency_max_seconds": (
                None if self.target_eval_delivery_latency_count == 0
                else round(self.target_eval_delivery_latency_max, 4)
            ),
            "first_exposure_source_stamp": first_exposure,
            "first_match_source_stamp": first_match,
            "first_depth_visible_source_stamp": first_depth_visible,
            "first_depth_visible_match_source_stamp": first_depth_visible_match,
            "first_match_after_exposure_seconds": (
                None if first_exposure is None or first_match is None
                else round(max(0.0, first_match - first_exposure), 4)
            ),
            "first_match_after_depth_visible_seconds": (
                None
                if first_depth_visible is None or first_depth_visible_match is None
                else round(
                    max(0.0, first_depth_visible_match - first_depth_visible), 4
                )
            ),
            "active_episode_id": (
                None if self.target_eval_active_episode is None
                else self.target_eval_active_episode["id"]
            ),
            "active_depth_visible_episode_id": (
                None if self.target_eval_depth_active_episode is None
                else self.target_eval_depth_active_episode["id"]
            ),
        }
