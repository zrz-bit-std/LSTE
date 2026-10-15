"""Detection-side target evaluation for navigation telemetry.

The mixin owns detector-message matching and frame-level evidence. Gazebo
projection/depth helpers live in ``navigation_metrics_target_eval``; the
host node supplies those helpers plus the telemetry lock and counters.
"""

import time

import rospy


class NavigationMetricsDetectionMixin:
    def _evaluate_target_detection_locked(self, message):
        """Measure geometric detector recall without influencing any ROS control path."""
        if not self.target_eval_enabled:
            return
        source_stamp = float(message.header.stamp.to_sec())
        if source_stamp <= 0.0:
            self._target_eval_note_unavailable_locked("detection_stamp_unavailable")
            return
        if (
            self.target_eval_last_source_stamp is not None
            and source_stamp <= self.target_eval_last_source_stamp
        ):
            return
        self.target_eval_last_source_stamp = source_stamp
        if not self.target_eval_target_model:
            self._target_eval_note_unavailable_locked("target_eval_configuration_incomplete")
            return
        if not self.target_eval_states:
            self._target_eval_note_unavailable_locked("gazebo_model_state_unavailable")
            return
        now_wall = time.monotonic()
        newest_age = now_wall - self.target_eval_states[-1]["wall_time"]
        if newest_age > self.target_eval_max_state_age:
            self._target_eval_note_unavailable_locked(
                "gazebo_model_state_stale", model_state_rx_age_seconds=round(newest_age, 4)
            )
            return
        state = min(
            self.target_eval_states,
            key=lambda item: abs(float(item["ros_time"]) - source_stamp),
        )
        source_state_age = abs(float(state["ros_time"]) - source_stamp)
        if source_state_age > self.target_eval_max_state_age:
            self._target_eval_note_unavailable_locked(
                "gazebo_model_state_timestamp_mismatch",
                source_to_state_age_seconds=round(source_state_age, 4),
            )
            return
        projection, unavailable_reason = self._target_eval_projected_box_locked(
            source_stamp, state
        )
        if unavailable_reason is not None:
            self._target_eval_note_unavailable_locked(unavailable_reason)
            return

        self.target_eval_evaluated_frames += 1
        delivery_latency = max(0.0, rospy.Time.now().to_sec() - source_stamp)
        self.target_eval_delivery_latency_total += delivery_latency
        self.target_eval_delivery_latency_max = max(
            self.target_eval_delivery_latency_max, delivery_latency
        )
        self.target_eval_delivery_latency_count += 1
        exposed = bool(projection["exposed"])
        started_episode = self._target_eval_update_exposure_locked(source_stamp, exposed)
        depth_visibility = self._target_eval_depth_visibility_locked(
            source_stamp, projection
        )
        depth_visible = bool(depth_visibility["visible"])
        started_depth_episode = self._target_eval_update_depth_visibility_locked(
            source_stamp, depth_visible
        )
        if exposed:
            if depth_visibility["available"]:
                self.target_eval_depth_evaluated_frames += 1
                if depth_visibility["state"] == "occluded":
                    self.target_eval_depth_occluded_frames += 1
                elif depth_visibility["state"] == "inconsistent":
                    self.target_eval_depth_inconsistent_frames += 1
            elif depth_visibility["state"] == "unavailable":
                self.target_eval_depth_unavailable_frames += 1

        task_id = str(message.task_id).strip()
        task_matches = not self.task_id or not task_id or task_id == self.task_id
        target_candidates = []
        for detection in message.target_dets:
            label = self._normalize_label(detection.label)
            score = float(detection.score)
            if not task_matches or score < self.target_eval_min_score:
                continue
            target_candidates.append({
                "label": label,
                "score": score,
                "box": (
                    float(detection.cx) - 0.5 * float(detection.w),
                    float(detection.cy) - 0.5 * float(detection.h),
                    float(detection.cx) + 0.5 * float(detection.w),
                    float(detection.cy) + 0.5 * float(detection.h),
                ),
            })
        predicted_box_px = projection["predicted_box_px"]
        predicted_box_normalized = projection["predicted_box_normalized"]
        best_candidate = None
        best_iou = 0.0
        if predicted_box_normalized is not None:
            for candidate in target_candidates:
                iou = self._target_eval_bbox_iou(
                    predicted_box_normalized, candidate["box"]
                )
                if best_candidate is None or iou > best_iou:
                    best_candidate = candidate
                    best_iou = iou
        matched = bool(
            exposed
            and best_candidate is not None
            and best_iou >= self.target_eval_min_match_iou
        )
        depth_confirmed_matched = bool(depth_visible and matched)
        episode = self.target_eval_active_episode
        if exposed and episode is not None:
            episode["frames"] += 1
            self.target_eval_exposed_frames += 1
            if matched:
                episode["matches"] += 1
                self.target_eval_matched_frames += 1
                if episode["first_match_source_stamp"] is None:
                    episode["first_match_source_stamp"] = source_stamp
                if self.target_eval_first_match_stamp is None:
                    self.target_eval_first_match_stamp = source_stamp
                    self._write(
                        "INFO",
                        "target_first_spatial_match",
                        truth_source="gazebo_evaluation_only",
                        episode_id=episode["id"],
                        source_stamp=round(source_stamp, 4),
                        iou=round(best_iou, 4),
                        score=round(best_candidate["score"], 4),
                        label=best_candidate["label"],
                    )
        depth_episode = self.target_eval_depth_active_episode
        if depth_visible and depth_episode is not None:
            depth_episode["frames"] += 1
            self.target_eval_depth_visible_frames += 1
            if depth_confirmed_matched:
                depth_episode["matches"] += 1
                self.target_eval_depth_visible_matched_frames += 1
                if depth_episode["first_match_source_stamp"] is None:
                    depth_episode["first_match_source_stamp"] = source_stamp
                if self.target_eval_first_depth_visible_match_stamp is None:
                    self.target_eval_first_depth_visible_match_stamp = source_stamp
                    self._write(
                        "INFO",
                        "target_first_depth_visible_spatial_match",
                        truth_source="gazebo_depth_evaluation_only",
                        episode_id=depth_episode["id"],
                        source_stamp=round(source_stamp, 4),
                        iou=round(best_iou, 4),
                        score=round(best_candidate["score"], 4),
                        label=best_candidate["label"],
                    )
        if target_candidates and not exposed:
            self.target_eval_outside_exposure_candidates += 1
        if target_candidates and not matched:
            self.target_eval_unmatched_target_candidate_frames += 1

        # Periodic frame evidence keeps logs bounded at long-running detector
        # rates, while every match/false positive and exposure transition stays
        # individually auditable.
        interesting = (
            started_episode
            or started_depth_episode
            or matched
            or depth_confirmed_matched
            or bool(target_candidates and not matched)
        )
        if interesting or now_wall - self.target_eval_last_frame_log_wall >= 1.0:
            self._write(
                "INFO",
                "target_eval_detection_frame",
                truth_source="gazebo_evaluation_only",
                task_id=task_id or None,
                task_matches=task_matches,
                prompt_a=str(message.prompt_a),
                source_stamp=round(source_stamp, 4),
                detector_delivery_latency_seconds=round(delivery_latency, 4),
                model_state_source_age_seconds=round(source_state_age, 4),
                model_state_rx_age_seconds=round(newest_age, 4),
                frustum_exposed=exposed,
                exposure_reason=projection["reason"],
                active_episode_id=None if episode is None else episode["id"],
                depth_visibility_state=depth_visibility["state"],
                depth_visibility_reason=depth_visibility["reason"],
                depth_visible=depth_visible,
                depth_available=depth_visibility["available"],
                depth_source_age_seconds=(
                    None
                    if depth_visibility["source_age_seconds"] is None
                    else round(depth_visibility["source_age_seconds"], 4)
                ),
                depth_valid_samples=depth_visibility["valid_samples"],
                depth_matching_samples=depth_visibility["matching_samples"],
                depth_foreground_samples=depth_visibility["foreground_samples"],
                depth_background_samples=depth_visibility["background_samples"],
                depth_sample_pixels=depth_visibility["sample_pixels"],
                observed_depth_median_m=(
                    None
                    if depth_visibility["observed_depth_median_m"] is None
                    else round(depth_visibility["observed_depth_median_m"], 4)
                ),
                expected_depth_min_m=(
                    None
                    if depth_visibility["expected_depth_min_m"] is None
                    else round(depth_visibility["expected_depth_min_m"], 4)
                ),
                expected_depth_max_m=(
                    None
                    if depth_visibility["expected_depth_max_m"] is None
                    else round(depth_visibility["expected_depth_max_m"], 4)
                ),
                depth_tolerance_m=(
                    None
                    if depth_visibility["tolerance_m"] is None
                    else round(depth_visibility["tolerance_m"], 4)
                ),
                active_depth_visible_episode_id=(
                    None if depth_episode is None else depth_episode["id"]
                ),
                center_depth_m=round(projection["center_depth_m"], 4),
                center_px=(
                    None if projection.get("center_px") is None
                    else [round(value, 2) for value in projection["center_px"]]
                ),
                predicted_box_px=(
                    None if predicted_box_px is None
                    else [round(value, 2) for value in predicted_box_px]
                ),
                predicted_box_normalized=(
                    None if predicted_box_normalized is None
                    else [round(value, 5) for value in predicted_box_normalized]
                ),
                target_candidate_count=len(target_candidates),
                best_candidate=(
                    None if best_candidate is None else {
                        "label": best_candidate["label"],
                        "score": round(best_candidate["score"], 4),
                        "box_normalized": [
                            round(value, 5) for value in best_candidate["box"]
                        ],
                    }
                ),
                best_iou=round(best_iou, 4),
                matched=matched,
                depth_confirmed_matched=depth_confirmed_matched,
            )
            self.target_eval_last_frame_log_wall = now_wall

    def on_detections(self, message):
        target = None
        if message.target_dets:
            target = max(message.target_dets, key=lambda item: float(item.score))
        with self.lock:
            self.detector_messages += 1
            self._evaluate_target_detection_locked(message)
            if target is not None:
                self.target_messages += 1
                stamp = message.header.stamp.to_sec()
                if stamp != self.last_detection_stamp:
                    self.last_detection_stamp = stamp
                    if self.target_first_seen_ros is None:
                        self.target_first_seen_ros = rospy.Time.now().to_sec()
                        self._write(
                            "INFO",
                            "target_acquired",
                            latency_seconds=round(self.target_first_seen_ros - self.start_ros, 3),
                            score=round(float(target.score), 4),
                            center=[round(float(target.cx), 4), round(float(target.cy), 4)],
                        )
                    self._write(
                        "INFO",
                        "target_observation",
                        score=round(float(target.score), 4),
                        center=[round(float(target.cx), 4), round(float(target.cy), 4)],
                        box=[round(float(target.w), 4), round(float(target.h), 4)],
                        detector_stamp=stamp,
                    )
                self.target = {
                    "label": target.label,
                    "score": float(target.score),
                    "cx": float(target.cx),
                    "cy": float(target.cy),
                    "w": float(target.w),
                    "h": float(target.h),
                }
            else:
                self.target = None


