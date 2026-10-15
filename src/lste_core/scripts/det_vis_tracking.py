"""Frame selection and lightweight tracker support for detection visualization.

This module intentionally owns only display-track state.  The ROS node owns
subscriptions and publication, while ``DetectionVisualizer`` supplies the
configuration and state attributes referenced by this mixin.
"""

import copy
import math
import time

import cv2
import numpy as np
import rospy


class DetectionTrackingMixin:
    """Maintain display boxes between detector results without ROS callbacks."""

    @staticmethod
    def _stamp_ns(header):
        try:
            return int(header.stamp.to_nsec())
        except Exception:
            return 0

    def _tracking_image(self, image):
        if self.tracking_scale >= 0.999:
            return image
        return cv2.resize(
            image,
            None,
            fx=self.tracking_scale,
            fy=self.tracking_scale,
            interpolation=cv2.INTER_LINEAR,
        )

    @staticmethod
    def _make_tracker():
        return cv2.TrackerCSRT_create()

    def _select_detection_frame(self, detections):
        """Use the exact detector source image instead of the latest camera frame."""
        stamp_ns = self._stamp_ns(detections.header)
        if not stamp_ns or not self._display_frame_history:
            rospy.logwarn_throttle(2.0, "No camera frame available for detection visualization")
            return
        history = list(self._display_frame_history)
        frame_stamp, frame_header, frame = min(history, key=lambda item: abs(item[0] - stamp_ns))
        if frame_stamp != stamp_ns:
            rospy.logwarn_throttle(
                2.0,
                "Exact detector source frame is no longer in visualization history; skipping result",
            )
            return
        self._display_image = frame
        self._display_image_header = frame_header

    def _start_trackers(self, detections):
        """Initialize CSRT on the source frame and replay buffered frames to now."""
        self.display_detections = copy.deepcopy(detections)
        self._trackers = []
        self._tracking_started_at = time.monotonic()
        if not self.tracking_enabled or not self._frame_history:
            return

        stamp_ns = self._stamp_ns(detections.header)
        if not stamp_ns:
            return
        history = list(self._frame_history)
        frame_index = min(range(len(history)), key=lambda index: abs(history[index][0] - stamp_ns))
        matched_stamp, source_frame = history[frame_index]
        if abs(matched_stamp - stamp_ns) > int(self.max_tracking_lag * 1e9):
            rospy.logwarn_throttle(2.0, "Detection image is no longer in the tracker history")
            return

        self._previous_tracking_image = source_frame.copy()
        height, width = source_frame.shape[:2]
        for kind in ("target_dets", "env_dets"):
            for detection in getattr(self.display_detections, kind):
                self._append_tracker(source_frame, detection, kind)
        rospy.loginfo("Initialized %d CSRT trackers for the latest detection", len(self._trackers))

        # A detector returns after its source frame was captured. Replay the
        # camera frames in between before drawing the next live frame.
        replay_frames = history[frame_index + 1::self.tracking_update_stride]
        if history[-1:] and (not replay_frames or replay_frames[-1][0] != history[-1][0]):
            replay_frames.append(history[-1])
        for _, frame in replay_frames:
            self._update_trackers(frame, width, height)

    def _append_tracker(self, source_frame, detection, kind):
        """Create one CSRT display track without changing the other tracks."""
        height, width = source_frame.shape[:2]
        x = max(0.0, (float(detection.cx) - 0.5 * float(detection.w)) * width)
        y = max(0.0, (float(detection.cy) - 0.5 * float(detection.h)) * height)
        box_width = min(max(2.0, float(detection.w) * width), width - x)
        box_height = min(max(2.0, float(detection.h) * height), height - y)
        if box_width <= 1.0 or box_height <= 1.0:
            return False
        try:
            tracker = self._make_tracker()
            if not tracker.init(source_frame, (x, y, box_width, box_height)):
                return False
            self._trackers.append(
                {
                    "tracker": tracker,
                    "det": detection,
                    "kind": kind,
                    "edge_exit_count": 0,
                    "edge_exit_direction": None,
                    "edge_exit_offset": (0.0, 0.0),
                    "edge_exit_speed": 0.0,
                    "last_center": (x + 0.5 * box_width, y + 0.5 * box_height),
                    "last_bbox": (x, y, box_width, box_height),
                    "flow_lost_count": 0,
                    "last_measurement_time": time.monotonic(),
                }
            )
            return True
        except cv2.error as exc:
            rospy.logwarn_throttle(2.0, "Failed to initialize CSRT tracker: %s", exc)
            return False

    def _associate_trackers(self, detections):
        """Blend frequent WeDetect measurements into persistent display tracks."""
        stamp_ns = self._stamp_ns(detections.header)
        if stamp_ns and stamp_ns == self._last_associated_detection_stamp:
            return
        if stamp_ns:
            self._last_associated_detection_stamp = stamp_ns

        if not self.tracking_enabled or not self._frame_history:
            self.display_detections = copy.deepcopy(detections)
            return
        if self.display_detections is None or not self._trackers or not stamp_ns:
            self._start_trackers(detections)
            return

        history = list(self._frame_history)
        frame_index = min(range(len(history)), key=lambda index: abs(history[index][0] - stamp_ns))
        matched_stamp, source_frame = history[frame_index]
        if abs(matched_stamp - stamp_ns) > int(self.max_tracking_lag * 1e9):
            rospy.logwarn_throttle(2.0, "Detection image is no longer in the tracker history")
            return

        now = time.monotonic()
        self._tracking_started_at = now
        matched_tracks = set()
        height, width = source_frame.shape[:2]
        for kind in ("target_dets", "env_dets"):
            for measurement in getattr(detections, kind):
                track_index = self._best_matching_track(kind, measurement, matched_tracks)
                if track_index is None:
                    display_detection = copy.deepcopy(measurement)
                    getattr(self.display_detections, kind).append(display_detection)
                    self._append_tracker(source_frame, display_detection, kind)
                    continue
                track = self._trackers[track_index]
                matched_tracks.add(track_index)
                self._blend_measurement(track, measurement, width, height)
                track["last_measurement_time"] = now

        expired = {
            id(track["det"])
            for track in self._trackers
            if now - track.get("last_measurement_time", now) > self.associated_track_ttl
        }
        if expired:
            self._trackers = [track for track in self._trackers if id(track["det"]) not in expired]
            self._remove_display_tracks(expired)

    def _best_matching_track(self, kind, measurement, used_indices):
        best_index = None
        best_iou = self.associated_iou_threshold
        for index, track in enumerate(self._trackers):
            if index in used_indices or track.get("kind") != kind:
                continue
            if (track["det"].label or "").lower() != (measurement.label or "").lower():
                continue
            overlap = self._normalized_iou(track["det"], measurement)
            if overlap > best_iou:
                best_index = index
                best_iou = overlap
        return best_index

    @staticmethod
    def _normalized_iou(first, second):
        first_x1 = float(first.cx) - 0.5 * float(first.w)
        first_y1 = float(first.cy) - 0.5 * float(first.h)
        second_x1 = float(second.cx) - 0.5 * float(second.w)
        second_y1 = float(second.cy) - 0.5 * float(second.h)
        inter_x1 = max(first_x1, second_x1)
        inter_y1 = max(first_y1, second_y1)
        inter_x2 = min(first_x1 + float(first.w), second_x1 + float(second.w))
        inter_y2 = min(first_y1 + float(first.h), second_y1 + float(second.h))
        intersection = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)
        union = float(first.w) * float(first.h) + float(second.w) * float(second.h) - intersection
        return intersection / union if union > 1e-9 else 0.0

    def _blend_measurement(self, track, measurement, width, height):
        detection = track["det"]
        alpha = min(1.0, max(0.0, self.associated_measurement_alpha))
        for field in ("cx", "cy", "w", "h"):
            value = (1.0 - alpha) * getattr(detection, field) + alpha * getattr(measurement, field)
            setattr(detection, field, float(value))
        detection.score = float(measurement.score)
        x = max(0.0, (float(detection.cx) - 0.5 * float(detection.w)) * width)
        y = max(0.0, (float(detection.cy) - 0.5 * float(detection.h)) * height)
        box_width = min(max(2.0, float(detection.w) * width), width - x)
        box_height = min(max(2.0, float(detection.h) * height), height - y)
        if box_width > 1.0 and box_height > 1.0:
            track["last_bbox"] = (x, y, box_width, box_height)
            track["last_center"] = (x + 0.5 * box_width, y + 0.5 * box_height)

    def _update_trackers(self, tracking_image, full_width, full_height):
        if (
            self._tracking_started_at is not None
            and time.monotonic() - self._tracking_started_at > self.max_tracking_age
        ):
            self._clear_display_tracks()
            self._trackers = []
            return
        if not self._trackers:
            self._previous_tracking_image = tracking_image.copy()
            return

        height, width = tracking_image.shape[:2]
        image_flow = self._global_image_flow(self._previous_tracking_image, tracking_image)
        self._previous_tracking_image = tracking_image.copy()
        alive = []
        expired = set()
        for track in self._trackers:
            detection = track["det"]
            try:
                ok, bbox = track["tracker"].update(tracking_image)
            except cv2.error:
                ok = False

            last_bbox = track["last_bbox"]
            x, y, box_width, box_height = last_bbox
            if ok:
                candidate = tuple(float(value) for value in bbox)
                tracker_motion = (
                    candidate[0] + 0.5 * candidate[2] - track["last_center"][0],
                    candidate[1] + 0.5 * candidate[3] - track["last_center"][1],
                )
                if self._flow_explains_stalled_tracker(image_flow, tracker_motion):
                    x, y, box_width, box_height = self._apply_image_flow(last_bbox, image_flow)
                else:
                    x, y, box_width, box_height = candidate
                    track["flow_lost_count"] = 0
            elif image_flow is not None and track["flow_lost_count"] < self.tracking_flow_max_lost_frames:
                x, y, box_width, box_height = self._apply_image_flow(last_bbox, image_flow)
                track["flow_lost_count"] += 1
                rospy.logdebug_throttle(1.0, "CSRT lost; using image-flow box fallback")
            else:
                expired.add(id(detection))
                continue

            center = (x + 0.5 * box_width, y + 0.5 * box_height)
            previous_center = track["last_center"]
            motion = (center[0] - previous_center[0], center[1] - previous_center[1])
            direction = self._outward_edge_direction(x, y, box_width, box_height, width, height, motion)
            active_direction = track["edge_exit_direction"]
            if direction is not None:
                if active_direction != direction:
                    track["edge_exit_direction"] = direction
                    track["edge_exit_count"] = 0
                    track["edge_exit_offset"] = (0.0, 0.0)
                    track["edge_exit_speed"] = self._outward_speed(direction, motion)
                active_direction = direction
            elif active_direction is not None and self._moves_back_into_frame(active_direction, motion):
                track["edge_exit_direction"] = None
                track["edge_exit_count"] = 0
                track["edge_exit_offset"] = (0.0, 0.0)
                track["edge_exit_speed"] = 0.0
                active_direction = None

            draw_x, draw_y = x, y
            if active_direction is not None:
                speed = max(
                    1.0,
                    track["edge_exit_speed"] * 0.8,
                    self._outward_speed(active_direction, motion),
                )
                track["edge_exit_speed"] = speed
                offset_x, offset_y = track["edge_exit_offset"]
                step_x, step_y = self._edge_step(active_direction, speed)
                offset_x += step_x
                offset_y += step_y
                track["edge_exit_offset"] = (offset_x, offset_y)
                track["edge_exit_count"] += 1
                draw_x = x + offset_x
                draw_y = y + offset_y

            track["last_center"] = center
            track["last_bbox"] = (x, y, box_width, box_height)
            if (
                active_direction is not None
                and (
                    self._visible_fraction(draw_x, draw_y, box_width, box_height, width, height)
                    <= self.tracking_edge_min_visible_fraction
                    or track["edge_exit_count"] >= self.tracking_edge_exit_frames
                )
            ):
                expired.add(id(detection))
                continue

            detection.cx = float((draw_x + 0.5 * box_width) / width)
            detection.cy = float((draw_y + 0.5 * box_height) / height)
            detection.w = float(box_width / width)
            detection.h = float(box_height / height)
            alive.append(track)

        self._trackers = alive
        if expired:
            self._remove_display_tracks(expired)

    def _global_image_flow(self, previous, current):
        """Estimate global 2D image motion from sparse optical flow."""
        if not self.tracking_flow_enabled or previous is None or current is None:
            return None
        if previous.shape[:2] != current.shape[:2]:
            return None
        previous_gray = cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY)
        current_gray = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)
        points = cv2.goodFeaturesToTrack(
            previous_gray,
            maxCorners=120,
            qualityLevel=0.01,
            minDistance=6,
            blockSize=7,
        )
        if points is None or len(points) < self.tracking_flow_min_points:
            return None
        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            previous_gray,
            current_gray,
            points,
            None,
            winSize=(21, 21),
            maxLevel=3,
        )
        if next_points is None or status is None:
            return None
        valid = status.reshape(-1).astype(bool)
        if error is not None:
            valid &= error.reshape(-1) < 25.0
        vectors = (next_points.reshape(-1, 2) - points.reshape(-1, 2))[valid]
        if len(vectors) < self.tracking_flow_min_points:
            return None
        median = np.median(vectors, axis=0)
        deviations = np.linalg.norm(vectors - median, axis=1)
        inliers = vectors[deviations <= max(0.5, 2.5 * np.median(deviations))]
        if len(inliers) < self.tracking_flow_min_points:
            return None
        return tuple(float(value) for value in np.median(inliers, axis=0))

    @staticmethod
    def _apply_image_flow(bbox, image_flow):
        x, y, box_width, box_height = bbox
        dx, dy = image_flow
        return x + dx, y + dy, box_width, box_height

    @staticmethod
    def _flow_explains_stalled_tracker(image_flow, tracker_motion):
        if image_flow is None:
            return False
        return math.hypot(*image_flow) >= 0.75 and math.hypot(*tracker_motion) <= 0.25

    def _outward_edge_direction(self, x, y, box_width, box_height, width, height, motion):
        """Return the edge touched while the tracked center is moving outward."""
        dx, dy = motion
        touch = self.tracking_edge_touch_pixels
        candidates = []
        if x <= touch and dx < -0.2:
            candidates.append((abs(dx), "left"))
        if x + box_width >= width - touch and dx > 0.2:
            candidates.append((abs(dx), "right"))
        if y <= touch and dy < -0.2:
            candidates.append((abs(dy), "top"))
        if y + box_height >= height - touch and dy > 0.2:
            candidates.append((abs(dy), "bottom"))
        return max(candidates)[1] if candidates else None

    def _outward_speed(self, direction, motion):
        dx, dy = motion
        component = {"left": -dx, "right": dx, "top": -dy, "bottom": dy}[direction]
        return max(0.0, component * self.tracking_edge_exit_speed_scale)

    @staticmethod
    def _edge_step(direction, speed):
        return {
            "left": (-speed, 0.0),
            "right": (speed, 0.0),
            "top": (0.0, -speed),
            "bottom": (0.0, speed),
        }[direction]

    @staticmethod
    def _moves_back_into_frame(direction, motion):
        dx, dy = motion
        return {
            "left": dx > 0.2,
            "right": dx < -0.2,
            "top": dy > 0.2,
            "bottom": dy < -0.2,
        }[direction]

    @staticmethod
    def _visible_fraction(x, y, box_width, box_height, width, height):
        if box_width <= 0.0 or box_height <= 0.0:
            return 0.0
        visible_width = max(0.0, min(x + box_width, width) - max(x, 0.0))
        visible_height = max(0.0, min(y + box_height, height) - max(y, 0.0))
        return (visible_width * visible_height) / (box_width * box_height)

    def _clear_display_tracks(self):
        if self.display_detections is None:
            return
        del self.display_detections.target_dets[:]
        del self.display_detections.env_dets[:]

    def _remove_display_tracks(self, expired_ids):
        if self.display_detections is None:
            return
        self.display_detections.target_dets[:] = [
            detection
            for detection in self.display_detections.target_dets
            if id(detection) not in expired_ids
        ]
        self.display_detections.env_dets[:] = [
            detection
            for detection in self.display_detections.env_dets
            if id(detection) not in expired_ids
        ]
