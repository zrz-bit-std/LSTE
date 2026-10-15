"""Compressed-depth decoding and visibility evidence for target evaluation."""

import math
import struct
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class NavigationMetricsTargetEvaluationDepthMixin:
    """Decode registered depth images and classify target visibility."""

    def _target_eval_decode_depth_message(self, message):
        """Decode a compressedDepth message into a depth image plus metre scale.

        The ROS ``compressed_depth_image_transport`` convention writes 16UC1
        images as a PNG containing millimetres.  Its 32FC1 form prepends a
        twelve-byte ``ConfigHeader`` and stores inverse depth in PNG.  Gazebo
        normally publishes the former, but accepting both avoids silently
        weakening the experiment when the sensor encoding changes.
        """
        if cv2 is None or np is None:
            return None, None, "depth_decoder_unavailable"
        format_text = str(message.format or "").strip()
        lower_format = format_text.lower()
        if "compresseddepth" not in lower_format:
            return None, None, "unsupported_depth_transport"
        payload = bytes(message.data)
        if not payload:
            return None, None, "empty_depth_payload"
        encoding = lower_format.split(";", 1)[0].strip()
        png_offset = 0
        depth_quant_a = None
        depth_quant_b = None
        if "32fc1" in encoding:
            # ``ConfigHeader`` is C++ ``int + float + float``. It has no
            # padding on the ROS platforms used here (twelve bytes).
            if len(payload) <= 12:
                return None, None, "truncated_32fc1_depth_header"
            try:
                _format, depth_quant_a, depth_quant_b = struct.unpack(
                    "<iff", payload[:12]
                )
            except struct.error:
                return None, None, "invalid_32fc1_depth_header"
            if not math.isfinite(depth_quant_a) or depth_quant_a <= 0.0:
                return None, None, "invalid_32fc1_depth_quantization"
            png_offset = 12
        encoded = np.frombuffer(payload[png_offset:], dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        if image is None or image.ndim != 2 or image.size == 0:
            return None, None, "depth_png_decode_failed"
        if "16uc1" in encoding:
            if image.dtype != np.uint16:
                return None, None, "unexpected_16uc1_depth_dtype"
            return image, 0.001, None
        if "32fc1" in encoding:
            if image.dtype != np.uint16:
                return None, None, "unexpected_32fc1_depth_dtype"
            denominator = image.astype(np.float32) - float(depth_quant_b)
            with np.errstate(divide="ignore", invalid="ignore"):
                depth = float(depth_quant_a) / denominator
            depth[image == 0] = np.nan
            return depth, 1.0, None
        # Some image_transport versions omit the encoding prefix but still
        # produce a valid 16-bit PNG. Treat only that unambiguous case as mm.
        if image.dtype == np.uint16:
            return image, 0.001, None
        return None, None, "unsupported_depth_encoding"

    def on_target_eval_depth(self, message):
        """Keep a small timestamp-indexed depth history for detector evidence."""
        if not self.target_eval_enabled or not self.target_eval_depth_enabled:
            return
        source_stamp = float(message.header.stamp.to_sec())
        with self.lock:
            self.target_eval_depth_frames_received += 1
        if source_stamp <= 0.0:
            decoded, scale, reason = None, None, "depth_stamp_unavailable"
        else:
            decoded, scale, reason = self._target_eval_decode_depth_message(message)
        with self.lock:
            if reason is not None:
                self.target_eval_depth_decode_failures += 1
                if reason != self.target_eval_depth_last_decode_reason:
                    self._write(
                        "WARN",
                        "target_eval_depth_decode_unavailable",
                        truth_source="gazebo_depth_evaluation_only",
                        depth_topic=self.target_eval_depth_topic,
                        reason=reason,
                        format=str(message.format or ""),
                    )
                    self.target_eval_depth_last_decode_reason = reason
                return
            self.target_eval_depth_frames.append({
                "ros_time": source_stamp,
                "wall_time": time.monotonic(),
                "image": decoded,
                "scale": float(scale),
                "encoding": str(message.format or ""),
                "width": int(decoded.shape[1]),
                "height": int(decoded.shape[0]),
            })
            self.target_eval_depth_frames_decoded += 1
            self.target_eval_depth_last_decode_reason = ""

    def _target_eval_depth_visibility_locked(self, source_stamp, projection):
        """Validate frustum exposure against aligned, time-matched depth.

        The result is intentionally tri-state.  ``unavailable`` means the
        experiment has insufficient evidence and must not be counted as an
        occlusion; ``occluded`` means a majority of usable support pixels are
        materially closer than the target's rendered depth interval.
        """
        result = {
            "state": "not_in_frustum",
            "visible": False,
            "available": False,
            "reason": "target_not_frustum_exposed",
            "source_age_seconds": None,
            "valid_samples": 0,
            "matching_samples": 0,
            "foreground_samples": 0,
            "background_samples": 0,
            "sample_pixels": 0,
            "observed_depth_median_m": None,
            "expected_depth_min_m": projection.get("target_depth_min_m"),
            "expected_depth_max_m": projection.get("target_depth_max_m"),
            "tolerance_m": None,
        }
        if not projection.get("exposed"):
            return result
        if not self.target_eval_depth_enabled:
            result.update(
                state="disabled",
                reason="depth_validation_disabled",
            )
            return result
        if cv2 is None or np is None:
            result.update(state="unavailable", reason="depth_decoder_unavailable")
            return result
        if not self.target_eval_depth_frames:
            result.update(state="unavailable", reason="depth_frame_unavailable")
            return result
        depth_frame = min(
            self.target_eval_depth_frames,
            key=lambda item: abs(float(item["ros_time"]) - source_stamp),
        )
        source_age = abs(float(depth_frame["ros_time"]) - source_stamp)
        result["source_age_seconds"] = source_age
        if source_age > self.target_eval_depth_max_source_age:
            result.update(
                state="unavailable",
                reason="depth_timestamp_mismatch",
            )
            return result
        expected_min = projection.get("target_depth_min_m")
        expected_max = projection.get("target_depth_max_m")
        predicted_box = projection.get("predicted_box_px")
        camera = self.target_eval_camera
        if (
            expected_min is None or expected_max is None or predicted_box is None
            or camera is None or expected_min <= 0.0 or expected_max < expected_min
        ):
            result.update(state="unavailable", reason="target_depth_projection_invalid")
            return result
        depth_width = int(depth_frame["width"])
        depth_height = int(depth_frame["height"])
        if depth_width <= 0 or depth_height <= 0:
            result.update(state="unavailable", reason="depth_dimensions_invalid")
            return result
        left = max(0.0, min(float(camera["width"]), float(predicted_box[0])))
        top = max(0.0, min(float(camera["height"]), float(predicted_box[1])))
        right = max(0.0, min(float(camera["width"]), float(predicted_box[2])))
        bottom = max(0.0, min(float(camera["height"]), float(predicted_box[3])))
        if right <= left or bottom <= top:
            result.update(state="unavailable", reason="depth_support_outside_image")
            return result

        # Sample the interior rather than the hard bbox edge: projection of a
        # rotated cuboid contains background at the corners, whereas interior
        # points retain a useful amount of rendered-target support. Normalised
        # coordinates preserve alignment when a registered depth image is a
        # different resolution from the RGB CameraInfo image.
        sample_pixels = set()
        grid = self.target_eval_depth_sample_grid
        for row in range(grid):
            rgb_y = top + (bottom - top) * (0.20 + 0.60 * row / (grid - 1))
            depth_y = int((rgb_y / float(camera["height"])) * depth_height)
            depth_y = min(depth_height - 1, max(0, depth_y))
            for column in range(grid):
                rgb_x = left + (right - left) * (0.20 + 0.60 * column / (grid - 1))
                depth_x = int((rgb_x / float(camera["width"])) * depth_width)
                depth_x = min(depth_width - 1, max(0, depth_x))
                sample_pixels.add((depth_x, depth_y))
        values = []
        image = depth_frame["image"]
        scale = float(depth_frame["scale"])
        for depth_x, depth_y in sample_pixels:
            value = float(image[depth_y, depth_x]) * scale
            if (
                math.isfinite(value)
                and self.target_eval_min_depth <= value <= self.target_eval_max_depth
            ):
                values.append(value)
        result["sample_pixels"] = len(sample_pixels)
        result["valid_samples"] = len(values)
        if len(values) < self.target_eval_depth_min_valid_samples:
            result.update(state="unavailable", reason="insufficient_valid_depth_samples")
            return result
        values.sort()
        middle = len(values) // 2
        result["observed_depth_median_m"] = (
            values[middle]
            if len(values) % 2
            else 0.5 * (values[middle - 1] + values[middle])
        )
        tolerance = max(
            self.target_eval_depth_abs_tolerance,
            self.target_eval_depth_relative_tolerance * max(expected_max, 0.01),
        )
        lower_bound = expected_min - tolerance
        upper_bound = expected_max + tolerance
        matching = sum(lower_bound <= value <= upper_bound for value in values)
        foreground = sum(value < lower_bound for value in values)
        background = sum(value > upper_bound for value in values)
        result.update(
            available=True,
            matching_samples=matching,
            foreground_samples=foreground,
            background_samples=background,
            tolerance_m=tolerance,
        )
        if matching >= self.target_eval_depth_min_matching_samples:
            result.update(state="visible", visible=True, reason="depth_support_matches_target")
            return result
        occlusion_quorum = max(
            self.target_eval_depth_min_valid_samples,
            int(math.ceil(0.70 * len(values))),
        )
        if foreground >= occlusion_quorum:
            result.update(state="occluded", reason="foreground_depth_occludes_target")
            return result
        result.update(state="inconsistent", reason="depth_support_outside_target_range")
        return result
