#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WeDetect-only LSTE detection node.

This node is independent from GroundingDINO and keeps the same ROS output
contract so it can be selected without changing the rest of LSTE.
"""

import os
import time

import cv2
import rospy
import torch

from utils.detection_node_common import DetectionNodeBase
from utils.wedetect_backend import (
    DEFAULT_CHECKPOINT,
    DEFAULT_LANGUAGE_MODEL,
    DEFAULT_SOURCE_DIR,
    DEFAULT_TRT_CACHE,
    DEFAULT_VISION_ONNX,
    WeDetectBackend,
    WeDetectUnavailable,
)


class WeDetectDetNode(DetectionNodeBase):
    detector_name = "wedetect"

    def __init__(self):
        super().__init__()
        expected_env = rospy.get_param("~expected_conda_env", "dino")
        current_env = os.environ.get("CONDA_DEFAULT_ENV", "")
        if expected_env and current_env != expected_env:
            rospy.logwarn("Current CONDA_DEFAULT_ENV=%s; expected %s", current_env, expected_env)
        label_map = rospy.get_param("~wedetect_label_map", {})
        if not isinstance(label_map, dict):
            raise ValueError("~wedetect_label_map must map English labels to Chinese labels")
        tile_search = rospy.get_param("~wedetect_small_object_tile_search_enabled", True)
        self.small_object_tile_search_enabled = str(tile_search).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.small_object_tile_grid = max(
            1, int(rospy.get_param("~wedetect_small_object_tile_grid", 2))
        )
        tile_layout = str(
            rospy.get_param("~wedetect_small_object_tile_layout", "uniform_grid")
        ).strip().lower()
        if tile_layout not in ("uniform_grid", "center_band"):
            rospy.logwarn(
                "Unsupported WeDetect small-object tile layout %r; using uniform_grid.",
                tile_layout,
            )
            tile_layout = "uniform_grid"
        self.small_object_tile_layout = tile_layout
        self.small_object_tile_overlap = min(0.45, max(
            0.0, float(rospy.get_param("~wedetect_small_object_tile_overlap", 0.20))
        ))
        self.small_object_center_band_height_ratio = min(1.0, max(
            0.10, float(rospy.get_param(
                "~wedetect_small_object_center_band_height_ratio", 0.72
            ))
        ))
        self.small_object_center_band_crops = max(
            1, int(rospy.get_param("~wedetect_small_object_center_band_crops", 4))
        )
        self.small_object_tile_interval = max(
            0.10, float(rospy.get_param("~wedetect_small_object_tile_interval", 2.0))
        )
        self._last_tile_search_monotonic = float("-inf")
        self._last_inference_metadata = {}
        variant = rospy.get_param("~wedetect_variant", "base")
        rospy.loginfo("Loading WeDetect-%s model with cached text embeddings...", variant)
        try:
            self.model = WeDetectBackend(
                source_dir=rospy.get_param("~wedetect_source_dir", str(DEFAULT_SOURCE_DIR)),
                variant=variant,
                checkpoint=rospy.get_param("~wedetect_checkpoint", str(DEFAULT_CHECKPOINT)),
                language_model=rospy.get_param("~wedetect_language_model", str(DEFAULT_LANGUAGE_MODEL)),
                score_threshold=float(rospy.get_param("~wedetect_score_threshold", 0.20)),
                nms_iou=float(rospy.get_param("~wedetect_nms_iou", 0.70)),
                pre_nms_topk=int(rospy.get_param("~wedetect_pre_nms_topk", 3000)),
                max_detections=int(rospy.get_param("~wedetect_max_detections", 100)),
                use_fp16=bool(rospy.get_param("~wedetect_use_fp16", True)),
                label_map=label_map,
                runtime=rospy.get_param("~wedetect_runtime", "tensorrt"),
                vision_onnx=rospy.get_param("~wedetect_vision_onnx", str(DEFAULT_VISION_ONNX)),
                trt_engine_cache_path=rospy.get_param("~wedetect_trt_engine_cache", str(DEFAULT_TRT_CACHE)),
                trt_max_classes=int(rospy.get_param("~wedetect_trt_max_classes", 16)),
            )
        except WeDetectUnavailable as exc:
            rospy.logfatal("WeDetect initialization failed: %s", exc)
            raise
        rospy.loginfo(
            "WeDetect-%s model loaded; small-object tile search=%s layout=%s "
            "grid=%dx%d center_band=%d@%.2f interval=%.2fs.",
            self.model.variant,
            self.small_object_tile_search_enabled,
            self.small_object_tile_layout,
            self.small_object_tile_grid,
            self.small_object_tile_grid,
            self.small_object_center_band_crops,
            self.small_object_center_band_height_ratio,
            self.small_object_tile_interval,
        )

    def infer(self, image_bgr, prompt_a, prompt_b_terms):
        inference_started = time.monotonic()
        image_source = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        labels_for_model = [prompt_a] + list(prompt_b_terms)
        boxes, scores, labels = self.model.detect(image_source, labels_for_model)
        primary_target_count = sum(1 for label in labels if label == prompt_a)
        metadata = {
            "small_object_tile_search_enabled": self.small_object_tile_search_enabled,
            "small_object_tile_search_ran": False,
            "small_object_tile_search_layout": self.small_object_tile_layout,
            "small_object_tile_search_grid": self.small_object_tile_grid,
            "small_object_tile_search_center_band_height_ratio": (
                self.small_object_center_band_height_ratio
            ),
            "small_object_tile_search_center_band_crops": (
                self.small_object_center_band_crops
            ),
            "small_object_tile_search_primary_target_candidates": primary_target_count,
            "small_object_tile_search_tiles": 0,
            "small_object_tile_search_elapsed_seconds": 0.0,
        }
        # Keep the normal full-frame path at its established TensorRT rate. A
        # crop pass is a search fallback only and maps every resulting box
        # back to the original camera frame before common post-processing.
        now = time.monotonic()
        if (
            self.small_object_tile_search_enabled
            and primary_target_count == 0
            and now - self._last_tile_search_monotonic
            >= self.small_object_tile_interval
        ):
            tile_started = time.monotonic()
            if self.small_object_tile_layout == "center_band":
                tiled_boxes, tiled_scores, tiled_labels, tile_count = (
                    self.model.detect_center_band(
                        image_source,
                        labels_for_model,
                        crop_count=self.small_object_center_band_crops,
                        band_height_ratio=self.small_object_center_band_height_ratio,
                    )
                )
            else:
                tiled_boxes, tiled_scores, tiled_labels, tile_count = self.model.detect_tiles(
                    image_source,
                    labels_for_model,
                    grid=self.small_object_tile_grid,
                    overlap=self.small_object_tile_overlap,
                )
            boxes, scores, labels = self.model.merge_normalized_detections(
                list(boxes) + tiled_boxes,
                list(scores) + tiled_scores,
                list(labels) + tiled_labels,
            )
            self._last_tile_search_monotonic = now
            metadata.update({
                "small_object_tile_search_ran": True,
                "small_object_tile_search_tiles": tile_count,
                "small_object_tile_search_elapsed_seconds": round(
                    time.monotonic() - tile_started, 4
                ),
                "small_object_tile_search_target_candidates": sum(
                    1 for label in tiled_labels if label == prompt_a
                ),
            })
        metadata["inference_elapsed_seconds"] = round(
            time.monotonic() - inference_started, 4
        )
        self._last_inference_metadata = metadata
        target_boxes, target_scores, target_labels = [], [], []
        env_boxes, env_scores, env_labels = [], [], []
        env_set = set(prompt_b_terms)
        for box, score, label in zip(boxes, scores, labels):
            if label == prompt_a:
                target_boxes.append(box)
                target_scores.append(score)
                target_labels.append(label)
            elif label in env_set:
                env_boxes.append(box)
                env_scores.append(score)
                env_labels.append(label)
        return (
            image_source,
            torch.tensor(target_boxes, dtype=torch.float32).reshape(-1, 4),
            torch.tensor(target_scores, dtype=torch.float32),
            target_labels,
            torch.tensor(env_boxes, dtype=torch.float32).reshape(-1, 4),
            torch.tensor(env_scores, dtype=torch.float32),
            env_labels,
        )

    def inference_metadata(self):
        return dict(self._last_inference_metadata)


if __name__ == "__main__":
    WeDetectDetNode().spin()
