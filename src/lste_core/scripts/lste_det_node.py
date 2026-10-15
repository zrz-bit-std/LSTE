#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GroundingDINO-only LSTE detection node.

This is the established DINO path. It deliberately has no WeDetect import,
configuration, or runtime branch.
"""

import os
import tempfile

import cv2
import rospy

from utils import detector as det_utils
from utils.detection_node_common import DetectionNodeBase


class GroundingDINODetNode(DetectionNodeBase):
    detector_name = "groundingdino"

    def __init__(self):
        super().__init__()
        self.expected_env = rospy.get_param("~expected_conda_env", "dino")
        current_env = os.environ.get("CONDA_DEFAULT_ENV", "")
        if self.expected_env and current_env != self.expected_env:
            rospy.logwarn("Current CONDA_DEFAULT_ENV=%s; expected %s", current_env, self.expected_env)
        self.config_path = rospy.get_param("~dino_config_path", str(det_utils.DEFAULT_DINO_CONFIG_PATH))
        self.weights_path = rospy.get_param("~dino_weights_path", str(det_utils.DEFAULT_DINO_WEIGHTS_PATH))
        self.box_threshold = float(rospy.get_param("~box_threshold", 0.35))
        self.text_threshold = float(rospy.get_param("~text_threshold", 0.25))
        self.tmp_image_path = os.path.join(tempfile.gettempdir(), "lste_det_node_latest.png")
        rospy.loginfo("Loading GroundingDINO model...")
        self.model = det_utils.load_model(self.config_path, self.weights_path)
        rospy.loginfo("GroundingDINO model loaded.")

    def infer(self, image_bgr, prompt_a, prompt_b_terms):
        os.makedirs(os.path.dirname(self.tmp_image_path), exist_ok=True)
        if not cv2.imwrite(self.tmp_image_path, image_bgr):
            raise RuntimeError("cv2.imwrite returned false")
        image_source, image = det_utils.load_image(self.tmp_image_path)
        env_caption = " . ".join(prompt_b_terms).strip()
        target_boxes, target_scores, target_labels = det_utils.run_grounding_dino_with_caption(
            model=self.model, image_source=image_source, image=image, caption=prompt_a,
            run_label="TARGET", box_threshold=self.box_threshold, text_threshold=self.text_threshold,
            output_path=None,
        )
        env_boxes, env_scores, env_labels = det_utils.run_grounding_dino_with_caption(
            model=self.model, image_source=image_source, image=image, caption=env_caption,
            run_label="ENV", box_threshold=self.box_threshold, text_threshold=self.text_threshold,
            output_path=None,
        )
        return image_source, target_boxes, target_scores, target_labels, env_boxes, env_scores, env_labels


if __name__ == "__main__":
    GroundingDINODetNode().spin()
