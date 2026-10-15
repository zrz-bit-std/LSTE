#!/usr/bin/env python3
"""Regression tests for detector colour validation on ROS BGR camera frames."""

import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))


try:
    import torch
    from utils import detector
except ImportError as exc:  # pragma: no cover - exercised in the dino runtime.
    raise unittest.SkipTest("detector runtime dependencies unavailable: %s" % exc)


class DetectorColorValidationTest(unittest.TestCase):
    def setUp(self):
        # cv_bridge delivers bgr8; this is a saturated yellow BGR image.
        self.image_bgr = np.full((32, 32, 3), (0, 255, 255), dtype=np.uint8)
        self.boxes = torch.tensor([[0.5, 0.5, 1.0, 1.0]], dtype=torch.float32)
        self.scores = torch.tensor([0.9], dtype=torch.float32)
        self.labels = ["yellow cup"]

    def test_phrase_validation_keeps_yellow_bgr_detection(self):
        boxes, scores, labels = detector.validate_color_by_phrase(
            self.image_bgr, self.boxes, self.scores, self.labels
        )
        self.assertEqual(len(boxes), 1)
        self.assertEqual(len(scores), 1)
        self.assertEqual(labels, self.labels)

    def test_requested_color_filter_keeps_yellow_bgr_detection(self):
        boxes, scores, labels, filtered, removed = detector.filter_boxes_by_color(
            self.image_bgr, self.boxes, self.scores, self.labels, ["yellow"]
        )
        self.assertEqual(len(boxes), 1)
        self.assertEqual(len(scores), 1)
        self.assertEqual(labels, self.labels)
        self.assertTrue(filtered)
        self.assertEqual(removed, [])

    def test_target_candidate_survives_when_color_is_not_observable(self):
        """Model evidence must survive a tiny or occluded color crop."""
        image_bgr = np.zeros((32, 32, 3), dtype=np.uint8)
        boxes, scores, labels = detector.postprocess_target_candidates(
            image_bgr,
            self.boxes,
            self.scores,
            self.labels,
            ["yellow"],
        )
        self.assertEqual(len(boxes), 1)
        self.assertEqual(len(scores), 1)
        self.assertEqual(labels, self.labels)

    def test_target_color_evidence_can_still_narrow_multiple_candidates(self):
        """Positive color evidence remains useful without being mandatory."""
        boxes = torch.tensor(
            [[0.25, 0.5, 0.25, 0.25], [0.75, 0.5, 0.25, 0.25]],
            dtype=torch.float32,
        )
        scores = torch.tensor([0.8, 0.7], dtype=torch.float32)
        labels = ["yellow cup", "yellow cup"]
        image_bgr = np.zeros((32, 32, 3), dtype=np.uint8)
        image_bgr[:, :16] = (0, 255, 255)
        kept_boxes, kept_scores, kept_labels = detector.postprocess_target_candidates(
            image_bgr, boxes, scores, labels, ["yellow"]
        )
        self.assertEqual(len(kept_boxes), 1)
        self.assertAlmostEqual(float(kept_scores[0]), 0.8, places=5)
        self.assertEqual(kept_labels, ["yellow cup"])


if __name__ == "__main__":
    unittest.main()
