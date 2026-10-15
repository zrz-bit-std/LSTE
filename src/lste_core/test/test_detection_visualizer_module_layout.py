#!/usr/bin/env python3
"""Guard the visualizer's intentionally small ROS entry point."""

import ast
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
ENTRY_POINT = SCRIPTS_DIR / "lste_det_vis_node.py"
COMPONENTS = (
    "det_vis_tracking.py",
    "det_vis_overlay.py",
    "det_vis_score_panel.py",
    "det_vis_goal_projection.py",
)


class DetectionVisualizerModuleLayoutTest(unittest.TestCase):
    def test_ros_entry_uses_specialized_mixins(self):
        source = ENTRY_POINT.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(ENTRY_POINT))
        visualizer = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "DetectionVisualizer"
        )
        base_names = {base.id for base in visualizer.bases if isinstance(base, ast.Name)}
        self.assertEqual(
            base_names,
            {
                "DetectionTrackingMixin",
                "DetectionOverlayMixin",
                "DetectionScorePanelMixin",
                "DetectionGoalProjectionMixin",
            },
        )

    def test_entry_and_components_remain_readable_units(self):
        self.assertLessEqual(len(ENTRY_POINT.read_text(encoding="utf-8").splitlines()), 350)
        for filename in COMPONENTS:
            component = SCRIPTS_DIR / filename
            self.assertTrue(component.is_file(), filename)
            self.assertLessEqual(len(component.read_text(encoding="utf-8").splitlines()), 500)


if __name__ == "__main__":
    unittest.main()
