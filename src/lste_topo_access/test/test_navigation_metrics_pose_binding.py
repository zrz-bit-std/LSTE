#!/usr/bin/env python3
"""Regression coverage for NavigationMetrics instance helper binding."""

import ast
from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "navigation_metrics_pose.py"


def pose_helper_node():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    metrics = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "NavigationMetricsPoseMixin"
    )
    return next(
        node for node in metrics.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_pose_xy_in_frame_locked"
    )


class NavigationMetricsPoseBindingTest(unittest.TestCase):
    def test_pose_transform_helper_is_an_instance_method(self):
        helper = pose_helper_node()
        decorators = {
            decorator.id
            for decorator in helper.decorator_list
            if isinstance(decorator, ast.Name)
        }
        self.assertNotIn("staticmethod", decorators)
        self.assertEqual([argument.arg for argument in helper.args.args[:2]], [
            "self", "frame",
        ])

    def test_odom_pose_path_binds_self_and_returns_the_cached_pose(self):
        helper = pose_helper_node()
        namespace = {}
        exec(
            compile(ast.Module(body=[helper], type_ignores=[]), str(SOURCE), "exec"),
            namespace,
        )

        class MetricsFixture:
            _pose_xy_in_frame_locked = namespace["_pose_xy_in_frame_locked"]

            def __init__(self):
                self.pose = (1.25, -2.5, 0.75)

        self.assertEqual(
            MetricsFixture()._pose_xy_in_frame_locked("odom"),
            (1.25, -2.5, 0.75),
        )


if __name__ == "__main__":
    unittest.main()
