"""Regression tests for the map-versus-physical progress contract."""

import ast
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_physical_progress import physical_progress_decision


def load_progress_recorder():
    source_path = SCRIPTS / "global_frontier_execution_observation.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierExecutionObservationMixin"
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_record_active_route_progress"
    )
    namespace = {"physical_progress_decision": physical_progress_decision}
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(source_path), "exec"),
        namespace,
    )
    return namespace["_record_active_route_progress"]


RECORD_PROGRESS = load_progress_recorder()


class PhysicalProgressContractTest(unittest.TestCase):
    def test_integrated_recorder_ignores_slam_only_progress(self):
        explorer = type("Explorer", (), {})()
        explorer.active_progress_time = 10.0
        explorer.active_last_progress_signal = "previous"

        RECORD_PROGRESS(
            explorer,
            40.0,
            True,
            True,
            False,
            False,
        )

        self.assertEqual(explorer.active_progress_time, 10.0)
        self.assertEqual(
            explorer.active_last_progress_signal,
            "map_progress_without_physical_motion",
        )

    def test_integrated_recorder_renews_on_odom_progress(self):
        explorer = type("Explorer", (), {})()
        explorer.active_progress_time = 10.0
        explorer.active_last_progress_signal = "previous"

        RECORD_PROGRESS(
            explorer,
            40.0,
            False,
            False,
            True,
            False,
        )

        self.assertEqual(explorer.active_progress_time, 40.0)
        self.assertEqual(explorer.active_last_progress_signal, "odom_detour")

    def test_slam_only_route_improvement_does_not_renew_watchdog(self):
        decision = physical_progress_decision(
            route_progress=True,
            goal_progress=True,
            odom_detour_progress=False,
            odom_novel_coverage=False,
        )

        self.assertFalse(decision.renew_watchdog)
        self.assertEqual(
            decision.signal,
            "map_progress_without_physical_motion",
        )

    def test_odom_motion_renews_even_when_map_distance_grows(self):
        decision = physical_progress_decision(
            route_progress=False,
            goal_progress=False,
            odom_detour_progress=True,
            odom_novel_coverage=False,
        )

        self.assertTrue(decision.renew_watchdog)
        self.assertEqual(decision.signal, "odom_detour")

    def test_new_physical_cell_is_independent_progress_evidence(self):
        decision = physical_progress_decision(odom_novel_coverage=True)

        self.assertTrue(decision.renew_watchdog)
        self.assertEqual(decision.signal, "odom_novel_coverage")

    def test_no_evidence_does_not_change_watchdog(self):
        decision = physical_progress_decision()

        self.assertFalse(decision.renew_watchdog)
        self.assertEqual(decision.signal, "none")


if __name__ == "__main__":
    unittest.main()
