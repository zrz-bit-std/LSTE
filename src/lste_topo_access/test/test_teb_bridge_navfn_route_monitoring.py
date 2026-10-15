"""Regression tests for Navfn endpoint identity and callback ordering."""

from pathlib import Path
import sys
from types import SimpleNamespace
import threading
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_goal_bridge_route_monitoring import TebGoalBridgeRouteMonitoringMixin


class _FakeBridge(TebGoalBridgeRouteMonitoringMixin):
    def __init__(self):
        self.lock = threading.RLock()
        self.action_active = True
        self.global_frame = "map"
        self.active_goal_global = self._pose("map", 2.0, 0.0)
        self.active_feedback_pose = None
        self.active_navfn_plan_points = []
        self.active_navfn_plan_endpoint = None
        self.active_navfn_remaining = None
        self.active_navfn_best_remaining = None
        self.active_navfn_progress_monotonic = 0.0
        self.progress_epsilon = 0.01

    @staticmethod
    def _pose(frame, x, y):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
            ),
        )

    def navfn_message(self):
        poses = [
            self._pose("map", 0.0, 0.0),
            self._pose("map", 1.0, 0.0),
            self._pose("map", 2.0, 0.0),
        ]
        return SimpleNamespace(header=SimpleNamespace(frame_id="map"), poses=poses)


class TebBridgeNavfnRouteMonitoringTest(unittest.TestCase):
    def test_navfn_plan_before_feedback_keeps_endpoint_identity(self):
        bridge = _FakeBridge()

        bridge.on_navfn_plan(bridge.navfn_message())

        self.assertEqual(bridge.active_navfn_plan_endpoint, [2.0, 0.0])
        self.assertEqual(bridge.active_navfn_plan_points[-1], (2.0, 0.0))
        # Progress projection waits for feedback, but endpoint identity does
        # not. This is the callback-order case seen in the endpoint mismatch.
        self.assertIsNone(bridge.active_navfn_remaining)

    def test_late_feedback_projects_onto_the_saved_plan(self):
        bridge = _FakeBridge()
        bridge.on_navfn_plan(bridge.navfn_message())
        bridge.active_feedback_pose = (0.5, 0.0, 0.0)

        bridge._update_navfn_path_progress_locked(now=10.0)

        self.assertAlmostEqual(bridge.active_navfn_remaining, 1.5)
        self.assertAlmostEqual(bridge.active_navfn_best_remaining, 1.5)
        self.assertEqual(bridge.active_navfn_progress_monotonic, 10.0)


if __name__ == "__main__":
    unittest.main()
