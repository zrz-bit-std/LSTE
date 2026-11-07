"""Regression tests for the wall-clock Place coverage fallback."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        loginfo=lambda *_args, **_kwargs: None,
        logwarn=lambda *_args, **_kwargs: None,
    ),
)

from global_frontier_place_wall_fallback import PlaceWallFallback  # noqa: E402
import global_frontier_planning_runtime as runtime  # noqa: E402
from global_frontier_planning_runtime import (  # noqa: E402
    GlobalFrontierPlanningRuntimeMixin,
)


class PlaceWallFallbackTest(unittest.TestCase):
    def test_expires_only_after_the_wall_budget(self):
        helper = PlaceWallFallback(seconds=45.0)
        pose = (1.0, 2.0)
        self.assertIsNone(helper.observe(1, True, pose, 100.0))
        self.assertIsNone(helper.observe(1, True, pose, 144.9))
        self.assertEqual(helper.observe(1, True, pose, 145.0), 1)

    def test_physical_progress_restarts_the_budget(self):
        helper = PlaceWallFallback(seconds=45.0)
        self.assertIsNone(helper.observe(1, True, (1.0, 2.0), 100.0))
        self.assertIsNone(helper.observe(1, True, (1.4, 2.0), 140.0))
        # The old anchor would have expired at 145 s; movement renewed it.
        self.assertIsNone(helper.observe(1, True, (1.4, 2.0), 150.0))
        self.assertEqual(helper.observe(1, True, (1.4, 2.0), 185.0), 1)

    def test_no_waiting_state_clears_the_budget(self):
        helper = PlaceWallFallback(seconds=45.0)
        self.assertIsNone(helper.observe(1, True, (1.0, 2.0), 100.0))
        self.assertIsNone(helper.observe(1, False, (1.0, 2.0), 144.0))
        self.assertIsNone(helper.observe(1, True, (1.0, 2.0), 150.0))
        self.assertEqual(helper.observe(1, True, (1.0, 2.0), 195.0), 1)

    def test_place_change_restarts_the_budget(self):
        helper = PlaceWallFallback(seconds=45.0)
        self.assertIsNone(helper.observe(1, True, (1.0, 2.0), 100.0))
        self.assertIsNone(helper.observe(2, True, (1.0, 2.0), 144.0))
        self.assertEqual(helper.observe(2, True, (1.0, 2.0), 189.0), 2)

    def test_missing_pose_is_a_no_op(self):
        helper = PlaceWallFallback(seconds=45.0)
        self.assertIsNone(helper.observe(1, True, None, 100.0))
        self.assertIsNone(helper.observe(1, True, None, 200.0))


class _Pose:
    def __init__(self, x=1.0, y=2.0):
        self.x = x
        self.y = y


class _Ledger:
    def __init__(self, actionable):
        self.actionable = actionable

    def actionable_frontier_observation_count(self, place_id):
        return self.actionable


class _Owner(GlobalFrontierPlanningRuntimeMixin):
    def __init__(self, seconds=45.0, actionable=0, observed=False):
        self.place_wall_fallback = PlaceWallFallback(seconds=seconds)
        self.pose_odom = _Pose()
        self.place_work_items = _Ledger(actionable)
        self.report = {
            "place_id": 1,
            "observed": observed,
            # A portal-only obligation still appears in this count; it is not
            # dispatchable observation work.
            "unresolved_work_items": 1,
        }
        self._sim_wall_anchor = None
        self.marked = []
        self.events = []

    def place_progress_report(self):
        return self.report

    def _mark_place_observation_best_effort(self, place_id, now):
        self.marked.append((place_id, now))
        return True

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class PlaceWallFallbackWrapperTest(unittest.TestCase):
    def _apply(self, owner, seconds):
        with mock.patch.object(runtime.time, "monotonic", return_value=seconds), \
                mock.patch.object(runtime, "now_for", return_value=7.0):
            return owner._apply_place_wall_fallback()

    def test_held_place_is_released_once_after_the_wall_budget(self):
        owner = _Owner()
        self.assertFalse(self._apply(owner, 0.0))
        self.assertFalse(self._apply(owner, 44.0))
        self.assertEqual(owner.marked, [])
        self.assertTrue(self._apply(owner, 46.0))
        self.assertEqual(owner.marked, [(1, 7.0)])
        self.assertEqual(owner.events[0][0], "place_observation_wall_fallback")
        self.assertEqual(
            owner.events[0][1]["reason"],
            "place_observation_no_wall_progress",
        )
        # The dwell restarts after a release; it must not mark every tick.
        self.assertFalse(self._apply(owner, 47.0))
        self.assertEqual(len(owner.marked), 1)

    def test_actionable_observation_work_keeps_the_place_open(self):
        owner = _Owner(actionable=1)
        self.assertFalse(self._apply(owner, 0.0))
        self.assertFalse(self._apply(owner, 60.0))
        self.assertEqual(owner.marked, [])

    def test_observed_place_is_not_touched(self):
        owner = _Owner(observed=True)
        self.assertFalse(self._apply(owner, 0.0))
        self.assertFalse(self._apply(owner, 60.0))
        self.assertEqual(owner.marked, [])


if __name__ == "__main__":
    unittest.main()
