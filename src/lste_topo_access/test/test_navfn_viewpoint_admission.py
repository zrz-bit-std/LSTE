"""Regression for pre-dispatch Navfn rejection of a WorkItem viewpoint."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_selection_validation import (  # noqa: E402
    GlobalFrontierSelectionValidationMixin,
)


class NavfnAdmissionFixture(GlobalFrontierSelectionValidationMixin):
    def __init__(self):
        self.rejected_frontiers = []
        self.rejected_timeout = 180.0
        self.frontier_validation_pending = False
        self.frontier_validation_budget_exhausted = False
        self.navfn_last_validation_state = "endpoint_offset"
        self.navfn_last_plan_endpoint = (3.0, 3.8)
        self.calls = 0
        self.rejections = []

    def choose_frontier(self, *_args, **_kwargs):
        self.calls += 1
        route = [0, 0, 3.0, 4.0, 1.0, 0.0, 0.0, 0.0, None, 0,
                 "frontier_endpoint", None, 9]
        if self.calls == 1:
            return tuple(route)
        route[2], route[3], route[12] = 3.5, 4.5, 10
        return tuple(route)

    def navfn_goal_reachable(self, _robot_map, _goal, _frame):
        return self.calls > 1

    def record_work_item_route_rejection(self, message, candidate, now, reason,
                                         **kwargs):
        self.rejections.append((message, candidate, now, reason, kwargs))


class NavfnViewpointAdmissionTest(unittest.TestCase):
    def test_offset_rejection_is_recorded_before_next_candidate(self):
        fixture = NavfnAdmissionFixture()
        message = SimpleNamespace(header=SimpleNamespace(frame_id="map"))
        components = SimpleNamespace(epoch=12)

        with patch(
            "global_frontier_selection_validation.rospy.logwarn",
            create=True,
        ):
            selected = fixture.choose_valid_frontier(
                message,
                steps=None,
                frontier=None,
                unknown=None,
                occupied=None,
                now=4.0,
                robot_map=(0.0, 0.0),
                components=components,
                allow_heading_fallback=False,
            )

        self.assertEqual(selected[12], 10)
        self.assertEqual(len(fixture.rejections), 1)
        _message, rejected, _now, reason, fields = fixture.rejections[0]
        self.assertEqual(rejected[12], 9)
        self.assertEqual(reason, "navfn_endpoint_offset")
        self.assertEqual(fields["map_epoch"], 12)
        self.assertEqual(fields["plan_endpoint"], (3.0, 3.8))
        self.assertEqual(fixture.rejected_frontiers, [(4.0, 3.0, 4.0)])


if __name__ == "__main__":
    unittest.main()
