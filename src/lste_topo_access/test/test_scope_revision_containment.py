"""A Place scope that still contains the active task must not suspend it.

Runs ``20261101_010504`` (T), ``20261101_020324`` (cross), ``20261101_040242``
(rotated T) and ``20261101_053934`` suspended their active WorkItem 174, 204,
153 and 66 times, and every one of those suspensions was
``place_scope_revision``.  The Place scope is republished on almost every SLAM
update; the publisher treated any effective change as a reason to abort the
route, so no Attempt ever lasted more than about two seconds, the same WorkItem
accumulated 15-25 Attempts, and the deepest Place never left ``observe``.  The
DFS therefore never returned and tree/lifecycle/dfs could not pass.

Only a revision that removes the robot, the active endpoint, or the segment
between them may abort the route.  Growth is discovery, not invalidation.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_place_navigation import PlaceNavigationConstraints  # noqa: E402


SIZE = 40
RESOLUTION = 0.5
ORIGIN = (0.0, 0.0)


def message(size=SIZE, resolution=RESOLUTION, origin=ORIGIN):
    return SimpleNamespace(
        info=SimpleNamespace(
            width=size,
            height=size,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin[0], y=origin[1]),
            ),
        ),
    )


def mask(lo=0, hi=20, size=SIZE):
    grid = np.zeros((size, size), dtype=bool)
    grid[lo:hi, lo:hi] = True
    return grid


def node(robot=(5.0, 5.0), goal=(12.0, 12.0)):
    return SimpleNamespace(
        pose_odom=SimpleNamespace(x=robot[0], y=robot[1], theta=0.0),
        active_frontier=(0, 0, goal[0], goal[1]),
        active_route_id=7,
    )


class ScopeRevisionTest(unittest.TestCase):
    def keeps(self, allowed, robot=(5.0, 5.0), goal=(12.0, 12.0), project=None):
        owner = PlaceNavigationConstraints.__new__(PlaceNavigationConstraints)
        projector = project or (lambda x, y: (x, y))
        return owner._scope_revision_keeps_active_task(
            node(robot, goal), message(), allowed, project=projector
        )

    def test_growth_that_still_contains_robot_goal_and_segment_keeps_the_task(self):
        self.assertTrue(self.keeps(mask(0, 30)))

    def test_goal_outside_the_revised_scope_aborts(self):
        self.assertFalse(self.keeps(mask(0, 20), goal=(30.0, 30.0)))

    def test_robot_outside_the_revised_scope_aborts(self):
        self.assertFalse(self.keeps(mask(0, 20), robot=(30.0, 30.0)))

    def test_shrunken_corridor_between_robot_and_goal_aborts(self):
        allowed = mask(0, 20)
        # Carve a hole on the segment so the route would cross excluded cells.
        allowed[10:13, 10:13] = False
        self.assertFalse(self.keeps(allowed))

    def test_unbounded_or_missing_inputs_never_claim_safety(self):
        owner = PlaceNavigationConstraints.__new__(PlaceNavigationConstraints)
        self.assertFalse(
            owner._scope_revision_keeps_active_task(node(), message(), None)
        )
        self.assertFalse(
            owner._scope_revision_keeps_active_task(node(), None, mask())
        )

    def test_robot_pose_is_projected_into_the_map_frame(self):
        # Robot odom (5, 5) projects to map (15, 15); goal is map (12, 12).
        # At 0.5 m cells those are cells (30, 30) and (24, 24).
        def covering():
            grid = np.zeros((SIZE, SIZE), dtype=bool)
            grid[23:31, 23:31] = True
            return grid

        self.assertTrue(
            self.keeps(covering(), robot=(5.0, 5.0),
                       project=lambda x, y: (x + 10, y + 10))
        )
        allowed_out = covering()
        allowed_out[30, 30] = False
        self.assertFalse(
            self.keeps(allowed_out, robot=(5.0, 5.0),
                       project=lambda x, y: (x + 10, y + 10))
        )


class CallerContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (SCRIPTS / "global_frontier_place_navigation.py").read_text(
            encoding="utf-8"
        )

    def test_suspend_is_guarded_by_the_containment_check(self):
        self.assertIn(
            "and not self._scope_revision_keeps_active_task(", self.source
        )
        guard = self.source.index("_scope_revision_keeps_active_task(\n                        node,message,allowed)")
        suspend = self.source.index("suspend(now_for(node),'place_scope_revision')")
        self.assertLess(guard, suspend)


if __name__ == "__main__":
    unittest.main(verbosity=2)
