#!/usr/bin/env python3
"""Stage-1 shadow diagnostic: walls parallel vs perpendicular to a gate cut.

The false place 4 in env04 (20261114_185222 / _193957) is a vertical cut across
the room: both endpoints sit on the room's horizontal north/south walls, so the
wall tangent is perpendicular to the section (|cos| ~ 0).  A real doorway is a
gap in a wall, so the wall runs along the section (|cos| ~ 1).
"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_place_portal_validation import (  # noqa: E402
    _publish_portal_cut_diagnostic,
    _wall_tangent_cos,
    portal_cut_suspect,
)


def _map_with_walls(rows, cols=40):
    free = np.ones((rows, cols), dtype=bool)
    unknown = np.zeros_like(free)
    return free, unknown


class WallTangentTest(unittest.TestCase):
    def test_a_horizontal_wall_is_perpendicular_to_a_vertical_section(self):
        free, unknown = _map_with_walls(40)
        free[20, :] = False  # east-west wall at y ~ 2.05
        cos = _wall_tangent_cos(
            free, unknown, 0.1, (0.0, 0.0), (2.0, 2.05), (0.0, 1.0))
        self.assertIsNotNone(cos)
        self.assertLess(cos, 0.35)

    def test_the_same_wall_is_parallel_to_a_horizontal_section(self):
        free, unknown = _map_with_walls(40)
        free[20, :] = False
        cos = _wall_tangent_cos(
            free, unknown, 0.1, (0.0, 0.0), (2.0, 2.05), (1.0, 0.0))
        self.assertIsNotNone(cos)
        self.assertGreater(cos, 0.9)

    def test_sparse_occupied_cells_are_inconclusive(self):
        free, unknown = _map_with_walls(40)
        free[20, 20] = False
        cos = _wall_tangent_cos(
            free, unknown, 0.1, (0.0, 0.0), (2.05, 2.05), (0.0, 1.0))
        self.assertIsNone(cos)


class SuspectClassifierTest(unittest.TestCase):
    def test_wide_perpendicular_cut_inside_the_scope_is_a_suspect(self):
        self.assertTrue(portal_cut_suspect(3.5, 0.1, 0.2, True))

    def test_parallel_wall_doorway_is_not_a_suspect(self):
        self.assertFalse(portal_cut_suspect(3.5, 0.95, 0.9, True))

    def test_narrow_cut_is_not_a_suspect(self):
        self.assertFalse(portal_cut_suspect(1.2, 0.1, 0.2, True))

    def test_unknown_scope_membership_is_not_a_suspect(self):
        self.assertFalse(portal_cut_suspect(3.5, 0.1, 0.2, None))


class _Node:
    def __init__(self, inside=True):
        self.current_physical_place_id = 2
        self.events = []
        self.gp_exploration_tree = None
        if inside:
            self.current_place_scope_mask = np.ones((40, 40), dtype=bool)
            self.current_place_scope_message = SimpleNamespace(
                info=SimpleNamespace(
                    resolution=0.1,
                    origin=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0)),
                ))
            self.current_place_scope_valid = True
            self.current_place_scope_id = 2

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


def _context():
    return SimpleNamespace(
        known_free=_map_with_walls(40)[0],
        unknown=np.zeros((40, 40), dtype=bool),
        components=SimpleNamespace(epoch=7),
    )


def _message():
    return SimpleNamespace(info=SimpleNamespace(
        resolution=0.1,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
    ))


def _auth():
    return SimpleNamespace(node_id=3, branch_id=7, source_xy=(2.0, 1.5),
                           inside_xy=(1.8, 1.8), gate_xy=(2.0, 2.0),
                           destination_xy=(1.2, 2.0))


class DiagnosticPublishTest(unittest.TestCase):
    def test_false_cut_is_reported_once(self):
        node = _Node()
        context = _context()
        # two horizontal walls and a vertical section across them
        context.known_free[5, :] = False
        context.known_free[35, :] = False
        host = _publish_portal_cut_diagnostic(
            node, _message(), context, _auth(),
            {"reason": "distinct_destination_component"},
            source=(2.0, 1.5), gate=(2.0, 2.0), destination=(1.2, 2.0),
            width=3.0, bounds=[(2.0, 3.55), (2.0, 0.55)], existing=())
        self.assertTrue(host)
        names = [name for name, _fields in node.events]
        self.assertEqual(names.count("portal_cut_suspect_diagnostic"), 1)
        fields = node.events[0][1]
        self.assertTrue(fields["suspect"])
        self.assertTrue(fields["destination_inside_scope"])
        self.assertLess(fields["left_wall_cos"], 0.35)
        self.assertLess(fields["right_wall_cos"], 0.35)
        # same branch/epoch does not publish twice
        _publish_portal_cut_diagnostic(
            node, _message(), context, _auth(),
            {"reason": "distinct_destination_component"},
            source=(2.0, 1.5), gate=(2.0, 2.0), destination=(1.2, 2.0),
            width=3.0, bounds=[(2.0, 3.55), (2.0, 0.55)], existing=())
        self.assertEqual(
            [name for name, _f in node.events].count(
                "portal_cut_suspect_diagnostic"), 1)

    def test_a_real_doorway_is_not_flagged(self):
        node = _Node()
        context = _context()
        context.known_free[20, :] = False  # wall with a gap
        context.known_free[20, 10:30] = True
        _publish_portal_cut_diagnostic(
            node, _message(), context, _auth(),
            {"reason": "distinct_destination_component"},
            source=(2.0, 1.5), gate=(2.0, 2.0), destination=(1.2, 2.0),
            width=2.0, bounds=[(1.0, 2.05), (3.0, 2.05)], existing=())
        fields = node.events[0][1]
        self.assertFalse(fields["suspect"])
        self.assertGreater(fields["left_wall_cos"], 0.9)


if __name__ == "__main__":
    unittest.main()
