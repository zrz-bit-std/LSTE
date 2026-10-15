#!/usr/bin/env python3
"""Regression for structural-revision-scoped endpoint transition caching."""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_execution_prefetch import (  # noqa: E402
    GlobalFrontierExecutionPrefetchMixin,
)


class TransitionCacheProbe(GlobalFrontierExecutionPrefetchMixin):
    def __init__(self):
        self.active_route_id = 4
        self.event_graph = SimpleNamespace(structural_revision=7)
        self.nearest_seed_calls = 0
        self.bfs_calls = 0
        self.statuses = []

    def nearest_seed(self, *_args):
        self.nearest_seed_calls += 1
        return (2, 2)

    def bfs(self, *_args):
        self.bfs_calls += 1
        return np.ones((5, 5), dtype=np.int32)

    def publish_status(self, event, **fields):
        self.statuses.append((event, fields))


class FrontierTransitionTopologyCacheTest(unittest.TestCase):
    def test_same_route_reuses_advisory_until_route_lease_changes(self):
        probe = TransitionCacheProbe()
        message = SimpleNamespace(
            info=SimpleNamespace(
                resolution=0.1,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
            )
        )
        grid = np.zeros((5, 5), dtype=bool)

        first_steps, first_seed, first_hit = probe._endpoint_transition_topology(
            message, grid, 2, 2, 1.0, 2.0
        )
        second_steps, second_seed, second_hit = probe._endpoint_transition_topology(
            message, grid, 2, 2, 1.0, 2.0
        )

        self.assertFalse(first_hit)
        self.assertTrue(second_hit)
        self.assertIs(first_steps, second_steps)
        self.assertEqual(first_seed, second_seed)
        self.assertEqual(probe.nearest_seed_calls, 1)
        self.assertEqual(probe.bfs_calls, 1)

        message.info.origin.position.x = 0.1
        _steps, _seed, cache_hit = probe._endpoint_transition_topology(
            message, grid, 2, 2, 1.0, 2.0
        )
        self.assertFalse(cache_hit)
        self.assertEqual(probe.bfs_calls, 2)

        probe.active_route_id = 5
        _steps, _seed, cache_hit = probe._endpoint_transition_topology(
            message, grid, 2, 2, 1.0, 2.0
        )
        self.assertFalse(cache_hit)
        self.assertEqual(probe.nearest_seed_calls, 3)
        self.assertEqual(probe.bfs_calls, 3)
        self.assertEqual(probe.statuses[-1][0], "transition_topology_cached")


if __name__ == "__main__":
    unittest.main()
