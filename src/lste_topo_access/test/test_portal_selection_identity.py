"""Ensure graph-owned Portal selection is side-effect free for other doors."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_models import PortalSource  # noqa: E402
from global_frontier_portal_selection import (  # noqa: E402
    GlobalFrontierPortalSelectionMixin,
)


class PortalSelectionFixture(GlobalFrontierPortalSelectionMixin):
    def __init__(self):
        self.graph_route_portal_id = 7
        self.task_semantic_value_enabled = False
        self.mutations = 0
        self.completed_radius = 0.5
        self.last_portal_excluded = 0
        self.last_portal_rejected = 0
        self.last_portal_costmap_rejected = 0
        self.last_portal_heading_rejected = 0
        self.last_portal_endpoint_depth_rejections = 0
        self.last_portal_source_side_rejections = 0
        self.last_portal_covered_destination_skips = 0
        self.last_portal_covered_cycle_skips = 0
        self.last_portal_reverse_egress_count = 0
        self.last_graph_policy_rejections = 0

    def _refresh_or_certify_portal_hypothesis(self, *_args):
        self.mutations += 1
        raise AssertionError("unrelated Portal must not be certified")


class PortalSelectionIdentityTest(unittest.TestCase):
    def test_preferred_graph_portal_filters_before_ledger_mutation(self):
        fixture = PortalSelectionFixture()
        request = SimpleNamespace(
            allow_portal_transitions=True,
            allowed_region_tiers=None,
            excluded=(),
            message=SimpleNamespace(info=SimpleNamespace(resolution=0.1)),
            validation=None,
            steps=np.ones((4, 4), dtype=np.int32),
            score_path_from_steps=False,
            unknown=np.zeros((4, 4), dtype=bool),
            components=None,
            map_to_physical_xy=None,
            route_anchor_xy=(0.0, 0.0),
            semantic_pursuit=False,
            semantic_hint=None,
            now=1.0,
        )
        unrelated = PortalSource(
            frontier_row=1,
            frontier_col=1,
            identity_row=1,
            identity_col=1,
            gate_row=1,
            gate_col=0,
            endpoint_row=1,
            endpoint_col=2,
            hypothesis_id=None,
        )

        selected = fixture._select_portal_transition(
            request, {("other", 1): unrelated},
        )

        self.assertIsNone(selected)
        self.assertEqual(fixture.mutations, 0)
        self.assertEqual(fixture.last_portal_candidates_seen, 1)
        self.assertEqual(fixture.last_portal_executable_candidates, 0)


if __name__ == "__main__":
    unittest.main()
