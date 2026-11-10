"""Ensure graph-owned Portal selection is side-effect free for other doors."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_models import PortalSource  # noqa: E402
from global_frontier_portal_belief import PortalHypothesisLedger  # noqa: E402
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

    def publish_status(self, *_args, **_kwargs):
        """Status emission is not part of the identity contract under test."""


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


class SemanticHeadingOverrideTest(unittest.TestCase):
    """A high-value doorway may turn in place; low-value rooms may not."""

    def setUp(self):
        self.ledger = PortalHypothesisLedger()
        self.edge, _ = self.ledger.certify(
            1, (2.0, 0.0), (5.0, 0.0), now=1.0,
        )
        self.ledger.bind_destination(self.edge["id"], 2, now=1.0)
        self.ledger.get(self.edge["id"])["semantic_score"] = 0.73
        self.source = SimpleNamespace(hypothesis_id=self.edge["id"])
        self.fixture = PortalSelectionFixture()
        self.fixture.portal_hypothesis_ledger = self.ledger
        self.fixture.heading_semantic_override_enabled = True
        self.fixture.task_semantic_value_enabled = True
        self.fixture.heading_semantic_override_max_attempts = 2
        self.fixture.heading_semantic_override_min_score = 0.5

    def test_high_semantic_doorway_may_turn_and_is_bounded(self):
        delta = math.radians(178.0)

        self.assertIsNotNone(
            self.fixture._portal_semantic_heading_override(self.source, delta)
        )
        self.assertIsNotNone(
            self.fixture._portal_semantic_heading_override(self.source, delta)
        )
        # A failed in-place turn must not be retried forever.
        self.assertIsNone(
            self.fixture._portal_semantic_heading_override(self.source, delta)
        )
        self.assertEqual(
            self.ledger.get(self.edge["id"])["heading_override_count"], 2,
        )
        # The override is not a rejection: the edge stays selectable.
        self.assertIsNone(
            self.ledger.get(self.edge["id"])["rejected_map_epoch"]
        )

    def test_low_semantic_doorway_keeps_the_hard_bound(self):
        self.ledger.get(self.edge["id"])["semantic_score"] = -0.69

        self.assertIsNone(
            self.fixture._portal_semantic_heading_override(
                self.source, math.radians(178.0)
            )
        )

    def test_unbound_or_unknown_doorway_is_never_overridden(self):
        unbound, _ = self.ledger.certify(
            3, (6.0, 0.0), (8.0, 0.0), now=1.0,
        )
        unbound_source = SimpleNamespace(hypothesis_id=unbound["id"])

        self.assertIsNone(
            self.fixture._portal_semantic_heading_override(
                unbound_source, math.radians(178.0)
            )
        )
        self.assertIsNone(
            self.fixture._portal_semantic_heading_override(
                SimpleNamespace(hypothesis_id=None), math.radians(178.0)
            )
        )

    def test_override_can_be_disabled(self):
        self.fixture.heading_semantic_override_enabled = False

        self.assertIsNone(
            self.fixture._portal_semantic_heading_override(
                self.source, math.radians(178.0)
            )
        )

    def test_override_is_bounded_by_the_parameter(self):
        self.fixture.heading_semantic_override_max_attempts = 0

        self.assertIsNone(
            self.fixture._portal_semantic_heading_override(
                self.source, math.radians(178.0)
            )
        )


if __name__ == "__main__":
    unittest.main()
