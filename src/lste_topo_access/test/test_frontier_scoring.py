#!/usr/bin/env python3
"""Unit tests for the ROS-independent frontier ranking policy."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_scoring import (
    candidate_score,
    initialise_score_pools,
    record_scored_candidate,
    select_score_pool_candidate,
)
from global_frontier_action_policy import unique_action_tiers


class FrontierScoringTest(unittest.TestCase):
    def test_new_local_candidate_has_priority_over_revisit_and_portal(self):
        pools = initialise_score_pools(("local", "adjacent"))
        new_local = (0, 0, 0.0, 0.0, 1.0, 10.0, 5.0, 1.0, None, 0, "frontier_endpoint")
        revisit_local = (0, 1, 1.0, 0.0, 1.0, 10.0, 50.0, 9.0, None, 0, "frontier_endpoint")
        record_scored_candidate(
            pools, "local", new_local, "new", False, 20, 10,
            "strict_clearance", 1,
        )
        record_scored_candidate(
            pools, "local", revisit_local, "revisit", False, 20, 10,
            "strict_clearance", 1,
        )

        selected, tier = select_score_pool_candidate(
            pools, ("local", "adjacent"), None, False,
        )

        self.assertIs(selected, new_local)
        self.assertEqual(tier, "new")

    def test_short_candidate_is_recovery_only(self):
        pools = initialise_score_pools(("local",))
        short = (0, 0, 0.0, 0.0, 1.0, 10.0, 5.0, 1.0, None, 0, "frontier_endpoint")
        record_scored_candidate(
            pools, "local", short, "new", False, 2, 10,
            "strict_clearance", 1,
        )
        selected, _ = select_score_pool_candidate(pools, ("local",), None, False)
        self.assertIsNone(selected)

        record_scored_candidate(
            pools, "local", short, "new", False, 2, 10,
            "navfn_observation_recovery", 1,
        )
        selected, _ = select_score_pool_candidate(pools, ("local",), None, False)
        self.assertIs(selected, short)

    def test_semantic_hint_penalizes_distant_candidates(self):
        near = candidate_score(
            10.0, 2.0, 2.0, (1.0, 0.0), 0.0, (0.0, 0.0),
            0.2, 1.0, 10.0,
        )
        far = candidate_score(
            10.0, 2.0, 2.0, (8.0, 0.0), 0.0, (0.0, 0.0),
            0.2, 1.0, 10.0,
        )
        self.assertGreater(near, far)

    def test_probe_candidate_is_representable_when_context_omits_probe(self):
        """Structural probing must not crash an unobserved-place cycle."""
        pools = initialise_score_pools(("local", "adjacent"))
        probe = (
            0, 0, 0.0, 0.0, 2.0, 4.0, 3.0, 1.0, None, 0,
            "frontier_endpoint", None, None, None, 0, object(), False,
        )
        record_scored_candidate(
            pools, "probe", probe, "new", False, 20, 10,
            "strict_clearance", 1,
        )
        selected, tier = select_score_pool_candidate(
            pools,
            unique_action_tiers(("local", "adjacent"), include_probe=True),
            None,
            False,
        )
        self.assertIs(selected, probe)
        self.assertEqual(tier, "new")


if __name__ == "__main__":
    unittest.main()
