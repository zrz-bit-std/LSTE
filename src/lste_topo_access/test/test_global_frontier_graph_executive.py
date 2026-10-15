"""Regression tests for the graph-first exploration action boundary."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_graph_executive import (
    ACTION_BOOTSTRAP,
    ACTION_HOLD,
    ACTION_LOCAL_WORK,
    ACTION_PORTAL_PROBE,
    ACTION_VIEWPOINT_RETRY,
    choose_graph_action,
)
from global_frontier_place_states import PLACE_DORMANT, PLACE_OPEN


class GlobalFrontierGraphExecutiveTest(unittest.TestCase):
    def test_observed_place_requires_owned_work_before_local_dispatch(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=0,
            work_item_required=True,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(
            decision.reason,
            "observed_place_local_candidate_without_work_item",
        )
        self.assertFalse(decision.executable)

    def test_unresolved_work_is_the_only_local_action(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=0,
            work_item_id=17,
            work_item_available=True,
            work_item_required=True,
        )

        self.assertEqual(decision.kind, ACTION_LOCAL_WORK)
        self.assertTrue(decision.executable)

    def test_failed_viewpoint_reuses_work_item_as_a_new_viewpoint(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=0,
            work_item_id=17,
            work_item_available=True,
            viewpoint_retry=True,
            work_item_required=True,
        )

        self.assertEqual(decision.kind, ACTION_VIEWPOINT_RETRY)
        self.assertEqual(decision.work_item_id, 17)

    def test_wall_bounded_unknown_becomes_probe_without_crossing(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=0,
            portal_probe=True,
            work_item_required=True,
        )

        self.assertEqual(decision.kind, ACTION_PORTAL_PROBE)
        self.assertEqual(decision.place_hops, 0)

    def test_dormant_place_can_never_reopen_local_work(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_DORMANT,
            source_place_observed=True,
            place_hops=0,
            work_item_id=17,
            work_item_available=True,
            work_item_required=True,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(decision.reason, "covered_place_is_transit_only")

    def test_cross_place_route_requires_independent_portal_proof(self):
        without_proof = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=1,
        )
        with_proof = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=1,
            portal_certified=True,
        )

        self.assertEqual(without_proof.kind, ACTION_HOLD)
        self.assertEqual(with_proof.kind, "cross_portal")

    def test_pending_local_work_blocks_crossing_until_observation_is_resolved(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=1,
            portal_certified=True,
            work_item_required=True,
            local_work_pending=True,
            must_complete_local_work=True,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(decision.reason, "local_work_pending_before_crossing")

    def test_branch_first_allows_certified_portal_after_first_observation(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=1,
            portal_certified=True,
            work_item_required=True,
            local_work_pending=True,
            must_complete_local_work=True,
            portal_branch_priority=True,
            portal_destination_unobserved=True,
        )

        self.assertEqual(decision.kind, "cross_portal")
        self.assertEqual(
            decision.reason,
            "certified_portal_branch_to_unobserved_place",
        )

    def test_branch_first_cannot_bypass_work_for_covered_transit(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=1,
            portal_certified=True,
            work_item_required=True,
            local_work_pending=True,
            must_complete_local_work=True,
            portal_branch_priority=True,
            portal_destination_unobserved=False,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(decision.reason, "local_work_pending_before_crossing")

    def test_branch_first_cannot_suspend_target_observation_work(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=1,
            portal_certified=True,
            work_item_required=True,
            local_work_pending=True,
            portal_branch_priority=True,
            portal_destination_unobserved=True,
            target_work_pending=True,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(
            decision.reason,
            "target_observation_required_before_crossing",
        )

    def test_branch_first_still_requires_first_observation(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=4,
            source_place_state=PLACE_OPEN,
            source_place_observed=False,
            place_hops=1,
            portal_certified=True,
            work_item_required=True,
            portal_branch_priority=True,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(
            decision.reason,
            "place_observation_required_before_crossing",
        )

    def test_unready_graph_preserves_geometry_baseline(self):
        decision = choose_graph_action(
            graph_ready=False,
            source_place_id=None,
            place_hops=None,
        )

        self.assertEqual(decision.kind, "geometry_frontier")
        self.assertTrue(decision.executable)

    def test_open_place_before_first_observation_is_bootstrap(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=1,
            source_place_state=PLACE_OPEN,
            source_place_observed=False,
            place_hops=0,
            work_item_required=True,
        )

        self.assertEqual(decision.kind, ACTION_BOOTSTRAP)

    def test_unobserved_destination_place_cannot_immediately_cross_another_portal(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=2,
            source_place_state=PLACE_OPEN,
            source_place_observed=False,
            place_hops=1,
            portal_certified=True,
            work_item_required=True,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(
            decision.reason,
            "place_observation_required_before_crossing",
        )

    def test_local_work_gate_does_not_depend_on_target_claim(self):
        decision = choose_graph_action(
            graph_ready=True,
            source_place_id=2,
            source_place_state=PLACE_OPEN,
            source_place_observed=True,
            place_hops=1,
            portal_certified=True,
            work_item_required=True,
            local_work_pending=True,
        )

        self.assertEqual(decision.kind, ACTION_HOLD)
        self.assertEqual(
            decision.reason,
            "local_work_pending_before_crossing",
        )


if __name__ == "__main__":
    unittest.main()
