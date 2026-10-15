"""Regression tests for the graph method's local action boundary."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_frontier_decision import (
    FrontierActionCandidate,
    FrontierDecisionEvidence,
    candidate_from_route,
    decision_violations,
    select_frontier_action,
)
from global_frontier_experiment_policy import resolve_exploration_method


def route(
    *, category="local", work_item=1, information=10.0, path=4.0,
    structure=1.0, graph_action="observe_local_work", hops=0, gate=None,
):
    """Build the compatibility tuple produced by candidate scoring."""
    return (
        3, 4, 1.0, 2.0, path, information, structure, 99.0, None, hops,
        "frontier_endpoint", gate, work_item, "created", 2, None, False,
        graph_action, "test", category,
    )


class FrontierActionDecisionTest(unittest.TestCase):
    def test_category_precedes_legacy_score(self):
        local = candidate_from_route(
            route(category="local", information=100.0, path=1.0),
            ranking_category="local",
        )
        retry = candidate_from_route(
            route(category="viewpoint_retry", information=1.0, path=100.0),
            ranking_category="viewpoint_retry",
        )
        decision = select_frontier_action((local, retry))
        self.assertIs(decision.selected, retry)
        self.assertEqual(decision.category, "viewpoint_retry")

    def test_pareto_keeps_incomparable_information_and_path(self):
        information = candidate_from_route(
            route(information=20.0, path=8.0), ranking_category="local",
        )
        short = candidate_from_route(
            route(information=5.0, path=2.0), ranking_category="local",
        )
        decision = select_frontier_action((information, short))
        self.assertEqual(len(decision.pareto_front), 2)
        self.assertIn(decision.selected, decision.pareto_front)

    def test_input_order_does_not_change_stable_selection(self):
        first = candidate_from_route(
            route(work_item=8, information=5.0), ranking_category="local",
        )
        second = candidate_from_route(
            route(work_item=9, information=5.0), ranking_category="local",
        )
        forward = select_frontier_action((first, second)).selected
        reverse = select_frontier_action((second, first)).selected
        self.assertEqual(forward.candidate_id, reverse.candidate_id)

    def test_legacy_score_field_cannot_break_an_evidence_tie(self):
        left_route = list(route(work_item=2, information=5.0))
        right_route = list(route(work_item=1, information=5.0))
        left_route[7] = -1000.0
        right_route[7] = 1000.0
        left = candidate_from_route(tuple(left_route), ranking_category="local")
        right = candidate_from_route(tuple(right_route), ranking_category="local")
        decision = select_frontier_action((left, right))
        self.assertEqual(decision.selected.candidate_id[-1], 1)

    def test_invalid_portal_cannot_win_with_high_information(self):
        candidate = FrontierActionCandidate(
            candidate_id="uncertified",
            action_category="adjacent",
            route=route(category="adjacent", work_item=None, hops=1),
            evidence=FrontierDecisionEvidence(1000, 1000, 0, 1, 1),
            portal_certified=False,
        )
        decision = select_frontier_action((candidate,))
        self.assertIsNone(decision.selected)
        self.assertEqual(decision.rejected[0].reasons, ("portal_not_certified",))

    def test_resolved_work_item_is_never_eligible(self):
        candidate = FrontierActionCandidate(
            candidate_id="resolved",
            action_category="local",
            route=route(),
            evidence=FrontierDecisionEvidence(10, 1, 0, 1, 1),
            work_item_unresolved=False,
        )
        self.assertIn("work_item_not_unresolved", decision_violations(candidate))
        self.assertIsNone(select_frontier_action((candidate,)).selected)

    def test_full_methods_use_event_pareto_and_baselines_do_not(self):
        self.assertEqual(
            resolve_exploration_method("place_portal_workitem").frontier_action_policy,
            "event_pareto",
        )
        self.assertEqual(
            resolve_exploration_method("place_portal_workitem_strict").frontier_action_policy,
            "event_pareto",
        )
        self.assertEqual(
            resolve_exploration_method("frontier").frontier_action_policy,
            "legacy_scalar",
        )
        self.assertEqual(
            resolve_exploration_method("place_portal_workitem_legacy_rank").frontier_action_policy,
            "legacy_scalar",
        )


if __name__ == "__main__":
    unittest.main()
