"""Regression tests for the target-reinspection graph action contract."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_graph_route_gate import (  # noqa: E402
    candidate_refines_graph_plan,
    candidate_matches_graph_plan,
    graph_plan_is_exclusive,
)
from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_BOOTSTRAP,
    ACTION_CROSS_PORTAL,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    ACTION_REINSPECT_TARGET,
    GraphRoutePlan,
    PLAN_READY,
)


def route(action, *, work_item_id=7, hops=0, route_kind="frontier_endpoint", probe=None):
    return (
        1, 2, 1.0, 2.0, 1.0, 10.0, 0.0, 0.0, None, hops,
        route_kind, None, work_item_id, "matched", 4, probe, False,
        action, "test_candidate",
    )


def target_plan():
    return GraphRoutePlan(
        PLAN_READY,
        ACTION_REINSPECT_TARGET,
        current_place_id=1,
        target_place_id=1,
        obligation_kind="target_observation",
        reason="target_observation_required",
    )


def local_plan():
    return GraphRoutePlan(
        PLAN_READY,
        ACTION_OBSERVE_LOCAL_WORK,
        current_place_id=1,
        target_place_id=1,
        obligation_kind="work_item",
        obligation_id=7,
        reason="local_work_required",
    )


def bootstrap_plan():
    return GraphRoutePlan(
        PLAN_READY,
        ACTION_BOOTSTRAP,
        current_place_id=1,
        target_place_id=1,
        obligation_kind="work_item",
        obligation_id=7,
        reason="place_observation_required",
    )


class GraphRouteGateTest(unittest.TestCase):
    def test_bootstrap_accepts_same_work_item_rehydration(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=7),
            place_id=1,
        )

        self.assertTrue(candidate_refines_graph_plan(bootstrap_plan(), candidate))

    def test_bootstrap_accepts_same_place_work_item_refinement(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=8),
            place_id=1,
        )

        self.assertTrue(candidate_refines_graph_plan(bootstrap_plan(), candidate))

    def test_bootstrap_rejects_unidentified_refinement(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=8),
        )

        self.assertFalse(candidate_refines_graph_plan(bootstrap_plan(), candidate))

    def test_local_work_reconciliation_allows_same_place_same_action_class(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=8),
            place_id=1,
        )

        self.assertTrue(candidate_refines_graph_plan(local_plan(), candidate))

    def test_local_work_reconciliation_rejects_cross_place_candidate(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=8),
            place_id=2,
        )

        self.assertFalse(candidate_refines_graph_plan(local_plan(), candidate))

    def test_local_work_reconciliation_rejects_missing_place_identity(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=8),
        )

        self.assertFalse(candidate_refines_graph_plan(local_plan(), candidate))

    def test_target_reinspection_is_an_exclusive_graph_action(self):
        self.assertTrue(graph_plan_is_exclusive(target_plan()))
        self.assertTrue(
            candidate_matches_graph_plan(
                target_plan(), route(ACTION_OBSERVE_LOCAL_WORK)
            )
        )

    def test_target_reinspection_rejects_probe_and_portal(self):
        probe = type("Probe", (), {"probe_id": 4})()
        self.assertFalse(
            candidate_matches_graph_plan(
                target_plan(), route(ACTION_PROBE_PORTAL, probe=probe)
            )
        )
        self.assertFalse(
            candidate_matches_graph_plan(
                target_plan(),
                route(
                    ACTION_CROSS_PORTAL,
                    work_item_id=None,
                    hops=1,
                    route_kind="portal_transition",
                ),
            )
        )

    def test_target_reinspection_requires_explicit_same_place_hops(self):
        malformed = route(ACTION_OBSERVE_LOCAL_WORK)
        malformed = malformed[:9] + (None,) + malformed[10:]
        wrong_place = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK),
            place_id=2,
        )

        self.assertFalse(candidate_matches_graph_plan(target_plan(), malformed))
        self.assertFalse(candidate_matches_graph_plan(target_plan(), wrong_place))


if __name__ == "__main__":
    unittest.main()
