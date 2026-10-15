"""Focused tests for the pure PortalProbeValue/Pareto selector."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_portal_probe_value import (
    PortalProbeCandidate,
    PortalProbeConstraints,
    PortalProbeValue,
    dominates,
    filter_hard_constraints,
    pareto_front,
    select_portal_probe,
)


def value(**changes):
    fields = {
        "information_gain": 5.0,
        "task_relevance": 0.5,
        "novelty": 0.5,
        "path_cost": 4.0,
        "risk": 0.2,
        "clearance": 0.8,
    }
    fields.update(changes)
    return PortalProbeValue(**fields)


def constraints(**changes):
    fields = {
        "route_reachable": True,
        "collision_free": True,
        "source_place_current": True,
        "physical_identity_valid": True,
        "probe_available": True,
        "viewpoint_available": True,
    }
    fields.update(changes)
    return PortalProbeConstraints(**fields)


def candidate(
    candidate_id,
    category="probe",
    probe_value=None,
    probe_constraints=None,
    tie_break_key=(),
):
    return PortalProbeCandidate(
        candidate_id=candidate_id,
        action_category=category,
        value=value() if probe_value is None else probe_value,
        constraints=(constraints() if probe_constraints is None else probe_constraints),
        tie_break_key=tie_break_key,
    )


class PortalProbeValueTest(unittest.TestCase):
    def test_hard_constraints_are_applied_before_any_value_comparison(self):
        valid = candidate("valid")
        blocked = candidate(
            "blocked",
            probe_value=value(information_gain=100.0),
            probe_constraints=constraints(collision_free=False),
        )

        self.assertEqual(filter_hard_constraints((valid, blocked)), (valid,))
        result = select_portal_probe((blocked, valid))
        self.assertIs(result.selected, valid)
        self.assertEqual(result.rejected[0].reasons, ("collision_free",))

    def test_action_category_order_beats_a_stronger_lower_priority_value(self):
        retry = candidate(
            "retry",
            category="viewpoint_retry",
            probe_value=value(
                information_gain=1.0,
                task_relevance=0.0,
                novelty=0.0,
                path_cost=99.0,
                risk=1.0,
                clearance=0.1,
            ),
        )
        probe = candidate(
            "probe",
            category="probe",
            probe_value=value(
                information_gain=99.0,
                task_relevance=1.0,
                novelty=1.0,
                path_cost=1.0,
                risk=0.0,
                clearance=1.0,
            ),
        )

        result = select_portal_probe((probe, retry))

        self.assertIs(result.selected, retry)
        self.assertEqual(result.action_category, "viewpoint_retry")
        self.assertEqual(result.pareto_front, (retry,))

    def test_pareto_front_keeps_incomparable_information_cost_tradeoff(self):
        information = candidate(
            "information",
            probe_value=value(information_gain=10.0, path_cost=10.0),
        )
        cheap = candidate(
            "cheap",
            probe_value=value(information_gain=5.0, path_cost=2.0),
        )
        dominated = candidate(
            "dominated",
            probe_value=value(information_gain=4.0, path_cost=12.0),
        )

        self.assertTrue(dominates(information, dominated))
        self.assertTrue(dominates(cheap, dominated))
        self.assertEqual(
            {item.candidate_id for item in pareto_front((information, cheap, dominated))},
            {"information", "cheap"},
        )

    def test_no_weighted_total_score_is_used_for_pareto_tie_break(self):
        high_information = candidate(
            "high_information",
            probe_value=value(
                information_gain=100.0,
                task_relevance=0.0,
                novelty=0.0,
                path_cost=100.0,
                risk=1.0,
                clearance=0.1,
            ),
            tie_break_key=(2,),
        )
        cheap_safe = candidate(
            "cheap_safe",
            probe_value=value(
                information_gain=0.0,
                task_relevance=1.0,
                novelty=1.0,
                path_cost=0.0,
                risk=0.0,
                clearance=1.0,
            ),
            tie_break_key=(1,),
        )

        result = select_portal_probe((high_information, cheap_safe))

        self.assertEqual(
            {item.candidate_id for item in result.pareto_front},
            {"high_information", "cheap_safe"},
        )
        self.assertIs(result.selected, cheap_safe)

    def test_stable_tie_break_is_independent_of_input_order(self):
        first = candidate("b", tie_break_key=(2,))
        second = candidate("a", tie_break_key=(1,))

        forward = select_portal_probe((first, second))
        reverse = select_portal_probe((second, first))

        self.assertIs(forward.selected, second)
        self.assertIs(reverse.selected, second)

    def test_non_finite_value_is_a_rejection(self):
        invalid = candidate("nan", probe_value=value(path_cost=float("nan")))
        result = select_portal_probe((invalid,))

        self.assertIsNone(result.selected)
        self.assertEqual(result.rejected[0].reasons, ("path_cost",))

    def test_experiment_can_supply_an_explicit_category_order_mapping(self):
        local = candidate("local", category="local")
        probe = candidate("probe", category="probe")

        result = select_portal_probe(
            (local, probe),
            {"local": 0, "probe": 1},
        )

        self.assertIs(result.selected, local)
        self.assertEqual(result.action_category, "local")


if __name__ == "__main__":
    unittest.main()
