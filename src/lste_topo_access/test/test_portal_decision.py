"""Regression tests for the task-conditioned Portal decision contract."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_decision import (
    PortalDecisionCandidate,
    PortalDecisionEvidence,
    dominates,
    portal_action_category,
    select_portal_transition,
)


def evidence(**changes):
    values = {
        "task_relevance": 0.0,
        "novelty": 1.0,
        "information_gain": 10.0,
        "clearance": 0.8,
        "path_cost": 4.0,
        "risk": 0.1,
    }
    values.update(changes)
    return PortalDecisionEvidence(**values)


def candidate(identifier, category="new_place", values=None, tie_break=()):
    return PortalDecisionCandidate(
        candidate_id=identifier,
        category=category,
        evidence=evidence(**(values or {})),
        tie_break_key=tie_break,
    )


class PortalDecisionTest(unittest.TestCase):
    def test_action_category_is_discrete_graph_semantics(self):
        self.assertEqual(
            portal_action_category(
                task_relevant=True,
                destination_unobserved=False,
                information_gain=0,
            ),
            "target_relevant",
        )
        self.assertEqual(
            portal_action_category(
                task_relevant=False,
                destination_unobserved=True,
                information_gain=0,
            ),
            "new_place",
        )
        self.assertEqual(
            portal_action_category(
                task_relevant=False,
                destination_unobserved=False,
                information_gain=0,
            ),
            "covered_transit",
        )

    def test_task_relevant_action_precedes_more_distant_geometry(self):
        ordinary = candidate(
            "ordinary",
            values={"information_gain": 100.0, "path_cost": 1.0},
        )
        target = candidate(
            "target",
            category="target_relevant",
            values={"task_relevance": 1.0, "information_gain": 1.0, "path_cost": 30.0},
        )
        result = select_portal_transition((ordinary, target))
        self.assertIs(result.selected, target)
        self.assertEqual(result.category, "target_relevant")

    def test_new_place_precedes_covered_transit_without_a_weight(self):
        covered = candidate(
            "covered",
            category="covered_transit",
            values={"path_cost": 0.5, "information_gain": 100.0},
        )
        new_place = candidate(
            "new",
            category="new_place",
            values={"information_gain": 1.0, "path_cost": 20.0},
        )
        result = select_portal_transition((covered, new_place))
        self.assertIs(result.selected, new_place)

    def test_dominated_portal_is_removed_inside_one_action_class(self):
        strong = candidate(
            "strong",
            values={"information_gain": 20.0, "path_cost": 3.0, "risk": 0.1},
        )
        weak = candidate(
            "weak",
            values={"information_gain": 10.0, "path_cost": 5.0, "risk": 0.2},
        )
        self.assertTrue(dominates(strong, weak))
        result = select_portal_transition((weak, strong))
        self.assertEqual(result.pareto_front, (strong,))
        self.assertIs(result.selected, strong)

    def test_incomparable_portals_use_identity_tie_break(self):
        long_information = candidate(
            "long",
            values={"information_gain": 50.0, "path_cost": 10.0},
            tie_break=(2,),
        )
        short_path = candidate(
            "short",
            values={"information_gain": 5.0, "path_cost": 1.0},
            tie_break=(1,),
        )
        result = select_portal_transition((long_information, short_path))
        self.assertEqual(
            {item.candidate_id for item in result.pareto_front},
            {"long", "short"},
        )
        self.assertIs(result.selected, short_path)

    def test_invalid_value_is_rejected_before_selection(self):
        invalid = candidate("bad", values={"path_cost": float("nan")})
        result = select_portal_transition((invalid,))
        self.assertIsNone(result.selected)
        self.assertEqual(result.rejected[0].reasons, ("path_cost",))


if __name__ == "__main__":
    unittest.main()
