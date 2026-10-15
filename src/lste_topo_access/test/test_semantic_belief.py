#!/usr/bin/env python3
"""Regression tests for task-conditioned Place/Portal exploration policy."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_semantic_belief import (
    SemanticPlaceBelief,
    matched_terms,
    semantic_place_action,
    semantic_tokens,
    task_intent_from_message,
)


def task(**overrides):
    values = {
        "task_id": "yellow_cup",
        "target_name": "yellow cup",
        "target_attributes": ["ceramic", "handled"],
        "env_related_structures": ["office desk", "monitor", "chair"],
        "env_type_prior": ["office workspace"],
        "obj_key_objects": ["computer monitor", "other mugs"],
        "obj_negative_clues": ["door", "corridor"],
        "ctx_left": "monitor",
        "ctx_right": "monitor",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class SemanticBeliefTest(unittest.TestCase):
    def test_domain_aliases_make_detector_labels_backend_independent(self):
        self.assertEqual(semantic_tokens("computer monitors"), {"monitor"})
        self.assertEqual(semantic_tokens("office desks"), {"office", "desk"})
        intent = task_intent_from_message(task(), "version-a")
        self.assertIn("monitor", intent.context_terms)
        self.assertIn("cup", intent.target_terms)
        self.assertEqual(matched_terms("computer monitor", intent.context_terms), {"monitor"})

    def test_raw_task_json_contributes_location_prior_without_new_ros_fields(self):
        raw = (
            '{"task_parsed":{"target":{"typical_locations":["break room"]},'
            '"env":{"layout_prior":["long desks"]},'
            '"obj_related":{"search_strategy":"near monitors"}}}'
        )
        intent = task_intent_from_message(task(raw_json=raw), "version-a")
        self.assertIn("break", intent.context_terms)
        self.assertIn("long", intent.context_terms)
        self.assertIn("monitor", intent.context_terms)

    def test_negative_clues_do_not_become_positive_place_evidence(self):
        intent = task_intent_from_message(task(), "version-a")
        self.assertNotIn("door", intent.context_terms)
        self.assertEqual(matched_terms("door", intent.context_terms), set())
        self.assertEqual(matched_terms("door", intent.negative_terms), {"door"})

    def test_place_evidence_is_indexed_by_physical_place(self):
        belief = SemanticPlaceBelief(
            task_intent_from_message(task(), "version-a")
        )
        first = belief.observe(3, ["wall", "chair"], now=1.0)
        second = belief.observe(7, ["wall"], now=2.0)
        self.assertEqual(first["place_id"], 3)
        self.assertEqual(first["context_terms"], ["chair"])
        self.assertEqual(second["context_hits"], 0)
        self.assertEqual(belief.evidence(3)["observations"], 1)
        self.assertIsNone(belief.evidence(99))

    def test_target_detection_is_strong_evidence_for_current_place(self):
        intent = task_intent_from_message(task(), "version-a")
        belief = SemanticPlaceBelief(intent)
        evidence = belief.observe(
            4,
            ["yellow cup"],
            now=4.0,
            target_labels=["yellow cup"],
        )
        self.assertEqual(evidence["target_hits"], 1)
        self.assertEqual(
            semantic_place_action(intent, evidence, place_observed=True),
            "target_place",
        )

    def test_environment_label_sharing_target_noun_is_not_target_evidence(self):
        intent = task_intent_from_message(task(), "version-a")
        belief = SemanticPlaceBelief(intent)
        evidence = belief.observe(6, ["other cups"], now=4.0)
        self.assertEqual(evidence["target_hits"], 0)

    def test_unmatched_observed_place_expands_to_portal_without_thresholds(self):
        intent = task_intent_from_message(task(), "version-a")
        belief = SemanticPlaceBelief(intent)
        evidence = belief.observe(2, ["wall"], now=3.0)
        self.assertEqual(
            semantic_place_action(intent, evidence, place_observed=False),
            "bootstrap_observation",
        )
        self.assertEqual(
            semantic_place_action(intent, evidence, place_observed=True),
            "expand_unobserved_portal",
        )

    def test_context_evidence_does_not_lock_an_observed_place(self):
        intent = task_intent_from_message(task(), "version-a")
        belief = SemanticPlaceBelief(intent)
        evidence = belief.observe(5, ["monitor"], now=5.0)
        self.assertEqual(
            semantic_place_action(intent, evidence, place_observed=True),
            "context_place",
        )
        evidence = belief.observe(5, ["monitor"], now=6.0)
        self.assertEqual(
            semantic_place_action(intent, evidence, place_observed=True),
            "expand_unobserved_portal",
        )
        self.assertEqual(
            semantic_place_action(
                intent,
                evidence,
                place_observed=True,
                local_work_pending=False,
            ),
            "expand_unobserved_portal",
        )

    def test_reset_prevents_previous_task_semantic_leakage(self):
        belief = SemanticPlaceBelief(
            task_intent_from_message(task(), "version-a")
        )
        belief.observe(1, ["monitor"], now=1.0)
        belief.reset(task_intent_from_message(task(task_id="blue_mug"), "version-b"))
        self.assertIsNone(belief.evidence(1))
        self.assertEqual(belief.intent.task_id, "blue_mug")


if __name__ == "__main__":
    unittest.main()
