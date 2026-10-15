"""Regression tests for causal Place/Portal event envelopes."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_transition import build_transition_envelope  # noqa: E402


class TransitionEnvelopeTest(unittest.TestCase):
    def test_arrival_separates_source_plan_from_destination_commit(self):
        envelope = build_transition_envelope(
            "portal_place_entered",
            {
                "route_id": 7,
                "region_id": 2,
                "portal_transaction": {
                    "transaction_id": 3,
                    "route_id": 7,
                    "portal_id": 11,
                    "source_place_id": 1,
                    "state": "place_commit",
                },
            },
        )

        self.assertEqual(
            envelope,
            {
                "from_place_id": 1,
                "to_place_id": 2,
                "portal_id": 11,
                "transaction_id": 3,
                "route_id": 7,
                "phase": "place_commit",
                "event": "portal_place_entered",
                "evidence": "",
            },
        )

    def test_late_event_does_not_inherit_another_portal_transaction(self):
        envelope = build_transition_envelope(
            "portal_hypothesis_failed",
            {
                "portal_id": 17,
                "source_place_id": 3,
                "route_id": 14,
                "reason": "move_base_aborted",
                "portal_transaction": {
                    "transaction_id": 2,
                    "route_id": 5,
                    "portal_id": 2,
                    "source_place_id": 2,
                    "state": "idle",
                },
            },
        )

        self.assertEqual(envelope["from_place_id"], 3)
        self.assertIsNone(envelope["to_place_id"])
        self.assertEqual(envelope["portal_id"], 17)
        self.assertIsNone(envelope["transaction_id"])
        self.assertEqual(envelope["route_id"], 14)

    def test_crossed_portal_execution_failure_is_not_physical_failure(self):
        envelope = build_transition_envelope(
            "portal_execution_failure_observed",
            {
                "portal_id": 4,
                "source_place_id": 2,
                "route_id": 31,
                "state": "crossed",
                "execution_failure_count": 1,
                "traversal_source_place_id": 3,
                "traversal_destination_place_id": 2,
            },
        )

        self.assertEqual(envelope["from_place_id"], 2)
        self.assertIsNone(envelope["to_place_id"])
        self.assertEqual(envelope["portal_id"], 4)
        self.assertEqual(envelope["route_id"], 31)

    def test_departure_event_has_source_only(self):
        envelope = build_transition_envelope(
            "frontier_place_boundary_crossed",
            {"region_id": 4, "route_id": 8, "departure_basis": "structural_transition"},
        )

        self.assertEqual(envelope["from_place_id"], 4)
        self.assertIsNone(envelope["to_place_id"])
        self.assertEqual(envelope["route_id"], 8)

    def test_unrelated_status_has_no_envelope(self):
        self.assertIsNone(build_transition_envelope("heartbeat", {"route_id": 8}))


if __name__ == "__main__":
    unittest.main()
