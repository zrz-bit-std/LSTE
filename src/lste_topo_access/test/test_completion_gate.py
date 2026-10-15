"""Regression tests for graph-level exploration completion."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_completion_gate import evaluate_completion


class RegionMemory:
    def __init__(self, *place_ids):
        self.regions = [{"id": place_id} for place_id in place_ids]


class CountLedger:
    def __init__(self, counts):
        self.counts = counts

    def unresolved_count(self, place_id):
        return self.counts.get(place_id, 0)


class HypothesisLedger:
    def __init__(self, records):
        self.records = records

    def unbound_for_source(self, place_id):
        return [record for record in self.records if record == place_id]


class TargetWork:
    def __init__(self, pending):
        self.pending = pending

    def pending_place_ids(self):
        return tuple(self.pending)


class CompletionGateTest(unittest.TestCase):
    def test_empty_candidate_set_is_not_complete_with_unresolved_graph_work(self):
        result = evaluate_completion(
            graph_ready=True,
            region_memory=RegionMemory(1, 2),
            work_item_ledger=CountLedger({1: 2}),
            portal_probe_ledger=CountLedger({2: 1}),
            portal_hypothesis_ledger=HypothesisLedger([2]),
            target_observation_work=TargetWork([1]),
        )
        self.assertFalse(result.complete)
        self.assertEqual(
            result.reasons,
            (
                "unresolved_work_items",
                "unresolved_portal_probes",
                "unbound_portal_hypotheses",
                "pending_target_observations",
            ),
        )

    def test_only_settled_graph_can_be_terminal(self):
        result = evaluate_completion(
            graph_ready=True,
            region_memory=RegionMemory(1),
            work_item_ledger=CountLedger({1: 0}),
            portal_probe_ledger=CountLedger({1: 0}),
            portal_hypothesis_ledger=HypothesisLedger([]),
            target_observation_work=TargetWork([]),
        )
        self.assertTrue(result.complete)
        self.assertEqual(result.reasons, ())
        self.assertEqual(sum(result.counts.values()), 0)

    def test_graph_not_ready_cannot_claim_completion(self):
        result = evaluate_completion(graph_ready=False)
        self.assertFalse(result.complete)
        self.assertEqual(result.reasons, ("graph_not_ready",))


if __name__ == "__main__":
    unittest.main()
