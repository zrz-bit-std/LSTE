"""Tests for the fast/slow structural event boundary."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_event_graph import EvidenceEventGraph  # noqa: E402


class EvidenceEventGraphTest(unittest.TestCase):
    def test_reactive_status_does_not_wake_deliberation(self):
        graph = EvidenceEventGraph()

        event = graph.ingest("heartbeat", {"route_id": 3})

        self.assertFalse(event.structural)
        self.assertFalse(event.wake)
        self.assertIsNone(graph.consume_wake())

    def test_structural_fact_wakes_once_by_durable_identity(self):
        graph = EvidenceEventGraph()
        fields = {"route_id": 4, "place_id": 2, "reason": "selected"}

        first = graph.ingest("frontier_place_closed", fields)
        duplicate = graph.ingest("frontier_place_closed", dict(fields))

        self.assertTrue(first.wake)
        self.assertFalse(duplicate.wake)
        wake = graph.consume_wake()
        self.assertEqual(wake.event, "frontier_place_closed")
        self.assertIsNone(graph.consume_wake())

    def test_interleaved_duplicate_events_remain_deduplicated(self):
        graph = EvidenceEventGraph()
        fields = {"portal_id": 7, "route_id": 8, "state": "selected"}

        graph.ingest("portal_hypothesis_selected", fields)
        graph.ingest("heartbeat", {})
        duplicate = graph.ingest("portal_hypothesis_selected", dict(fields))

        self.assertFalse(duplicate.wake)
        self.assertEqual(graph.structural_revision, 1)

    def test_distinct_route_ids_are_distinct_structural_facts(self):
        graph = EvidenceEventGraph()

        first = graph.ingest("route_command", {"route_id": 1, "route_kind": "frontier_endpoint"})
        second = graph.ingest("route_command", {"route_id": 2, "route_kind": "frontier_endpoint"})

        self.assertTrue(first.wake)
        self.assertTrue(second.wake)
        self.assertEqual(graph.active_route_id, 2)
        self.assertEqual(graph.report()["structural_revision"], 2)

    def test_terminal_closes_the_active_route_projection(self):
        graph = EvidenceEventGraph()
        graph.ingest("route_command", {"route_id": 5, "route_kind": "portal_transition"})
        graph.ingest("route_terminal", {"route_id": 5})

        report = graph.report()
        self.assertEqual(report["active_route_id"], 0)
        self.assertEqual(report["completed_route_count"], 1)

    def test_route_context_projects_place_and_work_item_identity(self):
        graph = EvidenceEventGraph()
        graph.ingest(
            "route_command",
            {
                "route_id": 9,
                "goal_context": {
                    "owner_place_id": 4,
                    "work_item_id": 12,
                    "task_version": "task-v2",
                },
            },
        )

        report = graph.report()
        self.assertEqual(report["place_count"], 1)
        self.assertEqual(report["work_item_count"], 1)
        self.assertEqual(report["task_version"], "task-v2")

    def test_place_portal_and_target_facts_are_replayable(self):
        graph = EvidenceEventGraph()
        graph.ingest("physical_place_bootstrapped", {"place_id": 1, "state": "open"})
        graph.ingest("portal_hypothesis_certified", {"portal_id": 2, "place_id": 1})
        graph.ingest("frontier_action_selected", {"work_item_id": 3, "category": "local"})
        graph.ingest("target_track_started", {"place_id": 1})

        report = graph.report()
        self.assertEqual(report["place_count"], 1)
        self.assertEqual(report["portal_count"], 1)
        self.assertEqual(report["work_item_count"], 1)
        self.assertEqual(report["target_state"], "target_track_started")

    def test_work_item_route_rejection_is_a_deduplicated_structural_fact(self):
        graph = EvidenceEventGraph()
        fields = {"work_item_id": 12, "route_id": 4, "reason": "navfn_endpoint_offset"}

        first = graph.ingest("work_item_viewpoint_route_rejected", fields)
        duplicate = graph.ingest(
            "work_item_viewpoint_route_rejected", dict(fields)
        )

        self.assertTrue(first.structural)
        self.assertTrue(first.wake)
        self.assertFalse(duplicate.wake)
        self.assertEqual(graph.report()["work_item_count"], 1)


if __name__ == "__main__":
    unittest.main()
