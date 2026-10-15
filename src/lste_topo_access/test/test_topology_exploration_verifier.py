#!/usr/bin/env python3
"""Regression tests for physical-room topology acceptance criteria."""

import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts/tests/office_building"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
SOURCE = SCRIPTS / "verify_topology_exploration.py"
SPEC = importlib.util.spec_from_file_location("topology_verifier", SOURCE)
VERIFIER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(VERIFIER)


class TopologyExplorationVerifierTest(unittest.TestCase):
    def summary(self, reentries=0, dwell=45.0, aborts=0):
        return {
            "room_coverage": {
                "total_room_reentries": reentries,
                "max_dwell_seconds_by_region": {
                    "main_corridor": 55.0,
                    "open_office": dwell,
                    "printer": 42.0,
                    "lobby": 18.0,
                },
            },
            "move_base_aborts": aborts,
        }

    def verify(self, summary, events=()):
        return VERIFIER.verify(
            summary,
            events,
            min_distinct_rooms=3,
            max_room_dwell_seconds=120.0,
            max_room_reentries=0,
            max_move_base_aborts=0,
            max_bridge_retries=0,
        )

    def test_accepts_distinct_rooms_without_reentry_or_stall(self):
        result = self.verify(self.summary())

        self.assertTrue(result["passed"])
        self.assertEqual(result["evidence"]["distinct_room_count"], 3)

    def test_rejects_a_physical_room_reentry(self):
        result = self.verify(self.summary(reentries=1))

        self.assertFalse(result["passed"])
        self.assertIn("physical room re-entries=1, limit=0", result["violations"])

    def test_rejects_dwell_and_same_route_bridge_retry(self):
        events = [
            (
                "teb_bridge_event",
                {"bridge_event": "dispatch", "reason": "retry_move_base_goal"},
            )
        ]
        result = self.verify(self.summary(dwell=121.0), events)

        self.assertFalse(result["passed"])
        self.assertEqual(result["evidence"]["bridge_retry_move_base_goals"], 1)
        self.assertEqual(len(result["violations"]), 2)

    def test_rejects_an_observed_place_cross_room_endpoint(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "source_place_observed": True,
                    "route_id": 17,
                    "route_kind": "frontier_endpoint",
                    "place_graph_hops": 1,
                    "goal": [15.0, 4.0],
                    "ros_time": 123.0,
                },
            )
        ]

        result = self.verify(self.summary(), events)

        self.assertFalse(result["passed"])
        self.assertEqual(
            result["evidence"]["cross_place_endpoint_bypasses"],
            [{
                "route_id": 17,
                "route_kind": "frontier_endpoint",
                "place_hops": 1,
                "goal": [15.0, 4.0],
                "ros_time": 123.0,
            }],
        )
        self.assertIn(
            "observed-place cross-room endpoint bypasses=1; explicit portal required",
            result["violations"],
        )

    def test_rejects_a_portal_self_loop(self):
        events = [
            (
                "global_frontier_event",
                {
                    "event": "route_command",
                    "route_id": 22,
                    "ros_time": 456.0,
                    "portal_hypotheses": {
                        "records": [
                            {
                                "id": 7,
                                "source_place_id": 2,
                                "destination_place_id": 2,
                                "state": "crossed",
                            }
                        ]
                    },
                },
            )
        ]

        result = self.verify(self.summary(), events)

        self.assertFalse(result["passed"])
        self.assertEqual(
            result["evidence"]["portal_self_loops"],
            [{
                "portal_id": 7,
                "source_place_id": 2,
                "destination_place_id": 2,
                "route_id": 22,
                "ros_time": 456.0,
            }],
        )
        self.assertIn(
            "Portal self-loops=1; source and destination Place must differ",
            result["violations"],
        )

    def test_rejects_resolved_work_item_descendant_dispatch(self):
        summary = self.summary()
        summary["frontier_region_lifecycle"] = {
            "resolved_work_item_descendant_dispatches": [
                {"route_id": 9, "work_item_id": 3, "goal": [2.0, 4.0]}
            ],
        }

        result = self.verify(summary)

        self.assertFalse(result["passed"])
        self.assertIn(
            "resolved ObservationWorkItem descendants dispatched=1",
            result["violations"],
        )

    def test_corridor_dwell_is_a_stall_signal(self):
        result = self.verify({
            "room_coverage": {
                "total_room_reentries": 0,
                "max_dwell_seconds_by_region": {
                    "main_corridor": 121.0,
                    "open_office": 20.0,
                    "printer": 18.0,
                    "lobby": 10.0,
                },
            },
            "move_base_aborts": 0,
        })

        self.assertFalse(result["passed"])
        self.assertIn(
            "max physical-room dwell=121.000s, limit=120.000s",
            result["violations"],
        )

    def test_rejects_repeated_work_item_collision_and_stall_evidence(self):
        summary = self.summary()
        summary["frontier_region_lifecycle"] = {
            "repeated_work_item_dispatches": {"12": 1},
        }
        summary["failure_safety"] = {
            "collision_events": 1,
            "collision_truth_status": "measured",
        }
        summary["stall_diagnostics"] = {
            "route_stagnant_observed_count": 1,
            "frontier_route_unavailable_event_count": 2,
            "max_frontier_route_unavailable_span_seconds": 61.0,
        }
        summary["motion_smoothness"] = {
            "max_stop_duration_seconds": 61.0,
        }

        result = self.verify(summary)

        self.assertFalse(result["passed"])
        self.assertIn(
            "resolved ObservationWorkItem re-dispatches=1, limit=0",
            result["violations"],
        )
        self.assertIn("collision events=1, limit=0", result["violations"])
        self.assertIn(
            "route stagnation observations=1, limit=0",
            result["violations"],
        )
        self.assertIn(
            "frontier route unavailable span=61.000s, limit=60.000s",
            result["violations"],
        )
        self.assertIn(
            "max zero-velocity stop=61.000s, limit=60.000s",
            result["violations"],
        )


if __name__ == "__main__":
    unittest.main()
