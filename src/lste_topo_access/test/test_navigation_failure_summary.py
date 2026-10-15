#!/usr/bin/env python3
"""Regression tests for offline failure-episode aggregation."""

import importlib.util
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[3]
    / "scripts"
    / "tools"
    / "analyze_navigation_metrics.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_navigation_metrics", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class NavigationFailureSummaryTest(unittest.TestCase):
    def test_closed_and_open_episodes_remain_visible(self):
        records = [
            {
                "_event": "failure_started",
                "failure_id": "run-F0001",
                "trigger": "move_base_terminal_failure",
                "source": "move_base",
                "classification": {"label": "planner_no_path", "confidence": "high"},
                "started_wall_elapsed_seconds": 10.0,
            },
            {
                "_event": "failure_related_event",
                "failure_id": "run-F0001",
                "trigger": "frontier_route_failure",
            },
            {
                "_event": "failure_snapshot_ready",
                "failure_id": "run-F0001",
                "trigger": "move_base_terminal_failure",
                "source": "move_base",
                "classification": "planner_no_path",
                "confidence": "high",
                "ended_wall_elapsed_seconds": 15.0,
                "duration_seconds": 5.0,
                "sample_count": 26,
            },
            {
                "_event": "failure_started",
                "failure_id": "run-F0002",
                "trigger": "zero_velocity_stall",
                "source": "metrics_watchdog",
                "classification": {"label": "controller_stall", "confidence": "high"},
                "started_wall_elapsed_seconds": 30.0,
            },
        ]
        summary = MODULE.failure_episode_summary(records)
        self.assertEqual(summary["episode_count"], 2)
        self.assertEqual(summary["closed_count"], 1)
        self.assertEqual(summary["open_count"], 1)
        self.assertEqual(summary["by_classification"], {
            "controller_stall": 1,
            "planner_no_path": 1,
        })
        self.assertEqual(summary["episodes"][0]["related_event_count"], 1)

    def test_snapshot_adds_direct_trigger_location_and_planner_evidence(self):
        records = [
            {
                "_event": "failure_started",
                "failure_id": "run-F0003",
                "trigger": "unexpected_preemption",
                "source": "move_base",
                "classification": {"label": "owner_transaction_race", "confidence": "high"},
                "started_wall_elapsed_seconds": 12.0,
            },
            {
                "_event": "failure_snapshot_ready",
                "failure_id": "run-F0003",
                "classification": "owner_transaction_race",
                "confidence": "high",
                "ended_wall_elapsed_seconds": 17.0,
                "duration_seconds": 5.0,
                "sample_count": 26,
            },
        ]
        snapshots = {
            "run-F0003": {
                "failure_id": "run-F0003",
                "route_context_at_trigger": {
                    "route": {"route_id": 4, "route_kind": "portal_transition"},
                    "contexts": {"move_base": {"payload": {"status": 2}}},
                },
                "trigger_sample": {
                    "pose": [1.0, 2.0],
                    "goal": [3.0, 4.0],
                    "distance_to_goal": 2.8,
                    "teb_status": "trajectory_valid",
                    "scan": {"forward_min": 4.0},
                    "cmd_vel": [0.2, 0.0],
                },
                "end_sample": {"pose": [1.1, 2.0]},
                "diagnosis": {
                    "primary_cause": "planner_materialization_stall",
                    "layer": "global_frontier",
                    "route_id": 4,
                    "graph_action": "observe_local_work",
                    "confidence": "high",
                },
                "classification_at_end": {"label": "unknown", "confidence": "low"},
                "sample_count": 26,
            }
        }
        summary = MODULE.failure_episode_summary(records, snapshots)
        evidence = summary["episodes"][0]["evidence"]
        self.assertEqual(evidence["route_context_at_trigger"], {
            "route_id": 4,
            "route_kind": "portal_transition",
        })
        self.assertEqual(evidence["trigger_sample"]["pose"], [1.0, 2.0])
        self.assertEqual(evidence["trigger_sample"]["teb_status"], "trajectory_valid")
        self.assertEqual(
            summary["episodes"][0]["technical_diagnosis"]["primary_cause"],
            "planner_materialization_stall",
        )

    def test_technical_diagnosis_recovers_route_from_classifier_evidence(self):
        records = [
            {
                "_event": "failure_started",
                "failure_id": "run-F0004",
                "trigger": "route_invalidated_stall",
                "source": "global_frontier",
                "classification": {
                    "label": "controller_stall",
                    "confidence": "high",
                    "evidence": {
                        "route": {
                            "route_id": 19,
                            "route_kind": "frontier_endpoint",
                        }
                    },
                },
            },
            {
                "_event": "failure_snapshot_ready",
                "failure_id": "run-F0004",
                "classification": "controller_stall",
                "confidence": "high",
            },
        ]
        snapshots = {
            "run-F0004": {
                "diagnosis": {
                    "primary_cause": "controller_stall",
                    "layer": "global_frontier",
                    "route_id": 0,
                },
                "classification": {
                    "evidence": {
                        "route": {
                            "route_id": 19,
                            "route_kind": "frontier_endpoint",
                        }
                    }
                },
            }
        }
        summary = MODULE.failure_episode_summary(records, snapshots)
        diagnosis = summary["episodes"][0]["technical_diagnosis"]
        self.assertEqual(diagnosis["route_id"], 19)
        self.assertEqual(diagnosis["route_kind"], "frontier_endpoint")


if __name__ == "__main__":
    unittest.main()
