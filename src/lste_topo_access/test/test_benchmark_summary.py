#!/usr/bin/env python3
"""Regression tests for place-lifecycle benchmark evidence."""

import importlib.util
from pathlib import Path
import unittest


SOURCE = (
    Path(__file__).resolve().parents[3]
    / "scripts/tests/office_building/summarize_run.py"
)
SPEC = importlib.util.spec_from_file_location("office_benchmark_summary", SOURCE)
SUMMARY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(SUMMARY)


class FrontierLifecycleSummaryTest(unittest.TestCase):
    def test_room_truth_is_scoped_to_the_world_level_recorded_at_start(self):
        manifest = {
            "rooms": [
                {"id": "lobby", "bounds_m": [0.0, 0.0, 5.0, 5.0]},
                {"id": "open_office", "bounds_m": [5.0, 0.0, 10.0, 5.0]},
                {"id": "target_room", "bounds_m": [10.0, 0.0, 15.0, 5.0]},
            ],
            "main_corridor": {"bounds_m": [0.0, 5.0, 15.0, 7.0]},
            "levels": {
                "level_1": {
                    "world": "worlds/office_level_1.world",
                    "enabled_rooms": ["lobby", "main_corridor", "target_room"],
                },
                "level_2": {
                    "world": "worlds/office_level_2.world",
                    "enabled_rooms": [
                        "lobby", "main_corridor", "open_office", "target_room",
                    ],
                },
            },
        }
        events = [
            (
                "run_start",
                {"experiment": {"world": "/tmp/worlds/office_level_1.world"}},
            ),
        ]

        scoped, level = SUMMARY.scope_manifest_for_run(manifest, events)

        self.assertEqual(level, "level_1")
        self.assertEqual(
            [room["id"] for room in scoped["rooms"]],
            ["lobby", "target_room"],
        )
        self.assertIn("main_corridor", scoped)

        samples = [
            {"ros_time": 1.0, "pose": [6.0, 1.0]},
            {"ros_time": 1.5, "pose": [6.1, 1.0]},
            {"ros_time": 2.0, "pose": [6.0, 1.0]},
        ]
        coverage = SUMMARY.room_visit_summary(samples, scoped)
        self.assertEqual(coverage["entry_counts"], {})
        self.assertEqual(coverage["total_room_reentries"], 0)

    def test_missing_method_events_are_not_contract_consistent(self):
        result = SUMMARY.exploration_method_summary([])
        self.assertEqual(result["selected_method"], None)
        self.assertEqual(result["methods_observed"], [])
        self.assertFalse(result["method_contract_consistent"])
        self.assertFalse(result["frontier_action_policy_consistent"])

    def test_event_pareto_decisions_are_reduced_separately_from_graph_actions(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_action_selected",
                    "policy": "event_pareto",
                    "category": "local",
                    "candidate_count": 8,
                    "feasible_count": 6,
                    "pareto_front_count": 2,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_action_selected",
                    "policy": "event_pareto",
                    "category": "probe",
                    "candidate_count": 3,
                    "feasible_count": 1,
                    "pareto_front_count": 1,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "exploration_method": "place_portal_workitem",
                    "frontier_action_policy": "event_pareto",
                    "graph_action": "observe_local_work",
                },
            ),
        ]
        lifecycle = SUMMARY.frontier_region_summary(events)
        method = SUMMARY.exploration_method_summary(events)
        self.assertEqual(
            lifecycle["frontier_decision_category_counts"],
            {"local": 1, "probe": 1},
        )
        self.assertEqual(lifecycle["frontier_decision_count"], 2)
        self.assertEqual(lifecycle["frontier_decision_candidate_total"], 11)
        self.assertEqual(lifecycle["frontier_decision_feasible_total"], 7)
        self.assertEqual(lifecycle["frontier_decision_pareto_total"], 3)
        self.assertEqual(method["frontier_action_policies"], ["event_pareto"])
        self.assertTrue(method["frontier_action_policy_consistent"])

    def test_legacy_nested_goal_context_keeps_method_contract_comparable(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_dispatched",
                    "goal_context": {
                        "exploration_method": "place_portal_workitem",
                    },
                    "semantic_place": {
                        "frontier_action_policy": "event_pareto",
                    },
                },
            ),
        ]

        method = SUMMARY.exploration_method_summary(events)

        self.assertEqual(
            method["selected_method"], "place_portal_workitem",
        )
        self.assertEqual(
            method["frontier_action_policies"], ["event_pareto"],
        )
        self.assertTrue(method["method_contract_consistent"])
        self.assertTrue(method["frontier_action_policy_consistent"])

    def test_graph_route_events_preserve_multihop_evidence(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "graph_route_plan_selected",
                    "graph_route_plan": {
                        "status": "ready",
                        "action": "cross_portal",
                        "portal_path": [11, 12],
                        "first_portal_id": 11,
                    },
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "graph_route_edge_materialized",
                    "first_portal_id": 11,
                },
            ),
        ]

        lifecycle = SUMMARY.frontier_region_summary(events)

        self.assertEqual(lifecycle["graph_route_plan_count"], 1)
        self.assertEqual(lifecycle["graph_route_multihop_plan_count"], 1)
        self.assertEqual(lifecycle["graph_route_max_hops"], 2)
        self.assertEqual(lifecycle["graph_route_edge_materialized"], 1)

    def test_graph_route_transaction_events_are_audited(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "graph_route_plan_reconciled",
                    "transaction_id": 4,
                    "planned_action": "observe_local_work",
                    "committed_action": "cross_portal",
                    "reason": "certified_portal_branch_to_unobserved_place",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "graph_route_action_committed",
                    "action": "cross_portal",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "route_id": 8,
                    "graph_action": "cross_portal",
                    "graph_route_plan": {"action": "cross_portal"},
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "graph_route_plan_mismatch",
                    "transaction_id": 5,
                    "planned_action": "cross_portal",
                    "candidate_action": "observe_local_work",
                    "reason": "portal_crossing_intent_replaced",
                },
            ),
        ]

        lifecycle = SUMMARY.frontier_region_summary(events)
        invariants = lifecycle["graph_invariants"]

        self.assertEqual(lifecycle["graph_route_plan_reconciliation_count"], 1)
        self.assertEqual(lifecycle["graph_route_plan_mismatch_count"], 1)
        self.assertEqual(
            lifecycle["graph_route_committed_action_counts"],
            {"cross_portal": 1},
        )
        self.assertEqual(lifecycle["route_plan_action_mismatch_count"], 0)
        self.assertEqual(invariants["graph_plan_reconciliation_count"], 1)
        self.assertEqual(invariants["graph_plan_mismatch_count"], 1)

    def test_work_item_execution_uses_explicit_lifecycle_not_room_dwell(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_dispatched",
                    "work_item_id": 12,
                    "place_id": 4,
                    "route_id": 7,
                    "ros_time": 10.0,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_settled",
                    "work_item_id": 12,
                    "place_id": 4,
                    "work_item_state": "resolved",
                    "reason": "endpoint_coverage_recorded",
                    "ros_time": 16.25,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_dispatched",
                    "work_item_id": 13,
                    "place_id": 4,
                    "route_id": 8,
                    "ros_time": 20.0,
                },
            ),
        ]

        summary = SUMMARY.work_item_execution_summary(events, run_end_seconds=24.0)

        self.assertEqual(summary["status"], "measured")
        self.assertEqual(summary["completed_interval_count"], 1)
        self.assertEqual(summary["resolved_interval_count"], 1)
        self.assertEqual(summary["effective_work_seconds_by_place"], {"4": 6.25})
        self.assertEqual(summary["max_effective_work_seconds_in_one_place"], 6.25)
        self.assertEqual(len(summary["censored_active_intervals"]), 1)
        self.assertEqual(
            summary["censored_active_intervals"][0]["elapsed_to_run_end_seconds"],
            4.0,
        )

    def test_new_lifecycle_events_are_not_collapsed_into_failures(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "route_id": 7,
                    "active_region_id": 4,
                    "work_item_id": 12,
                    "work_item_match": "inherited",
                    "portal_observation_probe": {
                        "opening_cell": [4, 4], "normal": [0, 1],
                    },
                },
            ),
            (
                "global_frontier_event",
                {"frontier_event": "frontier_endpoint_observed", "region_id": 4},
            ),
            (
                "global_frontier_event",
                {"frontier_event": "frontier_place_closed", "region_id": 4},
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_prefetch_deferred",
                    "reason": "cross_place_action_requires_terminal",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_region_dormant",
                    "region_id": 8,
                    "reason": "repeated_route_failure:stall",
                },
            ),
        ]

        summary = SUMMARY.frontier_region_summary(events)

        self.assertEqual(summary["route_region_sequence"], [4])
        self.assertEqual(summary["endpoint_observation_counts"], {4: 1})
        self.assertEqual(summary["normal_place_closure_counts"], {4: 1})
        self.assertEqual(summary["exceptional_region_retirement_counts"], {8: 1})
        self.assertEqual(summary["selected_work_item_counts"], {12: 1})
        self.assertEqual(len(summary["portal_observation_probe_routes"]), 1)
        self.assertEqual(
            summary["cross_place_prefetch_deferred_counts"],
            {"cross_place_action_requires_terminal": 1},
        )

    def test_failed_attempt_then_alternative_viewpoint_is_not_duplicate_work(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_dispatched",
                    "work_item_id": 12,
                    "attempt_id": 1,
                    "place_id": 4,
                    "ros_time": 10.0,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "viewpoint_attempt_settled",
                    "work_item_id": 12,
                    "attempt_id": 1,
                    "place_id": 4,
                    "work_item_state": "unresolved",
                    "attempt_state": "failed",
                    "ros_time": 12.0,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_dispatched",
                    "work_item_id": 12,
                    "attempt_id": 2,
                    "place_id": 4,
                    "ros_time": 13.0,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_settled",
                    "work_item_id": 12,
                    "attempt_id": 2,
                    "place_id": 4,
                    "work_item_state": "resolved",
                    "attempt_state": "succeeded",
                    "ros_time": 16.0,
                },
            ),
        ]
        work = SUMMARY.work_item_execution_summary(events, run_end_seconds=20.0)
        lifecycle = SUMMARY.frontier_region_summary(events)

        self.assertEqual(work["failed_attempt_count"], 1)
        self.assertEqual(work["succeeded_attempt_count"], 1)
        self.assertEqual(work["effective_work_seconds_by_place"], {"4": 3.0})
        self.assertEqual(lifecycle["viewpoint_attempt_counts"], {12: 2})
        self.assertEqual(lifecycle["repeated_work_item_dispatches"], {})

    def test_duplicate_active_attempt_is_reported_as_lifecycle_error(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_dispatched",
                    "work_item_id": 12,
                    "attempt_id": 1,
                    "place_id": 4,
                    "ros_time": 10.0,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "work_item_dispatched",
                    "work_item_id": 12,
                    "attempt_id": 1,
                    "place_id": 4,
                    "ros_time": 11.0,
                },
            ),
        ]

        result = SUMMARY.work_item_execution_summary(events, run_end_seconds=15.0)

        self.assertEqual(result["duplicate_dispatch_count"], 1)
        self.assertEqual(result["active_attempt_count"], 1)

    def test_route_unavailable_and_watchdog_stall_evidence_are_separate(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_route_unavailable",
                    "route_id": 7,
                    "ros_time": 10.0,
                    "reason": "snapshot_missing",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_route_unavailable",
                    "route_id": 7,
                    "ros_time": 18.5,
                    "reason": "snapshot_missing",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_stagnant_observed",
                    "route_id": 9,
                    "route_kind": "frontier_endpoint",
                    "active_elapsed": 6.0,
                    "ros_time": 20.0,
                    "reason": "stall",
                },
            ),
        ]

        result = SUMMARY.stall_evidence_summary(events)

        self.assertEqual(result["frontier_route_unavailable_event_count"], 2)
        self.assertEqual(result["max_frontier_route_unavailable_span_seconds"], 8.5)
        self.assertEqual(result["route_stagnant_observed_count"], 1)

    def test_failure_summary_includes_direct_snapshot_evidence(self):
        events = [
            (
                "failure_started",
                {
                    "failure_id": "run-F0001",
                    "trigger": "route_invalidated_stall",
                    "source": "global_frontier",
                    "classification": {
                        "label": "controller_stall",
                        "confidence": "high",
                    },
                    "started_wall_elapsed_seconds": 12.0,
                },
            ),
            (
                "failure_snapshot_ready",
                {
                    "failure_id": "run-F0001",
                    "classification": "controller_stall",
                    "confidence": "high",
                    "duration_seconds": 5.0,
                },
            ),
        ]
        snapshots = {
            "run-F0001": {
                "failure_id": "run-F0001",
                "resolution": "post_window_complete",
                "route_context_at_trigger": {
                    "route": {"route_id": 10, "reason": "stall"}
                },
                "trigger_sample": {
                    "pose": [1.0, 2.0],
                    "goal": [4.0, 5.0],
                    "teb_status": "trajectory_valid",
                },
                "end_sample": {"pose": [1.0, 2.0]},
                "diagnosis": {
                    "primary_cause": "planner_materialization_stall",
                    "layer": "global_frontier",
                    "route_id": 10,
                },
                "sample_count": 26,
            }
        }
        summary = SUMMARY.failure_episode_summary(events, snapshots)
        evidence = summary["episodes"][0]["evidence"]
        self.assertEqual(evidence["route_context_at_trigger"]["reason"], "stall")
        self.assertEqual(evidence["trigger_sample"]["pose"], [1.0, 2.0])
        self.assertEqual(evidence["trigger_sample"]["teb_status"], "trajectory_valid")
        self.assertEqual(
            summary["episodes"][0]["technical_diagnosis"]["primary_cause"],
            "planner_materialization_stall",
        )

    def test_legacy_normal_closure_remains_comparable(self):
        summary = SUMMARY.frontier_region_summary(
            [
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "frontier_region_dormant",
                        "region_id": 3,
                        "reason": "place_observation_complete",
                    },
                )
            ]
        )

        self.assertEqual(summary["normal_place_closure_counts"], {3: 1})
        self.assertEqual(summary["exceptional_region_retirement_counts"], {})

    def test_semantic_and_portal_hypothesis_events_are_reduced(self):
        summary = SUMMARY.frontier_region_summary(
            [
                (
                    "global_frontier_event",
                    {"frontier_event": "semantic_task_updated"},
                ),
                (
                    "global_frontier_event",
                    {"frontier_event": "semantic_place_evidence"},
                ),
                (
                    "global_frontier_event",
                    {"frontier_event": "semantic_topology_expansion_selected"},
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_hypothesis_certified",
                        "portal_id": 4,
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_hypothesis_selected",
                        "portal_id": 4,
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_probe_started",
                        "probe_id": 9,
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_probe_settled",
                        "probe_id": 9,
                        "result": "observed",
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "route_selected",
                        "graph_action": "probe_portal",
                        "graph_action_reason": "unknown_side_is_wall_bounded",
                        "graph_policy_rejections": 3,
                    },
                ),
            ]
        )
        self.assertEqual(summary["semantic_task_updates"], 1)
        self.assertEqual(summary["semantic_place_evidence_events"], 1)
        self.assertEqual(summary["semantic_topology_expansion_events"], 1)
        self.assertEqual(summary["portal_hypothesis_count"], 1)
        self.assertEqual(
            summary["portal_hypothesis_event_counts"],
            {
                "portal_hypothesis_certified": 1,
                "portal_hypothesis_selected": 1,
            },
        )
        self.assertEqual(summary["portal_probe_count"], 1)
        self.assertEqual(
            summary["portal_probe_event_counts"],
            {"portal_probe_settled": 1, "portal_probe_started": 1},
        )
        self.assertEqual(summary["portal_probe_result_counts"], {"observed": 1})
        self.assertEqual(summary["graph_action_counts"], {"probe_portal": 1})
        self.assertEqual(summary["graph_policy_rejections"], 3)

    def test_portal_probe_value_diagnostics_are_reduced(self):
        summary = SUMMARY.frontier_region_summary(
            [
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_probe_value_selected",
                        "action_category": "probe",
                        "candidate_count": 4,
                        "feasible_count": 3,
                        "pareto_front_count": 2,
                        "rejected": [
                            {
                                "candidate_id": 8,
                                "reasons": ["route_reachable"],
                            }
                        ],
                    },
                )
            ]
        )

        self.assertEqual(summary["portal_probe_value_selection_count"], 1)
        self.assertEqual(summary["portal_probe_value_action_categories"], {"probe": 1})
        self.assertEqual(summary["portal_probe_value_candidate_total"], 4)
        self.assertEqual(summary["portal_probe_value_feasible_total"], 3)
        self.assertEqual(summary["portal_probe_value_pareto_total"], 2)
        self.assertEqual(
            summary["portal_probe_value_rejection_reasons"],
            {"route_reachable": 1},
        )

    def test_graph_event_reducer_verifies_branch_commit_order(self):
        summary = SUMMARY.frontier_region_summary(
            [
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_transaction_started",
                        "transaction_id": 1,
                        "route_id": 7,
                        "portal_id": 3,
                        "source_place_id": 2,
                        "state": "source_probe",
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "route_selected",
                        "route_id": 7,
                        "graph_action": "cross_portal",
                        "graph_action_reason": "certified_portal_branch_to_unobserved_place",
                        "portal_transaction": {
                            "route_id": 7,
                            "portal_id": 3,
                            "state": "source_probe",
                        },
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_transaction_phase",
                        "transaction_id": 1,
                        "route_id": 7,
                        "state": "place_commit",
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_place_entered",
                        "route_id": 7,
                        "region_id": 4,
                        "portal_transaction": {
                            "route_id": 7,
                            "state": "place_commit",
                        },
                    },
                ),
            ]
        )
        invariants = summary["graph_invariants"]
        self.assertEqual(invariants["branch_first_crossings"], 1)
        self.assertEqual(invariants["phantom_place_count"], 0)
        self.assertEqual(invariants["illegal_graph_action_count"], 0)

    def test_graph_event_reducer_flags_reselection_of_failed_portal(self):
        summary = SUMMARY.frontier_region_summary(
            [
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_hypothesis_failed",
                        "portal_id": 8,
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_transaction_started",
                        "transaction_id": 3,
                        "route_id": 12,
                        "portal_id": 8,
                        "source_place_id": 4,
                        "state": "source_probe",
                    },
                ),
            ]
        )
        invariants = summary["graph_invariants"]
        self.assertEqual(invariants["same_edge_reselection_count"], 1)

    def test_graph_event_reducer_accepts_finish_before_delayed_arrival(self):
        """Map callbacks may publish arrival after the transaction is idle."""
        summary = SUMMARY.frontier_region_summary(
            [
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_transaction_started",
                        "transaction_id": 9,
                        "route_id": 17,
                        "portal_id": 4,
                        "source_place_id": 2,
                        "state": "source_probe",
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_transaction_phase",
                        "transaction_id": 9,
                        "route_id": 17,
                        "state": "place_commit",
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_transaction_finished",
                        "transaction_id": 9,
                        "route_id": 17,
                        "portal_transaction": {
                            "transaction_id": 9,
                            "route_id": 0,
                            "state": "idle",
                        },
                    },
                ),
                (
                    "global_frontier_event",
                    {
                        "frontier_event": "portal_place_entered",
                        "transaction_id": 9,
                        "route_id": 17,
                        "region_id": 5,
                        "portal_transaction": {
                            "transaction_id": 9,
                            "route_id": 0,
                            "state": "idle",
                        },
                    },
                ),
            ]
        )
        invariants = summary["graph_invariants"]
        self.assertEqual(invariants["phantom_place_count"], 0)
        self.assertEqual(invariants["place_entry_count"], 1)

    def test_route_goal_summary_correlates_recovery_and_route_churn(self):
        events = [
            (
                "mission_goal_transaction",
                {
                    "ros_time": 1.0,
                    "route_id": 1,
                    "route_kind": "frontier_endpoint",
                    "goal": [1.0, 2.0],
                    "source": "global_slam_frontier",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "ros_time": 1.0,
                    "route_id": 1,
                    "route_kind": "frontier_endpoint",
                    "goal": [1.0, 2.0],
                    "graph_action": "observe_local_work",
                    "work_item_id": 7,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_route_unavailable",
                    "ros_time": 2.0,
                    "route_id": 1,
                    "reason": "navfn_validation_pending",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_endpoint_observed",
                    "ros_time": 3.0,
                    "route_id": 1,
                },
            ),
            (
                "mission_goal_transaction",
                {
                    "ros_time": 3.2,
                    "route_id": 2,
                    "route_kind": "portal_transition",
                    "goal": [8.0, 2.0],
                    "source": "global_slam_frontier",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "ros_time": 3.2,
                    "route_id": 2,
                    "route_kind": "portal_transition",
                    "goal": [8.0, 2.0],
                    "graph_action": "cross_portal",
                    "portal_id": 4,
                },
            ),
        ]

        summary = SUMMARY.route_goal_lifecycle_summary(events)

        self.assertEqual(summary["accepted_route_count"], 2)
        self.assertEqual(summary["mission_goal_transaction_count"], 2)
        self.assertEqual(summary["frontier_route_unavailable_total"], 1)
        self.assertEqual(summary["routes"][0]["end_event"], "frontier_endpoint_observed")
        self.assertEqual(summary["routes"][1]["graph_action"], "cross_portal")
        self.assertAlmostEqual(summary["mission_goal_delta_max_m"], 7.0)

    def test_transition_topology_cache_is_counted_per_route_lease(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "transition_topology_cached",
                    "route_id": 3,
                    "endpoint": [2.0, 4.0],
                    "grid_shape": [100, 80],
                    "map_resolution": 0.1,
                    "map_origin": [0.0, 0.0],
                    "structural_revision": 3,
                    "ros_time": 5.0,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "transition_topology_cached",
                    "route_id": 3,
                    "endpoint": [2.0, 4.0],
                    "grid_shape": [100, 80],
                    "map_resolution": 0.1,
                    "map_origin": [0.0, 0.0],
                    "structural_revision": 4,
                    "ros_time": 6.0,
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "transition_topology_cached",
                    "route_id": 4,
                    "endpoint": [4.0, 4.0],
                    "grid_shape": [100, 80],
                    "map_resolution": 0.1,
                    "map_origin": [0.0, 0.0],
                    "structural_revision": 5,
                    "ros_time": 7.0,
                },
            ),
        ]

        summary = SUMMARY.transition_topology_cache_summary(events)

        self.assertEqual(summary["build_count"], 3)
        self.assertEqual(summary["unique_route_count"], 2)
        self.assertEqual(summary["rebuild_count"], 1)
        self.assertEqual(summary["builds_by_route"], {3: 2, 4: 1})

    def test_termination_diagnosis_does_not_call_active_timeout_a_failure(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "ros_time": 10.0,
                    "route_id": 4,
                    "route_kind": "portal_transition",
                },
            ),
        ]
        result = SUMMARY.termination_diagnosis(
            events,
            {
                "ros_time": 20.0,
                "pose": [2.0, 3.0],
                "goal": [8.0, 3.0],
                "distance_to_goal": 6.0,
                "teb_bridge_active": True,
                "cmd": [0.4, 0.0],
                "teb_status": "trajectory_valid",
                "navfn_plan": {"poses": 12},
                "move_base_feedback": {"status_name": "ACTIVE"},
                "task_done": False,
            },
        )

        self.assertEqual(result["state"], "in_progress")
        self.assertEqual(result["reason"], "active_route_not_finished_at_trial_end")
        self.assertEqual(result["active_route_id"], 4)
        self.assertEqual(result["terminal_failure_event_count"], 0)

    def test_termination_diagnosis_recovers_route_kind_from_route_history(self):
        events = [
            (
                "global_frontier_event",
                {
                    "frontier_event": "route_selected",
                    "ros_time": 1.0,
                    "route_id": 9,
                    "route_kind": "portal_transition",
                },
            ),
            (
                "global_frontier_event",
                {
                    "frontier_event": "frontier_action_selected",
                    "ros_time": 2.0,
                    "route_id": 9,
                },
            ),
        ]

        result = SUMMARY.termination_diagnosis(
            events,
            {
                "ros_time": 2.5,
                "teb_bridge_active": True,
                "cmd": [0.0, 0.0],
                "teb_status": "trajectory_valid",
                "navfn_plan": {"poses": 8},
                "move_base_feedback": {"status_name": "ACTIVE"},
            },
        )

        self.assertEqual(result["active_route_kind"], "portal_transition")

    def test_termination_diagnosis_does_not_call_goal_settle_a_stall(self):
        result = SUMMARY.termination_diagnosis(
            [],
            {
                "ros_time": 5.0,
                "distance_to_goal": 0.30,
                "teb_xy_goal_tolerance": 0.50,
                "teb_bridge_active": True,
                "cmd": [0.0, 0.0],
                "teb_status": "trajectory_valid",
                "navfn_plan": {"poses": 8},
                "move_base_feedback": {"status_name": "ACTIVE"},
            },
        )

        self.assertEqual(result["state"], "terminal_settle")
        self.assertEqual(
            result["reason"], "within_goal_tolerance_waiting_for_terminal"
        )
        self.assertTrue(result["within_goal_tolerance"])


if __name__ == "__main__":
    unittest.main()
