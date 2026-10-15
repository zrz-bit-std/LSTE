#!/usr/bin/env python3
"""Regression tests for correlated navigation failure evidence."""

import sys
import tempfile
import threading
import time
import json
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from navigation_metrics_failure_evidence import (  # noqa: E402
    NavigationMetricsFailureEvidenceMixin,
    classify_failure,
    diagnose_failure_sample,
    merge_failure_classifications,
)
from navigation_metrics_execution_events import (  # noqa: E402
    NavigationMetricsExecutionEventsMixin,
    route_invalidation_failure_trigger,
    route_recovery_preemption_reason,
)
from navigation_metrics_action_lifecycle import (  # noqa: E402
    NavigationMetricsActionLifecycleMixin,
)
from navigation_metrics_goal_events import (  # noqa: E402
    NavigationMetricsGoalEventsMixin,
)


def sample(**overrides):
    value = {
        "pose": [1.0, 2.0, 0.0],
        "distance_to_goal": 3.0,
        "scan": {"min": 2.0, "forward_min": 2.0},
        "obstacle_clearance_threshold": 0.5,
        "navfn_plan": {"poses": 20, "length": 3.0, "endpoint": [4.0, 2.0]},
        "teb_feedback": {"selected_velocity": {"linear_x": 0.2, "angular_z": 0.0}},
        "teb_status": "trajectory_valid",
        "move_base_status": "ACTIVE",
        "recovery": {"current": 0, "total": 5, "behavior": "-"},
        "cmd_vel": [0.2, 0.0],
        "bridge_active": True,
        "distance_transform_failures": 0,
    }
    value.update(overrides)
    return value


class TargetTerminalPreemptionHarness(
    NavigationMetricsGoalEventsMixin,
    NavigationMetricsExecutionEventsMixin,
    NavigationMetricsActionLifecycleMixin,
):
    """Minimal observer fixture for the explicit terminal handoff contract."""

    def __init__(self):
        self.lock = threading.RLock()
        self.lifecycle_event_wall = {}
        self.status_seen = set()
        self.move_base_goal_ids = set()
        self.status_counts = {}
        self.last_move_base_status = "UNKNOWN"
        self.preemptions = 0
        self.frontier_observation_preemptions = 0
        self.frontier_terminal_settle_preemptions = 0
        self.frontier_continuous_prefetch_preemptions = 0
        self.frontier_segment_preemptions = 0
        self.target_segment_preemptions = 0
        self.target_terminal_observation_preemptions = 0
        self.priority_preemptions = 0
        self.task_done_preemptions = 0
        self.unexpected_preemptions = 0
        self.pending_frontier_observation_preemptions = 0
        self.pending_frontier_terminal_settle_preemptions = 0
        self.pending_frontier_continuous_prefetch_preemptions = 0
        self.pending_frontier_segment_preemptions = 0
        self.pending_target_segment_preemptions = 0
        self.pending_target_terminal_observation_preemptions = 0
        self.pending_priority_preemptions = 0
        self.pending_task_done_preemptions = 0
        self.pending_route_recovery_preemptions = 0
        self.route_recovery_preemption_reasons = {}
        self.aborts = 0
        self.successes = 0
        self.bridge_events = 0
        self.bridge_last_event = ""
        self.bridge_active = False
        self.bridge_active_intent_source = "unknown"
        self.bridge_latest_intent_source = "unknown"
        self.bridge_deferred_goal_updates = 0
        self.bridge_dispatches = 0
        self.persistent_execution = True
        self.execution_architecture = "persistent_stream"
        self.goal_source = "unknown"
        self.goal_transaction_id = 0
        self.goal_transition_kind = "unknown"
        self.goal_predecessor_route_id = 0
        self.goal_transition_distance = None
        self.mission_goal_messages = 0
        self.failure_calls = 0
        self.writes = []

    @staticmethod
    def _as_bool(value):
        return bool(value)

    def _failure_record_context_locked(self, *_args, **_kwargs):
        return None

    def _begin_failure_episode_locked(self, *_args, **_kwargs):
        self.failure_calls += 1

    def _write(self, level, event, **fields):
        self.writes.append((level, event, fields))


class TargetTerminalPreemptionTest(unittest.TestCase):
    def test_expected_terminal_observation_cancel_is_not_a_failure(self):
        metrics = TargetTerminalPreemptionHarness()
        metrics.on_bridge_status(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "cancel",
                        "reason": "target_terminal_observation",
                        "was_active": True,
                        "persistent_execution": True,
                    }
                )
            )
        )

        self.assertEqual(
            metrics.pending_target_terminal_observation_preemptions,
            1,
        )
        metrics.on_status(
            SimpleNamespace(
                status_list=[
                    SimpleNamespace(
                        goal_id=SimpleNamespace(id="target-goal-1"),
                        status=2,
                        text="-",
                    )
                ]
            )
        )

        self.assertEqual(metrics.target_terminal_observation_preemptions, 1)
        self.assertEqual(metrics.pending_target_terminal_observation_preemptions, 0)
        self.assertEqual(metrics.unexpected_preemptions, 0)
        self.assertEqual(metrics.failure_calls, 0)
        status = next(
            fields for _level, event, fields in metrics.writes
            if event == "move_base_status"
        )
        self.assertEqual(status["target_terminal_observation_preemptions"], 1)

    def test_terminal_observation_intent_arms_before_cancel_callback(self):
        metrics = TargetTerminalPreemptionHarness()
        metrics.on_mission_goal(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "target_terminal_observation",
                        "transaction_id": 7,
                        "target_epoch": 12,
                        "target_track_id": "yellow_cup:yellow cup:1",
                        "reason": "target_terminal_reobserve_expired",
                    }
                )
            )
        )
        metrics.on_status(
            SimpleNamespace(
                status_list=[
                    SimpleNamespace(
                        goal_id=SimpleNamespace(id="target-goal-2"),
                        status=2,
                        text="-",
                    )
                ]
            )
        )

        self.assertEqual(metrics.target_terminal_observation_preemptions, 1)
        self.assertEqual(metrics.unexpected_preemptions, 0)
        self.assertEqual(metrics.failure_calls, 0)


class FailureClassificationTest(unittest.TestCase):
    def test_normal_standoff_release_is_not_a_failure(self):
        self.assertIsNone(
            route_invalidation_failure_trigger(
                "route_invalidated",
                {"reason": "frontier_observed_at_standoff"},
            )
        )

    def test_stall_release_starts_a_failure_before_recovery_preemption(self):
        self.assertEqual(
            route_invalidation_failure_trigger(
                "route_invalidated", {"reason": "stall"}
            ),
            "route_invalidated_stall",
        )
        result = classify_failure(
            "route_invalidated_stall",
            sample(cmd_vel=[0.0, 0.0]),
            {"reason": "stall", "route_id": 10},
        )
        self.assertEqual(result["label"], "controller_stall")

    def test_forward_only_teb_output_gap_is_classified_explicitly(self):
        result = classify_failure(
            "controller_output_gap",
            sample(
                distance_to_goal=0.57,
                cmd_vel=[0.0, 0.19],
                teb_planner_cmd=[-0.01, 0.14],
                teb_feedback={
                    "selected_velocity": {"linear_x": -0.003, "angular_z": 0.30}
                },
                cmd_vel_mux={
                    "filter_reason": "forward_only_reverse_clamp",
                    "output": {"linear_x": 0.0, "angular_z": 0.19},
                },
            ),
            {"goal_tolerance": 0.50},
        )
        self.assertEqual(result["label"], "controller_output_gap")
        self.assertEqual(result["confidence"], "high")

    def test_frontier_route_failure_keeps_its_explicit_stall_cause(self):
        result = classify_failure(
            "frontier_route_failure",
            sample(cmd_vel=[0.1, 0.2]),
            {"reason": "local_egress_stall", "route_id": 6},
        )
        self.assertEqual(result["label"], "controller_stall")
        diagnosis = diagnose_failure_sample(
            sample(
                route_context={
                    "route": {
                        "route_id": 6,
                        "route_kind": "local_egress",
                    }
                }
            ),
            {
                "trigger": "frontier_route_failure",
                "reason": "local_egress_stall",
                "route_id": 6,
            },
        )
        self.assertEqual(diagnosis["primary_cause"], "controller_stall")
        self.assertEqual(diagnosis["layer"], "global_frontier")
        self.assertEqual(diagnosis["route_id"], 6)

    def test_target_plan_failure_stays_at_the_navfn_boundary(self):
        details = {
            "event": "target_route_failed",
            "trigger": "target_route_failed",
            "goal": [100.0, 100.0],
            "goal_frame": "map",
            "transaction_id": 900,
            "status": "NAVFN_NO_PATH",
            "reason": "persistent_navfn_target_unreachable",
            "route_id": 1,
            "route_kind": "target_approach",
        }
        result = classify_failure(
            "target_route_failed",
            sample(cmd_vel=[0.5, 0.0]),
            details,
            recent_events=[
                {"source": "move_base", "event": "preempted", "payload": {}}
            ],
        )
        self.assertEqual(result["label"], "planner_no_path")
        diagnosis = diagnose_failure_sample(sample(cmd_vel=[0.5, 0.0]), details)
        self.assertEqual(diagnosis["primary_cause"], "planner_no_path")
        self.assertEqual(diagnosis["layer"], "streaming_navfn")

    def test_route_context_does_not_lose_route_after_lease_release(self):
        class ContextHarness(NavigationMetricsFailureEvidenceMixin):
            pass

        harness = ContextHarness()
        harness.last_bridge_context = {
            "payload": {"route_id": 10, "route_kind": "frontier_endpoint"}
        }
        harness.last_frontier_context = {
            "payload": {"route_id": 10, "route_kind": "frontier_endpoint"}
        }
        harness.last_goal_context = {
            "payload": {"route_id": 0, "route_kind": "", "controller_lease": "released"}
        }
        harness.last_action_context = {
            "payload": {"route_id": 0, "route_kind": ""}
        }

        route = harness._failure_route_context_locked()["route"]

        self.assertEqual(route["route_id"], 10)
        self.assertEqual(route["route_kind"], "frontier_endpoint")

    def test_nested_producer_context_keeps_causal_route_identity(self):
        class ContextHarness(NavigationMetricsFailureEvidenceMixin):
            pass

        harness = ContextHarness()
        harness.failure_last_positive_route = None
        harness.failure_event_history = []
        sample_value = {
            "goal": [3.5, 13.5],
            "route_context": {
                "contexts": {
                    "bridge": {
                        "payload": {
                            "active_route_id": 5,
                            "active_route_kind": "portal_transition",
                            "goal": [3.5, 13.5],
                        }
                    },
                    "frontier": {
                        "payload": {
                            "route_id": 5,
                            "obligation_id": 67,
                            "obligation_kind": "work_item",
                        }
                    },
                }
            },
        }

        route = harness._failure_causal_route_locked(sample_value, {})

        self.assertEqual(route["route_id"], 5)
        self.assertEqual(route["route_kind"], "portal_transition")
        self.assertEqual(route["obligation_id"], 67)

    def test_route_context_does_not_double_truncate_bounded_payload(self):
        """Failure artifacts keep producer details beyond the route summary."""
        class ContextHarness(NavigationMetricsFailureEvidenceMixin):
            pass

        harness = ContextHarness()
        harness.last_bridge_context = {
            "event_sequence": 4,
            "source": "bridge",
            "event": "dispatch",
            "wall_elapsed_seconds": 2.0,
            "ros_time": 8.0,
            "payload": {
                "active_route_id": 12,
                "active_route_kind": "portal_transition",
                "graph_route_plan": {
                    "action": "cross_portal",
                    "portal_path": [3, 7],
                    "obligation_kind": "portal_edge",
                },
            },
        }

        context = harness._failure_route_context_locked()
        bridge = context["contexts"]["bridge"]

        self.assertEqual(bridge["payload"]["active_route_id"], 12)
        self.assertEqual(
            bridge["payload"]["graph_route_plan"]["action"],
            "cross_portal",
        )
        self.assertNotEqual(bridge["payload"], "<max-depth>")

    def test_disconnected_release_is_planner_evidence(self):
        self.assertEqual(
            route_invalidation_failure_trigger(
                "route_invalidated", {"reason": "disconnected"}
            ),
            "route_invalidated_disconnected",
        )
        self.assertEqual(
            route_invalidation_failure_trigger(
                "route_invalidated", {"reason": "local_egress_stall"}
            ),
            "route_invalidated_stall",
        )

    def test_only_explicit_frontier_cancellation_is_route_recovery(self):
        self.assertEqual(
            route_recovery_preemption_reason(
                "controller_lease_released",
                {"result_status": 2, "reason": "durable_identity_not_in_current_frontier_snapshot"},
                2,
            ),
            "durable_identity_not_in_current_frontier_snapshot",
        )
        self.assertEqual(
            route_recovery_preemption_reason(
                "cancel",
                {"result_status": 2, "reason": "frontier_route_invalidated_frontier_observed_at_standoff"},
                2,
            ),
            "frontier_route_invalidated_frontier_observed_at_standoff",
        )
        self.assertIsNone(
            route_recovery_preemption_reason(
                "cancel", {"status": 2, "reason": "target_failure"}, 2
            )
        )

    def test_empty_navfn_route_is_classified_without_manual_log_search(self):
        result = classify_failure(
            "move_base_terminal_failure",
            sample(
                navfn_plan={"poses": 0, "length": 0.0, "endpoint": None},
                teb_feedback=None,
                teb_status="no_selected_trajectory",
                cmd_vel=[0.0, 0.0],
            ),
            {"status_name": "ABORTED"},
        )
        self.assertEqual(result["label"], "planner_no_path")
        self.assertTrue(any(item["label"] == "controller_stall" for item in result["candidates"]))

    def test_zero_velocity_after_empty_navfn_plan_is_planner_boundary(self):
        """Do not call a never-materialized route a controller stall."""
        result = classify_failure(
            "zero_velocity_stall",
            sample(
                navfn_plan={"poses": 0, "length": 0.0, "endpoint": None},
                teb_feedback=None,
                teb_status="not_available",
                teb_planner_cmd=[0.0, 0.0],
                cmd_vel=[0.0, 0.0],
            ),
            {"duration_seconds": 3.2},
        )
        self.assertEqual(result["label"], "planner_no_path")
        self.assertEqual(result["confidence"], "high")
        self.assertTrue(
            any(
                item["label"] == "controller_stall"
                and item["confidence"] == "low"
                for item in result["candidates"]
            )
        )

    def test_explicit_target_navfn_failure_beats_release_race_tokens(self):
        result = classify_failure(
            "target_route_failed",
            sample(
                # The trigger sample can still contain the previous valid
                # route because the target is rejected before the next metrics
                # tick. The explicit target/Navfn reason is authoritative.
                route_context={
                    "route": {
                        "route_id": 1,
                        "route_kind": "target_approach",
                        "controller_lease": "released",
                    }
                }
            ),
            {
                "reason": "persistent_navfn_target_unreachable",
                "route_id": 1,
                "target_track_id": "probe:unreachable-wall",
            },
            recent_events=[
                {
                    "source": "bridge",
                    "event": "target_controller_lease_released",
                    "payload": {"reason": "target_plan_failed"},
                }
            ],
        )
        self.assertEqual(result["label"], "planner_no_path")
        self.assertEqual(result["confidence"], "high")

    def test_near_scan_is_preserved_as_a_local_obstacle_candidate(self):
        result = classify_failure(
            "move_base_terminal_failure",
            sample(scan={"min": 0.2, "forward_min": 0.2}),
            {"status_name": "ABORTED", "reason": "blocked"},
        )
        self.assertEqual(result["label"], "local_obstacle")
        self.assertEqual(result["confidence"], "high")

    def test_recent_handoff_is_explicitly_correlated_as_owner_race(self):
        result = classify_failure(
            "unexpected_preemption",
            sample(cmd_vel=[0.0, 0.0]),
            {"replacement_kind": "priority_intent"},
            recent_events=[
                {"source": "bridge", "event": "handoff_requested", "payload": {}}
            ],
        )
        self.assertEqual(result["label"], "owner_transaction_race")

    def test_endpoint_wait_dominates_accompanying_preemption_token(self):
        result = classify_failure(
            "unexpected_preemption",
            sample(
                route_context={
                    "route": {
                        "route_id": 17,
                        "released_controller_route": {
                            "controller_pending": True,
                            "terminal_received": True,
                        },
                    }
                }
            ),
            {"status_name": "PREEMPTED"},
            recent_events=(
                {
                    "event": "controller_lease_released",
                    "payload": {"reason": "endpoint_terminal"},
                },
            ),
        )
        self.assertEqual(result["label"], "planner_materialization_stall")
        self.assertEqual(result["evidence"]["route"]["route_id"], 17)

    def test_stale_handoff_does_not_override_current_planner_evidence(self):
        current = sample(
            navfn_plan={"poses": 0, "length": 0.0, "endpoint": None},
            teb_feedback=None,
            teb_status="no_selected_trajectory",
            cmd_vel=[0.0, 0.0],
            wall_elapsed_seconds=20.0,
        )
        result = classify_failure(
            "move_base_terminal_failure",
            current,
            {"status_name": "ABORTED"},
            recent_events=[
                {
                    "source": "bridge",
                    "event": "handoff_requested",
                    "wall_elapsed_seconds": 10.0,
                    "payload": {},
                }
            ],
        )
        self.assertEqual(result["label"], "planner_no_path")

    def test_endpoint_terminal_wait_is_materialization_not_controller_stall(self):
        result = classify_failure(
            "frontier_route_stall",
            sample(
                cmd_vel=[0.0, 0.0],
                teb_cmd=[0.0, 0.0],
                teb_planner_cmd=[0.0, 0.0],
                route_context={
                    "route": {
                        "route_id": 10,
                        "route_kind": "frontier_endpoint",
                        "released_controller_route": {
                            "route_id": 10,
                            "controller_pending": True,
                            "terminal_received": True,
                        },
                    }
                },
            ),
            {"route_id": 10, "unavailable_count": 4},
        )
        self.assertEqual(result["label"], "planner_materialization_stall")
        self.assertFalse(
            any(
                item["label"] == "controller_stall"
                and item["confidence"] == "high"
                for item in result["candidates"]
            )
        )

    def test_diagnosis_exposes_command_boundary_and_route_terminal(self):
        result = diagnose_failure_sample(
            sample(
                cmd_vel=[0.0, 0.0],
                teb_cmd=[0.0, 0.0],
                teb_planner_cmd=[0.0, 0.0],
                route_context={
                    "route": {
                        "route_id": 10,
                        "released_controller_route": {
                            "route_id": 10,
                            "route_kind": "frontier_endpoint",
                            "controller_pending": True,
                            "terminal_received": True,
                        },
                        "action": "observe_local_work",
                        "obligation_kind": "work_item",
                        "obligation_id": 76,
                    }
                },
                teb_feedback_age_seconds=4.1,
            )
        )
        self.assertEqual(result["primary_cause"], "planner_materialization_stall")
        self.assertEqual(result["layer"], "global_frontier")
        self.assertEqual(result["route_kind"], "frontier_endpoint")
        self.assertEqual(result["graph_action"], "observe_local_work")
        self.assertEqual(result["obligation"], {
            "kind": "work_item",
            "id": 76,
            "portal_path": None,
        })
        self.assertTrue(result["terminal_wait"])
        self.assertEqual(result["command_chain"]["teb_selected"], [0.2, 0.0])
        self.assertEqual(result["timing"]["teb_feedback_age_seconds"], 4.1)

    def test_diagnosis_preserves_atomic_turn_phase(self):
        result = diagnose_failure_sample(
            sample(
                teb_turn_supervisor={
                    "state": "TURNING",
                    "active_route_kind": "frontier_endpoint",
                    "turn_phase": "pre_route_alignment",
                    "yaw_error": 1.2,
                    "target_yaw": 1.57,
                }
            )
        )
        self.assertEqual(
            result["execution_phase"],
            {
                "turn_supervisor_state": "TURNING",
                "turn_supervisor_route_kind": "frontier_endpoint",
                "turn_phase": "pre_route_alignment",
                "yaw_error": 1.2,
                "target_yaw": 1.57,
                "mux_filter_reason": "",
                "mux_block_reason": "",
            },
        )

    def test_diagnosis_normalizes_frontier_active_route_identity(self):
        """A frontier invalidation must not be reported as route=None."""
        result = diagnose_failure_sample(
            sample(
                route_context={
                    "route": {
                        "active_route_id": 23,
                        "active_route_kind": "frontier_endpoint",
                    }
                }
            )
        )
        self.assertEqual(result["route_id"], 23)
        self.assertEqual(result["route_kind"], "frontier_endpoint")

    def test_diagnosis_uses_failure_details_after_active_lease_is_cleared(self):
        """The invalidation event remains the source of the causal route ID."""
        result = diagnose_failure_sample(
            sample(
                route_context={
                    "route": {
                        "active_route_id": 0,
                        "active_route_kind": "",
                    }
                }
            ),
            {
                "route_id": 17,
                "route_kind": "frontier_endpoint",
                "graph_route_plan": {
                    "action": "observe_local_work",
                    "obligation_id": 42,
                    "obligation_kind": "work_item",
                    "portal_path": [],
                },
            },
        )
        self.assertEqual(result["route_id"], 17)
        self.assertEqual(result["graph_action"], "observe_local_work")
        self.assertEqual(result["obligation"]["id"], 42)

    def test_failure_episode_keeps_causal_route_after_lease_clear(self):
        """The episode stores the trigger route independently of later samples."""
        class MetricsHarness(
            NavigationMetricsExecutionEventsMixin,
            NavigationMetricsFailureEvidenceMixin,
        ):
            pass

        metrics = MetricsHarness()
        metrics.start_wall = time.monotonic()
        metrics.run_timestamp = "20260908_120001"
        metrics.process_name = "lste_navigation_metrics"
        metrics.lock = threading.RLock()
        metrics.lifecycle_event_wall = {}
        metrics._write = lambda *_args, **_kwargs: None
        metrics._write_failure_log = lambda *_args, **_kwargs: None
        metrics._initialize_failure_evidence_state()
        metrics.pose = (0.0, 0.0, 0.0)
        metrics.goal = (2.0, 0.0)
        metrics.command = [0.0, 0.0]
        metrics.teb_command = [0.0, 0.0]
        metrics.teb_planner_command = [0.0, 0.0]
        metrics.scan_minimum = 2.0
        metrics.scan_forward_minimum = 2.0
        metrics.discontinuity_obstacle_clearance = 0.5
        metrics.bridge_active = True
        metrics.navfn_plan_stats = {"poses": 10, "length": 2.0, "endpoint": [2.0, 0.0]}
        metrics.teb_feedback_state = {"selected_velocity": {"linear_x": 0.2, "angular_z": 0.0}}
        metrics.teb_status = "trajectory_valid"
        metrics.last_frontier_context = {
            "payload": {
                "route_id": 17,
                "route_kind": "frontier_endpoint",
                "reason": "stall",
            }
        }
        failure_id = metrics._begin_failure_episode_locked(
            "route_invalidated_stall",
            "global_frontier",
            {"reason": "stall", "route_id": 17, "route_kind": "frontier_endpoint"},
        )
        self.assertEqual(failure_id, "20260908_120001-F0001")
        metrics.last_frontier_context = {
            "payload": {"route_id": 0, "route_kind": ""}
        }
        summary = metrics._finish_failure_episode_locked("test", force=True)
        self.assertEqual(summary["route_id"], 17)
        self.assertEqual(summary["route_kind"], "frontier_endpoint")

    def test_episode_end_does_not_erase_stronger_trigger_evidence(self):
        initial = classify_failure(
            "unexpected_preemption",
            sample(cmd_vel=[0.0, 0.0], wall_elapsed_seconds=10.0),
            {"status_name": "PREEMPTED"},
            recent_events=[
                {
                    "source": "bridge",
                    "event": "handoff_requested",
                    "wall_elapsed_seconds": 10.0,
                    "payload": {},
                }
            ],
        )
        end = classify_failure(
            "unexpected_preemption",
            sample(cmd_vel=[0.2, 0.0], wall_elapsed_seconds=15.0),
            {"status_name": "PREEMPTED"},
            recent_events=[],
        )
        merged = merge_failure_classifications(initial, end)
        self.assertEqual(initial["label"], "owner_transaction_race")
        self.assertEqual(end["label"], "unknown")
        self.assertEqual(merged["label"], "owner_transaction_race")
        self.assertEqual(merged["confidence"], "high")
        self.assertEqual(
            merged["candidates"][0]["sources"], ["trigger"],
        )


class FailureEpisodeTest(unittest.TestCase):
    def setUp(self):
        class MetricsHarness(
            NavigationMetricsExecutionEventsMixin,
            NavigationMetricsFailureEvidenceMixin,
        ):
            pass

        self.metrics = MetricsHarness()
        self.metrics.start_wall = time.monotonic()
        self.metrics.run_timestamp = "20260908_120000"
        self.metrics.process_name = "lste_navigation_metrics"
        self.metrics.lock = threading.RLock()
        self.metrics.lifecycle_event_wall = {}
        self.metrics._write = lambda *_args, **_kwargs: None
        self.failure_log_events = []
        self.metrics._write_failure_log = (
            lambda level, event, **fields: self.failure_log_events.append(
                (level, event, fields)
            )
        )
        self.metrics._initialize_failure_evidence_state()
        self.metrics.pose = (0.0, 0.0, 0.0)
        self.metrics.goal = (3.0, 0.0)
        self.metrics.command = [0.0, 0.0]
        self.metrics.teb_command = [0.0, 0.0]
        self.metrics.teb_planner_command = [0.0, 0.0]
        self.metrics.scan_minimum = 2.0
        self.metrics.scan_forward_minimum = 2.0
        self.metrics.discontinuity_obstacle_clearance = 0.5
        self.metrics.bridge_active = True
        self.metrics.navfn_plan_stats = {
            "poses": 12, "length": 3.0, "endpoint": [3.0, 0.0]
        }
        self.metrics.teb_feedback_state = {
            "selected_velocity": {"linear_x": 0.2, "angular_z": 0.0}
        }
        self.metrics.teb_status = "trajectory_valid"
        self.metrics.run_context = {
            "experiment": {
                "world": "office_level_2.world",
                "initial_pose": {"x": 2.8, "y": 11.0, "yaw": 0.0},
            },
            "task_definition": {"task_id": "topology_only"},
        }

    def test_episode_gets_one_id_and_contains_a_pre_failure_sample(self):
        self.metrics._failure_append_sample_locked(time.monotonic())
        failure_id = self.metrics._begin_failure_episode_locked(
            "move_base_terminal_failure",
            "move_base",
            {"status_name": "ABORTED"},
        )
        self.assertEqual(failure_id, "20260908_120000-F0001")
        self.assertIsNotNone(self.metrics.active_failure)
        summary = self.metrics._finish_failure_episode_locked("test", force=True)
        self.assertEqual(summary["failure_id"], failure_id)
        self.assertEqual(summary["classification"], "controller_stall")
        self.assertIsNone(self.metrics.active_failure)
        self.assertEqual(len(self.metrics.failure_history), 1)
        snapshot = next(
            fields for _level, event, fields in self.failure_log_events
            if event == "failure_snapshot"
        )
        self.assertEqual(snapshot["trigger_sample"]["pose"], [0.0, 0.0])
        self.assertIn("end_sample", snapshot)
        self.assertEqual(
            snapshot["diagnosis_at_trigger"]["primary_cause"],
            "stale_teb_feedback",
        )
        self.assertIn("diagnosis_at_end", snapshot)
        self.assertEqual(
            snapshot["run_context"]["experiment"]["world"],
            "office_level_2.world",
        )

    def test_route_invalidation_recovers_preceding_route_command_identity(self):
        """A lease-clear payload must not make a failure anonymous."""
        metrics = self.metrics
        metrics._failure_record_context_locked(
            "frontier",
            "route_command",
            {
                "route_id": 9,
                "route_kind": "frontier_endpoint",
                "command_goal": [4.45, 0.15],
                "graph_action": "observe_local_work",
                "work_item_id": 7,
            },
        )
        # The real invalidation status clears the active lease and leaves only
        # a boolean ``active`` marker. The predecessor is the only reliable
        # route identity at this callback boundary.
        metrics._failure_record_context_locked(
            "frontier",
            "route_invalidated",
            {"active": True, "reason": "stall", "route_id": 0},
        )
        route = metrics._failure_route_context_locked()["route"]
        self.assertEqual(route["route_id"], 9)
        self.assertEqual(route["route_kind"], "frontier_endpoint")
        self.assertEqual(route["graph_action"], "observe_local_work")

        trigger = sample(
            goal=[4.45, 0.15],
            route_context={
                "route": {
                    "active_route_id": 0,
                    "active_route_kind": "",
                }
            },
            cmd_vel=[0.0, 0.0],
        )
        metrics._begin_failure_episode_locked(
            "route_invalidated_stall",
            "global_frontier",
            {"active": True, "reason": "stall", "route_id": 0},
            trigger,
        )
        summary = metrics._finish_failure_episode_locked("test", force=True)
        self.assertEqual(summary["route_id"], 9)
        self.assertEqual(summary["route_kind"], "frontier_endpoint")
        snapshot = next(
            fields for _level, event, fields in self.failure_log_events
            if event == "failure_snapshot"
        )
        self.assertEqual(
            snapshot["diagnosis_at_trigger"]["graph_action"],
            "observe_local_work",
        )

    def test_trigger_sample_binds_target_event_goal_before_frontier_replan(self):
        metrics = self.metrics
        metrics.goal = (1.45, 0.35)
        metrics.goal_frame = "map"
        metrics.pose = (0.0, 0.0, 0.0)
        metrics._pose_xy_in_frame_locked = lambda _frame: (0.0, 0.0, 0.0)
        failure_id = metrics._begin_failure_episode_locked(
            "target_route_failed",
            "teb_goal_bridge",
            {
                "event": "target_route_failed",
                "goal": [100.0, 100.0],
                "goal_frame": "map",
                "transaction_id": 900,
                "status": "NAVFN_NO_PATH",
                "reason": "persistent_navfn_target_unreachable",
                "route_id": 1,
                "route_kind": "target_approach",
            },
        )
        self.assertEqual(failure_id, "20260908_120000-F0001")
        trigger = metrics.active_failure["trigger_sample"]
        self.assertEqual(trigger["goal"], [100.0, 100.0])
        self.assertEqual(trigger["goal_transaction_id"], 900)
        self.assertEqual(trigger["goal_source"], "target_route")
        self.assertEqual(trigger["failure_boundary"]["failure_id"], failure_id)
        self.assertEqual(trigger["failure_boundary"]["status"], "NAVFN_NO_PATH")
        self.assertEqual(trigger["target_plan_result"]["reason"], "persistent_navfn_target_unreachable")
        summary = metrics._finish_failure_episode_locked("test", force=True)
        self.assertEqual(summary["classification"], "planner_no_path")
        snapshot = next(
            fields for _level, event, fields in self.failure_log_events
            if event == "failure_snapshot"
        )
        self.assertEqual(
            snapshot["trigger_sample"]["goal"], [100.0, 100.0]
        )
        self.assertEqual(
            snapshot["diagnosis_at_trigger"]["layer"], "streaming_navfn"
        )

    def test_target_failure_keeps_target_action_when_frontier_successor_is_live(self):
        """A post-failure bootstrap route must not rename the failed action."""
        metrics = self.metrics
        metrics.goal = (1.45, 0.35)
        metrics.goal_frame = "map"
        metrics.pose = (0.0, 0.0, 0.0)
        metrics._pose_xy_in_frame_locked = lambda _frame: (0.0, 0.0, 0.0)
        metrics._failure_record_context_locked(
            "frontier",
            "work_item_dispatched",
            {
                "route_id": 1,
                "route_kind": "frontier_endpoint",
                "action": "bootstrap_observation",
                "work_item_id": 1,
            },
        )
        failure_id = metrics._begin_failure_episode_locked(
            "target_route_failed",
            "teb_goal_bridge",
            {
                "event": "target_route_failed",
                "goal": [100.0, 100.0],
                "goal_frame": "map",
                "transaction_id": 900,
                "status": "NAVFN_NO_PATH",
                "reason": "persistent_navfn_target_unreachable",
                "route_id": 0,
                "route_kind": "target_approach",
                "mission_route_kind": "target_approach",
            },
        )
        self.assertEqual(failure_id, "20260908_120000-F0001")
        self.assertEqual(
            metrics.active_failure["causal_route"]["route_kind"],
            "target_approach",
        )
        self.assertEqual(
            metrics.active_failure["causal_route"]["action"],
            "target_approach",
        )
        summary = metrics._finish_failure_episode_locked("test", force=True)
        self.assertEqual(summary["route_kind"], "target_approach")

    def test_forward_only_output_gap_opens_a_failure_episode(self):
        metrics = self.metrics
        metrics.goal = (0.57, 0.0)
        metrics.goal_frame = "odom"
        metrics._pose_xy_in_frame_locked = lambda _frame: (0.0, 0.0, 0.0)
        metrics.command = [0.0, 0.19]
        metrics.teb_command = [-0.01, 0.19]
        metrics.teb_planner_command = [-0.01, 0.14]
        metrics.cmd_vel_mux_status = {
            "filter_reason": "forward_only_reverse_clamp",
            "output": {"linear_x": 0.0, "angular_z": 0.19},
        }
        metrics.teb_feedback_state = {
            "selected_velocity": {"linear_x": -0.003, "angular_z": 0.30}
        }
        metrics.teb_turn_supervisor_status = {"state": "PASS_THROUGH"}
        metrics.failure_zero_velocity_limit = 1.0
        metrics.failure_controller_gap_start_wall = time.monotonic() - 2.0
        metrics.last_failure_evidence_wall = 0.0
        metrics.on_failure_evidence_sample()
        self.assertIsNotNone(metrics.active_failure)
        self.assertEqual(
            metrics.active_failure["trigger"], "controller_output_gap"
        )

    def test_public_sample_keeps_route_context_after_bounded_serialization(self):
        public = self.metrics._failure_public_sample(
            {
                "schema_version": 1,
                "pose": [0.0, 0.0],
                "goal": [3.0, 0.0],
                "cmd_vel": [0.0, 0.0],
                "route_context": {
                    "route": {
                        "route_id": 17,
                        "released_controller_route": {
                            "controller_pending": True,
                            "terminal_received": True,
                        },
                    }
                },
            }
        )
        self.assertEqual(public["route_context"]["route"]["route_id"], 17)
        self.assertTrue(
            public["route_context"]["route"]["released_controller_route"][
                "controller_pending"
            ]
        )

    def test_recorded_frontier_context_keeps_compact_graph_action(self):
        metrics = self.metrics
        metrics._failure_record_context_locked(
            "frontier",
            "route_command",
            {
                "route_id": 18,
                "route_kind": "portal_transition",
                "graph_route_plan": {
                    "action": "cross_portal",
                    "portal_path": [4, 8],
                    "obligation_kind": "portal_edge",
                },
                "event_graph": {
                    "place_count": 3,
                    "portal_count": 2,
                    "sequence": 41,
                },
            },
        )

        payload = metrics.last_frontier_context["payload"]
        self.assertEqual(payload["route_identity"]["route_id"], 18)
        self.assertEqual(payload["graph_route_plan"]["action"], "cross_portal")
        self.assertEqual(payload["event_graph"]["place_count"], 3)
        route = metrics._failure_route_context_locked()["route"]
        self.assertEqual(route["route_id"], 18)
        self.assertEqual(route["action"], "cross_portal")

        metrics.pose = (0.0, 0.0, 0.0)
        metrics.goal = (1.0, 0.0)
        metrics.goal_frame = "odom"
        metrics._pose_xy_in_frame_locked = lambda _frame: (0.0, 0.0, 0.0)
        public = metrics._failure_public_sample(
            metrics._failure_sample_locked(time.monotonic())
        )
        public_frontier = public["route_context"]["contexts"]["frontier"]
        self.assertEqual(
            public_frontier["payload"]["graph_route_plan"]["action"],
            "cross_portal",
        )
        self.assertNotEqual(public_frontier["payload"], "<max-depth>")

    def test_failure_sample_contains_robot_centred_costmap_window(self):
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="odom", stamp=None),
            info=SimpleNamespace(
                width=5,
                height=5,
                resolution=1.0,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0)
                ),
            ),
            data=list(range(25)),
        )
        window = self.metrics._failure_costmap_window_locked(
            message, radius_cells=1
        )
        self.assertEqual(window["center_cell"], [0, 0])
        self.assertEqual(window["values"][1][1], 0)
        self.assertEqual(window["values"][-1][1], 5)
        self.assertEqual(window["values"][-1][0], -1)

    def test_failure_sample_uses_route_status_when_navfn_path_races_trigger(self):
        metrics = self.metrics
        metrics.navfn_plan_stats = None
        metrics.last_frontier_context = {
            "payload": {
                "route_id": 31,
                "route_kind": "frontier_endpoint",
                "path_distance": 0.62,
                "goal": [3.0, 0.0],
            }
        }
        metrics.goal_frame = "odom"
        metrics._pose_xy_in_frame_locked = lambda _frame: (0.0, 0.0, 0.0)

        sample_value = metrics._failure_sample_locked(time.monotonic())

        self.assertEqual(
            sample_value["navfn_plan"]["source"],
            "frontier_route_summary",
        )
        self.assertTrue(sample_value["navfn_plan"]["derived"])
        self.assertEqual(sample_value["navfn_path_remaining_m"], 0.62)
        self.assertTrue(sample_value["channel_health"]["navfn_plan"]["available"])

    def test_failure_sample_marks_unavailable_diagnostic_channels(self):
        metrics = self.metrics
        metrics.navfn_plan_stats = None
        metrics.last_frontier_context = None
        metrics.last_bridge_context = None
        metrics.teb_feedback_state = None
        metrics.global_costmap_stats = None
        metrics.local_costmap_stats = None
        metrics.move_base_feedback_state = None

        sample_value = metrics._failure_sample_locked(time.monotonic())
        health = sample_value["channel_health"]

        self.assertEqual(health["navfn_plan"]["reason"], "no_value_at_boundary")
        self.assertEqual(health["teb_feedback"]["status"], "missing")
        self.assertEqual(health["global_costmap"]["status"], "missing")

    def test_closed_episode_writes_individual_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            self.metrics.failure_log_path = (
                Path(directory) / "20260908_120000_failure_evidence.log"
            )
            failure_id = self.metrics._begin_failure_episode_locked(
                "move_base_terminal_failure",
                "move_base",
                {"status_name": "ABORTED"},
            )
            summary = self.metrics._finish_failure_episode_locked(
                "test", force=True
            )
            artifact = Path(summary["artifact_path"])
            self.assertEqual(failure_id, "20260908_120000-F0001")
            self.assertTrue(artifact.is_file())
            self.assertIn(failure_id, artifact.read_text(encoding="utf-8"))
            self.assertEqual(self.metrics.failure_artifact_paths[-1], str(artifact))

    def test_failure_artifact_preserves_route_context_storage_channels(self):
        with tempfile.TemporaryDirectory() as directory:
            metrics = self.metrics
            metrics.failure_log_path = (
                Path(directory) / "20260908_120000_failure_evidence.log"
            )
            metrics._failure_record_context_locked(
                "frontier",
                "route_command",
                {
                    "route_id": 23,
                    "route_kind": "portal_transition",
                    "graph_route_plan": {
                        "action": "cross_portal",
                        "portal_path": [2, 9],
                        "obligation_kind": "portal_edge",
                    },
                },
            )
            metrics._begin_failure_episode_locked(
                "route_invalidated_stall",
                "global_frontier",
                {"route_id": 23, "reason": "stall"},
            )
            summary = metrics._finish_failure_episode_locked("test", force=True)
            payload = json.loads(Path(summary["artifact_path"]).read_text())

        route_context = payload["trigger_sample"]["route_context"]
        frontier_payload = route_context["contexts"]["frontier"]["payload"]
        self.assertEqual(frontier_payload["route_identity"]["route_id"], 23)
        self.assertEqual(
            frontier_payload["graph_route_plan"]["action"],
            "cross_portal",
        )
        self.assertNotEqual(frontier_payload, "<max-depth>")

    def test_failure_details_keep_contract_fields_without_recursive_graph_dump(self):
        with tempfile.TemporaryDirectory() as directory:
            metrics = self.metrics
            metrics.failure_log_path = (
                Path(directory) / "20260908_120000_failure_evidence.log"
            )
            failure_id = metrics._begin_failure_episode_locked(
                "frontier_route_stall",
                "global_frontier",
                {
                    "route_id": 7,
                    "route_kind": "portal_transition",
                    "reason": "selected_graph_obligation_not_executable_in_snapshot",
                    "event_graph": {
                        "active_route_id": 7,
                        "place_count": 3,
                        "portal_count": 4,
                        "evidence_contract": {
                            "requirements": [{"kind": "place_identity"}],
                        },
                    },
                    "graph_route_plan": {
                        "action": "cross_portal",
                        "portal_path": [2],
                        "obligation_kind": "portal_edge",
                    },
                },
            )
            self.assertEqual(failure_id, "20260908_120000-F0001")
            details = metrics.active_failure["details"]
            self.assertEqual(details["route_id"], 7)
            self.assertEqual(details["event_graph"]["place_count"], 3)
            self.assertEqual(details["graph_route_plan"]["action"], "cross_portal")
            self.assertNotIn("evidence_contract", details["event_graph"])
            self.assertNotIn("<max-depth>", json.dumps(details))

    def test_related_trigger_does_not_create_a_second_id(self):
        first = self.metrics._begin_failure_episode_locked(
            "unexpected_preemption", "move_base", {}
        )
        second = self.metrics._begin_failure_episode_locked(
            "frontier_route_failure", "global_frontier", {}
        )
        self.assertEqual(first, second)
        self.assertEqual(len(self.metrics.active_failure["related_events"]), 1)

    def test_post_trigger_events_keep_one_failure_id_and_are_not_duplicated(self):
        metrics = self.metrics
        metrics._failure_record_context_locked(
            "frontier",
            "route_invalidated",
            {"route_id": 8, "route_kind": "frontier_endpoint", "reason": "stall"},
        )
        failure_id = metrics._begin_failure_episode_locked(
            "route_invalidated_stall",
            "global_frontier",
            {"route_id": 8, "reason": "stall"},
        )

        trigger_context = next(
            item for item in reversed(metrics.failure_event_history)
            if item.get("event") == "route_invalidated"
        )
        self.assertEqual(trigger_context["failure_id"], failure_id)

        metrics._failure_record_context_locked(
            "bridge",
            "controller_lease_released",
            {
                "route_id": 0,
                "reason": "route_invalidated_stall",
                "released_controller_route": {
                    "route_id": 8,
                    "route_kind": "frontier_endpoint",
                },
            },
        )
        successor_context = metrics._failure_record_context_locked(
            "frontier",
            "dispatch",
            {
                "route_id": 9,
                "route_kind": "local_egress",
                "replacement": True,
            },
        )
        # The context callback is a single ownership boundary. Reattaching
        # the same object must not make the artifact report two events.
        metrics._failure_append_related_context_locked(successor_context)

        summary = metrics._finish_failure_episode_locked("test", force=True)
        self.assertEqual(summary["failure_id"], failure_id)
        snapshot = next(
            fields for _level, event, fields in self.failure_log_events
            if event == "failure_snapshot"
        )
        related = snapshot["related_events"]
        self.assertEqual(
            [item["event"] for item in related],
            ["controller_lease_released", "dispatch"],
        )
        self.assertTrue(all(item["failure_id"] == failure_id for item in related))

        timeline = snapshot["event_timeline"]
        correlated = [
            item for item in timeline
            if item.get("event") in {
                "route_invalidated",
                "failure_started",
                "controller_lease_released",
                "dispatch",
            }
        ]
        self.assertTrue(correlated)
        self.assertTrue(all(item.get("failure_id") == failure_id for item in correlated))

    def test_persistent_frontier_unavailability_is_promoted_only_after_a_grace_window(self):
        now = time.monotonic()
        self.metrics.failure_no_progress_limit = 1.0
        self.metrics.failure_frontier_unavailable_since_wall = now - 2.0
        self.metrics.failure_frontier_unavailable_count = 3
        # A sustained route-unavailable condition is promotable only when the
        # producer supplied a durable route identity.  This mirrors the
        # production status payload and prevents an unlocalizable episode.
        self.metrics.failure_frontier_unavailable_key = (
            10,
            "materialization_unavailable",
            1,
        )
        self.metrics.last_bridge_context = {
            "payload": {"target_failure_latched": True},
        }
        self.metrics.on_failure_evidence_sample()
        self.assertEqual(
            self.metrics.active_failure["trigger"], "frontier_route_stall"
        )
        self.assertEqual(
            self.metrics.active_failure["classification_initial"]["label"],
            "owner_transaction_race",
        )

    def _promote_frontier_route_stall(self, generation=4, reason="blocked"):
        self.metrics.failure_no_progress_limit = 1.0
        self.metrics._failure_record_context_locked(
            "frontier",
            "frontier_route_unavailable",
            {
                "route_id": 10,
                "reason": reason,
                "graph_route_action_transaction": {
                    "transaction_id": generation,
                },
            },
        )
        self.metrics.failure_frontier_unavailable_since_wall = (
            time.monotonic() - 2.0
        )
        self.metrics.failure_frontier_unavailable_count = 3
        self.metrics.last_failure_evidence_wall = 0.0
        self.metrics.on_failure_evidence_sample()

    def test_same_route_unavailable_identity_is_latched_after_post_window(self):
        self._promote_frontier_route_stall()
        failure_id = self.metrics.active_failure["failure_id"]
        self.assertEqual(
            self.metrics.active_failure["details"]["generation"], 4
        )
        self.metrics._finish_failure_episode_locked(
            "post_window_complete", force=True
        )
        self.metrics.last_failure_evidence_wall = 0.0
        self.metrics.on_failure_evidence_sample()
        self.assertEqual(len(self.metrics.failure_history), 1)
        self.assertIsNone(self.metrics.active_failure)
        self.assertEqual(
            self.metrics.failure_frontier_unavailable_latch["failure_id"],
            failure_id,
        )

    def test_generation_change_releases_unavailable_latch(self):
        self._promote_frontier_route_stall(generation=4)
        self.metrics._finish_failure_episode_locked(
            "post_window_complete", force=True
        )
        self.metrics._failure_record_context_locked(
            "frontier",
            "frontier_route_unavailable",
            {
                "route_id": 10,
                "reason": "blocked",
                "graph_route_action_transaction": {"transaction_id": 5},
            },
        )
        self.assertEqual(
            self.metrics.failure_frontier_unavailable_key,
            (10, "blocked", 5),
        )
        self.assertIsNone(self.metrics.failure_frontier_unavailable_latch)

    def test_planning_generation_churn_does_not_split_one_route(self):
        first = self.metrics._failure_frontier_unavailable_identity(
            {
                "route_id": 10,
                "reason": "selected_graph_obligation_not_executable_in_snapshot",
                "planning_contract": {"proposal_generation": 109},
                "graph_route_action_transaction": {"transaction_id": 15},
            }
        )
        second = self.metrics._failure_frontier_unavailable_identity(
            {
                "route_id": 10,
                "reason": "durable_identity_not_in_current_frontier_snapshot",
                "planning_contract": {"proposal_generation": 115},
                "graph_route_action_transaction": {"transaction_id": 15},
            }
        )
        self.assertEqual(first, second)

    def test_proposal_generation_is_compatibility_fallback(self):
        identity = self.metrics._failure_frontier_unavailable_identity(
            {
                "route_id": 10,
                "reason": "blocked",
                "planning_contract": {"proposal_generation": 7},
            }
        )
        self.assertEqual(identity, (10, "blocked", 7))

    def test_route_terminal_and_recovery_events_release_latch(self):
        for source, event, payload in (
            ("frontier", "route_command", {"route_id": 11}),
            ("bridge", "terminal", {"status": 3}),
            ("bridge", "controller_lease_released", {"route_id": 10}),
            ("move_base", "recovery", {"current": 1, "total": 4}),
        ):
            with self.subTest(source=source, event=event):
                self._promote_frontier_route_stall()
                self.metrics._finish_failure_episode_locked(
                    "post_window_complete", force=True
                )
                self.assertIsNotNone(
                    self.metrics.failure_frontier_unavailable_latch
                )
                self.metrics._failure_record_context_locked(
                    source, event, payload
                )
                self.assertIsNone(
                    self.metrics.failure_frontier_unavailable_latch
                )
                self.assertIsNone(
                    self.metrics.failure_frontier_unavailable_since_wall
                )

    def test_repeated_selection_of_same_route_does_not_release_latch(self):
        self._promote_frontier_route_stall()
        self.metrics._finish_failure_episode_locked(
            "post_window_complete", force=True
        )
        self.metrics._failure_record_context_locked(
            "frontier",
            "frontier_action_selected",
            {"route_id": 10},
        )
        self.assertIsNotNone(self.metrics.failure_frontier_unavailable_latch)
        self.assertIsNotNone(self.metrics.failure_frontier_unavailable_since_wall)

    def test_one_transient_frontier_candidate_miss_is_not_a_failure_episode(self):
        self.metrics._failure_record_context_locked(
            "frontier",
            "frontier_route_unavailable",
            {"reason": "durable_identity_not_in_current_frontier_snapshot"},
        )
        self.assertIsNone(self.metrics.active_failure)
        self.assertEqual(self.metrics.failure_frontier_unavailable_count, 1)

    def test_sustained_unidentified_route_is_context_only(self):
        """A missing route ID cannot create a failure nobody can locate."""
        self.metrics.failure_no_progress_limit = 1.0
        self.metrics._failure_record_context_locked(
            "frontier",
            "frontier_route_unavailable",
            {
                "reason": "selected_graph_obligation_not_executable_in_snapshot",
            },
        )
        self.metrics.failure_frontier_unavailable_since_wall = (
            time.monotonic() - 10.0
        )
        self.metrics.failure_frontier_unavailable_count = 20
        self.metrics.last_bridge_context = {
            "payload": {"target_failure_latched": True},
        }
        self.metrics.on_failure_evidence_sample()
        self.assertIsNone(self.metrics.active_failure)
        self.assertIsNone(self.metrics.failure_frontier_unavailable_key)

    def test_bounded_context_restores_route_failure_fields(self):
        self.metrics._failure_record_context_locked(
            "frontier",
            "route_invalidated",
            {
                "route_id": 10,
                "reason": "stall",
                "goal": [4.95, 17.35],
                "distance": 1.52,
                "last_progress_signal": "odom_novel_coverage",
                "active_intent_priority": 2,
                "latest_intent_priority": 0,
                "active_intent_source": "target_continuous_handoff",
                "latest_intent_source": "waiting_global_slam_frontier",
                **{"graph_field_%d" % index: index for index in range(64)},
            },
        )
        route = self.metrics._failure_route_context_locked()["route"]
        self.assertEqual(route["route_id"], 10)
        self.assertEqual(route["reason"], "stall")
        self.assertEqual(route["goal"], [4.95, 17.35])
        self.assertEqual(route["last_progress_signal"], "odom_novel_coverage")
        self.assertEqual(route["active_intent_priority"], 2)
        self.assertEqual(route["latest_intent_priority"], 0)

    def test_failure_distance_uses_goal_frame_transform(self):
        self.metrics.goal = (4.0, 6.0)
        self.metrics.goal_frame = "map"
        self.metrics.pose = (100.0, 100.0, 0.0)
        self.metrics._pose_xy_in_frame_locked = (
            lambda frame: (1.0, 2.0, 0.25) if frame == "map" else self.metrics.pose
        )
        sample = self.metrics._failure_sample_locked(time.monotonic())
        self.assertEqual(sample["pose"], [100.0, 100.0])
        self.assertEqual(sample["pose_goal_frame"], [1.0, 2.0, 0.25])
        self.assertEqual(sample["distance_to_goal"], 5.0)
        self.assertTrue(sample["pose_transform_available"])

    def test_frontier_route_invalidation_callback_opens_episode(self):
        self.metrics.execution_architecture = "persistent_stream"

        class Message:
            data = (
                '{"event":"route_invalidated","reason":"stall",'
                '"route_id":10,"goal":[4.95,17.35],"distance":1.52,'
                '"last_progress_signal":"odom_novel_coverage"}'
            )

        self.metrics.on_global_frontier_status(Message())
        self.assertIsNotNone(self.metrics.active_failure)
        self.assertEqual(
            self.metrics.active_failure["failure_id"],
            "20260908_120000-F0001",
        )
        self.assertEqual(
            self.metrics.active_failure["trigger"],
            "route_invalidated_stall",
        )
        self.assertEqual(
            self.metrics.active_failure["classification_initial"]["label"],
            "controller_stall",
        )
        summary = self.metrics._finish_failure_episode_locked(
            "post_window_complete", force=True
        )
        self.assertEqual(summary["classification"], "controller_stall")
        snapshot = next(
            fields for _level, event, fields in self.failure_log_events
            if event == "failure_snapshot"
        )
        self.assertEqual(
            snapshot["route_context_at_trigger"]["route"]["reason"],
            "stall",
        )
        self.assertEqual(
            snapshot["classification"]["evidence"]["route_invalidation_reason"],
            "stall",
        )


if __name__ == "__main__":
    unittest.main()
