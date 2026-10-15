#!/usr/bin/env python3
"""Unit tests for metrics used in the paper-comparison table."""

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
BENCHMARK = ROOT / "scripts/tests/office_building"
if str(BENCHMARK) not in sys.path:
    sys.path.insert(0, str(BENCHMARK))

from experiment_aggregation import _artifact_errors, aggregate_rows, normalized_row
from aggregate_experiment_results import (
    TRIAL_KEY_FIELDS,
    compare_trial_keys,
    expected_trial_keys,
)
from experiment_protocol import load_matrix


class OfficeExperimentAggregationTest(unittest.TestCase):
    def test_matrix_completeness_accepts_exact_formal_trial_set(self):
        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        expected = expected_trial_keys(matrix)
        observed = [dict(zip(TRIAL_KEY_FIELDS, key)) for key in expected]

        result = compare_trial_keys(observed, expected)

        self.assertTrue(result["complete"])
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["expected_trial_count"], 144)
        self.assertEqual(result["missing_trial_count"], 0)
        self.assertEqual(result["unexpected_trial_count"], 0)
        self.assertEqual(result["duplicate_trial_count"], 0)

    def test_matrix_completeness_detects_missing_formal_trial(self):
        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        expected = expected_trial_keys(matrix)
        observed = [dict(zip(TRIAL_KEY_FIELDS, key)) for key in expected[:-1]]

        result = compare_trial_keys(observed, expected)

        self.assertFalse(result["complete"])
        self.assertEqual(result["status"], "incomplete")
        self.assertEqual(result["expected_trial_count"], len(expected))
        self.assertEqual(result["observed_record_count"], len(expected) - 1)
        self.assertEqual(result["missing_trial_count"], 1)
        self.assertEqual(
            result["missing_trial_keys"],
            [dict(zip(TRIAL_KEY_FIELDS, expected[-1]))],
        )

    def test_matrix_completeness_rejects_duplicate_and_unexpected_trial(self):
        expected = [
            ("study", "phase", "frontier", "level_2", 1, 7),
        ]
        observed = [
            dict(zip(TRIAL_KEY_FIELDS, expected[0])),
            dict(zip(TRIAL_KEY_FIELDS, expected[0])),
            {
                "study_id": "study",
                "phase": "phase",
                "method": "not_declared",
                "level": "level_2",
                "trial_id": 2,
                "seed": 8,
            },
        ]

        result = compare_trial_keys(observed, expected)

        self.assertFalse(result["complete"])
        self.assertEqual(result["duplicate_trial_count"], 1)
        self.assertEqual(result["unexpected_trial_count"], 1)
        self.assertEqual(result["duplicate_trial_keys"][0]["observed_count"], 2)
        self.assertEqual(
            result["unexpected_trial_keys"][0]["method"], "not_declared"
        )

    def test_normalization_preserves_missing_truth_instead_of_zeroing_it(self):
        row = normalized_row(
            {"method": "frontier", "level": "level_2", "trial_id": 1, "video_status": "missing"},
            {
                "task_done": False,
                "coverage": {"building_truth_fraction": None, "status": "pending"},
                "room_coverage": {"total_room_reentries": 2, "max_dwell_seconds_by_region": {"room": 3.0}},
                "frontier_region_lifecycle": {"repeated_work_item_dispatches": {"4": 2}, "resolved_work_item_descendant_dispatches": [{}, {}]},
                "failure_safety": {"move_base_aborts": 1, "target_route_failures": 0, "safety_override_events": 0, "collision_events": None},
                "motion_smoothness": {"straight_path_angular_energy_per_m": 0.2, "straight_path_steering_sign_flips": 1, "unexplained_clear_path_brake_events": 3},
                "exploration_method": {"selected_method": "frontier", "method_contract_consistent": True},
            },
        )
        self.assertIsNone(row["building_coverage"])
        self.assertIsNone(row["collision_events"])
        self.assertEqual(row["repeated_work_item_dispatches"], 2)
        self.assertEqual(row["resolved_work_item_descendant_dispatches"], 2)

    def test_aggregation_groups_only_like_method_and_level_trials(self):
        rows = [
            {"method_requested": "frontier", "method_observed": "frontier", "method_contract_valid": True, "level": "level_2", "task_success": True, "video_status": "missing", "collision_events": None, "building_coverage": None, "path_length_m": 10.0},
            {"method_requested": "frontier", "method_observed": "frontier", "method_contract_valid": True, "level": "level_2", "task_success": False, "video_status": "recorded", "collision_events": None, "building_coverage": 0.5, "path_length_m": 14.0},
        ]
        result = aggregate_rows(rows)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["trials"], 2)
        self.assertEqual(result[0]["task_success_rate"], 0.5)
        self.assertEqual(result[0]["path_length_m_mean"], 12.0)
        self.assertEqual(result[0]["missing_video_trials"], 1)
        self.assertEqual(result[0]["missing_collision_truth_trials"], 2)

    def test_invalid_method_evidence_is_excluded_from_metrics(self):
        rows = [
            {
                "method_requested": "frontier",
                "method_observed": "frontier",
                "method_contract_valid": True,
                "level": "level_2",
                "task_success": True,
                "path_length_m": 10.0,
            },
            {
                "method_requested": "frontier",
                "method_observed": None,
                "method_contract_valid": False,
                "method_contract_error": "missing_method_event",
                "level": "level_2",
                "task_success": False,
                "path_length_m": 100.0,
                "trial_id": 2,
            },
        ]
        result = aggregate_rows(rows)
        self.assertEqual(result[0]["trials"], 1)
        self.assertEqual(result[0]["excluded_trials"], 1)
        self.assertEqual(result[0]["excluded_trial_ids"], [2])
        self.assertEqual(result[0]["task_success_rate"], 1.0)
        self.assertEqual(result[0]["path_length_m_mean"], 10.0)

    def test_measured_zero_collision_and_coverage_are_not_treated_as_missing(self):
        row = normalized_row(
            {"method": "place_portal_workitem", "level": "level_1", "trial_id": 1},
            {
                "task_done": False,
                "coverage": {
                    "building_truth_fraction": 0.527911,
                    "status": "measured",
                },
                "room_coverage": {},
                "frontier_region_lifecycle": {},
                "work_item_execution": {
                    "status": "measured",
                    "max_effective_work_seconds_in_one_place": 11.5,
                },
                "failure_safety": {
                    "move_base_aborts": 0,
                    "target_route_failures": 0,
                    "safety_override_events": 0,
                    "collision_events": 0,
                },
                "motion_smoothness": {},
                "exploration_method": {},
            },
        )
        self.assertEqual(row["collision_events"], 0)
        self.assertEqual(row["building_coverage"], 0.527911)
        self.assertEqual(row["work_item_execution_status"], "measured")
        self.assertEqual(row["max_effective_work_seconds_in_one_place"], 11.5)

    def test_phase_and_task_identity_are_not_merged(self):
        rows = [
            {
                "study_id": "study",
                "phase": "topology_main",
                "method_requested": "frontier",
                "method_observed": "frontier",
                "method_contract_valid": True,
                "level": "level_4",
                "task_id": "topology_only",
                "terminal_event": "frontier_exhausted",
                "controller": "teb",
                "profile": "primary",
                "world_sha256": "world-a",
                "pipeline_config_sha256": "config-a",
                "trial_id": 1,
                "task_success": False,
            },
            {
                "study_id": "study",
                "phase": "semantic_main",
                "method_requested": "frontier",
                "method_observed": "frontier",
                "method_contract_valid": True,
                "level": "level_4",
                "task_id": "yellow_cup",
                "terminal_event": "task_done",
                "controller": "teb",
                "profile": "primary",
                "world_sha256": "world-a",
                "pipeline_config_sha256": "config-a",
                "trial_id": 1,
                "task_success": True,
            },
        ]

        result = aggregate_rows(rows)

        self.assertEqual(len(result), 2)
        self.assertEqual(
            {(item["phase"], item["task_id"]) for item in result},
            {("topology_main", "topology_only"), ("semantic_main", "yellow_cup")},
        )

    def test_incomplete_trial_is_excluded_but_behavioral_verifier_failure_is_visible(self):
        summary = {
            "task_done": False,
            "room_coverage": {},
            "frontier_region_lifecycle": {},
            "failure_safety": {
                "move_base_aborts": 0,
                "move_base_unexpected_preemptions": 0,
                "target_route_failures": 0,
                "safety_override_events": 0,
                "collision_events": 0,
            },
            "motion_smoothness": {},
            "exploration_method": {
                "selected_method": "frontier",
                "method_contract_consistent": True,
            },
        }
        row = normalized_row(
            {
                "study_id": "study",
                "phase": "topology_main",
                "method": "frontier",
                "level": "level_4",
                "task_id": "topology_only",
                "terminal_event": "frontier_exhausted",
                "outcome": "timeout",
                "video_required": True,
                "video_status": "failed",
                "topology_verification_exit_code": 1,
                "trial_id": 1,
            },
            summary,
        )

        self.assertFalse(row["trial_valid"])
        self.assertIn("terminal_event_not_observed", row["artifact_errors"])
        self.assertIn("video_not_recorded", row["artifact_errors"])
        result = aggregate_rows([row])
        self.assertEqual(result[0]["trials"], 0)
        self.assertEqual(result[0]["excluded_trials"], 1)
        self.assertIn("terminal_event_not_observed", result[0]["excluded_trial_reasons"])
        self.assertEqual(result[0]["topology_verification_failures"], 1)

    def test_diagnostic_failure_stop_is_an_observed_failed_trial(self):
        errors = _artifact_errors(
            {
                "terminal_event": "frontier_exhausted",
                "outcome": "failure_snapshot",
                "stop_on_failure": True,
                "failure_stop_id": "20260908_120000-F0001",
                "summary_status": "written",
                "video_required": False,
                "topology_verification_exit_code": 1,
            }
        )
        self.assertNotIn("terminal_event_not_observed", errors)

    def test_timeout_requires_new_trial_end_diagnostic_when_runner_exposes_it(self):
        record = {
            "terminal_event": "frontier_exhausted",
            "outcome": "timeout",
            "trial_end_diagnostic_path": "",
            "summary_status": "written",
            "video_required": False,
        }
        self.assertIn("trial_end_diagnostic_missing", _artifact_errors(record))

        record["trial_end_diagnostic_path"] = "/tmp/trial_end_diagnostic.json"
        self.assertNotIn("trial_end_diagnostic_missing", _artifact_errors(record))

        row_record = dict(record)
        row_record.update({
                "study_id": "study",
                "phase": "topology_main",
                "method": "frontier",
                "level": "level_2",
                "task_id": "topology_only",
                "trial_id": 1,
                "trial_end_diagnostic_state": "stall_candidate",
                "trial_end_diagnostic_reason": "active_route_without_effective_command",
            })
        row = normalized_row(
            row_record,
            {
                "task_done": False,
                "room_coverage": {},
                "frontier_region_lifecycle": {},
                "failure_safety": {},
                "motion_smoothness": {},
                "exploration_method": {},
            },
        )
        self.assertEqual(row["trial_end_diagnostic_state"], "stall_candidate")
        self.assertEqual(
            row["trial_end_diagnostic_reason"],
            "active_route_without_effective_command",
        )

    def test_startup_failure_provenance_is_flattened_for_audit(self):
        row = normalized_row(
            {
                "study_id": "study",
                "phase": "topology_smoke",
                "method": "place_portal_workitem",
                "level": "level_2",
                "task_id": "topology_only",
                "terminal_event": "frontier_exhausted",
                "outcome": "startup_failure",
                "startup_readiness": {
                    "status": "timeout",
                    "reason": "navfn_startup_probe_not_ready",
                },
                "startup_readiness_timeout_seconds": 45,
                "startup_readiness_elapsed_wall_seconds": 45.001,
                "startup_failure_id": "20260908_050000-S0001",
                "startup_failure_reason": "navfn_startup_probe_not_ready",
                "startup_failure_artifact_path": "/tmp/startup.json",
                "startup_failure_log_path": "/tmp/startup.log",
                "trial_id": 1,
            },
            {
                "task_done": False,
                "room_coverage": {},
                "frontier_region_lifecycle": {},
                "failure_safety": {},
                "motion_smoothness": {},
                "exploration_method": {},
            },
        )

        self.assertEqual(row["startup_readiness_status"], "timeout")
        self.assertEqual(row["startup_readiness_reason"], "navfn_startup_probe_not_ready")
        self.assertEqual(row["startup_failure_id"], "20260908_050000-S0001")
        self.assertEqual(row["startup_readiness_elapsed_wall_seconds"], 45.001)
        aggregate = aggregate_rows([row])[0]
        self.assertEqual(aggregate["startup_failure_trials"], 1)
        self.assertEqual(
            aggregate["startup_failure_reasons"],
            ["navfn_startup_probe_not_ready"],
        )

    def test_collision_failure_and_target_truth_are_flattened(self):
        row = normalized_row(
            {
                "study_id": "study",
                "phase": "semantic_main",
                "method": "place_portal_workitem",
                "level": "level_4",
                "task_id": "yellow_cup",
                "terminal_event": "task_done",
                "outcome": "task_done",
                "video_required": False,
                "topology_verification_exit_code": 0,
                "trial_id": 1,
            },
            {
                "task_done": True,
                "room_coverage": {},
                "frontier_region_lifecycle": {},
                "failure_safety": {
                    "move_base_aborts": 1,
                    "move_base_unexpected_preemptions": 2,
                    "target_route_failures": 0,
                    "safety_override_events": 0,
                    "collision_events": 1,
                },
                "motion_smoothness": {},
                "coverage": {"building_truth_fraction": 0.5, "status": "measured"},
                "exploration_method": {
                    "selected_method": "place_portal_workitem",
                    "method_contract_consistent": True,
                },
                "target_geometric_evaluation": {
                    "enabled": True,
                    "camera_ready": True,
                    "model_state_ready": True,
                    "exposed_detector_frames": 10,
                    "matched_detector_frames": 4,
                    "exposure_episodes": 2,
                    "matched_exposure_episodes": 1,
                    "episode_geometric_recall": 0.5,
                },
            },
        )

        self.assertEqual(row["collision_events"], 1)
        self.assertEqual(row["collision_rate"], 1.0)
        self.assertEqual(row["failure_events"], 3)
        self.assertEqual(row["failure_rate"], 1.0)
        self.assertTrue(row["target_success_verified"])
        self.assertEqual(row["target_truth_matched_frames"], 4)
        result = aggregate_rows([row])
        self.assertEqual(result[0]["collision_events_mean"], 1.0)
        self.assertEqual(result[0]["failure_rate_mean"], 1.0)
        self.assertEqual(result[0]["target_truth_episode_recall_mean"], 0.5)
        self.assertEqual(result[0]["target_success_verified_rate"], 1.0)
        self.assertEqual(result[0]["target_truth_match_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
