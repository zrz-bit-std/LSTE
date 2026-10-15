"""Pure result normalization and aggregation for office-study artifacts."""

from __future__ import annotations

from collections import defaultdict
from statistics import mean


NUMERIC_COLUMNS = (
    "elapsed_wall_seconds",
    "startup_elapsed_wall_seconds",
    "startup_readiness_timeout_seconds",
    "startup_readiness_elapsed_wall_seconds",
    "trial_elapsed_wall_seconds",
    "cleanup_elapsed_wall_seconds",
    "summary_elapsed_wall_seconds",
    "failure_stop_observed_wall_seconds",
    "failure_stop_evidence_duration_seconds",
    "path_length_m",
    "task_done_seconds",
    "physical_room_reentries",
    "max_room_dwell_seconds",
    "max_effective_work_seconds_in_one_place",
    "repeated_work_item_dispatches",
    "duplicate_work_item_dispatches",
    "unmatched_work_item_settlements",
    "resolved_work_item_descendant_dispatches",
    "move_base_aborts",
    "move_base_unexpected_preemptions",
    "target_route_failures",
    "safety_override_events",
    "collision_events",
    "collision_rate",
    "failure_events",
    "failure_rate",
    "failure_episode_count",
    "failure_episode_open_count",
    "failure_owner_race_count",
    "failure_planner_no_path_count",
    "failure_planner_materialization_stall_count",
    "failure_controller_stall_count",
    "straight_path_angular_energy_per_m",
    "straight_path_steering_sign_flips",
    "unexplained_clear_path_brake_events",
    "max_stop_duration_seconds",
    "average_stop_duration_seconds",
    "stop_rate_per_minute",
    "zero_duration_seconds",
    "turn_only_duration_seconds",
    "route_stagnant_observed_count",
    "frontier_route_unavailable_event_count",
    "frontier_route_unavailable_route_count",
    "max_frontier_route_unavailable_span_seconds",
    "target_belief_updates",
    "target_belief_geometry_bindings",
    "target_direction_candidates",
    "portal_new_place_routes",
    "portal_covered_transit_routes",
    "portal_probe_count",
    "portal_probe_started",
    "portal_probe_observed",
    "portal_probe_failed",
    "graph_policy_rejections",
    "branch_first_crossings",
    "illegal_graph_action_count",
    "illegal_graph_action_rate",
    "phantom_place_count",
    "phantom_place_rate",
    "stale_transaction_commit_count",
    "transaction_violation_count",
    "same_edge_reselection_count",
    "suspended_place_count",
    "graph_route_plan_count",
    "graph_route_edge_materialized",
    "graph_route_multihop_plan_count",
    "graph_route_max_hops",
    "target_truth_exposed_frames",
    "target_truth_matched_frames",
    "target_truth_exposure_episodes",
    "target_truth_matched_exposure_episodes",
    "target_truth_episode_recall",
)


# These fields identify the contract under which trials are comparable.  A
# phase is not merely a label: topology-only and semantic runs intentionally
# have different terminal conditions and must never share an aggregate.
GROUP_COLUMNS = (
    "study_id",
    "phase",
    "method_requested",
    "level",
    "task_id",
        "terminal_event",
        "controller",
        "profile",
        "world_sha256",
        "pipeline_config_sha256",
)


def _number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _boolean_rate(items, field):
    values = [item.get(field) for item in items if isinstance(item.get(field), bool)]
    return None if not values else round(sum(values) / len(values), 6)


def _artifact_errors(record: dict) -> list[str]:
    """Return explicit evidence failures for one completed suite record.

    Hand-written unit fixtures predating the artifact contract intentionally do
    not carry these fields and remain usable for testing the pure reducer. Real
    suite records always carry ``terminal_event`` and the artifact status, so a
    timeout or failed verification cannot silently become a benchmark sample.
    """
    errors = []
    if record.get("infrastructure_failure"):
        errors.append("infrastructure_failure")
    if record.get("summary_status") not in (None, "written"):
        errors.append("summary_not_written")

    expected = str(record.get("terminal_event") or "").strip()
    observed = str(record.get("outcome") or "").strip()
    failure_stop = bool(record.get("stop_on_failure")) and observed == "failure_snapshot"
    failure_stop = failure_stop and bool(str(record.get("failure_stop_id") or "").strip())
    if expected:
        if not observed:
            errors.append("missing_terminal_outcome")
        elif observed != expected and not failure_stop:
            errors.append("terminal_event_not_observed")

    if record.get("video_required") is True and record.get("video_status") != "recorded":
        errors.append("video_not_recorded")

    # A timeout is not automatically a navigation failure, but it must still
    # leave a bounded final-state artifact so an incomplete trial is
    # diagnosable without reopening the full metrics stream.  Keep fixtures
    # from older runner versions backward compatible by enforcing this only
    # when the runner exposes the new field.
    if (
        observed in {"timeout", "interrupted"}
        and "trial_end_diagnostic_path" in record
        and not str(record.get("trial_end_diagnostic_path") or "").strip()
    ):
        errors.append("trial_end_diagnostic_missing")

    # A suite record with this field was produced by a runner that requested
    # topology verification.  A non-zero verifier result is a behavioral
    # failure that must remain in the comparison, while a missing result is an
    # incomplete artifact.
    if "topology_verification_exit_code" in record:
        code = record.get("topology_verification_exit_code")
        try:
            if code is None:
                errors.append("topology_verification_missing")
            else:
                int(code)
        except (TypeError, ValueError):
            errors.append("topology_verification_invalid")

    return sorted(set(errors))


def normalized_row(record: dict, summary: dict) -> dict:
    """Flatten one timestamped trial without discarding missing-evidence state."""
    lifecycle = summary.get("frontier_region_lifecycle") or {}
    room = summary.get("room_coverage") or {}
    dwell = room.get("max_dwell_seconds_by_region") or {}
    method_info = summary.get("exploration_method") or {}
    repeated_items = lifecycle.get("repeated_work_item_dispatches") or {}
    descendants = lifecycle.get("resolved_work_item_descendant_dispatches") or []
    probe_events = lifecycle.get("portal_probe_event_counts") or {}
    probe_results = lifecycle.get("portal_probe_result_counts") or {}
    smoothness = summary.get("motion_smoothness") or {}
    failures = summary.get("failure_safety") or {}
    work_execution = summary.get("work_item_execution") or {}
    stall = summary.get("stall_diagnostics") or {}
    failure_evidence = summary.get("failure_evidence") or {}
    failure_episodes = failure_evidence.get("episodes") or []
    failure_class_counts = defaultdict(int)
    for episode in failure_episodes:
        failure_class_counts[str(episode.get("classification", "unknown"))] += 1
    target_truth = summary.get("target_geometric_evaluation") or {}
    trial_end = summary.get("trial_end_diagnostic") or {}
    graph_invariants = summary.get("frontier_region_lifecycle", {}).get(
        "graph_invariants"
    ) or {}
    requested_method = str(record.get("method") or "").strip() or None
    observed_method = method_info.get("selected_method")
    observed_method = (
        str(observed_method).strip() if observed_method is not None else None
    )
    contract_consistent = method_info.get("method_contract_consistent") is True
    method_contract_valid = bool(
        requested_method
        and observed_method
        and contract_consistent
        and observed_method == requested_method
    )
    if not observed_method:
        method_contract_error = "missing_method_event"
    elif not contract_consistent:
        method_contract_error = "multiple_methods_observed"
    elif observed_method != requested_method:
        method_contract_error = "requested_observed_mismatch"
    else:
        method_contract_error = None

    artifact_errors = _artifact_errors(record)
    verification_code = record.get("topology_verification_exit_code")
    if verification_code is not None:
        try:
            verification_code = int(verification_code)
        except (TypeError, ValueError):
            verification_code = None
    method_contract_valid = bool(method_contract_valid)
    trial_valid = method_contract_valid and not artifact_errors
    task_success = bool(summary.get("task_done"))
    collision_events = _number(failures.get("collision_events"))
    collision_rate = (
        None if collision_events is None else float(collision_events > 0)
    )
    failure_values = [
        _number(failures.get("move_base_aborts")),
        _number(failures.get("move_base_unexpected_preemptions")),
        _number(failures.get("target_route_failures")),
        _number(failures.get("safety_override_events")),
    ]
    failure_events = (
        None if all(value is None for value in failure_values)
        else sum(value or 0.0 for value in failure_values)
    )
    failure_rate = None if failure_events is None else float(failure_events > 0)
    truth_enabled = target_truth.get("enabled")
    truth_available = (
        None
        if truth_enabled is not True
        else bool(
            target_truth.get("camera_ready")
            and target_truth.get("model_state_ready")
        )
    )
    truth_match = (
        None
        if truth_enabled is not True
        else bool(
            int(target_truth.get("matched_exposure_episodes") or 0) > 0
            or int(target_truth.get("matched_detector_frames") or 0) > 0
        )
    )
    target_success_verified = (
        None
        if truth_enabled is not True
        else bool(task_success and truth_available and truth_match)
    )
    return {
        "study_id": record.get("study_id"),
        "phase": record.get("phase"),
        "method_requested": requested_method,
        "method_observed": observed_method,
        "method_contract_consistent": contract_consistent,
        "method_contract_valid": method_contract_valid,
        "method_contract_error": method_contract_error,
        "artifact_errors": artifact_errors,
        "trial_valid": trial_valid,
        "level": record.get("level"),
        "trial_id": record.get("trial_id"),
        "seed": record.get("seed"),
        "task_id": record.get("task_id"),
        "terminal_event": record.get("terminal_event"),
        "outcome": record.get("outcome"),
        # Startup readiness is a separate lifecycle boundary. Keep it in the
        # normalized row so an infrastructure timeout can be diagnosed without
        # opening the nested experiment record or guessing from elapsed time.
        "startup_readiness_status": (
            (record.get("startup_readiness") or {}).get("status")
        ),
        "startup_readiness_reason": (
            (record.get("startup_readiness") or {}).get("reason")
            or record.get("startup_failure_reason")
        ),
        "startup_readiness_timeout_seconds": _number(
            record.get("startup_readiness_timeout_seconds")
        ),
        "startup_readiness_elapsed_wall_seconds": _number(
            record.get("startup_readiness_elapsed_wall_seconds")
        ),
        "elapsed_wall_seconds": _number(record.get("elapsed_wall_seconds")),
        "startup_elapsed_wall_seconds": _number(
            record.get("startup_elapsed_wall_seconds")
        ),
        "trial_elapsed_wall_seconds": _number(
            record.get("trial_elapsed_wall_seconds")
        ),
        "cleanup_elapsed_wall_seconds": _number(
            record.get("cleanup_elapsed_wall_seconds")
        ),
        "summary_elapsed_wall_seconds": _number(
            record.get("summary_elapsed_wall_seconds")
        ),
        "startup_failure_id": record.get("startup_failure_id"),
        "startup_failure_reason": record.get("startup_failure_reason"),
        "startup_failure_artifact_path": record.get(
            "startup_failure_artifact_path"
        ),
        "startup_failure_log_path": record.get("startup_failure_log_path"),
        "stop_on_failure": bool(record.get("stop_on_failure", False)),
        "failure_stop_id": record.get("failure_stop_id"),
        "failure_stop_classification": record.get("failure_stop_classification"),
        "failure_stop_diagnosis": record.get("failure_stop_diagnosis"),
        "failure_stop_artifact_path": record.get("failure_stop_artifact_path"),
        "failure_stop_route_id": record.get("failure_stop_route_id"),
        "failure_stop_route_kind": record.get("failure_stop_route_kind"),
        "failure_stop_trigger_pose": record.get("failure_stop_trigger_pose"),
        "failure_stop_trigger_goal": record.get("failure_stop_trigger_goal"),
        "failure_stop_command_chain": record.get("failure_stop_command_chain"),
        "failure_stop_started_metrics_seconds": record.get(
            "failure_stop_started_metrics_seconds"
        ),
        "failure_stop_completed_metrics_seconds": record.get(
            "failure_stop_completed_metrics_seconds"
        ),
        "failure_stop_evidence_duration_seconds": record.get(
            "failure_stop_evidence_duration_seconds"
        ),
        "failure_stop_observed_wall_seconds": record.get(
            "failure_stop_observed_wall_seconds"
        ),
        "trial_end_diagnostic_path": (
            record.get("trial_end_diagnostic_path")
            or trial_end.get("artifact_path")
        ),
        "trial_end_diagnostic_state": (
            record.get("trial_end_diagnostic_state")
            or trial_end.get("state")
        ),
        "trial_end_diagnostic_reason": (
            record.get("trial_end_diagnostic_reason")
            or trial_end.get("reason")
        ),
        "trial_end_diagnostic_termination_id": (
            record.get("trial_end_diagnostic_termination_id")
            or trial_end.get("termination_id")
        ),
        "trial_end_diagnostic_classification": (
            record.get("trial_end_diagnostic_classification")
            or trial_end.get("classification")
        ),
        "trial_end_diagnostic_progress": (
            record.get("trial_end_diagnostic_progress")
            or trial_end.get("progress")
        ),
        "controller_requested": record.get("controller"),
        "controller": record.get("controller_observed") or record.get("controller"),
        "profile": record.get("profile"),
        "world_sha256": record.get("world_sha256"),
        "pipeline_config_sha256": record.get("pipeline_config_sha256"),
        "git_revision": record.get("git_revision"),
        "git_dirty": record.get("git_dirty"),
        "detector": record.get("detector"),
        "record_path": record.get("record_path"),
        "run_directory": record.get("run_directory"),
        "metrics_log": record.get("metrics_log"),
        "video_required": record.get("video_required"),
        "topology_verification_exit_code": verification_code,
        "topology_verification_passed": (
            None
            if "topology_verification_exit_code" not in record
            else verification_code == 0
        ),
        "video_status": record.get("video_status"),
        "target_eval_enabled": truth_enabled,
        "target_truth_available": truth_available,
        "target_truth_match": truth_match,
        "target_success_verified": target_success_verified,
        "target_truth_exposed_frames": _number(
            target_truth.get("exposed_detector_frames")
        ),
        "target_truth_matched_frames": _number(
            target_truth.get("matched_detector_frames")
        ),
        "target_truth_exposure_episodes": _number(
            target_truth.get("exposure_episodes")
        ),
        "target_truth_matched_exposure_episodes": _number(
            target_truth.get("matched_exposure_episodes")
        ),
        "target_truth_episode_recall": _number(
            target_truth.get("episode_geometric_recall")
        ),
        "task_success": task_success,
        "path_length_m": _number(summary.get("path_length_m")),
        "task_done_seconds": _number(summary.get("task_done_seconds")),
        "physical_room_reentries": _number(room.get("total_room_reentries")),
        "max_room_dwell_seconds": max(
            (_number(value) or 0.0 for value in dwell.values()), default=None,
        ),
        "work_item_execution_status": work_execution.get("status"),
        "max_effective_work_seconds_in_one_place": _number(
            work_execution.get("max_effective_work_seconds_in_one_place")
        ),
        "repeated_work_item_dispatches": sum(
            int(value) for value in repeated_items.values()
        ),
        "duplicate_work_item_dispatches": _number(
            work_execution.get("duplicate_dispatch_count")
        ),
        "unmatched_work_item_settlements": _number(
            work_execution.get("unmatched_settlement_count")
        ),
        "resolved_work_item_descendant_dispatches": len(descendants),
        "move_base_aborts": _number(failures.get("move_base_aborts")),
        "move_base_unexpected_preemptions": _number(
            failures.get("move_base_unexpected_preemptions")
        ),
        "target_route_failures": _number(failures.get("target_route_failures")),
        "safety_override_events": _number(failures.get("safety_override_events")),
        "collision_events": collision_events,
        "collision_rate": collision_rate,
        "collision_truth_status": failures.get("collision_truth_status"),
        "failure_events": failure_events,
        "failure_rate": failure_rate,
        "failure_episode_count": _number(
            failure_evidence.get("episode_count")
        ),
        "failure_episode_open_count": _number(
            failure_evidence.get("open_count")
        ),
        "failure_owner_race_count": failure_class_counts.get(
            "owner_transaction_race", 0
        ),
        "failure_planner_no_path_count": failure_class_counts.get(
            "planner_no_path", 0
        ),
        "failure_planner_materialization_stall_count": failure_class_counts.get(
            "planner_materialization_stall", 0
        ),
        "failure_controller_stall_count": failure_class_counts.get(
            "controller_stall", 0
        ),
        "building_coverage": (summary.get("coverage") or {}).get(
            "building_truth_fraction"
        ),
        "coverage_status": (summary.get("coverage") or {}).get("status"),
        "straight_path_angular_energy_per_m": _number(
            smoothness.get("straight_path_angular_energy_per_m")
        ),
        "straight_path_steering_sign_flips": _number(
            smoothness.get("straight_path_steering_sign_flips")
        ),
        "unexplained_clear_path_brake_events": _number(
            smoothness.get("unexplained_clear_path_brake_events")
        ),
        "max_stop_duration_seconds": _number(
            smoothness.get("max_stop_duration_seconds")
        ),
        "average_stop_duration_seconds": _number(
            smoothness.get("average_stop_duration_seconds")
        ),
        "stop_rate_per_minute": _number(smoothness.get("stop_rate_per_minute")),
        "zero_duration_seconds": _number(smoothness.get("zero_duration_seconds")),
        "turn_only_duration_seconds": _number(
            smoothness.get("turn_only_duration_seconds")
        ),
        "route_stagnant_observed_count": _number(
            stall.get("route_stagnant_observed_count")
        ),
        "frontier_route_unavailable_event_count": _number(
            stall.get("frontier_route_unavailable_event_count")
        ),
        "frontier_route_unavailable_route_count": _number(
            stall.get("frontier_route_unavailable_route_count")
        ),
        "max_frontier_route_unavailable_span_seconds": _number(
            stall.get("max_frontier_route_unavailable_span_seconds")
        ),
        "target_belief_updates": _number(
            lifecycle.get("target_belief_updates")
        ),
        "target_belief_geometry_bindings": _number(
            lifecycle.get("target_belief_geometry_bindings")
        ),
        "target_direction_candidates": _number(
            lifecycle.get("target_direction_candidates")
        ),
        "portal_new_place_routes": _number(
            (lifecycle.get("portal_destination_class_counts") or {}).get(
                "new_place", 0
            )
        ),
        "portal_covered_transit_routes": _number(
            (lifecycle.get("portal_destination_class_counts") or {}).get(
                "covered_transit", 0
            )
        ),
        "portal_probe_count": _number(lifecycle.get("portal_probe_count")),
        "portal_probe_started": _number(
            probe_events.get("portal_probe_started", 0)
        ),
        "portal_probe_observed": _number(
            probe_results.get("observed", 0)
        ),
        "portal_probe_failed": _number(
            int(probe_results.get("failed", 0))
            + int(probe_results.get("blocked", 0))
        ),
        "graph_policy_rejections": _number(
            lifecycle.get("graph_policy_rejections")
        ),
        "branch_first_crossings": _number(
            graph_invariants.get("branch_first_crossings")
        ),
        "illegal_graph_action_count": _number(
            graph_invariants.get("illegal_graph_action_count")
        ),
        "illegal_graph_action_rate": _number(
            graph_invariants.get("illegal_graph_action_rate")
        ),
        "phantom_place_count": _number(
            graph_invariants.get("phantom_place_count")
        ),
        "phantom_place_rate": _number(
            graph_invariants.get("phantom_place_rate")
        ),
        "stale_transaction_commit_count": _number(
            graph_invariants.get("stale_transaction_commit_count")
        ),
        "transaction_violation_count": _number(
            graph_invariants.get("transaction_violation_count")
        ),
        "same_edge_reselection_count": _number(
            graph_invariants.get("same_edge_reselection_count")
        ),
        "suspended_place_count": _number(
            sum((graph_invariants.get("suspended_place_counts") or {}).values())
        ),
        "graph_route_plan_count": _number(
            lifecycle.get("graph_route_plan_count")
        ),
        "graph_route_edge_materialized": _number(
            lifecycle.get("graph_route_edge_materialized")
        ),
        "graph_route_multihop_plan_count": _number(
            lifecycle.get("graph_route_multihop_plan_count")
        ),
        "graph_route_max_hops": _number(
            lifecycle.get("graph_route_max_hops")
        ),
    }


def aggregate_rows(rows: list[dict]) -> list[dict]:
    """Compute aggregates from complete, contract-valid trials only.

    Invalid runs remain visible through ``excluded_trials`` and their IDs, but
    cannot dilute a method's success or cost metrics. Behavioral verification
    failures remain valid observations; missing terminal/artifact evidence is
    a data-quality failure rather than an accidental baseline result.
    """
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row.get(column) for column in GROUP_COLUMNS)].append(row)
    output = []
    for group_key, all_items in sorted(groups.items(), key=lambda item: str(item[0])):
        # A repeated trial identity is ambiguous evidence.  Exclude every
        # copy rather than choosing by filesystem order.
        trial_counts = defaultdict(int)
        for item in all_items:
            trial_id = item.get("trial_id")
            if trial_id is not None:
                trial_counts[trial_id] += 1
        duplicate_ids = {
            trial_id for trial_id, count in trial_counts.items() if count > 1
        }
        invalid = []
        items = []
        for item in all_items:
            reasons = list(item.get("artifact_errors") or [])
            if not bool(item.get("method_contract_valid")):
                reasons.append(
                    str(item.get("method_contract_error") or "invalid_method_contract")
                )
            if not bool(item.get("trial_valid", True)) and not reasons:
                reasons.append("invalid_trial_contract")
            if item.get("trial_id") in duplicate_ids:
                reasons.append("duplicate_trial_identity")
            if reasons:
                invalid.append((item, sorted(set(reasons))))
            else:
                items.append(item)
        group_values = dict(zip(GROUP_COLUMNS, group_key))
        aggregate = {
            **group_values,
            "trials": len(items),
            "excluded_trials": len(invalid),
            "excluded_trial_ids": [
                item.get("trial_id") for item, _reasons in invalid
            ],
            "excluded_trial_reasons": sorted({
                reason for _item, reasons in invalid for reason in reasons
            }),
            "startup_failure_trials": sum(
                item.get("outcome") == "startup_failure" for item in all_items
            ),
            "startup_failure_reasons": sorted({
                str(item.get("startup_failure_reason"))
                for item in all_items
                if item.get("startup_failure_reason")
            }),
            "task_success_rate": round(
                sum(bool(item.get("task_success")) for item in items) / len(items), 6
            ) if items else None,
            "target_success_verified_rate": _boolean_rate(
                items, "target_success_verified"
            ),
            "target_truth_match_rate": _boolean_rate(items, "target_truth_match"),
            "method_contract_mismatches": sum(
                not bool(item.get("method_contract_valid"))
                for item in all_items
            ),
            "missing_video_trials": sum(
                item.get("video_status") != "recorded"
                for item in all_items
                if item.get("video_required", True) is not False
            ),
            "missing_collision_truth_trials": sum(
                item.get("collision_events") is None for item in all_items
            ),
            "missing_building_coverage_trials": sum(
                item.get("building_coverage") is None for item in all_items
            ),
            "missing_target_truth_trials": sum(
                item.get("target_eval_enabled") is True
                and item.get("target_truth_available") is not True
                for item in all_items
            ),
            "topology_verification_failures": sum(
                item.get("topology_verification_exit_code") not in (None, 0)
                for item in all_items
            ),
            "topology_verification_pass_rate": (
                None
                if not any(
                    item.get("topology_verification_exit_code") is not None
                    for item in items
                )
                else round(
                    sum(
                        item.get("topology_verification_exit_code") == 0
                        for item in items
                        if item.get("topology_verification_exit_code") is not None
                    )
                    / sum(
                        item.get("topology_verification_exit_code") is not None
                        for item in items
                    ),
                    6,
                )
            ),
        }
        for field in NUMERIC_COLUMNS:
            values = [_number(item.get(field)) for item in items]
            values = [value for value in values if value is not None]
            aggregate[field + "_mean"] = (
                None if not values else round(float(mean(values)), 6)
            )
        output.append(aggregate)
    return output
