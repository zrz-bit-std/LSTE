"""Bounded failure episodes for the read-only navigation metrics node.

The normal metrics stream is intentionally compact and mostly counter based.
That is useful for benchmark aggregation, but it is a poor debugging artifact:
the pose, planner state, and route ownership at an abort are spread across
several callback streams.  This mixin keeps a small in-memory ring and writes
one correlated episode when a navigation failure is observed.  It never
publishes a command and never changes the navigation state machine.
"""

import datetime
import fcntl
import json
import math
import os
import re
import time
from collections import deque


FAILURE_SCHEMA_VERSION = 1
_CONFIDENCE_RANK = {"high": 3, "medium": 2, "low": 1}
_TEB_FEEDBACK_BOUNDARY_REASONS = frozenset(
    {
        "new_action_dispatch",
        "route_owner_changed",
        "persistent_endpoint_terminal",
        "execution_terminal",
        "controller_lease_released",
        "action_lease_cleared",
        "bridge_inactive",
        "hard_reset",
        "task_done",
    }
)


def _as_float(value, default=None):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _finite_or_none(value):
    value = _as_float(value)
    return None if value is None else round(value, 4)


def _json_safe(value, depth=0, max_items=32, max_string=512):
    """Bound arbitrary ROS payloads before putting them in a failure log."""
    if depth > 5:
        return "<max-depth>"
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        return value if len(value) <= max_string else value[:max_string] + "..."
    if isinstance(value, dict):
        items = list(value.items())[:max_items]
        return {
            str(key): _json_safe(item, depth + 1, max_items, max_string)
            for key, item in items
        }
    if isinstance(value, (list, tuple, deque)):
        return [
            _json_safe(item, depth + 1, max_items, max_string)
            for item in list(value)[:max_items]
        ]
    return str(value)[:max_string]


def _xy(value):
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    x = _as_float(value[0])
    y = _as_float(value[1])
    return None if x is None or y is None else (x, y)


def _command_pair(value):
    """Normalize one command payload for the failure-cause reducer."""
    if isinstance(value, dict):
        try:
            return [
                float(value.get("linear_x", 0.0) or 0.0),
                float(value.get("angular_z", 0.0) or 0.0),
            ]
        except (TypeError, ValueError):
            return [0.0, 0.0]
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return [float(value[0]), float(value[1])]
        except (TypeError, ValueError):
            return [0.0, 0.0]
    return [0.0, 0.0]


def _context_navfn_summary(mapping):
    """Extract a bounded planner summary from a producer status payload.

    The Navfn ``Path`` topic and the frontier/bridge status topic are separate
    ROS streams. A watchdog can invalidate a route before the next Path
    message reaches this observer, even though the status callback already
    contains the remaining-distance estimate. Mark that fallback as derived
    instead of presenting it as a raw Navfn sample.
    """
    if not isinstance(mapping, dict):
        return None
    raw_plan = mapping.get("navfn_plan")
    if isinstance(raw_plan, dict) and raw_plan:
        result = dict(raw_plan)
        result.setdefault("source", "producer_navfn_plan")
        return result

    remaining = None
    source = ""
    for key in (
        "navfn_path_remaining", "remaining_estimate_m", "path_distance",
        "best_path_distance",
    ):
        value = _as_float(mapping.get(key))
        if value is not None:
            remaining = value
            source = (
                "bridge_status_navfn_summary"
                if key.startswith("navfn_")
                else "frontier_route_summary"
            )
            break
    endpoint = None
    for key in (
        "navfn_path_endpoint", "endpoint", "goal", "command_goal",
        "mission_goal",
    ):
        endpoint = _xy(mapping.get(key))
        if endpoint is not None:
            break
    if remaining is None and endpoint is None:
        return None
    return {
        "poses": None,
        "length": _finite_or_none(remaining),
        "remaining_estimate_m": _finite_or_none(remaining),
        "endpoint": None if endpoint is None else [
            round(float(endpoint[0]), 4), round(float(endpoint[1]), 4)
        ],
        "source": source or "planner_status_summary",
        "derived": True,
    }


def _controller_output_gap(sample, goal_tolerance):
    """Return whether forward-only execution suppresses TEB's only motion.

    TEB can publish a valid band with a tiny negative linear sample while it
    is close to an obstacle.  The mux intentionally removes that reverse
    component on the production base.  A non-zero angular output is still an
    effective actuator command: the base may be deliberately reorienting before
    TEB resumes forward motion.  Only classify this as an execution gap when
    the mux has removed the reverse component and has also left no meaningful
    angular command.  The predicate is intentionally tied to the explicit mux
    filter reason and the command contract, not to a guessed timer.
    """
    if not isinstance(sample, dict):
        return False
    if sample.get("teb_feedback_valid") is False:
        return False
    mux = sample.get("cmd_vel_mux")
    mux = mux if isinstance(mux, dict) else {}
    if str(mux.get("filter_reason", "")).strip() != "forward_only_reverse_clamp":
        return False
    planner = _command_pair(sample.get("teb_planner_cmd"))
    output = _command_pair(mux.get("output"))
    feedback = sample.get("teb_feedback")
    feedback = feedback if isinstance(feedback, dict) else {}
    selected = _command_pair(feedback.get("selected_velocity"))
    distance = _as_float(sample.get("distance_to_goal"))
    if distance is None or distance <= float(goal_tolerance) + 0.02:
        return False
    if planner[0] >= -0.002 or selected[0] > 0.02:
        return False
    if abs(output[0]) > 0.02:
        return False
    # The forward-only clamp removes only linear reverse motion.  If the mux
    # still forwards TEB's meaningful angular command, the robot is executing
    # a bounded route reorientation rather than receiving no controller output.
    # Keep this threshold aligned with the existing telemetry definition of a
    # native TEB reorientation in navigation_metrics_command_events.py.
    if abs(output[1]) >= 0.10 and abs(selected[1]) >= 0.10:
        return False
    turn = sample.get("teb_turn_supervisor")
    turn = turn if isinstance(turn, dict) else {}
    return str(turn.get("state", "")).strip().upper() != "TURNING"


def _sample_route(sample):
    """Return the compact route state retained in a ring sample."""
    context = sample.get("route_context") if isinstance(sample, dict) else None
    if not isinstance(context, dict):
        return {}
    route = context.get("route", context)
    if not isinstance(route, dict):
        return {}
    # Frontier status uses ``active_route_id`` while bridge/goal status uses
    # ``route_id``.  Normalize the read-only diagnostic view so a failure
    # raised by the frontier still has one stable route identity in the
    # compact diagnosis and benchmark summary.
    normalized = dict(route)
    route_id = normalized.get("route_id")
    route_id_number = _as_float(route_id)
    if route_id_number is None or route_id_number <= 0.0:
        # A route-invalidated callback can clear the active lease before the
        # metrics sampler runs. Prefer the still-live positive identity from
        # the same context, and leave the field absent when none exists so the
        # failure details can supply the causal route below.
        normalized.pop("route_id", None)
        for key in ("active_route_id", "released_route_id"):
            value = normalized.get(key)
            value_number = _as_float(value)
            if value_number is not None and value_number > 0.0:
                normalized["route_id"] = value
                break
    if normalized.get("route_kind") in (None, ""):
        for key in ("active_route_kind", "released_route_kind"):
            value = normalized.get(key)
            if value not in (None, ""):
                normalized["route_kind"] = value
                break
    return normalized


def _failure_causal_identity(route=None, sample=None, details=None):
    """Expose one flat identity tuple for compact failure records."""
    sources = []
    # The producer's terminal contract is authoritative.  A later ring sample
    # may already belong to a newer projection epoch while the failure episode
    # is still waiting for its post-window to close.
    for value in (details, route, _sample_route(sample)):
        if isinstance(value, dict):
            sources.append(value)
            for nested_key in (
                "released_controller_route",
                "route_context",
                "route_identity",
                "frontier_execution_terminal_contract",
            ):
                nested = value.get(nested_key)
                if isinstance(nested, dict):
                    sources.append(nested)

    def first(*keys):
        for source in sources:
            for key in keys:
                value = source.get(key)
                if value is None or value == "":
                    continue
                try:
                    number = int(value)
                except (TypeError, ValueError):
                    continue
                if number > 0:
                    return number
        return None

    identity = {}
    for name, keys in (
        ("route_id", ("route_id", "active_route_id", "released_route_id")),
        ("action_generation", ("action_generation", "active_action_generation")),
        ("transaction_id", ("transaction_id", "active_goal_transaction_id")),
        ("lifecycle_transaction_id", ("lifecycle_transaction_id",)),
        (
            "graph_transaction_id",
            ("graph_transaction_id", "active_graph_transaction_id"),
        ),
        ("map_epoch", ("map_epoch", "active_route_map_epoch", "active_map_epoch")),
        ("origin_epoch", ("origin_epoch",)),
        ("projection_epoch", ("projection_epoch",)),
    ):
        value = first(*keys)
        if value is not None:
            identity[name] = value
    return identity


# Route status payloads are not uniform across the ROS boundary.  A live
# command carries ``route_id`` at the top level, while an invalidation often
# carries only the route state in an event history entry and clears the bridge
# lease in the same callback.  Keep the identity reducer independent of any
# one producer's payload shape so a failure can always be opened at the route
# that caused it.
_ROUTE_ID_KEYS = ("route_id", "active_route_id", "released_route_id")
_ROUTE_FIELD_KEYS = (
    "route_id", "active_route_id", "released_route_id", "route_kind",
    "active_route_kind", "released_route_kind", "mission_route_kind",
    "transaction_id", "lifecycle_transaction_id", "action_generation",
    "origin_epoch", "projection_epoch",
    "active_action_generation", "map_epoch", "active_route_map_epoch",
    "graph_transaction_id", "active_graph_transaction_id",
    "latest_graph_transaction_id",
    "goal", "command_goal", "mission_goal", "distance", "elapsed",
    "path_distance", "best_path_distance", "best_goal_distance",
    "last_progress_signal", "odom_detour_distance", "odom_novel_cells",
    "reason", "action", "graph_action", "obligation_kind", "obligation_id",
    "portal_path", "first_portal_id", "region_id", "region_state",
    "work_item_id", "attempt_id", "place_id", "source_place_id",
    "destination_place_id", "active_goal_transaction_id",
    "active_intent_priority", "active_intent_source",
)
_ROUTE_NESTED_KEYS = (
    "route", "active_route", "released_controller_route", "route_context",
    "payload", "active", "graph_route_plan", "graph_route_action_transaction",
    # ``_failure_route_context_locked`` groups producer snapshots under
    # ``contexts``. Walk those named channels as well, otherwise an
    # unexpected-preemption artifact can retain a diagnosis route id but lose
    # its top-level causal route after the lease is released.
    "contexts", "bridge", "frontier", "goal", "move_base",
)


def _positive_route_id(mapping):
    """Return the first positive route identity exposed by one mapping."""
    if not isinstance(mapping, dict):
        return None
    for key in _ROUTE_ID_KEYS:
        value = mapping.get(key)
        try:
            if value is not None and int(value) > 0:
                return int(value)
        except (TypeError, ValueError):
            continue
    return None


def _route_fields(mapping):
    """Extract compact route fields from a producer payload, if present."""
    if not isinstance(mapping, dict):
        return {}
    route_id = _positive_route_id(mapping)
    if route_id is None:
        return {}
    result = {}
    for key in _ROUTE_FIELD_KEYS:
        value = mapping.get(key)
        if value is None:
            continue
        if key in {"route_kind", "active_route_kind", "released_route_kind"}:
            if not str(value).strip():
                continue
        result[key] = value
    # Consumers should not have to know which of the three producer names was
    # used for the identity.  Preserve the source field too for forensics, but
    # always expose one canonical positive ``route_id``.
    result["route_id"] = route_id
    if not result.get("route_kind"):
        for key in ("active_route_kind", "released_route_kind", "mission_route_kind"):
            value = result.get(key)
            if value not in (None, ""):
                result["route_kind"] = value
                break
    return result


def _nested_route_fields(mapping, depth=0):
    """Find route identity in known nested status containers."""
    if not isinstance(mapping, dict) or depth > 4:
        return {}
    direct = _route_fields(mapping)
    if direct:
        return direct
    merged = {}
    for key in _ROUTE_NESTED_KEYS:
        nested = mapping.get(key)
        candidate = _nested_route_fields(nested, depth + 1)
        if not candidate:
            continue
        if not merged:
            merged = dict(candidate)
            continue
        # Producer snapshots can expose complementary fields in bridge and
        # frontier channels. Merge only matching route identities so a stale
        # neighboring status cannot overwrite the causal route.
        if _positive_route_id(merged) != _positive_route_id(candidate):
            continue
        for field, value in candidate.items():
            if field not in merged or merged.get(field) in (None, ""):
                merged[field] = value
    return merged


_GRAPH_ROUTE_FIELDS = (
    "action", "current_place_id", "target_place_id", "obligation_id",
    "obligation_kind", "portal_path", "first_portal_id", "portal_probe_phase",
    "reason", "status", "source_place_id", "destination_place_id",
    "place_id", "work_item_id", "route_id", "route_kind",
    "graph_transaction_id",
)


def _compact_graph_route_plan(mapping):
    """Keep the small graph action contract from a large frontier status."""
    if not isinstance(mapping, dict):
        return {}
    candidates = []
    direct = mapping.get("graph_route_plan")
    if isinstance(direct, dict):
        candidates.append(direct)
    transaction = mapping.get("graph_route_action_transaction")
    if isinstance(transaction, dict):
        plan = transaction.get("plan")
        if isinstance(plan, dict):
            candidates.append(plan)
        candidates.append(transaction)
    result = {}
    for candidate in candidates:
        for key in _GRAPH_ROUTE_FIELDS:
            value = candidate.get(key)
            if value is not None and result.get(key) in (None, ""):
                result[key] = _json_safe(value, max_items=32, max_string=512)
    return result


def _bounded_route_context(value):
    """Serialize route contexts without counting their wrapper depth twice."""
    if not isinstance(value, dict):
        return _json_safe(value, max_items=64, max_string=1024)
    result = {}
    route = value.get("route")
    if isinstance(route, dict):
        result["route"] = _json_safe(route, max_items=96, max_string=1024)
    contexts = value.get("contexts")
    if isinstance(contexts, dict):
        bounded_contexts = {}
        for name, context in list(contexts.items())[:8]:
            if not isinstance(context, dict):
                bounded_contexts[str(name)] = _json_safe(
                    context, max_items=64, max_string=1024
                )
                continue
            item = {
                key: context[key]
                for key in (
                    "event_sequence", "source", "event", "wall_elapsed_seconds",
                    "ros_time", "failure_id",
                )
                if key in context
            }
            payload = context.get("payload")
            # Producer payloads have already passed through the generic bound
            # in _failure_record_context_locked. Bound this object once at its
            # own root instead of recursing through context->payload again.
            item["payload"] = _json_safe(
                payload, max_items=96, max_string=1024
            )
            bounded_contexts[str(name)] = item
        result["contexts"] = bounded_contexts
    for key, item in value.items():
        if key not in result and key not in {"route", "contexts"}:
            result[key] = _json_safe(item, max_items=32, max_string=512)
    return result


_FAILURE_DETAIL_SCALARS = frozenset(
    (
        "event", "reason", "status", "status_name", "status_text", "action",
        "graph_action",
        "failure_reason", "route_invalidation_reason", "route_id",
        "active_route_id", "released_route_id", "route_kind",
        "active_route_kind", "released_route_kind", "mission_route_kind",
        "graph_transaction_id", "active_graph_transaction_id",
        "latest_graph_transaction_id",
        "goal", "goal_frame", "command_goal", "mission_goal", "distance", "elapsed",
        "path_distance", "best_path_distance", "best_goal_distance",
        "last_progress_signal", "odom_detour_distance", "odom_novel_cells",
        "obligation_kind", "obligation_id", "portal_path", "first_portal_id",
        "region_id", "region_state", "work_item_id", "attempt_id",
        "place_id", "source_place_id", "destination_place_id",
        "active_goal_transaction_id", "latest_goal_transaction_id",
        "lifecycle_transaction_id", "action_generation",
        "origin_epoch", "projection_epoch",
        "active_action_generation", "active_route_map_epoch",
        "active_intent_priority", "latest_intent_priority",
        "active_intent_source", "latest_intent_source", "target_epoch",
        "target_transaction_id",
        "target_failure_epoch", "target_failure_latched", "target_track_id",
        "target_failure_track_id", "target_terminal_boundary_transaction",
        "target_lease_tombstone_transaction_id", "target_lease_tombstone_epoch",
        "target_lease_tombstone_track_id",
        "transaction_id", "frame_id", "source", "failure_count",
        "controller_lease", "identity_validation",
        "target_viewpoint_candidate_id",
        "target_viewpoint_attempt_id",
        "duration_seconds", "unavailable_count", "identity_reason",
        "generation", "replacement", "next_owner", "lifecycle",
        "materialization_reason", "last_rejection_reason", "map_epoch",
        "rejected_map_epoch", "recovery_behavior", "recovery_reason",
    )
)
_FAILURE_DETAIL_NESTED = frozenset(
    (
        "event_graph", "graph_route_plan", "graph_route_action_transaction",
        "planning_contract", "place_progress", "decision_wake",
        "directional_branch_coverage", "portal_transaction",
        "released_controller_route", "goal_context", "portal_hypotheses",
        "portal_probes", "semantic_place", "pending_goal",
    )
)


def _compact_failure_details(value):
    """Keep diagnostic identity while dropping recursive producer state.

    Frontier status messages intentionally contain the entire Place/Portal graph
    and can be thousands of nested fields deep.  The failure sample already
    stores bounded producer channels; this projection keeps the contract-level
    fields in ``details`` without making the artifact unreadable or filling the
    report with repeated ``<max-depth>`` markers.
    """
    if not isinstance(value, dict):
        return _json_safe(value, max_items=64, max_string=1024)
    result = {}
    omitted = []
    for key, item in value.items():
        key = str(key)
        if key in _FAILURE_DETAIL_SCALARS:
            result[key] = _json_safe(item, max_items=32, max_string=512)
            continue
        if key not in _FAILURE_DETAIL_NESTED:
            omitted.append(key)
            continue
        if key == "graph_route_plan":
            compact = _compact_graph_route_plan({"graph_route_plan": item})
        elif key == "graph_route_action_transaction":
            compact = _compact_graph_route_plan(
                {"graph_route_action_transaction": item}
            )
            if isinstance(item, dict):
                for field in (
                    "transaction_id", "graph_transaction_id", "phase", "map_epoch"
                ):
                    if item.get(field) is not None:
                        compact[field] = _json_safe(item[field])
        elif key == "event_graph":
            compact = {
                field: _json_safe(item[field])
                for field in (
                    "active_route_id", "active_route_kind", "place_count",
                    "portal_count", "work_item_count", "structural_revision",
                    "sequence", "last_wake_event", "target_state",
                    "task_version",
                )
                if isinstance(item, dict) and item.get(field) is not None
            }
        elif key in {"planning_contract", "place_progress", "decision_wake"}:
            fields = {
                "planning_contract": (
                    "cycle_active", "cycle_skipped", "preempt_reason",
                    "preempt_requested", "proposal_generation",
                ),
                "place_progress": (
                    "place_id", "phase", "reason", "covered", "observed",
                    "local_observation_complete", "has_durable_progress",
                    "unresolved_portals", "unresolved_work_items",
                ),
                "decision_wake": (
                    "active_route_id", "decision_sequence", "halted",
                    "pending",
                ),
            }[key]
            compact = {
                field: _json_safe(item[field])
                for field in fields
                if isinstance(item, dict) and item.get(field) is not None
            }
        elif key == "portal_transaction":
            fields = (
                "state", "portal_id", "route_id", "transaction_id",
                "source_place_id", "destination", "gate", "last_reason",
                "last_transition", "retry_count", "source_side_proven",
                "active_crossing_observed", "active_crossing_rejected",
            )
            compact = {
                field: _json_safe(item[field], max_items=16, max_string=256)
                for field in fields
                if isinstance(item, dict) and item.get(field) is not None
            }
        elif key in {"directional_branch_coverage", "portal_hypotheses", "portal_probes"}:
            fields = (
                "count", "pending_work_items", "reopens", "transit_edges",
                "active_id", "states",
            )
            compact = {
                field: _json_safe(item[field], max_items=32, max_string=512)
                for field in fields
                if isinstance(item, dict) and item.get(field) is not None
            }
        elif key == "released_controller_route":
            fields = (
                "route_id", "route_kind", "graph_transaction_id",
                "controller_pending",
                "terminal_received",
            )
            compact = {
                field: _json_safe(item[field])
                for field in fields
                if isinstance(item, dict) and item.get(field) is not None
            }
        elif key == "goal_context":
            fields = (
                "schema_version", "task_id", "task_version", "mission_id",
                "goal_role", "owner_place_id", "source_place_id",
                "portal_crossing_certified", "portal_probe_id",
                "portal_probe_phase", "work_item_id", "target_track_id",
            )
            compact = {
                field: _json_safe(item[field], max_items=16, max_string=512)
                for field in fields
                if isinstance(item, dict) and item.get(field) is not None
            }
        else:
            compact = _json_safe(item, max_items=16, max_string=512)
        result[key] = compact
    if omitted:
        result["omitted_detail_keys"] = sorted(set(omitted))[:64]
    return result


def _compact_failure_event(value):
    """Keep one event's causal fields without serializing graph state twice."""
    if not isinstance(value, dict):
        return _json_safe(value, max_items=32, max_string=512)
    result = {
        key: _json_safe(value[key], max_items=16, max_string=512)
        for key in (
            "failure_id", "event_sequence", "source", "event",
            "wall_elapsed_seconds", "ros_time",
        )
        if key in value
    }
    payload = value.get("payload")
    if isinstance(payload, dict):
        result["payload"] = _compact_failure_details(payload)
    elif "details" in value:
        result["details"] = _compact_failure_details(value.get("details"))
    return result


# A target-plan rejection is a planner boundary, not an ownership race.  The
# bridge may immediately release the target lease and accept a frontier route,
# so later PREEMPTED/active-command samples must not replace the original
# Navfn decision when an investigator asks why the target failed.
_TARGET_ROUTE_FAILURE_TRIGGERS = frozenset(
    ("target_route_failed", "persistent_target_plan_failed", "target_plan_failed")
)


def _normalize_failure_route_identity(route, trigger="", details=None):
    """Keep the trigger owner visible when a successor route is already live.

    Target validation can fail synchronously, then the frontier immediately
    installs a bootstrap route before the metrics timer samples again.  The
    successor's graph action is useful in the post-window context, but it is
    not the action that failed.  Preserve the target route contract as the
    causal identity and let the surrounding context retain the successor.
    """
    result = dict(route) if isinstance(route, dict) else {}
    details = details if isinstance(details, dict) else {}
    trigger_name = str(trigger or "").strip().lower()
    mission_kind = str(
        details.get("mission_route_kind")
        or details.get("route_kind")
        or result.get("mission_route_kind")
        or ""
    ).strip().lower()
    if trigger_name not in _TARGET_ROUTE_FAILURE_TRIGGERS and mission_kind != "target_approach":
        return result
    # A target failure is a semantic route owner even if the bridge has
    # already cleared its lease.  The route id comes from the predecessor
    # context; the kind/action come from the failure boundary.
    result["route_kind"] = "target_approach"
    result["mission_route_kind"] = "target_approach"
    result["action"] = "target_approach"
    for key in (
        "target_epoch", "target_track_id", "target_failure_epoch",
        "target_failure_track_id", "target_viewpoint_attempt_id",
        "target_viewpoint_candidate_id", "transaction_id",
    ):
        if details.get(key) is not None:
            result[key] = details[key]
    return result


def _explicit_route_failure(trigger, details):
    """Map a route-level terminal reason to its owning layer."""
    trigger = str(trigger or "").strip().lower()
    details = details if isinstance(details, dict) else {}
    if trigger == "no_active_controller_route":
        return (
            "no_active_controller_route",
            "goal_manager/teb_goal_bridge",
            "high",
            "the experiment boundary has no executable controller route",
        )
    if trigger == "handoff_timeout":
        return (
            "handoff_timeout",
            "teb_goal_bridge/global_frontier",
            "high",
            "the route handoff lease expired before a matching successor was acknowledged",
        )
    if trigger == "controller_stall":
        return (
            "controller_stall",
            "teb_goal_bridge/cmd_vel_mux",
            "high",
            "the active route remained executable but produced no effective motion",
        )
    if trigger in {
        "graph_route_materialization_expired",
        "graph_route_materialization_invalidated",
    }:
        return (
            "planner_materialization_stall",
            "global_frontier",
            "high",
            "the durable graph obligation was dropped after it could not be materialized from the current frontier snapshot",
        )
    if trigger == "controller_output_gap":
        return "controller_output_gap", "teb_planner/cmd_vel_mux", "high", (
            "TEB selected a reverse-only terminal sample and the forward-only "
            "mux removed the only translational command"
        )
    reason = str(
        details.get("route_invalidation_reason")
        or details.get("reason")
        or details.get("failure_reason")
        or ""
    ).strip().lower()
    status = str(
        details.get("status") or details.get("status_name") or ""
    ).strip().lower()
    if trigger in _TARGET_ROUTE_FAILURE_TRIGGERS:
        if (
            any(
                token in reason
                for token in ("no_path", "no path", "unreachable", "disconnect")
            )
            or any(
                token in status
                for token in ("no_path", "no path", "unreachable")
            )
        ):
            return "planner_no_path", "streaming_navfn", "high", (
                "StreamingNavfnPlanner rejected the target before controller execution"
            )
        if any(token in reason for token in ("timeout", "stall", "controller")):
            return "controller_stall", "teb_goal_bridge", "medium", (
                "target route failed after the planner handoff and requires controller evidence"
            )
        # Keep a target failure explicit even when an older bridge only emits a
        # generic reason. It is still more useful than inferring an ownership
        # race from the subsequent action cancellation.
        return "planner_no_path", "streaming_navfn", "medium", (
            "target route validation failed before an executable controller route was installed"
        )
    if trigger in {
        "route_invalidated_stall",
        "route_invalidated_timeout",
        "route_invalidated_controller_failure",
        "route_invalidated_failure",
    }:
        if trigger == "route_invalidated_timeout" or "timeout" in reason:
            return "controller_stall", "global_frontier", "high", (
                "frontier watchdog invalidated the route after its progress deadline"
            )
        if "no_path" in reason or "unreachable" in reason or "disconnect" in reason:
            return "planner_no_path", "global_frontier", "high", (
                "frontier route became disconnected or unreachable"
            )
        return "controller_stall", "global_frontier", "high", (
            "frontier watchdog invalidated a route with no usable progress"
        )
    if trigger == "frontier_route_failure":
        if "no_path" in reason or "unreachable" in reason or "disconnect" in reason:
            return "planner_no_path", "global_frontier", "high", (
                "frontier reported an unreachable route"
            )
        if "stall" in reason or "no_progress" in reason or reason in {
            "controller_failure", "recovery_exhausted",
        }:
            return "controller_stall", "global_frontier", "high", (
                "frontier reported a route execution stall"
            )
    return None


def diagnose_failure_sample(sample, details=None):
    """Reduce one snapshot sample to an actionable layer/cause diagnosis.

    The raw channels remain in the snapshot.  This small deterministic reducer
    only names the first boundary at which the execution contract diverged,
    which is what an investigator needs before opening the full timeline.
    """
    sample = sample if isinstance(sample, dict) else {}
    details = details if isinstance(details, dict) else {}
    route = _normalize_failure_route_identity(
        _sample_route(sample),
        details.get("trigger") or details.get("event"),
        details,
    )
    released = route.get("released_controller_route")
    released = released if isinstance(released, dict) else {}
    terminal_wait = bool(
        released.get("controller_pending")
        and released.get("terminal_received")
    )
    planner = _command_pair(sample.get("teb_planner_cmd"))
    supervisor = _command_pair(sample.get("teb_cmd"))
    actuator = _command_pair(sample.get("cmd_vel"))
    mux = sample.get("cmd_vel_mux")
    mux = mux if isinstance(mux, dict) else {}
    mux_input = _command_pair(mux.get("input"))
    mux_output = _command_pair(mux.get("output"))
    feedback = sample.get("teb_feedback")
    feedback = feedback if isinstance(feedback, dict) else {}
    selected = _command_pair(feedback.get("selected_velocity"))
    turn_supervisor = sample.get("teb_turn_supervisor")
    turn_supervisor = (
        turn_supervisor if isinstance(turn_supervisor, dict) else {}
    )
    navfn = sample.get("navfn_plan")
    navfn = navfn if isinstance(navfn, dict) else {}
    scan = sample.get("scan")
    scan = scan if isinstance(scan, dict) else {}
    threshold = _as_float(sample.get("obstacle_clearance_threshold"), 0.0) or 0.0
    forward_clearance = _as_float(scan.get("forward_min"))
    explicit_failure = _explicit_route_failure(
        sample.get("failure_trigger")
        or details.get("trigger")
        or details.get("event"),
        details,
    )
    if explicit_failure is not None and (
        not terminal_wait
        or explicit_failure[0] in {"handoff_timeout", "no_active_controller_route"}
    ):
        cause, layer, confidence, explanation = explicit_failure
    elif terminal_wait:
        cause = "planner_materialization_stall"
        layer = "global_frontier"
        confidence = "high"
        explanation = "endpoint terminal received but successor route lease remains pending"
    elif (
        selected[0] > 0.05
        and planner[0] <= 0.05
        and sample.get("teb_feedback_valid", True)
        and sample.get("teb_status") == "trajectory_valid"
    ):
        cause = "stale_teb_feedback"
        layer = "teb_planner"
        confidence = "high"
        explanation = "feedback advertises motion after the planner command became zero"
    elif abs(planner[0]) > 0.05 or abs(planner[1]) > 0.05:
        if abs(supervisor[0]) <= 0.05 and abs(supervisor[1]) <= 0.03:
            cause = "turn_supervisor_output_gap"
            layer = "teb_turn_supervisor"
            confidence = "high"
            explanation = "TEB produced a command that the execution adapter did not forward"
        elif abs(actuator[0]) <= 0.05 and abs(actuator[1]) <= 0.03:
            cause = "mux_output_gap"
            layer = "cmd_vel_mux"
            confidence = "high"
            explanation = "the selected command was non-zero before the actuator mux"
        else:
            cause = "active_command"
            layer = "controller"
            confidence = "low"
            explanation = "controller command remains non-zero at the actuator boundary"
    elif navfn.get("poses") == 0 or (
        sample.get("teb_status") == "no_selected_trajectory"
    ):
        cause = "planner_no_path"
        layer = "navfn_teb"
        confidence = "high"
        explanation = "no executable global or local trajectory was available"
    elif (
        forward_clearance is not None
        and threshold > 0.0
        and forward_clearance <= threshold
    ):
        cause = "local_obstacle"
        layer = "costmap_safety"
        confidence = "high"
        explanation = "forward laser clearance is inside the braking/footprint envelope"
    else:
        cause = "unknown"
        layer = "undetermined"
        confidence = "low"
        explanation = "recorded channels do not distinguish a single causal boundary"
    route_id = route.get("route_id")
    route_id_number = _as_float(route_id)
    if route_id_number is None or route_id_number <= 0.0:
        route_id = None
        for key in ("route_id", "active_route_id", "released_route_id"):
            value = details.get(key)
            value_number = _as_float(value)
            if value_number is not None and value_number > 0.0:
                route_id = value
                break
    route_kind = (
        route.get("route_kind")
        or route.get("active_route_kind")
        or route.get("released_route_kind")
        or details.get("route_kind")
        or details.get("active_route_kind")
        or details.get("released_route_kind")
    )
    turn_status = sample.get("teb_turn_supervisor")
    turn_status = turn_status if isinstance(turn_status, dict) else {}
    route_kind = route_kind or turn_status.get("active_route_kind") or turn_status.get(
        "route_kind"
    )
    graph_plan = details.get("graph_route_plan")
    graph_plan = graph_plan if isinstance(graph_plan, dict) else {}
    graph_transaction = details.get("graph_route_action_transaction")
    graph_transaction = (
        graph_transaction if isinstance(graph_transaction, dict) else {}
    )
    graph_transaction_plan = graph_transaction.get("plan")
    graph_transaction_plan = (
        graph_transaction_plan if isinstance(graph_transaction_plan, dict) else {}
    )
    graph_action = (
        route.get("action")
        or route.get("graph_action")
        or details.get("action")
        or details.get("graph_action")
        or graph_plan.get("action")
        or graph_transaction_plan.get("action")
    )
    obligation_kind = (
        route.get("obligation_kind")
        or details.get("obligation_kind")
        or graph_plan.get("obligation_kind")
        or graph_transaction_plan.get("obligation_kind")
    )
    obligation_id = (
        route.get("obligation_id")
        or details.get("obligation_id")
        or graph_plan.get("obligation_id")
        or graph_transaction_plan.get("obligation_id")
    )
    portal_path = (
        route.get("portal_path")
        or details.get("portal_path")
        or graph_plan.get("portal_path")
        or graph_transaction_plan.get("portal_path")
    )
    return {
        "primary_cause": cause,
        "layer": layer,
        "confidence": confidence,
        "explanation": explanation,
        "route_id": route_id if route_id is not None else released.get("route_id"),
        "route_kind": route_kind or released.get("route_kind"),
        "graph_action": graph_action,
        "obligation": {
            "kind": obligation_kind,
            "id": obligation_id,
            "portal_path": _json_safe(portal_path),
        },
        "terminal_wait": terminal_wait,
        "execution_phase": {
            "turn_supervisor_state": str(
                turn_supervisor.get("state", "UNKNOWN") or "UNKNOWN"
            ).strip().upper(),
            "turn_supervisor_route_kind": str(
                turn_supervisor.get("active_route_kind")
                or turn_supervisor.get("route_kind", "")
                or ""
            ).strip().lower(),
            "turn_phase": str(
                turn_supervisor.get("turn_phase", "") or ""
            ).strip().lower(),
            "yaw_error": _finite_or_none(turn_supervisor.get("yaw_error")),
            "target_yaw": _finite_or_none(turn_supervisor.get("target_yaw")),
            "mux_filter_reason": str(
                mux.get("filter_reason", "") or ""
            ).strip().lower(),
            "mux_block_reason": str(
                mux.get("block_reason", "") or ""
            ).strip().lower(),
        },
        "command_chain": {
            "teb_selected": [round(value, 4) for value in selected],
            "teb_planner": [round(value, 4) for value in planner],
            "teb_supervisor": [round(value, 4) for value in supervisor],
            "mux_input": [round(value, 4) for value in mux_input],
            "mux_output": [round(value, 4) for value in mux_output],
            "cmd_vel": [round(value, 4) for value in actuator],
        },
        "spatial": {
            "pose": _json_safe(sample.get("pose_goal_frame")),
            "pose_frame": _json_safe(sample.get("goal_frame")),
            "goal": _json_safe(sample.get("goal")),
            "goal_frame": _json_safe(sample.get("goal_frame")),
            "distance_to_goal": _finite_or_none(sample.get("distance_to_goal")),
            "navfn_path_remaining_m": _finite_or_none(
                sample.get("navfn_path_remaining_m")
            ),
            "forward_clearance_m": _finite_or_none(forward_clearance),
        },
        "timing": {
            "teb_feedback_age_seconds": _finite_or_none(
                sample.get("teb_feedback_age_seconds")
            ),
            "mux_status_age_seconds": _finite_or_none(
                sample.get("mux_status_age_seconds")
            ),
        },
    }


def _event_text(details, recent_events, sample_elapsed=None):
    values = []
    if isinstance(details, dict):
        values.extend(
            str(details.get(key, ""))
            for key in (
                "event", "reason", "status", "status_text", "failure_reason",
                "route_invalidation_reason",
            )
        )
    for event in recent_events:
        if isinstance(event, dict):
            event_elapsed = _as_float(event.get("wall_elapsed_seconds"))
            if (
                sample_elapsed is not None
                and event_elapsed is not None
                and sample_elapsed - event_elapsed > 3.0
            ):
                continue
            values.append(str(event.get("event", "")))
            values.append(str(event.get("source", "")))
            payload = event.get("payload")
            if isinstance(payload, dict):
                values.extend(str(payload.get(key, "")) for key in (
                    "event", "reason", "replacement_kind", "active_intent_source",
                    "latest_intent_source",
                ))
    return " ".join(values).lower()


_ROUTE_INVALIDATION_EXPLICIT_TRIGGERS = frozenset(
    (
        "route_invalidated_stall",
        "route_invalidated_timeout",
        "route_invalidated_controller_failure",
        "route_invalidated_failure",
    )
)
_ROUTE_INVALIDATION_STALL_TRIGGERS = frozenset(
    ("route_invalidated_stall", "route_invalidated_timeout")
)

# A route-unavailable status is emitted on every planning wake while the graph
# cannot materialize the selected obligation.  These events are useful
# context, but they are one sustained condition until a lifecycle boundary
# occurs.  Keep the boundary vocabulary here so the watchdog and the
# cross-topic context recorder use the same definition.
_FRONTIER_UNAVAILABLE_EVENTS = frozenset(
    ("frontier_route_unavailable", "graph_route_candidate_unavailable")
)
_FRONTIER_NEW_ROUTE_EVENTS = frozenset(
    (
        "route_command",
        "frontier_action_selected",
        "graph_route_edge_materialized",
        "dispatch",
        "persistent_mission_goal_adopted",
    )
)
_FRONTIER_TERMINAL_EVENTS = frozenset(
    (
        "terminal",
        "route_terminal",
        "route_invalidated",
        "execution_terminal_failure",
        "frontier_route_failed",
        "portal_hypothesis_failed",
        "portal_execution_failure_observed",
        "terminal_prefetch_promoted",
        "persistent_execution_terminal",
        "persistent_frontier_endpoint_terminal",
        "endpoint_action_terminal",
        "succeeded",
        "aborted",
        "rejected",
        "preempted",
        "controller_stall",
        "graph_route_materialization_invalidated",
        "graph_route_materialization_expired",
    )
)
_FRONTIER_RECOVERY_EVENTS = frozenset(
    (
        "recovery",
        "move_base_recovery",
        "graph_recovery_requested",
        "controller_lease_released",
    )
)
# A materialization watchdog can fire while the executive is between two
# controller leases.  Keep this class recoverable until the post window proves
# that no successor route was installed.  Explicit terminal failures and hard
# reset boundaries are never eligible for this recovery path.
_RECOVERABLE_MATERIALIZATION_TRIGGERS = frozenset(
    (
        "frontier_route_stall",
        "planner_materialization_stall",
        "zero_velocity_stall",
    )
)
_RECOVERY_ROUTE_EVENTS = frozenset(
    (
        "route_command",
        "frontier_action_selected",
        "graph_route_edge_materialized",
        "dispatch",
        "persistent_mission_goal_adopted",
        "persistent_mission_path_adopted",
        "active",
        "lifecycle_transition",
    )
)
_FRONTIER_UNAVAILABLE_REASON_ALIASES = {
    # Both records are emitted by the same graph-route materialization wait;
    # the first is the bridge's durable-identity check and the second is the
    # graph adapter's candidate check. Treating them as different identities
    # would reopen one physical stall every time the producer alternates its
    # diagnostic wording.
    "durable_identity_not_in_current_frontier_snapshot": "materialization_unavailable",
    "selected_graph_obligation_not_executable_in_snapshot": "materialization_unavailable",
}


def classify_failure(trigger, sample, details=None, recent_events=()):
    """Return an evidence-based, conservative failure classification.

    The classifier is deliberately a ranked explanation, not a claim that a
    single sensor proves causality.  ``candidates`` remains in the snapshot
    so a later offline audit can reject an uncertain label without rerunning
    Gazebo.
    """
    sample = sample if isinstance(sample, dict) else {}
    details = details if isinstance(details, dict) else {}
    trigger = str(trigger or "unknown")
    if bool(sample.get("navigation_hold")) and trigger in {
        "controller_stall",
        "zero_velocity_stall",
        "no_progress_stall",
        "controller_output_gap",
    }:
        return {
            "label": "intentional_navigation_hold",
            "confidence": "high",
            "candidates": [{
                "label": "intentional_navigation_hold",
                "confidence": "high",
                "reason": "navigation hold intentionally suppresses controller output",
            }],
            "evidence": {"navigation_hold": True},
        }
    text = _event_text(
        details,
        recent_events,
        sample_elapsed=_as_float(sample.get("wall_elapsed_seconds")),
    )
    candidates = []
    route = _sample_route(sample)
    released_route = route.get("released_controller_route")
    released_route = released_route if isinstance(released_route, dict) else {}
    endpoint_terminal_wait = bool(
        released_route.get("controller_pending")
        and released_route.get("terminal_received")
    )
    explicit_route_failure = _explicit_route_failure(trigger, details)
    if explicit_route_failure is not None:
        label, _layer, confidence, reason = explicit_route_failure
        candidates.append((label, confidence, reason))

    if trigger == "frontier_route_stall":
        if bool(details.get("target_failure_latched")):
            candidates.append(
                ("owner_transaction_race", "high", "target lease and frontier route are both waiting")
            )
        elif endpoint_terminal_wait:
            # The persistent action deliberately outputs zero after its
            # endpoint terminal.  A route that remains pending at that point
            # is a graph/materialization deadlock, not a local-controller
            # stall, even when the forward laser is clear.
            candidates.append(
                (
                    "planner_materialization_stall",
                    "high",
                    "endpoint terminal received but successor route lease remains pending",
                )
            )
        else:
            candidates.append(
                ("planner_no_path", "medium", "frontier remained unmaterialized across planning cycles")
            )

    # A terminal followed by a released controller lease is a stronger causal
    # boundary than the accompanying PREEMPTED/hand-off event.  Without this
    # explicit candidate, the generic ownership token below can label a
    # materialization deadlock as an owner race even though the command chain
    # and route context prove that the successor lease is what is missing.
    if endpoint_terminal_wait and not explicit_route_failure:
        candidates.append(
            (
                "planner_materialization_stall",
                "high",
                "endpoint terminal received but successor route lease remains pending",
            )
        )

    route_race_tokens = (
        "handoff", "replacement", "priority", "preempt", "owner", "higher_priority",
    )
    if any(token in text for token in route_race_tokens) and not endpoint_terminal_wait and trigger not in (
        "zero_velocity_stall", "no_progress_stall",
    ) and trigger not in _ROUTE_INVALIDATION_EXPLICIT_TRIGGERS and trigger not in {
        "frontier_route_failure",
        # Target-plan release/preempt events are expected consequences of the
        # failed target, not evidence that ownership was the root cause.
    } and trigger not in _TARGET_ROUTE_FAILURE_TRIGGERS:
        candidates.append(("owner_transaction_race", "high", "recent ownership transition"))

    recovery = sample.get("recovery")
    recovery_exhausted = False
    if isinstance(recovery, dict):
        current = _as_float(recovery.get("current"))
        total = _as_float(recovery.get("total"))
        recovery_exhausted = (
            current is not None and total is not None and total > 0 and current >= total - 1
        )
    if recovery_exhausted or "recovery" in text:
        candidates.append(("recovery_exhausted", "high" if recovery_exhausted else "medium",
                           "move_base recovery state"))

    navfn = sample.get("navfn_plan")
    teb = sample.get("teb_feedback")
    navfn_empty = isinstance(navfn, dict) and (
        _as_float(navfn.get("poses"), 0.0) or 0.0
    ) == 0.0
    teb_selected = _command_pair(
        teb.get("selected_velocity") if isinstance(teb, dict) else None
    )
    teb_status = str(sample.get("teb_status", "") or "").strip().lower()
    # A zero command is not, by itself, a controller failure.  When Navfn has
    # no path and TEB has neither feedback nor a valid trajectory, the zero is
    # the expected consequence of the planner boundary.  Keep this identity
    # explicit so a watchdog-triggered ``zero_velocity_stall`` cannot hide the
    # more useful answer: the route was never materialized.
    planner_boundary = (
        navfn_empty
        and teb_status != "trajectory_valid"
        and abs(teb_selected[0]) <= 0.05
        and abs(teb_selected[1]) <= 0.03
        and abs(_command_pair(sample.get("teb_planner_cmd"))[0]) <= 0.05
        and abs(_command_pair(sample.get("teb_planner_cmd"))[1]) <= 0.03
    )
    explicit_no_path = any(token in text for token in (
        "no_path", "no path", "unreachable", "navfn_no_path", "planner_failed",
    ))
    if explicit_no_path or planner_boundary or navfn_empty or (
        isinstance(teb, dict) and sample.get("teb_status") == "no_selected_trajectory"
    ):
        candidates.append(("planner_no_path", "high" if explicit_no_path or planner_boundary else "medium",
                           "Navfn/TEB did not provide a usable route"))

    # An explicit planner failure is stronger than a missing observer pose:
    # startup ordering can hide odometry briefly, while NAVFN_NO_PATH is the
    # controller's direct reason for rejecting this route.
    if (
        (sample.get("pose") is None
         or (_as_float(sample.get("distance_transform_failures"), 0.0) or 0.0) > 0)
        and not explicit_no_path
    ):
        candidates.append(("tf_or_transform", "high", "pose transform unavailable"))

    scan = sample.get("scan") or {}
    clearance = _as_float(scan.get("min"))
    forward_clearance = _as_float(scan.get("forward_min"))
    obstacle_limit = _as_float(sample.get("obstacle_clearance_threshold"), 0.0) or 0.0
    # The all-direction minimum is often a normal corridor side wall.  Only a
    # forward hit is strong enough to call the route locally blocked; the raw
    # minimum is still retained in the evidence for offline inspection.
    local_obstacle = (
        forward_clearance is not None and obstacle_limit > 0.0
        and forward_clearance <= obstacle_limit
    )
    if local_obstacle or any(token in text for token in ("obstacle", "blocked", "collision")):
        candidates.append(("local_obstacle", "high" if local_obstacle else "medium",
                           "laser/costmap indicates a nearby obstacle"))

    if trigger in _ROUTE_INVALIDATION_STALL_TRIGGERS and (
        explicit_route_failure is None
    ):
        candidates.append(
            ("controller_stall", "high", "frontier watchdog invalidated a stalled route")
        )
    elif trigger == "route_invalidated_disconnected" and (
        explicit_route_failure is None
    ):
        candidates.append(
            ("planner_no_path", "high", "frontier route became disconnected")
        )
    elif trigger == "route_invalidated_controller_failure" and (
        explicit_route_failure is None
    ):
        candidates.append(
            ("controller_stall", "medium", "frontier reported controller failure")
        )

    command = sample.get("cmd_vel") or [0.0, 0.0]
    linear = abs(_as_float(command[0], 0.0) or 0.0) if len(command) > 0 else 0.0
    angular = abs(_as_float(command[1], 0.0) or 0.0) if len(command) > 1 else 0.0
    teb_status = str(sample.get("teb_status", ""))
    zero_command = linear <= 0.05 and angular <= 0.03
    # Missing scan data is unknown, not evidence of a clear path. Treating it
    # as clear used to turn a startup sensor gap into a false controller-stall
    # diagnosis.
    clear_path = (
        forward_clearance is not None
        and (
            obstacle_limit <= 0.0
            or forward_clearance > obstacle_limit
        )
    )
    controller_stall = (
        bool(sample.get("bridge_active"))
        and zero_command
        and clear_path
        and not endpoint_terminal_wait
    )
    if trigger in ("zero_velocity_stall", "no_progress_stall") or controller_stall:
        # Retain the controller-stall alternative for auditability, but rank it
        # below the planner boundary when no executable route ever existed.
        stall_confidence = (
            "low" if planner_boundary
            else "high" if trigger.endswith("stall")
            else "medium"
        )
        candidates.append(("controller_stall", stall_confidence,
                           "active route has no effective motion"))

    if not candidates:
        candidates.append(("unknown", "low", "insufficient discriminating evidence"))

    rank = {"high": 3, "medium": 2, "low": 1}
    candidates.sort(key=lambda item: rank[item[1]], reverse=True)
    label, confidence, _reason = candidates[0]
    route_evidence = {
        key: details.get(key)
        for key in (
            "route_id", "route_kind", "reason", "goal", "distance",
            "elapsed", "path_distance", "best_path_distance",
            "best_goal_distance", "odom_detour_distance", "generation",
            "proposal_generation", "odom_novel_cells", "last_progress_signal",
        )
        if details.get(key) is not None
    }
    # Trigger details are sometimes emitted by move_base before the bridge's
    # richer route event.  Use the retained cross-topic route context to fill
    # the same identity fields instead of leaving the compact classification
    # evidence anonymous.
    for key, value in route.items():
        if key not in route_evidence and value is not None:
            route_evidence[key] = value

    return {
        "label": label,
        "confidence": confidence,
        "candidates": [
            {"label": item[0], "confidence": item[1], "reason": item[2]}
            for item in candidates
        ],
        "evidence": {
            "trigger": trigger,
            "route_invalidation_reason": details.get(
                "route_invalidation_reason", details.get("reason")
            ),
            "goal_distance": _finite_or_none(sample.get("distance_to_goal")),
            "scan_min": _finite_or_none(clearance),
            "scan_forward_min": _finite_or_none(forward_clearance),
            "teb_status": teb_status or None,
            "move_base_status": sample.get("move_base_status"),
            "recovery": _json_safe(recovery),
            # Keep the route watchdog's discriminating values beside the
            # sensor evidence. The route context is also stored in the full
            # snapshot, but this compact block makes offline classification
            # useful without reopening the large event timeline.
            "route": _json_safe(route_evidence),
        },
    }


def merge_failure_classifications(initial, at_end):
    """Keep the strongest evidence observed during an episode.

    A failure's trigger sample is often the most informative one.  For
    example, a ``PREEMPTED`` status is accompanied by a route handoff event,
    but that event may be older than the three-second correlation window by
    the time the post-failure snapshot is written.  Reclassifying from only
    the last sample would then turn a known ownership race into ``unknown``.
    Merge both views, prefer the higher-confidence explanation, and keep the
    initial explanation on confidence ties so the trigger evidence cannot be
    silently discarded.
    """
    initial = initial if isinstance(initial, dict) else {}
    at_end = at_end if isinstance(at_end, dict) else {}
    merged = []
    for source, result in (("trigger", initial), ("end", at_end)):
        candidates = result.get("candidates") or []
        if not candidates and result.get("label"):
            candidates = [{
                "label": result.get("label"),
                "confidence": result.get("confidence", "low"),
                "reason": "classification_%s" % source,
            }]
        for candidate in candidates:
            if not isinstance(candidate, dict):
                continue
            label = str(candidate.get("label", "unknown")) or "unknown"
            confidence = str(candidate.get("confidence", "low"))
            if confidence not in _CONFIDENCE_RANK:
                confidence = "low"
            key = label
            entry = {
                "label": label,
                "confidence": confidence,
                "reason": candidate.get("reason", ""),
                "sources": [source],
            }
            existing = next((item for item in merged if item["label"] == key), None)
            if existing is None:
                merged.append(entry)
            else:
                if _CONFIDENCE_RANK[confidence] > _CONFIDENCE_RANK[existing["confidence"]]:
                    existing["confidence"] = confidence
                    existing["reason"] = entry["reason"]
                if source not in existing["sources"]:
                    existing["sources"].append(source)

    if not merged:
        return at_end or initial or {
            "label": "unknown",
            "confidence": "low",
            "candidates": [{
                "label": "unknown",
                "confidence": "low",
                "reason": "insufficient discriminating evidence",
            }],
        }

    # ``merged`` is ordered trigger-first, so max() is deterministic and
    # retains the trigger explanation when two candidates have equal strength.
    selected = max(
        merged,
        key=lambda item: _CONFIDENCE_RANK.get(item["confidence"], 1),
    )
    ordered = sorted(
        merged,
        key=lambda item: _CONFIDENCE_RANK.get(item["confidence"], 1),
        reverse=True,
    )
    selected_source = initial if "trigger" in selected["sources"] else at_end
    return {
        "label": selected["label"],
        "confidence": selected["confidence"],
        "candidates": [
            {
                "label": item["label"],
                "confidence": item["confidence"],
                "reason": item["reason"],
                "sources": item["sources"],
            }
            for item in ordered
        ],
        "evidence": _json_safe(selected_source.get("evidence", {})),
    }


class NavigationMetricsFailureEvidenceMixin:
    """Maintain a bounded evidence ring and correlated failure episodes."""

    def _initialize_failure_evidence_state(self):
        self.failure_evidence_enabled = self._failure_bool_param(
            "failure_evidence_enabled", True
        )
        self.failure_evidence_period = max(
            0.10, self._failure_float_param("failure_evidence_sample_period", 0.20)
        )
        self.failure_pre_window = max(
            2.0, self._failure_float_param("failure_evidence_pre_window", 12.0)
        )
        self.failure_post_window = max(
            1.0, self._failure_float_param("failure_evidence_post_window", 12.0)
        )
        self.failure_zero_velocity_limit = max(
            1.0, self._failure_float_param("failure_evidence_zero_velocity_seconds", 5.0)
        )
        self.failure_no_progress_limit = max(
            2.0, self._failure_float_param("failure_evidence_no_progress_seconds", 8.0)
        )
        try:
            import rospy
            configured_goal_tolerance = rospy.get_param(
                "/move_base/TebLocalPlannerROS/xy_goal_tolerance", 0.50
            )
        except Exception:
            configured_goal_tolerance = 0.50
        try:
            self.failure_goal_tolerance = max(
                0.05, float(configured_goal_tolerance)
            )
        except (TypeError, ValueError):
            self.failure_goal_tolerance = 0.50
        ring_seconds = self.failure_pre_window + self.failure_post_window + 3.0
        self.failure_evidence_ring = deque(
            maxlen=max(32, int(math.ceil(ring_seconds / self.failure_evidence_period)) + 4)
        )
        self.failure_event_history = deque(maxlen=64)
        self.failure_sequence = 0
        # Every cross-topic context receives a monotonically increasing local
        # sequence.  The sequence is deliberately separate from ROS time:
        # simulated time can pause or jump while callbacks still arrive in a
        # well-defined order.
        self.failure_event_sequence = 0
        self.failure_sample_sequence = 0
        self.failure_count = 0
        self.active_failure = None
        self.failure_history = deque(maxlen=64)
        self.last_failure_id = None
        self.last_failure_summary = None
        self.last_failure_evidence_wall = 0.0
        self.failure_zero_start_wall = None
        self.failure_zero_triggered = False
        self.failure_no_progress_start_wall = None
        self.failure_no_progress_triggered = False
        self.failure_controller_gap_start_wall = None
        self.failure_controller_gap_triggered = False
        self.failure_last_pose_xy = None
        self.failure_last_pose_wall = None
        self.failure_frontier_unavailable_since_wall = None
        self.failure_frontier_unavailable_count = 0
        # ``frontier_route_unavailable`` is a repeated observation of one
        # planning condition. Keep its identity separate from the active
        # episode so the post window cannot reopen it on every timer tick.
        self.failure_frontier_unavailable_key = None
        self.failure_frontier_unavailable_latch = None
        self.failure_frontier_unavailable_latest_reason = None
        self.last_bridge_context = None
        self.last_frontier_context = None
        self.last_goal_context = None
        self.last_action_context = None
        # Last positive route identity observed before a lease release.  The
        # active bridge intentionally reports route_id=0 after cancellation;
        # retaining this compact predecessor lets both periodic samples and
        # failure artifacts identify the physical action that was cancelled.
        self.failure_last_positive_route = None
        self.failure_stream = None
        self.failure_log_path = None
        self.failure_sequence_path = None
        self.failure_sequence_lock_path = None
        # Full episode payloads are written as individual artifacts as well as
        # the append-only evidence log.  The artifact makes one failure
        # inspectable without parsing a potentially very large JSON log line.
        self.failure_artifact_paths = deque(maxlen=64)
        # Warm slices use the same metrics process.  Keep the reset identity
        # so a repeated latched/request retry cannot close and clear the same
        # evidence twice.
        self.failure_last_reset_id = ""
        self.failure_reset_count = 0
        self.failure_last_boundary = None

    def _failure_boundary_has_route_expectation_locked(self, sample):
        """Return whether a route was actually requested before this boundary.

        An idle startup and a temporary graph/candidate wait both legitimately
        have no controller route.  A boundary may only promote
        ``no_active_controller_route`` after a positive route/goal identity or
        a producer event proves that dispatch was expected.
        """
        sample = sample if isinstance(sample, dict) else {}
        if bool(sample.get("bridge_active")):
            return True

        route = _sample_route(sample)
        released = route.get("released_controller_route")
        released = released if isinstance(released, dict) else {}
        if (
            _positive_route_id(released) is not None
            and not bool(released.get("terminal_received"))
        ):
            return True

        expected_events = {
            "route_command", "frontier_action_selected",
            "graph_route_edge_materialized", "dispatch", "active",
            "handoff_requested", "mission_goal", "target_route_accepted",
            "target_segment_committed", "target_route_held",
        }
        normal_terminal_events = {
            "succeeded", "terminal_prefetch_promoted",
            "persistent_execution_terminal", "persistent_frontier_endpoint_terminal",
            "endpoint_action_terminal", "target_approach_terminal",
        }
        failure_terminal_events = {
            "execution_terminal_failure", "frontier_route_failed",
            "portal_hypothesis_failed", "portal_execution_failure_observed",
            "route_invalidated_stall", "route_invalidated_timeout",
            "route_invalidated_controller_failure",
            "controller_stall",
            "graph_route_materialization_invalidated",
            "graph_route_materialization_expired",
        }
        expected = False
        for item in list(getattr(self, "failure_event_history", ()))[-64:]:
            if not isinstance(item, dict):
                continue
            payload = item.get("payload")
            payload = payload if isinstance(payload, dict) else {}
            event = str(item.get("event", "")).strip().lower()
            if event in normal_terminal_events or event in failure_terminal_events:
                expected = False
                continue
            if event == "terminal":
                status = payload.get("status", payload.get("result_status"))
                try:
                    status = int(status)
                except (TypeError, ValueError):
                    status = None
                if status in (3, 4, 5, 8, 9):
                    expected = False
                    continue
            if event == "route_invalidated":
                reason = str(payload.get("reason", "")).strip().lower()
                if reason in {
                    "frontier_observed_at_standoff", "completed", "succeeded",
                    "goal_reached", "observation_complete",
                }:
                    expected = False
                    continue
            if event not in expected_events:
                continue
            route_payload = _nested_route_fields(payload)
            has_route_id = _positive_route_id(route_payload) is not None
            has_goal_transaction = any(
                (_as_float(payload.get(key), 0.0) or 0.0) > 0.0
                for key in (
                    "transaction_id", "goal_transaction_id",
                    "active_goal_transaction_id", "latest_goal_transaction_id",
                )
            )
            if has_route_id or has_goal_transaction:
                expected = True
        return expected

    def _failure_boundary_has_sustained_stall_locked(self):
        """Require watchdog-duration evidence before closing a stall at reset."""
        now = time.monotonic()
        checks = (
            ("failure_zero_start_wall", "failure_zero_velocity_limit"),
            ("failure_no_progress_start_wall", "failure_no_progress_limit"),
        )
        for start_name, limit_name in checks:
            started = getattr(self, start_name, None)
            if started is None:
                continue
            try:
                limit = float(getattr(self, limit_name))
                if limit > 0.0 and now - float(started) >= limit:
                    return True
            except (TypeError, ValueError):
                continue
        for item in reversed(list(getattr(self, "failure_event_history", ()))[-32:]):
            if not isinstance(item, dict):
                continue
            if str(item.get("event", "")).strip().lower() in {
                "controller_stall", "zero_velocity_stall", "no_progress_stall",
                "route_invalidated_stall", "route_invalidated_timeout",
                "route_invalidated_controller_failure",
            }:
                return True
        return False

    def _failure_boundary_trigger_locked(self, sample, boundary):
        """Choose a terminal failure only from explicit boundary evidence.

        A planning wait is not a failure while the run is alive.  At a
        declared slice/trial boundary, however, an unresolved controller
        lease must be classified if the retained evidence proves it lasted
        past the route handoff watchdog.  This keeps ordinary candidate
        misses out of the failure stream while eliminating ambiguous timeout
        records.
        """
        sample = sample if isinstance(sample, dict) else {}
        boundary = boundary if isinstance(boundary, dict) else {}
        if bool(sample.get("task_done")):
            return None
        forced_trigger = str(
            boundary.get("failure_trigger", "") or ""
        ).strip().lower()
        if bool(boundary.get("force_failure")) and forced_trigger in {
            "no_active_controller_route",
        }:
            # The runner's trial-end diagnostic is read from a bounded log
            # tail and can lag the live ROS callbacks.  The metrics sample
            # captured while handling the reset is authoritative; never close
            # a no-route episode when that sample still owns a live route.
            current_route = _sample_route(sample)
            if bool(sample.get("bridge_active")) or (
                _positive_route_id(current_route) is not None
            ):
                self._write(
                    "INFO",
                    "failure_boundary_trigger_suppressed",
                    reason="live_controller_route_present",
                    requested_trigger=forced_trigger,
                    route_id=_positive_route_id(current_route),
                )
                return None
            forced_details = boundary.get("failure_details", {})
            forced_details = (
                dict(forced_details)
                if isinstance(forced_details, dict)
                else {}
            )
            forced_details.setdefault("event", "trial_boundary")
            forced_details.setdefault("reason", forced_trigger)
            forced_details["boundary"] = boundary
            return forced_trigger, forced_details
        # A warm reset is an intentional ownership boundary. Its topic may be
        # delivered to metrics after bridge/frontier have already published
        # reset acknowledgements and replayed latched route state, so those
        # post-reset observations cannot prove a new no-route failure.
        if str(boundary.get("event", "")).strip().lower() == "hard_reset_boundary":
            return None

        route = _sample_route(sample)
        released = route.get("released_controller_route")
        released = released if isinstance(released, dict) else {}
        recent = list(getattr(self, "failure_event_history", ()))
        handoff_timeout = False
        handoff_evidence = []
        for item in reversed(recent[-32:]):
            if not isinstance(item, dict):
                continue
            event = str(item.get("event", "")).strip().lower()
            payload = item.get("payload")
            payload = payload if isinstance(payload, dict) else {}
            if event in {
                "route_lease_watchdog_expired",
                "handoff_timeout",
            }:
                handoff_timeout = True
                handoff_evidence.append(event)
                break
            if event in {
                "handoff_requested",
                "frontier_continuous_prefetch_handoff_requested",
                "frontier_continuous_prefetch_handoff_fallback",
            }:
                for key in ("waited_seconds", "handoff_wait_seconds", "duration_seconds"):
                    try:
                        if float(payload.get(key, 0.0) or 0.0) >= 10.0:
                            handoff_timeout = True
                            handoff_evidence.append(
                                "%s:%s" % (event, key)
                            )
                            break
                    except (TypeError, ValueError):
                        continue
                if handoff_timeout:
                    break

        if (
            bool(released.get("controller_pending"))
            and bool(released.get("terminal_received"))
            and handoff_timeout
        ):
            return "handoff_timeout", {
                "boundary": boundary,
                "handoff_evidence": handoff_evidence,
                "released_controller_route": released,
            }
        if handoff_timeout:
            return "handoff_timeout", {
                "boundary": boundary,
                "handoff_evidence": handoff_evidence,
                "route": route,
            }

        command = _command_pair(sample.get("cmd_vel"))
        zero_command = abs(command[0]) <= 0.05 and abs(command[1]) <= 0.03
        forward_clearance = _as_float(
            (sample.get("scan") or {}).get("forward_min")
            if isinstance(sample.get("scan"), dict) else None
        )
        obstacle_limit = _as_float(
            sample.get("obstacle_clearance_threshold"), 0.0
        ) or 0.0
        distance = _as_float(sample.get("distance_to_goal"))
        active_route = bool(sample.get("bridge_active"))
        if (
            active_route
            and zero_command
            and forward_clearance is not None
            and (obstacle_limit <= 0.0 or forward_clearance > obstacle_limit)
            and (distance is None or distance > 0.80)
            and self._failure_boundary_has_sustained_stall_locked()
        ):
            return "controller_stall", {
                "boundary": boundary,
                "duration_seconds": max(
                    float(getattr(self, "failure_zero_velocity_limit", 0.0)),
                    float(getattr(self, "failure_no_progress_limit", 0.0)),
                ),
                "route": route,
            }

        # A successful action can intentionally leave the bridge inactive for
        # a short successor-selection boundary.  Do not call that a failure
        # unless the retained context also shows that the controller lease
        # was still pending.
        status = str(sample.get("move_base_status", "")).strip().upper()
        if status == "SUCCEEDED" and any(
            str(item.get("event", "")).strip().lower()
            in {"terminal", "endpoint_action_terminal", "persistent_execution_terminal"}
            for item in recent[-16:]
            if isinstance(item, dict)
        ):
            return None
        if (
            not active_route
            and self._failure_boundary_has_route_expectation_locked(sample)
        ):
            boundary_reason = str(
                boundary.get("reason") or boundary.get("event") or ""
            ).strip().lower()
            if boundary_reason == "warm_slice_boundary":
                # A warm boundary intentionally cuts between two lifecycle
                # owners.  The bridge can be inactive for the short interval
                # between predecessor release and successor adoption; the
                # runner's dispatch/adoption gate owns that bounded wait.
                # Keep it as context unless the earlier handoff-timeout path
                # above already proved a sustained lease failure.
                self._write(
                    "INFO",
                    "failure_boundary_trigger_suppressed",
                    reason="warm_slice_successor_handoff_context",
                    requested_trigger="no_active_controller_route",
                )
                return None
            return "no_active_controller_route", {
                "boundary": boundary,
                "route": route,
                "last_bridge_event": getattr(self, "bridge_last_event", ""),
                "last_goal_source": getattr(self, "goal_source", ""),
            }
        return None

    def _close_boundary_failure_locked(self, boundary, sample=None, force=True):
        """Promote one explicit trial/slice boundary into a closed episode."""
        boundary = boundary if isinstance(boundary, dict) else {}
        if sample is None:
            sample = self._failure_append_sample_locked(time.monotonic())
        selected = self._failure_boundary_trigger_locked(sample, boundary)
        self.failure_last_boundary = _json_safe(boundary)
        self._write(
            "INFO",
            "failure_boundary_observed",
            boundary=_json_safe(boundary),
            selected_trigger=None if selected is None else selected[0],
        )
        if selected is not None and self.active_failure is None:
            trigger, details = selected
            details = dict(details)
            details["boundary_reason"] = str(
                boundary.get("reason") or boundary.get("event") or "boundary"
            )
            self._begin_failure_episode_locked(
                trigger,
                "experiment_boundary",
                details,
                sample,
            )
        if self.active_failure is None:
            return None
        return self._finish_failure_episode_locked(
            "boundary:%s" % str(
                boundary.get("reason") or boundary.get("event") or "closed"
            ),
            force=bool(force),
        )

    def on_experiment_boundary(self, message):
        """Close the current trial before the launcher tears down ROS."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        event = str(payload.get("event", "trial_boundary") or "").strip().lower()
        if event not in {"trial_boundary", "experiment_boundary"}:
            return
        with self.lock:
            now = time.monotonic()
            self._failure_record_context_locked("experiment", event, payload)
            self._close_boundary_failure_locked(
                payload,
                sample=self._failure_append_sample_locked(now),
                force=True,
            )

    def on_hard_reset(self, message):
        """Close the slice and discard context before a warm reset is applied."""
        try:
            from experiment_reset_contract import decode_reset_request
            request = decode_reset_request(message)
        except (ImportError, TypeError, ValueError, json.JSONDecodeError):
            request = None
        if request is None:
            return
        reset_id = str(request.get("reset_id", "") or "").strip()
        if not reset_id:
            return
        with self.lock:
            if reset_id == self.failure_last_reset_id:
                return
            had_active_action = bool(
                getattr(self, "bridge_active", False)
                or str(getattr(self, "last_move_base_status", "")).upper()
                in {"ACTIVE", "PENDING", "PREEMPTING"}
            )
            now = time.monotonic()
            boundary = dict(request)
            boundary["event"] = "hard_reset_boundary"
            self._failure_record_context_locked(
                "experiment", "hard_reset_boundary", boundary
            )
            self._close_boundary_failure_locked(
                boundary,
                sample=self._failure_append_sample_locked(now),
                force=True,
            )
            self.failure_last_reset_id = reset_id
            self.failure_reset_count += 1
            self._write(
                "INFO",
                "metrics_hard_reset",
                reset_id=reset_id,
                reset_transaction_id=int(request.get("transaction_id", 0) or 0),
                slice_id=str(request.get("slice_id", "") or ""),
                reason=str(request.get("reason", "slice_boundary")),
                failure_reset_count=int(self.failure_reset_count),
            )
            self._reset_failure_context_locked()
            arm_reset = getattr(
                self, "_arm_hard_reset_preemption_locked", None
            )
            if callable(arm_reset):
                arm_reset(had_active_action)

    def _reset_failure_context_locked(self):
        """Clear slice-local observer context while retaining run history."""
        self.failure_evidence_ring.clear()
        self.failure_event_history.clear()
        self.failure_frontier_unavailable_since_wall = None
        self.failure_frontier_unavailable_count = 0
        self.failure_frontier_unavailable_key = None
        self.failure_frontier_unavailable_latch = None
        self.failure_frontier_unavailable_latest_reason = None
        self.failure_last_positive_route = None
        self.failure_zero_start_wall = None
        self.failure_zero_triggered = False
        self.failure_no_progress_start_wall = None
        self.failure_no_progress_triggered = False
        self.failure_controller_gap_start_wall = None
        self.failure_controller_gap_triggered = False
        self.failure_last_pose_xy = None
        self.failure_last_pose_wall = None
        self.last_bridge_context = None
        self.last_frontier_context = None
        self.last_goal_context = None
        self.last_action_context = None
        self.active_failure = None
        self.last_failure_evidence_wall = 0.0
        self.goal = None
        self.goal_message = None
        self.goal_source = "unknown"
        self.goal_transaction_id = 0
        self.bridge_active = False
        self.bridge_last_event = "hard_reset"
        self.task_done = False
        self.navigation_hold = False
        self.last_move_base_status = "UNKNOWN"
        self.teb_status = "not_available"
        self.move_base_feedback_state = None
        self.teb_feedback_state = None
        self.teb_feedback_wall = None
        self.teb_feedback_valid = False
        self.teb_feedback_invalid_reason = "hard_reset"
        self.teb_feedback_requires_fresh_planner_command = True
        self.teb_feedback_identity = None
        self.teb_feedback_owner_identity = None
        self.planner_contract_identity = None
        self.planner_contract_state = None
        self.planner_contract_validation = "hard_reset"
        self.planner_contract_reason = "hard_reset"
        self.planner_contract_sequence_by_producer = {}
        self.pending_planner_contract = None
        self.terminal_contract_identity = None
        self.terminal_contract_validation = "hard_reset"
        self.pending_terminal_contracts.clear()
        self.bridge_dispatch_contracts.clear()
        self.bridge_latest_dispatch_contract = None
        self.bridge_active_contract = None
        self.recovery_state = None
        self.navfn_plan_stats = None
        self.global_planner_plan_stats = None
        self.teb_global_plan_stats = None
        self.teb_local_plan_stats = None
        for name in ("command", "teb_command", "teb_planner_command"):
            command = getattr(self, name, None)
            if command is None:
                continue
            try:
                command.linear.x = 0.0
                command.angular.z = 0.0
            except AttributeError:
                pass

    def on_teb_goal_failure(self, message):
        """Correlate the bridge's semantic target-route failure immediately."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {"raw": getattr(message, "data", "")}
        if not isinstance(payload, dict):
            payload = {"raw": getattr(message, "data", "")}
        with self.lock:
            event = str(payload.get("event", "target_route_failed"))
            self._failure_record_context_locked("bridge", event, payload)
            self._begin_failure_episode_locked(
                "target_route_failed", "teb_goal_bridge", payload
            )

    @staticmethod
    def _failure_bool_param(name, default):
        try:
            import rospy
            value = rospy.get_param("~" + name, default)
        except Exception:
            value = default
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    @staticmethod
    def _failure_float_param(name, default):
        try:
            import rospy
            return float(rospy.get_param("~" + name, default))
        except Exception:
            return float(default)

    def _failure_existing_sequence(self, run_timestamp=None):
        """Read the highest failure sequence already reserved for this run."""
        log_path = getattr(self, "failure_log_path", None)
        if log_path is None:
            return 0
        run_timestamp = str(
            run_timestamp or getattr(self, "run_timestamp", "run")
        )
        sequence = 0
        for artifact in log_path.parent.glob(
            run_timestamp + "_failure_[0-9][0-9][0-9][0-9].json"
        ):
            try:
                sequence = max(sequence, int(artifact.stem.rsplit("_", 1)[-1]))
            except (IndexError, ValueError):
                continue
        sequence_path = getattr(self, "failure_sequence_path", None)
        persisted_sequence = None
        if sequence_path is not None:
            try:
                persisted_sequence = int(
                    sequence_path.read_text(encoding="utf-8").strip()
                )
                sequence = max(sequence, persisted_sequence)
            except (OSError, TypeError, ValueError):
                pass
        # Existing runs may predate the reservation file. Recover their last
        # ID once so a metrics restart cannot reuse an old episode number.
        if persisted_sequence is None:
            try:
                pattern = re.compile(
                    re.escape(run_timestamp) + r"-F([0-9]+)\b"
                )
                with log_path.open(
                    "r", encoding="utf-8", errors="replace"
                ) as stream:
                    for line in stream:
                        for match in pattern.finditer(line):
                            sequence = max(sequence, int(match.group(1)))
            except (OSError, TypeError, ValueError):
                pass
        return sequence

    def _allocate_failure_sequence_locked(self):
        """Reserve one run-unique failure sequence across process restarts."""
        lock_path = getattr(self, "failure_sequence_lock_path", None)
        sequence_path = getattr(self, "failure_sequence_path", None)
        if lock_path is None or sequence_path is None:
            return None
        lock_fd = None
        temporary = None
        try:
            lock_fd = os.open(str(lock_path), os.O_CREAT | os.O_RDWR, 0o644)
            fcntl.flock(lock_fd, fcntl.LOCK_EX)
            sequence = max(
                int(getattr(self, "failure_sequence", 0) or 0),
                self._failure_existing_sequence(),
            ) + 1
            run_timestamp = str(getattr(self, "run_timestamp", "run"))
            while (
                self.failure_log_path.parent
                / ("%s_failure_%04d.json" % (run_timestamp, sequence))
            ).exists():
                sequence += 1
            temporary = sequence_path.with_name(
                sequence_path.name + ".tmp"
            )
            temporary.write_text("%d\n" % sequence, encoding="utf-8")
            os.replace(str(temporary), str(sequence_path))
            self.failure_sequence = sequence
            return sequence
        except (OSError, TypeError, ValueError):
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass
            return None
        finally:
            if lock_fd is not None:
                try:
                    fcntl.flock(lock_fd, fcntl.LOCK_UN)
                except OSError:
                    pass
                try:
                    os.close(lock_fd)
                except OSError:
                    pass

    def _open_failure_evidence_log(self):
        if not self.failure_evidence_enabled or self.failure_log_path is None:
            return
        try:
            run_timestamp = str(getattr(self, "run_timestamp", "run"))
            existing_artifacts = sorted(
                self.failure_log_path.parent.glob(
                    run_timestamp + "_failure_[0-9][0-9][0-9][0-9].json"
                )
            )
            existing_sequences = []
            for artifact in existing_artifacts:
                try:
                    existing_sequences.append(
                        int(artifact.stem.rsplit("_", 1)[-1])
                    )
                except (IndexError, ValueError):
                    continue
            self.failure_sequence_path = self.failure_log_path.parent / (
                run_timestamp + "_failure_sequence.state"
            )
            self.failure_sequence_lock_path = self.failure_log_path.parent / (
                run_timestamp + "_failure_sequence.lock"
            )
            if existing_sequences:
                self.failure_sequence = max(
                    int(getattr(self, "failure_sequence", 0) or 0),
                    max(existing_sequences),
                )
                self.failure_artifact_paths.extend(
                    str(path) for path in existing_artifacts
                )
            self.failure_sequence = max(
                int(getattr(self, "failure_sequence", 0) or 0),
                self._failure_existing_sequence(run_timestamp),
            )
            log_mode = "a" if self.failure_log_path.exists() else "w"
            self.failure_stream = self.failure_log_path.open(
                log_mode, encoding="utf-8", buffering=1
            )
        except (OSError, AttributeError):
            self.failure_stream = None

    def _close_failure_evidence_log(self):
        try:
            if self.failure_stream is not None:
                self.failure_stream.close()
        except (OSError, ValueError):
            pass
        self.failure_stream = None

    def _write_failure_log(self, level, event, **fields):
        if self.failure_stream is None:
            return
        try:
            import rospy
            fields.setdefault("ros_time", round(rospy.Time.now().to_sec(), 3))
        except Exception:
            fields.setdefault("ros_time", None)
        # A normal context event is intentionally capped at 32 children. A
        # completed snapshot is different: its bounded ring is the actual
        # pre/post evidence and must not be silently truncated to the first
        # 32 samples while serializing the log record.
        max_items = 32
        if event == "failure_snapshot":
            max_items = max(
                64,
                min(256, len(getattr(self, "failure_evidence_ring", ())) + 32),
            )
        if event == "failure_snapshot":
            # Snapshot samples and route contexts have already been bounded at
            # their own roots. Serializing the complete snapshot recursively
            # here would add another wrapper depth and hide those fields.
            safe_fields = self._failure_snapshot_storage_payload(fields)
        else:
            safe_fields = _json_safe(fields, max_items=max_items)
        payload = json.dumps(
            safe_fields, sort_keys=True, separators=(",", ":"), default=str
        )
        line = "%s level=%s process=%s event=%s data=%s\n" % (
            datetime.datetime.now().isoformat(timespec="milliseconds"),
            level,
            getattr(self, "process_name", "lste_navigation_metrics"),
            event,
            payload,
        )
        try:
            self.failure_stream.write(line)
        except (OSError, ValueError):
            pass

    def _write_failure_artifact(self, snapshot):
        """Write one atomic JSON artifact for a closed failure episode.

        This is deliberately owned by the metrics process and stays in the
        invocation's timestamped directory.  A failed write must never alter
        the navigation state machine or prevent the compact log from closing.
        """
        if not isinstance(snapshot, dict):
            return None
        log_path = getattr(self, "failure_log_path", None)
        if log_path is None:
            return None
        try:
            sequence = int(self.failure_sequence)
            run_timestamp = str(getattr(self, "run_timestamp", "run"))
            artifact = log_path.parent / (
                "%s_failure_%04d.json" % (run_timestamp, sequence)
            )
            if artifact.exists():
                self._write_failure_log(
                    "ERROR",
                    "failure_artifact_sequence_collision",
                    failure_id=snapshot.get("failure_id"),
                    artifact_path=str(artifact),
                )
                return None
            temporary = artifact.with_name(artifact.name + ".tmp")
            payload = self._failure_snapshot_storage_payload(snapshot)
            payload["artifact_path"] = str(artifact)
            run_context = getattr(self, "run_context", None)
            if isinstance(run_context, dict):
                payload["run_context"] = _json_safe(
                    run_context,
                    max_items=256,
                    max_string=2048,
                )
            temporary.write_text(
                json.dumps(
                    payload,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            os.replace(str(temporary), str(artifact))
            self.failure_artifact_paths.append(str(artifact))
            snapshot["artifact_path"] = str(artifact)
            return str(artifact)
        except (OSError, OverflowError, TypeError, ValueError) as exc:
            try:
                if "temporary" in locals():
                    temporary.unlink(missing_ok=True)
            except OSError:
                pass
            self._write_failure_log(
                "WARN",
                "failure_artifact_write_failed",
                failure_id=snapshot.get("failure_id"),
                error_type=type(exc).__name__,
            )
            return None

    def _failure_snapshot_storage_payload(self, snapshot):
        """Bound one snapshot by channel without re-traversing its wrappers."""
        if not isinstance(snapshot, dict):
            return {}
        result = {}
        for key, value in snapshot.items():
            if key in {"trigger_sample", "end_sample"}:
                result[key] = self._failure_public_sample(value)
            elif key == "samples" and isinstance(value, (list, tuple, deque)):
                result[key] = [
                    self._failure_public_sample(item)
                    for item in list(value)[:256]
                    if isinstance(item, dict)
                ]
            elif key in {"event_timeline", "related_events"} and isinstance(
                value, (list, tuple, deque)
            ):
                # Event payloads often contain the complete graph status. The
                # route/Place/Portal/WorkItem contract is already projected by
                # ``_compact_failure_details``; retaining that projection
                # avoids a second recursive bound and makes the timeline
                # readable without ``<max-depth>`` placeholders.
                result[key] = [
                    _compact_failure_event(item)
                    for item in list(value)[-64:]
                ]
            elif key in {"route_context_at_trigger", "route_context_at_end"}:
                result[key] = _bounded_route_context(value)
            else:
                # Each field is bounded from its own root. This preserves the
                # compact run context and classification while avoiding a
                # second depth count through sample/route_context wrappers.
                result[key] = _json_safe(value, max_items=256, max_string=2048)
        return result

    def _failure_ros_time(self):
        try:
            import rospy
            return round(float(rospy.Time.now().to_sec()), 3)
        except Exception:
            return None

    @staticmethod
    def _failure_identity_value(value):
        """Normalize route and planner generations for stable comparisons."""
        if value is None:
            return None
        try:
            number = float(value)
            if math.isfinite(number) and number.is_integer():
                return int(number)
        except (TypeError, ValueError):
            pass
        text = str(value).strip()
        return text or None

    @classmethod
    def _failure_frontier_generation(cls, payload):
        """Extract the graph/planner generation from old and new status forms."""
        if not isinstance(payload, dict):
            return None
        for key in (
            "generation",
            "route_generation",
            "action_generation",
            "graph_transaction_id",
        ):
            if payload.get(key) is not None:
                return cls._failure_identity_value(payload.get(key))
        for container_key in (
            "planning_contract",
            "graph_route_plan",
            "graph_route_action_transaction",
        ):
            nested = payload.get(container_key)
            if not isinstance(nested, dict):
                continue
            for key in (
                "generation",
                "route_generation",
                "action_generation",
                "graph_transaction_id",
            ):
                if nested.get(key) is not None:
                    return cls._failure_identity_value(nested.get(key))
        transaction = payload.get("graph_route_action_transaction")
        if isinstance(transaction, dict):
            # ``proposal_generation`` changes on every planning wake. The
            # committed graph-route transaction remains stable while one
            # unavailable route is being retried, so it is the correct
            # fallback generation for this latch.
            if transaction.get("transaction_id") is not None:
                return cls._failure_identity_value(transaction.get("transaction_id"))
        # Older status producers did not expose a graph-route transaction. In
        # that compatibility form proposal generation is the only available
        # generation signal. Prefer the stable transaction above whenever it
        # exists, because proposal generation advances on every planning wake.
        planning_contract = payload.get("planning_contract")
        if isinstance(planning_contract, dict):
            if planning_contract.get("proposal_generation") is not None:
                return cls._failure_identity_value(
                    planning_contract.get("proposal_generation")
                )
        return None

    @classmethod
    def _failure_frontier_unavailable_identity(cls, payload):
        """Return the durable identity of one unavailable route observation."""
        payload = payload if isinstance(payload, dict) else {}
        route_id = payload.get("route_id")
        if route_id is None:
            route_id = payload.get("active_route_id")
        if route_id is None:
            route_id = payload.get("released_route_id")
        reason = str(payload.get("reason", "graph_route_unavailable")).strip()
        reason = _FRONTIER_UNAVAILABLE_REASON_ALIASES.get(reason, reason)
        route_id = cls._failure_identity_value(route_id)
        if route_id is None:
            # Without a durable route identity this is context only.  It must
            # not become a failure episode that an investigator cannot attach
            # to a physical action.
            return None
        return (
            route_id,
            reason or "graph_route_unavailable",
            cls._failure_frontier_generation(payload),
        )

    def _failure_release_frontier_latch_locked(self, reason):
        """Release unavailable state at an explicit navigation boundary."""
        previous = self.failure_frontier_unavailable_latch
        previous_key = self.failure_frontier_unavailable_key
        self.failure_frontier_unavailable_since_wall = None
        self.failure_frontier_unavailable_count = 0
        self.failure_frontier_unavailable_key = None
        self.failure_frontier_unavailable_latch = None
        self.failure_frontier_unavailable_latest_reason = None
        if previous is None:
            return
        fields = {
            "reason": str(reason),
            "failure_id": previous.get("failure_id"),
            "route_identity": _json_safe(previous_key),
        }
        self._write("INFO", "failure_frontier_latch_released", **fields)
        self._write_failure_log("INFO", "failure_frontier_latch_released", **fields)

    def _failure_update_frontier_latch_locked(self, source, event, payload):
        """Track route-unavailable identity and its lifecycle release edges."""
        source = str(source).strip().lower()
        event = str(event).strip().lower()
        if event in _FRONTIER_UNAVAILABLE_EVENTS:
            identity = self._failure_frontier_unavailable_identity(payload)
            raw_reason = str(
                (payload or {}).get("reason", "graph_route_unavailable")
            ).strip() or "graph_route_unavailable"
            if identity != self.failure_frontier_unavailable_key:
                # A changed route/reason/generation is a new planning
                # condition. It invalidates a previous latch without needing
                # a timer or a distance threshold to guess that fact.
                previous = self.failure_frontier_unavailable_latch
                if previous is not None:
                    fields = {
                        "reason": "unavailable_identity_changed",
                        "failure_id": previous.get("failure_id"),
                        "route_identity": _json_safe(
                            self.failure_frontier_unavailable_key
                        ),
                        "replacement_identity": _json_safe(identity),
                    }
                    self._write(
                        "INFO", "failure_frontier_latch_released", **fields
                    )
                    self._write_failure_log(
                        "INFO", "failure_frontier_latch_released", **fields
                    )
                self.failure_frontier_unavailable_key = identity
                self.failure_frontier_unavailable_latch = None
                self.failure_frontier_unavailable_since_wall = time.monotonic()
                self.failure_frontier_unavailable_count = 0
            self.failure_frontier_unavailable_latest_reason = raw_reason
            if self.failure_frontier_unavailable_since_wall is None:
                self.failure_frontier_unavailable_since_wall = time.monotonic()
            self.failure_frontier_unavailable_count += 1
            return

        if event in _FRONTIER_NEW_ROUTE_EVENTS:
            # ``frontier_action_selected`` may be replayed while the same
            # route is waiting for a map snapshot.  It is not a new route
            # identity and must not reopen the episode after its post window.
            if event == "frontier_action_selected":
                try:
                    selected_route_id = int((payload or {}).get("route_id", 0) or 0)
                except (TypeError, ValueError):
                    selected_route_id = 0
                current_key = self.failure_frontier_unavailable_key
                if (
                    current_key is not None
                    and selected_route_id > 0
                    and selected_route_id == current_key[0]
                ):
                    return
            self._failure_release_frontier_latch_locked("new_route:%s" % event)
            return

        if event in _FRONTIER_TERMINAL_EVENTS:
            self._failure_release_frontier_latch_locked("terminal:%s" % event)
            return

        if event in _FRONTIER_RECOVERY_EVENTS:
            self._failure_release_frontier_latch_locked("recovery:%s" % event)
            return

        # Target route transitions are a new owner of the controller even
        # though they arrive on the goal-arbitration stream.
        if source == "goal" and event in {
            "target_route_accepted",
            "target_segment_committed",
            "target_route_released",
            "target_route_failed",
        }:
            self._failure_release_frontier_latch_locked("goal:%s" % event)

    def _failure_next_event_sequence_locked(self):
        """Allocate one bounded-history sequence for a context event."""
        self.failure_event_sequence = int(
            getattr(self, "failure_event_sequence", 0) or 0
        ) + 1
        return self.failure_event_sequence

    @staticmethod
    def _failure_context_source_matches(expected, actual):
        """Match lifecycle source names across the ROS producer boundary."""
        aliases = {
            "frontier": {"frontier", "global_frontier"},
            "bridge": {"bridge", "teb_goal_bridge"},
            "move_base": {"move_base"},
            "goal": {"goal"},
        }
        expected = str(expected or "").strip().lower()
        actual = str(actual or "").strip().lower()
        for canonical, names in aliases.items():
            if expected in names:
                expected = canonical
                break
        if expected == actual:
            return True
        return actual in aliases.get(expected, {expected})

    @classmethod
    def _failure_context_matches_trigger(cls, context, source, trigger, details):
        """Return whether a context is the callback that opened an episode."""
        if not isinstance(context, dict):
            return False
        if not cls._failure_context_source_matches(source, context.get("source")):
            return False
        event = str(context.get("event", "")).strip().lower()
        trigger = str(trigger or "").strip().lower()
        if not event or not trigger:
            return False
        # Most frontier failures preserve the producer event as a prefix of
        # the more specific evidence trigger (for example
        # route_invalidated -> route_invalidated_stall).
        matches = event == trigger or trigger.startswith(event + "_")
        matches = matches or (
            trigger == "unexpected_preemption" and event == "preempted"
        )
        matches = matches or (
            trigger == "move_base_terminal_failure"
            and event in {"aborted", "rejected", "recalled", "lost"}
        )
        matches = matches or (
            trigger == "bridge_terminal" and event == "terminal"
        )
        matches = matches or (
            trigger == "frontier_route_failure"
            and event in {
                "execution_terminal_failure",
                "frontier_route_failed",
                "portal_hypothesis_failed",
                "portal_execution_failure_observed",
            }
        )
        matches = matches or (
            trigger == "target_route_failed"
            and event in {
                "target_route_failed",
                "persistent_target_plan_failed",
                "target_plan_failed",
            }
        )
        if not matches:
            return False
        # If both sides expose a positive route identity, never attach a
        # trigger to a stale event from a neighboring route.
        context_route = _nested_route_fields(context.get("payload"))
        detail_route = _nested_route_fields(details)
        context_id = _positive_route_id(context_route)
        detail_id = _positive_route_id(detail_route)
        return context_id is None or detail_id is None or context_id == detail_id

    def _failure_append_related_context_locked(self, context):
        """Attach one already-recorded context to the active episode once."""
        active = getattr(self, "active_failure", None)
        if not isinstance(active, dict) or not isinstance(context, dict):
            return
        failure_id = str(active.get("failure_id", ""))
        if not failure_id:
            return
        context["failure_id"] = failure_id
        event_sequence = context.get("event_sequence")
        related = active.setdefault("related_events", [])
        if event_sequence is not None and any(
            item.get("event_sequence") == event_sequence
            for item in related
            if isinstance(item, dict)
        ):
            return
        related.append(
            {
                "failure_id": failure_id,
                "event_sequence": event_sequence,
                "source": context.get("source"),
                "event": context.get("event"),
                "wall_elapsed_seconds": context.get("wall_elapsed_seconds"),
                "ros_time": context.get("ros_time"),
                "details": _json_safe(
                    context.get("payload"), max_items=64, max_string=1024
                ),
            }
        )
        # A producer can emit status heartbeats for a long-lived episode.
        # Keep this second index bounded just like failure_event_history.
        del related[:-64]

    def _failure_append_episode_marker_locked(self, event, payload):
        """Add a lifecycle marker to the correlated event timeline."""
        active = getattr(self, "active_failure", None)
        if not isinstance(active, dict):
            return None
        context = {
            "failure_id": str(active.get("failure_id", "")),
            "event_sequence": self._failure_next_event_sequence_locked(),
            "source": "failure_evidence",
            "event": str(event),
            "wall_elapsed_seconds": round(
                max(0.0, time.monotonic() - self.start_wall), 3
            ),
            "ros_time": self._failure_ros_time(),
            "payload": _json_safe(payload, max_items=64, max_string=1024),
        }
        self.failure_event_history.append(context)
        return context

    def _failure_bind_trigger_context_locked(self, failure_id, source, trigger, details):
        """Backfill the ID onto the producer context that opened an episode."""
        history = getattr(self, "failure_event_history", ())
        now_elapsed = max(0.0, time.monotonic() - self.start_wall)
        for context in reversed(history):
            # A previous episode may have used the same producer event. Never
            # rewrite its immutable correlation after that episode closed.
            if context.get("failure_id"):
                continue
            if not self._failure_context_matches_trigger(
                context, source, trigger, details
            ):
                continue
            context_elapsed = context.get("wall_elapsed_seconds")
            try:
                # A timer-promoted episode has no matching producer callback;
                # do not bind a stale context from minutes earlier.
                if (
                    context_elapsed is not None
                    and now_elapsed - float(context_elapsed) > 2.0
                ):
                    continue
            except (TypeError, ValueError):
                pass
            context["failure_id"] = str(failure_id)
            return context
        return None

    def _failure_record_context_locked(self, source, event, payload=None):
        """Keep route state and correlate contexts with the active episode."""
        payload = payload if isinstance(payload, dict) else {}
        # Global-frontier status messages also carry graph reports and work-item
        # lists. The generic JSON bound intentionally truncates those large
        # reports, so restore the causal route fields after bounding the value.
        bounded_payload = _json_safe(payload)
        if isinstance(bounded_payload, dict):
            for key in (
                "reason", "route_id", "active_route_id", "released_route_id",
                "route_kind", "active_route_kind", "released_route_kind", "goal", "distance",
                "elapsed", "path_distance", "best_path_distance",
                "best_goal_distance", "odom_detour_distance", "odom_novel_cells",
                "generation", "proposal_generation",
                "last_progress_signal", "post_turn_elapsed", "post_turn_launched",
                "post_turn_odom_translation", "recovery_behavior", "recovery_reason",
                "region_id", "region_state", "work_item_id", "attempt_id",
                "portal_id", "source_place_id", "destination_place_id",
                "action", "graph_action", "obligation_kind", "obligation_id",
                "portal_path", "first_portal_id", "target_place_id",
                "released_controller_route", "controller_pending", "terminal_received",
                "active_goal_transaction_id", "latest_goal_transaction_id",
                "lifecycle_transaction_id", "action_generation",
                "origin_epoch", "projection_epoch",
                "active_action_generation", "active_route_map_epoch",
                "active_graph_transaction_id", "latest_graph_transaction_id",
                "active_intent_priority", "latest_intent_priority",
                "active_intent_source", "latest_intent_source",
                "transaction_id", "graph_transaction_id",
                "origin_epoch", "projection_epoch",
                "active_graph_transaction_id", "latest_graph_transaction_id",
                "goal_frame", "frame_id", "source",
                "status", "status_name", "failure_count",
                "identity_validation",
                "target_transaction_id",
                "navfn_path_remaining", "navfn_path_endpoint", "navfn_plan_topic",
                "navfn_path_progress_age", "physical_motion_progress_age",
                "active_goal", "active_goal_frame", "active_source_goal",
                "active_source_goal_frame", "active_feedback_frame",
                "last_dispatch_identity", "result_status", "active",
                "target_terminal_boundary_transaction", "target_failure_latched",
                "target_failure_epoch", "target_failure_track_id",
                "target_transaction_id", "controller_lease",
                "target_lease_tombstone_transaction_id",
                "target_lease_tombstone_epoch", "target_lease_tombstone_track_id",
                "next_owner", "lifecycle", "map_epoch", "rejected_map_epoch",
                "last_rejection_reason", "materialization_reason",
            ):
                if key in payload:
                    bounded_payload[key] = _json_safe(payload[key])
            # Keep one compact route identity even when the producer's route
            # fields are nested below a large event graph. This projection is
            # what lets an investigator identify a Portal/WorkItem action
            # without reopening the multi-megabyte metrics stream.
            route_identity = _nested_route_fields(payload)
            if route_identity:
                bounded_payload["route_identity"] = _json_safe(
                    route_identity, max_items=64, max_string=1024
                )
            graph_plan = _compact_graph_route_plan(payload)
            if graph_plan:
                bounded_payload["graph_route_plan"] = graph_plan
            event_graph = payload.get("event_graph")
            if isinstance(event_graph, dict):
                bounded_payload["event_graph"] = {
                    key: _json_safe(event_graph[key])
                    for key in (
                        "active_route_id", "active_route_kind", "place_count",
                        "portal_count", "work_item_count", "structural_revision",
                        "sequence", "last_wake_event",
                    )
                    if key in event_graph
                }
        context = {
            "event_sequence": self._failure_next_event_sequence_locked(),
            "source": str(source),
            "event": str(event),
            "wall_elapsed_seconds": round(
                max(0.0, time.monotonic() - self.start_wall), 3
            ),
            "ros_time": self._failure_ros_time(),
            "payload": bounded_payload,
        }
        self.failure_event_history.append(context)
        # A context arriving after the trigger belongs to this episode and is
        # copied into its direct related-event index.  The top-level ID also
        # survives bounded serialization in the standalone artifact.
        self._failure_append_related_context_locked(context)
        route_fields = _nested_route_fields(payload)
        if route_fields:
            self.failure_last_positive_route = _json_safe(route_fields)
        self._failure_update_frontier_latch_locked(str(source), str(event), payload)
        if source == "bridge":
            self.last_bridge_context = context
        elif source == "frontier":
            self.last_frontier_context = context
        elif source == "goal":
            self.last_goal_context = context
        elif source == "move_base":
            self.last_action_context = context
        return context

    def _failure_route_context_locked(self):
        contexts = {}
        for name, value in (
            ("bridge", getattr(self, "last_bridge_context", None)),
            ("frontier", getattr(self, "last_frontier_context", None)),
            ("goal", getattr(self, "last_goal_context", None)),
            ("move_base", getattr(self, "last_action_context", None)),
        ):
            if value is not None:
                # ``_failure_record_context_locked`` already bounds the
                # producer payload.  Applying ``_json_safe`` to the complete
                # context a second time increments the depth counter through
                # ``context -> payload -> graph status`` and turns the entire
                # payload into ``<max-depth>``.  That left only the compact
                # route projection readable in a failure artifact.  Preserve
                # the bounded payload at its original depth and bound only
                # unexpected metadata supplied by a test/adapter.
                context = dict(value) if isinstance(value, dict) else value
                if isinstance(context, dict) and isinstance(
                    context.get("payload"), dict
                ):
                    context["payload"] = dict(context["payload"])
                    contexts[name] = context
                else:
                    contexts[name] = _json_safe(context)
        route = {}
        for context in contexts.values():
            payload = context.get("payload") if isinstance(context, dict) else None
            if not isinstance(payload, dict):
                continue
            for key in (
                "route_id", "active_route_id", "route_kind", "active_route_kind",
                "mission_route_kind", "place_id", "source_place_id",
                "destination_place_id", "work_item_id", "attempt_id",
                "transaction_id", "graph_transaction_id",
                "lifecycle_transaction_id", "action_generation",
                "origin_epoch", "projection_epoch",
                "active_action_generation", "active_route_map_epoch",
                "active_graph_transaction_id", "latest_graph_transaction_id",
                "target_epoch", "target_track_id",
                "reason", "goal", "distance", "elapsed", "path_distance",
                "best_path_distance", "best_goal_distance", "odom_detour_distance",
                "odom_novel_cells", "generation", "proposal_generation",
                "last_progress_signal", "post_turn_elapsed",
                "post_turn_launched", "post_turn_odom_translation",
                "recovery_behavior", "recovery_reason", "region_id", "region_state",
                "portal_id", "released_controller_route",
                "action", "graph_action", "obligation_kind", "obligation_id",
                "portal_path", "first_portal_id", "target_place_id",
                "active_goal_transaction_id", "latest_goal_transaction_id",
                "active_intent_priority", "latest_intent_priority",
                "active_intent_source", "latest_intent_source",
                "navfn_path_remaining", "navfn_path_endpoint", "navfn_plan_topic",
                "navfn_path_progress_age", "physical_motion_progress_age",
                "active_goal", "active_goal_frame", "active_source_goal",
                "active_source_goal_frame", "active_feedback_frame",
                "last_dispatch_identity", "result_status", "active",
                "target_terminal_boundary_transaction", "target_failure_latched",
                "target_failure_epoch", "target_failure_track_id",
                "target_transaction_id", "controller_lease",
                "target_lease_tombstone_transaction_id",
                "target_lease_tombstone_epoch", "target_lease_tombstone_track_id",
                "next_owner", "lifecycle", "map_epoch", "rejected_map_epoch",
                "last_rejection_reason", "materialization_reason",
            ):
                if key not in payload or payload[key] is None:
                    continue
                value = payload[key]
                # Lease release events intentionally use route_id=0 and an
                # empty route kind. Preserve the preceding causal route when
                # merging independent bridge/frontier callbacks.
                if key in {"route_id", "active_route_id", "released_route_id"}:
                    try:
                        if int(value) <= 0 and route.get(key) not in (None, 0):
                            continue
                    except (TypeError, ValueError):
                        pass
                if key in {
                    "route_kind", "active_route_kind", "released_route_kind",
                } and not str(value).strip() and route.get(key):
                    continue
                route[key] = value
            released_route = payload.get("released_controller_route")
            if isinstance(released_route, dict):
                for key in (
                    "route_id", "route_kind", "graph_transaction_id",
                    "terminal_received",
                    "controller_pending",
                ):
                    if released_route.get(key) is None:
                        continue
                    value = released_route[key]
                    output_key = "released_%s" % key
                    if key == "route_id":
                        try:
                            if int(value) <= 0 and route.get(output_key) not in (None, 0):
                                continue
                        except (TypeError, ValueError):
                            pass
                    if key == "route_kind" and not str(value).strip() and route.get(output_key):
                        continue
                    route[output_key] = value
            # The bounded context carries these two explicit projections when
            # the original graph status was too large to serialize directly.
            # Merge them after the producer's direct fields so action/Portal/
            # WorkItem identity remains available in the final route view.
            for projection_key in ("route_identity", "graph_route_plan"):
                projection = payload.get(projection_key)
                if not isinstance(projection, dict):
                    continue
                for key, value in projection.items():
                    if value is None or route.get(key) not in (None, ""):
                        continue
                    route[key] = value
        # A route-invalidated event commonly arrives after the bridge has
        # cleared its lease and after the frontier status has moved the route
        # fields into an event-specific predecessor.  Fill only missing or
        # non-positive fields from that predecessor; a newer positive route
        # already present in the live contexts remains authoritative.
        predecessor = getattr(self, "failure_last_positive_route", None)
        if isinstance(predecessor, dict):
            current_id = _positive_route_id(route)
            for key, value in predecessor.items():
                if value is None:
                    continue
                if key in _ROUTE_ID_KEYS:
                    if current_id is not None:
                        continue
                    route[key] = value
                    continue
                if key in {"route_kind", "active_route_kind", "released_route_kind"}:
                    if route.get(key) not in (None, ""):
                        continue
                elif key in route and route.get(key) is not None:
                    continue
                route[key] = value
            if _positive_route_id(route) is not None:
                route["route_id"] = _positive_route_id(route)
        return {"route": route, "contexts": contexts}

    def _failure_pair(self, value):
        if value is None:
            return None
        try:
            return [round(float(value[0]), 4), round(float(value[1]), 4)]
        except (TypeError, ValueError, IndexError):
            return None

    def _failure_command(self, command):
        if command is None:
            return [0.0, 0.0]
        try:
            return [round(float(command.linear.x), 4), round(float(command.angular.z), 4)]
        except (AttributeError, TypeError, ValueError):
            try:
                return [round(float(command[0]), 4), round(float(command[1]), 4)]
            except (TypeError, ValueError, IndexError):
                return [0.0, 0.0]

    def _failure_costmap_window_locked(self, message, radius_cells=7):
        """Extract a small robot-centred occupancy patch for failure replay.

        Costmap statistics can say that a map is occupied without saying which
        side of the robot caused the stop.  A 15x15 window is enough to inspect
        a doorway/corner while staying bounded in the 0.2 s evidence stream;
        the full ``OccupancyGrid`` is never copied into a log record.
        """
        if message is None:
            return None
        try:
            info = message.info
            width = int(info.width)
            height = int(info.height)
            resolution = float(info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0.0:
                return None
            origin = info.origin.position
            origin_xy = (float(origin.x), float(origin.y))
            frame = str(getattr(message.header, "frame_id", "") or "odom")
            frame = frame.strip().lstrip("/") or "odom"
            pose = None
            transform = getattr(self, "_pose_xy_in_frame_locked", None)
            if callable(transform):
                try:
                    pose = transform(frame)
                except Exception:
                    pose = None
            if pose is None and frame == "odom":
                pose = getattr(self, "pose", None)
            if not isinstance(pose, (tuple, list)) or len(pose) < 2:
                return None
            world_x = float(pose[0])
            world_y = float(pose[1])
            center_x = int(math.floor((world_x - origin_xy[0]) / resolution))
            center_y = int(math.floor((world_y - origin_xy[1]) / resolution))
            radius = max(1, min(15, int(radius_cells)))
            values = list(message.data)
            rows = []
            for row in range(center_y - radius, center_y + radius + 1):
                cells = []
                for column in range(center_x - radius, center_x + radius + 1):
                    if 0 <= column < width and 0 <= row < height:
                        index = row * width + column
                        cells.append(int(values[index]) if index < len(values) else -1)
                    else:
                        # Outside the published grid is unknown for replay;
                        # keep it distinct from a known free/occupied cell.
                        cells.append(-1)
                rows.append(cells)
            stamp = None
            header_stamp = getattr(message.header, "stamp", None)
            if header_stamp is not None:
                try:
                    stamp = round(float(header_stamp.to_sec()), 3)
                except (AttributeError, TypeError, ValueError):
                    stamp = None
            return {
                "frame": frame,
                "resolution": round(resolution, 4),
                "origin": [round(origin_xy[0], 4), round(origin_xy[1], 4)],
                "center_world": [round(world_x, 4), round(world_y, 4)],
                "center_cell": [center_x, center_y],
                "radius_cells": radius,
                "stamp": stamp,
                "values": rows,
            }
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
            # Diagnostics must be best effort; malformed/partially initialized
            # maps cannot be allowed to affect the navigation node.
            return None

    def _failure_pose_in_goal_frame_locked(self):
        """Return the latest robot pose expressed in the goal frame.

        Frontier and target goals are normally expressed in ``map`` while
        wheel odometry is expressed in ``odom``. Subtracting those coordinates
        directly can report a plausible-looking but wrong failure distance
        after an online-SLAM correction, which makes a snapshot hard to
        localize.
        """
        if getattr(self, "pose", None) is None:
            return None
        frame = str(getattr(self, "goal_frame", "odom") or "odom")
        transform = getattr(self, "_pose_xy_in_frame_locked", None)
        if not callable(transform):
            return None
        try:
            return transform(frame)
        except Exception:
            # The observer must never let a diagnostic transform failure take
            # down the metrics callback thread. The existing pose mixin logs
            # and counts the normal TF exceptions; this catches only a
            # malformed test/adapter implementation.
            return None

    def _failure_distance_to_goal_locked(self, pose_in_goal_frame=None):
        goal = _xy(getattr(self, "goal", None))
        pose = (
            pose_in_goal_frame
            if pose_in_goal_frame is not None
            else self._failure_pose_in_goal_frame_locked()
        )
        if goal is None or not isinstance(pose, (list, tuple)) or len(pose) < 2:
            return None
        try:
            return math.hypot(goal[0] - float(pose[0]), goal[1] - float(pose[1]))
        except (TypeError, ValueError):
            return None

    def _failure_navfn_plan_locked(self, route_context, route_identity):
        """Return raw Navfn evidence or a clearly marked status fallback."""
        raw_plan = getattr(self, "navfn_plan_stats", None)
        if isinstance(raw_plan, dict) and raw_plan:
            result = dict(raw_plan)
            result.setdefault("source", "/move_base/NavfnROS/plan")
            result.setdefault("derived", False)
            return result

        expected_route_id = _positive_route_id(route_identity)
        candidates = []
        for context_name in ("last_bridge_context", "last_frontier_context"):
            context = getattr(self, context_name, None)
            if isinstance(context, dict):
                payload = context.get("payload")
                if isinstance(payload, dict):
                    candidates.append(payload)
        if isinstance(route_context, dict):
            route = route_context.get("route")
            if isinstance(route, dict):
                candidates.append(route)

        for payload in candidates:
            candidate_route_id = _positive_route_id(payload)
            if (
                expected_route_id is not None
                and candidate_route_id is not None
                and candidate_route_id != expected_route_id
            ):
                continue
            summary = _context_navfn_summary(payload)
            if summary is not None:
                return summary
        return None

    def _failure_channel_health(self, sample, now):
        """Describe which causal channels were actually captured at the boundary."""
        sample = sample if isinstance(sample, dict) else {}
        now = float(now)

        def age(attribute):
            stamp = getattr(self, attribute, None)
            if stamp is None:
                return None
            try:
                return round(max(0.0, now - float(stamp)), 4)
            except (TypeError, ValueError):
                return None

        teb_feedback = sample.get("teb_feedback")
        teb_feedback_captured = isinstance(teb_feedback, dict)
        teb_feedback_valid = bool(
            sample.get("teb_feedback_valid", True)
        ) if teb_feedback_captured else False
        channels = {
            "pose": (
                sample.get("pose") is not None,
                "wheel_odom",
                age("last_odom_wall"),
            ),
            "goal": (
                sample.get("goal") is not None,
                "goal_topic",
                age("last_goal_publish_wall"),
            ),
            "cmd_vel": (
                bool(getattr(self, "cmd_messages", 0))
                and sample.get("cmd_vel") is not None,
                "cmd_vel_actuator",
                age("last_cmd_wall"),
            ),
            "scan": (
                isinstance(sample.get("scan"), dict)
                and sample.get("scan", {}).get("min") is not None,
                "laser_scan",
                age("last_scan_wall"),
            ),
            "navfn_plan": (
                isinstance(sample.get("navfn_plan"), dict),
                (sample.get("navfn_plan") or {}).get(
                    "source", "navfn_plan_topic"
                ),
                age("last_navfn_plan_wall"),
            ),
            "teb_feedback": (
                teb_feedback_captured,
                "teb_feedback_topic",
                age("teb_feedback_wall"),
            ),
            "global_costmap": (
                isinstance(sample.get("global_costmap"), dict),
                "global_costmap_topic",
                age("last_global_costmap_wall"),
            ),
            "local_costmap": (
                isinstance(sample.get("local_costmap"), dict),
                "local_costmap_topic",
                age("last_local_costmap_wall"),
            ),
            "move_base_feedback": (
                isinstance(sample.get("move_base_feedback"), dict),
                "move_base_feedback_topic",
                age("last_move_base_feedback_wall"),
            ),
            "route_identity": (
                isinstance(sample.get("route_identity"), dict)
                and bool(_positive_route_id(sample.get("route_identity"))),
                "bridge_frontier_status",
                None,
            ),
        }
        health = {
            name: {
                "available": bool(available),
                "status": "present" if available else "missing",
                "source": str(source),
                "age_seconds": age_seconds,
                "reason": (
                    "observed_at_boundary"
                    if available else "no_value_at_boundary"
                ),
            }
            for name, (available, source, age_seconds) in channels.items()
        }
        # A controller-stall boundary intentionally invalidates the latest
        # identity-free FeedbackMsg.  The retained trajectory summary is still
        # causal evidence, so distinguish "captured but invalid" from a topic
        # that never supplied a value instead of reporting a false channel gap.
        teb_health = health["teb_feedback"]
        teb_health["valid"] = bool(teb_feedback_valid)
        if teb_feedback_captured and not teb_feedback_valid:
            teb_health["status"] = "present_invalid"
            teb_health["reason"] = "invalid_at_boundary"
            teb_health["invalid_reason"] = str(
                sample.get("teb_feedback_invalid_reason", "invalid_feedback")
                or "invalid_feedback"
            )
        elif not teb_feedback_captured:
            teb_health["valid"] = False
        return health

    def _failure_sample_locked(self, now=None):
        now = time.monotonic() if now is None else float(now)
        self.failure_sample_sequence += 1
        pose_in_goal_frame = self._failure_pose_in_goal_frame_locked()
        distance = self._failure_distance_to_goal_locked(pose_in_goal_frame)
        route_context = self._failure_route_context_locked()
        route_identity = _nested_route_fields(route_context)
        navfn_plan = self._failure_navfn_plan_locked(
            route_context, route_identity
        )
        goal_frame = str(getattr(self, "goal_frame", "odom") or "odom")
        command = self._failure_command(getattr(self, "command", None))
        selected_feedback = getattr(self, "teb_feedback_state", None)
        sample = {
            "schema_version": FAILURE_SCHEMA_VERSION,
            "sample_sequence": int(self.failure_sample_sequence),
            "wall_time": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "wall_elapsed_seconds": round(max(0.0, now - self.start_wall), 3),
            "ros_time": self._failure_ros_time(),
            "pose": self._failure_pair(getattr(self, "pose", None)),
            "pose_frame": "odom",
            "goal": self._failure_pair(getattr(self, "goal", None)),
            "goal_frame": goal_frame,
            "pose_goal_frame": (
                None
                if pose_in_goal_frame is None
                else [
                    round(float(pose_in_goal_frame[0]), 4),
                    round(float(pose_in_goal_frame[1]), 4),
                    round(float(pose_in_goal_frame[2]), 4),
                ]
            ),
            "pose_transform_available": pose_in_goal_frame is not None,
            "distance_to_goal": _finite_or_none(distance),
            "teb_xy_goal_tolerance": _finite_or_none(
                getattr(self, "failure_goal_tolerance", None)
            ),
            "goal_source": getattr(self, "goal_source", "unknown"),
            "goal_transaction_id": int(getattr(self, "goal_transaction_id", 0) or 0),
            "subgoal": self._failure_pair(getattr(self, "subgoal", None)),
            "cmd_vel": command,
            "teb_cmd": self._failure_command(getattr(self, "teb_command", None)),
            "teb_planner_cmd": self._failure_command(
                getattr(self, "teb_planner_command", None)
            ),
            "cmd_vel_mux": _json_safe(getattr(self, "cmd_vel_mux_status", None)),
            "scan": {
                "min": _finite_or_none(getattr(self, "scan_minimum", None)),
                "forward_min": _finite_or_none(
                    getattr(self, "scan_forward_minimum", None)
                ),
                "left_min": _finite_or_none(getattr(self, "scan_left_minimum", None)),
                "right_min": _finite_or_none(getattr(self, "scan_right_minimum", None)),
            },
            "obstacle_clearance_threshold": _finite_or_none(
                getattr(self, "discontinuity_obstacle_clearance", None)
            ),
            "navfn_plan": _json_safe(navfn_plan),
            "navfn_path_remaining_m": _finite_or_none(
                (navfn_plan or {}).get(
                    "remaining_estimate_m"
                )
            ),
            "navfn_path_endpoint": _json_safe(
                (navfn_plan or {}).get("endpoint")
            ),
            "global_planner_plan": _json_safe(
                getattr(self, "global_planner_plan_stats", None)
            ),
            "teb_global_plan": _json_safe(getattr(self, "teb_global_plan_stats", None)),
            "teb_local_plan": _json_safe(getattr(self, "teb_local_plan_stats", None)),
            "teb_feedback": _json_safe(selected_feedback),
            "teb_feedback_valid": bool(
                getattr(self, "teb_feedback_valid", False)
            ),
            "teb_feedback_invalid_reason": str(
                getattr(self, "teb_feedback_invalid_reason", "no_feedback")
                or "no_feedback"
            ),
            "teb_feedback_requires_fresh_planner_command": bool(
                getattr(
                    self,
                    "teb_feedback_requires_fresh_planner_command",
                    True,
                )
            ),
            "teb_feedback_identity": _json_safe(
                getattr(self, "teb_feedback_identity", None)
            ),
            "teb_feedback_invalidation_count": int(
                getattr(self, "teb_feedback_invalidation_count", 0) or 0
            ),
            "planner_command_contract": _json_safe({
                "identity": getattr(self, "planner_contract_identity", None),
                "state": getattr(self, "planner_contract_state", None),
                "validation": getattr(
                    self, "planner_contract_validation", "unobserved"
                ),
                "reason": getattr(self, "planner_contract_reason", "startup"),
                "pending": bool(
                    getattr(self, "pending_planner_contract", None) is not None
                ),
            }),
            "frontier_execution_terminal_contract": _json_safe({
                "identity": getattr(self, "terminal_contract_identity", None),
                "validation": getattr(
                    self, "terminal_contract_validation", "unobserved"
                ),
                "count": int(
                    getattr(self, "terminal_contract_count", 0) or 0
                ),
                "pending": len(
                    getattr(self, "pending_terminal_contracts", ())
                ),
            }),
            "teb_turn_supervisor": _json_safe(
                getattr(self, "teb_turn_supervisor_status", None)
            ),
            "teb_turn_supervisor_last_event": getattr(
                self, "teb_turn_supervisor_last_event", "unknown"
            ),
            "teb_turn_supervisor_events": int(
                getattr(self, "teb_turn_supervisor_events", 0) or 0
            ),
            "teb_feedback_age_seconds": (
                None
                if getattr(self, "teb_feedback_wall", None) is None
                else round(max(0.0, now - self.teb_feedback_wall), 4)
            ),
            "mux_status_age_seconds": (
                None
                if getattr(self, "cmd_vel_mux_status_wall", None) is None
                else round(max(0.0, now - self.cmd_vel_mux_status_wall), 4)
            ),
            "teb_status": getattr(self, "teb_status", "not_available"),
            "global_costmap": _json_safe(
                getattr(self, "global_costmap_stats", None)
            ),
            "local_costmap": _json_safe(getattr(self, "local_costmap_stats", None)),
            "global_costmap_window": _json_safe(
                self._failure_costmap_window_locked(
                    getattr(self, "global_costmap_message", None)
                )
            ),
            "local_costmap_window": _json_safe(
                self._failure_costmap_window_locked(
                    getattr(self, "local_costmap_message", None)
                )
            ),
            "move_base_feedback": _json_safe(
                getattr(self, "move_base_feedback_state", None)
            ),
            "move_base_status": getattr(self, "last_move_base_status", "UNKNOWN"),
            "recovery": _json_safe(getattr(self, "recovery_state", None)),
            "bridge_active": bool(getattr(self, "bridge_active", False)),
            "controller": {
                "mode": getattr(self, "controller_mode", "unknown"),
                "source": getattr(self, "controller_source", "unknown"),
                "reason": getattr(self, "controller_reason", "unknown"),
                "requested": _json_safe(getattr(self, "controller_requested", None)),
                "action": _json_safe(getattr(self, "controller_action", None)),
                "predicted_clearance": _finite_or_none(
                    getattr(self, "controller_predicted_clearance", None)
                ),
            },
            "state": getattr(self, "state", "unknown"),
            "task_done": bool(getattr(self, "task_done", False)),
            "navigation_hold": bool(getattr(self, "navigation_hold", False)),
            "distance_transform_failures": int(
                getattr(self, "distance_transform_failures", 0) or 0
            ),
            "route_context": route_context,
            # The full producer context is useful for replay but can be deeply
            # nested. Keep route/Place/Portal/WorkItem identity flat so the
            # bounded JSON serializer cannot replace the causal id with a
            # max-depth marker.
            "route_identity": _json_safe(route_identity, max_items=64),
            "bridge_target_failure_latched": bool(
                isinstance(getattr(self, "last_bridge_context", None), dict)
                and isinstance(
                    getattr(self, "last_bridge_context", {}).get("payload"), dict
                )
                and getattr(self, "last_bridge_context", {}).get("payload", {}).get(
                    "target_failure_latched", False
                )
            ),
            "frontier_route_unavailable_count": int(
                self.failure_frontier_unavailable_count
            ),
            "frontier_route_unavailable_identity": _json_safe(
                self.failure_frontier_unavailable_key
            ),
            "frontier_route_unavailable_latch": _json_safe(
                self.failure_frontier_unavailable_latch
            ),
            "frontier_route_unavailable_duration_seconds": (
                None
                if self.failure_frontier_unavailable_since_wall is None
                else round(
                    max(0.0, now - self.failure_frontier_unavailable_since_wall), 3
                )
            ),
        }
        sample["controller_output_gap"] = _controller_output_gap(
            sample,
            getattr(self, "failure_goal_tolerance", 0.50),
        )
        sample["channel_health"] = self._failure_channel_health(sample, now)
        # The private monotonic value is used only to select the pre/post
        # window. It is removed before the snapshot is serialized.
        sample["_wall_monotonic"] = now
        return sample

    @staticmethod
    def _failure_public_sample(sample):
        result = dict(sample)
        result.pop("_wall_monotonic", None)
        # ``route_context`` and command/costmap fields are intentionally near
        # the end of the sample schema.  The generic 32-item bound used for
        # lifecycle payloads would silently drop them and make a failure look
        # like an unexplained PREEMPTED event.  Samples are already bounded by
        # the ring and each nested value by ``_json_safe``; retain all of the
        # compact top-level channels here.
        route_context = result.pop("route_context", None)
        bounded = _json_safe(result, max_items=256, max_string=2048)
        if route_context is not None:
            # Keep this nested channel readable for post-mortem inspection;
            # the generic sample bound would otherwise count the wrapper depth
            # and replace every producer payload with ``<max-depth>``.
            bounded["route_context"] = _bounded_route_context(route_context)
        return bounded

    def _failure_trigger_sample_locked(self, sample, source, trigger, details):
        """Bind producer fields to the sample captured at a failure boundary.

        A producer callback can release a route and publish its replacement
        before the periodic evidence timer runs again. The ring sample is
        still valuable pre/post context, but it is no longer the state that
        caused the failure. Keep the ring immutable and overlay the callback
        identity on a separate trigger sample so the artifact answers which
        goal, transaction, and planner result failed.
        """
        if not isinstance(sample, dict):
            return sample
        details = details if isinstance(details, dict) else {}
        result = dict(sample)

        event_goal = None
        for key in ("goal", "target_goal", "command_goal", "mission_goal"):
            candidate = self._failure_pair(details.get(key))
            if candidate is not None:
                event_goal = candidate
                break
        if event_goal is not None:
            result["goal"] = event_goal
        if str(trigger).strip().lower() in _TARGET_ROUTE_FAILURE_TRIGGERS:
            result["goal_source"] = (
                details.get("goal_source")
                or details.get("source")
                or "target_route"
            )

        event_frame = str(
            details.get("goal_frame")
            or details.get("frame_id")
            or result.get("goal_frame")
            or "odom"
        ).strip() or "odom"
        result["goal_frame"] = event_frame

        for key in ("transaction_id", "target_transaction_id", "goal_transaction_id"):
            value = details.get(key)
            try:
                value = int(value)
            except (TypeError, ValueError):
                continue
            if value > 0:
                result["goal_transaction_id"] = value
                break

        # Recompute the distance only when the pose is expressed in the same
        # frame as the callback goal. Never turn a frame mismatch into a
        # fabricated geometric measurement.
        pose = result.get("pose_goal_frame")
        if event_goal is not None and isinstance(pose, (list, tuple)) and len(pose) >= 2:
            try:
                result["distance_to_goal"] = round(
                    math.hypot(
                        float(event_goal[0]) - float(pose[0]),
                        float(event_goal[1]) - float(pose[1]),
                    ),
                    4,
                )
            except (TypeError, ValueError):
                pass

        route_identity = _nested_route_fields(details)
        if route_identity:
            existing_identity = result.get("route_identity")
            merged_identity = (
                dict(existing_identity)
                if isinstance(existing_identity, dict) else {}
            )
            for key, value in route_identity.items():
                if value is not None:
                    merged_identity[key] = _json_safe(value)
            result["route_identity"] = _normalize_failure_route_identity(
                merged_identity, trigger, details
            )
        elif result.get("route_identity"):
            result["route_identity"] = _normalize_failure_route_identity(
                result["route_identity"], trigger, details
            )

        result["failure_boundary"] = {
            "source": str(source),
            "trigger": str(trigger),
            "event": str(details.get("event", trigger)),
            "goal": event_goal,
            "goal_frame": event_frame,
            "transaction_id": result.get("goal_transaction_id"),
            "status": details.get("status") or details.get("status_name"),
            "reason": details.get("reason") or details.get("failure_reason"),
            "route": _json_safe(route_identity),
            "details": _compact_failure_details(details),
        }
        if str(trigger).strip().lower() in _TARGET_ROUTE_FAILURE_TRIGGERS:
            result["target_plan_result"] = {
                "status": details.get("status") or details.get("status_name"),
                "reason": details.get("reason") or details.get("failure_reason"),
                "transaction_id": result.get("goal_transaction_id"),
                "target_epoch": details.get("target_epoch"),
                "target_track_id": details.get("target_track_id"),
                "viewpoint_candidate_id": details.get(
                    "target_viewpoint_candidate_id"
                ),
                "viewpoint_attempt_id": details.get(
                    "target_viewpoint_attempt_id"
                ),
            }
        return result

    def _failure_append_sample_locked(self, now=None):
        sample = self._failure_sample_locked(now)
        self.failure_evidence_ring.append(sample)
        self.last_failure_evidence_wall = sample["_wall_monotonic"]
        return sample

    def on_failure_evidence_sample(self, _event=None):
        """Timer callback: sample state and promote sustained stalls to episodes."""
        if not self.failure_evidence_enabled:
            return
        with self.lock:
            now = time.monotonic()
            if (
                self.last_failure_evidence_wall
                and now - self.last_failure_evidence_wall < self.failure_evidence_period * 0.5
            ):
                return
            sample = self._failure_append_sample_locked(now)
            command = sample["cmd_vel"]
            navigation_hold = bool(sample.get("navigation_hold"))
            if navigation_hold:
                sample["failure_watchdog_suppressed"] = {
                    "reason": "navigation_hold",
                    "detail": "intentional observation hold",
                }
            zero_command = abs(command[0]) <= 0.05 and abs(command[1]) <= 0.03
            active_route = bool(sample["bridge_active"]) and not sample["task_done"]
            goal_far = (
                sample["distance_to_goal"] is None
                or sample["distance_to_goal"] >= 0.80
            )
            clear_path = (
                sample["scan"]["forward_min"] is not None
                and sample["scan"]["forward_min"]
                > float(sample["obstacle_clearance_threshold"] or 0.0)
            )
            feedback_boundary = (
                not bool(sample.get("teb_feedback_valid", True))
                and str(
                    sample.get("teb_feedback_invalid_reason", "") or ""
                ).strip().lower()
                in _TEB_FEEDBACK_BOUNDARY_REASONS
            )
            if (
                active_route
                and zero_command
                and goal_far
                and clear_path
                and not feedback_boundary
                and not navigation_hold
            ):
                if self.failure_zero_start_wall is None:
                    self.failure_zero_start_wall = now
                zero_seconds = now - self.failure_zero_start_wall
                sample["zero_command_duration_seconds"] = round(zero_seconds, 3)
                if (
                    zero_seconds >= self.failure_zero_velocity_limit
                    and not self.failure_zero_triggered
                ):
                    self.failure_zero_triggered = True
                    self._begin_failure_episode_locked(
                        "zero_velocity_stall",
                        "metrics_watchdog",
                        {"duration_seconds": round(zero_seconds, 3)},
                        sample,
                    )
            else:
                self.failure_zero_start_wall = None
                self.failure_zero_triggered = False

            # A forward-only base can receive a non-zero angular command while
            # TEB's small reverse sample is removed by the mux.  This is not
            # covered by the generic all-zero watchdog, but it is a bounded
            # failure when the goal is still outside the controller's own XY
            # success envelope.  Use the explicit mux reason and TEB feedback
            # as the evidence contract; do not infer it from a guessed timer.
            controller_gap = (
                active_route
                and _controller_output_gap(
                    sample,
                    getattr(self, "failure_goal_tolerance", 0.50),
                )
                and clear_path
                and not navigation_hold
            )
            if controller_gap:
                if self.failure_controller_gap_start_wall is None:
                    self.failure_controller_gap_start_wall = now
                gap_seconds = now - self.failure_controller_gap_start_wall
                sample["controller_output_gap_duration_seconds"] = round(
                    gap_seconds, 3
                )
                if (
                    gap_seconds >= self.failure_zero_velocity_limit
                    and not self.failure_controller_gap_triggered
                ):
                    self.failure_controller_gap_triggered = True
                    self._begin_failure_episode_locked(
                        "controller_output_gap",
                        "metrics_watchdog",
                        {
                            "duration_seconds": round(gap_seconds, 3),
                            "goal_tolerance": round(
                                float(getattr(self, "failure_goal_tolerance", 0.50)),
                                3,
                            ),
                            "mux_filter_reason": str(
                                (sample.get("cmd_vel_mux") or {}).get(
                                    "filter_reason", ""
                                )
                            ),
                        },
                        sample,
                    )
            else:
                self.failure_controller_gap_start_wall = None
                self.failure_controller_gap_triggered = False

            pose_xy = _xy(sample.get("pose"))
            if pose_xy is not None:
                if self.failure_last_pose_xy is None:
                    self.failure_last_pose_xy = pose_xy
                    self.failure_last_pose_wall = now
                moved = math.hypot(
                    pose_xy[0] - self.failure_last_pose_xy[0],
                    pose_xy[1] - self.failure_last_pose_xy[1],
                )
                expected_motion = (
                    active_route
                    and abs(command[0]) > 0.08
                    and not navigation_hold
                )
                if moved >= 0.04:
                    self.failure_last_pose_xy = pose_xy
                    self.failure_last_pose_wall = now
                    self.failure_no_progress_start_wall = None
                    self.failure_no_progress_triggered = False
                elif expected_motion:
                    if self.failure_no_progress_start_wall is None:
                        self.failure_no_progress_start_wall = now
                    no_progress_seconds = now - self.failure_no_progress_start_wall
                    sample["no_progress_duration_seconds"] = round(no_progress_seconds, 3)
                    if (
                        no_progress_seconds >= self.failure_no_progress_limit
                        and not self.failure_no_progress_triggered
                    ):
                        self.failure_no_progress_triggered = True
                        self._begin_failure_episode_locked(
                            "no_progress_stall",
                            "metrics_watchdog",
                            {"duration_seconds": round(no_progress_seconds, 3)},
                            sample,
                        )
                else:
                    self.failure_no_progress_start_wall = None
                    self.failure_no_progress_triggered = False

            # A failed target can leave the bridge idle while GlobalFrontier
            # repeatedly fails to materialize the same durable obligation.
            # Promote that cross-node lease wait only after it persists; one
            # transient candidate miss remains a normal planning event.
            unavailable_since = self.failure_frontier_unavailable_since_wall
            unavailable_key = self.failure_frontier_unavailable_key
            if unavailable_since is not None and unavailable_key is None:
                # Keep adapters that populated the pre-latch fields directly
                # (and old replay fixtures) compatible. Production callbacks
                # always set the identity when they receive the event.
                context_payload = {}
                context = getattr(self, "last_frontier_context", None)
                if isinstance(context, dict) and isinstance(
                    context.get("payload"), dict
                ):
                    context_payload = context["payload"]
                unavailable_key = self._failure_frontier_unavailable_identity(
                    context_payload
                )
                self.failure_frontier_unavailable_key = unavailable_key
            latched_key = (
                None
                if self.failure_frontier_unavailable_latch is None
                else self.failure_frontier_unavailable_latch.get("key")
            )
            target_failure_latched = bool(sample["bridge_target_failure_latched"])
            released_route = _sample_route(sample).get(
                "released_controller_route"
            )
            released_route = (
                released_route if isinstance(released_route, dict) else {}
            )
            controller_command = sample.get("cmd_vel") or (0.0, 0.0)
            try:
                controller_linear = abs(float(controller_command[0]))
                controller_angular = abs(float(controller_command[1]))
            except (IndexError, TypeError, ValueError):
                controller_linear = 0.0
                controller_angular = 0.0
            controller_route_moving = bool(
                sample.get("bridge_active")
                and str(sample.get("move_base_status", "")).upper()
                in {"ACTIVE", "PENDING", "PREEMPTING"}
                and bool(sample.get("teb_feedback_valid", True))
                and str(sample.get("teb_status", "")) == "trajectory_valid"
                and (
                    controller_linear > 0.08
                    or controller_angular > 0.05
                )
                and not bool(released_route.get("terminal_received"))
            )
            if (
                self.active_failure is None
                and unavailable_since is not None
                and unavailable_key is not None
                and unavailable_key != latched_key
                and now - unavailable_since >= self.failure_no_progress_limit
                and (
                    target_failure_latched
                    or self.failure_frontier_unavailable_count >= 3
                )
                # ``frontier_route_unavailable`` is a graph-lease observation.
                # Do not call it a planner failure while the old controller
                # route still has a valid TEB trajectory and is moving; the
                # no-progress/zero-command watchdogs own that boundary.
                and not controller_route_moving
            ):
                failure_id = self._begin_failure_episode_locked(
                    "frontier_route_stall",
                    "global_frontier",
                    {
                        "route_id": unavailable_key[0],
                        "reason": (
                            self.failure_frontier_unavailable_latest_reason
                            or unavailable_key[1]
                        ),
                        "identity_reason": unavailable_key[1],
                        "generation": unavailable_key[2],
                        "duration_seconds": round(now - unavailable_since, 3),
                        "unavailable_count": int(self.failure_frontier_unavailable_count),
                        "target_failure_latched": target_failure_latched,
                    },
                    sample,
                )
                if failure_id is not None:
                    self.failure_frontier_unavailable_latch = {
                        "key": unavailable_key,
                        "failure_id": str(failure_id),
                        "latched_wall_elapsed_seconds": round(
                            max(0.0, now - self.start_wall), 3
                        ),
                    }

            if self.active_failure is not None and now >= self.active_failure["post_deadline_wall"]:
                self._finish_failure_episode_locked("post_window_complete")

    def _failure_causal_route_locked(self, sample, details, trigger=""):
        """Resolve the route that existed at the failure boundary.

        ``route_invalidated`` is intentionally emitted after the frontier
        lease is released.  Its payload therefore contains a boolean
        ``active`` and a route_id of zero, while the immediately preceding
        ``route_command``/``dispatch`` event contains the useful identity.
        Prefer a route embedded in the trigger details, then the positive
        identity cache updated by every cross-topic event, and finally walk
        the event history for compatibility with older producers.
        """
        candidates = []
        direct = _nested_route_fields(details)
        if direct:
            candidates.append((100, direct))
        sample_context = sample.get("route_context") if isinstance(sample, dict) else None
        sample_identity = sample.get("route_identity") if isinstance(sample, dict) else None
        sample_route = _nested_route_fields(sample_identity)
        if sample_route:
            candidates.append((95, sample_route))
        sample_route = _nested_route_fields(sample_context)
        if sample_route:
            candidates.append((90, sample_route))
        if isinstance(self.failure_last_positive_route, dict):
            candidates.append((80, self.failure_last_positive_route))
        for index, event in enumerate(reversed(self.failure_event_history)):
            if not isinstance(event, dict):
                continue
            payload = event.get("payload")
            route = _nested_route_fields(payload)
            if not route:
                continue
            name = str(event.get("event", "")).strip().lower()
            # Route-command and dispatch records are the strongest historical
            # anchors; generic status records remain useful as a fallback.
            event_weight = 70 if name in _FRONTIER_NEW_ROUTE_EVENTS else 50
            candidates.append((event_weight - min(index, 20), route))
        if not candidates:
            return {}

        goal = _xy(sample.get("goal")) if isinstance(sample, dict) else None

        def score(item):
            weight, route = item
            route_goal = _xy(
                route.get("goal")
                or route.get("command_goal")
                or route.get("mission_goal")
            )
            goal_match = 0
            if goal is not None and route_goal is not None:
                goal_match = 20 if math.hypot(
                    goal[0] - route_goal[0], goal[1] - route_goal[1]
                ) <= 0.75 else 0
            return weight + goal_match

        selected = dict(max(candidates, key=score)[1])
        return _normalize_failure_route_identity(
            selected,
            trigger or details.get("trigger") or details.get("event"),
            details,
        )

    def _begin_failure_episode_locked(
        self, trigger, source, details=None, sample=None
    ):
        """Start or annotate one failure episode without duplicate IDs."""
        if not self.failure_evidence_enabled:
            return None
        now = time.monotonic()
        details = details if isinstance(details, dict) else {}
        if self.active_failure is not None:
            failure_id = str(self.active_failure["failure_id"])
            event_sequence = self._failure_next_event_sequence_locked()
            related = self.active_failure.setdefault("related_events", [])
            related.append({
                "failure_id": failure_id,
                "event_sequence": event_sequence,
                "source": str(source),
                "event": str(trigger),
                "wall_elapsed_seconds": round(max(0.0, now - self.start_wall), 3),
                "ros_time": self._failure_ros_time(),
                "details": _json_safe(details),
            })
            del related[:-64]
            self._failure_append_episode_marker_locked(
                "failure_related_event",
                {
                    "source": str(source),
                    "event": str(trigger),
                    "details": _json_safe(details),
                },
            )
            self._write(
                "WARN",
                "failure_related_event",
                failure_id=failure_id,
                source=str(source),
                trigger=str(trigger),
                details=_json_safe(details),
            )
            self._write_failure_log(
                "WARN",
                "failure_related_event",
                failure_id=self.active_failure["failure_id"],
                source=str(source),
                trigger=str(trigger),
                details=_json_safe(details),
            )
            return self.active_failure["failure_id"]

        if sample is None:
            sample = self._failure_append_sample_locked(now)
        causal_route = self._failure_causal_route_locked(sample, details, trigger)
        if causal_route:
            # Promote the stable route fields into the trigger details before
            # classification.  This makes the compact summary useful without
            # requiring an investigator to parse the full event timeline.
            enriched = dict(details)
            for key, value in causal_route.items():
                if value is None:
                    continue
                if key in _ROUTE_ID_KEYS:
                    current = _positive_route_id(enriched)
                    if current is None:
                        enriched[key] = value
                    continue
                if key in {"route_kind", "active_route_kind", "released_route_kind"}:
                    if not str(enriched.get(key, "") or "").strip():
                        enriched[key] = value
                    continue
                if key not in enriched or enriched.get(key) is None:
                    enriched[key] = value
            details = enriched
        # Producer callbacks can already have advanced the live goal/route by
        # the time the timer sample is inspected. Bind the exact callback
        # payload to a separate trigger sample before classification.
        trigger_sample = self._failure_trigger_sample_locked(
            sample, source, trigger, details
        )
        causal_identity = _failure_causal_identity(
            causal_route,
            sample=trigger_sample,
            details=details,
        )
        sequence = self._allocate_failure_sequence_locked()
        if sequence is None:
            self._write_failure_log(
                "ERROR",
                "failure_sequence_reservation_failed",
                trigger=str(trigger),
                source=str(source),
            )
            return None
        failure_id = "%s-F%04d" % (
            getattr(self, "run_timestamp", "run"), sequence
        )
        boundary = trigger_sample.get("failure_boundary")
        if isinstance(boundary, dict):
            boundary["failure_id"] = failure_id
        recent_events = list(self.failure_event_history)[-16:]
        classification = classify_failure(
            trigger, trigger_sample, details=details, recent_events=recent_events
        )
        causal_route = {}
        if isinstance(classification, dict):
            evidence = classification.get("evidence")
            if isinstance(evidence, dict) and isinstance(evidence.get("route"), dict):
                causal_route = dict(evidence["route"])
        causal_route = _normalize_failure_route_identity(
            causal_route, trigger, details
        )
        causal_identity = _failure_causal_identity(
            causal_route,
            sample=trigger_sample,
            details=details,
        )
        for key, value in causal_identity.items():
            causal_route[key] = value
        # Preserve the route identity that existed when the trigger was
        # classified.  A route-invalidated callback can clear the active lease
        # before the next metrics sample, so the later post-window sample is
        # not a reliable source for the causal route.
        if causal_route:
            enriched_details = dict(details)
            for key, value in causal_route.items():
                if value is None:
                    continue
                if key in _ROUTE_ID_KEYS:
                    current = _positive_route_id(enriched_details)
                    if current is None:
                        enriched_details[key] = value
                    continue
                if key in {"route_kind", "active_route_kind", "released_route_kind"}:
                    if not str(enriched_details.get(key, "") or "").strip():
                        enriched_details[key] = value
                    continue
                if key not in enriched_details or enriched_details.get(key) is None:
                    enriched_details[key] = value
            details = enriched_details
        active = {
            "failure_id": failure_id,
            "trigger": str(trigger),
            "source": str(source),
            "details": _compact_failure_details(details),
            "started_wall": now,
            "started_wall_elapsed_seconds": round(max(0.0, now - self.start_wall), 3),
            "started_ros_time": trigger_sample.get("ros_time"),
            "post_deadline_wall": now + self.failure_post_window,
            "classification_initial": classification,
            "causal_route": _json_safe(causal_route),
            "causal_identity": _json_safe(causal_identity),
            "trigger_sample": trigger_sample,
            "related_events": [],
        }
        self.active_failure = active
        # The producer callback normally records its context immediately
        # before this method is called. Bind that latest matching context to
        # the new ID so the trigger itself is searchable without timestamp
        # heuristics. Timer-promoted failures simply have no matching context.
        self._failure_bind_trigger_context_locked(
            failure_id, source, trigger, details
        )
        self._failure_append_episode_marker_locked(
            "failure_started",
            {
                "trigger": str(trigger),
                "source": str(source),
                "details": _compact_failure_details(details),
            },
        )
        self.failure_count += 1
        self.last_failure_id = failure_id
        self._write(
            "WARN",
            "failure_started",
            failure_id=failure_id,
            trigger=str(trigger),
            source=str(source),
            classification=classification,
            details=_compact_failure_details(details),
            started_wall_elapsed_seconds=active["started_wall_elapsed_seconds"],
            started_ros_time=active["started_ros_time"],
            pre_window_seconds=self.failure_pre_window,
            post_window_seconds=self.failure_post_window,
            causal_identity=_json_safe(causal_identity),
            **causal_identity,
        )
        self._write_failure_log(
            "WARN",
            "failure_started",
            failure_id=failure_id,
            trigger=str(trigger),
            source=str(source),
            classification=classification,
            details=_compact_failure_details(details),
            started_wall_elapsed_seconds=active["started_wall_elapsed_seconds"],
            started_ros_time=active["started_ros_time"],
            pre_window_seconds=self.failure_pre_window,
            post_window_seconds=self.failure_post_window,
            causal_identity=_json_safe(causal_identity),
            **causal_identity,
        )
        return failure_id

    def _failure_recovery_from_post_window_locked(self, active, final_sample):
        """Return successor-route evidence for a recoverable materialization wait.

        ``frontier_route_stall`` and endpoint zero-velocity stalls can be
        promoted while the executive is between two controller leases.  That
        condition is not a terminal navigation failure when the executive
        installs and activates a successor route during the evidence window.
        The successor may itself reach its endpoint before the window closes,
        so inspect the bounded ring for a healthy successor sample instead of
        requiring the final sample to still own that route.
        """
        if not isinstance(active, dict) or not isinstance(final_sample, dict):
            return None
        trigger = str(active.get("trigger", "") or "").strip().lower()
        if trigger not in _RECOVERABLE_MATERIALIZATION_TRIGGERS:
            return None

        causal_route = active.get("causal_route")
        causal_route = causal_route if isinstance(causal_route, dict) else {}
        failed_route_id = _positive_route_id(causal_route)
        failed_graph_transaction = _as_float(
            causal_route.get("graph_transaction_id")
            or causal_route.get("active_graph_transaction_id")
            or causal_route.get("latest_graph_transaction_id")
        )
        started_elapsed = _as_float(
            active.get("started_wall_elapsed_seconds"), 0.0
        )
        recovery_event = None
        for context in reversed(
            list(getattr(self, "failure_event_history", ()))
        ):
            if not isinstance(context, dict):
                continue
            event = str(context.get("event", "") or "").strip().lower()
            if event not in _RECOVERY_ROUTE_EVENTS:
                continue
            event_elapsed = _as_float(context.get("wall_elapsed_seconds"))
            if event_elapsed is None or event_elapsed <= started_elapsed:
                continue
            payload = context.get("payload")
            payload = payload if isinstance(payload, dict) else {}
            route = _nested_route_fields(payload)
            route_id = _positive_route_id(route)
            if route_id is None:
                continue
            graph_transaction = _as_float(
                route.get("graph_transaction_id")
                or route.get("active_graph_transaction_id")
                or route.get("latest_graph_transaction_id")
            )
            identity_changed = (
                failed_route_id is None
                or route_id != failed_route_id
                or (
                    graph_transaction is not None
                    and failed_graph_transaction is not None
                    and graph_transaction != failed_graph_transaction
                )
            )
            if not identity_changed:
                continue
            recovery_event = {
                "event": event,
                "event_sequence": context.get("event_sequence"),
                "route_id": route_id,
                "route_kind": route.get("route_kind")
                or route.get("active_route_kind")
                or route.get("mission_route_kind"),
                "graph_transaction_id": graph_transaction,
                "wall_elapsed_seconds": event_elapsed,
            }
            break
        if recovery_event is None:
            return None

        recovery_sample = None
        for candidate in reversed(list(self.failure_evidence_ring)):
            if not isinstance(candidate, dict):
                continue
            candidate_elapsed = _as_float(
                candidate.get("wall_elapsed_seconds")
            )
            if candidate_elapsed is None or candidate_elapsed <= started_elapsed:
                continue
            candidate_route = _sample_route(candidate)
            candidate_route_id = _positive_route_id(candidate_route)
            if candidate_route_id != recovery_event["route_id"]:
                continue
            if not bool(candidate.get("bridge_active")):
                continue
            if str(candidate.get("move_base_status", "") or "").upper() not in {
                "ACTIVE", "PENDING", "PREEMPTING"
            }:
                continue
            teb_status = str(
                candidate.get("teb_status", "") or ""
            ).strip().lower()
            command = _command_pair(candidate.get("cmd_vel"))
            healthy_feedback = teb_status == "trajectory_valid"
            healthy_command = abs(command[0]) > 0.05 or abs(command[1]) > 0.03
            if healthy_feedback or healthy_command:
                recovery_sample = candidate
                break
        if recovery_sample is None:
            return None
        recovery_route = _sample_route(recovery_sample)
        recovery_event["recovery_route_id"] = recovery_event["route_id"]
        recovery_event["recovery_route_kind"] = (
            recovery_route.get("route_kind")
            or recovery_route.get("active_route_kind")
        )
        recovery_event["recovery_wall_elapsed_seconds"] = _as_float(
            recovery_sample.get("wall_elapsed_seconds")
        )
        recovery_event["recovery_move_base_status"] = recovery_sample.get(
            "move_base_status"
        )
        recovery_event["recovery_teb_status"] = recovery_sample.get(
            "teb_status"
        )
        return recovery_event

    def _discard_recovered_failure_locked(self, recovery, reason):
        """Close a watchdog candidate as context without publishing a failure.

        The candidate ID remains in the event history for replay, but no
        ``failure_snapshot_ready`` event or JSON artifact is emitted.  The
        experiment runner therefore keeps the live runtime when route recovery
        is proven inside the bounded post window.
        """
        active = self.active_failure
        if not isinstance(active, dict):
            return None
        failure_id = str(active.get("failure_id", "") or "")
        payload = {
            "failure_id": failure_id,
            "trigger": active.get("trigger"),
            "source": active.get("source"),
            "reason": str(reason),
            "recovery": _json_safe(recovery),
        }
        self._failure_append_episode_marker_locked(
            "failure_episode_recovered", payload
        )
        self._write("INFO", "failure_episode_recovered", **payload)
        self._write_failure_log("INFO", "failure_episode_recovered", **payload)
        self._failure_release_frontier_latch_locked("episode_recovered")
        self.active_failure = None
        return payload

    def _finish_failure_episode_locked(self, reason, force=False):
        if self.active_failure is None:
            return None
        now = time.monotonic()
        active = self.active_failure
        if not force and now < active["post_deadline_wall"]:
            return None
        if not self.failure_evidence_ring or (
            self.failure_evidence_ring[-1].get("_wall_monotonic", 0.0) < now
        ):
            self._failure_append_sample_locked(now)
        start = active["started_wall"] - self.failure_pre_window
        end = now
        samples = [
            self._failure_public_sample(sample)
            for sample in self.failure_evidence_ring
            if start <= sample.get("_wall_monotonic", 0.0) <= end
        ]
        final_sample = (
            self.failure_evidence_ring[-1]
            if self.failure_evidence_ring else active["trigger_sample"]
        )
        if not force:
            recovery = self._failure_recovery_from_post_window_locked(
                active, final_sample
            )
            if recovery is not None:
                return self._discard_recovered_failure_locked(
                    recovery, "successor_route_active"
                )
        end_classification = classify_failure(
            active["trigger"],
            final_sample,
            details=active["details"],
            recent_events=list(self.failure_event_history)[-16:],
        )
        final_classification = merge_failure_classifications(
            active["classification_initial"], end_classification
        )
        diagnosis_details = dict(active["details"])
        diagnosis_details.setdefault("trigger", active["trigger"])
        for key, value in (active.get("causal_route") or {}).items():
            if value is not None:
                diagnosis_details.setdefault(key, value)
        diagnosis_at_trigger = diagnose_failure_sample(
            active["trigger_sample"], details=diagnosis_details
        )
        diagnosis_at_end = diagnose_failure_sample(
            final_sample, details=diagnosis_details
        )
        causal_identity = _failure_causal_identity(
            active.get("causal_route"),
            sample=active.get("trigger_sample"),
            details=active.get("details"),
        )
        snapshot = {
            "schema_version": FAILURE_SCHEMA_VERSION,
            "failure_id": active["failure_id"],
            "trigger": active["trigger"],
            "source": active["source"],
            "resolution": str(reason),
            "classification": final_classification,
            "classification_initial": active["classification_initial"],
            "classification_at_end": end_classification,
            "details": active["details"],
            "causal_route": _json_safe(active.get("causal_route")),
            "causal_identity": _json_safe(causal_identity),
            "started_wall_elapsed_seconds": active["started_wall_elapsed_seconds"],
            "ended_wall_elapsed_seconds": round(max(0.0, now - self.start_wall), 3),
            "started_ros_time": active["started_ros_time"],
            "ended_ros_time": final_sample.get("ros_time"),
            "pre_window_seconds": self.failure_pre_window,
            "post_window_seconds": round(
                max(0.0, now - active["started_wall"]), 3
            ),
            "route_context_at_trigger": active["trigger_sample"].get("route_context"),
            "route_context_at_end": final_sample.get("route_context"),
            # Keep the two causally important samples directly addressable.
            # The full ring remains available below, but an investigator
            # should not have to guess which of its samples is the trigger.
            "trigger_sample": self._failure_public_sample(active["trigger_sample"]),
            "end_sample": self._failure_public_sample(final_sample),
            # The trigger sample is the causal boundary.  The robot may have
            # recovered or moved to a successor route during the post window,
            # so keep its diagnosis separate from the end-state diagnosis
            # instead of letting a later healthy command hide the failure.
            "diagnosis": diagnosis_at_trigger,
            "diagnosis_at_trigger": diagnosis_at_trigger,
            "diagnosis_at_end": diagnosis_at_end,
            "related_events": _json_safe(active["related_events"]),
            "event_timeline": _json_safe(list(self.failure_event_history)[-32:]),
            "sample_count": len(samples),
            "samples": samples,
        }
        snapshot.update(causal_identity)
        run_context = getattr(self, "run_context", None)
        if isinstance(run_context, dict):
            # Keep the immutable startup contract in the append-only event as
            # well as the standalone artifact.  This preserves localization
            # when a filesystem interruption leaves only the log line.
            snapshot["run_context"] = _json_safe(
                run_context,
                max_items=256,
                max_string=2048,
            )
        artifact_path = self._write_failure_artifact(snapshot)
        if artifact_path is not None:
            snapshot["artifact_path"] = artifact_path
        summary = {
            "failure_id": active["failure_id"],
            "trigger": active["trigger"],
            "source": active["source"],
            "resolution": str(reason),
            "classification": final_classification["label"],
            "confidence": final_classification["confidence"],
            "diagnosis": {
                "primary_cause": diagnosis_at_trigger["primary_cause"],
                "layer": diagnosis_at_trigger["layer"],
                "confidence": diagnosis_at_trigger["confidence"],
            },
            "started_wall_elapsed_seconds": active["started_wall_elapsed_seconds"],
            "ended_wall_elapsed_seconds": snapshot["ended_wall_elapsed_seconds"],
            "duration_seconds": snapshot["post_window_seconds"],
            "sample_count": len(samples),
            "artifact_path": artifact_path,
            "route_id": diagnosis_at_trigger.get("route_id"),
            "route_kind": diagnosis_at_trigger.get("route_kind"),
            "causal_identity": _json_safe(causal_identity),
            "trigger_pose": diagnosis_at_trigger.get("spatial", {}).get("pose"),
            "trigger_goal": diagnosis_at_trigger.get("spatial", {}).get("goal"),
            "command_chain": diagnosis_at_trigger.get("command_chain"),
        }
        summary.update(causal_identity)
        self.failure_history.append(summary)
        self.last_failure_summary = summary
        self._write_failure_log("ERROR", "failure_snapshot", **snapshot)
        # Keep the artifact pointer in the bounded failure stream too. This
        # lets an investigator open the episode directly from the dedicated
        # evidence log without parsing the high-volume metrics stream.
        self._write_failure_log("ERROR", "failure_snapshot_ready", **summary)
        self._write("ERROR", "failure_snapshot_ready", **summary)
        self.active_failure = None
        return summary

    def _failure_snapshot_state(self):
        active = self.active_failure
        return {
            "enabled": bool(self.failure_evidence_enabled),
            "active_failure_id": None if active is None else active["failure_id"],
            "failure_count": int(self.failure_count),
            "last_failure_id": self.last_failure_id,
            "last_failure": _json_safe(self.last_failure_summary),
            "ring_samples": len(self.failure_evidence_ring),
            "frontier_route_unavailable_identity": _json_safe(
                self.failure_frontier_unavailable_key
            ),
            "frontier_route_unavailable_latch": _json_safe(
                self.failure_frontier_unavailable_latch
            ),
            "failure_log_path": (
                None if self.failure_log_path is None else str(self.failure_log_path)
            ),
            "failure_artifacts": list(self.failure_artifact_paths),
            "reset_count": int(getattr(self, "failure_reset_count", 0) or 0),
            "last_reset_id": getattr(self, "failure_last_reset_id", "") or None,
            "last_boundary": _json_safe(
                getattr(self, "failure_last_boundary", None)
            ),
        }
