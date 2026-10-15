#!/usr/bin/env python3
"""Machine-readable status publishing for the TEB goal bridge."""

import json
import time

import rospy
from std_msgs.msg import String

from clock_provider import now_for


class TebGoalBridgeStatusMixin:
    """Build status snapshots without mixing diagnostics into action control."""

    def publish_bridge_status(self, event, **fields):
        now = now_for(self)
        action_contract = getattr(self, "active_action_contract", None)
        if not hasattr(action_contract, "get"):
            pending = getattr(self, "pending_terminal_prepare", None)
            if isinstance(pending, dict):
                action_contract = pending.get("old_contract")
        if not hasattr(action_contract, "get"):
            action_contract = getattr(self, "last_dispatch_identity", None)

        def identity_value(name, *fallbacks):
            if name in fields and fields[name] is not None:
                return fields[name]
            if hasattr(action_contract, "get"):
                value = action_contract.get(name)
                if value is not None:
                    return value
                if name == "action_generation":
                    value = action_contract.get("generation")
                    if value is not None:
                        return value
            for value in fallbacks:
                if value is not None:
                    return value
            return 0

        def identity_int(value):
            try:
                return int(value or 0)
            except (TypeError, ValueError):
                return 0

        status_route_id = identity_int(
            identity_value(
                "route_id",
                getattr(self, "active_route_id", 0),
                getattr(self, "latest_route_id", 0),
            )
        )
        status_action_generation = identity_int(
            identity_value("action_generation", getattr(self, "action_generation", 0))
        )
        status_transaction_id = identity_int(
            identity_value(
                "transaction_id",
                getattr(self, "active_goal_transaction_id", 0),
                getattr(self, "latest_goal_transaction_id", 0),
            )
        )
        status_lifecycle_transaction_id = identity_int(
            identity_value(
                "lifecycle_transaction_id",
                self._canonical_lifecycle_transaction_id(),
            )
        )
        active_action_lifecycle_transaction_id = identity_int(
            action_contract.get("lifecycle_transaction_id", 0)
            if hasattr(action_contract, "get")
            else 0
        )
        if active_action_lifecycle_transaction_id <= 0:
            active_action_lifecycle_transaction_id = status_lifecycle_transaction_id
        status_graph_transaction_id = identity_int(
            identity_value(
                "graph_transaction_id",
                getattr(self, "active_graph_transaction_id", 0),
                getattr(self, "latest_graph_transaction_id", 0),
            )
        )
        status_map_epoch = identity_value(
            "map_epoch",
            getattr(self, "active_route_map_epoch", None),
            getattr(self, "latest_route_map_epoch", None),
            getattr(self, "latest_frontier_map_epoch", None),
        )
        status_origin_epoch = identity_value(
            "origin_epoch",
            getattr(self, "active_origin_epoch", None),
            getattr(self, "latest_origin_epoch", None),
        )
        status_projection_epoch = identity_value(
            "projection_epoch",
            getattr(self, "active_projection_epoch", None),
            getattr(self, "latest_projection_epoch", None),
        )
        transport_pending = getattr(self, "transport_handoff_pending", None)
        transport_pending_contract = (
            transport_pending.get("old_contract", {})
            if isinstance(transport_pending, dict)
            else {}
        )
        payload = {
            "event": str(event),
            # These scalar fields are the canonical correlation surface. The
            # active_* fields below remain useful for comparing old and new
            # ownership during a terminal handoff.
            "route_id": status_route_id,
            "action_generation": status_action_generation,
            "transaction_id": status_transaction_id,
            "lifecycle_transaction_id": status_lifecycle_transaction_id,
            # ``lifecycle_transaction_id`` describes this status event.  The
            # explicit active-action field is the immutable identity that
            # planner/turn callbacks must validate against while a newer
            # semantic route is being adopted on the same persistent action.
            "active_action_lifecycle_transaction_id": (
                active_action_lifecycle_transaction_id
            ),
            "graph_transaction_id": status_graph_transaction_id,
            "map_epoch": status_map_epoch,
            "origin_epoch": status_origin_epoch,
            "projection_epoch": status_projection_epoch,
            "dedupe_key": (
                [status_route_id, status_action_generation]
                if status_action_generation > 0
                else None
            ),
            "mode": self.mode,
            "persistent_execution": bool(self.persistent_execution),
            "active": bool(self.action_active),
            "navigation_hold": bool(getattr(self, "navigation_hold", False)),
            "navigation_hold_age_seconds": (
                None
                if float(
                    getattr(self, "navigation_hold_started_monotonic", 0.0)
                    or 0.0
                )
                <= 0.0
                else round(
                    max(
                        0.0,
                        now
                        - float(
                            getattr(
                                self,
                                "navigation_hold_started_monotonic",
                                0.0,
                            )
                            or 0.0
                        ),
                    ),
                    3,
                )
            ),
            "allow_in_place_replacement": bool(self.allow_in_place_replacement),
            "allow_route_continuation_replacement": bool(
                self.allow_route_continuation_replacement
            ),
            "result_status": self.last_result_status,
            "deferred_goal_updates": int(self.deferred_goal_updates),
            "dispatches": int(self.dispatch_count),
            "terminals": int(self.terminal_count),
            "active_intent_source": self.active_intent_source,
            "active_intent_priority": int(self.active_intent_priority),
            "active_goal_transaction_id": int(
                getattr(self, "active_goal_transaction_id", 0) or 0
            ),
            "active_graph_transaction_id": int(
                getattr(self, "active_graph_transaction_id", 0) or 0
            ),
            "origin_epoch": status_origin_epoch,
            "projection_epoch": status_projection_epoch,
            "active_origin_epoch": _epoch_value(
                None if action_contract is None else action_contract.get("origin_epoch"),
                getattr(self, "active_origin_epoch", None),
            ),
            "active_projection_epoch": _epoch_value(
                None if action_contract is None else action_contract.get("projection_epoch"),
                getattr(self, "active_projection_epoch", None),
            ),
            "latest_origin_epoch": _epoch_value(
                getattr(self, "latest_origin_epoch", None)
            ),
            "latest_projection_epoch": _epoch_value(
                getattr(self, "latest_projection_epoch", None)
            ),
            "latest_graph_transaction_id": int(
                getattr(self, "latest_graph_transaction_id", 0) or 0
            ),
            "latest_intent_source": self.latest_intent_source,
            "latest_intent_priority": int(self.latest_intent_priority),
            "require_intent": bool(self.require_intent),
            "intent_seen": bool(self.intent_seen),
            "use_goal_command": bool(self.use_goal_command),
            "require_canonical_identity": bool(
                getattr(self, "require_canonical_identity", False)
            ),
            "canonical_lifecycle_high_water": int(
                getattr(self, "canonical_lifecycle_high_water", 0) or 0
            ),
            "canonical_route_high_water": int(
                getattr(self, "canonical_route_high_water", 0) or 0
            ),
            "canonical_graph_high_water": int(
                getattr(self, "canonical_graph_high_water", 0) or 0
            ),
            "canonical_map_epoch_high_water": int(
                getattr(self, "canonical_map_epoch_high_water", 0) or 0
            ),
            "goal_command_topic": self.goal_command_topic,
            "latest_goal_transaction_id": int(self.latest_goal_transaction_id),
            "last_dispatch_identity": _identity_snapshot(
                getattr(self, "last_dispatch_identity", None)
            ),
            "route_lease_watchdog": _watchdog_snapshot(
                getattr(self, "route_lease_watchdog", None), now
            ),
            "route_lease_watchdog_active": bool(
                getattr(self, "route_lease_watchdog", None) is not None
            ),
            "target_terminal_boundary_transaction": int(
                getattr(
                    self, "persistent_target_terminal_boundary_transaction", 0
                )
                or 0
            ),
            "latest_intent_goal": _point_snapshot(self.latest_intent_goal),
            "active_route_kind": self.active_route_kind,
            "latest_route_kind": self.latest_route_kind,
            "active_mission_route_kind": self.active_mission_route_kind,
            "latest_mission_route_kind": self.latest_mission_route_kind,
            "active_graph_action": str(
                getattr(self, "active_graph_action", "") or ""
            ),
            "latest_graph_action": str(
                getattr(self, "latest_graph_action", "") or ""
            ),
            "active_graph_obligation_kind": str(
                getattr(self, "active_graph_obligation_kind", "") or ""
            ),
            "latest_graph_obligation_kind": str(
                getattr(self, "latest_graph_obligation_kind", "") or ""
            ),
            "active_route_id": int(self.active_route_id),
            "latest_route_id": int(self.latest_route_id),
            "active_action_generation": int(
                action_contract.get("generation", 0)
                if action_contract is not None
                else getattr(self, "action_generation", 0) or 0
            ),
            "active_route_map_epoch": getattr(
                self, "active_route_map_epoch", None
            ),
            "latest_route_map_epoch": getattr(
                self, "latest_route_map_epoch", None
            ),
            "frontier_lease_released_route_id": int(
                getattr(self, "frontier_lease_released_route_id", 0) or 0
            ),
            "frontier_lease_released_reason": str(
                getattr(self, "frontier_lease_released_reason", "") or ""
            ),
            "active_goal_context": self.active_goal_context,
            "latest_goal_context": self.latest_goal_context,
            "active_goal": _pose_snapshot(self, self.active_goal_global),
            "active_goal_frame": _frame_snapshot(self.active_goal_global),
            "active_source_goal": _pose_snapshot(self, self.last_dispatched_goal),
            "active_source_goal_frame": _frame_snapshot(self.last_dispatched_goal),
            "active_portal_source_goal": _pose_snapshot(
                self, getattr(self, "active_portal_source_goal", None)
            ),
            "turn_supervisor_state": self.turn_supervisor_state,
            "turn_supervisor_last_event": self.turn_supervisor_last_event,
            "turn_transition_ready": bool(self.turn_transition_ready),
            "move_base_terminal_pending": bool(self.move_base_terminal_pending),
            "transport_handoff_pending": bool(
                isinstance(transport_pending, dict)
            ),
            "transport_handoff_route_id": identity_int(
                transport_pending_contract.get("route_id", 0)
            ),
            "transport_handoff_action_generation": identity_int(
                transport_pending.get("old_generation", 0)
                if isinstance(transport_pending, dict)
                else 0
            ),
            "transport_handoff_status": (
                transport_pending.get("status_before_cancel")
                if isinstance(transport_pending, dict)
                else None
            ),
            "transport_handoff_age_seconds": (
                round(
                    max(
                        0.0,
                        now
                        - float(
                            transport_pending.get("started_monotonic", now)
                        ),
                    ),
                    3,
                )
                if isinstance(transport_pending, dict)
                else None
            ),
            "transport_handoff_wall_age_seconds": (
                round(
                    max(
                        0.0,
                        time.monotonic()
                        - float(
                            transport_pending.get(
                                "started_wall_monotonic",
                                time.monotonic(),
                            )
                        ),
                    ),
                    3,
                )
                if isinstance(transport_pending, dict)
                else None
            ),
            "transport_handoff_timeout_seconds": (
                transport_pending.get("handoff_timeout_seconds")
                if isinstance(transport_pending, dict)
                else None
            ),
            "transport_handoff_controller_restart_requested": bool(
                isinstance(transport_pending, dict)
                and transport_pending.get("controller_restart_requested")
            ),
            "transport_handoff_controller_restart_attempts": int(
                transport_pending.get("controller_restart_attempts", 0) or 0
            )
            if isinstance(transport_pending, dict)
            else 0,
            "transport_handoff_server_disconnect_observed": bool(
                isinstance(transport_pending, dict)
                and transport_pending.get(
                    "controller_restart_server_disconnect_observed"
                )
            ),
            "active_feedback_frame": self.active_feedback_frame,
            "feedback_transform_failures": int(self.feedback_transform_failures),
            "navfn_plan_topic": self.navfn_plan_topic,
            "navfn_path_remaining": _rounded_or_none(self.active_navfn_remaining, 4),
            "navfn_path_endpoint": self.active_navfn_plan_endpoint,
            "navfn_path_progress_age": _elapsed_or_none(
                self.active_navfn_progress_monotonic, 4, now
            ),
            "physical_motion_progress_age": _elapsed_or_none(
                self.active_motion_progress_monotonic, 4, now
            ),
            "teb_selected_velocity": _velocity_snapshot(
                self.latest_teb_selected_linear,
                self.latest_teb_selected_angular,
                self.latest_teb_feedback_monotonic,
                now=now,
            ),
            "teb_planner_command": _planner_command_snapshot(self, now=now),
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
            "teb_feedback_generation": int(
                getattr(self, "teb_feedback_generation", 0) or 0
            ),
            "teb_feedback_route_id": int(
                getattr(self, "teb_feedback_route_id", 0) or 0
            ),
            "teb_feedback_transaction_id": int(
                getattr(self, "teb_feedback_transaction_id", 0) or 0
            ),
            "teb_feedback_map_epoch": getattr(
                self, "teb_feedback_map_epoch", None
            ),
            "teb_feedback_graph_transaction_id": int(
                getattr(self, "teb_feedback_graph_transaction_id", 0) or 0
            ),
            "teb_feedback_invalidation_count": int(
                getattr(self, "teb_feedback_invalidation_count", 0) or 0
            ),
            "teb_reorientation": _reorientation_snapshot(self, now=now),
            "priority_handoffs": int(self.priority_handoff_count),
            "target_segment_handoffs": int(self.target_segment_handoff_count),
            "frontier_segment_handoffs": int(self.frontier_segment_handoff_count),
            "frontier_observation_completions": int(
                self.frontier_observation_completion_count
            ),
            "frontier_continuous_prefetch_handoffs": int(
                self.frontier_continuous_prefetch_handoff_count
            ),
            "frontier_continuous_prefetch_fallbacks": int(
                self.frontier_continuous_prefetch_handoff_fallback_count
            ),
            "frontier_continuous_prefetch_pending": _prefetch_pending_snapshot(self),
            "prefetched_frontier": _point_snapshot(self.prefetched_frontier_goal),
            "prefetched_frontier_route_id": int(self.prefetched_frontier_route_id),
            "active_target_epoch": int(self.active_target_epoch),
            "latest_target_epoch": int(self.latest_target_epoch),
            "active_map_epoch": getattr(self, "active_frontier_map_epoch", None),
            "latest_map_epoch": getattr(self, "latest_frontier_map_epoch", None),
            "active_epoch": int(getattr(self, "active_target_epoch", 0) or 0),
            "latest_epoch": int(getattr(self, "latest_target_epoch", 0) or 0),
            "active_target_track_id": self.active_target_track_id,
            "latest_target_track_id": self.latest_target_track_id,
            "active_target_viewpoint_candidate_id": str(
                getattr(self, "active_target_viewpoint_candidate_id", "") or ""
            ),
            "latest_target_viewpoint_candidate_id": str(
                getattr(self, "latest_target_viewpoint_candidate_id", "") or ""
            ),
            "active_target_viewpoint_attempt_id": str(
                getattr(self, "active_target_viewpoint_attempt_id", "") or ""
            ),
            "latest_target_viewpoint_attempt_id": str(
                getattr(self, "latest_target_viewpoint_attempt_id", "") or ""
            ),
            "target_failure_latched": bool(self.target_failure_latched),
            "target_failure_count": int(self.target_failure_count),
            "target_failure_epoch": int(self.target_failure_epoch),
            "target_failure_track_id": self.target_failure_track_id,
            "target_lease_tombstone_transaction_id": int(
                getattr(self, "target_lease_tombstone_transaction_id", 0) or 0
            ),
            "target_lease_tombstone_epoch": int(
                getattr(self, "target_lease_tombstone_epoch", 0) or 0
            ),
            "target_lease_tombstone_track_id": str(
                getattr(self, "target_lease_tombstone_track_id", "") or ""
            ),
        }
        payload.update(fields)
        self.bridge_events += 1
        try:
            self.bridge_status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        except (TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "TEB goal bridge status serialization failed")


def _point_snapshot(point):
    if point is None:
        return None
    return [round(float(point[0]), 3), round(float(point[1]), 3)]


def _identity_snapshot(identity):
    """Return only scalar dispatch identity fields for JSON status output."""
    if not isinstance(identity, dict):
        return None
    return {
        "action_generation": int(identity.get("action_generation", 0) or 0),
        "transaction_id": int(identity.get("transaction_id", 0) or 0),
        "lifecycle_transaction_id": int(
            identity.get("lifecycle_transaction_id", 0) or 0
        ),
        "graph_transaction_id": int(
            identity.get("graph_transaction_id", 0) or 0
        ),
        "route_id": int(identity.get("route_id", 0) or 0),
        "epoch": identity.get("epoch", identity.get("target_epoch", 0)),
        "map_epoch": identity.get("map_epoch"),
        "origin_epoch": identity.get("origin_epoch"),
        "projection_epoch": identity.get("projection_epoch"),
        "route_kind": str(identity.get("route_kind", "") or ""),
        "mission_route_kind": str(
            identity.get("mission_route_kind", "") or ""
        ),
        "source": str(identity.get("source", "unknown") or "unknown"),
        "priority": int(identity.get("priority", 0) or 0),
    }


def _watchdog_snapshot(record, now):
    """Return the scalar watchdog contract for machine-readable status."""
    if not isinstance(record, dict):
        return None
    deadline = record.get("deadline")
    try:
        remaining = None if deadline is None else round(
            max(0.0, float(deadline) - float(now)), 3
        )
    except (TypeError, ValueError):
        remaining = None
    return {
        "route_id": int(record.get("route_id", 0) or 0),
        "transaction_id": int(record.get("transaction_id", 0) or 0),
        "lifecycle_transaction_id": int(
            record.get("lifecycle_transaction_id", 0) or 0
        ),
        "action_generation": int(record.get("action_generation", 0) or 0),
        "epoch": record.get("epoch"),
        "map_epoch": record.get("map_epoch"),
        "epoch_source": str(record.get("epoch_source", "unavailable") or "unavailable"),
        "source": str(record.get("source", "unknown") or "unknown"),
        "route_kind": str(record.get("route_kind", "") or ""),
        "status": record.get("status"),
        "status_text": str(record.get("status_text", "UNKNOWN") or "UNKNOWN"),
        "reason": str(record.get("reason", "") or ""),
        "armed_at": record.get("armed_at"),
        "deadline": deadline,
        "remaining_seconds": remaining,
    }


def _pose_snapshot(bridge, goal):
    if goal is None:
        return None
    return [
        round(float(goal.pose.position.x), 3),
        round(float(goal.pose.position.y), 3),
        round(float(bridge._yaw(goal)), 4),
    ]


def _frame_snapshot(goal):
    return None if goal is None else goal.header.frame_id


def _rounded_or_none(value, digits):
    return None if value is None else round(float(value), digits)


def _elapsed_or_none(start, digits, now=None):
    if start <= 0.0:
        return None
    if now is None:
        now = start
    return round(max(0.0, now - start), digits)


def _velocity_snapshot(linear, angular, timestamp, now=None):
    if linear is None:
        return None
    if now is None:
        now = timestamp
    return {
        "linear_x": round(float(linear), 4),
        "angular_z": round(float(angular), 4),
        "age": round(max(0.0, now - timestamp), 3),
    }


def _planner_command_snapshot(bridge, now=None):
    if now is None:
        now = now_for(bridge)
    snapshot = _velocity_snapshot(
        bridge.latest_teb_planner_linear,
        bridge.latest_teb_planner_angular,
        bridge.latest_teb_planner_command_monotonic,
        now=now,
    )
    if snapshot is None:
        return None
    snapshot["stationary_for"] = (
        round(max(0.0, now - bridge.teb_planner_stationary_since), 3)
        if bridge.teb_planner_stationary_since > 0.0
        else 0.0
    )
    return snapshot


def _reorientation_snapshot(bridge, now=None):
    started = bridge.teb_reorientation_started_monotonic
    if now is None:
        now = now_for(bridge)
    return {
        "active": bool(started > 0.0),
        "duration": 0.0 if started <= 0.0 else round(max(0.0, now - started), 3),
        "yaw_progress": round(float(bridge.teb_reorientation_total_yaw), 4),
        "deferrals": int(bridge.teb_reorientation_deferrals),
    }


def _prefetch_pending_snapshot(bridge):
    pending = bridge.frontier_continuous_prefetch_handoff_pending
    if pending is None:
        return None
    return {
        "route_id": int(pending["route_id"]),
        "successor_route_id": int(pending["successor_route_id"]),
    }


def _epoch_value(*values):
    """Return the first non-negative epoch without inventing a lineage."""
    for value in values:
        if value is None or value == "":
            continue
        try:
            value = int(value)
        except (TypeError, ValueError):
            continue
        if value >= 0:
            return value
    return None
