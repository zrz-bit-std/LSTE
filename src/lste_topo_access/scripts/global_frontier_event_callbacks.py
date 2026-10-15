#!/usr/bin/env python3

"""Asynchronous ROS event callbacks for global-frontier exploration."""

import json
import math
from contextlib import nullcontext
from types import SimpleNamespace

import rospy

from goal_context import task_version_from_task
from global_frontier_semantic_belief import (
    semantic_place_action,
    task_intent_from_message,
)
from lifecycle_manager import EventType
from clock_provider import now_for


class GlobalFrontierEventCallbacksMixin:
    def _target_visual_control_suppressed(self):
        """Return whether room-entry completion owns the mission boundary."""
        return str(
            getattr(self, "target_done_policy", "visual_target") or ""
        ).strip().lower() == "target_room_entry"

    def _apply_target_done_policy(self, policy):
        """Apply the shared completion policy at the graph ownership boundary."""
        policy = str(policy or "").strip().lower()
        if policy not in (
            "visual_target",
            "target_room_entry",
            "target_context",
        ):
            return False
        previous = str(
            getattr(self, "target_done_policy", "visual_target") or ""
        ).strip().lower()
        self.target_done_policy = policy
        if policy == "target_room_entry" and previous != policy:
            # A room-entry benchmark measures topology, so any detector-driven
            # target obligation already queued before the policy arrived must
            # be discarded at the same ownership boundary. Generic environment
            # labels remain valid exploration evidence.
            target_belief = getattr(self, "target_belief", None)
            if target_belief is not None:
                target_belief.reset(getattr(self, "current_task_version", ""))
            target_work = getattr(self, "target_observation_work", None)
            if target_work is not None:
                target_work.reset(getattr(self, "current_task_version", ""))
            self.pending_target_belief_observations = []
            self.pending_target_track_ids = []
            self.target_reinspection_pending = False
            self.publish_status(
                "target_visual_control_suppressed",
                target_done_policy=policy,
                reason="target_room_entry_completion_owns_mission",
            )
        return True

    @staticmethod
    def _enqueue_or_apply(owner, event_type, message, apply):
        enqueue = getattr(owner, "_enqueue_lifecycle_event", None)
        if callable(enqueue):
            return enqueue(event_type, message)
        return apply(message)

    def gather_task(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.TASK_UPDATED, message, self.apply_task
        )

    def on_task(self, message):
        """Install a new semantic mission without touching route ownership."""
        task_id = str(getattr(message, "task_id", "") or "").strip()
        task_version = task_version_from_task(message)
        if (
            task_id == getattr(self, "current_task_id", "")
            and task_version == getattr(self, "current_task_version", "")
        ):
            self.latest_task = message
            return
        self.latest_task = message
        self.current_task_id = task_id
        self.current_task_version = task_version
        self.current_mission_id = task_id
        self.semantic_place_belief.reset(
            task_intent_from_message(message, task_version)
        )
        completion_state = getattr(self, "graph_completion_state", None)
        if completion_state is not None:
            completion_state.reset()
        self.last_completion_gate_signature = None
        target_belief = getattr(self, "target_belief", None)
        if target_belief is not None:
            target_belief.reset(task_version)
        target_observation_work = getattr(self, "target_observation_work", None)
        if target_observation_work is not None:
            target_observation_work.reset(task_version)
        reopen_target_observation = getattr(
            self, "_clear_target_observation_materialization_deferred", None
        )
        if callable(reopen_target_observation):
            reopen_target_observation(None, reason="new_task")
        self.pending_semantic_observations = []
        self.pending_target_belief_observations = []
        self.pending_target_track_ids = []
        self._semantic_last_report = {}
        self._target_belief_last_report = {}
        self._target_belief_geometry_reports = set()
        self.publish_status(
            "semantic_task_updated",
            task_id=task_id or None,
            task_version=task_version or None,
            target_terms=sorted(self.semantic_place_belief.intent.target_terms),
            context_terms=sorted(self.semantic_place_belief.intent.context_terms),
            negative_terms=sorted(self.semantic_place_belief.intent.negative_terms),
        )

    @staticmethod
    def _semantic_stamp(message):
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        try:
            value = float(stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            value = 0.0
        return value

    @staticmethod
    def _detection_labels(message):
        target = tuple(getattr(message, "target_dets", []) or ())
        environment = tuple(getattr(message, "env_dets", []) or ())
        labels = [str(getattr(item, "label", "") or "").strip() for item in environment]
        labels.extend(
            str(getattr(item, "label", "") or "").strip() for item in target
        )
        labels = [label for label in labels if label]
        target_labels = [
            str(getattr(item, "label", "") or "").strip()
            for item in target
            if str(getattr(item, "label", "") or "").strip()
        ]
        return labels, target_labels

    def _record_semantic_observation(self, place_id, labels, target_labels, stamp):
        if not labels:
            return
        previous = self.semantic_place_belief.evidence(place_id) or {}
        evidence = self.semantic_place_belief.observe(
            place_id,
            labels,
            now=stamp,
            target_labels=target_labels,
        )
        if evidence is None:
            return
        report_key = (
            tuple(evidence["context_terms"]),
            tuple(evidence["target_terms"]),
            tuple(evidence["negative_terms"]),
        )
        if report_key == getattr(self, "_semantic_last_report", {}).get(int(place_id)):
            return
        self._semantic_last_report[int(place_id)] = report_key
        place = self.region_memory.by_id(place_id)
        place_observed = bool(
            place is not None and int(place.get("endpoint_observations", 0)) > 0
        )
        ledger = getattr(self, "place_work_items", None)
        local_work_pending = (
            True
            if ledger is None or place_id is None
            else bool(ledger.unresolved_count(place_id) > 0)
        )
        self.publish_status(
            "semantic_place_evidence",
            task_id=str(getattr(self, "current_task_id", "") or "") or None,
            task_version=str(getattr(self, "current_task_version", "") or "") or None,
            place_id=int(place_id),
            labels=sorted(set(labels)),
            new_context_terms=sorted(
                set(evidence["context_terms"]) - set(previous.get("context_terms", []))
            ),
            new_target_terms=sorted(
                set(evidence["target_terms"]) - set(previous.get("target_terms", []))
            ),
            action=semantic_place_action(
                self.semantic_place_belief.intent,
                evidence,
                place_observed,
                local_work_pending,
            ),
            evidence=evidence,
        )

    def _record_target_belief_observation(
        self, place_id, labels, target_labels, stamp,
    ):
        """Keep target evidence separate from generic semantic counters."""
        target_belief = getattr(self, "target_belief", None)
        if target_belief is None or place_id is None:
            return
        if target_labels:
            reopen_target_observation = getattr(
                self, "_clear_target_observation_materialization_deferred", None
            )
            if callable(reopen_target_observation):
                reopen_target_observation(
                    place_id, reason="new_target_detection_evidence"
                )
            evidence = target_belief.observe_target(
                self.current_task_version,
                place_id,
                labels=target_labels,
                now=stamp,
            )
            target_observation_work = getattr(
                self, "target_observation_work", None
            )
            if target_observation_work is not None:
                target_observation_work.observe_target(
                    self.current_task_version,
                    place_id,
                    labels=target_labels,
                    now=stamp,
                )
        else:
            evidence = target_belief.observe_context(
                self.current_task_version,
                place_id,
                labels=labels,
                now=stamp,
            )
        if evidence is not None:
            report_key = (
                evidence.get("state"),
                tuple(sorted(evidence.get("target_labels", {}))),
                tuple(sorted(evidence.get("context_labels", {}))),
                tuple(sorted(evidence.get("negative_labels", {}))),
            )
            reports = getattr(self, "_target_belief_last_report", {})
            if report_key == reports.get(int(place_id)):
                return
            reports[int(place_id)] = report_key
            self._target_belief_last_report = reports
            self.publish_status(
                "target_belief_updated",
                place_id=int(place_id),
                source="detections",
                evidence=evidence,
            )

    def _bind_pending_semantic_observations(self, place_id):
        """Attach pre-bootstrap detector frames to the first physical Place."""
        pending = list(getattr(self, "pending_semantic_observations", []))
        self.pending_semantic_observations = []
        for labels, target_labels, stamp in pending:
            self._record_semantic_observation(
                place_id, labels, target_labels, stamp,
            )

    def _bind_pending_target_belief_observations(self, place_id):
        pending = list(getattr(self, "pending_target_belief_observations", []))
        self.pending_target_belief_observations = []
        for labels, target_labels, stamp in pending:
            self._record_target_belief_observation(
                place_id, labels, target_labels, stamp,
            )
        target_work = getattr(self, "target_observation_work", None)
        pending_tracks = list(getattr(self, "pending_target_track_ids", []))
        self.pending_target_track_ids = []
        if target_work is not None:
            for track_id in pending_tracks:
                target_work.bind_track(
                    self.current_task_version, place_id, track_id,
                )

    def gather_detections(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.DETECTIONS_UPDATED, message, self.apply_detections
        )

    def on_detections(self, message):
        """Associate semantic detections with the current physical Place."""
        task_id = str(getattr(message, "task_id", "") or "").strip()
        current_task_id = str(getattr(self, "current_task_id", "") or "").strip()
        if current_task_id and task_id and task_id != current_task_id:
            return
        labels, target_labels = self._detection_labels(message)
        if not labels:
            return
        if self._target_visual_control_suppressed():
            # Keep detector labels available for generic room/context evidence,
            # but do not let target labels create a durable target route.
            target_labels = []
        stamp = self._semantic_stamp(message)
        place_id = getattr(self, "current_physical_place_id", None)
        if place_id is None:
            pending = getattr(self, "pending_semantic_observations", [])
            pending.append((labels, target_labels, stamp))
            self.pending_semantic_observations = pending[-8:]
            pending_target = getattr(self, "pending_target_belief_observations", [])
            pending_target.append((labels, target_labels, stamp))
            self.pending_target_belief_observations = pending_target[-8:]
            return
        self._record_target_belief_observation(
            int(place_id), labels, target_labels, stamp,
        )
        self._record_semantic_observation(
            int(place_id), labels, target_labels, stamp,
        )

    def gather_goal_arbitration(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.GOAL_UPDATED, message, self.apply_goal_arbitration
        )

    def on_goal_arbitration(self, message):
        """Attach committed visual bearings to durable task evidence."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        if str(payload.get("task_version", "")) != str(
            getattr(self, "current_task_version", "")
        ):
            return
        self._apply_target_done_policy(payload.get("target_done_policy"))
        event = str(payload.get("event", "")).strip()
        if self._target_visual_control_suppressed() and event.startswith("target_"):
            # Room-entry completion is intentionally independent of visual
            # target tracking. Ignore late target-track/segment events so they
            # cannot recreate target observation work after suppression.
            return
        place_id = getattr(self, "current_physical_place_id", None)
        if event == "target_track_started":
            track_id = str(payload.get("target_track_id", "")).strip()
            if place_id is None:
                pending_tracks = list(
                    getattr(self, "pending_target_track_ids", [])
                )
                if track_id and track_id not in pending_tracks:
                    pending_tracks.append(track_id)
                self.pending_target_track_ids = pending_tracks[-8:]
                return
            target_observation_work = getattr(
                self, "target_observation_work", None
            )
            if target_observation_work is not None:
                record = target_observation_work.bind_track(
                    self.current_task_version,
                    place_id,
                    payload.get("target_track_id", ""),
                )
                if record is not None:
                    self.publish_status(
                        "target_observation_work_bound",
                        place_id=int(place_id),
                        target_track_id=track_id,
                        work=record,
                    )
            return
        if place_id is None:
            return
        if event not in ("target_segment_committed",):
            return
        bearing = payload.get("target_bearing_odom")
        origin = payload.get("target_observation_origin_odom")
        if bearing is None and payload.get("heading") is not None:
            try:
                heading = float(payload["heading"])
                bearing = [math.cos(heading), math.sin(heading)]
            except (TypeError, ValueError):
                bearing = None
        target_belief = getattr(self, "target_belief", None)
        if target_belief is None:
            return
        record = target_belief.bind_observation_geometry(
            self.current_task_version,
            place_id,
            bearing_xy=bearing,
            origin_xy=origin,
            work_item_id=getattr(self, "active_work_item_id", None),
            portal_id=getattr(self, "last_portal_hypothesis_id", None),
            track_id=payload.get("target_track_id", ""),
            now=rospy.Time.now().to_sec(),
            confirmed=(event == "target_segment_committed"),
        )
        if record is not None:
            geometry_key = (
                int(place_id),
                record.get("track_id"),
                record.get("bearing_xy"),
                record.get("origin_xy"),
                record.get("state"),
            )
            geometry_reports = getattr(
                self, "_target_belief_geometry_reports", set()
            )
            if geometry_key in geometry_reports:
                return
            geometry_reports.add(geometry_key)
            self._target_belief_geometry_reports = geometry_reports
            self.publish_status(
                "target_belief_geometry_bound",
                place_id=int(place_id),
                target_track_id=record.get("track_id"),
                event_source=event,
                evidence=record,
            )

    def semantic_place_action(
        self, place_id, place_observed, local_work_pending=True,
    ):
        """Expose the finite-state semantic policy to the planner."""
        evidence = self.semantic_place_belief.evidence(place_id)
        target_belief = (
            None
            if self._target_visual_control_suppressed()
            else getattr(self, "target_belief", None)
        )
        target_evidence = (
            None if target_belief is None else target_belief.evidence(place_id)
        )
        if target_evidence is not None and target_evidence.get("target_hits", 0):
            evidence = dict(evidence or {})
            # The independent ledger is authoritative for target ownership;
            # generic semantic labels remain useful context but cannot erase a
            # target that temporarily disappeared from the detector stream.
            evidence["target_hits"] = max(
                int(evidence.get("target_hits", 0)),
                int(target_evidence["target_hits"]),
            )
        return semantic_place_action(
            self.semantic_place_belief.intent,
            evidence,
            place_observed,
            local_work_pending,
        )

    def gather_task_done(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.TASK_COMPLETED, message, self.apply_task_done
        )

    def on_task_done(self, message):
        done = bool(message.data)
        if done == self.task_done:
            return
        self.task_done = done
        if done:
            self.publish_status("task_done", task_done=True)
            self.target_reinspection_pending = False
            target_observation_work = getattr(
                self, "target_observation_work", None
            )
            if target_observation_work is not None:
                target_observation_work.complete(
                    self.current_task_version,
                    now=rospy.Time.now().to_sec(),
                    reason="task_done",
                )
            self.clear_target_region_claim("task_done")
            transaction = getattr(self, "portal_transaction", None)
            if transaction is not None and transaction.active:
                aborted = transaction.abort("task_done")
                if aborted is not None:
                    self.publish_status(
                        "portal_transaction_aborted",
                        transaction_id=int(aborted.transaction_id),
                        route_id=int(aborted.route_id),
                        state=aborted.state,
                        reason=aborted.last_reason,
                    )
            if transaction is not None and transaction.state == "aborted":
                transaction.finish("task_done")
            # The TEB bridge and mux stop the vehicle when the target is
            # confirmed. Clear the exploration commitment as well so this node
            # cannot publish another endpoint after task completion.
            self.active_frontier = None
            self.active_frontier_component = None
            self.active_portal_gate_xy = None
            self.active_portal_gate_odom_xy = None
            self.active_portal_gate_approached_at = None
            self.active_portal_crossing_observed = False
            self.active_portal_crossing_preobserved = False
            self.active_portal_crossing_rejected = False
            self.pending_portal_arrivals.clear()
            self.active_frontier_region_id = None
            self.active_work_item_id = None
            self.active_work_item_attempt_id = None
            self.active_work_item_place_id = None
            self.active_work_item_goal = None
            self.active_work_item_route_kind = None
            self.active_work_item_dispatch_announced = False
            self.active_portal_probe_id = None
            self.active_portal_probe_phase = ""
            self.active_observation_session_started_at = None
            self._clear_active_place_departure()
            self.observation_departure_source = None
            self.active_last_robot_xy = None
            self.clear_active_route_history()
            self.active_last_waypoint_map = None
            self.active_last_waypoint_yaw = None
            self.active_route_kind = "frontier_endpoint"
            self.active_mission_route_kind = "frontier_endpoint"
            self.active_terminal_received = False
            self.recovery_pending_route_id = 0
            self.recovery_pending_behavior = ""
            self.recovery_pending_reason = ""
            self.turn_connector_released = True
            self.last_status_command_map = None
            self.last_status_command_yaw = None
            self.last_status_mission_map = None
            self.active_best_distance = None
            self.active_best_goal_distance = None
            self.active_best_path_distance = None
            self.active_last_progress_signal = "none"
            self.active_visited_odom_cells.clear()
            self.active_unreachable_since = None
            self.clear_prefetched_frontier()
            self.pending_local_egress = None
            self.local_egress_place_lease.clear()
            self.pending_portal_retry = None
            self.active_portal_retry = False
            self.active_local_egress_resumes_portal = False
            rospy.loginfo("Global frontier paused: task_done=true")
        else:
            completion_state = getattr(self, "graph_completion_state", None)
            if completion_state is not None:
                completion_state.reset()
            decision_scheduler = getattr(self, "decision_wake_scheduler", None)
            if decision_scheduler is not None:
                decision_scheduler.resume()
            self.last_completion_gate_signature = None
            self.frontier_exhausted = False
            rospy.loginfo("Global frontier resumed: task_done=false")

    def gather_move_base_recovery(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self,
            EventType.RECOVERY_OBSERVED,
            message,
            self.apply_move_base_recovery,
        )

    def on_move_base_recovery(self, message):
        """Record a local recovery without preempting the global action.

        ``RecoveryStatus`` is emitted when a recovery *starts*, including the
        first behaviour in a multi-step sequence. Treating that first event as
        a global failure used to cancel the action before TEB could complete
        its own recovery, then publish a distant frontier every few seconds.
        The action bridge reports the authoritative terminal outcome through
        :meth:`on_bridge_status`; this callback only keeps the no-progress
        watchdog from racing the local recovery.
        """
        if self.task_done or self.active_frontier is None or self.active_route_id <= 0:
            return
        behavior = str(message.recovery_behavior_name or "unknown")
        route_id = int(self.active_route_id)
        forward_clearance = self.scan_forward_minimum
        self.active_progress_time = now_for(self)
        self.active_last_progress_signal = "local_recovery:%s" % behavior
        self.publish_status(
            "planner_recovery_observed",
            route_id=route_id,
            recovery_behavior=behavior,
            recovery_index=int(message.current_recovery_number),
            recovery_total=int(message.total_number_of_recoveries),
            forward_clearance=(
                None if not math.isfinite(forward_clearance)
                else round(float(forward_clearance), 3)
            ),
            required_clearance=round(float(self.clearance), 3),
        )
        rospy.logwarn(
            "Global frontier retains route_id=%d during local recovery "
            "behavior=%s (%d/%d); waiting for action outcome",
            route_id,
            behavior,
            int(message.current_recovery_number) + 1,
            int(message.total_number_of_recoveries),
        )

    def gather_bridge_status(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.NAV_FAILED, message, self.apply_bridge_status
        )

    @staticmethod
    def _bridge_status_contract(payload):
        """Extract the immutable dispatch identity from one bridge event."""
        identity = payload.get("last_dispatch_identity")
        identity = identity if isinstance(identity, dict) else {}

        def positive_int(*values):
            for value in values:
                try:
                    value = int(value or 0)
                except (TypeError, ValueError):
                    continue
                if value > 0:
                    return value
            return 0

        def text_value(*values):
            for value in values:
                value = str(value or "").strip().lower()
                if value:
                    return value
            return ""

        def optional_int(*values):
            for value in values:
                if value is None:
                    continue
                try:
                    return int(value)
                except (TypeError, ValueError):
                    continue
            return None

        return {
            "route_id": positive_int(
                payload.get("route_id"), identity.get("route_id")
            ),
            "action_generation": positive_int(
                payload.get("action_generation"),
                identity.get("action_generation"),
            ),
            "transaction_id": positive_int(
                payload.get("transaction_id"), identity.get("transaction_id")
            ),
            "route_kind": text_value(
                payload.get("route_kind"),
                identity.get("route_kind"),
                payload.get("active_route_kind"),
            ),
            "mission_route_kind": text_value(
                payload.get("mission_route_kind"),
                identity.get("mission_route_kind"),
                payload.get("active_mission_route_kind"),
            ),
            "source": text_value(
                payload.get("intent_source"),
                identity.get("source"),
                payload.get("active_intent_source"),
                payload.get("source"),
            ),
            "priority": positive_int(
                payload.get("intent_priority"),
                identity.get("priority"),
                payload.get("active_intent_priority"),
            ),
            "lifecycle_transaction_id": positive_int(
                payload.get("lifecycle_transaction_id"),
                identity.get("lifecycle_transaction_id"),
            ),
            "graph_transaction_id": positive_int(
                payload.get("graph_transaction_id"),
                identity.get("graph_transaction_id"),
            ),
            "map_epoch": optional_int(
                payload.get("map_epoch"),
                payload.get("active_route_map_epoch"),
                identity.get("map_epoch"),
            ),
            "origin_epoch": optional_int(
                payload.get("origin_epoch"),
                payload.get("active_origin_epoch"),
                identity.get("origin_epoch"),
            ),
            "projection_epoch": optional_int(
                payload.get("projection_epoch"),
                payload.get("active_projection_epoch"),
                identity.get("projection_epoch"),
            ),
        }

    def _remember_bridge_dispatch_contract(self, payload):
        """Remember only routes admitted by the bridge execution boundary."""
        event = str(payload.get("event", "")).strip()
        if event not in {
            "dispatch",
            "persistent_mission_goal_adopted",
            "persistent_mission_path_adopted",
            "dispatch_contract_reaffirmed",
        }:
            return None
        contract = self._bridge_status_contract(payload)
        if (
            contract["route_id"] <= 0
            or contract["action_generation"] <= 0
            or contract["source"] != "global_slam_frontier"
            or contract["priority"] != 0
            or contract["route_kind"]
            not in {"frontier_endpoint", "portal_transition", "local_egress"}
        ):
            return None
        contracts = getattr(self, "_bridge_dispatch_contracts", None)
        if contracts is None:
            contracts = {}
            self._bridge_dispatch_contracts = contracts
        previous = contracts.get(contract["route_id"])
        if previous is None or contract["action_generation"] >= previous["action_generation"]:
            contracts[contract["route_id"]] = contract
        if int(getattr(self, "active_route_id", 0) or 0) == contract["route_id"]:
            # Complete the activation-time predecessor record with the
            # mission transaction and action generation assigned downstream.
            # Keep the route/lifecycle/epoch fields captured at activation if
            # an out-of-order status carries a zero compatibility sentinel.
            active_contract = getattr(
                self, "active_route_dispatch_contract", None
            )
            merged = dict(active_contract) if isinstance(active_contract, dict) else {}
            for field, value in contract.items():
                if value not in (None, "", 0):
                    merged[field] = value
            if merged:
                self.active_route_dispatch_contract = merged
        # Route IDs are monotonic. Keep the evidence lookup bounded during a
        # long exploration run.
        for route_id in sorted(contracts)[:-64]:
            contracts.pop(route_id, None)
        return contracts.get(contract["route_id"])

    def _bridge_terminal_matches_dispatch(self, payload, contract):
        """Reject terminal failures that have no matching real dispatch."""
        if contract is None:
            return False, "dispatch_contract_missing"
        expected = getattr(self, "_bridge_dispatch_contracts", {}).get(
            contract["route_id"]
        )
        if expected is None:
            return False, "dispatch_contract_missing"
        if contract["action_generation"] <= 0:
            return False, "action_generation_missing"
        for field in (
            "action_generation",
            "route_kind",
            "mission_route_kind",
            "source",
            "priority",
            "lifecycle_transaction_id",
            "graph_transaction_id",
        ):
            if contract[field] != expected[field]:
                return False, "dispatch_contract_%s_mismatch" % field
        # Persistent execution can advance the GoalManager semantic transaction
        # while retaining one route/action lease.  Route, action generation,
        # graph, and lifecycle identities above remain the hard boundary; the
        # semantic transaction may only move forward, never backward.
        expected_transaction_id = int(expected.get("transaction_id", 0) or 0)
        received_transaction_id = int(contract.get("transaction_id", 0) or 0)
        if (
            expected_transaction_id > 0
            and received_transaction_id < expected_transaction_id
        ):
            return False, "dispatch_contract_transaction_id_mismatch"
        if received_transaction_id > expected_transaction_id:
            expected["transaction_id"] = received_transaction_id
        if (
            expected.get("map_epoch") is not None
            and contract.get("map_epoch") != expected.get("map_epoch")
        ):
            return False, "dispatch_contract_map_epoch_mismatch"
        for field in ("origin_epoch", "projection_epoch"):
            expected_epoch = expected.get(field)
            received_epoch = contract.get(field)
            if (
                expected_epoch is not None
                and received_epoch is not None
                and int(received_epoch) != int(expected_epoch)
            ):
                return False, "dispatch_contract_%s_mismatch" % field
        return True, "matched"

    def on_bridge_status(self, message):
        """Promote only a matching failed frontier action to a route change."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return False
        if not isinstance(payload, dict):
            return False
        dispatch_contract = self._remember_bridge_dispatch_contract(payload)
        if dispatch_contract is not None:
            retry = getattr(self, "_retry_terminals_after_dispatch", None)
            if callable(retry):
                retry(dispatch_contract)
            return False
        if payload.get("event") == "lease_release_ack":
            return self._accept_controller_lease_release_ack(payload)
        if payload.get("event") != "terminal":
            return False
        if self.task_done:
            return False
        status = int(payload.get("status", -1) or -1)
        # PREEMPTED is an intentional lifecycle handoff/cancel and therefore
        # cannot be used as evidence that the frontier is unreachable.
        if status not in (4, 5, 8, 9):  # ABORTED, REJECTED, RECALLED, LOST
            return False
        contract = self._bridge_status_contract(payload)
        if contract["route_id"] <= 0 or contract["route_id"] != self.active_route_id:
            return False
        matches, mismatch_reason = self._bridge_terminal_matches_dispatch(
            payload, contract
        )
        if not matches:
            self.publish_status(
                "bridge_terminal_ignored",
                reason=mismatch_reason,
                terminal_route_id=contract["route_id"],
                active_route_id=int(self.active_route_id),
                action_generation=contract["action_generation"],
            )
            return False
        route_id = contract["route_id"]
        route_kind = contract["route_kind"]
        mission_route_kind = contract["mission_route_kind"]
        active_frontier_live = self.active_frontier is not None
        released_kind = str(
            getattr(self, "last_released_route_kind", "") or ""
        ).strip().lower()
        released_route_matches = bool(
            not active_frontier_live
            and bool(
                getattr(self, "last_released_route_controller_pending", False)
            )
            and route_id == int(getattr(self, "last_released_route_id", 0) or 0)
            and released_kind in {
                str(route_kind or "").strip().lower(),
                str(mission_route_kind or "").strip().lower(),
            }
        )
        if not active_frontier_live and not released_route_matches:
            # ``active_route_id`` is monotonic and therefore cannot by itself
            # prove that a controller still owns the route. A delayed action
            # callback is admissible only through the explicit release
            # tombstone recorded by ``release_active_frontier``.
            return False
        status_name = str(payload.get("status_text", "FAILED")).upper()
        self.recovery_pending_route_id = route_id
        self.recovery_pending_behavior = "move_base_terminal"
        self.recovery_pending_reason = "move_base_%s" % status_name.lower()
        self.recovery_pending_wall = now_for(self)
        released_after_logical_terminal = bool(
            released_route_matches
            and getattr(self, "last_released_route_terminal_received", False)
        )
        if (
            (route_kind == "portal_transition" or mission_route_kind == "portal_transition")
            and not released_after_logical_terminal
        ):
            portal_id = int(getattr(self, "last_portal_hypothesis_id", 0) or 0)
            ledger = getattr(self, "portal_hypothesis_ledger", None)
            failed = None
            failure_event = "portal_hypothesis_failed"
            if ledger is not None and portal_id > 0:
                record = getattr(ledger, "get", lambda _id: None)(portal_id)
                is_crossed = (
                    isinstance(record, dict)
                    and str(record.get("state", "")).strip().lower()
                    == "crossed"
                )
                recorder = getattr(ledger, "execution_failed", None)
                if is_crossed:
                    failed = (
                        record
                        if not callable(recorder)
                        else recorder(
                            portal_id, now=rospy.Time.now().to_sec(),
                        )
                    )
                    failure_event = "portal_execution_failure_observed"
                else:
                    failed = ledger.failed(
                        portal_id, now=rospy.Time.now().to_sec(),
                    )
            if failed is not None:
                self.publish_status(
                    failure_event,
                    portal_id=int(failed["id"]),
                    source_place_id=int(failed["source_place_id"]),
                    route_id=route_id,
                    status_text=status_name,
                    state=str(failed.get("state", "")),
                    failure_count=int(failed.get("failure_count", 0)),
                    execution_failure_count=int(
                        failed.get("execution_failure_count", 0)
                    ),
                )
        if released_route_matches:
            # The graph already consumed this route's logical terminal. The
            # delayed MoveBase failure is controller cleanup, not a second
            # Portal attempt; consume the tombstone exactly once.
            self.last_released_route_controller_pending = False
        # An action terminal is conclusive. Wake the selector immediately;
        # it will still perform map, costmap, and Navfn validation before
        # publishing a replacement endpoint.
        self.last_planning_wall = 0.0
        self.publish_status(
            "execution_terminal_failure",
            route_id=route_id,
            route_kind=route_kind,
            mission_route_kind=mission_route_kind,
            status=status,
            status_text=status_name,
            reason=self.recovery_pending_reason,
            lease_phase=(
                "post_terminal_controller_cleanup"
                if released_after_logical_terminal
                else "active_route_failure"
            ),
        )
        rospy.logwarn(
            "Global frontier will replace route_id=%d after terminal action failure: %s",
            route_id,
            status_name,
        )
        return True

    def gather_scan(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.SCAN_UPDATED, message, self.apply_scan
        )

    def on_scan(self, message):
        """Retain recovery clearance and the current physical observation horizon."""
        ranges = []
        observed_ranges = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if (
                abs(angle) <= math.radians(30.0)
                and math.isfinite(value)
                and value > 0.01
            ):
                ranges.append(float(value))
            if math.isfinite(value) and value > float(message.range_min):
                observed_ranges.append(float(value))
        self.scan_forward_minimum = min(ranges) if ranges else float("nan")
        if math.isfinite(float(message.range_max)) and message.range_max > 0.0:
            self.scan_range_max = float(message.range_max)
        if observed_ranges:
            # The upper quartile is the robust geometric horizon reached by
            # this scan's real returns. It rejects a few close chair/wall hits
            # without treating the sensor's 15 m capability (or ``inf`` rays)
            # as evidence that every unseen room is one local task.
            observed_ranges.sort()
            index = int(round(0.75 * (len(observed_ranges) - 1)))
            self.scan_observation_horizon = observed_ranges[index]

    def gather_replan_request(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.REPLAN_REQUESTED, message, self.apply_replan_request
        )

    def on_replan_request(self, message):
        """Discard cached route ownership and rebuild from the current pose."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict) or payload.get("event") != "replan_request":
            return
        request_id = max(0, int(payload.get("request_id", 0) or 0))
        if request_id <= 0 or self.task_done:
            return
        portal_transaction = getattr(self, "portal_transaction", None)
        if (
            portal_transaction is not None
            and portal_transaction.active
            and not portal_transaction.preemption_allowed("target_replan")
        ):
            # A target update cannot interrupt a physical doorway crossing.
            # Keep the complete request so the semantic layer is not lost;
            # the planning timer replays it after the Portal transaction has
            # reached place_commit/finish.
            self.deferred_replan_request = dict(payload)
            self.publish_status(
                "replan_deferred_portal_transaction",
                request_id=request_id,
                reason=str(payload.get("reason", "unknown")),
                portal_transaction=portal_transaction.snapshot().state,
                portal_transaction_id=int(
                    portal_transaction.snapshot().transaction_id
                ),
            )
            return
        # Replanning is a route-identity boundary.  Before the reset below
        # clears active map fields, reconcile any durable WorkItem/Portal
        # Attempt owned by that route.  Otherwise the graph would retain an
        # ``active`` obligation forever and reject every later candidate.
        reconcile_lease = getattr(
            self, "reconcile_active_durable_lease_for_replan", None
        )
        if callable(reconcile_lease):
            lease_decision = reconcile_lease(
                str(payload.get("reason", "replan")),
                rospy.Time.now().to_sec(),
            )
            if getattr(lease_decision, "action", None) == "defer":
                self.deferred_replan_request = dict(payload)
                self.publish_status(
                    "replan_deferred_durable_route_lease",
                    request_id=request_id,
                    reason=str(payload.get("reason", "unknown")),
                    lease_reason=str(
                        getattr(lease_decision, "reason", "lease_deferred")
                    ),
                    route_id=int(getattr(lease_decision, "route_id", 0) or 0),
                )
                return
        old_goal = self.active_frontier
        # ``on_replan_request`` is also an execution boundary. Target-route
        # failures can release the bridge lease before this callback runs, so
        # ``release_active_frontier`` is not guaranteed to have cleared the
        # event-driven scheduler. If its old route identity survives here,
        # ``claim()`` rejects every successor wake and the runner appears to
        # wait for a map event that can never arrive. Release that scheduler
        # lease before clearing graph route state; the next planning cycle
        # still owns goal selection.
        decision_scheduler = getattr(self, "decision_wake_scheduler", None)
        active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        if decision_scheduler is not None and active_route_id > 0:
            released = decision_scheduler.finish_route(
                active_route_id, "replan_request"
            )
            if released:
                self.publish_status(
                    "decision_route_lease_released",
                    route_id=active_route_id,
                    reason="replan_request",
                    request_id=request_id,
                )
        hint = payload.get("semantic_hint_map")
        semantic_hint = None
        if isinstance(hint, (list, tuple)) and len(hint) >= 2:
            try:
                x, y = float(hint[0]), float(hint[1])
                if math.isfinite(x) and math.isfinite(y):
                    semantic_hint = (x, y)
            except (TypeError, ValueError):
                pass
        target_pursuit = bool(payload.get("target_pursuit", False)) and semantic_hint is not None
        target_room_claim = bool(payload.get("target_room_claim", False))
        target_reinspection = bool(payload.get("target_reinspection", False))
        target_room_claim_release = bool(
            payload.get("target_room_claim_release", False)
        )
        target_track_id = str(payload.get("target_track_id", "")).strip()
        target_release_accepted = bool(
            target_room_claim_release
            and not getattr(self, "target_region_claim_active", False)
        )
        if target_room_claim_release:
            target_observation_work = getattr(
                self, "target_observation_work", None
            )
            if target_observation_work is not None:
                released = target_observation_work.release(
                    self.current_task_version,
                    track_id=target_track_id,
                    now=rospy.Time.now().to_sec(),
                    reason=str(
                        payload.get(
                            "target_room_claim_release_reason", "target_lost"
                        )
                    ),
                )
                if released:
                    self.publish_status(
                        "target_observation_work_released",
                        target_track_id=target_track_id or None,
                        work=released,
                    )
        if target_room_claim_release and self.target_region_claim_active:
            active_track_id = str(self.target_region_claim_track_id).strip()
            if not target_track_id or target_track_id == active_track_id:
                self.clear_target_region_claim(
                    str(payload.get("target_room_claim_release_reason", "target_track_expired"))
                )
                target_release_accepted = True
            else:
                # Replan requests are asynchronous. An expired older visual
                # track must never unlock a newer target's observation room.
                self.publish_status(
                    "target_region_claim_release_ignored",
                    reason="track_mismatch",
                    requested_target_track_id=target_track_id,
                    active_target_track_id=active_track_id or None,
                )
        if target_room_claim_release:
            # Release is authoritative even if a queued reinspection request
            # carries the old ``target_reinspection=true`` bit.
            self.target_reinspection_pending = False
        elif target_reinspection:
            # This event-level obligation survives the asynchronous binding of
            # TargetObservationWork to a Place.  It is cleared only by an
            # accepted explicit release or task completion.
            reopen_target_observation = getattr(
                self, "_clear_target_observation_materialization_deferred", None
            )
            if callable(reopen_target_observation):
                reopen_target_observation(
                    getattr(self, "current_physical_place_id", None),
                    reason="explicit_target_reinspection",
                )
            self.target_reinspection_pending = True
        elif target_release_accepted:
            self.target_reinspection_pending = False
        self.pending_replan_request_id = request_id
        self.pending_replan_reason = str(payload.get("reason", "unknown"))
        self.pending_semantic_hint_map = semantic_hint
        self.pending_semantic_pursuit = target_pursuit
        # A release is a terminal ownership transition.  An asynchronous
        # replan may carry stale ``target_room_claim=true`` metadata from the
        # failed target request, but that must never recreate the claim in the
        # same callback after it was explicitly released.
        if target_room_claim and not target_room_claim_release:
            # The map can be mid-update while the callback runs. Bind the
            # claim to the current robot location on the next complete map
            # snapshot, before selecting any replacement route.
            self.target_region_claim_active = True
            self.target_region_claim_anchor_map = None
            self.target_region_claim_component = None
            self.target_region_claim_track_id = target_track_id
            self.target_region_claim_request_id = request_id
            self.target_region_claim_waiting_reported = False
            self.target_region_claim_bound_reported = False
            self.target_region_claim_cross_region_skips = 0
        elif target_room_claim_release:
            self.target_region_claim_anchor_map = None
            self.target_region_claim_component = None
            self.target_region_claim_waiting_reported = False
            self.target_region_claim_bound_reported = False
            self.target_region_claim_cross_region_skips = 0
        self.active_frontier = None
        self.active_frontier_component = None
        self.active_portal_gate_xy = None
        self.active_portal_gate_odom_xy = None
        self.active_portal_gate_approached_at = None
        self.active_portal_crossing_observed = False
        self.active_portal_crossing_preobserved = False
        self.active_portal_crossing_rejected = False
        self.active_frontier_region_id = None
        self.active_observation_session_started_at = None
        # A replan can preempt an action after its source-room doorway has
        # already been crossed. Keep the prepared departure until the next
        # complete map snapshot can confirm or reject that physical crossing.
        self.active_since = 0.0
        self.active_best_distance = None
        self.active_best_goal_distance = None
        self.active_best_path_distance = None
        self.active_progress_time = 0.0
        self.active_last_progress_signal = "none"
        self.active_last_robot_xy = None
        self.clear_active_route_history()
        self.active_visited_odom_cells.clear()
        self.active_unreachable_since = None
        self.active_last_waypoint_map = None
        self.active_last_waypoint_yaw = None
        self.active_route_kind = "frontier_endpoint"
        self.active_mission_route_kind = "frontier_endpoint"
        self.active_portal_probe_phase = ""
        self.active_terminal_received = False
        self.recovery_pending_route_id = 0
        self.recovery_pending_behavior = ""
        self.recovery_pending_reason = ""
        self.turn_connector_released = True
        self._clear_active_post_turn_watchdog()
        self.last_status_command_map = None
        self.last_status_command_yaw = None
        self.last_status_mission_map = None
        self.clear_prefetched_frontier()
        self.pending_local_egress = None
        self.local_egress_place_lease.clear()
        self.pending_portal_retry = None
        self.active_portal_retry = False
        self.active_local_egress_resumes_portal = False
        self.last_planning_wall = 0.0
        self.publish_status(
            "replan_acknowledged",
            replan_request_id=request_id,
            reason=self.pending_replan_reason,
            previous_goal=(
                None
                if old_goal is None
                else [round(float(old_goal[2]), 3), round(float(old_goal[3]), 3)]
            ),
            semantic_hint_map=(
                None if semantic_hint is None
                else [round(semantic_hint[0], 3), round(semantic_hint[1], 3)]
            ),
            target_pursuit=target_pursuit,
            target_room_claim=target_room_claim,
            target_reinspection=target_reinspection,
            target_room_claim_release=target_room_claim_release,
            target_track_id=target_track_id or None,
        )
        rospy.loginfo(
            "Global frontier accepted replan request id=%d reason=%s old=%s hint=%s target_pursuit=%s target_room_claim=%s release=%s track=%s",
            request_id, self.pending_replan_reason, old_goal, semantic_hint,
            target_pursuit, target_room_claim, target_room_claim_release,
            target_track_id or "-",
        )

    def replay_deferred_replan_if_ready(self):
        """Replay one queued semantic request after a Portal lease finishes."""
        payload = getattr(self, "deferred_replan_request", None)
        if not payload or self.task_done:
            return False
        portal_transaction = getattr(self, "portal_transaction", None)
        if portal_transaction is not None and portal_transaction.active:
            return False
        self.deferred_replan_request = None
        self.publish_status(
            "replan_replayed_after_portal_transaction",
            request_id=int(payload.get("request_id", 0) or 0),
        )
        self.apply_replan_request(SimpleNamespace(data=json.dumps(payload)))
        return True

    def gather_turn_status(self, message):
        return GlobalFrontierEventCallbacksMixin._enqueue_or_apply(
            self, EventType.TURN_STATUS, message, self.apply_turn_status
        )

    def on_turn_status(self, message):
        """Receive the execution adapter's atomic-turn state."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with (
            getattr(self, "planning_lock", None)
            if getattr(self, "planning_lock", None) is not None
            else nullcontext()
        ):
            self.turn_supervisor_state = (
                str(payload.get("state", "UNKNOWN")).strip().upper() or "UNKNOWN"
            )
            self.turn_supervisor_route_kind = str(
                payload.get("active_route_kind", payload.get("route_kind", ""))
            ).strip().lower()
            self.turn_supervisor_turn_phase = str(
                payload.get("turn_phase", "")
            ).strip().lower()
            raw_yaw_error = payload.get("yaw_error")
            try:
                self.turn_supervisor_yaw_error = (
                    None if raw_yaw_error is None else float(raw_yaw_error)
                )
            except (TypeError, ValueError):
                self.turn_supervisor_yaw_error = None
            raw_target_yaw = payload.get("target_yaw")
            try:
                self.turn_supervisor_target_yaw = (
                    None if raw_target_yaw is None else float(raw_target_yaw)
                )
            except (TypeError, ValueError):
                self.turn_supervisor_target_yaw = None
            turn_event = str(
                payload.get("turn_event", payload.get("event", ""))
            ).strip().lower()
            if turn_event == "turn_started":
                self.turn_connector_released = False
            elif turn_event == "turn_completed":
                self.turn_connector_released = True
                previous_key = payload.get("previous_key")
                if (
                    self.post_turn_stall_timeout is None
                    or self.active_frontier is None
                    or self.active_route_id <= 0
                    or not isinstance(previous_key, (list, tuple))
                    or len(previous_key) < 2
                ):
                    return
                try:
                    turn_goal_x = float(previous_key[0])
                    turn_goal_y = float(previous_key[1])
                except (TypeError, ValueError):
                    return
                endpoint_x = float(self.active_frontier[2])
                endpoint_y = float(self.active_frontier[3])
                if math.hypot(turn_goal_x - endpoint_x, turn_goal_y - endpoint_y) > 0.10:
                    return
                self.active_turn_completed_route_id = int(self.active_route_id)
                self.active_turn_completed_goal = (endpoint_x, endpoint_y)
                self.active_turn_completed_wall = now_for(self)
                self.active_turn_completed_odom_xy = (
                    None if self.pose_odom is None else (
                        float(self.pose_odom.x), float(self.pose_odom.y)
                    )
                )
                self.active_turn_completed_translation = 0.0
                self.active_turn_completed_launched = False
                # The normal progress clock may predate a multi-second atomic
                # turn. Restart it at the execution boundary so a successful
                # turn cannot immediately inherit an old stall deadline.
                self.active_progress_time = self.active_turn_completed_wall
                self.active_last_progress_signal = "post_turn_watchdog_armed"
                self.publish_status(
                    "post_turn_progress_watchdog_armed",
                    route_id=int(self.active_route_id),
                    goal=[round(endpoint_x, 3), round(endpoint_y, 3)],
                    timeout=round(float(self.post_turn_stall_timeout), 3),
                    odom_start=(
                        None if self.active_turn_completed_odom_xy is None
                        else [
                            round(self.active_turn_completed_odom_xy[0], 3),
                            round(self.active_turn_completed_odom_xy[1], 3),
                        ]
                    ),
                    launch_distance=round(float(self.progress_epsilon), 3),
                )

    def on_immediate_plan(self, event):
        del event
        return self._enqueue_lifecycle_event(
            EventType.REPLAN_REQUESTED, {"event": "immediate_plan"}
        )

    def apply_task(self, message):
        return GlobalFrontierEventCallbacksMixin.on_task(self, message)

    def apply_detections(self, message):
        return GlobalFrontierEventCallbacksMixin.on_detections(self, message)

    def apply_goal_arbitration(self, message):
        return GlobalFrontierEventCallbacksMixin.on_goal_arbitration(
            self, message
        )

    def apply_task_done(self, message):
        return GlobalFrontierEventCallbacksMixin.on_task_done(self, message)

    def apply_move_base_recovery(self, message):
        return GlobalFrontierEventCallbacksMixin.on_move_base_recovery(
            self, message
        )

    def apply_bridge_status(self, message):
        return GlobalFrontierEventCallbacksMixin.on_bridge_status(self, message)

    def apply_scan(self, message):
        return GlobalFrontierEventCallbacksMixin.on_scan(self, message)

    def apply_replan_request(self, message):
        return GlobalFrontierEventCallbacksMixin.on_replan_request(
            self, message
        )

    def apply_turn_status(self, message):
        return GlobalFrontierEventCallbacksMixin.on_turn_status(self, message)
