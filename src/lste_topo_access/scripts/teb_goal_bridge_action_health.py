"""Action health and target-failure policy for the TEB goal bridge.

The bridge composition root owns ROS wiring and route dispatch. This mixin
keeps cancellation, progress-state reset, handoff geometry, and target
failure latching together so those lifecycle rules can be edited safely.
The host provides publishers, action state, and status helpers.
"""

import copy
import json
import math

import rospy
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

from clock_provider import now_for


class TebGoalBridgeActionHealthMixin:
    _ROUTE_LEASE_WATCHDOG_SECONDS = 10.0
    _ACTION_TOMBSTONE_LIMIT = 128

    @staticmethod
    def _watchdog_int(value, default=0):
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return int(default)

    @staticmethod
    def _action_identity_key(route_id, action_generation):
        """Return the stable deduplication key for one transport action."""
        try:
            route_id = int(route_id or 0)
            action_generation = int(action_generation or 0)
        except (TypeError, ValueError):
            return None
        if action_generation <= 0:
            return None
        return route_id, action_generation

    def _action_identity_key_locked(self, action_contract=None):
        """Resolve one action key without synthesizing a new generation."""
        contract = action_contract
        if contract is None:
            pending = getattr(self, "pending_terminal_prepare", None)
            if isinstance(pending, dict):
                contract = pending.get("old_contract")
        if contract is None:
            contract = getattr(self, "active_action_contract", None)
        if contract is None:
            contract = getattr(self, "last_dispatch_identity", None)
        if hasattr(contract, "get"):
            route_id = contract.get(
                "route_id",
                getattr(self, "active_route_id", 0)
                or getattr(self, "latest_route_id", 0),
            )
            action_generation = contract.get(
                "generation",
                contract.get(
                    "action_generation",
                    getattr(self, "action_generation", 0),
                ),
            )
        else:
            route_id = getattr(self, "active_route_id", 0) or getattr(
                self, "latest_route_id", 0
            )
            action_generation = getattr(self, "action_generation", 0)
        return self._action_identity_key(route_id, action_generation)

    def _terminal_tombstone_for_key_locked(self, key):
        if key is None:
            return None
        records = getattr(self, "terminal_action_tombstones", None)
        if not isinstance(records, dict):
            return None
        return records.get(key)

    def _remember_terminal_tombstone_locked(self, key, record):
        """Retain bounded termination state for duplicate callback suppression."""
        if key is None or not isinstance(record, dict):
            return
        records = getattr(self, "terminal_action_tombstones", None)
        if not isinstance(records, dict):
            records = {}
            self.terminal_action_tombstones = records
        records[key] = record
        while len(records) > self._ACTION_TOMBSTONE_LIMIT:
            records.pop(next(iter(records)))

    def _released_action_key_seen_locked(self, key):
        if key is None:
            return False
        records = getattr(self, "released_action_tombstones", None)
        return isinstance(records, dict) and key in records

    def _remember_released_action_key_locked(self, key, reason):
        """Retain the exact release key so repeated ACKs cannot release twice."""
        if key is None:
            return
        records = getattr(self, "released_action_tombstones", None)
        if not isinstance(records, dict):
            records = {}
            self.released_action_tombstones = records
        records[key] = {"reason": str(reason or "")}
        while len(records) > self._ACTION_TOMBSTONE_LIMIT:
            records.pop(next(iter(records)))

    def _route_lease_watchdog_identity_locked(self, action_contract=None):
        """Build the immutable identity that a terminal route hands off."""
        contract = action_contract
        if contract is None:
            contract = getattr(self, "active_action_contract", None)
        if contract is None:
            contract = getattr(self, "last_dispatch_identity", None)

        if contract is not None:
            generation = self._watchdog_int(
                contract.get("generation", contract.get("action_generation", 0))
            )
            transaction_id = self._watchdog_int(
                contract.get("transaction_id", 0)
            )
            graph_transaction_id = self._watchdog_int(
                contract.get("graph_transaction_id", 0)
            )
            lifecycle_transaction_id = self._watchdog_int(
                contract.get("lifecycle_transaction_id", 0)
            )
            route_id = self._watchdog_int(contract.get("route_id", 0))
            raw_map_epoch = contract.get("map_epoch")
            raw_epoch = (
                raw_map_epoch
                if raw_map_epoch is not None
                else contract.get("epoch", contract.get("target_epoch", None))
            )
            epoch_source = (
                "map_epoch" if raw_map_epoch is not None else "target_epoch"
            )
            source = str(contract.get("source", "unknown") or "unknown")
            route_kind = str(contract.get("route_kind", "") or "")
            mission_route_kind = str(
                contract.get("mission_route_kind", "") or ""
            )
            priority = self._watchdog_int(contract.get("priority", 0))
        else:
            generation = self._watchdog_int(
                getattr(self, "action_generation", 0)
            )
            transaction_id = self._watchdog_int(
                getattr(self, "active_goal_transaction_id", 0)
                or getattr(self, "latest_goal_transaction_id", 0)
            )
            graph_transaction_id = self._watchdog_int(
                getattr(self, "active_graph_transaction_id", 0)
                or getattr(self, "latest_graph_transaction_id", 0)
            )
            lifecycle_transaction_id = self._watchdog_int(
                self._canonical_lifecycle_transaction_id()
            )
            route_id = self._watchdog_int(
                getattr(self, "active_route_id", 0)
                or getattr(self, "latest_route_id", 0)
            )
            raw_epoch = getattr(self, "active_route_map_epoch", None)
            epoch_source = "map_epoch"
            if raw_epoch is None:
                raw_epoch = getattr(self, "active_target_epoch", None)
                epoch_source = "target_epoch"
            source = str(
                getattr(self, "active_intent_source", "unknown") or "unknown"
            )
            route_kind = str(getattr(self, "active_route_kind", "") or "")
            mission_route_kind = str(
                getattr(self, "active_mission_route_kind", "") or ""
            )
            priority = self._watchdog_int(
                getattr(self, "active_intent_priority", 0)
            )

        try:
            epoch = None if raw_epoch is None else max(0, int(raw_epoch))
        except (TypeError, ValueError):
            epoch = None
        return {
            "route_id": route_id,
            "transaction_id": transaction_id,
            "graph_transaction_id": graph_transaction_id,
            "lifecycle_transaction_id": lifecycle_transaction_id,
            "action_generation": generation,
            "epoch": epoch,
            "epoch_source": epoch_source if epoch is not None else "unavailable",
            "map_epoch": raw_map_epoch if contract is not None else raw_epoch,
            "source": source.strip().lower() or "unknown",
            "route_kind": route_kind.strip().lower(),
            "mission_route_kind": mission_route_kind.strip().lower(),
            "graph_action": str(
                (contract.get("graph_action", "") if contract is not None
                 else getattr(self, "active_graph_action", ""))
                or ""
            ).strip().lower(),
            "graph_obligation_kind": str(
                (contract.get("graph_obligation_kind", "") if contract is not None
                 else getattr(self, "active_graph_obligation_kind", ""))
                or ""
            ).strip().lower(),
            "priority": priority,
        }

    def _shutdown_route_lease_watchdog_timer_locked(self):
        timer = getattr(self, "route_lease_watchdog_timer", None)
        self.route_lease_watchdog_timer = None
        if timer is None:
            return
        shutdown = getattr(timer, "shutdown", None)
        if callable(shutdown):
            try:
                shutdown()
            except Exception as exc:  # pragma: no cover - ROS shutdown edge
                rospy.logwarn(
                    "P0 route lease watchdog timer shutdown failed: "
                    "transaction_id=%d epoch=%s error=%s",
                    self._watchdog_int(
                        getattr(self, "route_lease_watchdog", {}).get(
                            "transaction_id", 0
                        )
                    ),
                    getattr(self, "route_lease_watchdog", {}).get("epoch"),
                    exc,
                )

    def _cancel_route_lease_watchdog_locked(self, reason):
        """Cancel the one pending route lease watchdog, if any."""
        record = getattr(self, "route_lease_watchdog", None)
        if record is None:
            return False
        self._shutdown_route_lease_watchdog_timer_locked()
        self.route_lease_watchdog = None
        self.publish_bridge_status(
            "route_lease_watchdog_cancelled",
            route_id=self._watchdog_int(record.get("route_id")),
            graph_transaction_id=self._watchdog_int(
                record.get("graph_transaction_id")
            ),
            transaction_id=self._watchdog_int(record.get("transaction_id")),
            action_generation=self._watchdog_int(
                record.get("action_generation")
            ),
            lifecycle_transaction_id=self._watchdog_int(
                record.get("lifecycle_transaction_id")
            ),
            epoch=record.get("epoch"),
            map_epoch=record.get("map_epoch"),
            epoch_source=str(record.get("epoch_source", "unavailable")),
            reason=str(reason),
            watchdog_record=dict(record),
            severity="INFO",
            priority="P0",
            machine_readable=True,
        )
        return True

    def _arm_route_lease_watchdog_locked(
        self, status=None, reason="move_base_terminal", action_contract=None
    ):
        """Arm one bounded handoff lease after a real MoveBase terminal."""
        identity = self._route_lease_watchdog_identity_locked(action_contract)
        identity_key = self._action_identity_key(
            identity.get("route_id"), identity.get("action_generation")
        )
        if identity["action_generation"] <= 0 and getattr(
            self, "last_dispatched_goal", None
        ) is None:
            return False
        existing = getattr(self, "route_lease_watchdog", None)
        if isinstance(existing, dict):
            existing_key = self._action_identity_key(
                existing.get("route_id"), existing.get("action_generation")
            )
            if existing_key == identity_key and identity_key is not None:
                self.publish_bridge_status(
                    "route_lease_watchdog_duplicate_ignored",
                    route_id=identity["route_id"],
                    transaction_id=identity["transaction_id"],
                    lifecycle_transaction_id=identity[
                        "lifecycle_transaction_id"
                    ],
                    action_generation=identity["action_generation"],
                    graph_transaction_id=identity["graph_transaction_id"],
                    map_epoch=identity.get("map_epoch"),
                    origin_epoch=identity.get("origin_epoch"),
                    projection_epoch=identity.get("projection_epoch"),
                    reason=str(reason),
                    dedupe_key=[int(identity_key[0]), int(identity_key[1])],
                )
                return True
        if existing is not None:
            self._cancel_route_lease_watchdog_locked("rearmed")

        now = float(now_for(self))
        try:
            status_code = None if status is None else int(status)
        except (TypeError, ValueError):
            status_code = None
        if status_code is None:
            status_text = "UNKNOWN"
        else:
            status_text = GoalStatus.to_string(status_code)
        record = dict(identity)
        record.update(
            {
                "identity_key": identity_key,
                "armed_at": now,
                "deadline": now + self._ROUTE_LEASE_WATCHDOG_SECONDS,
                "status": status_code,
                "status_text": status_text,
                "reason": str(reason),
            }
        )
        self.route_lease_watchdog = record
        self.route_lease_watchdog_timer = None
        try:
            is_shutdown = getattr(rospy, "is_shutdown", lambda: False)
            if not is_shutdown():
                self.route_lease_watchdog_timer = rospy.Timer(
                    rospy.Duration(self._ROUTE_LEASE_WATCHDOG_SECONDS),
                    self._route_lease_watchdog_timer_callback,
                    oneshot=True,
                )
        except Exception as exc:  # pragma: no cover - ROS startup edge
            # The regular lifecycle tick still enforces the deadline. Keep the
            # record armed so a timer construction failure cannot leak owner.
            rospy.logerr(
                "P0 route lease watchdog timer start failed: "
                "transaction_id=%d epoch=%s error=%s",
                identity["transaction_id"],
                identity["epoch"],
                exc,
            )
        self.publish_bridge_status(
            "route_lease_watchdog_armed",
            route_id=identity["route_id"],
            graph_transaction_id=identity["graph_transaction_id"],
            transaction_id=identity["transaction_id"],
            action_generation=identity["action_generation"],
            lifecycle_transaction_id=identity["lifecycle_transaction_id"],
            epoch=identity["epoch"],
            map_epoch=identity.get("map_epoch"),
            epoch_source=identity["epoch_source"],
            status=status_code,
            status_text=status_text,
            deadline=record["deadline"],
            timeout_seconds=self._ROUTE_LEASE_WATCHDOG_SECONDS,
            reason=str(reason),
            watchdog_record=dict(record),
            severity="WARN",
            priority="P0",
            machine_readable=True,
        )
        rospy.logwarn(
            "P0 route lease watchdog armed: route_id=%d transaction_id=%d "
            "action_generation=%d epoch=%s status=%s deadline=%.3f",
            identity["route_id"],
            identity["transaction_id"],
            identity["action_generation"],
            identity["epoch"],
            status_text,
            record["deadline"],
        )
        return True

    def _route_lease_watchdog_has_new_route_locked(self, record):
        """Return true when a successor route or dispatch has been observed."""
        captured_route = self._watchdog_int(record.get("route_id"))
        captured_transaction = self._watchdog_int(
            record.get("transaction_id")
        )
        captured_graph_transaction = self._watchdog_int(
            record.get("graph_transaction_id")
        )
        latest_route = self._watchdog_int(
            getattr(self, "latest_route_id", 0)
        )
        active_route = self._watchdog_int(
            getattr(self, "active_route_id", 0)
        )
        if latest_route > captured_route or active_route > captured_route:
            return True
        latest_transaction = self._watchdog_int(
            getattr(self, "latest_goal_transaction_id", 0)
        )
        if (
            latest_transaction > 0
            and latest_transaction > captured_transaction
        ):
            return True
        latest_lifecycle = self._watchdog_int(
            getattr(self, "latest_lifecycle_transaction_id", 0)
        )
        captured_lifecycle = self._watchdog_int(
            record.get("lifecycle_transaction_id")
        )
        if (
            captured_lifecycle > 0
            and latest_lifecycle > 0
            and latest_lifecycle != captured_lifecycle
        ):
            return True
        latest_graph_transaction = self._watchdog_int(
            getattr(self, "latest_graph_transaction_id", 0)
        )
        if (
            captured_graph_transaction > 0
            and latest_graph_transaction > 0
            and latest_graph_transaction != captured_graph_transaction
        ):
            return True
        if getattr(self, "latest_goal", None) is None:
            return False
        return bool(
            latest_route != captured_route
            or str(getattr(self, "latest_route_kind", "") or "").strip().lower()
            != str(record.get("route_kind", "") or "").strip().lower()
            or str(
                getattr(self, "latest_mission_route_kind", "") or ""
            ).strip().lower()
            != str(record.get("mission_route_kind", "") or "").strip().lower()
            or str(
                getattr(self, "latest_intent_source", "unknown") or "unknown"
            ).strip().lower()
            != str(record.get("source", "unknown") or "unknown").strip().lower()
            or self._watchdog_int(getattr(self, "latest_intent_priority", 0))
            != self._watchdog_int(record.get("priority"))
        )

    def _observe_route_lease_watchdog_status_locked(self, payload):
        """Consume explicit downstream acknowledgements for the old route."""
        record = getattr(self, "route_lease_watchdog", None)
        if not isinstance(payload, dict) or record is None:
            return False
        event = str(payload.get("event", "") or "").strip().lower()
        ack_events = {
            "frontier_route_unavailable",
            "frontier_prefetched",
            "replan_acknowledged",
            "terminal_replan_requested",
            "terminal_prefetch_promoted",
            "route_invalidated",
        }
        acknowledged = bool(
            payload.get("ack")
            or payload.get("acknowledged")
            or payload.get("accepted")
            or payload.get("successor_route_id")
        )
        route_ids = []
        for key in (
            "route_id",
            "acknowledged_route_id",
            "released_route_id",
            "pending_route_id",
            "successor_route_id",
        ):
            value = self._watchdog_int(payload.get(key))
            if value > 0:
                route_ids.append(value)
        captured_route = self._watchdog_int(record.get("route_id"))
        matching_route = not route_ids or captured_route in route_ids
        newer_route = any(value > captured_route for value in route_ids)
        explicit_ack = acknowledged and matching_route
        repeated_route_command = event == "route_command" and not newer_route
        if (event in ack_events or explicit_ack) and matching_route and not repeated_route_command:
            self._cancel_route_lease_watchdog_locked(
                "downstream_ack:%s" % (event or "unspecified")
            )
            return True
        if newer_route:
            self._cancel_route_lease_watchdog_locked("new_route_observed")
            return True
        return False

    def _route_lease_watchdog_timer_callback(self, _event):
        """Wake the lifecycle owner; this callback never mutates route state."""
        lifecycle = getattr(self, "lifecycle_manager", None)
        enqueue = getattr(lifecycle, "enqueue_type", None)
        if callable(enqueue):
            from lifecycle_manager import EventType

            enqueue(EventType.BRIDGE_WAKE, None)
            return
        with self.lock:
            self._tick_route_lease_watchdog_locked(now_for(self))

    def _tick_route_lease_watchdog_locked(self, now=None):
        """Run the watchdog state machine from the single lifecycle tick."""
        record = getattr(self, "route_lease_watchdog", None)
        if record is None:
            return False
        if self._route_lease_watchdog_has_new_route_locked(record):
            self._cancel_route_lease_watchdog_locked("new_route_or_dispatch")
            return False
        current = float(now_for(self) if now is None else now)
        if current < float(record.get("deadline", current)):
            return False
        return self._expire_route_lease_watchdog_locked(current)

    def _expire_route_lease_watchdog_locked(self, now=None):
        """Force release a terminal route whose downstream handoff is silent."""
        record = getattr(self, "route_lease_watchdog", None)
        if record is None:
            return False
        if self._route_lease_watchdog_has_new_route_locked(record):
            self._cancel_route_lease_watchdog_locked("new_route_or_dispatch")
            return False
        self._shutdown_route_lease_watchdog_timer_locked()
        self.route_lease_watchdog = None
        current = float(now_for(self) if now is None else now)
        route_id = self._watchdog_int(record.get("route_id"))
        transaction_id = self._watchdog_int(record.get("transaction_id"))
        action_generation = self._watchdog_int(
            record.get("action_generation")
        )
        epoch = record.get("epoch")
        action_contract = dict(record)
        if "generation" not in action_contract:
            action_contract["generation"] = action_generation
        termination = self._commit_termination(
            "route_lease_watchdog_expired",
            source_goal=getattr(self, "last_dispatched_goal", None),
            action_contract=action_contract,
            watchdog_reason="route_lease_watchdog_expired",
            publish_terminal=False,
            arm_watchdog=False,
        )
        transport_cancelled = bool(
            termination.get("transport_cancelled", False)
        )
        transport_error = termination.get("transport_error")
        # The watchdog is now closed; keep only the callback tombstone and
        # clear the retry record so a late result cannot arm it again.
        self.handoff_requested = False
        self.persistent_frontier_endpoint_pending = None
        self.frontier_observation_completion_pending = None
        self.frontier_continuous_prefetch_handoff_pending = None
        clear_request = getattr(
            self, "_clear_persistent_target_request_locked", None
        )
        if callable(clear_request) and getattr(
            self, "persistent_execution", False
        ):
            clear_request("route_lease_watchdog_expired", force=True)
        self.persistent_target_pending_transaction = 0
        self.persistent_target_pending_goal = None
        self.persistent_installed_target_goal = None
        self.persistent_installed_target_transaction = 0
        self.persistent_target_request_transaction = 0
        clear_health = getattr(self, "_clear_action_health_locked", None)
        if callable(clear_health):
            clear_health()
        self._clear_failed_route_lease_locked()

        source = str(record.get("source", "unknown") or "unknown")
        if source == "global_slam_frontier" and route_id > 0:
            self.frontier_lease_released_route_id = max(
                self._watchdog_int(
                    getattr(self, "frontier_lease_released_route_id", 0)
                ),
                route_id,
            )
            self.frontier_lease_released_reason = (
                "route_lease_watchdog_expired"
            )
            # The watchdog is the last owner of a terminal frontier route when
            # no explicit release request arrived. Close the semantic lease
            # with the immutable predecessor identity; otherwise Global
            # Frontier waits forever for the ACK that would authorize a
            # successor. The transport action has already gone through the
            # generation barrier above, so this is idempotent ownership
            # bookkeeping, not a second cancel.
            self._publish_frontier_release_ack_locked(
                route_id,
                self._route_lease_watchdog_identity_locked(action_contract),
                str(record.get("route_kind", "") or ""),
                release_phase="route_lease_watchdog_expired",
            )
            self.pending_lease_release_contract = None
        if self._watchdog_int(record.get("priority")) >= 2:
            self.target_lease_tombstone_transaction_id = max(
                self._watchdog_int(
                    getattr(self, "target_lease_tombstone_transaction_id", 0)
                ),
                transaction_id,
            )
            if epoch is not None:
                self.target_lease_tombstone_epoch = max(
                    self._watchdog_int(
                        getattr(self, "target_lease_tombstone_epoch", 0)
                    ),
                    self._watchdog_int(epoch),
                )

        # Keep transaction tombstones for stale-message rejection, but remove
        # all live route/intent ownership and goal geometry.
        self.latest_goal = None
        self.last_dispatched_goal = None
        self.last_dispatch_identity = None
        self.last_terminal_goal = None
        self.latest_intent_source = "waiting_global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_kind = ""
        self.latest_mission_route_kind = ""
        self.latest_route_id = 0
        self.latest_intent_goal = None
        self.latest_target_epoch = 0
        self.latest_target_track_id = ""
        self.latest_target_viewpoint_candidate_id = ""
        self.latest_target_viewpoint_attempt_id = ""
        self.latest_goal_context = {}
        self.intent_seen = False
        self.active_intent_source = "unknown"
        self.active_intent_priority = 0
        self.active_goal_transaction_id = 0
        self.active_route_kind = ""
        self.active_mission_route_kind = ""
        self.active_route_id = 0
        self.active_target_epoch = 0
        self.active_target_track_id = ""
        self.active_target_viewpoint_candidate_id = ""
        self.active_target_viewpoint_attempt_id = ""
        self.active_goal_context = {}
        self.persistent_target_terminal_boundary_transaction = 0
        self.last_result_status = record.get("status")
        self.last_result_monotonic = current
        self.publish_bridge_status(
            "route_lease_watchdog_expired",
            route_id=route_id,
            graph_transaction_id=self._watchdog_int(
                record.get("graph_transaction_id")
            ),
            transaction_id=transaction_id,
            action_generation=action_generation,
            lifecycle_transaction_id=self._watchdog_int(
                record.get("lifecycle_transaction_id")
            ),
            epoch=epoch,
            map_epoch=record.get("map_epoch"),
            epoch_source=str(record.get("epoch_source", "unavailable")),
            timeout_seconds=self._ROUTE_LEASE_WATCHDOG_SECONDS,
            armed_at=record.get("armed_at"),
            expired_at=current,
            status=record.get("status"),
            status_text=record.get("status_text"),
            reason=str(record.get("reason", "move_base_terminal")),
            severity="ERROR",
            priority="P0",
            machine_readable=True,
            watchdog_action="force_release",
            transport_cancelled=transport_cancelled,
            transport_error=transport_error,
            owner_revoked=True,
            route_intent_cleared=True,
            cleanup_complete=True,
            watchdog_record=dict(record),
        )
        rospy.logerr(
            "P0 route lease watchdog expired: route_id=%d transaction_id=%d "
            "action_generation=%d epoch=%s transport_cancelled=%s "
            "owner_revoked=true route_intent_cleared=true",
            route_id,
            transaction_id,
            action_generation,
            epoch,
            transport_cancelled,
        )
        return True

    def _clear_failed_route_lease_locked(self):
        """Forget a failed route only when a new lifecycle supersedes it."""
        self.failed_route_id = 0
        self.failed_route_source = "unknown"
        self.failed_route_priority = 0
        self.failed_route_kind = ""

    def _remember_failed_route_lease_locked(self, action_contract=None):
        """Retain route ownership after action-scoped metrics are cleared."""
        if action_contract is not None:
            self.failed_route_id = int(action_contract.get("route_id", 0) or 0)
            self.failed_route_source = str(
                action_contract.get("source", "unknown") or "unknown"
            )
            self.failed_route_priority = int(
                action_contract.get("priority", 0) or 0
            )
            self.failed_route_kind = str(
                action_contract.get("route_kind", "") or ""
            )
            return
        self.failed_route_id = int(getattr(self, "active_route_id", 0) or 0)
        self.failed_route_source = str(
            getattr(self, "active_intent_source", "unknown") or "unknown"
        )
        self.failed_route_priority = int(
            getattr(self, "active_intent_priority", 0) or 0
        )
        self.failed_route_kind = str(
            getattr(self, "active_route_kind", "") or ""
        )

    def _release_failed_target_controller_lease_locked(self, reason):
        """Prevent a failed target command from being replayed by the bridge.

        A target failure is a terminal ownership event.  Clear the pending
        target intent and, when the persistent action is still alive (for
        example a Navfn plan failed before MoveBase produced a result), cancel
        that transport lease as well.  The transaction identity is retained as
        a tombstone so latched/queued copies cannot resurrect the failed target.
        """
        target_owned = bool(
            int(getattr(self, "active_intent_priority", 0) or 0) >= 2
            or int(getattr(self, "latest_intent_priority", 0) or 0) >= 2
            or bool(getattr(self, "target_failure_latched", False))
        )
        if not target_owned:
            return False
        cancel_watchdog = getattr(
            self, "_cancel_route_lease_watchdog_locked", None
        )
        if callable(cancel_watchdog):
            cancel_watchdog("target_lease_released")
        route_id = int(
            getattr(self, "active_route_id", 0)
            or getattr(self, "latest_route_id", 0)
            or 0
        )
        target_transaction_id = max(
            int(getattr(self, "active_goal_transaction_id", 0) or 0),
            int(getattr(self, "latest_goal_transaction_id", 0) or 0),
            int(getattr(self, "persistent_target_pending_transaction", 0) or 0),
            int(getattr(self, "persistent_installed_target_transaction", 0) or 0),
        )
        target_epoch = max(
            int(getattr(self, "active_target_epoch", 0) or 0),
            int(getattr(self, "latest_target_epoch", 0) or 0),
            int(getattr(self, "target_failure_epoch", 0) or 0),
        )
        track_id = str(
            getattr(self, "active_target_track_id", "")
            or getattr(self, "latest_target_track_id", "")
            or getattr(self, "target_failure_track_id", "")
            or ""
        )
        self.target_lease_tombstone_transaction_id = max(
            int(getattr(self, "target_lease_tombstone_transaction_id", 0) or 0),
            target_transaction_id,
        )
        self.target_lease_tombstone_epoch = max(
            int(getattr(self, "target_lease_tombstone_epoch", 0) or 0),
            target_epoch,
        )
        if track_id:
            self.target_lease_tombstone_track_id = track_id

        transport_cancelled = False
        # A persistent planner can report an unreachable target while the
        # single MoveBase action is still ACTIVE.  Route the transport close
        # through the generation barrier so the late PREEMPTED result is
        # diagnostic only.
        if self.action_active:
            termination = self._commit_termination(
                "target_controller_lease_release",
                source_goal=getattr(self, "last_dispatched_goal", None),
                action_contract=getattr(self, "active_action_contract", None),
                watchdog_reason="target_controller_lease_release",
                publish_terminal=False,
                arm_watchdog=False,
            )
            transport_cancelled = bool(
                termination.get("transport_cancelled", False)
            )
            self.action_active = False
            self.last_result_status = GoalStatus.PREEMPTED
            self.last_result_monotonic = now_for(self)
        if self.persistent_execution:
            clear_request = getattr(
                self, "_clear_persistent_target_request_locked", None
            )
            if callable(clear_request):
                clear_request(str(reason), force=True)
            self.persistent_target_pending_transaction = 0
            self.persistent_target_pending_goal = None
            self.persistent_installed_target_goal = None
            self.persistent_installed_target_transaction = 0
        self.latest_goal = None
        self.last_dispatched_goal = None
        self.last_dispatch_identity = None
        self.last_terminal_goal = None
        self.latest_intent_source = "waiting_global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_kind = ""
        self.latest_mission_route_kind = ""
        self.latest_route_id = 0
        self.latest_intent_goal = None
        self.latest_target_epoch = 0
        self.latest_target_track_id = ""
        self.latest_target_viewpoint_candidate_id = ""
        self.latest_target_viewpoint_attempt_id = ""
        self.latest_goal_context = {}
        # A late terminal callback after this release is diagnostic only; it
        # must not schedule another handoff from the already-cancelled lease.
        self.handoff_requested = False
        self.publish_bridge_status(
            "target_controller_lease_released",
            route_id=route_id,
            target_track_id=track_id,
            reason=str(reason),
            controller_lease="released",
            next_owner="global_slam_frontier",
            target_transaction_id=target_transaction_id,
            target_epoch=target_epoch,
            transport_cancelled=transport_cancelled,
            tombstone_transaction_id=int(
                self.target_lease_tombstone_transaction_id
            ),
        )
        return True

    def _release_frontier_controller_lease_locked(
        self,
        route_id,
        reason,
        publish_release_ack=False,
        release_contract=None,
    ):
        """Atomically release a stale frontier action by route identity.

        Global Frontier can finish its planning lease before the persistent
        MoveBase action reaches a terminal callback.  In that interval the
        bridge must cancel the old controller lease and remember which route
        was released.  The tombstone is consumed only by a newer route ID;
        coordinates and retry timers never authorize the old route again.
        """
        try:
            route_id = max(0, int(route_id or 0))
        except (TypeError, ValueError):
            route_id = 0
        if route_id <= 0:
            return False
        active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        latest_route_id = int(getattr(self, "latest_route_id", 0) or 0)
        explicit_contract = (
            release_contract if hasattr(release_contract, "get") else None
        )
        explicit_route_id = self._watchdog_int(
            None if explicit_contract is None
            else explicit_contract.get("route_id")
        )
        if route_id < active_route_id:
            return False
        if (
            route_id < latest_route_id
            and explicit_route_id != route_id
        ):
            return False
        active_source = str(
            getattr(self, "active_intent_source", "unknown") or "unknown"
        )
        active_kind = str(getattr(self, "active_route_kind", "") or "")
        was_active = bool(self.action_active)
        if explicit_contract is not None:
            release_contract = explicit_contract
        else:
            release_contract = getattr(self, "active_action_contract", None)
        if release_contract is None:
            pending = getattr(self, "pending_terminal_prepare", None)
            if isinstance(pending, dict):
                release_contract = pending.get("old_contract")
        if release_contract is None:
            release_contract = getattr(self, "last_dispatch_identity", None)
        if release_contract is None:
            release_contract = {
                "route_id": route_id,
                "generation": int(getattr(self, "action_generation", 0) or 0),
            }
        release_key = self._action_identity_key_locked(release_contract)
        if release_key is not None and int(release_key[0]) not in (0, route_id):
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                route_id=route_id,
                action_generation=int(release_key[1]),
                reason="release_contract_route_mismatch",
                contract_route_id=int(release_key[0]),
            )
            return False
        if self._released_action_key_seen_locked(release_key):
            if publish_release_ack:
                release_identity = self._route_lease_watchdog_identity_locked(
                    release_contract
                )
                self._publish_frontier_release_ack_locked(
                    route_id,
                    release_identity,
                    str(
                        release_identity.get("route_kind")
                        or active_kind
                        or ""
                    ),
                    release_phase="forced_controller_release_duplicate",
                )
            self.publish_bridge_status(
                "controller_lease_release_duplicate_ignored",
                route_id=route_id,
                action_generation=(
                    0 if release_key is None else int(release_key[1])
                ),
                reason="release_key_already_applied",
                dedupe_key=(
                    None
                    if release_key is None
                    else [int(release_key[0]), int(release_key[1])]
                ),
            )
            return bool(publish_release_ack)
        release_identity = self._route_lease_watchdog_identity_locked(
            release_contract
        )
        cancel_watchdog = getattr(
            self, "_cancel_route_lease_watchdog_locked", None
        )
        if callable(cancel_watchdog):
            cancel_watchdog("frontier_lease_released")
        self.frontier_lease_released_route_id = max(
            int(getattr(self, "frontier_lease_released_route_id", 0) or 0),
            route_id,
        )
        self.frontier_lease_released_reason = str(reason)
        self._commit_termination(
            "frontier_controller_release",
            source_goal=getattr(self, "last_dispatched_goal", None),
            action_contract=release_contract,
            watchdog_reason="frontier_controller_release",
            publish_terminal=False,
            arm_watchdog=False,
        )
        # This method is the Bridge-side application of the upstream semantic
        # release request. The transport prepare may remain pending until
        # MoveBase reports terminal, but the separate semantic tombstone has
        # already been consumed and must not block the successor indefinitely.
        self.pending_lease_release_contract = None
        self.handoff_requested = False
        self.frontier_observation_completion_pending = None
        self.frontier_continuous_prefetch_handoff_pending = None
        self._clear_target_failure_locked("frontier_route_unavailable")
        self._clear_action_health_locked()
        self._clear_failed_route_lease_locked()
        self.last_result_status = GoalStatus.PREEMPTED
        self.last_result_monotonic = now_for(self)
        self._remember_released_action_key_locked(release_key, reason)
        self.publish_bridge_status(
            "controller_lease_released",
            route_id=route_id,
            released_route_id=route_id,
            reason=str(reason),
            was_active=was_active,
            active_route_id=active_route_id,
            latest_route_id=latest_route_id,
            active_source=active_source,
            active_route_kind=active_kind,
            controller_lease="released",
            transaction_id=release_identity["transaction_id"],
            lifecycle_transaction_id=release_identity[
                "lifecycle_transaction_id"
            ],
            action_generation=release_identity["action_generation"],
            new_action_generation=int(
                getattr(self, "action_generation", 0) or 0
            ),
            graph_transaction_id=release_identity["graph_transaction_id"],
            map_epoch=release_identity.get("map_epoch"),
            origin_epoch=release_identity.get("origin_epoch"),
            projection_epoch=release_identity.get("projection_epoch"),
            dedupe_key=(
                None
                if release_key is None
                else [int(release_key[0]), int(release_key[1])]
            ),
        )
        if publish_release_ack:
            self._publish_frontier_release_ack_locked(
                route_id,
                release_identity,
                str(
                    release_identity.get("route_kind") or active_kind or ""
                ),
                release_phase="forced_controller_release",
            )
        rospy.logwarn(
            "TEB goal bridge released unavailable frontier route_id=%d: %s",
            route_id,
            reason,
        )
        return True

    def _publish_frontier_release_ack_locked(
        self, route_id, release_identity, released_route_kind, release_phase
    ):
        """Publish an idempotent ACK for a forced frontier release."""
        released_controller_route = {
            "route_id": int(route_id),
            "route_kind": str(released_route_kind or ""),
            "transaction_id": int(release_identity.get("transaction_id", 0) or 0),
            "lifecycle_transaction_id": int(
                release_identity.get("lifecycle_transaction_id", 0) or 0
            ),
            "action_generation": int(
                release_identity.get("action_generation", 0) or 0
            ),
            "graph_transaction_id": int(
                release_identity.get("graph_transaction_id", 0) or 0
            ),
            "map_epoch": release_identity.get("map_epoch"),
            "origin_epoch": release_identity.get("origin_epoch"),
            "projection_epoch": release_identity.get("projection_epoch"),
        }
        self.pending_successor_release_ack = dict(released_controller_route)
        self.publish_bridge_status(
            "lease_release_ack",
            released_route_id=int(route_id),
            released_route_kind=str(released_route_kind or ""),
            route_id=int(route_id),
            transaction_id=released_controller_route["transaction_id"],
            lifecycle_transaction_id=released_controller_route[
                "lifecycle_transaction_id"
            ],
            action_generation=released_controller_route["action_generation"],
            new_action_generation=int(getattr(self, "action_generation", 0) or 0),
            graph_transaction_id=released_controller_route[
                "graph_transaction_id"
            ],
            map_epoch=released_controller_route["map_epoch"],
            origin_epoch=released_controller_route["origin_epoch"],
            projection_epoch=released_controller_route["projection_epoch"],
            controller_lease="released",
            release_phase=str(release_phase or "forced_controller_release"),
            released_controller_route=released_controller_route,
        )

    def cancel_locked(self, reason):
        cancel_watchdog = getattr(
            self, "_cancel_route_lease_watchdog_locked", None
        )
        if callable(cancel_watchdog):
            cancel_watchdog("cancel:%s" % reason)
        was_active = bool(self.action_active)
        self._commit_termination(
            "cancel:%s" % reason,
            source_goal=getattr(self, "last_dispatched_goal", None),
            action_contract=getattr(self, "active_action_contract", None),
            watchdog_reason="cancel:%s" % reason,
            publish_terminal=False,
            arm_watchdog=False,
        )
        self.handoff_requested = False
        self.persistent_frontier_endpoint_pending = None
        self.persistent_target_terminal_boundary_transaction = 0
        self.frontier_observation_completion_pending = None
        self.frontier_continuous_prefetch_handoff_pending = None
        self._clear_target_failure_locked("cancel:%s" % reason)
        self._clear_action_health_locked()
        self._clear_failed_route_lease_locked()
        self.last_result_status = GoalStatus.PREEMPTED
        self.last_result_monotonic = now_for(self)
        self.publish_bridge_status(
            "cancel",
            reason=reason,
            was_active=was_active,
        )
        rospy.loginfo("TEB goal bridge cancelled move_base action: reason=%s", reason)

    def _clear_action_health_locked(self):
        invalidate_feedback = getattr(
            self, "_invalidate_teb_feedback_locked", None
        )
        if callable(invalidate_feedback):
            feedback_already_invalid = bool(
                not getattr(self, "teb_feedback_valid", False)
                and getattr(self, "teb_feedback_requires_fresh_planner_command", False)
                and getattr(self, "latest_teb_selected_linear", None) is None
            )
            invalidation_reason = (
                str(
                    getattr(
                        self, "teb_feedback_invalid_reason", "action_lease_cleared"
                    )
                    or "action_lease_cleared"
                )
                if feedback_already_invalid
                else "action_lease_cleared"
            )
            invalidate_feedback(invalidation_reason, clear_planner=True)
        self.active_goal_global = None
        self.active_feedback_distance = None
        self.active_feedback_pose = None
        self.active_feedback_frame = ""
        self.active_best_distance = None
        self.active_progress_monotonic = 0.0
        self.active_motion_reference = None
        self.active_motion_progress_monotonic = 0.0
        self.active_navfn_plan_points = []
        self.active_navfn_plan_endpoint = None
        self.active_navfn_remaining = None
        self.active_navfn_best_remaining = None
        self.active_navfn_progress_monotonic = 0.0
        self.last_feedback_monotonic = 0.0
        self._reset_teb_reorientation_locked()
        self.frontier_stale_wait_started_monotonic = 0.0
        self.turn_transition_ready = False
        self.move_base_terminal_pending = False
        self.active_action_contract = None
        self.active_intent_source = "unknown"
        self.active_intent_priority = 0
        self.active_goal_transaction_id = 0
        self.active_route_kind = ""
        self.active_route_id = 0
        self.active_route_map_epoch = None
        self.active_graph_transaction_id = 0
        self.active_target_epoch = 0
        self.active_target_track_id = ""

    def _pending_goal_delta_locked(self):
        """Return the pending-vs-active distance in the source goal frame.

        Normal LSTE goals are odom-frame poses, so this is a cheap Euclidean
        comparison.  For a caller using another frame, transform the pending
        goal once and compare it to the already transformed action goal.
        """
        if self.latest_goal is None or self.last_dispatched_goal is None:
            return 0.0
        if (
            (self.latest_goal.header.frame_id or "")
            == (self.last_dispatched_goal.header.frame_id or "")
        ):
            return math.hypot(
                self.latest_goal.pose.position.x
                - self.last_dispatched_goal.pose.position.x,
                self.latest_goal.pose.position.y
                - self.last_dispatched_goal.pose.position.y,
            )
        pending_global = self._goal_in_global_frame(self.latest_goal)
        if pending_global is None or self.active_goal_global is None:
            return 0.0
        return math.hypot(
            pending_global.pose.position.x - self.active_goal_global.pose.position.x,
            pending_global.pose.position.y - self.active_goal_global.pose.position.y,
        )

    def _pending_heading_delta_locked(self):
        """Return the bearing change from the active to pending goal.

        This is deliberately measured at the latest move_base feedback pose,
        rather than at the robot's initial pose.  A branch can be far away but
        still be a smooth continuation; only the local turn required at the
        handoff should block an in-place replacement.
        """
        if self.latest_goal is None or self.active_goal_global is None:
            return None
        feedback = self.active_feedback_pose
        if feedback is None:
            return None
        pending_global = self._goal_in_global_frame(self.latest_goal)
        if pending_global is None:
            return None
        base_x, base_y, base_yaw = feedback
        active_bearing = math.atan2(
            self.active_goal_global.pose.position.y - base_y,
            self.active_goal_global.pose.position.x - base_x,
        )
        pending_bearing = math.atan2(
            pending_global.pose.position.y - base_y,
            pending_global.pose.position.x - base_x,
        )
        return abs(self._angle_delta(pending_bearing, active_bearing))

    def _frontier_prefetch_heading_delta_locked(self, source_goal, successor):
        """Return the local turn needed to enter a prefetched frontier branch.

        A prefetched point is intentionally not promoted to ``latest_goal``
        until its source observation region is terminal.  Consequently the
        generic ``_pending_heading_delta_locked`` cannot evaluate it.  Compare
        both bearings at the latest action feedback pose instead: this is the
        turn TEB would need to absorb if the action were replaced in-place.
        """
        feedback = self.active_feedback_pose
        if feedback is None:
            return None, "feedback_unavailable"
        # This route tangent is calculated by GlobalFrontier from the actual
        # connected BFS path, with the same map/costmap validation that admits
        # the successor. Compare it directly to the current base yaw at the
        # action boundary. The old endpoint-bearing calculation below is only
        # a compatibility fallback for an external/older frontier publisher.
        if self.prefetched_frontier_entry_yaw is not None:
            _base_x, _base_y, base_yaw = feedback
            return (
                abs(self._angle_delta(self.prefetched_frontier_entry_yaw, base_yaw)),
                self.prefetched_frontier_entry_yaw_basis or "bfs_initial_tangent",
            )
        if source_goal is None or successor is None:
            return None, "endpoint_bearing_unavailable"
        source_global = self._goal_in_global_frame(source_goal)
        if source_global is None:
            return None, "endpoint_bearing_unavailable"
        try:
            successor_x, successor_y = float(successor[0]), float(successor[1])
        except (TypeError, ValueError, IndexError):
            return None, "endpoint_bearing_unavailable"
        if not (math.isfinite(successor_x) and math.isfinite(successor_y)):
            return None, "endpoint_bearing_unavailable"
        base_x, base_y, _base_yaw = feedback
        active_bearing = math.atan2(
            source_global.pose.position.y - base_y,
            source_global.pose.position.x - base_x,
        )
        successor_bearing = math.atan2(
            successor_y - base_y,
            successor_x - base_x,
        )
        return abs(self._angle_delta(successor_bearing, active_bearing)), "legacy_endpoint_bearing"

    def _navfn_entry_tangent_locked(self, path, feedback_xy):
        """Return the actual Navfn entry heading sampled from live feedback.

        Frontier prefetch metadata is useful for choosing a candidate, but it
        can be stale by the time the bridge receives a new Navfn plan.  This
        helper samples the plan returned by the admission request itself, so
        the handoff state describes the route passed to PersistentTEB.
        """
        if path is None or len(path.poses) < 2 or feedback_xy is None:
            return None
        start_x, start_y = float(feedback_xy[0]), float(feedback_xy[1])
        previous_x, previous_y = start_x, start_y
        travelled = 0.0
        sample_distance = self.persistent_frontier_entry_tangent_distance
        for pose in path.poses:
            point = pose.pose.position
            point_x, point_y = float(point.x), float(point.y)
            segment = math.hypot(point_x - previous_x, point_y - previous_y)
            if segment <= 1e-6:
                continue
            if travelled + segment >= sample_distance:
                fraction = (sample_distance - travelled) / segment
                sample_x = previous_x + fraction * (point_x - previous_x)
                sample_y = previous_y + fraction * (point_y - previous_y)
                if math.hypot(sample_x - start_x, sample_y - start_y) > 1e-4:
                    return math.atan2(sample_y - start_y, sample_x - start_x)
            travelled += segment
            previous_x, previous_y = point_x, point_y
        # Short admissible routes still need a deterministic classification.
        # Their final point is the furthest real heading evidence available.
        if math.hypot(previous_x - start_x, previous_y - start_y) > 1e-4:
            return math.atan2(previous_y - start_y, previous_x - start_x)
        return None

    def _sharp_frontier_recovery_heading_locked(self, pending_delta):
        """Return the pending branch turn if stale recovery is near its end.

        A frontier branch is allowed to replace an active action in-place only
        when the replacement is a short continuation (the segment handoff
        path above). A large or sharp branch still has to change direction,
        but when the old endpoint is already close, an explicit cancel creates
        an avoidable zero-velocity pulse. This helper identifies exactly that
        narrow recovery case; it never relaxes collision or progress checks.
        """
        if (
            self.active_intent_priority != 0
            or self.latest_intent_priority != 0
            or self.latest_intent_source != "global_slam_frontier"
            or self.active_feedback_distance is None
            or self.active_feedback_distance > self.frontier_stale_recovery_max_distance
        ):
            return None
        heading_delta = self._pending_heading_delta_locked()
        if (
            pending_delta <= self.frontier_early_handoff_max_delta
            and (
                heading_delta is None
                or heading_delta <= self.frontier_early_handoff_max_heading_delta
            )
        ):
            return None
        # A missing feedback pose should not disable the distance gate for a
        # clearly oversized branch; zero here means "heading unavailable" in
        # the diagnostic payload, while the branch-size condition still holds.
        return heading_delta if heading_delta is not None else 0.0

    def _latch_target_failure_locked(
        self, reason, status_text="NO_PROGRESS", cancel_action=True,
        action_contract=None,
    ):
        """End a target transaction and notify mission arbitration exactly once.

        A target is an observation-backed hypothesis, not an action that can be
        retried forever. Once its Navfn route has stopped making route progress
        (or, before a plan is available, the base has stopped moving), this
        method emits one failure event and lets Goal Manager select a map route.
        The bridge-side latch prevents the terminal callback/timer race from
        reissuing the same target.
        """
        target_priority = (
            int(action_contract.get("priority", 0) or 0)
            if action_contract is not None
            else int(self.active_intent_priority)
        )
        if target_priority < 2:
            return False
        failure_generation = (
            int(action_contract.get("generation", self.action_generation))
            if action_contract is not None
            else int(self.action_generation)
        )
        source_goal = copy.deepcopy(
            action_contract.get("source_goal")
            if action_contract is not None
            else self.last_dispatched_goal
        )
        active_goal = (
            action_contract.get("execution_goal")
            if action_contract is not None
            else self.active_goal_global
        )
        target_epoch = (
            int(action_contract.get("target_epoch", 0) or 0)
            if action_contract is not None
            else int(self.active_target_epoch)
        )
        target_track_id = (
            str(action_contract.get("target_track_id", "") or "")
            if action_contract is not None
            else self.active_target_track_id
        )
        # MoveBase may invoke a second terminal callback after the bridge has
        # already cancelled/released the target action. The dispatch
        # generation changes during that release, so generation equality alone
        # does not deduplicate it. The target track/epoch is the immutable
        # semantic identity and must suppress that late callback as well.
        same_failure_generation = bool(
            getattr(self, "target_failure_latched", False)
            and int(getattr(self, "target_failure_generation", 0) or 0)
            == failure_generation
        )
        same_target_identity = bool(
            getattr(self, "target_failure_latched", False)
            and target_epoch > 0
            and bool(target_track_id)
            and int(getattr(self, "target_failure_epoch", 0) or 0)
            == target_epoch
            and str(getattr(self, "target_failure_track_id", "") or "")
            == target_track_id
        )
        if same_failure_generation or same_target_identity:
            self.publish_bridge_status(
                "target_failure_duplicate_ignored",
                reason=str(reason),
                target_epoch=target_epoch,
                target_track_id=target_track_id,
                failure_generation=failure_generation,
                latched_generation=int(
                    getattr(self, "target_failure_generation", 0) or 0
                ),
            )
            return False
        target_viewpoint_candidate_id = (
            str(
                action_contract.get("target_viewpoint_candidate_id", "") or ""
            )
            if action_contract is not None
            else str(
                getattr(self, "active_target_viewpoint_candidate_id", "") or ""
            )
        )
        target_viewpoint_attempt_id = (
            str(action_contract.get("target_viewpoint_attempt_id", "") or "")
            if action_contract is not None
            else str(
                getattr(self, "active_target_viewpoint_attempt_id", "") or ""
            )
        )
        now = now_for(self)
        progress_basis, target_progress_age = self._target_progress_state_locked(now)
        self.target_failure_latched = True
        self.target_failure_goal = source_goal
        self.target_failure_epoch = target_epoch
        self.target_failure_track_id = target_track_id
        self.target_failure_generation = failure_generation
        self.target_failure_count += 1
        self.handoff_requested = True
        payload = {
            "event": "target_route_failed",
            "reason": str(reason),
            "status": str(status_text),
            "goal": (
                None
                if source_goal is None
                else [
                    round(float(source_goal.pose.position.x), 4),
                    round(float(source_goal.pose.position.y), 4),
                ]
            ),
            "goal_frame": (
                None if source_goal is None else source_goal.header.frame_id
            ),
            "active_goal": (
                None
                if active_goal is None
                else [
                    round(float(active_goal.pose.position.x), 4),
                    round(float(active_goal.pose.position.y), 4),
                ]
            ),
            "active_goal_frame": (
                None
                if active_goal is None else active_goal.header.frame_id
            ),
            "target_epoch": target_epoch,
            "target_track_id": target_track_id,
            "target_transaction_id": int(
                getattr(self, "latest_goal_transaction_id", 0) or 0
            ),
            "target_viewpoint_candidate_id": target_viewpoint_candidate_id,
            "target_viewpoint_attempt_id": target_viewpoint_attempt_id,
            "failure_count": int(self.target_failure_count),
            "feedback_distance": (
                None
                if self.active_feedback_distance is None
                else round(float(self.active_feedback_distance), 4)
            ),
            "progress_age": (
                None
                if self.active_progress_monotonic <= 0.0
                else round(float(now - self.active_progress_monotonic), 4)
            ),
            "target_progress_basis": progress_basis,
            "target_progress_age": (
                None
                if not math.isfinite(target_progress_age)
                else round(float(target_progress_age), 4)
            ),
            "navfn_path_remaining": (
                None
                if self.active_navfn_remaining is None
                else round(float(self.active_navfn_remaining), 4)
            ),
            "motion_progress_age": (
                None
                if self.active_motion_progress_monotonic <= 0.0
                else round(float(now - self.active_motion_progress_monotonic), 4)
            ),
        }
        # Release the controller lease before notifying GoalManager. ROS topic
        # delivery is asynchronous but can still reach the manager before this
        # callback returns; publishing the failure first lets a replacement
        # frontier race with an action that is still marked target-owned.
        released = self._release_failed_target_controller_lease_locked(reason)
        payload.update(
            {
                "controller_lease": "released" if released else "unchanged",
                "target_lease_tombstone_transaction_id": int(
                    getattr(self, "target_lease_tombstone_transaction_id", 0)
                    or 0
                ),
                "target_lease_tombstone_epoch": int(
                    getattr(self, "target_lease_tombstone_epoch", 0) or 0
                ),
                "target_lease_tombstone_track_id": str(
                    getattr(self, "target_lease_tombstone_track_id", "") or ""
                ),
            }
        )
        self.target_failure_pub.publish(
            String(data=json.dumps(payload, sort_keys=True))
        )
        self.publish_bridge_status(
            "target_route_failed",
            reason=str(reason),
            status_text=str(status_text),
            target_epoch=target_epoch,
            target_track_id=target_track_id,
            target_viewpoint_candidate_id=target_viewpoint_candidate_id,
            target_viewpoint_attempt_id=target_viewpoint_attempt_id,
            failure_count=int(self.target_failure_count),
            feedback_distance=payload["feedback_distance"],
            progress_age=payload["progress_age"],
            target_progress_basis=progress_basis,
            target_progress_age=payload["target_progress_age"],
            navfn_path_remaining=payload["navfn_path_remaining"],
            motion_progress_age=payload["motion_progress_age"],
            lifecycle="release_to_frontier",
            controller_lease="released" if released else "unchanged",
        )
        rospy.logwarn(
            "TEB goal bridge released failed target to mission layer: "
            "reason=%s status=%s goal=%s epoch=%d",
            reason,
            status_text,
            payload["goal"],
            target_epoch,
        )
        return True
