"""Mission-intent ownership for the TEB action bridge."""

import json
import math

from goal_context import goal_context_identity, normalize_goal_context


class TebGoalBridgeIntentMixin:
    def _intent_signature_locked(self):
        """Return the pending mission transaction identity.

        Position equality alone is insufficient for a route connector: the
        same map cell can carry a different execution contract.  Conversely,
        the exact same latched goal and intent must be a no-op while active.
        """
        goal = self.latest_goal
        if goal is None:
            return None
        return (
            (goal.header.frame_id or "").strip().lstrip("/"),
            round(float(goal.pose.position.x), 3),
            round(float(goal.pose.position.y), 3),
            self.latest_intent_source,
            int(self.latest_intent_priority),
            self.latest_route_kind,
            int(self.latest_route_id),
            int(self.latest_target_epoch),
            goal_context_identity(self.latest_goal_context),
        )

    def _clear_target_failure_locked(self, reason):
        """Release a failed-target latch after a real mission transition."""
        if not self.target_failure_latched:
            return
        previous_goal = self.target_failure_goal
        previous_epoch = self.target_failure_epoch
        previous_track_id = self.target_failure_track_id
        self.target_failure_latched = False
        self.target_failure_goal = None
        self.target_failure_epoch = 0
        self.target_failure_track_id = ""
        self.target_failure_generation = 0
        self.publish_bridge_status(
            "target_failure_cleared",
            reason=str(reason),
            previous_target=(
                None
                if previous_goal is None
                else [
                    round(float(previous_goal.pose.position.x), 3),
                    round(float(previous_goal.pose.position.y), 3),
                ]
            ),
            previous_target_epoch=int(previous_epoch),
            previous_target_track_id=previous_track_id,
        )

    def _target_failure_blocks_identity_locked(
        self,
        priority,
        target_track_id="",
        target_epoch=0,
        source="unknown",
        transaction_id=0,
    ):
        """Reject a failed target before an incoming message mutates state.

        Persistent execution keeps the bridge process alive while the frontier
        planner computes a replacement.  During that gap, a queued or latched
        target message can have a newer transport transaction number even
        though it is the same failed visual track.  Transaction ordering alone
        therefore cannot authorize it.  A genuinely different track is the
        explicit semantic boundary that clears the latch; a frontier route is
        allowed through but does not clear it until physical frontier progress
        is observed.
        """
        try:
            priority = int(priority or 0)
        except (TypeError, ValueError):
            priority = 0
        if priority < 2:
            # A lower-priority frontier route is the recovery owner. It must
            # be admitted so it can make the progress that clears the latch.
            return False
        incoming_track = str(target_track_id or "").strip()
        try:
            incoming_transaction = int(transaction_id or 0)
        except (TypeError, ValueError):
            incoming_transaction = 0
        try:
            incoming_epoch = int(target_epoch or 0)
        except (TypeError, ValueError):
            incoming_epoch = 0
        tombstone_transaction = int(
            getattr(self, "target_lease_tombstone_transaction_id", 0) or 0
        )
        tombstone_epoch = int(
            getattr(self, "target_lease_tombstone_epoch", 0) or 0
        )
        tombstone_track = str(
            getattr(self, "target_lease_tombstone_track_id", "") or ""
        ).strip()
        if (
            incoming_transaction > 0
            and tombstone_transaction > 0
            and incoming_transaction <= tombstone_transaction
        ) or (
            tombstone_epoch > 0
            and incoming_epoch > 0
            and incoming_epoch <= tombstone_epoch
            and tombstone_track
            and incoming_track == tombstone_track
        ):
            self.publish_bridge_status(
                "target_intent_ignored",
                reason="released_target_transaction",
                source=str(source or "unknown"),
                priority=priority,
                transaction_id=incoming_transaction,
                target_epoch=incoming_epoch,
                target_track_id=incoming_track,
                tombstone_transaction_id=tombstone_transaction,
                tombstone_epoch=tombstone_epoch,
            )
            return True
        if not getattr(self, "target_failure_latched", False):
            return False
        failed_track = str(
            getattr(self, "target_failure_track_id", "") or ""
        ).strip()
        if incoming_track and failed_track and incoming_track != failed_track:
            self._clear_target_failure_locked("new_target_track")
            return False
        self.publish_bridge_status(
            "target_intent_ignored",
            reason="failed_target_latched",
            source=str(source or "unknown"),
            priority=priority,
            target_epoch=incoming_epoch,
            target_track_id=incoming_track,
            failed_target_epoch=int(
                getattr(self, "target_failure_epoch", 0) or 0
            ),
            failed_target_track_id=failed_track,
        )
        return True

    def _target_failure_blocks_latest_locked(self):
        """Block only the visual track that already failed to execute."""
        return self._target_failure_blocks_identity_locked(
            getattr(self, "latest_intent_priority", 0),
            getattr(self, "latest_target_track_id", ""),
            getattr(self, "latest_target_epoch", 0),
            getattr(self, "latest_intent_source", "unknown"),
        )

    def _is_active_mode(self):
        return self.mode == self.active_mode and not self.task_done

    def on_intent(self, message):
        """Record mission ownership for the next ``PoseStamped`` goal."""
        if self.use_goal_command:
            return
        source = "unknown"
        priority = 0
        route_kind = ""
        mission_route_kind = ""
        route_id = 0
        transaction_id = 0
        intent_goal = None
        target_epoch = 0
        target_track_id = ""
        target_viewpoint_candidate_id = ""
        target_viewpoint_attempt_id = ""
        goal_context = None
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                source = str(payload.get("source", source)).strip().lower() or source
                priority = int(payload.get("priority", priority))
                route_kind = str(payload.get("route_kind", "")).strip().lower()
                mission_route_kind = str(
                    payload.get("mission_route_kind", route_kind)
                ).strip().lower()
                route_id = max(0, int(payload.get("route_id", 0) or 0))
                transaction_id = max(0, int(payload.get("transaction_id", 0) or 0))
                target_epoch = max(0, int(payload.get("target_epoch", 0)))
                target_track_id = str(payload.get("target_track_id", "")).strip()
                target_viewpoint_candidate_id = str(
                    payload.get("target_viewpoint_candidate_id", "")
                ).strip()
                target_viewpoint_attempt_id = str(
                    payload.get("target_viewpoint_attempt_id", "")
                ).strip()
                goal_context = payload.get("goal_context")
                raw_goal = payload.get("goal")
                if isinstance(raw_goal, (list, tuple)) and len(raw_goal) >= 2:
                    intent_goal = (float(raw_goal[0]), float(raw_goal[1]))
            else:
                source = str(message.data).strip().lower() or source
        except (TypeError, ValueError, json.JSONDecodeError):
            source = str(message.data).strip().lower() or source
        with self.lock:
            tombstone_transaction = int(
                getattr(self, "target_lease_tombstone_transaction_id", 0) or 0
            )
            tombstone_epoch = int(
                getattr(self, "target_lease_tombstone_epoch", 0) or 0
            )
            tombstone_track = str(
                getattr(self, "target_lease_tombstone_track_id", "") or ""
            ).strip()
            if priority >= 2 and (
                (
                    transaction_id > 0
                    and tombstone_transaction > 0
                    and transaction_id <= tombstone_transaction
                )
                or (
                    tombstone_epoch > 0
                    and target_epoch > 0
                    and target_epoch <= tombstone_epoch
                    and tombstone_track
                    and target_track_id == tombstone_track
                )
            ):
                self.publish_bridge_status(
                    "intent_ignored",
                    reason="released_target_transaction",
                    received_transaction_id=transaction_id,
                    tombstone_transaction_id=tombstone_transaction,
                    target_epoch=target_epoch,
                    tombstone_epoch=tombstone_epoch,
                    target_track_id=target_track_id,
                )
                return
            if self._frontier_route_is_stale_locked(source, priority, route_id):
                self.publish_bridge_status(
                    "intent_ignored",
                    reason="stale_frontier_route",
                    route_id=route_id,
                    current_route_id=max(
                        int(getattr(self, "active_route_id", 0) or 0),
                        int(getattr(self, "latest_route_id", 0) or 0),
                    ),
                )
                return
            if self._frontier_route_released_locked(source, priority, route_id):
                self.publish_bridge_status(
                    "intent_ignored",
                    reason="released_frontier_route",
                    route_id=route_id,
                    released_route_id=int(
                        getattr(self, "frontier_lease_released_route_id", 0)
                        or 0
                    ),
                )
                return
            if self._target_failure_blocks_identity_locked(
                priority,
                target_track_id,
                target_epoch,
                source,
            ):
                return
            self._accept_newer_frontier_route_locked(source, priority, route_id)
            self.intent_seen = True
            self.latest_intent_source = source
            self.latest_intent_priority = max(0, min(3, priority))
            self.latest_route_kind = route_kind
            self.latest_mission_route_kind = mission_route_kind
            self.latest_route_id = route_id
            self.latest_intent_goal = intent_goal
            self.latest_target_epoch = target_epoch
            self.latest_target_track_id = target_track_id
            self.latest_target_viewpoint_candidate_id = (
                target_viewpoint_candidate_id
            )
            self.latest_target_viewpoint_attempt_id = target_viewpoint_attempt_id
            self.latest_goal_context = normalize_goal_context(goal_context)
            if self._is_active_mode() and self.latest_goal is not None:
                # Goal Manager publishes intent before the matching pose. Do
                # not dispatch the previously latched goal in that gap.
                if not self._intent_matches_goal_locked(self.latest_goal):
                    return
                self.dispatch_locked(force=False, reason="intent_received")

    def _intent_matches_goal_locked(self, goal):
        """Return whether the current intent describes ``goal``."""
        if goal is None:
            return False
        if self.require_intent and not self.intent_seen:
            return False
        if self.latest_intent_goal is None:
            # Plain-string intents retain the old immediate-dispatch behavior.
            return True
        return math.hypot(
            float(goal.pose.position.x) - self.latest_intent_goal[0],
            float(goal.pose.position.y) - self.latest_intent_goal[1],
        ) <= max(self.position_epsilon, 0.05)
