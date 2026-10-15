"""Persistent Navfn target-plan result handling for the TEB bridge."""

import copy
import json

from std_msgs.msg import String


class TebGoalBridgePersistentTargetResultMixin:
    def on_persistent_target_plan_result(self, message):
        """Consume typed target lifecycle events from persistent Navfn/TEB."""
        try:
            payload = json.loads(message.data)
            transaction_id = int(payload.get("transaction_id", 0) or 0)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        event = str(payload.get("event", ""))
        result_goal = self._goal_from_persistent_result(payload)
        with self.lock:
            if event == "target_plan_installed":
                if result_goal is None:
                    self.publish_bridge_status(
                        "persistent_target_install_ignored",
                        reason="malformed_result_goal",
                        received_transaction_id=transaction_id,
                    )
                    return
                self._install_persistent_target_locked(transaction_id, result_goal)
                return
            if event == "target_approach":
                if result_goal is None:
                    return
                self._handle_persistent_target_approach_locked(
                    transaction_id, result_goal
                )
                return
            if event != "target_plan_failed":
                return
            # A request is pending only until its first installation. Later
            # replans can invalidate the same installed target and must still
            # reach GoalManager as a failure for this transaction.
            pending_transaction = int(self.persistent_target_pending_transaction)
            installed_transaction = int(self.persistent_installed_target_transaction)
            matching_target_transaction = (
                transaction_id == pending_transaction
                or transaction_id == installed_transaction
            )
            if (
                not self.persistent_execution
                or self.task_done
                or self.latest_goal is None
                or self.latest_intent_priority < 2
                or transaction_id <= 0
                or transaction_id != int(self.latest_goal_transaction_id)
                or not matching_target_transaction
            ):
                self.publish_bridge_status(
                    "persistent_target_plan_failure_ignored",
                    received_transaction_id=transaction_id,
                    latest_transaction_id=int(self.latest_goal_transaction_id),
                    pending_transaction_id=pending_transaction,
                    installed_transaction_id=installed_transaction,
                )
                return
            if self.target_failure_latched:
                return
            source_goal = copy.deepcopy(self.latest_goal)
            target_epoch = int(self.latest_target_epoch)
            target_track_id = str(self.latest_target_track_id or "")
            self.target_failure_latched = True
            self.target_failure_goal = copy.deepcopy(source_goal)
            self.target_failure_epoch = target_epoch
            self.target_failure_track_id = target_track_id
            self.target_failure_generation = int(self.action_generation)
            self.target_failure_count += 1
            # Keep the semantic terminal boundary in the event stream even
            # though the failed target lease will be cancelled immediately.
            # This lets a newer frontier transaction correlate its takeover
            # without waiting for a target retry or a transport restart.
            self.persistent_target_terminal_boundary_transaction = transaction_id
            failure = {
                "event": "target_route_failed",
                "reason": "persistent_navfn_target_unreachable",
                "status": "NAVFN_NO_PATH",
                "goal": [
                    round(float(source_goal.pose.position.x), 4),
                    round(float(source_goal.pose.position.y), 4),
                ],
                "goal_frame": source_goal.header.frame_id,
                "target_epoch": target_epoch,
                "map_epoch": getattr(self, "active_route_map_epoch", None),
                "route_kind": "direct_goal",
                "mission_route_kind": "direct_goal",
                "target_track_id": target_track_id,
                "target_viewpoint_candidate_id": str(
                    getattr(self, "latest_target_viewpoint_candidate_id", "") or ""
                ),
                "target_viewpoint_attempt_id": str(
                    getattr(self, "latest_target_viewpoint_attempt_id", "") or ""
                ),
                "transaction_id": transaction_id,
                "failure_count": int(self.target_failure_count),
            }
            # A persistent target-plan failure is already terminal evidence;
            # release the target intent immediately so the frontier can become
            # the next owner. The release helper also clears the pending
            # request tombstone and preserves the failed track latch.
            released = self._release_failed_target_controller_lease_locked(
                "target_plan_failed"
            )
            # The transport lease was invalidated above, but retain a bounded
            # failure contract so a silent downstream handoff still produces a
            # deterministic cleanup event instead of an unbounded wait.
            arm_watchdog = getattr(
                self, "_arm_route_lease_watchdog_locked", None
            )
            if callable(arm_watchdog):
                failure_contract = {
                    "generation": int(getattr(self, "action_generation", 0) or 0),
                    "transaction_id": transaction_id,
                    "lifecycle_transaction_id": int(
                        self._canonical_lifecycle_transaction_id()
                    ),
                    "route_id": int(getattr(self, "active_route_id", 0) or 0),
                    "map_epoch": getattr(self, "active_route_map_epoch", None),
                    "epoch": target_epoch,
                    "route_kind": "direct_goal",
                    "mission_route_kind": "direct_goal",
                    "source": "target_plan_failure",
                    "priority": 2,
                }
                arm_watchdog(
                    reason="persistent_target_plan_failure",
                    action_contract=failure_contract,
                )
            failure.update(
                {
                    "controller_lease": "released" if released else "unchanged",
                    "target_lease_tombstone_transaction_id": int(
                        getattr(
                            self, "target_lease_tombstone_transaction_id", 0
                        )
                        or 0
                    ),
                    "target_lease_tombstone_epoch": int(
                        getattr(self, "target_lease_tombstone_epoch", 0) or 0
                    ),
                    "target_lease_tombstone_track_id": str(
                        getattr(self, "target_lease_tombstone_track_id", "")
                        or ""
                    ),
                }
            )
            self.target_failure_pub.publish(
                String(data=json.dumps(failure, sort_keys=True))
            )
            self.publish_bridge_status(
                "persistent_target_plan_failed",
                transaction_id=transaction_id,
                reason=failure["reason"],
                target_track_id=target_track_id,
                target_epoch=target_epoch,
                controller_lease="released" if released else "unchanged",
                lifecycle=(
                    "release_to_frontier"
                    if released else "release_to_frontier_without_action_cancel"
                ),
            )
