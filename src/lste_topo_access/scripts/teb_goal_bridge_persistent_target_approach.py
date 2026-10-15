"""Commit verified target-approach terminals for persistent TEB execution."""

import rospy


class TebGoalBridgePersistentTargetApproachMixin:
    def _handle_persistent_target_approach_locked(self, transaction_id, message):
        """Forward an identity-bound persistent target arrival to GoalManager."""
        if not self.persistent_execution or self.task_done:
            return
        if (
            self.latest_goal is None
            or self.latest_intent_priority < 2
            or self.persistent_target_approach_reported_transaction
            == int(self.latest_goal_transaction_id)
        ):
            self.publish_bridge_status(
                "persistent_target_approach_ignored",
                received_transaction_id=transaction_id,
                latest_transaction_id=int(self.latest_goal_transaction_id),
            )
            return
        latest_global = self._goal_in_global_frame(self.latest_goal)
        if latest_global is None or not self._same_goal(latest_global, message):
            self.publish_bridge_status(
                "persistent_target_approach_ignored",
                reason="endpoint_mismatch",
                received_transaction_id=transaction_id,
                latest_transaction_id=int(self.latest_goal_transaction_id),
                received_goal=[
                    round(message.pose.position.x, 3),
                    round(message.pose.position.y, 3),
                ],
                latest_goal=(
                    None
                    if latest_global is None
                    else [
                        round(latest_global.pose.position.x, 3),
                        round(latest_global.pose.position.y, 3),
                    ]
                ),
            )
            self._republish_persistent_target_request_locked(
                "stale_target_approach_endpoint"
            )
            return
        strict_transaction_match = (
            transaction_id > 0
            and transaction_id == int(self.latest_goal_transaction_id)
            and self.persistent_installed_target_goal is not None
            and transaction_id == int(self.persistent_installed_target_transaction)
            and self._same_goal(self.persistent_installed_target_goal, message)
        )
        if not strict_transaction_match:
            self.publish_bridge_status(
                "persistent_target_approach_ignored",
                reason="missing_matching_installation",
                received_transaction_id=transaction_id,
                latest_transaction_id=int(self.latest_goal_transaction_id),
            )
            return
        # This is a semantic target-segment boundary, not the end of the
        # persistent MoveBase lease.  The next viewpoint must be able to enter
        # the same Navfn/TEB stream immediately; using the generic termination
        # transaction here would revoke the action generation and leave the
        # successor blocked behind a controller-release ACK that target routes
        # do not produce.
        terminal_goal = self._publish_execution_terminal_locked(self.latest_goal)
        if terminal_goal is None:
            return
        self.persistent_target_approach_reported_transaction = int(
            self.latest_goal_transaction_id
        )
        # This is a logical terminal of the target route. Persistent Navfn/TEB
        # keeps the native MoveBase action alive so the next mission command
        # can stream in without a cancel/restart gap.
        self.persistent_target_terminal_boundary_transaction = int(
            self.latest_goal_transaction_id
        )
        self.terminal_count += 1
        self.publish_bridge_status(
            "persistent_target_approach_terminal",
            transaction_id=int(self.latest_goal_transaction_id),
            received_transaction_id=transaction_id,
            identity_mode="transaction",
            target_epoch=int(self.latest_target_epoch),
            target_track_id=self.latest_target_track_id,
            terminal_goal=[
                round(terminal_goal.pose.position.x, 3),
                round(terminal_goal.pose.position.y, 3),
            ],
            lifecycle="single_action_target_approach",
        )
        # This is not a MoveBase terminal and does not release the controller
        # lease.  The persistent action is intentionally retained while the
        # next target/frontier goal is selected, so a release-ACK watchdog
        # would eventually cancel the still-valid action during that gap.
        rospy.loginfo(
            "TEB goal bridge forwarded persistent target approach: "
            "transaction=%d received=%d track=%s",
            self.latest_goal_transaction_id,
            transaction_id,
            self.latest_target_track_id,
        )
