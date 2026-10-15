"""Regression contract for persistent Navfn/TEB route identity handling."""

from pathlib import Path
import unittest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src/persistent_navigation_plugins.cpp"
)
HEADER = (
    Path(__file__).resolve().parents[1]
    / "include/lste_topo_access/persistent_navigation_plugins.h"
)


class PersistentPlannerRouteIdentityTest(unittest.TestCase):
    def test_non_global_goal_uses_latest_tf_and_mission_transaction(self):
        source = SOURCE.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")

        self.assertIn("latest.header.stamp = ros::Time(0);", source)
        self.assertIn("void onMissionCommand", header)
        self.assertIn("latest_mission_transaction_", header)
        self.assertIn("has_reported_frontier_goal_ = false;", source)

    def test_route_identity_reset_is_guarded_by_transaction_change(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn(
            "if (message->transaction_id == latest_mission_transaction_)",
            source,
        )
        self.assertIn(
            "latest_mission_transaction_ = message->transaction_id;",
            source,
        )

    def test_terminal_hold_keeps_persistent_action_out_of_recovery(self):
        source = SOURCE.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")

        self.assertIn("terminal_hold_active_", header)
        self.assertIn(
            "terminal_hold_active_.load(std::memory_order_acquire)",
            source,
        )
        self.assertIn(
            "terminal_hold_active_.store(false, std::memory_order_release)",
            source,
        )

    def test_target_clear_invalidates_installed_target_and_propagates_tombstone(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn(
            "if (message->kind == PersistentGoalCommand::KIND_CLEAR)",
            source,
        )
        self.assertIn("has_installed_target_goal_ = false;", source)
        self.assertIn("installed_target_command_publisher_.publish(installed_clear);", source)
        self.assertIn("failed_target_sequence_ = std::max(failed_target_sequence_, clear_sequence);", source)


if __name__ == "__main__":
    unittest.main()
