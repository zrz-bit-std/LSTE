"""Small lifecycle regressions for terminal-to-observation handoffs."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from navigation_metrics_command_events import (  # noqa: E402
    NavigationMetricsCommandEventsMixin,
)
from navigation_metrics_action_lifecycle import (  # noqa: E402
    NavigationMetricsActionLifecycleMixin,
)
from goal_manager_target_follow import GoalManagerTargetFollowMixin  # noqa: E402
from goal_manager_target_approach_transaction import (  # noqa: E402
    TargetApproachTransaction,
)


class MetricsTerminalFixture(
    NavigationMetricsCommandEventsMixin,
    NavigationMetricsActionLifecycleMixin,
):
    """Only the state read by the command discontinuity classifier."""

    def __init__(self, now):
        self.task_done = False
        self.navigation_hold = False
        self.cmd_vel_mux_status = None
        self.lifecycle_event_wall = {
            "persistent_execution_terminal": float(now) - 0.02,
        }
        self.teb_feedback_state = None


class TargetObservationFixture(GoalManagerTargetFollowMixin):
    """Exercise the route-unavailable terminal branch without ROS."""

    def __init__(self):
        self.target_track_id = "yellow_cup:yellow cup:1"
        self.target_approach_track_id = self.target_track_id
        self.target_follow_confirmed = True
        self.target_completed_segments = 3
        self.target_terminal_reobserve_until = 0.0
        self.target_route_validation_last_result = "unavailable"
        self.target_execution_state = "TARGET_CANDIDATE"
        self.goal_source = "target_waiting_navfn_route"
        self.target_terminal_observation_intent_sent = False
        self.target_approach_transaction = TargetApproachTransaction()
        self.target_approach_transaction.begin(self.target_track_id, 1.0)
        self.target_approach_transaction.segment_committed(
            self.target_track_id, 2.0
        )
        self.target_approach_transaction.segment_arrived(
            self.target_track_id, 3.0
        )
        self.navigation_holds = []
        self.intent_reasons = []
        self.events = []

    def target_observation_loss_certified(self):
        return False

    def target_tracking_active(self, _now):
        return True

    def set_navigation_hold(self, active, reason):
        self.navigation_holds.append((bool(active), str(reason)))

    def publish_target_terminal_observation_intent(self, reason):
        self.intent_reasons.append(str(reason))
        self.target_terminal_observation_intent_sent = True
        return True

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((str(event), fields))


class TerminalHandoffContractTest(unittest.TestCase):
    def test_persistent_terminal_classifies_following_zero_as_action_terminal(self):
        fixture = MetricsTerminalFixture(now=10.0)

        reason, age = fixture._command_discontinuity_reason_locked(10.1)

        self.assertEqual(reason, "action_terminal")
        self.assertAlmostEqual(age, 0.12, places=3)

    def test_unavailable_target_route_releases_old_goal_ownership(self):
        fixture = TargetObservationFixture()

        result = fixture._finish_unavailable_terminal_continuation(4.0)

        self.assertIsNone(result)
        self.assertEqual(fixture.goal_source, "target_terminal_observation")
        self.assertEqual(
            fixture.navigation_holds[-1],
            (True, "target_terminal_observation"),
        )
        self.assertEqual(
            fixture.intent_reasons,
            ["navfn_unavailable_after_target_terminal"],
        )
        self.assertIn(
            "target_terminal_observation_waiting",
            [event for event, _fields in fixture.events],
        )


if __name__ == "__main__":
    unittest.main()
