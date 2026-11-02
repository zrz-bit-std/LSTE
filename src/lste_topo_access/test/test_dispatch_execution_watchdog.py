"""A dispatched route must be proven to execute, not merely deferred to.

Run ``20261102_054037`` dispatched a WorkItem at sim 1362.9 and then emitted
nothing for the remaining 180 s of the attempt; the stall watchdog found the
base never moved more than 0.13 m.  Three gates allowed that silence:

* ``active_route_planning_is_deferred`` gated its stall trigger on
  ``not controller_owned_route_failure``, which is a *mode* flag that is always
  true under ``persistent_execution`` - so the stall clock never re-entered
  planning for a controller-owned route.
* ``evaluate_active_route_watchdogs`` set ``active_progress_time = now`` on
  every cycle while a turn phase was active, with no bound, so one latched
  TURNING phase disabled every translational watchdog at once.
* ``route_lease_failure_decision`` waited for a controller terminal forever
  (now bounded, 45 s -> 12 s: the measured terminal latency is < 1 s).

The stall event now also carries the state of every gate, so the next
occurrence identifies itself instead of needing a forensic rerun.
"""

from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_snapshot import (  # noqa: E402
    GlobalFrontierPlanningSnapshotMixin,
)
from global_frontier_execution_watchdogs import (  # noqa: E402
    GlobalFrontierExecutionWatchdogMixin,
)


class DeferralFixture(GlobalFrontierPlanningSnapshotMixin):
    def __init__(self, controller_owned=True):
        self.active_frontier = (0, 0, 10.0, 0.0)
        self.active_last_waypoint_map = (10.0, 0.0)
        self.recovery_pending_route_id = 0
        self.active_route_id = 52
        self.prefetched_frontier = None
        self.prefetch_distance = 2.5
        self.waypoint_release_radius = 0.8
        self.active_last_waypoint_map = (5.0, 0.0)
        self.controller_owned_route_failure = controller_owned
        self.active_progress_time = 100.0
        self.stall_timeout = 8.0
        self.last_planning_wall = 100.0
        self.planning_period = 1.5


class DeferralTest(unittest.TestCase):
    def test_a_controller_owned_stall_re_enters_planning(self):
        # 12 s without progress, far from both the goal and the waypoint.
        fixture = DeferralFixture(controller_owned=True)
        self.assertFalse(
            fixture.active_route_planning_is_deferred((0.0, 0.0), 112.0)
        )

    def test_a_healthy_route_is_still_throttled(self):
        fixture = DeferralFixture()
        fixture.active_progress_time = 111.0
        self.assertFalse(
            fixture.active_route_planning_is_deferred((0.0, 0.0), 112.0)
        )
        # Inside one planning period with recent progress: skip the heavy plan.
        fixture.active_progress_time = 112.0
        fixture.last_planning_wall = 112.0
        self.assertTrue(
            fixture.active_route_planning_is_deferred((0.0, 0.0), 112.5)
        )

    def test_proximity_still_forces_planning(self):
        fixture = DeferralFixture()
        fixture.active_progress_time = 112.0
        fixture.last_planning_wall = 112.0
        # Within prefetch_distance of the goal.
        self.assertFalse(
            fixture.active_route_planning_is_deferred((9.0, 0.0), 112.5)
        )


class WatchdogFixture(GlobalFrontierExecutionWatchdogMixin):
    def __init__(self, turn_age=0.0):
        self.turn_supervisor_state = "TURNING"
        self.turn_supervisor_route_kind = "frontier_turn_connector"
        self.active_route_kind = "frontier_turn_connector"
        self.turn_connector_released = False
        self.recovery_pending_route_id = 0
        self.active_route_id = 52
        self.active_progress_time = 100.0
        self.turn_phase_since = 100.0 - turn_age
        self.turn_phase_max_seconds = 20.0
        self.post_turn_stall_timeout = None
        self.active_turn_completed_launched = True
        self.active_since = 100.0
        self.active_timeout = 18.0
        self.stall_timeout = 8.0
        self.controller_owned_route_failure = True
        self.controller_terminal_wait_seconds = 12.0
        self.last_route_stagnation_reported_route_id = 0
        self.expired_turn_phase_route_id = 0
        self.active_last_progress_signal = "none"
        self.active_portal_gate_approached_at = None
        self.waypoint_release_radius = 0.8
        self.active_last_waypoint_map = (5.0, 0.0)
        self.active_portal_authentication_phase = ""
        self.active_portal_authentication_entry_approach = False
        self.events = []

    def update_post_turn_watchdog(self, x, y, now):
        return False, 0.0, None

    def active_region_is_stagnant(self, *args, **kwargs):
        return False

    def active_waypoint_distance(self, robot_map):
        return 5.0

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class TurnPhaseBoundTest(unittest.TestCase):
    def test_a_fresh_turn_pins_the_progress_clock(self):
        fixture = WatchdogFixture(turn_age=5.0)
        state = fixture.evaluate_active_route_watchdogs(
            0.0, 0.0, None, 105.0, False, None, None)
        self.assertTrue(state.turn_phase_active)
        self.assertFalse(state.stalled)
        self.assertEqual(fixture.active_progress_time, 105.0)

    def test_a_latched_turn_stops_hiding_the_stall(self):
        fixture = WatchdogFixture(turn_age=25.0)
        state = fixture.evaluate_active_route_watchdogs(
            0.0, 0.0, None, 125.0, False, None, None)
        self.assertFalse(state.turn_phase_active)
        self.assertTrue(state.stalled)
        self.assertIn(
            "turn_phase_progress_expired",
            [event for event, _fields in fixture.events],
        )

    def test_the_expiry_is_reported_once_per_route(self):
        fixture = WatchdogFixture(turn_age=25.0)
        for now in (125.0, 126.0):
            fixture.evaluate_active_route_watchdogs(
                0.0, 0.0, None, now, False, None, None)
        self.assertEqual(
            [event for event, _fields in fixture.events].count(
                "turn_phase_progress_expired"),
            1,
        )

    def test_the_stall_event_carries_every_gate(self):
        fixture = WatchdogFixture(turn_age=25.0)
        fixture.active_work_item_dispatch_announced = True
        fixture.pending_controller_lease_release = None
        fixture.evaluate_active_route_watchdogs(
            0.0, 0.0, None, 125.0, False, None, None)
        fields = dict(fixture.events)["route_stagnant_observed"] if False else None
        blockers = [f for e, f in fixture.events if e == "route_stagnant_observed"]
        self.assertEqual(len(blockers), 1)
        self.assertGreaterEqual(blockers[0]["progress_age_seconds"], 8.0)
        self.assertFalse(blockers[0]["turn_phase_active"])
        self.assertTrue(blockers[0]["dispatch_announced"])


class ConfigContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = (SCRIPTS / "global_frontier_config_navigation.py").read_text(
            encoding="utf-8")
        cls.snapshot = (
            SCRIPTS / "global_frontier_planning_snapshot.py").read_text(
            encoding="utf-8")

    def test_the_terminal_wait_is_short_enough_to_be_immediate(self):
        self.assertIn('"~controller_terminal_wait_seconds", 12.0', self.config)
        self.assertNotIn('"~controller_terminal_wait_seconds", 45.0', self.config)

    def test_the_turn_bound_is_a_parameter(self):
        self.assertIn("turn_phase_max_seconds", self.config)

    def test_the_stall_clock_is_not_gated_by_persistent_mode(self):
        self.assertNotIn(
            "not getattr(self, \"controller_owned_route_failure\", False)",
            self.snapshot,
        )


class DriverBudgetTest(unittest.TestCase):
    """The drive ceiling must cover each scenario's measured sim-time cost."""

    @classmethod
    def setUpClass(cls):
        cls.driver = (
            Path(__file__).resolve().parents[3]
            / "scripts/tests/targeted_navigation/run_four_until_pass.sh"
        ).read_text(encoding="utf-8")

    def budget(self, case):
        import re
        match = re.search(r"\[%s\]=(\d+)" % re.escape(case), self.driver)
        self.assertIsNotNone(match, case)
        return int(match.group(1))

    def test_l_is_not_inflated(self):
        # L completes at 1168 s sim, well inside 2400 s wall.
        self.assertGreaterEqual(self.budget("semantic_l_junction"), 2000)
        self.assertLessEqual(self.budget("semantic_l_junction"), 3000)

    def test_the_three_long_scenarios_get_enough_sim_time(self):
        # 0.65 sim/wall is the worst observed ratio.
        for case, need in (
            ("semantic_t_junction", 2400),
            ("semantic_t_junction_rotated", 2400),
            ("semantic_cross_junction", 3000),
        ):
            self.assertGreaterEqual(self.budget(case) * 0.65, need, case)

    def test_an_explicit_override_still_wins(self):
        self.assertIn('DRIVE_OVERRIDE="${GP_SUITE_SECONDS:-}"', self.driver)


if __name__ == "__main__":
    unittest.main(verbosity=2)
