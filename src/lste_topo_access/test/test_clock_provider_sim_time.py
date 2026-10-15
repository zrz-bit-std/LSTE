#!/usr/bin/env python3
"""Red/green coverage for ROS-clock-aware lifecycle timing."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from lifecycle_manager import EventType, LifecycleManager, State
from clock_provider import ClockProvider


class ClockProviderSimTimeTest(unittest.TestCase):
    def test_paused_sim_clock_does_not_consume_host_fallback_timeout(self):
        sim_time = [0.0]
        host_time = [0.0]
        provider = ClockProvider(
            fallback=lambda: host_time[0],
            ros_initialized=lambda: True,
            ros_shutdown=lambda: False,
            ros_now=lambda: sim_time[0],
        )
        manager = LifecycleManager(
            time_provider=provider,
            timeouts={State.DISPATCHED: 5.0},
        )
        transaction_id = manager.begin_transaction(State.DISPATCHED)
        manager.enqueue_type(EventType.NAV_DISPATCHED, transaction_id=transaction_id)
        manager.tick()

        host_time[0] = 100.0
        self.assertFalse(manager.tick().timed_out)
        self.assertEqual(manager.current_state, State.DISPATCHED)

        sim_time[0] = 5.1
        self.assertTrue(manager.tick().timed_out)
        self.assertEqual(manager.current_state, State.FAILED)

    def test_ros_clock_is_preferred_over_fallback_when_node_is_live(self):
        provider = ClockProvider(
            fallback=lambda: 900.0,
            ros_initialized=lambda: True,
            ros_shutdown=lambda: False,
            ros_now=lambda: 0.2,
        )

        self.assertEqual(provider.now(), 0.2)

    def test_live_node_at_sim_time_zero_does_not_fallback_to_host_clock(self):
        with patch("clock_provider.rospy.core.is_initialized", return_value=True), \
                patch("clock_provider.rospy.is_shutdown", return_value=False), \
                patch("clock_provider.rospy.Time.now") as ros_now:
            ros_now.return_value.to_sec.return_value = 0.0
            provider = ClockProvider(fallback=lambda: 900.0)

            self.assertEqual(provider.now(), 0.0)


if __name__ == "__main__":
    unittest.main()
