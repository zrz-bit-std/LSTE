"""Regression tests for non-blocking frontier terminal ingress."""

from collections import deque
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
import sys

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import global_frontier_terminal_replan as terminal_module
from global_frontier_terminal_replan import GlobalFrontierTerminalReplanMixin


class _Timer:
    def __init__(self, _duration, _callback, oneshot=False):
        self.oneshot = oneshot

    def shutdown(self):
        pass


class _IngressExplorer(GlobalFrontierTerminalReplanMixin):
    def __init__(self):
        self.planning_lock = threading.Lock()
        self.terminal_ingress_lock = threading.Lock()
        self.pending_execution_terminals = deque(maxlen=16)
        self.terminal_ingress_dropped = 0
        self.terminal_drain_timer = None
        self.processed = []
        self.match_calls = 0

    def matching_execution_terminal(self, message):
        self.match_calls += 1
        if getattr(message, "stale", False):
            return None
        return ((0, 0, 4.0, 5.0), 0.2)

    def consume_connector_terminal(self):
        return False

    def record_execution_terminal(self, completed):
        self.processed.append(("record", completed))

    def promote_prefetched_terminal(self, _message):
        return False

    def schedule_terminal_replan(self, completed, delta):
        self.processed.append(("replan", completed, delta))


class FrontierTerminalIngressTest(unittest.TestCase):
    def setUp(self):
        self.timer = terminal_module.rospy.Timer
        self.duration = terminal_module.rospy.Duration
        self.is_shutdown = terminal_module.rospy.is_shutdown
        terminal_module.rospy.Timer = _Timer
        terminal_module.rospy.Duration = lambda seconds: seconds
        terminal_module.rospy.is_shutdown = lambda: False

    def tearDown(self):
        terminal_module.rospy.Timer = self.timer
        terminal_module.rospy.Duration = self.duration
        terminal_module.rospy.is_shutdown = self.is_shutdown

    def test_terminal_callback_does_not_wait_for_slow_planning_lock(self):
        explorer = _IngressExplorer()
        explorer.planning_lock.acquire()
        finished = threading.Event()

        def ingress():
            explorer.on_execution_terminal(SimpleNamespace(stale=False))
            finished.set()

        thread = threading.Thread(target=ingress)
        thread.start()
        self.assertTrue(
            finished.wait(0.20),
            "terminal ROS callback must not wait for the planning cycle",
        )
        thread.join(timeout=1.0)
        self.assertEqual(explorer.match_calls, 0)
        self.assertEqual(len(explorer.pending_execution_terminals), 1)
        explorer.planning_lock.release()

    def test_drain_applies_identity_check_and_replans_under_lock(self):
        explorer = _IngressExplorer()
        explorer.pending_execution_terminals.append(SimpleNamespace(stale=False))
        explorer.pending_execution_terminals.append(SimpleNamespace(stale=True))

        with explorer.planning_lock:
            explorer._drain_execution_terminals_locked()

        self.assertEqual(explorer.match_calls, 2)
        self.assertEqual(
            explorer.processed,
            [("record", (0, 0, 4.0, 5.0)), ("replan", (0, 0, 4.0, 5.0), 0.2)],
        )
        self.assertFalse(explorer.pending_execution_terminals)

    def test_busy_drain_timer_rearms_without_waiting(self):
        explorer = _IngressExplorer()
        explorer.pending_execution_terminals.append(SimpleNamespace(stale=False))
        explorer.terminal_drain_timer = object()
        explorer.planning_lock.acquire()
        started = time.monotonic()
        try:
            explorer.on_terminal_drain_timer(None)
        finally:
            explorer.planning_lock.release()
        self.assertLess(time.monotonic() - started, 0.20)
        self.assertEqual(explorer.match_calls, 0)
        self.assertTrue(explorer.pending_execution_terminals)
        self.assertIsNotNone(explorer.terminal_drain_timer)


if __name__ == "__main__":
    unittest.main()
