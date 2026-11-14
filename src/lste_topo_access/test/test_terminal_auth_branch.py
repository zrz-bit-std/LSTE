#!/usr/bin/env python3
"""A terminal active authentication branch must be settled, not re-preempted.

env04 20261114_162926: place 5 observation, branches 11 and 12 failed, branch
13 pending, but active_branch_id stayed 11.  start_next() only runs when the
active id is None, so the pending doorway never started and the observation
period preempted the graph route every 1.2 s (242 times) until the stall.
"""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from gp_branch_frontier import GPBranchFrontierMixin  # noqa: E402


class _Host(GPBranchFrontierMixin):
    def __init__(self, advance=True):
        self.advance = advance
        self.calls = []
        self.task = None

    def finish_portal_authentication_branch(self, now=0.0):
        self.calls.append(now)
        if self.advance:
            self.task.active_branch_id = None
        return None


def _task(active, states):
    return SimpleNamespace(
        active_branch_id=active,
        branches={bid: SimpleNamespace(state=state)
                  for bid, state in states.items()},
    )


class TerminalAuthBranchTest(unittest.TestCase):
    def test_a_failed_active_branch_is_settled(self):
        host = _Host(advance=True)
        host.task = _task(11, {11: "failed", 13: "pending"})
        self.assertTrue(host.settle_terminal_auth_branch(host.task, 100.0))
        self.assertIsNone(host.task.active_branch_id)
        self.assertEqual(host.calls, [100.0])

    def test_a_completed_active_branch_is_settled(self):
        host = _Host(advance=True)
        host.task = _task(11, {11: "completed"})
        self.assertTrue(host.settle_terminal_auth_branch(host.task, 5.0))

    def test_a_live_branch_is_not_settled(self):
        host = _Host()
        host.task = _task(11, {11: "returning"})
        self.assertFalse(host.settle_terminal_auth_branch(host.task, 5.0))
        self.assertEqual(host.calls, [])

    def test_a_settle_that_does_not_advance_returns_false(self):
        # Keep the old preemption behaviour when the ledger cannot advance.
        host = _Host(advance=False)
        host.task = _task(11, {11: "failed"})
        self.assertFalse(host.settle_terminal_auth_branch(host.task, 5.0))
        self.assertEqual(host.task.active_branch_id, 11)

    def test_no_active_branch_is_a_noop(self):
        host = _Host()
        host.task = _task(None, {13: "pending"})
        self.assertFalse(host.settle_terminal_auth_branch(host.task, 5.0))
        self.assertEqual(host.calls, [])


if __name__ == "__main__":
    unittest.main()
