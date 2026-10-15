"""Regression tests for optimistic slow-planner proposal ownership."""

from pathlib import Path
from collections import deque
import sys
import threading
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_contract import (  # noqa: E402
    PlanningProposal,
    PlanningProposalGate,
)
from global_frontier_planning_runtime import (  # noqa: E402
    GlobalFrontierPlanningRuntimeMixin,
)


class _RuntimeProbe(GlobalFrontierPlanningRuntimeMixin):
    """Minimal runtime shell used to test the lock ownership boundary."""

    def __init__(self):
        from global_frontier_planning_contract import PlanningProposalGate

        self.planning_cycle_lock = threading.Lock()
        self.planning_lock = threading.RLock()
        self.planning_proposal_gate = PlanningProposalGate()
        self.planning_preempt_requested = False
        self.planning_preempt_reason = ""
        self.terminal_ingress_lock = threading.Lock()
        self.pending_execution_terminals = deque(maxlen=4)
        self.planning_cycle_skipped = 0
        self.planning_cycle_proposal = None
        self.active_route_id = 0
        self.active_frontier = object()
        self.prefetched_frontier = None
        self.map_msg = object()
        self.commit_started = threading.Event()
        self.release_planning = threading.Event()

    def _prepare_planning_cycle(self):
        token = self.planning_proposal_gate.begin(
            active_route_id=self.active_route_id,
        )
        return token, False

    def _on_timer(self, _event, token=None):
        del token
        self.commit_started.set()
        self.assert_true(self.release_planning.wait(1.0))

    @staticmethod
    def assert_true(value):
        if not value:
            raise AssertionError("test runtime did not receive release")

    def fresh_costmap(self):
        return None


class PlanningRuntimeBoundaryTest(unittest.TestCase):
    def test_slow_cycle_does_not_hold_planning_lock(self):
        runtime = _RuntimeProbe()
        worker = threading.Thread(target=runtime.on_timer, args=(None,))
        worker.start()
        self.assertTrue(runtime.commit_started.wait(1.0))

        acquired = runtime.planning_lock.acquire(False)
        if acquired:
            runtime.planning_lock.release()
        runtime.release_planning.set()
        worker.join(1.0)

        self.assertTrue(acquired)
        self.assertFalse(worker.is_alive())


class PlanningProposalGateTest(unittest.TestCase):
    def test_terminal_invalidation_rejects_old_snapshot(self):
        gate = PlanningProposalGate()
        token = gate.begin(active_route_id=0, wake_sequence=7)
        gate.invalidate("execution_terminal")

        decision = gate.check(token, active_route_id=0)

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "execution_terminal")

    def test_route_identity_is_part_of_commit_contract(self):
        gate = PlanningProposalGate()
        token = gate.begin(active_route_id=11)
        proposal = PlanningProposal(token=token, candidate=(1, 2))
        called = []

        decision = gate.commit_if_current(
            proposal.token,
            active_route_id=12,
            commit=lambda: called.append(True),
        )

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "active_route_changed")
        self.assertEqual(called, [])

    def test_commit_is_atomic_against_concurrent_invalidation(self):
        gate = PlanningProposalGate()
        token = gate.begin(active_route_id=0)
        entered = threading.Event()
        release = threading.Event()
        invalidation_done = threading.Event()
        committed = []

        def commit():
            entered.set()
            self.assertTrue(release.wait(1.0))
            committed.append("route")

        def invalidate():
            self.assertTrue(entered.wait(1.0))
            gate.invalidate("execution_terminal")
            invalidation_done.set()

        invalidator = threading.Thread(target=invalidate)
        invalidator.start()
        decision = None
        try:
            holder = threading.Thread(
                target=lambda: gate.commit_if_current(token, 0, commit)
            )
            holder.start()
            self.assertTrue(entered.wait(1.0))
            self.assertFalse(invalidation_done.is_set())
            release.set()
            holder.join(1.0)
            invalidator.join(1.0)
            decision = gate.check(token, active_route_id=0)
        finally:
            release.set()
            invalidator.join(1.0)

        self.assertEqual(committed, ["route"])
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "execution_terminal")


if __name__ == "__main__":
    unittest.main()
