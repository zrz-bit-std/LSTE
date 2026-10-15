"""Regression tests for the durable route-lease boundary."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_durable_action_lease import (  # noqa: E402
    DurableActionLease,
    LEASE_DEFER,
    LEASE_NOOP,
    LEASE_PREEMPT,
    decide_replan_lease,
)
from global_frontier_durable_lease_lifecycle import (  # noqa: E402
    GlobalFrontierDurableLeaseLifecycleMixin,
)


class DurableActionLeaseTest(unittest.TestCase):
    def test_empty_route_has_no_reconciliation(self):
        decision = decide_replan_lease(DurableActionLease())
        self.assertEqual(decision.action, LEASE_NOOP)

    def test_active_probe_and_work_item_are_preemptible(self):
        decision = decide_replan_lease(
            DurableActionLease(
                route_id=7,
                route_kind="frontier_endpoint",
                work_item_id=20,
                work_item_attempt_id=31,
                portal_probe_id=9,
            ),
            "target_room_claim_release",
        )
        self.assertEqual(decision.action, LEASE_PREEMPT)
        self.assertEqual(
            decision.obligations,
            (("portal_probe", 9), ("work_item", 20)),
        )
        self.assertIn("target_room_claim_release", decision.reason)

    def test_crossed_portal_waits_for_arrival_commit(self):
        decision = decide_replan_lease(
            DurableActionLease(
                route_id=8,
                route_kind="portal_transition",
                portal_probe_id=4,
                portal_crossing_observed=True,
            ),
            "semantic_replan",
        )
        self.assertEqual(decision.action, LEASE_DEFER)
        self.assertEqual(decision.reason, "portal_arrival_commit_owns_route")

    def test_portal_transaction_is_non_preemptible(self):
        decision = decide_replan_lease(
            DurableActionLease(
                route_id=9,
                route_kind="portal_transition",
                portal_transaction_active=True,
            )
        )
        self.assertEqual(decision.action, LEASE_DEFER)

    def test_preemption_closes_both_durable_attempts_before_reset(self):
        class Harness(GlobalFrontierDurableLeaseLifecycleMixin):
            active_route_id = 12
            active_route_kind = "frontier_endpoint"
            active_work_item_id = 20
            active_work_item_attempt_id = 31
            active_portal_probe_id = 9
            active_portal_crossing_observed = False
            portal_transaction = None

            def __init__(self):
                self.calls = []
                self.events = []

            def settle_active_portal_probe(self, result, now, reason):
                self.calls.append(("probe", result, now, reason))
                self.active_portal_probe_id = None
                return {"state": "pending"}

            def settle_active_work_item(self, now, state, reason):
                self.calls.append(("work", now, state, reason))
                self.active_work_item_id = None
                return {"state": "unresolved"}

            def publish_status(self, event, **fields):
                self.events.append((event, fields))

        harness = Harness()
        decision = harness.reconcile_active_durable_lease_for_replan(
            "target_reinspection", 4.5
        )
        self.assertEqual(decision.action, LEASE_PREEMPT)
        self.assertEqual(harness.calls[0][0], "probe")
        self.assertEqual(harness.calls[1][0], "work")
        self.assertEqual(harness.events[-1][0], "durable_route_lease_reconciled")
        self.assertEqual(harness.events[-1][1]["route_id"], 12)


if __name__ == "__main__":
    unittest.main()
