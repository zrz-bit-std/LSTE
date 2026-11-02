#!/usr/bin/env python3
"""Regression for settling a terminal contract after its exact Bridge ACK."""
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_callbacks import TebTurnSupervisorCallbacksMixin  # noqa: E402
from lste_topo_access.msg import PlannerCommandContract  # noqa: E402


class ContractFixture(TebTurnSupervisorCallbacksMixin):
    def __init__(self, ack, identity, state=None):
        self.active_action = False
        self.latest_bridge_release_ack = ack
        self.bridge_release_ack_history = [] if ack is None else [ack]
        self.bridge_terminal_history = [] if ack is None else [ack]
        self.planner_contract_pending = {
            "identity": identity,
            "command": object(),
            "state": (PlannerCommandContract.STATE_INVALIDATED if state is None else state),
            "reason": "terminal_boundary_route_release",
            "producer": "teb_goal_bridge",
        }
        self.applied = []
        self.events = []

    def _lifecycle_transaction_id_locked(self):
        return 12345

    def _apply_planner_contract_locked(self, identity, command, state, reason, producer):
        self.applied.append((identity, state, reason, producer))
        self.planner_contract_pending = None
        return True

    def publish_status_locked(self, event, **kwargs):
        self.events.append((event, kwargs))


def identity(route=90, generation=166, lifecycle=12345):
    return {
        "route_id": route,
        "transaction_id": route,
        "graph_transaction_id": 1,
        "action_generation": generation,
        "lifecycle_transaction_id": lifecycle,
        "map_epoch": 1,
        "origin_epoch": 1,
        "projection_epoch": 1,
    }


def ack(route=90, generation=166, lifecycle=12345, active=False,
        event="lease_release_ack", lease="released", active_generation=None):
    return {
        "active": active,
        "event": event,
        "controller_lease": lease,
        "frontier_lease_released_route_id": route,
        "action_generation": generation,
        "active_action_generation": (
            generation if active_generation is None else active_generation
        ),
        "active_action_lifecycle_transaction_id": lifecycle,
    }


class TerminalReleaseAckTest(unittest.TestCase):
    def test_exact_terminal_release_ack_settles_pending(self):
        # Live Bridge status reports the released action generation separately
        # from the next active-action generation used by the terminal contract.
        fixture = ContractFixture(
            ack(generation=165, active_generation=166), identity(generation=166)
        )
        self.assertTrue(fixture._promote_pending_planner_contract_locked())
        self.assertEqual(len(fixture.applied), 1)
        self.assertIsNone(fixture.planner_contract_pending)

    def test_mismatched_or_missing_ack_keeps_terminal_pending(self):
        cases = [
            None,
            ack(route=91),
            ack(generation=165),
            ack(lifecycle=12346),
            ack(active=True),
            ack(event="heartbeat"),
            ack(lease="active"),
        ]
        for release in cases:
            with self.subTest(release=release):
                fixture = ContractFixture(release, identity())
                self.assertFalse(fixture._promote_pending_planner_contract_locked())
                self.assertEqual(fixture.applied, [])
                self.assertIsNotNone(fixture.planner_contract_pending)

    def test_recent_exact_ack_survives_a_later_unrelated_ack(self):
        fixture = ContractFixture(ack(route=90), identity())
        fixture.bridge_release_ack_history.append(ack(route=91, generation=167))
        self.assertTrue(fixture._promote_pending_planner_contract_locked())
        self.assertEqual(len(fixture.applied), 1)

    def test_active_contract_for_exact_released_route_is_discarded(self):
        release = ack(
            generation=165, active_generation=166, event="transport_terminal"
        )
        fixture = ContractFixture(
            release,
            identity(generation=165),
            PlannerCommandContract.STATE_ACTIVE,
        )
        fixture.planner_contract_pending["producer"] = "persistent_teb_local_planner"
        self.assertFalse(fixture._promote_pending_planner_contract_locked())
        self.assertEqual(fixture.applied, [])
        self.assertIsNone(fixture.planner_contract_pending)
        self.assertTrue(any(
            event == "planner_contract_pending_ignored"
            and fields.get("reason") == "planner_active_contract_for_released_route"
            for event, fields in fixture.events
        ))

    def test_future_or_unconfirmed_active_contract_stays_pending(self):
        cases = [
            (ack(route=90), identity(route=91), False),
            (ack(event="heartbeat"), identity(), False),
            (ack(lease="active"), identity(), False),
            (ack(generation=165), identity(), False),
            (ack(lifecycle=12346), identity(), False),
            (ack(), identity(), True),
        ]
        for terminal, contract, active in cases:
            with self.subTest(terminal=terminal, contract=contract, active=active):
                fixture = ContractFixture(
                    terminal, contract, PlannerCommandContract.STATE_ACTIVE
                )
                fixture.planner_contract_pending["producer"] = (
                    "persistent_teb_local_planner"
                )
                fixture.active_action = active
                self.assertFalse(
                    fixture._promote_pending_planner_contract_locked()
                )
                self.assertIsNotNone(fixture.planner_contract_pending)
                self.assertEqual(fixture.applied, [])

    def test_active_or_nonterminal_contract_is_never_settled_by_release_ack(self):
        fixture = ContractFixture(ack(), identity(), PlannerCommandContract.STATE_ACTIVE)
        self.assertFalse(fixture._terminal_contract_matches_release_ack_locked(
            identity(), PlannerCommandContract.STATE_ACTIVE, "teb_goal_bridge"
        ))
        self.assertFalse(fixture._terminal_contract_matches_release_ack_locked(
            identity(), PlannerCommandContract.STATE_INVALIDATED, "persistent_teb_local_planner"
        ))


if __name__ == "__main__":
    unittest.main()
