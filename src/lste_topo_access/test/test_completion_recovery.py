"""Pure tests for graph-level completion recovery decisions."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_completion_recovery import (  # noqa: E402
    ACTION_CONTINUE,
    ACTION_BLOCK,
    ACTION_COMPLETE,
    ACTION_REOBSERVE,
    ACTION_TRANSIT,
    ACTION_WAIT,
    GraphCompletionStateMachine,
    STATE_BLOCKED,
    STATE_COMPLETE,
    STATE_RECOVERING,
    STATE_WAITING,
)


def gate(complete, reasons=(), **counts):
    return SimpleNamespace(
        complete=bool(complete),
        reasons=tuple(reasons),
        counts=counts,
    )


class CompletionRecoveryTest(unittest.TestCase):
    def test_unresolved_work_gets_one_reobserve_transition(self):
        machine = GraphCompletionStateMachine()

        decision = machine.decide(
            gate(False, ("unresolved_work_items",), unresolved_work_items=1),
            evidence_signature=(7, "work"),
        )

        self.assertEqual(decision.state, STATE_RECOVERING)
        self.assertEqual(decision.action, ACTION_REOBSERVE)
        self.assertFalse(decision.terminal)
        self.assertTrue(decision.transition)

    def test_same_unresolved_evidence_becomes_explicitly_blocked(self):
        machine = GraphCompletionStateMachine()
        unresolved = gate(
            False,
            ("unresolved_work_items",),
            unresolved_work_items=1,
        )
        machine.decide(unresolved, evidence_signature=(7, "work"))

        decision = machine.decide(unresolved, evidence_signature=(7, "work"))

        self.assertEqual(decision.state, STATE_BLOCKED)
        self.assertEqual(decision.action, ACTION_BLOCK)
        self.assertTrue(decision.terminal)
        self.assertTrue(decision.transition)

        # A repeated timer tick is quiet and remains an explicit block. It
        # cannot be mistaken for successful frontier exhaustion.
        repeated = machine.decide(unresolved, evidence_signature=(7, "work"))
        self.assertEqual(repeated.action, ACTION_BLOCK)
        self.assertFalse(repeated.transition)

    def test_new_evidence_reopens_a_previously_blocked_obligation(self):
        machine = GraphCompletionStateMachine()
        unresolved = gate(
            False,
            ("unresolved_portal_probes",),
            unresolved_portal_probes=1,
        )
        machine.decide(unresolved, evidence_signature=(7, "old"))
        machine.decide(unresolved, evidence_signature=(7, "old"))

        decision = machine.decide(unresolved, evidence_signature=(7, "new"))

        self.assertEqual(decision.state, STATE_RECOVERING)
        self.assertEqual(decision.action, ACTION_REOBSERVE)
        self.assertFalse(decision.terminal)

    def test_reachable_pending_place_prefers_graph_transit(self):
        machine = GraphCompletionStateMachine()

        decision = machine.decide(
            gate(False, ("unresolved_work_items",), unresolved_work_items=2),
            graph_transit_available=True,
            evidence_signature=(2, "transit"),
        )

        self.assertEqual(decision.action, ACTION_TRANSIT)
        self.assertEqual(decision.state, STATE_RECOVERING)

    def test_validation_and_portal_transactions_are_wait_states(self):
        machine = GraphCompletionStateMachine()
        unresolved = gate(
            False,
            ("unresolved_portal_probes",),
            unresolved_portal_probes=1,
        )

        validation = machine.decide(
            unresolved,
            validation_pending=True,
            evidence_signature=(1, "validation"),
        )
        transaction = machine.decide(
            unresolved,
            active_transaction=True,
            evidence_signature=(1, "transaction"),
        )

        self.assertEqual(validation.state, STATE_WAITING)
        self.assertEqual(validation.action, ACTION_WAIT)
        self.assertFalse(validation.terminal)
        self.assertEqual(transaction.state, STATE_WAITING)
        self.assertEqual(transaction.action, ACTION_WAIT)

    def test_recovery_reopens_after_validation_wait_ends(self):
        machine = GraphCompletionStateMachine()
        unresolved = gate(
            False,
            ("unresolved_work_items",),
            unresolved_work_items=1,
        )

        machine.decide(
            unresolved,
            validation_pending=True,
            evidence_signature=(1, "same"),
        )
        decision = machine.decide(
            unresolved,
            validation_pending=False,
            evidence_signature=(1, "same"),
        )

        self.assertEqual(decision.action, ACTION_REOBSERVE)
        self.assertEqual(decision.state, STATE_RECOVERING)

    def test_settled_graph_is_the_only_completion_terminal(self):
        machine = GraphCompletionStateMachine()

        decision = machine.decide(
            gate(True),
            evidence_signature=(1, "settled"),
        )

        self.assertEqual(decision.state, STATE_COMPLETE)
        self.assertEqual(decision.action, ACTION_COMPLETE)
        self.assertTrue(decision.terminal)

    def test_executable_viewpoint_keeps_graph_in_search(self):
        machine = GraphCompletionStateMachine()

        decision = machine.decide(
            gate(
                False,
                ("unresolved_work_items",),
                unresolved_work_items=1,
            ),
            has_executable_viewpoint=True,
            evidence_signature=(1, "candidate"),
        )

        self.assertEqual(decision.action, ACTION_CONTINUE)
        self.assertEqual(decision.state, "searching")
        self.assertFalse(decision.terminal)

    def test_live_structural_frontier_blocks_completion_claim(self):
        machine = GraphCompletionStateMachine()

        decision = machine.decide(
            gate(True),
            structural_candidate_pending=True,
            evidence_signature=(1, "remote-frontier"),
        )

        self.assertEqual(decision.action, ACTION_REOBSERVE)
        self.assertNotEqual(decision.action, ACTION_COMPLETE)


if __name__ == "__main__":
    unittest.main()
