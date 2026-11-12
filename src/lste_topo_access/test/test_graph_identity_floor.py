"""The graph identity floor that keeps authentication routes acceptable.

env04 ``20261112_220714``: node 6 authenticated three doorways after a portal
crossing had raised the controller high water to 40.  The authentication routes
carried freshly allocated IDs 29..39, one per activation, so every command was
discarded as ``graph_transaction_id_below_high_water``; the car never moved and
all three doorways timed out.  Node 5, which ran before that crossing, had zero
rejections and completed all three.

The floor absorbs every graph ID this node has seen (allocation, release,
bridge status) so the next allocation clears the controller mark locally.
The terminal check compares the immutable dispatch snapshot instead of the
mutable graph transaction, otherwise a freshly numbered route would reject its
own arrival.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_activation import GlobalFrontierActivationMixin  # noqa: E402
from global_frontier_terminal_validation import (  # noqa: E402
    GlobalFrontierTerminalValidationMixin,
)


class Fixture(
    GlobalFrontierActivationMixin,
    GlobalFrontierTerminalValidationMixin,
):
    pass


class GraphIdentityFloorTest(unittest.TestCase):
    def test_the_floor_only_moves_up(self):
        fixture = Fixture()
        fixture.graph_route_identity_floor = 0
        self.assertEqual(fixture._raise_graph_identity_floor(40), 40)
        self.assertEqual(fixture._raise_graph_identity_floor(29), 40)
        self.assertEqual(fixture._raise_graph_identity_floor(None, "", 41), 41)

    def test_none_values_are_ignored(self):
        fixture = Fixture()
        fixture.graph_route_identity_floor = 5
        self.assertEqual(
            fixture._raise_graph_identity_floor(None, "x", 0), 5,
        )


class TerminalDispatchSnapshotTest(unittest.TestCase):
    def fixture(self, dispatch=None, transaction_id=40):
        fixture = Fixture()
        fixture._bridge_dispatch_contracts = dispatch or {}
        fixture.graph_route_action_transaction = SimpleNamespace(
            graph_transaction_id=transaction_id,
        )
        return fixture

    def test_the_dispatch_snapshot_wins(self):
        fixture = self.fixture(
            {42: {"graph_transaction_id": 41}}, transaction_id=40,
        )
        self.assertEqual(
            fixture._expected_terminal_graph_transaction_id(42), 41,
        )

    def test_without_a_snapshot_the_transaction_is_used(self):
        fixture = self.fixture({}, transaction_id=40)
        self.assertEqual(
            fixture._expected_terminal_graph_transaction_id(42), 40,
        )

    def test_a_zero_snapshot_falls_back(self):
        fixture = self.fixture(
            {42: {"graph_transaction_id": 0}}, transaction_id=40,
        )
        self.assertEqual(
            fixture._expected_terminal_graph_transaction_id(42), 40,
        )

    def test_an_unknown_route_falls_back(self):
        fixture = self.fixture(
            {7: {"graph_transaction_id": 41}}, transaction_id=40,
        )
        self.assertEqual(
            fixture._expected_terminal_graph_transaction_id(42), 40,
        )


if __name__ == "__main__":
    unittest.main()
