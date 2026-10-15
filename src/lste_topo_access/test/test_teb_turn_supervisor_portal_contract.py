"""Portal routes must enter the managed TEB execution contract."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_contract import (
    FRONTIER_ENDPOINT_KIND,
    FRONTIER_SOURCE,
    LOCAL_EGRESS_KIND,
    PORTAL_TRANSITION_KIND,
    TURN_ROUTE_KIND,
    is_managed_frontier_route,
)


class TebTurnSupervisorPortalContractTest(unittest.TestCase):
    def test_portal_transition_is_a_managed_frontier_route(self):
        self.assertTrue(
            is_managed_frontier_route(PORTAL_TRANSITION_KIND, FRONTIER_SOURCE)
        )

    def test_existing_managed_routes_keep_their_contract(self):
        self.assertTrue(is_managed_frontier_route(TURN_ROUTE_KIND, FRONTIER_SOURCE))
        self.assertTrue(
            is_managed_frontier_route(FRONTIER_ENDPOINT_KIND, FRONTIER_SOURCE)
        )

    def test_unrelated_sources_and_routes_remain_passthrough(self):
        self.assertFalse(
            is_managed_frontier_route(PORTAL_TRANSITION_KIND, "target_tracker")
        )
        self.assertTrue(is_managed_frontier_route(LOCAL_EGRESS_KIND, FRONTIER_SOURCE))
        self.assertFalse(
            is_managed_frontier_route(LOCAL_EGRESS_KIND, "target_tracker")
        )


if __name__ == "__main__":
    unittest.main()
