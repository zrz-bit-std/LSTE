"""Pure tests for the non-preemptible physical Portal transaction."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_transaction import (
    PORTAL_TX_ABORTED,
    PORTAL_TX_CROSSING_VERIFIED,
    PORTAL_TX_DESTINATION_STANDOFF,
    PORTAL_TX_IDLE,
    PORTAL_TX_PLACE_COMMIT,
    PORTAL_TX_SOURCE_PROBE,
    PORTAL_TX_THROAT,
    PortalTransaction,
)


class PortalTransactionTest(unittest.TestCase):
    def test_unconfirmed_standoff_retry_reuses_only_same_direction_source_evidence(self):
        tx = PortalTransaction()
        tx.start(11, portal_id=3, source_place_id=2, gate_xy=(1, 0),
                 destination_xy=(2, 0), source_side_proven=True,
                 source_signed_distance=-1.)
        tx.abort("portal_retry_after_unconfirmed_standoff")
        tx.finish("portal_transaction_closed_before_retry")
        self.assertTrue(tx.source_evidence_matches(3, 2, (1, 0), (3, 0)))
        self.assertFalse(tx.source_evidence_matches(3, 1, (1, 0), (3, 0)))
        self.assertFalse(tx.source_evidence_matches(4, 2, (1, 0), (3, 0)))
        self.assertFalse(tx.source_evidence_matches(3, 2, (1, 0), (0, 0)))
        self.assertFalse(tx.source_evidence_matches(3, 2, (1.2, 0), (3, 0)))
        restarted = tx.start(12, portal_id=3, source_place_id=2,
                             gate_xy=(1, 0), destination_xy=(3, 0))
        self.assertTrue(restarted.source_side_proven)
        self.assertEqual(restarted.source_signed_distance, -1.)
        tx.crossing_verified(12);tx.place_commit();tx.finish()
        self.assertFalse(tx.source_evidence_matches(3, 2, (1, 0), (3, 0)))

    def test_phase_order_is_explicit_and_monotonic(self):
        tx = PortalTransaction()
        self.assertEqual(tx.state, PORTAL_TX_IDLE)
        tx.start(11, portal_id=3, source_place_id=2, gate_xy=(1, 0), destination_xy=(2, 0))
        self.assertEqual(tx.state, PORTAL_TX_SOURCE_PROBE)
        tx.gate_reached(11)
        self.assertEqual(tx.state, PORTAL_TX_THROAT)
        tx.destination_standoff(11)
        self.assertEqual(tx.state, PORTAL_TX_DESTINATION_STANDOFF)
        tx.crossing_verified(11)
        self.assertEqual(tx.state, PORTAL_TX_CROSSING_VERIFIED)
        tx.place_commit()
        self.assertEqual(tx.state, PORTAL_TX_PLACE_COMMIT)
        tx.finish()
        self.assertEqual(tx.state, PORTAL_TX_IDLE)

    def test_crossing_evidence_fills_sparse_missing_stages(self):
        tx = PortalTransaction()
        tx.start(4, portal_id=8, source_place_id=1)
        tx.crossing_verified(4)
        self.assertEqual(tx.state, PORTAL_TX_CROSSING_VERIFIED)
        self.assertEqual(
            tx.snapshot().last_transition,
            "destination_standoff_to_crossing_verified",
        )

    def test_ordinary_frontier_cannot_preempt_active_edge(self):
        tx = PortalTransaction()
        tx.start(7, portal_id=2, source_place_id=5)
        self.assertFalse(tx.preemption_allowed("ordinary_frontier"))
        self.assertFalse(tx.preemption_allowed("target_replan"))
        self.assertFalse(tx.route_allowed(8, "frontier_endpoint"))
        self.assertTrue(tx.route_allowed(7, "portal_transition"))
        self.assertTrue(tx.route_allowed(8, "local_egress"))

    def test_same_edge_retry_preserves_identity_and_blocks_new_edge(self):
        tx = PortalTransaction()
        tx.start(9, portal_id=12, source_place_id=3)
        tx.gate_reached(9)
        tx.retry()
        rebound = tx.bind_route(10)
        self.assertEqual(rebound.route_id, 10)
        self.assertEqual(rebound.portal_id, 12)
        self.assertEqual(rebound.source_place_id, 3)
        self.assertEqual(rebound.retry_count, 1)
        self.assertFalse(tx.route_allowed(11, "portal_transition"))

    def test_unstarted_route_can_be_rebound_without_counting_a_physical_retry(self):
        tx = PortalTransaction()
        tx.start(19, portal_id=3, source_place_id=1, gate_xy=(2, 0), destination_xy=(4, 0))

        released = tx.release_unstarted_route(19)
        self.assertEqual(released.state, PORTAL_TX_SOURCE_PROBE)
        self.assertEqual(released.route_id, 0)
        self.assertEqual(released.retry_count, 0)
        self.assertEqual(tx.route_allowed(20, "portal_transition"), False)

        rebound = tx.bind_route(20, reason="portal_pre_dispatch_route_rebound")
        self.assertEqual(rebound.portal_id, 3)
        self.assertEqual(tx.portal_id, 3)
        self.assertEqual(rebound.source_place_id, 1)
        self.assertEqual(rebound.route_id, 20)
        self.assertEqual(rebound.retry_count, 0)

    def test_source_side_evidence_survives_a_retry_started_at_the_gate(self):
        """A recovery route may begin on the gate's destination-side envelope."""
        tx = PortalTransaction()
        tx.start(
            9,
            portal_id=12,
            source_place_id=3,
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            source_side_proven=True,
            source_signed_distance=-2.0,
        )
        tx.gate_reached(9)
        tx.retry()
        rebound = tx.bind_route(10)

        self.assertTrue(rebound.source_side_proven)
        self.assertEqual(rebound.source_signed_distance, -2.0)
        # The retry can add a destination-side observation without erasing the
        # source proof that belongs to the parent physical transaction.
        observed = tx.observe_source_side(0.5, route_id=10)
        self.assertTrue(observed.source_side_proven)
        self.assertEqual(observed.source_signed_distance, -2.0)

    def test_commit_is_rejected_before_crossing(self):
        tx = PortalTransaction()
        tx.start(2, portal_id=1, source_place_id=1)
        self.assertIsNone(tx.place_commit())
        self.assertEqual(tx.state, PORTAL_TX_SOURCE_PROBE)

    def test_stale_arrival_identity_cannot_commit_the_current_edge(self):
        tx = PortalTransaction()
        tx.start(
            12,
            portal_id=6,
            source_place_id=4,
            gate_xy=(1.0, 2.0),
            destination_xy=(3.0, 2.0),
        )
        tx.crossing_verified(12)

        self.assertFalse(tx.commit_authorized(route_id=11, portal_id=6, source_place_id=4))
        self.assertIsNone(
            tx.place_commit(route_id=11, portal_id=6, source_place_id=4)
        )
        self.assertEqual(tx.state, PORTAL_TX_CROSSING_VERIFIED)

        self.assertIsNotNone(
            tx.place_commit(route_id=12, portal_id=6, source_place_id=4)
        )
        self.assertEqual(tx.state, PORTAL_TX_PLACE_COMMIT)

    def test_aborting_is_explicit(self):
        tx = PortalTransaction()
        tx.start(2, portal_id=1, source_place_id=1)
        tx.abort("no_egress_anchor")
        self.assertEqual(tx.state, PORTAL_TX_ABORTED)
        self.assertTrue(tx.preemption_allowed("ordinary_frontier"))


if __name__ == "__main__":
    unittest.main()
