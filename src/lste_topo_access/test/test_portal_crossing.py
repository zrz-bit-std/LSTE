"""Unit tests for odometry-based directional portal evidence."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_crossing import (
    portal_crossing_depth,
    portal_crossing_evidence,
    portal_execution_goal,
    portal_signed_distance,
    portal_source_side_is_proven,
)


class PortalCrossingEvidenceTest(unittest.TestCase):
    def test_crossing_depth_uses_door_geometry_not_room_coverage(self):
        # A 0.52 m certified-clearance opening is crossed once the base centre
        # is 0.26 m on its far side. A 1.25 m room-observation radius would
        # incorrectly reject that physical transition.
        self.assertAlmostEqual(portal_crossing_depth(0.52), 0.26)
        evidence = portal_crossing_evidence(
            start_xy=(2.0, 0.0),
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            terminal_xy=(5.284, 0.0),
            gate_approached=True,
            minimum_destination_depth=portal_crossing_depth(0.52),
        )
        self.assertTrue(evidence.verified)

    def test_portal_command_uses_its_frozen_odom_destination(self):
        frame, goal = portal_execution_goal(
            "portal_transition", "map", (10.1, 13.2), (10.0, 13.0)
        )
        self.assertEqual(frame, "odom")
        self.assertEqual(goal, (10.0, 13.0))

        frame, goal = portal_execution_goal(
            "frontier_endpoint", "map", (10.1, 13.2), (10.0, 13.0)
        )
        self.assertEqual(frame, "map")
        self.assertEqual(goal, (10.1, 13.2))

    def test_destination_side_depth_proves_a_crossing(self):
        evidence = portal_crossing_evidence(
            start_xy=(2.0, 0.0),
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            terminal_xy=(5.6, 0.0),
            gate_approached=True,
            minimum_destination_depth=0.4,
        )

        self.assertTrue(evidence.verified)
        self.assertEqual(evidence.reason, "physical_gate_crossed")
        self.assertAlmostEqual(evidence.terminal_signed_distance, 0.6)

    def test_continuous_directed_plane_crossing_does_not_need_gate_sample(self):
        """A sparse planning tick can miss the gate centre on a valid route."""
        evidence = portal_crossing_evidence(
            start_xy=(2.0, 0.0),
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            terminal_xy=(5.6, 0.0),
            gate_approached=False,
            minimum_destination_depth=0.4,
        )

        self.assertTrue(evidence.verified)
        self.assertEqual(evidence.reason, "physical_gate_crossed")

    def test_goal_circle_success_on_source_side_does_not_cross(self):
        evidence = portal_crossing_evidence(
            start_xy=(2.0, 0.0),
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            terminal_xy=(4.9, 0.0),
            gate_approached=True,
            minimum_destination_depth=0.4,
        )

        self.assertFalse(evidence.verified)
        self.assertEqual(evidence.reason, "physical_destination_side_not_reached")

    def test_no_start_side_proof_does_not_close_a_source_place(self):
        evidence = portal_crossing_evidence(
            start_xy=(5.2, 0.0),
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            terminal_xy=(6.0, 0.0),
            gate_approached=True,
            minimum_destination_depth=0.4,
        )

        self.assertFalse(evidence.verified)
        self.assertEqual(evidence.reason, "physical_source_side_unproven")

    def test_transaction_source_proof_allows_retry_from_the_gate(self):
        """A second route starts at the gate but still belongs to one crossing."""
        evidence = portal_crossing_evidence(
            start_xy=(5.2, 0.0),
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            terminal_xy=(5.8, 0.0),
            gate_approached=True,
            minimum_destination_depth=0.4,
            source_side_proven=True,
        )

        self.assertTrue(evidence.verified)
        self.assertEqual(evidence.reason, "physical_gate_crossed")

    def test_destination_view_can_supply_explicit_preobserved_crossing(self):
        evidence = portal_crossing_evidence(
            start_xy=(5.8, 0.0),
            gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            terminal_xy=(5.8, 0.0),
            gate_approached=False,
            minimum_destination_depth=0.4,
            preobserved=True,
        )

        self.assertTrue(evidence.verified)
        self.assertEqual(evidence.reason, "portal_crossing_preobserved")

    def test_directional_portal_requires_a_source_side_start(self):
        self.assertAlmostEqual(
            portal_signed_distance((2.0, 0.0), (5.0, 0.0), (7.0, 0.0)),
            -3.0,
        )
        self.assertTrue(
            portal_source_side_is_proven(
                (2.0, 0.0), (5.0, 0.0), (7.0, 0.0),
            )
        )
        self.assertFalse(
            portal_source_side_is_proven(
                (5.2, 0.0), (5.0, 0.0), (7.0, 0.0),
            )
        )


if __name__ == "__main__":
    unittest.main()
