"""Pure tests for durable Portal projection and crossing geometry."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_durable_portal_route import (  # noqa: E402
    crossing_goal,
    project_record,
)
from global_frontier_portal_crossing import (  # noqa: E402
    portal_required_direction_depth,
)


class DurablePortalRouteTest(unittest.TestCase):
    def test_projected_goal_is_normal_to_measured_section(self):
        record=dict(id=2,source_place_id=1,physical_gate=(0.,0.),physical_destination=(2.,1.),
                    physical_bounds=((0.,-1.),(0.,1.)))
        projection=project_record(record,lambda x,y:(x+3.,y+4.))
        self.assertEqual(projection.gate_xy,(3.,4.))
        self.assertAlmostEqual(projection.destination_xy[1],4.)
        self.assertAlmostEqual(projection.destination_xy[0],3.+5.**.5)
        self.assertEqual(record['physical_destination'],(2.,1.))

    def test_current_physical_projection_wins_over_stale_map_projection(self):
        record = {
            "id": 7,
            "source_place_id": 3,
            "physical_gate": [10.0, 2.0],
            "physical_destination": [11.0, 2.0],
            "map_gate": [1.0, 1.0],
            "map_destination": [2.0, 1.0],
        }
        projection = project_record(record, lambda x, y: (x + 4.0, y - 1.0))

        self.assertEqual(projection.gate_xy, (14.0, 1.0))
        self.assertEqual(projection.destination_xy, (15.0, 1.0))

    def test_last_map_projection_is_a_fallback_when_tf_is_unavailable(self):
        record = {
            "id": 8,
            "source_place_id": 4,
            "physical_gate": [10.0, 2.0],
            "physical_destination": [11.0, 2.0],
            "map_gate": [1.0, 1.0],
            "map_destination": [2.0, 1.0],
        }

        projection = project_record(record, lambda *_args: None)

        self.assertEqual(projection.gate_xy, (1.0, 1.0))
        self.assertEqual(projection.destination_xy, (2.0, 1.0))

    def test_crossing_goal_extends_short_destination_evidence(self):
        projection = project_record(
            {
                "id": 9,
                "source_place_id": 5,
                "physical_gate": [0.0, 0.0],
                "physical_destination": [0.1, 0.0],
            }
        )

        self.assertEqual(crossing_goal(projection, 0.6), (0.6, 0.0))


class PortalGoalDepthPolicyTest(unittest.TestCase):
    def test_straight_section_adds_teb_tolerance_and_margin(self):
        record = {
            "gp_authentication_state": "completed",
            "gp_authentication_required_depth": 1.3,
            "physical_gate": [0.0, 0.0],
            "physical_destination": [0.0, 2.0],
            "physical_bounds": [[-1.0, 0.0], [1.0, 0.0]],
        }

        depth = portal_required_direction_depth(
            0.52, record, teb_goal_tolerance=0.5, margin=0.1,
        )

        self.assertAlmostEqual(depth, 1.9, places=6)

    def test_diagonal_section_compensates_for_normal_angle(self):
        record = {
            "gp_authentication_state": "completed",
            "gp_authentication_required_depth": 1.3,
            "physical_gate": [12.555, -1.196],
            "physical_destination": [12.4395, -2.9854],
            "physical_bounds": [
                [11.3499, -1.9086],
                [13.7604, -0.4841],
            ],
        }

        depth = portal_required_direction_depth(
            0.52, record, teb_goal_tolerance=0.5, margin=0.1,
        )

        self.assertAlmostEqual(depth, 2.17, places=2)
        self.assertGreater(depth, 1.9)

    def test_goal_depth_without_section_bounds_stays_conservative(self):
        record = {
            "gp_authentication_state": "completed",
            "gp_authentication_required_depth": 1.3,
            "physical_gate": [0.0, 0.0],
            "physical_destination": [0.0, 2.0],
        }

        depth = portal_required_direction_depth(
            0.52, record, teb_goal_tolerance=0.5, margin=0.1,
        )

        self.assertAlmostEqual(depth, 1.9, places=6)


if __name__ == "__main__":
    unittest.main()
