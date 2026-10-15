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


class DurablePortalRouteTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
