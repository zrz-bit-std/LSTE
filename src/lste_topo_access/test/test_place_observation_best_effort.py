"""Regression tests for the bounded best-effort Place coverage exit."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_place_memory import FrontierRegionMemory  # noqa: E402


class PlaceObservationBestEffortTest(unittest.TestCase):
    def test_best_effort_marks_an_open_region_once(self):
        memory = FrontierRegionMemory(
            radius=3.0,
            information_delta=8,
            stagnation_timeout=12,
            failure_limit=2,
            limit=16,
        )
        region, _tier = memory.enter(3.4, 0.0, now=1.0)
        self.assertEqual(int(region.get("endpoint_observations", 0)), 0)

        marked = memory.mark_observation_best_effort(
            region["id"], 10.0, reason="observation_work_exhausted",
        )
        self.assertIs(marked, region)
        self.assertEqual(int(region["endpoint_observations"]), 1)
        self.assertTrue(region["covered_best_effort"])
        self.assertEqual(region["last_reason"], "observation_work_exhausted")

        # A second call must not manufacture extra coverage evidence.
        memory.mark_observation_best_effort(region["id"], 20.0)
        self.assertEqual(int(region["endpoint_observations"]), 1)

    def test_closed_region_is_not_marked(self):
        memory = FrontierRegionMemory(
            radius=3.0,
            information_delta=8,
            stagnation_timeout=12,
            failure_limit=2,
            limit=16,
        )
        region, _tier = memory.enter(3.4, 0.0, now=1.0)
        memory.close_place(region["id"], 2.0) if hasattr(memory, "close_place") else None
        # close_place lives on the lifecycle mixin; if the state is not open,
        # the guard below is what matters.
        region["state"] = "dormant"
        self.assertIsNone(
            memory.mark_observation_best_effort(region["id"], 3.0)
        )


if __name__ == "__main__":
    unittest.main()
