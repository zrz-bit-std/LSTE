"""Contract: a cached global plan may only serve the goal it was planned for.

Run ``20261030_220218`` deadlocked because ``StreamingNavfnPlanner`` handed the
previous route's cached plan to every new mission goal when Navfn could not
plan. The persistent local planner re-installed that stale endpoint and
announced it as reached for the new route, so the graph believed each dispatch
had completed at a fixed pose: 215 failure artifacts, one "endpoint reached"
every ~9 s, zero motion for the whole run.

These assertions pin the two guards that keep that from recurring:
retention is gated on the cached goal matching the requested goal, and the
local planner only publishes ``endpoint_reached`` when an endpoint was really
reported.
"""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/persistent_navigation_plugins.cpp"


class GlobalPlanRetentionContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_retention_is_gated_on_the_cached_goal(self):
        self.assertIn("const bool same_cached_goal =", self.source)
        self.assertIn(
            "cached_goal_.pose.position.x -\n"
            "                           selected_goal.pose.position.x",
            self.source,
        )
        self.assertIn("if (same_cached_goal) {", self.source)

    def test_retention_still_returns_the_previous_plan(self):
        self.assertIn("retained_previous_plan = true;", self.source)
        self.assertIn("return retained_previous_plan;", self.source)

    def test_arrival_is_only_claimed_when_an_endpoint_was_reported(self):
        self.assertIn("const bool endpoint_reported =", self.source)
        self.assertIn(
            'endpoint_reported ? "endpoint_reached" : "planner_no_endpoint"',
            self.source,
        )
        # The old unconditional claim must be gone.
        self.assertNotIn(
            'cmd_vel, PlannerCommandContract::STATE_ZERO, "endpoint_reached");',
            self.source,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
