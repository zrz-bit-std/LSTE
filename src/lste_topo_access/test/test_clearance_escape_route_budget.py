"""The clearance escape must be reachable inside one failed route.

The plugin's bounded clearance escape is the only remedy for a robot that TEB
refuses to move because its own start pose sits inside the forbidden clearance
band (``footprint 0.30 + min_obstacle_dist 0.22 = 0.52 m``).

Two sealed return loops failed without ever emitting one escape:

* ``20261101_002029`` (T, node 6 branch 11): six ``authentication_physical_stall
  :returning`` cycles, then ``portal_authentication_return_unrecoverable``.
* ``20261101_050044`` (rotated T, node 14 branch 39): the same shape.

In both runs the bridge logged 130 ``planner_contract_planner_zero_translation``
contracts and zero ``planner_clearance_escape`` contracts.  The escape was armed
only after ``4 * clearance_escape_trigger_cycles_`` control cycles -- 160 cycles
at 20 Hz, i.e. 8 s -- while move_base fails a route that makes no physical
progress after ``controller_patience`` (5 s) and the following ``setPlan``
resets ``infeasible_cycle_count_``.  The budget could therefore never mature.

These tests pin the reachability argument to the sealed runs and to the source
contract, so a future tuning change cannot silently reintroduce the dead band.
"""

import json
import os
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "src/lste_topo_access/src/persistent_navigation_plugins.cpp"
LOGS = Path(os.environ.get(
    "LSTE_TJUNCTION_LOGS",
    "/home/zrz/work/lste_ws/runtime/targeted_navigation/t_junction_small/logs",
))

CONTROL_HZ = 20.0
TRIGGER_CYCLES = 40          # persistent_clearance_escape_trigger_cycles
OLD_FACTOR = 4               # the removed multiplier
ROUTE_PATIENCE_SECONDS = 5.0  # move_base controller_patience in teb_navigation

SEALED_RETURN_LOOPS = ("20261101_002029", "20261101_050044")


class SourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_zero_translation_branch_uses_the_plain_trigger(self):
        self.assertIn(
            "infeasible_cycle_count_ >= clearance_escape_trigger_cycles_ &&",
            self.source,
        )

    def test_no_multiplied_budget_remains(self):
        self.assertNotIn("4 * clearance_escape_trigger_cycles_", self.source)

    def test_reason_is_documented(self):
        self.assertIn("controller_patience", self.source)


class ReachabilityTest(unittest.TestCase):
    def test_old_budget_exceeded_one_route_life(self):
        old_seconds = OLD_FACTOR * TRIGGER_CYCLES / CONTROL_HZ
        self.assertGreater(old_seconds, ROUTE_PATIENCE_SECONDS)

    def test_new_budget_fits_inside_one_route_life(self):
        new_seconds = TRIGGER_CYCLES / CONTROL_HZ
        self.assertLess(new_seconds, ROUTE_PATIENCE_SECONDS)

    def test_sealed_routes_were_shorter_than_the_old_budget(self):
        checked = 0
        for timestamp in SEALED_RETURN_LOOPS:
            metrics = LOGS / timestamp / ("%s_navigation_metrics.log" % timestamp)
            if not metrics.is_file():
                continue
            lifetimes = _route_lifetimes(metrics)
            if not lifetimes:
                continue
            checked += 1
            longest = max(lifetimes)
            # Every observed route died before the old 8 s trigger could fire.
            self.assertLess(longest, OLD_FACTOR * TRIGGER_CYCLES / CONTROL_HZ)
            # At least one route lived long enough for the new 2 s trigger.
            self.assertGreaterEqual(longest, TRIGGER_CYCLES / CONTROL_HZ)
        if not checked:
            self.skipTest("sealed return loops are not present")


def _route_lifetimes(metrics):
    """Start-to-failure seconds for routes that hit a zero-translation stall.

    ``move_base_dispatch`` carries no route id, so the start is the first
    bridge/transaction event that names the route.  Only routes whose contract
    reported ``planner_contract_planner_zero_translation`` are kept: those are
    exactly the routes the clearance escape exists to rescue.
    """
    started = {}
    stalled = {}
    lifetimes = []
    with metrics.open(errors="replace") as handle:
        for line in handle:
            index = line.find("data=")
            if index < 0:
                continue
            try:
                data = json.loads(line[index + 5:])
            except ValueError:
                continue
            event = line.split("event=")[1].split()[0] if "event=" in line else ""
            if event not in ("teb_bridge_event", "mission_goal_transaction"):
                continue
            route_id = int(data.get("route_id", 0) or 0)
            if route_id <= 0:
                continue
            stamp = float(data.get("ros_time", 0.0) or 0.0)
            started.setdefault(route_id, stamp)
            reason = str(data.get("reason", "") or "")
            if reason == "planner_contract_planner_zero_translation":
                stalled.setdefault(route_id, stamp)
            elif reason == "move_base_failed_terminal" and route_id in stalled:
                # The escape budget starts when the first zero-translation
                # contract appears, not when the route was dispatched.
                lifetime = stamp - stalled[route_id]
                if lifetime > 0.0:
                    lifetimes.append(lifetime)
    return lifetimes


if __name__ == "__main__":
    unittest.main(verbosity=2)
