"""A clearance stall must be materially below the clearance threshold.

Run 20261031_214617 produced 12 ``planner_clearance_stall`` snapshots. Every
one measured a nearest lethal cell at exactly **0.50 m** against the 0.52 m
requirement (footprint 0.30 + min_obstacle_dist 0.22), inside ~1 m wide
corridors, and the bounded escape never fired for any of them (0 escapes).

Two centimetres is half of a 0.1 m costmap cell, so the label was asserting a
cause the measurement cannot support. The plugin now separates:

* ``planner_clearance_stall`` -- deficit larger than the quantisation margin
  (default 0.05 m), and
* ``planner_zero_translation`` -- TEB published no translation without a
  material clearance problem.

This test replays the 12 real snapshots through the new rule.
"""

from pathlib import Path
import glob
import json
import os
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import zero_command_diagnosis as zcd  # noqa: E402


LOGS = Path(os.environ.get(
    "LSTE_TJUNCTION_LOGS",
    "/home/zrz/work/lste_ws/runtime/targeted_navigation/"
    "t_junction_small/logs",
))
RUN = LOGS / "20261031_214617"

THRESHOLD = 0.52
MARGIN = 0.05


def snapshots():
    rows = []
    for path in sorted(RUN.glob("*_failure_[0-9][0-9][0-9][0-9].json")):
        with open(path) as handle:
            artifact = json.load(handle)
        sample = artifact.get("trigger_sample") or {}
        if sample.get("planner_command_contract", {}).get("reason") != \
                "planner_clearance_stall":
            continue
        profile = zcd.clearance_profile(sample.get("local_costmap_window")) or {}
        rows.append((path.name[-9:-5], profile.get("lethal_m")))
    return rows


class MaterialityRuleTest(unittest.TestCase):
    def test_two_centimetre_deficit_is_not_material(self):
        self.assertFalse(zcd.clearance_stall_is_material(
            0.50, THRESHOLD, MARGIN))

    def test_ten_centimetre_deficit_is_material(self):
        self.assertTrue(zcd.clearance_stall_is_material(
            0.40, THRESHOLD, MARGIN))

    def test_exactly_at_the_margin_is_not_material(self):
        # 0.52 - 0.05 = 0.47; equality is still quantisation noise.
        self.assertFalse(zcd.clearance_stall_is_material(
            0.47, THRESHOLD, MARGIN))

    def test_missing_measurement_is_not_material(self):
        self.assertFalse(zcd.clearance_stall_is_material(None, THRESHOLD, MARGIN))

    def test_zero_translation_reason_is_classified_as_planner_zero(self):
        verdict = zcd.classify({
            "planner_command_contract": {
                "state": 0, "reason": "planner_zero_translation"},
            "teb_planner_cmd": [0.0, 0.0],
            "cmd_vel_mux": {"block_reason": "none", "filter_reason": "none",
                            "governor_reason": "none",
                            "input": {"linear_x": 0.0, "angular_z": 0.0},
                            "output": {"linear_x": 0.0, "angular_z": 0.0}},
            "teb_turn_supervisor": {"planner_command": [0.0, 0.0],
                                    "output_command": [0.0, 0.0],
                                    "output_decision": "planner_contract_boundary"},
        })
        self.assertEqual(verdict["cause"], zcd.PLANNER_ZERO_COMMAND)


@unittest.skipUnless(RUN.is_dir(), "sealed L run not present")
class SealedSnapshotReplayTest(unittest.TestCase):
    def test_all_twelve_clearance_stalls_were_quantisation_level(self):
        rows = snapshots()
        self.assertEqual(len(rows), 12, rows)
        for ident, clearance in rows:
            with self.subTest(snapshot=ident, clearance=clearance):
                self.assertAlmostEqual(clearance, 0.50, places=6)
                self.assertFalse(zcd.clearance_stall_is_material(
                    clearance, THRESHOLD, MARGIN))


class PluginRuleContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (
            Path(__file__).resolve().parents[1]
            / "src/persistent_navigation_plugins.cpp"
        ).read_text(encoding="utf-8")

    def test_deficit_must_exceed_the_margin(self):
        self.assertIn("start_pose_clearance_threshold_ - "
                      "start_pose_clearance_margin_", self.source)

    def test_reason_branches_on_materiality(self):
        self.assertIn('clearance_material ? "planner_clearance_stall"',
                      self.source)
        self.assertIn('"planner_zero_translation"', self.source)

    def test_threshold_and_margin_are_parameters(self):
        self.assertIn("persistent_start_pose_clearance_margin", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
