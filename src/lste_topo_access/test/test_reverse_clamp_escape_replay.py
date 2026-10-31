"""Replay the sealed L/T failure snapshots through the new escape trigger.

Runs 20261031_183336 (L) and 20261031_195131 (T) produced seven failure
snapshots.  Six of them show a planner linear command of -0.01 -- TEB's reverse
entry sample saturated at ``max_vel_x_backwards`` -- which the forward-only mux
zeroed, leaving only rotation.  The plugin's clearance-escape trigger used
``fabs(linear) <= 0.001`` and therefore never recognised them.

The trigger is now ``linear <= 0.001`` (no forward component at all) combined
with a clearance-deficient start pose.  This test replays the real snapshots
through that predicate and asserts exactly which ones it would rescue, so the
change is verified against recorded data rather than a synthetic fixture.
"""

from pathlib import Path
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

RUNS = {
    "L": LOGS / "20261031_183336",
    "T": LOGS / "20261031_195131",
}

REQUIRED_CLEARANCE = 0.52

# Expected outcome per snapshot under the new trigger.  L#1 and L#2 sit in open
# space (0.99 m clearance), so they are the reorientation fix's job, not the
# escape's.
EXPECTED = {
    ("L", "0001"): False,
    ("L", "0002"): False,
    ("L", "0003"): True,
    ("T", "0001"): True,
    ("T", "0002"): True,
    ("T", "0003"): True,
    ("T", "0004"): True,
}


def planner_linear(sample):
    for key in ("teb_planner_cmd", "teb_cmd"):
        value = sample.get(key)
        if isinstance(value, (list, tuple)) and value:
            return float(value[0])
        if isinstance(value, dict) and "linear_x" in value:
            return float(value["linear_x"])
    supervisor = sample.get("teb_turn_supervisor") or {}
    command = supervisor.get("planner_command") or [0.0, 0.0]
    return float(command[0])


def clearance(sample):
    profile = zcd.clearance_profile(sample.get("local_costmap_window"))
    return None if profile is None else profile["lethal_m"]


def old_trigger(linear, gap):
    return gap is not None and gap < REQUIRED_CLEARANCE and abs(linear) <= 0.001


def new_trigger(linear, gap):
    return gap is not None and gap < REQUIRED_CLEARANCE and linear <= 0.001


def load(case):
    directory = RUNS[case]
    out = []
    for path in sorted(directory.glob("*_failure_[0-9][0-9][0-9][0-9].json")):
        with open(path) as handle:
            artifact = json.load(handle)
        sample = artifact.get("trigger_sample") or {}
        ident = path.name.rsplit("_failure_", 1)[-1][:4]
        out.append((ident, planner_linear(sample), clearance(sample),
                    round(sample.get("ros_time") or 0.0, 1)))
    return out


@unittest.skipUnless(all(d.is_dir() for d in RUNS.values()),
                     "sealed L/T failure snapshots not present")
class SealedSnapshotReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = {case: load(case) for case in RUNS}

    def test_expected_snapshot_counts(self):
        self.assertEqual(len(self.rows["L"]), 3)
        self.assertEqual(len(self.rows["T"]), 4)

    def test_new_trigger_matches_the_expected_set(self):
        for case, rows in self.rows.items():
            for ident, linear, gap, sim in rows:
                with self.subTest(case=case, snapshot=ident, sim=sim):
                    self.assertEqual(
                        new_trigger(linear, gap), EXPECTED[(case, ident)],
                        "linear=%.4f clearance=%s" % (linear, gap))

    def test_old_trigger_would_have_missed_them(self):
        rescued_new = sum(new_trigger(lin, gap)
                          for rows in self.rows.values()
                          for _, lin, gap, _ in rows)
        rescued_old = sum(old_trigger(lin, gap)
                          for rows in self.rows.values()
                          for _, lin, gap, _ in rows)
        self.assertEqual(rescued_old, 0)
        self.assertEqual(rescued_new, 5)

    def test_reverse_samples_really_are_at_the_backwards_bound(self):
        reverse = [lin for rows in self.rows.values()
                   for _, lin, _, _ in rows if lin < 0.0]
        self.assertTrue(reverse)
        for linear in reverse:
            self.assertGreaterEqual(linear, -0.01 - 1e-9)
            self.assertLessEqual(linear, -0.0007 + 1e-9)


class PluginTriggerContractTest(unittest.TestCase):
    """The C++ branch must use the widened predicate, not the old one."""

    @classmethod
    def setUpClass(cls):
        cls.source = (
            Path(__file__).resolve().parents[1]
            / "src/persistent_navigation_plugins.cpp"
        ).read_text(encoding="utf-8")

    def test_no_forward_component_is_the_trigger(self):
        self.assertIn("if (cmd_vel.linear.x <= 0.001 && clearance_deficient)",
                      self.source)
        # ``clearance_deficient`` must still come from the measured clearance,
        # not from a constant.
        self.assertIn("const bool clearance_deficient =", self.source)
        self.assertIn("measured_clearance < start_pose_clearance_threshold_",
                      self.source)

    def test_absolute_value_form_is_gone(self):
        self.assertNotIn("std::fabs(cmd_vel.linear.x) <= 0.001", self.source)

    def test_escape_is_still_bounded_and_forward_only(self):
        self.assertIn("clearance_escape_linear_velocity_", self.source)
        self.assertIn("clearance_escape_max_cycles_", self.source)
        self.assertIn("clearance_escape_margin_", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
