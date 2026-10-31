"""Recording gate: everything must be present except the two documented skips.

On 2026-10-31 the recording audit was relaxed for exactly two checks that the
installed stack cannot satisfy:

* ``enable_homotopy_class_planning`` is a static TEB parameter and never
  appears in dynamic_reconfigure ``parameter_updates``.
* ``/move_base/recovery_status`` only publishes when move_base actually
  recovers, so a clean run looks like a recording failure.

Both are recorded in the audit JSON under ``skipped`` with their evidence. Every
other requirement stays strict, and if the skipped parameter does appear it is
still validated.
"""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts" / "tests" / "targeted_navigation"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import check_place_dfs_recording as recording  # noqa: E402


def counts(**overrides):
    base = {topic: 5 for topic in recording.REQUIRED_TOPICS}
    base.update(overrides)
    return base


def config(**values):
    doubles = [SimpleNamespace(name="max_vel_x_backwards", value=0.01),
               SimpleNamespace(name="weight_kinematics_forward_drive", value=1000.0)]
    bools = [SimpleNamespace(name="allow_init_with_backwards_motion", value=False)]
    ints = [SimpleNamespace(name="max_number_classes", value=5)]
    if "enable_homotopy_class_planning" in values:
        bools.append(SimpleNamespace(
            name="enable_homotopy_class_planning",
            value=values["enable_homotopy_class_planning"]))
    for name in ("max_number_classes",):
        if name in values:
            ints[0] = SimpleNamespace(name=name, value=values[name])
    return SimpleNamespace(doubles=doubles, bools=bools, ints=ints)


class CountsGateTest(unittest.TestCase):
    def test_zero_recovery_is_not_a_recording_failure(self):
        result = recording.audit_counts(counts(**{"/move_base/recovery_status": 0}))
        self.assertTrue(result["passed"], result["missing_topics"])
        self.assertIn("/move_base/recovery_status", result["skipped"])

    def test_any_other_missing_topic_still_fails(self):
        result = recording.audit_counts(counts(**{"/map": 0}))
        self.assertFalse(result["passed"])
        self.assertIn("/map", result["missing_topics"])

    def test_master_costmap_samples_still_required(self):
        data = counts()
        data["/move_base/global_costmap/costmap"] = 0
        data["/move_base/global_costmap/costmap_updates"] = 1
        result = recording.audit_counts(data)
        self.assertFalse(result["passed"])


class TebConfigGateTest(unittest.TestCase):
    def test_absent_static_homotopy_param_is_skipped(self):
        result = recording.audit_teb_parameter_updates([config()])
        self.assertTrue(result["passed"], result["errors"])
        self.assertIn("enable_homotopy_class_planning", result["skipped"])

    def test_missing_dynamic_param_still_fails(self):
        message = config()
        message.ints = []
        result = recording.audit_teb_parameter_updates([message])
        self.assertFalse(result["passed"])
        self.assertIn("max_number_classes", result["missing"])

    def test_present_homotopy_param_is_still_validated(self):
        result = recording.audit_teb_parameter_updates(
            [config(enable_homotopy_class_planning=False)])
        self.assertFalse(result["passed"])
        self.assertIn("homotopy_class_planning_disabled", result["errors"])

    def test_present_and_enabled_is_accepted(self):
        result = recording.audit_teb_parameter_updates(
            [config(enable_homotopy_class_planning=True)])
        self.assertTrue(result["passed"], result["errors"])

    def test_backwards_initialization_still_rejected(self):
        message = config()
        message.bools = [SimpleNamespace(
            name="allow_init_with_backwards_motion", value=True)]
        result = recording.audit_teb_parameter_updates([message])
        self.assertFalse(result["passed"])
        self.assertIn("backwards_initialization_enabled", result["errors"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
