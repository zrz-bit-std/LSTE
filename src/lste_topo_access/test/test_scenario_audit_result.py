"""The scenario evaluator must not turn a killed run into a pass.

Run 20261031_225814 (L) was terminated by the 180 s stall watchdog. Its summary
line was ``semantic_l_junction launch_failed 143`` -- no key=value tokens. The
first evaluator parsed that into an empty dict, found no failing key, and
reported PASS, so the driver advanced to the next scenario and recorded a false
success.

The evaluator now requires the full expected status set and rejects
``launch_failed`` explicitly.
"""

from pathlib import Path
import sys
import tempfile
import unittest


SUITE = Path("/home/zrz/work/lste_ws/scripts/tests/targeted_navigation")
if str(SUITE) not in sys.path:
    sys.path.insert(0, str(SUITE))

import scenario_audit_result as evaluator  # noqa: E402


CASE = "semantic_l_junction"

PASSING = (
    "semantic_l_junction launch=0 recording=0 tree=0 auth=0 auth_all=0 "
    "turn_raw=0 lifecycle=0 tsp=0 dfs=0 geometry=0 trajectory=0 native=0 "
    "tsp_visual=0 dfs_visual=0 visual=0 run=/tmp/run\n"
)

KILLED = "semantic_l_junction launch_failed 143\n"


def write(text):
    handle = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False)
    handle.write(text)
    handle.close()
    return handle.name


class EvaluatorTest(unittest.TestCase):
    def test_all_zero_passes(self):
        self.assertEqual(evaluator.evaluate(write(PASSING), CASE), "PASS")

    def test_killed_run_is_not_a_pass(self):
        result = evaluator.evaluate(write(KILLED), CASE)
        self.assertTrue(result.startswith("FAIL"), result)
        self.assertIn("launch_failed", result)

    def test_single_failing_key_is_reported(self):
        line = PASSING.replace("native=0", "native=1")
        self.assertEqual(evaluator.evaluate(write(line), CASE), "FAIL native")

    def test_missing_status_fields_fail(self):
        line = "semantic_l_junction launch=0 recording=0 tree=0 auth=0\n"
        result = evaluator.evaluate(write(line), CASE)
        self.assertIn("missing_status_fields", result)
        self.assertIn("native", result)

    def test_absent_line_fails(self):
        self.assertEqual(evaluator.evaluate(write(""), CASE),
                         "FAIL no_summary_line")

    def test_unreadable_summary_fails(self):
        self.assertEqual(evaluator.evaluate("/nonexistent/summary.log", CASE),
                         "FAIL summary_unreadable")

    def test_sealed_killed_run_is_rejected(self):
        sealed = Path("/home/zrz/work/lste_ws/runtime/gp_branch_development/"
                      "place_dfs_suite_20261031_225814/summary.log")
        if not sealed.is_file():
            self.skipTest("sealed killed run not present")
        result = evaluator.evaluate(str(sealed), CASE)
        self.assertTrue(result.startswith("FAIL"), result)


if __name__ == "__main__":
    unittest.main(verbosity=2)
