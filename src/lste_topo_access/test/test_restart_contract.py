"""The semantic perception node must survive a crash.

Run 20261102_013932 (T attempt 2) was 11/15 with launch=1: the runner aborted
on child_process_exit because lste_wedetect_det_node segfaulted (exit code -11)
after ~1 h.  Every other audit in that run passed, so a single crash in the
perception sidecar threw away a healthy trial.  roslaunch now respawns it.
"""

from pathlib import Path
import re
import unittest


LAUNCH = (Path(__file__).resolve().parents[3]
          / "scripts/tests/targeted_navigation/portal_retry_single_door_nav.launch")


class PerceptionRestartTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = LAUNCH.read_text(encoding="utf-8")

    def node_block(self, name):
        pattern = re.compile(r"<node\b[^>]*name=\"" + re.escape(name) + r"\"[^>]*>", re.S)
        match = pattern.search(self.source)
        self.assertIsNotNone(match, name)
        return match.group(0)

    def test_wedetect_is_respawned(self):
        block = self.node_block("lste_wedetect_det_node")
        self.assertIn('respawn="true"', block)
        self.assertIn("respawn_delay=", block)

    def test_the_metrics_node_is_not_respawned(self):
        # The recorder must not silently restart and split a run's evidence.
        self.assertNotIn('name="lste_navigation_metrics"', self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
