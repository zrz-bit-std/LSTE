#!/usr/bin/env python3
"""Keep the business-node launcher small and responsibility-oriented."""

from pathlib import Path
import unittest


LIFECYCLE = Path(__file__).resolve().parents[1]
NODES = LIFECYCLE / "nodes"


class NodeLauncherLayoutTest(unittest.TestCase):
    def test_public_launcher_only_orchestrates(self):
        source = (LIFECYCLE / "run_nodes_tmux.sh").read_text(encoding="utf-8")
        for relative_path in (
            "config/mission.sh",
            "config/exploration.sh",
            "config/detector.sh",
            "config/controllers.sh",
            "session.sh",
            "windows/mission_perception.sh",
            "windows/exploration.sh",
            "windows/teb_navigation.sh",
            "windows/observability.sh",
        ):
            self.assertIn('source "$NODES_DIR/%s"' % relative_path, source)
        self.assertLessEqual(len(source.splitlines()), 120)

    def test_each_lifecycle_module_stays_focused(self):
        expected_modules = (
            "config/mission.sh",
            "config/exploration.sh",
            "config/detector.sh",
            "config/controllers.sh",
            "session.sh",
            "windows/mission_perception.sh",
            "windows/exploration.sh",
            "windows/teb_navigation.sh",
            "windows/observability.sh",
        )
        for relative_path in expected_modules:
            path = NODES / relative_path
            self.assertTrue(path.is_file(), relative_path)
            self.assertLessEqual(
                len(path.read_text(encoding="utf-8").splitlines()),
                160,
                relative_path,
            )


if __name__ == "__main__":
    unittest.main()
