"""Regression tests for completed-place transit planning."""

from pathlib import Path
import json
import re
import subprocess
import sys
import tempfile
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
REPLAY = WORKSPACE_ROOT / "scripts/tests/targeted_navigation/completed_place_transit.py"


class CompletedPlaceTransitReplayTest(unittest.TestCase):
    def run_replay(self, *arguments):
        command = [sys.executable, str(REPLAY), "run"]
        command.extend(arguments)
        return subprocess.run(
            command,
            cwd=str(WORKSPACE_ROOT),
            check=True,
            capture_output=True,
            text=True,
        )

    @staticmethod
    def event_lines(log_file, event):
        token = " event=%s " % event
        return [
            line for line in log_file.read_text(encoding="utf-8").splitlines()
            if token in line
        ]

    def test_cli_replays_complete_place_transit_and_destination_observation(self):
        with tempfile.TemporaryDirectory() as temporary:
            completed = self.run_replay("--log-root", temporary)
            result = json.loads(completed.stdout)
            timestamp = result["run_timestamp"]
            run_directory = Path(result["log_directory"])
            log_file = Path(result["log_file"])
            self.assertRegex(timestamp, r"^\d{8}_\d{6}$")
            self.assertEqual(run_directory.name, timestamp)
            self.assertEqual(run_directory.parent, Path(temporary))
            self.assertEqual(
                log_file.name,
                "%s_completed_place_transit.log" % timestamp,
            )
            self.assertTrue(log_file.is_file())
            self.assertEqual(
                sorted(path.suffix for path in run_directory.iterdir()), [".log"]
            )

            lines = log_file.read_text(encoding="utf-8").splitlines()
            self.assertTrue(lines)
            self.assertTrue(
                all("process=completed_place_transit" in line for line in lines)
            )
            self.assertTrue(all(re.search(r"\[[A-Z]+\]", line) for line in lines))
            self.assertTrue(result["passed"])
            self.assertEqual(result["initial_plan"]["action"], "cross_portal")
            self.assertEqual(result["initial_plan"]["target_place_id"], 3)
            self.assertEqual(result["initial_plan"]["portal_path"], [1, 2])
            self.assertEqual(result["initial_plan"]["first_portal_id"], 1)
            self.assertEqual(result["transit_plan"]["portal_path"], [2])
            self.assertEqual(result["transit_plan"]["first_portal_id"], 2)
            self.assertEqual(result["destination_plan"]["action"], "bootstrap_observation")
            self.assertEqual(result["destination_plan"]["current_place_id"], 3)
            self.assertEqual(result["destination_plan"]["obligation_id"], 31)
            self.assertEqual(result["branch_states"], {"a": "transit", "b": "transit"})
            self.assertEqual(result["pending_work_items"], 0)

            transit_line = self.event_lines(log_file, "planner_replanned")[0]
            self.assertIn('phase="from_completed_place_2"', transit_line)
            self.assertIn('place_state="dormant"', transit_line)
            self.assertIn("place_transit_only=true", transit_line)
            self.assertIn("local_work_items=0", transit_line)
            self.assertIn("portal_1_not_reselected=true", transit_line)
            destination_line = self.event_lines(log_file, "planner_selected")[1]
            self.assertIn('phase="from_unobserved_place_3"', destination_line)
            self.assertIn('action="bootstrap_observation"', destination_line)
            self.assertIn("observation_action=true", destination_line)

    def test_explicit_timestamp_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            timestamp = "20260907_140000"
            self.run_replay(
                "--log-root", temporary, "--timestamp", timestamp,
            )
            command = [
                sys.executable,
                str(REPLAY),
                "run",
                "--log-root",
                temporary,
                "--timestamp",
                timestamp,
            ]
            completed = subprocess.run(
                command,
                cwd=str(WORKSPACE_ROOT),
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("File exists", completed.stderr)

    def test_script_does_not_import_ros(self):
        source = REPLAY.read_text(encoding="utf-8")
        self.assertNotIn("import rospy", source)
        self.assertNotIn("import ros", source)


if __name__ == "__main__":
    unittest.main()
