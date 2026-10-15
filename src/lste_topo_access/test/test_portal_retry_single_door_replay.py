"""Regression tests for the ROS-free portal retry replay."""

from pathlib import Path
import json
import re
import subprocess
import sys
import tempfile
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
REPLAY = WORKSPACE_ROOT / "scripts/tests/targeted_navigation/portal_retry_single_door.py"


class PortalRetrySingleDoorReplayTest(unittest.TestCase):
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

    def test_cli_creates_timestamped_log_and_expected_event_trace(self):
        with tempfile.TemporaryDirectory() as temporary:
            completed = self.run_replay("--log-root", temporary)
            result = json.loads(completed.stdout)
            timestamp = result["run_timestamp"]
            self.assertRegex(timestamp, r"^\d{8}_\d{6}$")
            run_directory = Path(result["log_directory"])
            self.assertEqual(run_directory.name, timestamp)
            self.assertEqual(run_directory.parent, Path(temporary))
            log_file = Path(result["log_file"])
            self.assertEqual(log_file.name, "%s_portal_retry_single_door.log" % timestamp)
            self.assertTrue(log_file.is_file())
            self.assertEqual(sorted(path.suffix for path in run_directory.iterdir()), [".log"])

            lines = log_file.read_text(encoding="utf-8").splitlines()
            self.assertTrue(lines)
            self.assertTrue(all("process=portal_retry_single_door" in line for line in lines))
            self.assertTrue(all(re.search(r"\[[A-Z]+\]", line) for line in lines))
            events = [
                line.split(" event=", 1)[1].split(" ", 1)[0]
                for line in lines
                if " event=" in line
            ]
            expected = [
                "run_start",
                "source_evidence",
                "route_dispatched",
                "route_failed",
                "route_retry",
                "destination_evidence",
                "crossing_accepted",
                "run_complete",
            ]
            self.assertEqual(events, expected)
            self.assertTrue(result["passed"])
            self.assertEqual(result["attempts"], 2)

            retry_line = next(line for line in lines if " event=route_retry " in line)
            self.assertIn('projection_changed=true', retry_line)
            self.assertIn('same_work_item=true', retry_line)
            self.assertIn('transaction_route_id=102', retry_line)
            self.assertIn('source_side_proven=true', retry_line)
            source_line = next(line for line in lines if " event=source_evidence " in line)
            self.assertIn('portal_transaction_state="source_probe"', source_line)
            destination_line = next(
                line for line in lines if " event=destination_evidence " in line
            )
            self.assertIn('portal_transaction_state="place_commit"', destination_line)
            crossing_line = next(line for line in lines if " event=crossing_accepted " in line)
            self.assertIn('state="transit"', crossing_line)
            self.assertIn('work_item_closed=true', crossing_line)
            self.assertIn('portal_transaction_state="place_commit"', crossing_line)

    def test_explicit_timestamp_is_reproducible_and_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            timestamp = "20260907_120000"
            first = self.run_replay(
                "--log-root", temporary, "--timestamp", timestamp,
            )
            self.assertEqual(json.loads(first.stdout)["run_timestamp"], timestamp)
            command = [
                sys.executable,
                str(REPLAY),
                "run",
                "--log-root",
                temporary,
                "--timestamp",
                timestamp,
            ]
            second = subprocess.run(
                command,
                cwd=str(WORKSPACE_ROOT),
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(second.returncode, 0)
            self.assertIn("File exists", second.stderr)

    def test_script_does_not_import_ros(self):
        source = REPLAY.read_text(encoding="utf-8")
        self.assertNotIn("import rospy", source)
        self.assertNotIn("import ros", source)


if __name__ == "__main__":
    unittest.main()
