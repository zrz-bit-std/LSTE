"""Regression tests for the ROS-free two-door identity replay."""

from pathlib import Path
import json
import re
import subprocess
import sys
import tempfile
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
REPLAY = WORKSPACE_ROOT / "scripts/tests/targeted_navigation/portal_identity_two_door.py"


class PortalIdentityTwoDoorReplayTest(unittest.TestCase):
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
    def event_names(log_file):
        return [
            line.split(" event=", 1)[1].split(" ", 1)[0]
            for line in log_file.read_text(encoding="utf-8").splitlines()
            if " event=" in line
        ]

    def test_cli_replays_two_door_identity_trace(self):
        with tempfile.TemporaryDirectory() as temporary:
            completed = self.run_replay("--log-root", temporary)
            result = json.loads(completed.stdout)
            timestamp = result["run_timestamp"]
            self.assertRegex(timestamp, r"^\d{8}_\d{6}$")
            run_directory = Path(result["log_directory"])
            log_file = Path(result["log_file"])
            self.assertEqual(run_directory.name, timestamp)
            self.assertEqual(run_directory.parent, Path(temporary))
            self.assertEqual(
                log_file.name,
                "%s_portal_identity_two_door.log" % timestamp,
            )
            self.assertTrue(log_file.is_file())
            self.assertEqual(
                sorted(path.suffix for path in run_directory.iterdir()), [".log"]
            )

            lines = log_file.read_text(encoding="utf-8").splitlines()
            self.assertTrue(lines)
            self.assertTrue(
                all("process=portal_identity_two_door" in line for line in lines)
            )
            self.assertTrue(all(re.search(r"\[[A-Z]+\]", line) for line in lines))
            events = self.event_names(log_file)
            self.assertEqual(events[0], "run_start")
            self.assertEqual(events[-1], "run_complete")
            required = [
                "source_evidence",
                "transaction_started",
                "route_failed",
                "route_retry",
                "destination_evidence",
                "crossing_accepted",
            ]
            cursor = 0
            for required_event in required:
                cursor = events.index(required_event, cursor) + 1
            self.assertGreaterEqual(events.count("map_update"), 6)

            self.assertTrue(result["passed"])
            self.assertEqual(result["branch_ids"], {"a": "1:portal-a", "b": "1:portal-b"})
            self.assertNotEqual(result["work_item_ids"]["a"], result["work_item_ids"]["b"])
            self.assertEqual(result["transaction_portals"], {"a": 501, "b": 502})
            self.assertEqual(result["transaction_states"]["a"], "place_commit")
            self.assertEqual(result["transaction_states"]["b"], "source_probe")

            retry_line = next(line for line in lines if " event=route_retry " in line)
            self.assertIn('branch_id="1:portal-a"', retry_line)
            self.assertIn("retry_count=1", retry_line)
            self.assertIn("same_transaction=true", retry_line)
            crossing_line = next(
                line for line in lines if " event=crossing_accepted " in line
            )
            self.assertIn('state="transit"', crossing_line)
            self.assertIn("work_item_closed=true", crossing_line)
            self.assertIn("door_b_unchanged=true", crossing_line)

    def test_explicit_timestamp_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            timestamp = "20260907_130000"
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
