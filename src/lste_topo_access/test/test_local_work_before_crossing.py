"""Regression tests for local-work-before-crossing replay."""

from pathlib import Path
import json
import re
import subprocess
import sys
import tempfile
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
REPLAY = WORKSPACE_ROOT / "scripts/tests/targeted_navigation/local_work_before_crossing.py"


class LocalWorkBeforeCrossingTest(unittest.TestCase):
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
            line
            for line in log_file.read_text(encoding="utf-8").splitlines()
            if token in line
        ]

    def test_cli_enforces_local_work_then_probe_then_crossing(self):
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
                "%s_local_work_before_crossing.log" % timestamp,
            )
            self.assertTrue(log_file.is_file())
            self.assertEqual(
                sorted(path.suffix for path in run_directory.iterdir()), [".log"]
            )
            lines = log_file.read_text(encoding="utf-8").splitlines()
            self.assertTrue(lines)
            self.assertTrue(
                all("process=local_work_before_crossing" in line for line in lines)
            )
            self.assertTrue(all(re.search(r"\[[A-Z]+\]", line) for line in lines))
            self.assertTrue(result["passed"])
            self.assertEqual(result["crossing_plan"]["action"], "cross_portal")
            self.assertEqual(result["crossing_plan"]["first_portal_id"], 7)
            self.assertEqual(result["transaction_state"], "place_commit")

            events = [
                line.split(" event=", 1)[1].split(" ", 1)[0]
                for line in lines
                if " event=" in line
            ]
            required = [
                "local_work_pending_before_crossing",
                "local_work_resolved",
                "portal_probe",
                "portal_probe",
                "destination_evidence",
                "crossing_selected",
                "crossing_accepted",
            ]
            cursor = 0
            for expected in required:
                cursor = events.index(expected, cursor) + 1
            pending_line = self.event_lines(
                log_file, "local_work_pending_before_crossing"
            )[0]
            self.assertIn('planner_action="observe_local_work"', pending_line)
            self.assertIn("portal_crossing_allowed=false", pending_line)
            source_line, destination_line = self.event_lines(log_file, "portal_probe")
            self.assertIn('phase="source"', source_line)
            self.assertIn('phase="destination"', destination_line)
            crossing_line = self.event_lines(log_file, "crossing_accepted")[0]
            self.assertIn('transaction_state="place_commit"', crossing_line)
            self.assertIn("local_work_resolved=true", crossing_line)

    def test_explicit_timestamp_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            timestamp = "20260907_150000"
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
