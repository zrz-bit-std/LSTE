"""Regression tests for the ROS-free local failure-evidence replay."""

import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
REPLAY = (
    WORKSPACE_ROOT
    / "scripts/tests/targeted_navigation/replay_failure_evidence.py"
)
FIXTURE = (
    WORKSPACE_ROOT
    / "scripts/tests/targeted_navigation/fixtures/t_junction_controller_stall.json"
)


class FailureEvidenceReplayTest(unittest.TestCase):
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

    def test_fixture_replays_in_process_without_ros_or_gazebo(self):
        with tempfile.TemporaryDirectory() as temporary:
            completed = self.run_replay("--log-root", temporary)
            result = json.loads(completed.stdout)
            self.assertTrue(result["passed"])
            self.assertEqual(result["scenario_id"], "t_junction_controller_stall")
            self.assertEqual(result["source_failure_id"], "captured_t_junction-F0001")
            self.assertEqual(result["trigger_diagnosis"]["primary_cause"], "controller_stall")
            self.assertEqual(result["trigger_diagnosis"]["layer"], "global_frontier")
            self.assertEqual(result["trigger_diagnosis"]["route_id"], 9)
            self.assertEqual(
                result["trigger_diagnosis"]["route_kind"], "frontier_endpoint"
            )
            self.assertEqual(len(result["phases"]), 3)

            log_path = Path(result["log_file"])
            self.assertRegex(
                result["run_timestamp"], r"^\d{8}_\d{6}$"
            )
            self.assertTrue(log_path.is_file())
            self.assertEqual(
                log_path.name,
                "%s_failure_evidence_replay.log" % result["run_timestamp"],
            )
            lines = log_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(
                [
                    line.split(" event=", 1)[1].split(" ", 1)[0]
                    for line in lines
                ],
                [
                    "run_start",
                    "sample_diagnosed",
                    "sample_diagnosed",
                    "sample_diagnosed",
                    "failure_snapshot_replayed",
                    "run_complete",
                ],
            )
            self.assertTrue(all("process=failure_evidence_replay" in line for line in lines))

    def test_explicit_timestamp_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            timestamp = "20260908_100002"
            self.run_replay("--log-root", temporary, "--timestamp", timestamp)
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

    def test_fixture_identity_and_ros_independence_are_explicit(self):
        payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
        self.assertEqual(payload["schema_version"], 1)
        source = REPLAY.read_text(encoding="utf-8")
        self.assertNotIn("import rospy", source)
        self.assertNotIn("import ros", source)
        self.assertIn("gazebo_enabled=False", source)
        self.assertTrue(re.search(r"t_junction_controller_stall", source))


if __name__ == "__main__":
    unittest.main()
