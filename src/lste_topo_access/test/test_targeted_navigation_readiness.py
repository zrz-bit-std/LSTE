#!/usr/bin/env python3
"""Regression tests for the graph-based targeted-run startup contract."""

import json
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
HELPER = ROOT / "scripts/tests/targeted_navigation/readiness.sh"


class TargetedNavigationReadinessTest(unittest.TestCase):
    def _run_probe(self, nodes, topics, metrics="event=run_start data={}\n", profile="endpoint", map_publisher=True):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metrics_path = root / "metrics.log"
            metrics_path.write_text(metrics, encoding="utf-8")
            node_lines = "\\n".join(nodes)
            topic_lines = "\\n".join(topics)
            publisher = "true" if map_publisher else "false"
            command = """
source {helper}
rosnode() {{ printf '%b\\n' {nodes}; }}
rostopic() {{
  if [[ "$1" == info ]]; then
    printf 'Publishers:\\n'
    if {publisher}; then printf ' * /lste_online_slam (http://localhost:1234/)\\n'; fi
    printf 'Subscribers:\\n'
  else
    printf '%b\\n' {topics}
  fi
}}
readiness_check {profile} {metrics}
printf 'result=%s missing=%s\\n' "$?" "$(readiness_missing_summary)"
""".format(
                helper=shlex.quote(str(HELPER)),
                nodes=shlex.quote(node_lines),
                topics=shlex.quote(topic_lines),
                metrics=shlex.quote(str(metrics_path)),
                profile=shlex.quote(profile),
                publisher=publisher,
            )
            result = subprocess.run(
                ["bash", "-c", command],
                cwd=str(ROOT),
                check=True,
                capture_output=True,
                text=True,
            )
        return result.stdout

    def test_child_crash_is_distinguished_from_clean_spawn_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            launcher = Path(directory) / "launcher.log"
            command = "source %s; readiness_child_process_failure %s" % (
                shlex.quote(str(HELPER)), shlex.quote(str(launcher)),
            )
            for message, status in (
                ("[spawn_pro3-4] process has finished cleanly\n", 1),
                ("[lste_wedetect_det_node-15] process has died [exit code -11]\n", 0),
                ("[lste_global_frontier-8] process has died [exit code 1]\n", 0),
            ):
                launcher.write_text(message, encoding="utf-8")
                result = subprocess.run(["bash", "-c", command], capture_output=True)
                self.assertEqual(result.returncode, status)

    def test_ready_inventory_is_based_on_ros_graph_state(self):
        output = self._run_probe(
            ["/move_base", "/lste_navigation_metrics"],
            [
                "/map",
                "/move_base/global_costmap/costmap",
                "/move_base/local_costmap/costmap",
                "/pro3/wheel_odom",
                "/pro3/rlscan",
            ],
        )
        self.assertIn("result=0 missing=none", output)

    def test_t_junction_requires_live_online_slam_map_publisher(self):
        nodes = ["/move_base", "/lste_global_frontier", "/lste_online_slam", "/lste_navigation_metrics"]
        topics = ["/map", "/move_base/global_costmap/costmap", "/move_base/local_costmap/costmap", "/pro3/wheel_odom", "/pro3/rlscan"]
        failed = self._run_probe(nodes, topics, profile="t_junction", map_publisher=False)
        self.assertIn("map_publisher:/map:/lste_online_slam", failed)
        self.assertIn("result=1", failed)
        ready = self._run_probe(nodes, topics, profile="t_junction", map_publisher=True)
        self.assertIn("result=0 missing=none", ready)

    def test_missing_topic_is_addressable_without_a_buffered_console_line(self):
        output = self._run_probe(
            ["/move_base", "/lste_navigation_metrics"],
            [
                "/map",
                "/move_base/global_costmap/costmap",
                "/move_base/local_costmap/costmap",
                "/pro3/wheel_odom",
            ],
        )
        self.assertIn("topic:/pro3/rlscan", output)
        self.assertIn("result=1", output)

    def test_startup_failure_artifact_contains_inventory_and_log_tails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metrics = root / "metrics.log"
            launcher = root / "launcher.log"
            lifecycle = root / "lifecycle.log"
            artifact = root / "startup_failure.json"
            metrics.write_text("event=run_start data={}\n", encoding="utf-8")
            launcher.write_text("launcher tail\n", encoding="utf-8")
            lifecycle.write_text("lifecycle tail\n", encoding="utf-8")
            command = """
source {helper}
rosnode() {{ printf '/move_base\\n/lste_navigation_metrics\\n'; }}
rostopic() {{ printf '/map\\n/move_base/global_costmap/costmap\\n/move_base/local_costmap/costmap\\n/pro3/wheel_odom\\n'; }}
readiness_check endpoint {metrics} || true
readiness_write_failure_artifact {artifact} endpoint startup_readiness_timeout 20260908_100000 {launcher} {metrics} {lifecycle} '' 12 test=config
""".format(
                helper=shlex.quote(str(HELPER)),
                artifact=shlex.quote(str(artifact)),
                launcher=shlex.quote(str(launcher)),
                metrics=shlex.quote(str(metrics)),
                lifecycle=shlex.quote(str(lifecycle)),
            )
            subprocess.run(
                ["bash", "-c", command],
                cwd=str(ROOT),
                check=True,
                capture_output=True,
                text=True,
            )
            payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["failure_id"], "20260908_100000-S0001")
        self.assertEqual(payload["failure_kind"], "startup_failure")
        self.assertEqual(payload["artifact_path"], str(artifact))
        self.assertIn("/pro3/rlscan", payload["checks"]["missing_topics"])
        self.assertEqual(payload["logs"]["launcher_tail"], ["launcher tail"])
        self.assertEqual(payload["startup_config"], "test=config")

    def test_trial_end_artifact_preserves_active_route_timeout_context(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metrics = root / "metrics.log"
            artifact = root / "trial_end_diagnostic.json"
            metrics.write_text(
                "event=sample data="
                + json.dumps(
                    {
                        "ros_time": 12.0,
                        "pose": [1.0, 2.0, 0.0],
                        "goal": [4.0, 5.0],
                        "cmd": [0.0, 0.0],
                        "teb_bridge_active": True,
                        "teb_status": "trajectory_valid",
                        "navfn_plan": {"poses": 12, "remaining_estimate_m": 3.0},
                        "route_identity": {
                            "active_route_id": 9,
                            "active_route_kind": "portal_transition",
                        },
                    },
                    separators=(",", ":"),
                )
                + "\n",
                encoding="utf-8",
            )
            command = """
source {helper}
readiness_write_trial_end_artifact {artifact} {metrics} 20260908_100001 test=config {root} timeout
""".format(
                helper=shlex.quote(str(HELPER)),
                artifact=shlex.quote(str(artifact)),
                metrics=shlex.quote(str(metrics)),
                root=shlex.quote(str(ROOT)),
            )
            subprocess.run(
                ["bash", "-c", command],
                cwd=str(ROOT),
                check=True,
                capture_output=True,
                text=True,
            )
            payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["artifact_kind"], "trial_end_diagnostic")
        self.assertEqual(payload["outcome"], "timeout")
        self.assertEqual(payload["diagnostic"]["state"], "stall_candidate")
        self.assertEqual(payload["diagnostic"]["active_route_id"], 9)

    def test_trial_end_artifact_keeps_non_timeout_stop_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metrics = root / "metrics.log"
            artifact = root / "trial_end_diagnostic.json"
            metrics.write_text(
                "event=sample data="
                + json.dumps(
                    {
                        "pose": [0.0, 0.0, 0.0],
                        "goal": [1.0, 0.0],
                        "cmd": [0.0, 0.0],
                        "teb_bridge_active": False,
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            command = """
source {helper}
readiness_write_trial_end_artifact {artifact} {metrics} 20260908_100003 test=config {root} launch_process_exit
""".format(
                helper=shlex.quote(str(HELPER)),
                artifact=shlex.quote(str(artifact)),
                metrics=shlex.quote(str(metrics)),
                root=shlex.quote(str(ROOT)),
            )
            subprocess.run(
                ["bash", "-c", command],
                cwd=str(ROOT),
                check=True,
                capture_output=True,
                text=True,
            )
            payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["outcome"], "launch_process_exit")
        self.assertEqual(payload["stop_reason"], "launch_process_exit")


if __name__ == "__main__":
    unittest.main()
