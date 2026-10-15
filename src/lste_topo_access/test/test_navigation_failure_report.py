#!/usr/bin/env python3
"""Regression tests for the single-run navigation failure report."""

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
TOOLS = ROOT / "scripts/tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import navigation_failure_report as REPORT  # noqa: E402
from navigation_failure_report import build_report, render_markdown  # noqa: E402


def _metrics_line(event, payload, stamp="2026-09-08T12:00:00.000+0800"):
    return (
        "%s level=INFO process=lste_navigation_metrics event=%s data=%s\n"
        % (stamp, event, json.dumps(payload, separators=(",", ":")))
    )


class NavigationFailureReportTest(unittest.TestCase):
    def _write_failure_run(self, root):
        run = root / "20260908_120000"
        run.mkdir()
        metrics = run / (run.name + "_navigation_metrics.log")
        trigger_sample = {
            "pose": [1.0, 2.0, 0.0],
            "goal": [4.0, 2.0],
            "cmd_vel": [0.0, 0.0],
            "navfn_plan": {"poses": 12, "length": 3.0},
            "teb_status": "trajectory_valid",
            "scan": {"min": 2.0, "forward_min": 2.0},
            "move_base_status": "ACTIVE",
            "route_identity": {
                "route_id": 9,
                "route_kind": "frontier_endpoint",
            },
            "goal_transaction_id": 9,
            "cmd_vel_mux": {"output": [0.0, 0.0], "block_reason": "none"},
            "teb_feedback": {"selected_velocity": [0.0, 0.0]},
            "global_costmap": {"free": 10, "occupied": 2, "unknown": 0},
            "local_costmap": {"free": 10, "occupied": 2, "unknown": 0},
            "move_base_feedback": {"goal_id": "goal-9", "status": 1},
            "recovery": None,
            "channel_health": {
                name: {"available": True}
                for name in (
                    "pose", "goal", "cmd_vel", "scan", "navfn_plan",
                    "teb_feedback", "global_costmap", "local_costmap",
                    "move_base_feedback", "route_identity",
                )
            },
        }
        end_sample = dict(trigger_sample)
        artifact = run / (run.name + "_failure_0001.json")
        artifact.write_text(
            json.dumps(
                {
                    "failure_id": run.name + "-F0001",
                    "artifact_path": str(artifact),
                    "run_context": {
                        "experiment": {"world": "small.world"},
                        "resolved_params": {"teb_max_vel_x": 0.5},
                    },
                    "causal_route": {
                        "route_id": 9,
                        "route_kind": "frontier_endpoint",
                    },
                    "diagnosis": {
                        "primary_cause": "controller_stall",
                        "layer": "global_frontier",
                        "explanation": "frontier watchdog invalidated a stalled route",
                        "command_chain": {
                            "teb_planner": [0.0, 0.0],
                            "teb_supervisor": [0.0, 0.0],
                            "mux_output": [0.0, 0.0],
                            "cmd_vel": [0.0, 0.0],
                        },
                        "route_id": 9,
                        "route_kind": "frontier_endpoint",
                        "spatial": {
                            "pose": [1.0, 2.0, 0.0],
                            "goal": [4.0, 2.0],
                        },
                    },
                    "trigger_sample": trigger_sample,
                    "end_sample": end_sample,
                }
            ),
            encoding="utf-8",
        )
        metrics.write_text(
            "".join(
                [
                    _metrics_line(
                        "run_start",
                        {
                            "run_timestamp": run.name,
                            "resolved_params": {"teb_max_vel_x": 0.5},
                        },
                    ),
                    _metrics_line(
                        "failure_started",
                        {
                            "failure_id": run.name + "-F0001",
                            "trigger": "route_invalidated_stall",
                            "source": "global_frontier",
                            "started_wall_elapsed_seconds": 4.0,
                            "classification": {
                                "label": "controller_stall",
                                "confidence": "high",
                            },
                        },
                    ),
                    _metrics_line(
                        "failure_snapshot_ready",
                        {
                            "failure_id": run.name + "-F0001",
                            "classification": "controller_stall",
                            "confidence": "high",
                            "ended_wall_elapsed_seconds": 5.5,
                            "duration_seconds": 1.5,
                            "artifact_path": str(artifact),
                        },
                    ),
                ]
            ),
            encoding="utf-8",
        )
        lifecycle = run / (run.name + "_lifecycle.log")
        lifecycle.write_text(
            "\n".join(
                [
                    "2026-09-08T12:00:00+0800 [INFO] process=runner event=run_start run_timestamp=%s"
                    % run.name,
                    "2026-09-08T12:00:02+0800 [INFO] process=runner event=navigation_ready run_timestamp=%s"
                    % run.name,
                    "2026-09-08T12:00:07+0800 [INFO] process=runner event=failure_stop_requested run_timestamp=%s"
                    % run.name,
                    "2026-09-08T12:00:09+0800 [INFO] process=runner event=run_stop run_timestamp=%s"
                    % run.name,
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        return run

    def test_report_joins_failure_identity_and_phase_timings(self):
        with tempfile.TemporaryDirectory() as directory:
            report = build_report(self._write_failure_run(Path(directory)))

        self.assertEqual(report["outcome"], "failure")
        self.assertEqual(report["failure_summary"]["episode_count"], 1)
        self.assertEqual(
            report["outcome_summary"],
            {
                "kind": "failure",
                "failure_id": "20260908_120000-F0001",
                "cause": "controller_stall",
                "layer": "global_frontier",
                "confidence": "high",
                "route_id": 9,
                "route_kind": "frontier_endpoint",
                "evidence_complete": True,
                "explanation": "frontier watchdog invalidated a stalled route",
            },
        )
        episode = report["failure_summary"]["episodes"][0]
        self.assertEqual(episode["failure_id"], "20260908_120000-F0001")
        self.assertEqual(episode["route_id"], 9)
        self.assertEqual(episode["route_kind"], "frontier_endpoint")
        self.assertTrue(episode["evidence"]["complete"])
        self.assertEqual(report["phase_timings"]["launcher_to_ready_seconds"], 2.0)
        self.assertEqual(report["phase_timings"]["navigation_window_seconds"], 5.0)
        self.assertEqual(report["phase_timings"]["cleanup_seconds"], 2.0)
        self.assertEqual(
            report["parameter_evidence"]["values"]["teb_max_vel_x"],
            0.5,
        )
        markdown = render_markdown(report)
        self.assertIn("boundary explanation", markdown)
        self.assertIn("## Outcome Summary", markdown)
        self.assertIn("20260908_120000-F0001", markdown)
        self.assertIn("trigger pose", markdown)
        self.assertIn("command chain", markdown)

    def test_report_exposes_boundary_and_lifecycle_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self._write_failure_run(Path(directory))
            report = build_report(run)

        evidence = report["failure_summary"]["episodes"][0]["evidence"]
        self.assertTrue(evidence["complete"])
        self.assertEqual(evidence["unavailable_channels"], [])
        self.assertTrue(evidence["contract"]["failure_id_valid"])
        self.assertTrue(evidence["contract"]["artifact_exists"])
        self.assertTrue(evidence["contract"]["artifact_in_run_directory"])
        self.assertEqual(
            evidence["event_contract"]["missing"],
            [],
        )
        self.assertEqual(
            evidence["event_contract"]["counts"]["failure_snapshot_ready"],
            1,
        )

    def test_report_excludes_report_json_from_snapshot_loader(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self._write_failure_run(Path(directory))
            report_path = run / (run.name + "_failure_report.json")
            report_path.write_text(
                json.dumps({"report_kind": "navigation_failure_report"}),
                encoding="utf-8",
            )
            snapshots = REPORT.load_failure_snapshots(
                run / (run.name + "_navigation_metrics.log")
            )

        self.assertEqual(list(snapshots), [run.name + "-F0001"])

    def test_report_does_not_hide_missing_planner_channel(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self._write_failure_run(Path(directory))
            artifact = run / (run.name + "_failure_0001.json")
            payload = json.loads(artifact.read_text(encoding="utf-8"))
            payload["trigger_sample"].pop("teb_feedback")
            payload["trigger_sample"]["channel_health"]["teb_feedback"][
                "available"
            ] = False
            artifact.write_text(json.dumps(payload), encoding="utf-8")
            report = build_report(run)

        evidence = report["failure_summary"]["episodes"][0]["evidence"]
        self.assertFalse(evidence["complete"])
        self.assertIn("trigger_sample.teb_feedback", evidence["missing"])
        self.assertIn("teb_feedback", evidence["unavailable_channels"])

    def test_pre_dispatch_planner_rejection_is_complete_without_controller_channels(self):
        """No move_base/TEB feedback is expected before Navfn dispatch."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "20260908_120010"
            run.mkdir()
            artifact = run / (run.name + "_failure_0001.json")
            artifact.write_text("{}\n", encoding="utf-8")
            healthy = {
                "pose": [19.8, 13.8, 1.57],
                "goal": [19.2, 14.0],
                "cmd_vel": [0.0, 0.0],
                "navfn_plan": {"poses": 8, "length": 0.6},
                "teb_status": "trajectory_valid",
                "scan": {"min": 1.0, "forward_min": 2.0},
                "move_base_status": "ACTIVE",
                "cmd_vel_mux": {"output": [0.0, 0.0]},
                "teb_feedback": {"selected_velocity": [0.0, 0.0]},
                "global_costmap": {"free": 10, "occupied": 2, "unknown": 0},
                "local_costmap": {"free": 10, "occupied": 2, "unknown": 0},
                "move_base_feedback": {"goal_id": "goal-1", "status": 1},
                "recovery": None,
                "channel_health": {
                    name: {"available": True}
                    for name in (
                        "pose", "goal", "cmd_vel", "scan", "navfn_plan",
                        "teb_feedback", "global_costmap", "local_costmap",
                        "move_base_feedback", "route_identity",
                    )
                },
            }
            trigger = dict(healthy)
            trigger.update(
                {
                    "navfn_plan": {"poses": 0, "length": 0.0},
                    "move_base_feedback": None,
                    "teb_feedback": None,
                    "route_identity": {},
                    "failure_boundary": {
                        "reason": "navfn_empty_plan",
                        "details": {"reason": "navfn_empty_plan"},
                    },
                    "channel_health": dict(healthy["channel_health"]),
                }
            )
            for name in ("goal", "teb_feedback", "move_base_feedback", "route_identity"):
                trigger["channel_health"][name] = {"available": False}
            snapshot = {
                "failure_id": run.name + "-F0001",
                "artifact_path": str(artifact),
                "run_context": {
                    "experiment": {"world": "small.world"},
                    "resolved_params": {"teb_max_vel_x": 0.5},
                },
                "causal_route": {"route_kind": "target_approach"},
                "classification": {"label": "planner_no_path"},
                "diagnosis": {"primary_cause": "planner_no_path"},
                "trigger": "target_route_failed",
                "trigger_sample": trigger,
                "end_sample": healthy,
            }
            result = REPORT._evidence_completeness(
                snapshot, run_dir=run, run_timestamp=run.name
            )

        self.assertTrue(result["complete"], result)
        self.assertEqual(
            result["contract"]["execution_boundary"],
            "pre_dispatch_planner_rejection",
        )
        self.assertEqual(result["unavailable_channels"], [])

    def test_report_is_readable_without_metrics_for_startup_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "20260908_120001"
            run.mkdir()
            startup = run / (run.name + "_startup_failure.json")
            startup.write_text(
                json.dumps(
                    {
                        "failure_id": run.name + "-S0001",
                        "reason": "navigation_startup_readiness_timeout",
                        "artifact_path": str(startup),
                    }
                ),
                encoding="utf-8",
            )
            report = build_report(run)

        self.assertEqual(report["outcome"], "startup_failure")
        self.assertEqual(
            report["startup_failure"]["failure_id"],
            "20260908_120001-S0001",
        )
        self.assertIn("Startup Failure", render_markdown(report))

    def test_large_metrics_log_uses_bounded_head_and_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "20260908_120002"
            run.mkdir()
            metrics = run / (run.name + "_navigation_metrics.log")
            metrics.write_text(
                _metrics_line("run_start", {"run_timestamp": run.name})
                + ("x" * 256)
                + "\n"
                + _metrics_line(
                    "sample",
                    {
                        "pose": [0.0, 0.0, 0.0],
                        "goal": [1.0, 0.0],
                        "distance_to_goal": 1.0,
                    },
                ),
                encoding="utf-8",
            )
            original_limit = REPORT._METRICS_READ_LIMIT_BYTES
            original_head = REPORT._METRICS_HEAD_BYTES
            REPORT._METRICS_READ_LIMIT_BYTES = 256
            REPORT._METRICS_HEAD_BYTES = 256
            try:
                report = build_report(run)
            finally:
                REPORT._METRICS_READ_LIMIT_BYTES = original_limit
                REPORT._METRICS_HEAD_BYTES = original_head

        self.assertTrue(report["metrics_read_bounded"])
        self.assertGreaterEqual(report["metrics_records_loaded"], 1)

    def test_failure_artifact_survives_bounded_metrics_window(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self._write_failure_run(Path(directory))
            original_limit = REPORT._METRICS_READ_LIMIT_BYTES
            REPORT._METRICS_READ_LIMIT_BYTES = 1
            try:
                report = build_report(run)
            finally:
                REPORT._METRICS_READ_LIMIT_BYTES = original_limit

        self.assertTrue(report["metrics_read_bounded"])
        self.assertEqual(report["failure_summary"]["episode_count"], 1)
        self.assertEqual(
            report["failure_summary"]["episodes"][0]["failure_id"],
            "20260908_120000-F0001",
        )

    def test_report_exposes_trial_boundary_identity_and_progress(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "20260908_120003"
            run.mkdir()
            metrics = run / (run.name + "_navigation_metrics.log")
            # One successful sub-route must not turn a later trial timeout
            # into a run-level success.
            metrics.write_text(
                _metrics_line(
                    "move_base_status",
                    {
                        "status": 3,
                        "status_name": "SUCCEEDED",
                        "goal_id": "route-1",
                        "ros_time": 4.0,
                    },
                ),
                encoding="utf-8",
            )
            artifact = run / (run.name + "_trial_end_diagnostic.json")
            artifact.write_text(
                json.dumps(
                    {
                        "artifact_kind": "trial_end_diagnostic",
                        "termination_id": run.name + "-T0001",
                        "classification": "active_route_timeout",
                        "outcome": "timeout",
                        "diagnostic": {
                            "termination_id": run.name + "-T0001",
                            "classification": "active_route_timeout",
                            "state": "in_progress",
                            "reason": "active_route_at_trial_end",
                            "active_route_id": 7,
                            "active_route_kind": "portal_transition",
                            "latest_sample": {
                                "pose": [1.0, 2.0, 0.0],
                                "goal": [3.0, 2.0],
                                "distance_to_goal": 2.0,
                            },
                            "recent_samples": [{"sample": 1}],
                            "progress": {
                                "distance_to_goal_m": {
                                    "first": 3.0,
                                    "last": 2.0,
                                    "progress_toward_goal": 1.0,
                                },
                                "pose_displacement_m": 1.2,
                                "route_transition_count": 2,
                                "interpretation": "moving_toward_goal",
                            },
                            "evidence": {"complete": True},
                        },
                    }
                ),
                encoding="utf-8",
            )
            report = build_report(run)
            markdown = render_markdown(report)

        self.assertEqual(report["outcome"], "trial_end_diagnostic")
        self.assertTrue(report["success_evidence"]["observed"])
        trial_end = report["trial_end_diagnostic"]
        self.assertEqual(trial_end["termination_id"], "20260908_120003-T0001")
        self.assertEqual(trial_end["classification"], "active_route_timeout")
        self.assertEqual(
            trial_end["progress"]["distance_to_goal_m"]["progress_toward_goal"],
            1.0,
        )
        self.assertIn("20260908_120003-T0001", markdown)

    def test_action_success_is_run_success_without_trial_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "20260908_120004"
            run.mkdir()
            metrics = run / (run.name + "_navigation_metrics.log")
            metrics.write_text(
                _metrics_line(
                    "move_base_status",
                    {
                        "status": 3,
                        "status_name": "SUCCEEDED",
                        "goal_id": "endpoint-1",
                        "ros_time": 16.0,
                    },
                ),
                encoding="utf-8",
            )
            report = build_report(run)

        self.assertEqual(report["outcome"], "success")
        self.assertEqual(report["success_evidence"]["goal_id"], "endpoint-1")


if __name__ == "__main__":
    unittest.main()
