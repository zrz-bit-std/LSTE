#!/usr/bin/env python3
"""Regression coverage for the reproducible office comparison matrix."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import json


ROOT = Path(__file__).resolve().parents[3]
BENCHMARK = ROOT / "scripts/tests/office_building"
if str(BENCHMARK) not in sys.path:
    sys.path.insert(0, str(BENCHMARK))

from experiment_protocol import build_trials, load_matrix
from run_experiment_suite import (
    build_trial_end_diagnostic,
    latest_failure_snapshot,
    metrics_path_for_run,
    startup_readiness_snapshot,
    write_trial_end_diagnostic_artifact,
    trial_environment,
    wait_for_startup_readiness,
    write_startup_failure_artifact,
    write_navigation_failure_report,
)
from summarize_run import load_trial_end_diagnostic
from experiment_video import recorder_duration_seconds


class OfficeExperimentProtocolTest(unittest.TestCase):
    def test_video_duration_accepts_float_diagnostic_timeout(self):
        self.assertEqual(recorder_duration_seconds(35.0), 35)
        self.assertEqual(recorder_duration_seconds(35.01), 36)

    def test_topology_smoke_has_one_comparable_trial_per_method(self):
        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trials = build_trials(matrix, "topology_smoke")
        self.assertEqual(len(trials), 6)
        self.assertEqual({trial.level for trial in trials}, {"level_2"})
        self.assertEqual({trial.trial_id for trial in trials}, {1})
        self.assertEqual({trial.seed for trial in trials}, {20260905})
        self.assertEqual({trial.startup_timeout_seconds for trial in trials}, {180})
        self.assertEqual(
            {trial.startup_readiness_timeout_seconds for trial in trials}, {45}
        )
        self.assertEqual({trial.terminal_event for trial in trials}, {"frontier_exhausted"})
        self.assertEqual({trial.task_id for trial in trials}, {"topology_only"})
        self.assertTrue(all(trial.video_required for trial in trials))
        self.assertEqual({trial.video_window for trial in trials}, {"Gazebo"})
        self.assertEqual({trial.video_fps for trial in trials}, {20})
        self.assertEqual(
            {trial.method for trial in trials},
            {
                "frontier",
                "frontier_distance_dedup",
                "place_portal",
                "place_portal_workitem_legacy_rank",
                "place_portal_workitem",
                "place_portal_workitem_strict",
            },
        )

    def test_main_phase_expands_to_independent_process_restarts(self):
        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trials = build_trials(matrix, "topology_main")
        self.assertEqual(len(trials), 72)
        self.assertEqual({trial.trial_id for trial in trials}, {1, 2, 3})
        self.assertEqual({trial.seed for trial in trials}, {20260905, 20260906, 20260907})
        self.assertEqual({trial.startup_timeout_seconds for trial in trials}, {180})
        self.assertEqual(
            {trial.startup_readiness_timeout_seconds for trial in trials}, {45}
        )
        self.assertEqual(
            {trial.level for trial in trials},
            {"level_1", "level_2", "level_3", "level_4"},
        )

    def test_trial_environment_overrides_inherited_controller(self):
        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]

        environment = trial_environment(
            trial,
            {"LSTE_CONTROLLER": "sappo", "UNRELATED": "preserved"},
        )

        self.assertEqual(environment["LSTE_CONTROLLER"], trial.controller)
        self.assertEqual(environment["UNRELATED"], "preserved")

    def test_isolation_readiness_accepts_pose_without_frontier(self):
        """Target-entry startup must not wait for an intentionally absent node."""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text(
                "event=run_start data={\"experiment\":"
                "{\"global_frontier_enabled\":false}}\n"
                "event=sample data="
                + json.dumps(
                    {
                        "ros_time": 3.0,
                        "pose": [19.8, 13.8, 1.57],
                        "cmd": [0.0, 0.0],
                    },
                    separators=(",", ":"),
                )
                + "\n",
                encoding="utf-8",
            )
            snapshot = startup_readiness_snapshot(
                path,
                Path(directory) / "missing_global_frontier.log",
                require_frontier=False,
            )

        self.assertTrue(snapshot["ready"])
        self.assertEqual(
            snapshot["reason"], "navigation_readiness_without_frontier"
        )
        self.assertFalse(snapshot["route_seen"])

    def test_diagnostic_failure_windows_are_forwarded_without_config_mutation(self):
        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]

        environment = trial_environment(
            trial,
            {"LSTE_CONTROLLER": "sappo"},
            failure_post_window=1.5,
            failure_pre_window=4.0,
        )

        self.assertEqual(environment["LSTE_CONTROLLER"], trial.controller)
        self.assertEqual(environment["NAVIGATION_FAILURE_EVIDENCE_POST_WINDOW"], "1.5")
        self.assertEqual(environment["NAVIGATION_FAILURE_EVIDENCE_PRE_WINDOW"], "4.0")

    def test_no_video_is_an_explicit_diagnostic_trial_override(self):
        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]
        overridden = run_experiment_suite.apply_diagnostic_video_override(
            [trial], disable_video=True
        )[0]

        self.assertTrue(trial.video_required)
        self.assertFalse(overridden.video_required)
        self.assertEqual(
            run_experiment_suite.trial_as_dict(overridden)["video_override"],
            "disabled_by_cli",
        )

    def test_profile_override_is_recorded_on_selected_trial(self):
        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]
        overridden = run_experiment_suite.replace(trial, profile="office_entry")
        self.assertEqual(overridden.profile, "office_entry")
        self.assertEqual(trial.profile, "primary")

    def test_metrics_path_is_bound_to_the_run_directory_name(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260906_120000"
            run_dir.mkdir()
            unrelated = run_dir / "older_navigation_metrics.log"
            unrelated.write_text("old\n", encoding="utf-8")
            self.assertIsNone(metrics_path_for_run(run_dir))

            expected = run_dir / "20260906_120000_navigation_metrics.log"
            expected.write_text("new\n", encoding="utf-8")
            self.assertEqual(metrics_path_for_run(run_dir), expected)

    def test_finished_run_gets_single_failure_report_without_subprocess(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050005"
            run_dir.mkdir()
            (run_dir / (run_dir.name + "_navigation_metrics.log")).write_text(
                "", encoding="utf-8"
            )
            result = write_navigation_failure_report(run_dir)

            self.assertEqual(result["status"], "written")
            self.assertTrue(Path(result["json_path"]).is_file())
            self.assertTrue(Path(result["markdown_path"]).is_file())
            self.assertIn(
                "failure_report_ready",
                (run_dir / (run_dir.name + "_lifecycle.log")).read_text(
                    encoding="utf-8"
                ),
            )

    def test_startup_timeout_is_forwarded_and_classified(self):
        """A wedged launcher is bounded and recorded as infrastructure failure."""
        from unittest import mock

        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]

        def fake_start(_trial, _environment):
            raise subprocess.TimeoutExpired(
                [str(run_experiment_suite.RUNNER), "start", trial.level],
                trial.startup_timeout_seconds,
            )

        def fake_run(command, **kwargs):
            return subprocess.CompletedProcess(command, 0)

        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(
                run_experiment_suite, "CURRENT", Path(directory) / "current"
            ), mock.patch.object(
                run_experiment_suite, "_run_benchmark_start", side_effect=fake_start
            ), mock.patch.object(
                run_experiment_suite.subprocess, "run", side_effect=fake_run
            ) as run:
                record = run_experiment_suite.run_trial(trial)

        self.assertEqual(record["infrastructure_failure"], "benchmark_start_timeout")
        self.assertEqual(run_experiment_suite.build_trials(
            load_matrix(BENCHMARK / "experiment_matrix.yaml"), "topology_smoke"
        )[0].startup_timeout_seconds, 180)
        self.assertEqual(
            run.call_args_list[-1].kwargs["env"]["OFFICE_BUILDING_STOP_REASON"],
            "benchmark_start_timeout",
        )

    def test_nonzero_stop_exit_is_recorded_as_cleanup_failure(self):
        """A failed stop command must not look like a clean experiment close."""
        from unittest import mock

        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current = root / "current"
            run_dir = root / "20260908_050006"

            def fake_start(_trial, _environment):
                run_dir.mkdir()
                current.symlink_to(run_dir, target_is_directory=True)
                raise subprocess.TimeoutExpired(
                    "start", trial.startup_timeout_seconds
                )

            with mock.patch.object(
                run_experiment_suite, "CURRENT", current
            ), mock.patch.object(
                run_experiment_suite,
                "_run_benchmark_start",
                side_effect=fake_start,
            ), mock.patch.object(
                run_experiment_suite,
                "_run_benchmark_stop",
                return_value=17,
            ):
                record = run_experiment_suite.run_trial(trial)

        self.assertEqual(record["cleanup_return_code"], 17)
        self.assertEqual(
            record["cleanup_failure"],
            "benchmark_stop_failed:exit_17",
        )
        self.assertEqual(record["infrastructure_failure"], "benchmark_start_timeout")

    def test_startup_timeout_kills_the_launcher_process_group(self):
        """Timeout cleanup cannot leave run_all_tmux behind as a child."""
        from unittest import mock

        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]

        class FakeProcess:
            pid = 4321

            def __init__(self):
                self.wait_calls = 0
                self.kill_called = False

            def wait(self, timeout=None):
                self.wait_calls += 1
                if self.wait_calls == 1:
                    raise subprocess.TimeoutExpired("start", timeout)
                return -15

            def kill(self):
                self.kill_called = True

        process = FakeProcess()
        with mock.patch.object(
            run_experiment_suite.subprocess, "Popen", return_value=process
        ) as popen, mock.patch.object(run_experiment_suite.os, "killpg") as killpg:
            with self.assertRaises(subprocess.TimeoutExpired):
                run_experiment_suite._run_benchmark_start(trial, {})

        self.assertEqual(popen.call_args.kwargs["start_new_session"], True)
        self.assertEqual(killpg.call_args.args, (4321, run_experiment_suite.signal.SIGTERM))
        self.assertFalse(process.kill_called)

    def test_launcher_failure_with_a_run_directory_keeps_startup_artifact(self):
        from unittest import mock

        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current = root / "current"
            run_dir = root / "20260908_050003"

            def fake_start(_trial, _environment):
                run_dir.mkdir()
                current.symlink_to(run_dir, target_is_directory=True)
                raise subprocess.TimeoutExpired("start", trial.startup_timeout_seconds)

            with mock.patch.object(
                run_experiment_suite, "CURRENT", current
            ), mock.patch.object(
                run_experiment_suite, "_run_benchmark_start", side_effect=fake_start
            ), mock.patch.object(
                run_experiment_suite.subprocess, "run",
                return_value=subprocess.CompletedProcess(["stop"], 0),
            ):
                record = run_experiment_suite.run_trial(trial)

            artifact = run_dir / (run_dir.name + "_startup_failure.json")
            artifact_exists = artifact.is_file()

        self.assertEqual(record["outcome"], "startup_failure")
        self.assertEqual(record["infrastructure_failure"], "benchmark_start_timeout")
        self.assertEqual(record["startup_failure_id"], "20260908_050003-S0001")
        self.assertTrue(artifact_exists)

    def test_failure_snapshot_tail_is_a_valid_diagnostic_stop_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text(
                "2026-09-08T00:00:00 level=ERROR process=metrics "
                "event=failure_snapshot_ready data=%s\n"
                % json.dumps(
                    {
                        "failure_id": "run-F0001",
                        "classification": "planner_materialization_stall",
                    },
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            self.assertEqual(
                latest_failure_snapshot(path)["failure_id"],
                "run-F0001",
            )

    def test_runner_stops_on_first_failure_and_preserves_route_goal_evidence(self):
        """A Level 4-style failure ends the trial before its long timeout."""
        from unittest import mock

        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = next(
            trial
            for trial in build_trials(matrix, "topology_main")
            if trial.level == "level_4"
            and trial.method == "place_portal_workitem"
        )
        trial = run_experiment_suite.replace(trial, video_required=False)
        failure_id = "20260908_120001-F0001"
        failure_artifact = {
            "schema_version": 1,
            "failure_id": failure_id,
            "artifact_path": "placeholder",
            "source": "global_frontier",
            "trigger": "frontier_route_stall",
            "classification": {
                "label": "controller_stall",
                "confidence": "high",
            },
            "diagnosis": {
                "primary_cause": "controller_stall",
                "layer": "global_frontier",
                "route_id": 31,
                "route_kind": "frontier_endpoint",
            },
            "causal_route": {
                "route_id": 31,
                "route_kind": "frontier_endpoint",
            },
            "trigger_sample": {
                "pose": [4.2, 5.1],
                "goal": [4.8, 5.3],
                "cmd_vel": [0.0, 0.0],
                "navfn_plan": {"poses": 12, "length": 0.8},
                "teb_status": "trajectory_valid",
                "move_base_status": "ACTIVE",
                "route_identity": {
                    "active_route_id": 31,
                    "active_route_kind": "frontier_endpoint",
                },
            },
            "end_sample": {
                "pose": [4.2, 5.1],
                "goal": [4.8, 5.3],
                "cmd_vel": [0.0, 0.0],
                "navfn_plan": {"poses": 12, "length": 0.8},
                "teb_status": "trajectory_valid",
                "move_base_status": "ACTIVE",
                "route_identity": {
                    "active_route_id": 31,
                    "active_route_kind": "frontier_endpoint",
                },
            },
        }
        failure_summary = {
            "failure_id": failure_id,
            "classification": "controller_stall",
            "diagnosis": {
                "primary_cause": "controller_stall",
                "layer": "global_frontier",
            },
            "route_id": 31,
            "route_kind": "frontier_endpoint",
            "trigger_pose": [4.2, 5.1],
            "trigger_goal": [4.8, 5.3],
            "command_chain": {
                "teb_planner": [0.0, 0.0],
                "mux_output": [0.0, 0.0],
                "cmd_vel": [0.0, 0.0],
            },
            "artifact_path": "placeholder",
            "started_wall_elapsed_seconds": 12.0,
            "ended_wall_elapsed_seconds": 17.0,
            "duration_seconds": 5.0,
        }

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current = root / "current"
            run_dir = root / "20260908_120001"
            metrics_name = run_dir.name + "_navigation_metrics.log"
            lifecycle_name = run_dir.name + "_lifecycle.log"

            def fake_start(_trial, _environment):
                run_dir.mkdir()
                current.symlink_to(run_dir, target_is_directory=True)
                (run_dir / lifecycle_name).write_text(
                    "2026-09-08T12:00:01+0800 [INFO] "
                    "process=office_building_launcher event=run_start "
                    "run_timestamp=20260908_120001\n",
                    encoding="utf-8",
                )
                metrics = run_dir / metrics_name
                lines = [
                    "event=run_start data=%s\n"
                    % json.dumps({"ros_time": 1.0}),
                    "event=global_frontier_event data=%s\n"
                    % json.dumps(
                        {
                            "frontier_event": "navigation_readiness",
                            "ready": True,
                            "state": "ready",
                            "route_id": 0,
                        }
                    ),
                    "event=sample data=%s\n"
                    % json.dumps(
                        {
                            "ros_time": 12.0,
                            "pose": [4.2, 5.1, 0.0],
                            "goal": [4.8, 5.3],
                            "cmd": [0.0, 0.0],
                            "teb_bridge_active": True,
                            "teb_status": "trajectory_valid",
                            "navfn_plan": {"poses": 12, "length": 0.8},
                            "move_base_feedback": {
                                "status_name": "ACTIVE",
                                "goal_id": "goal-31",
                            },
                            "route_identity": {
                                "active_route_id": 31,
                                "active_route_kind": "frontier_endpoint",
                            },
                            "task_done": False,
                        }
                    ),
                    "event=failure_snapshot_ready data=%s\n"
                    % json.dumps(failure_summary),
                ]
                metrics.write_text("".join(lines), encoding="utf-8")
                artifact = run_dir / (run_dir.name + "_failure_0001.json")
                failure_artifact["artifact_path"] = str(artifact)
                artifact.write_text(
                    json.dumps(failure_artifact), encoding="utf-8"
                )

            stop_reasons = []

            def fake_stop(environment, timeout_seconds=30.0):
                stop_reasons.append(environment.get("OFFICE_BUILDING_STOP_REASON"))
                return 0

            with mock.patch.object(
                run_experiment_suite, "CURRENT", current
            ), mock.patch.object(
                run_experiment_suite, "_run_benchmark_start", side_effect=fake_start
            ), mock.patch.object(
                run_experiment_suite, "_run_benchmark_stop", side_effect=fake_stop
            ), mock.patch.object(
                run_experiment_suite.subprocess,
                "run",
                return_value=subprocess.CompletedProcess(["postprocess"], 0),
            ):
                record = run_experiment_suite.run_trial(
                    trial, stop_on_failure=True
                )

            report = json.loads(
                (run_dir / (run_dir.name + "_failure_report.json")).read_text(
                    encoding="utf-8"
                )
            )

        self.assertEqual(record["outcome"], "failure_snapshot")
        self.assertEqual(record["failure_stop_id"], failure_id)
        self.assertEqual(record["failure_stop_route_id"], 31)
        self.assertEqual(record["failure_stop_route_kind"], "frontier_endpoint")
        self.assertEqual(record["failure_stop_trigger_pose"], [4.2, 5.1])
        self.assertEqual(record["failure_stop_trigger_goal"], [4.8, 5.3])
        self.assertTrue(record["failure_report_json"])
        self.assertIn("failure_snapshot:20260908_120001-F0001", stop_reasons)
        self.assertEqual(report["outcome"], "failure")
        self.assertEqual(report["failure_summary"]["episodes"][0]["route_id"], 31)

    def test_failure_artifact_is_loaded_before_legacy_evidence_log(self):
        import sys

        tools = ROOT / "scripts/tools"
        if str(tools) not in sys.path:
            sys.path.insert(0, str(tools))
        import analyze_navigation_metrics

        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050004"
            run_dir.mkdir()
            metrics = run_dir / (run_dir.name + "_navigation_metrics.log")
            metrics.write_text("", encoding="utf-8")
            artifact = run_dir / (run_dir.name + "_failure_0001.json")
            artifact.write_text(
                json.dumps(
                    {
                        "failure_id": "20260908_050004-F0001",
                        "artifact_path": str(artifact),
                        "run_context": {"experiment": {"world": "small.world"}},
                    }
                ),
                encoding="utf-8",
            )
            evidence = run_dir / (run_dir.name + "_failure_evidence.log")
            evidence.write_text(
                "2026-09-08T00:00:00 level=ERROR process=metrics "
                "event=failure_snapshot data=%s\n"
                % json.dumps(
                    {
                        "failure_id": "20260908_050004-F0001",
                        "artifact_path": None,
                    }
                ),
                encoding="utf-8",
            )

            snapshots = analyze_navigation_metrics.load_failure_snapshots(metrics)

        self.assertEqual(
            snapshots["20260908_050004-F0001"]["artifact_path"],
            str(artifact),
        )
        self.assertEqual(
            snapshots["20260908_050004-F0001"]["run_context"]["experiment"]["world"],
            "small.world",
        )

    def test_operator_interrupt_is_recorded_and_cleanup_is_called(self):
        from unittest import mock

        import run_experiment_suite

        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]

        with mock.patch.object(
            run_experiment_suite, "_run_benchmark_start",
            side_effect=KeyboardInterrupt,
        ), mock.patch.object(
            run_experiment_suite.subprocess,
            "run",
            return_value=subprocess.CompletedProcess(["stop"], 0),
        ) as run:
            record = run_experiment_suite.run_trial(trial)

        self.assertEqual(record["outcome"], "interrupted")
        self.assertTrue(record["interrupted"])
        self.assertEqual(record["infrastructure_failure"], "operator_interrupt")
        self.assertEqual(
            run.call_args_list[-1].kwargs["env"]["OFFICE_BUILDING_STOP_REASON"],
            "operator_interrupt",
        )

    def test_navfn_empty_probe_is_a_startup_failure_not_a_navigation_trial(self):
        """A persistent stream waiting forever for Navfn is classified early."""
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050000"
            run_dir.mkdir()
            metrics = run_dir / (run_dir.name + "_navigation_metrics.log")
            metrics.write_text(
                "2026-09-08T00:00:00 level=INFO process=metrics "
                "event=global_frontier_event data=%s\n"
                % json.dumps(
                    {
                        "frontier_event": "navigation_readiness",
                        "ready": False,
                        "state": "navfn_probe_empty_after_warmup",
                        "route_id": 0,
                        "ros_time": 18.0,
                    },
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            frontier = run_dir / (run_dir.name + "_global_frontier.log")
            frontier.write_text(
                "2026-09-08T00:00:00 [INFO] process=global_frontier "
                "event=stdout line=Global frontier startup gate: "
                "navfn_probe_empty_after_warmup\n",
                encoding="utf-8",
            )

            snapshot = startup_readiness_snapshot(metrics, frontier)

        self.assertFalse(snapshot["ready"])
        self.assertFalse(snapshot["route_seen"])
        self.assertEqual(snapshot["reason"], "navfn_startup_probe_not_ready")
        self.assertEqual(
            snapshot["readiness_last_state"], "navfn_probe_empty_after_warmup"
        )
        self.assertEqual(
            snapshot["global_frontier_last_gate_state"],
            "navfn_probe_empty_after_warmup",
        )

    def test_startup_snapshot_keeps_last_pose_and_command_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050000"
            run_dir.mkdir()
            metrics = run_dir / (run_dir.name + "_navigation_metrics.log")
            metrics.write_text(
                "event=global_frontier_event data=%s\n"
                "event=sample data=%s\n"
                "event=teb_bridge_event data=%s\n"
                % (
                    json.dumps(
                        {
                            "frontier_event": "navigation_readiness",
                            "ready": False,
                            "state": "navfn_probe_empty_after_warmup",
                        },
                        separators=(",", ":"),
                    ),
                    json.dumps(
                        {
                            "pose": [1.0, 2.0, 0.5],
                            "pose_frame": "odom",
                            "goal": [3.0, 4.0],
                            "goal_frame": "map",
                            "cmd_vel": [0.0, 0.0],
                            "teb_status": "not_available",
                            "global_costmap": {"free": 12},
                            "state": "0:-",
                        },
                        separators=(",", ":"),
                    ),
                    json.dumps(
                        {
                            "bridge_event": "bridge_startup",
                            "active": False,
                            "active_route_id": 0,
                        },
                        separators=(",", ":"),
                    ),
                ),
                encoding="utf-8",
            )

            snapshot = startup_readiness_snapshot(metrics)

        self.assertEqual(snapshot["latest_sample"]["pose"], [1.0, 2.0, 0.5])
        self.assertEqual(snapshot["latest_sample"]["global_costmap"], {"free": 12})
        self.assertEqual(snapshot["latest_bridge_event"]["event"], "teb_bridge_event")
        self.assertEqual(snapshot["metrics_event_counts"]["sample"], 1)

    def test_startup_readiness_accepts_ready_or_first_real_route(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050001"
            run_dir.mkdir()
            metrics = run_dir / (run_dir.name + "_navigation_metrics.log")
            ready_payload = {
                "frontier_event": "navigation_readiness",
                "ready": True,
                "state": "ready",
                "route_id": 0,
            }
            metrics.write_text(
                "event=global_frontier_event data=%s\n"
                % json.dumps(ready_payload, separators=(",", ":")),
                encoding="utf-8",
            )
            snapshot = startup_readiness_snapshot(metrics)
            self.assertTrue(snapshot["ready"])
            self.assertEqual(snapshot["reason"], "navigation_readiness_announced")

            metrics.write_text(
                "event=global_frontier_event data=%s\n"
                % json.dumps(
                    {
                        "frontier_event": "route_selected",
                        "route_id": 7,
                    },
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            snapshot = startup_readiness_snapshot(metrics)

        self.assertTrue(snapshot["ready"])
        self.assertTrue(snapshot["route_seen"])
        self.assertEqual(snapshot["reason"], "route_observed_without_readiness_event")
        self.assertEqual(snapshot["route_last_id"], 7)

    def test_persistent_stream_gate_line_is_a_readiness_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050001"
            run_dir.mkdir()
            frontier = run_dir / (run_dir.name + "_global_frontier.log")
            frontier.write_text(
                "2026-09-08T00:00:00 [INFO] process=global_frontier "
                "event=stdout line=Global frontier startup gate: ready for "
                "persistent_stream from map/TF/costmap; defer Navfn proof to first mission\n",
                encoding="utf-8",
            )

            snapshot = startup_readiness_snapshot(None, frontier)

        self.assertTrue(snapshot["ready"])
        self.assertTrue(snapshot["global_frontier_gate_passed"])

    def test_startup_failure_artifact_points_to_logs_and_lifecycle_event(self):
        matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
        trial = build_trials(matrix, "topology_smoke")[0]
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050002"
            run_dir.mkdir()
            lifecycle = run_dir / (run_dir.name + "_lifecycle.log")
            lifecycle.write_text("run start\n", encoding="utf-8")
            result = write_startup_failure_artifact(
                run_dir,
                trial,
                {
                    "status": "timeout",
                    "ready": False,
                    "route_seen": False,
                    "failure_reason": "navfn_startup_probe_not_ready",
                },
            )
            artifact = Path(result["artifact_path"])
            event_log = Path(result["log_path"])
            payload = json.loads(artifact.read_text(encoding="utf-8"))

            self.assertEqual(payload["failure_kind"], "startup_failure")
            self.assertEqual(payload["failure_id"], "20260908_050002-S0001")
            self.assertEqual(payload["reason"], "navfn_startup_probe_not_ready")
            self.assertTrue(event_log.is_file())
            self.assertIn("event=startup_failure", event_log.read_text())
            self.assertIn("event=startup_failure", lifecycle.read_text())

    def test_startup_readiness_wait_with_zero_bound_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                wait_for_startup_readiness(
                    Path(directory), None, timeout_seconds=0.0
                )

    def test_trial_end_diagnostic_preserves_active_route_stall_context(self):
        """A timeout keeps the final route/plan/command boundary addressable."""
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050005"
            run_dir.mkdir()
            metrics = run_dir / (run_dir.name + "_navigation_metrics.log")
            metrics.write_text(
                "event=global_frontier_event data=%s\n"
                "event=sample data=%s\n"
                % (
                    json.dumps(
                        {
                            "frontier_event": "route_command",
                            "route_id": 12,
                            "route_kind": "portal_transition",
                            "goal": [4.0, 5.0],
                        },
                        separators=(",", ":"),
                    ),
                    json.dumps(
                        {
                            "ros_time": 31.2,
                            "sample": 99,
                            "pose": [1.0, 2.0, 0.5],
                            "pose_frame": "odom",
                            "goal": [4.0, 5.0],
                            "goal_frame": "map",
                            "distance_to_goal": 3.6,
                            "cmd": [0.0, 0.0],
                            "teb_status": "trajectory_valid",
                            "teb_bridge_active": True,
                            "move_base_feedback": {
                                "status_name": "ACTIVE",
                            },
                            "navfn_plan": {
                                "poses": 20,
                                "remaining_estimate_m": 3.6,
                            },
                            "route_identity": {
                                "active_route_id": 12,
                                "active_route_kind": "portal_transition",
                            },
                            "task_done": False,
                        },
                        separators=(",", ":"),
                    ),
                ),
                encoding="utf-8",
            )

            diagnostic = build_trial_end_diagnostic(metrics)
            self.assertEqual(diagnostic["state"], "stall_candidate")
            self.assertEqual(
                diagnostic["reason"], "active_route_without_effective_command"
            )
            self.assertEqual(diagnostic["active_route_id"], 12)
            self.assertEqual(
                diagnostic["active_route_kind"], "portal_transition"
            )
            self.assertEqual(diagnostic["latest_sample"]["navfn_plan"]["poses"], 20)

            matrix = load_matrix(BENCHMARK / "experiment_matrix.yaml")
            trial = build_trials(matrix, "topology_smoke")[0]
            artifact_path = write_trial_end_diagnostic_artifact(
                run_dir, trial, diagnostic, metrics_path=metrics
            )
            artifact = Path(artifact_path)
            payload = json.loads(artifact.read_text(encoding="utf-8"))

        self.assertEqual(payload["artifact_kind"], "trial_end_diagnostic")
        self.assertEqual(payload["outcome"], "timeout")
        self.assertEqual(
            payload["diagnostic"]["reason"],
            "active_route_without_effective_command",
        )
        self.assertEqual(payload["diagnostic"]["latest_sample"]["sample"], 99)

    def test_trial_end_diagnostic_keeps_progress_window_and_identity(self):
        """A timeout is independently localizable without replaying ROS."""
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050008"
            run_dir.mkdir()
            metrics = run_dir / (run_dir.name + "_navigation_metrics.log")
            samples = []
            for index, (distance, route_id, x) in enumerate(
                ((3.0, 12, 1.0), (2.2, 12, 1.5), (2.0, 13, 1.9)),
                start=1,
            ):
                samples.append(
                    "event=sample data=%s\n"
                    % json.dumps(
                        {
                            "ros_time": float(index),
                            "wall_elapsed_seconds": float(index),
                            "sample": index,
                            "pose": [x, 2.0, 0.0],
                            "goal": [4.0, 2.0],
                            "distance_to_goal": distance,
                            "cmd": [0.2, 0.0],
                            "teb_bridge_active": True,
                            "teb_status": "trajectory_valid",
                            "move_base_feedback": {"status_name": "ACTIVE"},
                            "navfn_plan": {"poses": 10},
                            "route_identity": {
                                "active_route_id": route_id,
                                "active_route_kind": "portal_transition",
                            },
                            "task_done": False,
                        },
                        separators=(",", ":"),
                    )
                )
            metrics.write_text("".join(samples), encoding="utf-8")

            diagnostic = build_trial_end_diagnostic(metrics)

        self.assertEqual(
            diagnostic["termination_id"], "20260908_050008-T0001"
        )
        self.assertEqual(diagnostic["classification"], "active_route_timeout")
        self.assertEqual(len(diagnostic["recent_samples"]), 3)
        self.assertEqual(
            diagnostic["progress"]["distance_to_goal_m"]["progress_toward_goal"],
            1.0,
        )
        self.assertEqual(diagnostic["progress"]["route_transition_count"], 2)
        self.assertEqual(
            diagnostic["progress"]["interpretation"], "moving_toward_goal"
        )
        self.assertEqual(
            diagnostic["progress"]["active_route"]["route_id"], 13
        )
        self.assertEqual(
            diagnostic["progress"]["active_route"]["interpretation"],
            "insufficient_progress_evidence",
        )
        self.assertTrue(diagnostic["evidence"]["complete"])

    def test_trial_end_diagnostic_does_not_call_idle_timeout_a_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text(
                "event=sample data=%s\n"
                % json.dumps(
                    {
                        "ros_time": 4.0,
                        "cmd": [0.0, 0.0],
                        "teb_bridge_active": False,
                        "task_done": False,
                    },
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            diagnostic = build_trial_end_diagnostic(path)

        self.assertEqual(diagnostic["state"], "idle")
        self.assertEqual(diagnostic["reason"], "no_active_controller_route")

    def test_trial_end_diagnostic_distinguishes_terminal_settle_from_stall(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text(
                "event=sample data=%s\n"
                % json.dumps(
                    {
                        "ros_time": 8.0,
                        "distance_to_goal": 0.32,
                        "teb_xy_goal_tolerance": 0.50,
                        "cmd": [0.0, 0.0],
                        "teb_bridge_active": True,
                        "teb_status": "trajectory_valid",
                        "navfn_plan": {"poses": 6},
                        "move_base_feedback": {"status_name": "ACTIVE"},
                    },
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            diagnostic = build_trial_end_diagnostic(path)

        self.assertEqual(diagnostic["state"], "terminal_settle")
        self.assertEqual(
            diagnostic["reason"], "within_goal_tolerance_waiting_for_terminal"
        )
        self.assertTrue(diagnostic["within_goal_tolerance"])

    def test_trial_end_diagnostic_recovers_goal_tolerance_from_teb_log(self):
        """Late move_base parameters are available before timeout reduction."""
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050007"
            run_dir.mkdir()
            path = run_dir / (run_dir.name + "_navigation_metrics.log")
            path.write_text(
                "event=sample data=%s\n"
                % json.dumps(
                    {
                        "ros_time": 8.0,
                        "distance_to_goal": 0.32,
                        "cmd": [0.0, 0.0],
                        "teb_bridge_active": True,
                        "teb_status": "trajectory_valid",
                        "navfn_plan": {"poses": 6},
                        "move_base_feedback": {"status_name": "ACTIVE"},
                    },
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            (run_dir / (run_dir.name + "_teb_navigation.log")).write_text(
                "event=stdout line= * /move_base/TebLocalPlannerROS/xy_goal_tolerance: 0.50\n",
                encoding="utf-8",
            )
            diagnostic = build_trial_end_diagnostic(path)

        self.assertEqual(diagnostic["goal_tolerance_m"], 0.50)
        self.assertEqual(diagnostic["state"], "terminal_settle")
        self.assertTrue(diagnostic["within_goal_tolerance"])

    def test_trial_end_diagnostic_exposes_forward_only_command_gap(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text(
                "event=sample data=%s\n"
                % json.dumps(
                    {
                        "ros_time": 8.0,
                        "distance_to_goal": 0.70,
                        "teb_xy_goal_tolerance": 0.50,
                        "cmd": [0.0, 0.18],
                        "teb_planner_cmd": [-0.01, 0.18],
                        "cmd_vel_mux": {
                            "filter_reason": "forward_only_reverse_clamp",
                            "output": {"linear_x": 0.0, "angular_z": 0.18},
                        },
                        "teb_bridge_active": True,
                        "teb_status": "trajectory_valid",
                        "navfn_plan": {"poses": 6},
                        "teb_turn_supervisor": {"state": "PASS_THROUGH"},
                        "move_base_feedback": {"status_name": "ACTIVE"},
                    },
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            diagnostic = build_trial_end_diagnostic(path)

        self.assertEqual(diagnostic["state"], "controller_output_gap_candidate")
        self.assertEqual(
            diagnostic["reason"], "forward_only_reverse_clamp_at_trial_end"
        )
        self.assertTrue(diagnostic["controller_output_gap"]["last_gap"])

    def test_summary_exposes_trial_end_artifact_without_reclassifying_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "20260908_050006"
            run_dir.mkdir()
            metrics = run_dir / (run_dir.name + "_navigation_metrics.log")
            metrics.write_text("", encoding="utf-8")
            artifact = run_dir / (run_dir.name + "_trial_end_diagnostic.json")
            artifact.write_text(
                json.dumps(
                    {
                        "artifact_kind": "trial_end_diagnostic",
                        "created_at": "2026-09-08T05:00:06+0800",
                        "outcome": "timeout",
                        "diagnostic": {
                            "state": "stall_candidate",
                            "reason": "active_route_without_effective_command",
                            "active_route_id": 4,
                            "active_route_kind": "frontier_endpoint",
                            "latest_sample": {"sample": 10},
                            "recent_events": [],
                        },
                    }
                ),
                encoding="utf-8",
            )
            result = load_trial_end_diagnostic(metrics)

        self.assertEqual(result["artifact_kind"], "trial_end_diagnostic")
        self.assertEqual(result["outcome"], "timeout")
        self.assertEqual(result["state"], "stall_candidate")
        self.assertEqual(result["active_route_id"], 4)
        self.assertEqual(result["latest_sample"]["sample"], 10)


if __name__ == "__main__":
    unittest.main()
