#!/usr/bin/env python3
"""Run a fixed number of audited yellow-cup search attempts.

Unlike repeat_local_search.py this driver does not stop at N successes: it
runs exactly N attempts, enforces a per-attempt total wall-clock limit, records
/lste/state, and writes an independent audit record per attempt.
"""
import argparse
import datetime as dt
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import yaml

WS = Path(__file__).resolve().parents[2]
TMUX_WINDOWS = ["goal", "global_frontier", "teb_nav", "cmd_vel_mux", "metrics"]


def save(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def locked_state(records):
    """Return the first perception LOCKED event in one metrics batch.

    The campaign contract is stricter than the search node: the perception
    state machine reaching LOCKED is task success, so the attempt ends without
    waiting for the follow/done dwell window.
    """
    for event, data in records:
        if event != "state_change":
            continue
        try:
            if int(data.get("state", -1)) == 2:
                return data
        except (TypeError, ValueError):
            continue
    return None


def total_limit_exceeded(elapsed, limit):
    """Return whether the per-attempt wall limit expired.

    A non-positive limit means the campaign runs without a total time cap.
    """
    try:
        limit = float(limit)
    except (TypeError, ValueError):
        return False
    try:
        elapsed = float(elapsed)
    except (TypeError, ValueError):
        return False
    return limit > 0.0 and elapsed > limit


class StationaryWatch:
    def __init__(self, seconds, radius):
        self.seconds, self.radius = seconds, radius
        self.anchor, self.since = None, None

    def observe(self, pose, now):
        if self.anchor is None or math.hypot(pose[0] - self.anchor[0],
                                            pose[1] - self.anchor[1]) > self.radius:
            self.anchor, self.since = list(pose), now

    def age(self, now):
        return 0.0 if self.since is None else now - self.since

    def stalled(self, now):
        return self.since is not None and self.age(now) > self.seconds


def lifecycle(script, environment, output, wait=True):
    command = ["bash", "-lc", "source scripts/config/pipeline_env.sh; exec " + script]
    process = subprocess.Popen(command, cwd=str(WS), env=environment,
                               stdout=output, stderr=subprocess.STDOUT)
    if wait:
        return process.wait(timeout=180)
    return process


def capture_windows(directory):
    for window in TMUX_WINDOWS:
        with (directory / (directory.name + "_" + window + "_tail.log")).open("w") as stream:
            subprocess.run(["tmux", "capture-pane", "-pJ", "-t", "lste:" + window,
                            "-S", "-400"], stdout=stream, stderr=subprocess.STDOUT,
                           timeout=15, check=False)


def read_new_records(path, offset):
    """Read complete appended records from a byte offset.

    Binary offsets are deliberate: a text-mode ``tell()`` cookie is not a
    stable position while another process is appending large JSON lines.  A
    partial trailing line is returned un-consumed and retried next cycle.
    """
    records = []
    if not path.exists():
        return records, offset
    with path.open("rb") as stream:
        stream.seek(offset)
        while True:
            start = stream.tell()
            raw = stream.readline()
            if not raw or not raw.endswith(b"\n"):
                return records, start
            line = raw.decode("utf-8", "replace")
            if " event=" not in line or " data=" not in line:
                continue
            event = line.split(" event=", 1)[1].split(" data=", 1)[0]
            try:
                data = json.loads(line.split(" data=", 1)[1])
            except Exception:
                continue
            try:
                data["_wall"] = dt.datetime.fromisoformat(
                    line.split(" level=", 1)[0]).timestamp()
            except Exception:
                data["_wall"] = time.time()
            records.append((event, data))
    return records, offset


def recover_latest_sample(path, tail_bytes=4_000_000):
    """Recover the newest complete sample from the file tail.

    Used only when the incremental reader has made no progress for several
    seconds while the metrics file is still growing.  This prevents a
    partial-line/offset problem from becoming a false stationary failure.
    """
    try:
        size = path.stat().st_size
        with path.open("rb") as stream:
            stream.seek(max(0, size - int(tail_bytes)))
            tail = stream.read()
    except (OSError, ValueError):
        return None
    for raw in reversed(tail.split(b"\n")):
        if b" event=sample" not in raw or b" data=" not in raw:
            continue
        line = raw.decode("utf-8", "replace")
        try:
            data = json.loads(line.split(" data=", 1)[1])
        except Exception:
            continue
        if not data.get("pose"):
            continue
        try:
            data["_wall"] = dt.datetime.fromisoformat(
                line.split(" level=", 1)[0]).timestamp()
        except Exception:
            data["_wall"] = time.time()
        return data
    return None


def start_recorder(directory, run, environment):
    path = directory / (run + "_state.jsonl")
    pose_path = directory / (run + "_pose.jsonl")
    command = ["bash", "-lc",
               "source scripts/config/pipeline_env.sh; exec python3 "
               "scripts/tools/record_search_events.py --out %s --pose-out %s"
               % (str(path), str(pose_path))]
    process = subprocess.Popen(command, cwd=str(WS), env=environment,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return process, path, pose_path


def read_latest_json_line(path, kind):
    """Return the newest complete record of one kind from a small JSONL file."""
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
    for line in reversed(lines):
        try:
            payload = json.loads(line)
        except Exception:
            continue
        if payload.get("kind") == kind:
            return payload
    return None


def run_attempt(campaign, number, args, config):
    run = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = WS / "runtime/navigation/logs" / run
    directory.mkdir(parents=True, exist_ok=True)
    shutil.copy2(config, directory / "pipeline_config.yaml")
    metrics = directory / (run + "_navigation_metrics.log")
    environment = dict(os.environ, LSTE_WS=str(WS), LSTE_NO_ATTACH="1",
                       NAVIGATION_RUN_DIRECTORY=str(directory),
                       PIPELINE_CONFIG=str(config))
    started = time.monotonic()
    result = {
        "run": run, "attempt": number, "status": "running",
        "started_at": dt.datetime.now().isoformat(),
        "source_log": str(metrics), "world": str(args.world),
        "stall_seconds": args.stall_seconds, "max_total_seconds": args.max_total_seconds,
        "stationary_radius_m": args.radius,
    }
    save(directory / "attempt_result.json", result)
    save(campaign / "current_attempt.json", result)

    recorder = None
    output = None
    try:
        with (directory / (run + "_lifecycle.log")).open("w", buffering=1) as output:
            lifecycle("./scripts/lifecycle/stop_all_tmux.sh", environment, output)
            launcher = lifecycle("./scripts/lifecycle/run_all_tmux.sh", environment, output, False)
            # Start the /lste/state recorder only after the new roscore and the
            # state node are up; starting it before the restart loses the
            # registration when the previous master dies.
            for _ in range(120):
                probe = subprocess.run(
                    ["bash", "-lc",
                     "source scripts/config/pipeline_env.sh; rosnode list 2>/dev/null | grep -qx /lste_state_node"],
                    cwd=str(WS), env=environment, capture_output=True, timeout=15)
                if probe.returncode == 0:
                    break
                time.sleep(2)
            recorder, state_path, pose_path = start_recorder(
                directory, run, environment
            )
            result["state_recording"] = str(state_path)
            result["pose_recording"] = str(pose_path)
            watch = StationaryWatch(args.stall_seconds, args.radius)
            offset = 0
            latest = {}
            pose_latest = {}
            exploration_started = None
            last_report = 0.0
            last_parsed = time.monotonic()
            reader_stale_seconds = 15.0
            while True:
                now = time.monotonic()
                pose_record = read_latest_json_line(pose_path, "pose")
                if pose_record is not None:
                    previous_pose_ros = pose_latest.get("ros")
                    pose_ros = pose_record.get("ros")
                    if (
                        previous_pose_ros is None
                        or pose_ros is None
                        or float(pose_ros) >= float(previous_pose_ros)
                    ):
                        pose_latest = pose_record
                        observed = now - max(
                            0.0, time.time() - pose_record.get("wall", 0.0)
                        )
                        watch.observe(
                            [pose_record["x"], pose_record["y"],
                             pose_record.get("theta", 0.0)],
                            observed,
                        )
                state_record = read_latest_json_line(state_path, "state")
                if (
                    state_record is not None
                    and int(state_record.get("state", -1)) == 2
                    and result["status"] != "success"
                ):
                    result.update(status="success", reason="target_locked",
                                  completion_event=state_record,
                                  locked_event=state_record,
                                  locked_source="state_recorder")
                records, offset = read_new_records(metrics, offset)
                if records:
                    last_parsed = now
                # The incremental reader is useful for event ownership but a
                # single unread sample line must never drive the stationary
                # watchdog.  Reconcile the watchdog against the newest
                # complete sample in the file tail on every cycle.
                tail_sample = recover_latest_sample(metrics, tail_bytes=262144)
                if tail_sample is not None and tail_sample.get("pose"):
                    previous_ros = latest.get("ros_time")
                    tail_ros = tail_sample.get("ros_time")
                    if (
                        previous_ros is None
                        or tail_ros is None
                        or float(tail_ros) >= float(previous_ros)
                    ):
                        latest = tail_sample
                        observed = now - max(
                            0.0, time.time() - tail_sample.get("_wall", 0.0)
                        )
                        watch.observe(tail_sample["pose"], observed)
                for event, data in records:
                    if event == "run_start":
                        exploration_started = data.get("_wall")
                    if event == "sample" and data.get("pose"):
                        previous_ros = latest.get("ros_time")
                        sample_ros = data.get("ros_time")
                        if (
                            previous_ros is None
                            or sample_ros is None
                            or float(sample_ros) >= float(previous_ros)
                        ):
                            if exploration_started is None:
                                exploration_started = data.get("_wall")
                            latest = data
                            observed = now - max(
                                0.0, time.time() - data["_wall"]
                            )
                            watch.observe(data["pose"], observed)
                    if event == "task_done" and data.get("value") is True:
                        result.update(status="success", completion_event=data)
                locked = locked_state(records)
                if locked is not None and result["status"] != "success":
                    # Perception LOCKED is the campaign success criterion.
                    result.update(status="success", reason="target_locked",
                                  completion_event=locked, locked_event=locked)
                elapsed_total = time.monotonic() - started
                if result["status"] == "success":
                    break
                if total_limit_exceeded(elapsed_total, args.max_total_seconds):
                    result.update(status="failed", reason="total_over_%g_wall_seconds" % args.max_total_seconds,
                                  stop_pose=latest.get("pose"), last_sample=latest)
                    break
                if watch.stalled(now):
                    # A non-sample line can keep the incremental reader
                    # advancing while every sample after the last pose is
                    # missed.  Always reconcile against the newest complete
                    # sample in the file tail before declaring a failure.
                    recovered = recover_latest_sample(metrics)
                    previous_ros = latest.get("ros_time")
                    recovered_ros = (
                        None if recovered is None else recovered.get("ros_time")
                    )
                    newer_sample = (
                        recovered is not None
                        and recovered.get("pose")
                        and (
                            previous_ros is None
                            or recovered_ros is None
                            or float(recovered_ros) > float(previous_ros)
                        )
                    )
                    if newer_sample:
                        latest = recovered
                        observed = now - max(
                            0.0, time.time() - recovered.get("_wall", 0.0)
                        )
                        watch.observe(recovered["pose"], observed)
                        try:
                            offset = metrics.stat().st_size
                        except OSError:
                            pass
                        last_parsed = now
                        result["metrics_reader_recoveries"] = int(
                            result.get("metrics_reader_recoveries", 0)
                        ) + 1
                        result["metrics_reader_last_recovery_ros"] = (
                            recovered.get("ros_time")
                        )
                    if watch.stalled(now):
                        reported_pose = (
                            pose_latest.get("pose")
                            if isinstance(pose_latest, dict)
                            and pose_latest.get("pose") is not None
                            else latest.get("pose")
                        )
                        if reported_pose is None and pose_latest:
                            reported_pose = [
                                pose_latest.get("x"), pose_latest.get("y"),
                                pose_latest.get("theta", 0.0),
                            ]
                        result.update(
                            status="failed",
                            reason="stationary_over_%g_wall_seconds" % args.stall_seconds,
                            stall_pose=reported_pose,
                            stall_anchor=watch.anchor,
                            stationary_wall_seconds=watch.age(now),
                            last_sample=latest,
                            last_pose_record=pose_latest or None,
                        )
                        break
                code = launcher.poll()
                if code is not None and code != 0:
                    result.update(status="infrastructure_failure", reason="launcher_failed",
                                  launcher_exit_code=code)
                    break
                if not latest and not pose_latest and elapsed_total > 420:
                    result.update(status="infrastructure_failure",
                                  reason="no_metrics_pose_after_420s")
                    break
                if now - last_report >= 60:
                    try:
                        metrics_size = metrics.stat().st_size
                    except OSError:
                        metrics_size = None
                    diagnostic = {
                        "run": run, "attempt": number,
                        "pose": (
                            pose_latest.get("x"), pose_latest.get("y")
                        ) if pose_latest else latest.get("pose"),
                        "total_s": round(elapsed_total, 1),
                        "stall_s": round(watch.age(now), 1),
                        "latest_ros": latest.get("ros_time"),
                        "tail_ros": (
                            None if tail_sample is None
                            else tail_sample.get("ros_time")
                        ),
                        "metrics_size": metrics_size,
                        "record_count": len(records),
                    }
                    print(json.dumps(diagnostic, ensure_ascii=False), flush=True)
                    save(campaign / "live_status.json",
                         dict(result, pose=(
                             [pose_latest.get("x"), pose_latest.get("y")]
                             if pose_latest else latest.get("pose")),
                              stationary_wall_seconds=watch.age(now),
                              total_elapsed_seconds=elapsed_total,
                              runner_pid=os.getpid(),
                              latest_ros=latest.get("ros_time"),
                              tail_ros=(
                                  None if tail_sample is None
                                  else tail_sample.get("ros_time")
                              ),
                              metrics_size=metrics_size))
                    last_report = now
                time.sleep(2)
    finally:
        if recorder is not None:
            recorder.terminate()
            try:
                recorder.wait(timeout=10)
            except Exception:
                recorder.kill()
        if "launcher" in locals() and launcher.poll() is None:
            launcher.terminate()
            try:
                launcher.wait(timeout=15)
            except Exception:
                launcher.kill()
        try:
            sink = output if output is not None else open(os.devnull, "w")
            lifecycle("./scripts/lifecycle/stop_all_tmux.sh", environment, sink)
        except Exception:
            pass
        capture_windows(directory)
    result["wall_seconds_including_startup"] = round(time.monotonic() - started, 3)
    result["ended_at"] = dt.datetime.now().isoformat()
    save(directory / "attempt_result.json", result)

    visualization = directory / (run + "_visualization.log")
    with visualization.open("w") as stream:
        subprocess.run([sys.executable, str(WS / "scripts/tools/plot_world_run.py"),
                        str(metrics), "--world", str(args.world),
                        "--output-dir", str(directory / "visualization")],
                       stdout=stream, stderr=subprocess.STDOUT, timeout=300, check=False)
        # Second per-run figure: measured route with WI dispatch indices,
        # physically committed Place tree, and actual DFS transactions.
        subprocess.run([sys.executable, str(WS / "scripts/tools/plot_place_dfs.py"),
                        str(metrics), "--world", str(args.world),
                        "--output", str(directory / "visualization"
                                        / (run + "_place_dfs.png"))],
                       stdout=stream, stderr=subprocess.STDOUT, timeout=300, check=False)
        # The per-run visualization contract is exactly two PNGs.  Keep the
        # summary in sync so downstream tooling can discover both without
        # re-deriving filenames.
        summary_path = directory / "visualization" / "visualization_summary.json"
        place_figure = directory / "visualization" / (run + "_place_dfs.png")
        if summary_path.exists() and place_figure.exists():
            try:
                report = json.loads(summary_path.read_text())
                figures = [
                    str(value) for value in report.get("figures", ())
                    if str(value).lower().endswith(".png")
                ]
                if str(place_figure) not in figures:
                    figures.append(str(place_figure))
                report["figures"] = figures
                summary_path.write_text(json.dumps(report, indent=2) + "\n")
            except (OSError, ValueError, TypeError):
                pass

    audit_path = directory / (run + "_audit.json")
    audit = subprocess.run([sys.executable, str(WS / "scripts/tools/audit_search_attempt.py"),
                            "--attempt-result", str(directory / "attempt_result.json"),
                            "--out", str(audit_path)],
                           capture_output=True, text=True, timeout=180)
    result["audit_path"] = str(audit_path)
    result["audit_stdout"] = audit.stdout
    result["audit_stderr"] = audit.stderr[-2000:]
    if audit_path.exists():
        result["audit"] = json.loads(audit_path.read_text())
    save(directory / "attempt_result.json", result)
    return result


def write_report(campaign, state):
    def num(value, digits=1):
        if value is None:
            return "—"
        try:
            return ("%%.%df" % digits) % float(value)
        except (TypeError, ValueError):
            return str(value)

    world_stem = Path(str(state.get("world", ""))).stem
    task_id = str(state.get("task_id") or "task")
    lines = ["# %s %s：%d 轮审计记录" % (
                 world_stem or "world", task_id, len(state["attempts"])), "",
             "判据：**成功 = 感知状态机进入 `LOCKED`（state=2）**；"
             "`task_done=true` 同样记为成功。单轮不设总时限；"
             "原地无进展超过看门狗阈值即判失败并进入下一轮。"
             "门洞、路径、停车只作观察记录，不影响成功判定。", "",
             "| 轮 | run | 结论 | 找到 | locked | 活跃墙钟s | 启动s | 最长停顿s | portal(观察) | 失败原因 |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for attempt in state["attempts"]:
        audit = attempt.get("audit") or {}
        timings = audit.get("timings") or {}
        checks = audit.get("structural_checks") or {}
        verdict = audit.get("verdict", attempt.get("status"))
        found = "是" if (audit.get("task_success")
                         or (audit.get("completion") or {}).get("task_done")) else "否"
        lines.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            attempt.get("attempt"), attempt.get("run"), verdict, found,
            "是" if audit.get("locked_seen") else "否",
            num(timings.get("active_wall_seconds")),
            num(timings.get("startup_seconds")),
            num(timings.get("stationary_wall_seconds") if timings.get("stationary_wall_seconds") is not None else timings.get("stopped_max_s")),
            "%s/%s/%s/%s" % (checks.get("portal_count"), checks.get("portal_matched"),
                             checks.get("portal_crossed"), checks.get("portal_false")),
            attempt.get("reason") or "—"))
    total = len(state["attempts"])
    ok = sum(1 for a in state["attempts"] if (a.get("audit") or {}).get("task_success"))
    timeouts = sum(1 for a in state["attempts"]
                   if (a.get("audit") or {}).get("verdict") == "timeout_failure")
    infra = sum(1 for a in state["attempts"]
                if (a.get("audit") or {}).get("verdict") == "infrastructure_failure")
    stalls = [t for a in state["attempts"]
              for t in [((a.get("audit") or {}).get("timings") or {}).get("stationary_wall_seconds")]
              if t is not None]
    active = [t for a in state["attempts"]
              for t in [((a.get("audit") or {}).get("timings") or {}).get("active_wall_seconds")]
              if t is not None]
    def median(values):
        if not values:
            return None
        ordered = sorted(values)
        mid = len(ordered) // 2
        return ordered[mid] if len(ordered) % 2 else (ordered[mid - 1] + ordered[mid]) / 2.0
    lines += ["", "成功 %d / %d；超时失败 %d；基础设施失败 %d。" % (ok, total, timeouts, infra),
              "活跃墙钟中位数 %s s；停顿中位数 %s s；停顿最长 %s s。" % (
                  num(median(active)), num(median(stalls)), num(max(stalls) if stalls else None)), ""]
    (campaign / "audit_report.md").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-dir", required=True)
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--stall-seconds", type=float, default=300.0)
    parser.add_argument("--radius", type=float, default=0.35)
    parser.add_argument("--max-total-seconds", type=float, default=7200.0)
    args = parser.parse_args()
    # Non-interactive bash does not read ~/.bashrc, so put the user's conda
    # installation on PATH before any tmux window is created.  pipeline_env.sh
    # then sources conda.sh and the detector/prompt windows can activate their
    # environments (dino / minicpm).
    # Expose only a conda shim, not anaconda's python: roslaunch runs xacro
    # with `#!/usr/bin/env python3`, and a conda python on PATH breaks the
    # Noetic xacro/Pro3 spawn.  pipeline_env.sh sees `conda`, sources conda.sh,
    # and the detector/prompt windows activate their own environments.
    campaign = Path(args.campaign_dir).resolve()
    campaign.mkdir(parents=True, exist_ok=True)
    shim = campaign / "conda_shim"
    shim.mkdir(exist_ok=True)
    conda = Path.home() / "anaconda3" / "bin" / "conda"
    if conda.exists():
        entry = shim / "conda"
        entry.write_text("#!/bin/sh\nexec %s \"$@\"\n" % conda)
        entry.chmod(0o755)
        os.environ["PATH"] = str(shim) + os.pathsep + os.environ.get("PATH", "")
    with (campaign / "runner.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Clear a stale lifecycle owner left by an interrupted launcher;
        # otherwise run_all_tmux.sh exits 75 before starting anything.
        for pattern in ("scripts/lifecycle/run_all_tmux.sh",
                        "scripts/lifecycle/run_env_tmux.sh",
                        "scripts/lifecycle/run_nodes_tmux.sh"):
            subprocess.run(["pkill", "-9", "-f", pattern],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           check=False)
        subprocess.run(["tmux", "kill-server"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)
        config = campaign / "pipeline_config.yaml"
        if not config.exists():
            values = yaml.safe_load((WS / "scripts/config/pipeline_defaults.yaml").read_text())
            values["TARGET_DONE_REQUIRE_LOCKED"] = True
            config.write_text(yaml.safe_dump(values, sort_keys=False, allow_unicode=True))
        values = yaml.safe_load(config.read_text())
        args.world = WS / values["WORLD"]
        shutil.copy2(args.world, campaign / args.world.name)
        state_path = campaign / "campaign.json"
        state = json.loads(state_path.read_text()) if state_path.exists() else {
            "created_at": dt.datetime.now().isoformat(), "attempts": [],
            "world": str(args.world), "task_id": values["TASK_ID"],
            "git_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(WS), text=True).strip(),
            "stall_seconds": args.stall_seconds, "radius_m": args.radius,
            "max_total_seconds": args.max_total_seconds, "status": "running",
        }
        save(state_path, state)
        save(campaign / "policy.json", {"stall_seconds": args.stall_seconds,
                                        "stationary_radius_m": args.radius,
                                        "max_total_seconds": args.max_total_seconds,
                                        "max_exploration_seconds": args.max_total_seconds})
        while len(state["attempts"]) < args.attempts:
            result = run_attempt(campaign, len(state["attempts"]) + 1, args, config)
            state["attempts"].append(result)
            save(state_path, state)
            write_report(campaign, state)
            print(json.dumps({"attempt": result["attempt"], "run": result["run"],
                              "status": result["status"],
                              "verdict": (result.get("audit") or {}).get("verdict")},
                             ensure_ascii=False), flush=True)
        state["status"] = "complete"
        state["completed_at"] = dt.datetime.now().isoformat()
        save(state_path, state)
        write_report(campaign, state)


if __name__ == "__main__":
    main()
