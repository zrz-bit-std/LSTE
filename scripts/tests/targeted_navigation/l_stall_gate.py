"""Post-run / mid-run stall gate for the L scenario.

The gate exists because two consecutive L runs looked "quiet" while the robot
had actually stopped moving: the failure counter stayed low and the audit only
reported at the end.  These four checks catch the stall while the run is still
worth stopping.

    python3 l_stall_gate.py <run_dir_or_metrics_log> [--max-idle-seconds 120]

Exit code 0 only when every gate passes.
"""

import argparse
import glob
import json
import math
import os
import sys
from collections import Counter


IDLE_GATE_DEFAULT = 120.0


def locate(run_dir):
    if os.path.isfile(run_dir):
        return run_dir, os.path.dirname(run_dir)
    logs = sorted(glob.glob(os.path.join(run_dir, "*navigation_metrics.log")))
    if not logs:
        # The suite directory and the metrics directory are different trees;
        # the sealed summary records the metrics path it used.
        summary = os.path.join(run_dir, "summary.log")
        if os.path.isfile(summary):
            for line in open(summary, errors="replace"):
                if "run=" in line:
                    candidate = line.strip().split("run=", 1)[1]
                    logs = sorted(glob.glob(os.path.join(
                        candidate, "*navigation_metrics.log")))
                    if logs:
                        return logs[-1], candidate
    if not logs:
        # Interrupted runs have no summary. The metrics directory is created
        # within a couple of seconds of the suite directory, so match on that.
        stamp = os.path.basename(run_dir.rstrip("/")).rsplit("_", 2)[-2:]
        if len(stamp) == 2:
            from datetime import datetime
            try:
                suite_time = datetime.strptime("_".join(stamp), "%Y%m%d_%H%M%S")
            except ValueError:
                suite_time = None
            if suite_time is not None:
                ws = os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.abspath(run_dir))))
                parent = os.path.join(ws, "runtime/targeted_navigation")
                best = None
                for metrics_dir in sorted(glob.glob(
                        os.path.join(parent, "*", "logs", "*"))):
                    name = os.path.basename(metrics_dir)
                    try:
                        delta = abs((datetime.strptime(name, "%Y%m%d_%H%M%S")
                                     - suite_time).total_seconds())
                    except ValueError:
                        continue
                    if delta <= 120 and (best is None or delta < best[0]):
                        best = (delta, metrics_dir)
                if best is not None:
                    logs = sorted(glob.glob(
                        os.path.join(best[1], "*navigation_metrics.log")))
                    if logs:
                        return logs[-1], best[1]
    if not logs:
        raise SystemExit("no navigation metrics log under %s" % run_dir)
    return logs[-1], run_dir


def read(metrics_path):
    reasons = Counter()
    last_contract_sim = None
    samples = []
    streak = 0
    longest_streak = 0
    escapes = 0
    for line in open(metrics_path, errors="replace"):
        if "event=planner_command_contract" in line:
            index = line.find("data=")
            try:
                data = json.loads(line[index + 5:])
            except Exception:
                continue
            reason = data.get("reason")
            reasons["s%s:%s" % (data.get("state"), reason)] += 1
            last_contract_sim = data.get("ros_time")
            if reason in ("planner_start_pose_infeasible", "planner_clearance_stall"):
                streak += 1
                longest_streak = max(longest_streak, streak)
            else:
                streak = 0
            if "escape" in str(reason):
                escapes += 1
        elif "event=sample" in line:
            index = line.find("data=")
            try:
                data = json.loads(line[index + 5:])
            except Exception:
                continue
            pose = data.get("pose") or [0.0, 0.0, 0.0]
            samples.append((data.get("ros_time"), pose[0], pose[1]))
    return reasons, last_contract_sim, samples, longest_streak, escapes


def movement_over(samples, window_seconds):
    if len(samples) < 2:
        return 0.0, 0.0
    last_sim = samples[-1][0] or 0.0
    recent = [s for s in samples if s[0] and s[0] > last_sim - window_seconds]
    if len(recent) < 2:
        return 0.0, 0.0
    span = (recent[-1][0] or 0.0) - (recent[0][0] or 0.0)
    distance = math.hypot(recent[-1][1] - recent[0][1],
                          recent[-1][2] - recent[0][2])
    return distance, span


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run")
    parser.add_argument("--idle-seconds", type=float, default=IDLE_GATE_DEFAULT,
                        help="fail when no planner contract was published for "
                             "this many simulated seconds")
    parser.add_argument("--min-move-meters", type=float, default=0.30)
    parser.add_argument("--move-window", type=float, default=300.0)
    parser.add_argument("--max-start-pose-streak", type=int, default=40)
    args = parser.parse_args(argv)

    metrics, directory = locate(args.run)
    reasons, last_contract, samples, streak, escapes = read(metrics)
    last_sim = samples[-1][0] if samples else 0.0
    moved, span = movement_over(samples, args.move_window)
    failures = len(glob.glob(os.path.join(directory, "*_failure_[0-9][0-9][0-9][0-9].json")))
    planner_commands = reasons.get("s1:planner_command", 0)

    idle = None if last_contract is None else last_sim - last_contract
    checks = []

    checks.append((
        "planner commands flowing",
        planner_commands > 0 and idle is not None and idle <= args.idle_seconds,
        "n=%d, idle=%.1f s (limit %.0f s)"
        % (planner_commands, idle if idle is not None else -1.0, args.idle_seconds),
    ))
    checks.append((
        "robot moving",
        moved >= args.min_move_meters,
        "%.2f m over %.0f s sim (min %.2f m)"
        % (moved, span, args.min_move_meters),
    ))
    checks.append((
        "no unbounded start-pose streak",
        streak <= args.max_start_pose_streak,
        "longest=%d (limit %d), escapes=%d"
        % (streak, args.max_start_pose_streak, escapes),
    ))
    checks.append((
        "failure artifacts bounded",
        failures <= 30,
        "n=%d" % failures,
    ))

    print("metrics: %s" % metrics)
    print("final sim: %.1f s" % (last_sim or 0.0))
    print("contract reasons: %s" % reasons.most_common(8))
    print()
    failed = 0
    for name, ok, detail in checks:
        print("%-32s %s  (%s)" % (name, "PASS" if ok else "FAIL", detail))
        failed += 0 if ok else 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
