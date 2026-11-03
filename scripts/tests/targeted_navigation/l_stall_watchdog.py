"""Terminate a running scenario as soon as it stalls longer than a threshold.

The user-facing rule for the L/T/Cross suite is:

* a healthy scenario finishes on its own in <= 40 minutes, and
* any uninterrupted loss of progress longer than 3 minutes is a failure.

Historically a pathological stall lasted 568-2282 s (mean ~1006 s), while the
longest *normal* pause in a successful run was 69 s.  A 180 s threshold
therefore sits at 2.6x the normal worst case and saves 10-35 minutes per failed
scenario.

The watchdog is deliberately external: it never edits the run path, it only
observes the metrics stream and asks roslaunch/wrapper to shut down.  The stall
evidence is written before the kill so the failure is still auditable.

Usage:
    python3 l_stall_watchdog.py --suite-root <dir> --wrapper-pid <pid>
"""

import argparse
import glob
import json
import os
import signal
import subprocess
import sys
import time
from collections import deque


def newest_metrics_run(since_wall):
    """Newest navigation metrics log created after the suite started."""
    best = None
    for path in glob.glob("/home/zrz/work/lste_ws/runtime/targeted_navigation/*/logs/*"):
        if not os.path.isdir(path):
            continue
        try:
            if os.path.getmtime(path) < since_wall:
                continue
        except OSError:
            continue
        logs = glob.glob(os.path.join(path, "*navigation_metrics.log"))
        if logs and (best is None or os.path.getmtime(path) > best[0]):
            best = (os.path.getmtime(path), logs[0], path)
    return best


def parse_samples(lines, out):
    for line in lines:
        if "event=sample" not in line:
            continue
        index = line.find("data=")
        if index < 0:
            continue
        try:
            data = json.loads(line[index + 5:])
        except Exception:
            continue
        pose = data.get("pose")
        sim = data.get("ros_time")
        if not pose or sim is None:
            continue
        out.append((float(sim), float(pose[0]), float(pose[1])))


def stalled_seconds(samples, threshold):
    """Sim seconds for which the robot never left a 0.30 m radius."""
    if len(samples) < 2:
        return 0.0
    latest = samples[-1][0]
    anchor = None
    for sim, x, y in reversed(samples):
        if latest - sim > threshold:
            break
        if anchor is None:
            anchor = (x, y)
            continue
        dx, dy = x - anchor[0], y - anchor[1]
        if dx * dx + dy * dy > 0.30 * 0.30:
            return 0.0
    first = samples[0][0]
    for sim, x, y in reversed(samples):
        if latest - sim >= threshold:
            return latest - sim if sim >= first else 0.0
    return latest - first


def write_artifact(suite_root, context, window):
    target = os.path.join(
        "/home/zrz/work/lste_ws/runtime/diagnostics/20261031_stall_watchdog")
    os.makedirs(target, exist_ok=True)
    path = os.path.join(target, "stall_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    payload = dict(stall=context, samples=[dict(sim=round(s, 2), x=round(x, 3),
                                                y=round(y, 3))
                                           for s, x, y in window],
                   suite_root=suite_root, timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"))
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2)
    return path


def launch_pids_for_run(lines, run_dir):
    """Select only the roslaunch that owns this attempt.

    Run 20261103_062322 died 17 s after launch because a leftover watchdog from
    the previous attempt matched the *new* roslaunch by command-line pattern.
    Every suite roslaunch carries ``metrics_run_directory:=<logs/stamp>``, so
    match on that instead of on the program name.
    """
    run_dir = str(run_dir).rstrip("/")
    pids = []
    for line in lines:
        if "t_junction_nav.launch" not in line or run_dir not in line:
            continue
        try:
            pids.append(int(line.split(None, 1)[0]))
        except (IndexError, ValueError):
            continue
    return pids


def terminate(metrics_path, wrapper_pid, reason):
    run_dir = os.path.dirname(os.path.abspath(metrics_path))
    launch = subprocess.run(["pgrep", "-af", "roslaunch.*t_junction_nav.launch"],
                            capture_output=True, text=True)
    pids = launch_pids_for_run(launch.stdout.splitlines(), run_dir)
    if not pids:
        print("watchdog: no roslaunch owns %s; leaving others alone" % run_dir,
              flush=True)
    for pid in pids:
        try:
            os.kill(int(pid), signal.SIGINT)
        except (OSError, ValueError):
            pass
    if wrapper_pid:
        try:
            os.kill(int(wrapper_pid), signal.SIGTERM)
        except (OSError, ValueError):
            pass


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite-root", required=True)
    parser.add_argument("--wrapper-pid", type=int, default=0)
    parser.add_argument("--stall-seconds", type=float, default=180.0)
    parser.add_argument("--poll-seconds", type=float, default=10.0)
    parser.add_argument("--startup-sim-seconds", type=float, default=60.0)
    parser.add_argument("--max-wall-seconds", type=float, default=3600.0)
    parser.add_argument("--since-wall", type=float, default=0.0)
    args = parser.parse_args(argv)

    start_wall = time.time()
    since_wall = args.since_wall if args.since_wall > 0.0 else start_wall - 300.0
    resolved = None
    offset = 0
    samples = deque(maxlen=4000)
    print("watchdog: waiting for metrics", flush=True)

    while True:
        if time.time() - start_wall > args.max_wall_seconds:
            print("watchdog: wall cap reached, leaving the suite alone", flush=True)
            return 0
        if resolved is None:
            found = newest_metrics_run(since_wall)
            if found is None:
                time.sleep(args.poll_seconds)
                continue
            resolved = found[1]
            print("watchdog: metrics=%s" % resolved, flush=True)
        try:
            size = os.path.getsize(resolved)
        except OSError:
            time.sleep(args.poll_seconds)
            continue
        if size > offset:
            with open(resolved, "r", errors="replace") as handle:
                handle.seek(offset)
                chunk = handle.read()
                offset = handle.tell()
            parse_samples(chunk.splitlines(), samples)
        if not samples or samples[-1][0] < args.startup_sim_seconds:
            time.sleep(args.poll_seconds)
            continue
        stalled = stalled_seconds(samples, args.stall_seconds)
        if stalled >= args.stall_seconds:
            sim, x, y = samples[-1]
            context = dict(reason="no_xy_progress",
                           stalled_sim_seconds=round(stalled, 1),
                           threshold=args.stall_seconds,
                           last_sim=round(sim, 1),
                           last_pose=[round(x, 3), round(y, 3)],
                           metrics=resolved)
            path = write_artifact(args.suite_root, context, list(samples)[-40:])
            print("watchdog: STALL %.1f s at sim %.1f pose (%.2f, %.2f); "
                  "artifact=%s" % (stalled, sim, x, y, path), flush=True)
            terminate(resolved, args.wrapper_pid, "stall")
            return 1
        time.sleep(args.poll_seconds)


if __name__ == "__main__":
    sys.exit(main())
