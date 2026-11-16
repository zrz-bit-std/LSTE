#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Summarise one navigation-metrics run: coverage, target evidence, task_done."""
import glob
import json
import os
import sys

run = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob("/home/zrz/work/lste_ws/runtime/navigation/logs/2026*"))[-1]
log = os.path.join(run, os.path.basename(run) + "_navigation_metrics.log")
last = None
samples = 0
for line in open(log, encoding="utf-8", errors="ignore"):
    if " event=sample " in line:
        try:
            last = json.loads(line[line.index("data=") + 5:])
            samples += 1
        except Exception:
            pass
if last is None:
    print("no samples in", log)
    sys.exit(1)

def g(k, d=None):
    return last.get(k, d)

print("run:", run)
print("samples:", samples, " ros_time:", g("ros_time"))
print("task:", g("goal_diagnostic", {}).get("task_id"), " state:", g("goal_diagnostic", {}).get("target_state"))
print("pose:", g("pose"))
print("path_length_m:", g("path_length"), " goals:", g("goal_changes"),
      " unique_goals:", g("move_base_unique_goal_ids"), " successes:", g("move_base_successes"))
print("task_done:", g("task_done"), " target_messages:", g("target_messages"))
print("target_first_seen_s:", g("target_first_seen_seconds"),
      " follow_confirmed_s:", g("target_follow_confirmed_seconds"))
print("map:", g("map", {}).get("free"), "free cells,",
      g("map", {}).get("occupied"), "occupied,", g("map", {}).get("unknown"), "unknown")
print("scan_min:", g("scan_min"), " stops:", g("stop_events"), " brakes:", g("linear_brake_events"))
print("collision_truth:", g("benchmark_collision_truth", {}).get("contact_messages"),
      " hard_stop:", g("hard_stop_events"))
geo = g("target_geometric_evaluation", {})
print("detector_frames_evaluated:", geo.get("evaluated_detector_frames"),
      " matched:", geo.get("matched_detector_frames"),
      " target_model:", geo.get("target_model"))
print("failure_count:", g("failure_evidence", {}).get("failure_count"),
      " recoveries:", g("move_base_recovery_events"))
