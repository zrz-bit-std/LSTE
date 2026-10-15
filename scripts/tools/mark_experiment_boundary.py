#!/usr/bin/env python3
"""Mark a task-validation slice boundary without resetting runtime state.

This is the continuation counterpart to ``hard_reset_runtime.py``.  It closes
the metrics-owned evidence episode for one slice, but leaves Gazebo, SLAM,
Graph, Goal Manager, Bridge, and the current task lease untouched.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
SOURCE_SCRIPTS = ROOT / "src/lste_topo_access/scripts"
if str(SOURCE_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SOURCE_SCRIPTS))

import rospy
from std_msgs.msg import String

from experiment_reset_contract import EXPERIMENT_BOUNDARY_TOPIC
from lifecycle_manager import LifecycleManager


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boundary-id", required=True)
    parser.add_argument("--slice-id", default="")
    parser.add_argument("--reason", default="chunk_boundary")
    parser.add_argument("--failure-trigger", default="")
    parser.add_argument("--failure-details", default="{}")
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")

    try:
        failure_details = json.loads(args.failure_details or "{}")
    except (TypeError, ValueError, json.JSONDecodeError):
        failure_details = {}
    if not isinstance(failure_details, dict):
        failure_details = {}

    rospy.init_node(
        "lste_experiment_boundary_marker", anonymous=True, disable_signals=True
    )
    publisher = rospy.Publisher(EXPERIMENT_BOUNDARY_TOPIC, String, queue_size=5)
    transaction_id = LifecycleManager._new_transaction_id()
    failure_trigger = str(args.failure_trigger or "").strip()
    payload = {
        "event": "experiment_boundary",
        "boundary_id": str(args.boundary_id),
        "slice_id": str(args.slice_id or ""),
        "reason": str(args.reason or "chunk_boundary"),
        "transaction_id": int(transaction_id),
        "failure_trigger": failure_trigger,
        "failure_details": failure_details,
        "force_failure": bool(failure_trigger),
        "preserve_task_state": True,
        "requested_wall_time": time.time(),
    }
    message = String(data=json.dumps(payload, sort_keys=True))
    deadline = time.monotonic() + float(args.timeout)
    while (
        not rospy.is_shutdown()
        and publisher.get_num_connections() == 0
        and time.monotonic() < deadline
    ):
        time.sleep(0.05)
    sent = False
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        publisher.publish(message)
        sent = True
        time.sleep(0.15)
        if time.monotonic() + 0.15 >= deadline:
            break
    result = {
        "event": "experiment_boundary_sent",
        "boundary_id": str(args.boundary_id),
        "slice_id": str(args.slice_id or ""),
        "transaction_id": int(transaction_id),
        "preserve_task_state": True,
        "published": sent,
        "topic": EXPERIMENT_BOUNDARY_TOPIC,
    }
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if sent else 2


if __name__ == "__main__":
    raise SystemExit(main())
