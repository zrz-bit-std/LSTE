#!/usr/bin/env python3
"""Inject one unreachable semantic target into a running persistent test.

This is a short ROS probe, not a second mission manager.  It publishes one
atomic ``mission_goal`` transaction and waits for the ownership chain:

    target_plan_failed -> target_controller_lease_released
    -> global_slam_frontier mission_goal_received

The probe is intentionally independent of the detector and Goal Manager.  It
lets a nearby T-junction run isolate target failure handling without replaying
the full office exploration.
"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import threading
import time

import rospy
from std_msgs.msg import String


PROCESS_NAME = "target_probe"


def _wall_timestamp():
    return datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S.%f%z")


def _timestamp_now():
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")


class ProbeLogger:
    """Write one timestamped process log with stable event fields."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("w", encoding="utf-8")
        self.lock = threading.Lock()

    def event(self, level, name, **fields):
        values = [
            _wall_timestamp(),
            "[%s]" % str(level).upper(),
            "process=%s" % PROCESS_NAME,
            "event=%s" % str(name),
        ]
        for key in sorted(fields):
            values.append("%s=%s" % (
                key,
                json.dumps(fields[key], ensure_ascii=True, sort_keys=True),
            ))
        with self.lock:
            self.stream.write(" ".join(values) + "\n")
            self.stream.flush()

    def close(self):
        with self.lock:
            self.stream.close()


def _decode(message):
    try:
        value = json.loads(message.data)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


class TargetMissionProbe:
    def __init__(self, args, logger):
        self.args = args
        self.logger = logger
        self.started_monotonic = time.monotonic()
        self.target_failed = False
        self.target_released = False
        self.frontier_takeover = False
        self.events = []
        self._failure_seen_at = None
        self._release_seen_at = None
        self._takeover_seen_at = None
        self._failure_payload = None
        self._release_payload = None
        self._lock = threading.Lock()
        self.publisher = rospy.Publisher(
            args.mission_topic, String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            args.plan_result_topic, String, self._on_plan_result, queue_size=10
        )
        rospy.Subscriber(
            args.bridge_status_topic, String, self._on_bridge_status, queue_size=20
        )
        rospy.Subscriber(
            args.frontier_status_topic, String, self._on_frontier_status, queue_size=20
        )

    def _record(self, level, name, **fields):
        with self._lock:
            self.events.append(str(name))
        self.logger.event(level, name, **fields)

    def _on_plan_result(self, message):
        payload = _decode(message)
        if payload is None:
            return
        event = str(payload.get("event", "unknown"))
        if event not in ("target_plan_failed", "target_plan_installed"):
            return
        if int(payload.get("transaction_id", 0) or 0) != self.args.transaction_id:
            return
        if event == "target_plan_installed":
            self._record(
                "WARN",
                "target_plan_installed_unexpected",
                transaction_id=self.args.transaction_id,
                goal=payload.get("goal"),
                frame_id=payload.get("frame_id"),
            )
            return
        with self._lock:
            if self.target_failed:
                return
            self.target_failed = True
            self._failure_seen_at = time.monotonic()
            self._failure_payload = dict(payload)
        status = payload.get("status") or "NAVFN_NO_PATH"
        reason = payload.get("reason") or "target_plan_failed"
        self._record(
            "WARN",
            "target_plan_failed",
            transaction_id=self.args.transaction_id,
            status=status,
            reason=reason,
            goal=payload.get("goal"),
        )

    def _on_bridge_status(self, message):
        payload = _decode(message)
        if payload is None:
            return
        event = payload.get("event")
        if event in (
            "mission_goal_received",
            "persistent_target_plan_requested",
            "persistent_target_plan_failure_ignored",
            "persistent_target_plan_installed",
            "persistent_target_request_cleared",
            "persistent_target_plan_failed",
            "target_controller_lease_released",
        ):
            self._record(
                "DEBUG",
                "bridge_status",
                bridge_event=event,
                transaction_id=payload.get("transaction_id", 0),
                latest_transaction_id=payload.get("latest_goal_transaction_id", 0),
                pending_transaction_id=payload.get(
                    "pending_target_transaction", 0
                ),
                target_transaction_id=payload.get("target_transaction_id", 0),
                tombstone_transaction_id=payload.get(
                    "target_lease_tombstone_transaction_id", 0
                ),
                route_id=payload.get("route_id", payload.get("latest_route_id", 0)),
                reason=payload.get("reason", ""),
            )
        if event == "target_controller_lease_released":
            transaction_id = int(
                payload.get("target_transaction_id", 0)
                or payload.get("tombstone_transaction_id", 0)
                or 0
            )
            if transaction_id < self.args.transaction_id:
                return
            with self._lock:
                if self.target_released:
                    return
                self.target_released = True
                self._release_seen_at = time.monotonic()
                self._release_payload = dict(payload)
            self._record(
                "INFO",
                "target_controller_lease_released",
                transaction_id=transaction_id,
                next_owner=payload.get("next_owner"),
                transport_cancelled=payload.get("transport_cancelled"),
            )
            return

        # This is the bridge's ownership boundary, rather than a mere frontier
        # candidate publication. Ignore a latched pre-probe frontier status.
        if event != "mission_goal_received":
            return
        if str(payload.get("source", "")).strip().lower() != "global_slam_frontier":
            return
        now = time.monotonic()
        with self._lock:
            if self._release_seen_at is None or now < self._release_seen_at:
                return
            if self.frontier_takeover:
                return
            self.frontier_takeover = True
            self._takeover_seen_at = now
        self._record(
            "INFO",
            "frontier_takeover",
            transaction_id=payload.get("transaction_id", 0),
            route_id=payload.get("route_id", 0),
            source=payload.get("source"),
            goal=payload.get("goal"),
        )

    def _on_frontier_status(self, message):
        # Keep the frontier stream in the process log even when the bridge has
        # not yet accepted a replacement. It is useful evidence for a timeout.
        payload = _decode(message)
        if payload is None or payload.get("event") not in (
            "route_command", "route_invalidated", "frontier_exhausted",
        ):
            return
        self._record(
            "DEBUG",
            "frontier_status",
            frontier_event=payload.get("event"),
            route_id=payload.get("route_id", 0),
            active=payload.get("active"),
        )

    def _mission_payload(self):
        return {
            "event": "mission_goal",
            "transaction_id": int(self.args.transaction_id),
            "source": "target_probe",
            "priority": 2,
            "route_id": 0,
            "route_kind": "target_approach",
            "mission_route_kind": "target_approach",
            "target_epoch": int(self.args.target_epoch),
            "target_track_id": self.args.target_track_id,
            "target_viewpoint_candidate_id": "probe-wall",
            "target_viewpoint_attempt_id": "probe-wall-1",
            "frame_id": self.args.frame_id,
            "goal": [float(self.args.goal_x), float(self.args.goal_y)],
            "yaw": 0.0,
            "goal_context": {
                "method": "target_failure_probe",
                "task_id": "t_junction_target_failure",
                "goal_role": "semantic_target_approach",
            },
        }

    def run(self):
        payload = self._mission_payload()
        deadline = time.monotonic() + max(0.5, float(self.args.timeout))
        # A latched publisher preserves the transaction across bridge startup,
        # but waiting briefly for one subscriber makes the probe less sensitive
        # to ROS graph discovery in a fast test.
        connection_deadline = min(deadline, time.monotonic() + 3.0)
        while (
            not rospy.is_shutdown()
            and self.publisher.get_num_connections() == 0
            and time.monotonic() < connection_deadline
        ):
            rospy.sleep(0.05)
        encoded = String(data=json.dumps(payload, sort_keys=True))
        self.publisher.publish(encoded)
        self._record(
            "INFO",
            "target_published",
            transaction_id=self.args.transaction_id,
            target_track_id=self.args.target_track_id,
            goal=payload["goal"],
            frame_id=self.args.frame_id,
            subscriber_count=self.publisher.get_num_connections(),
        )

        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                complete = (
                    self.target_failed
                    and self.target_released
                    and (
                        self.frontier_takeover
                        or not self.args.require_frontier_takeover
                    )
                )
            if complete:
                self._record(
                    "INFO",
                    "probe_complete",
                    passed=True,
                    required_frontier_takeover=(
                        self.args.require_frontier_takeover
                    ),
                    frontier_takeover=self.frontier_takeover,
                )
                return True
            rate.sleep()
        with self._lock:
            state = {
                "target_failed": bool(self.target_failed),
                "target_released": bool(self.target_released),
                "frontier_takeover": bool(self.frontier_takeover),
                "required_frontier_takeover": bool(
                    self.args.require_frontier_takeover
                ),
            }
        self._record("ERROR", "probe_timeout", passed=False, state=state)
        return False


def _write_result(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-directory", default="")
    parser.add_argument("--run-timestamp", default="")
    parser.add_argument("--result-file", default="")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--goal-x", type=float, default=0.0)
    parser.add_argument("--goal-y", type=float, default=1.25)
    parser.add_argument("--target-epoch", type=int, default=1)
    parser.add_argument("--transaction-id", type=int, default=1001)
    parser.add_argument("--target-track-id", default="probe:unreachable-wall")
    parser.add_argument(
        "--require-frontier-takeover",
        action="store_true",
        help="Require a new frontier mission after target lease release",
    )
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--mission-topic", default="/lste/mission_goal")
    parser.add_argument(
        "--plan-result-topic",
        default="/lste/persistent_execution/target_plan_result",
    )
    parser.add_argument(
        "--bridge-status-topic", default="/lste/teb_goal_bridge/status"
    )
    parser.add_argument(
        "--frontier-status-topic", default="/lste/global_frontier/status"
    )
    return parser.parse_args()


def main():
    args = _arguments()
    run_timestamp = args.run_timestamp or _timestamp_now()
    log_directory = Path(args.log_directory or (
        Path(__file__).resolve().parents[3]
        / "runtime/targeted_navigation/t_junction_small/logs"
        / run_timestamp
    ))
    log_path = log_directory / (run_timestamp + "_target_probe.log")
    result_path = Path(args.result_file or (
        log_directory / (run_timestamp + "_target_probe_result.json")
    ))
    logger = ProbeLogger(log_path)
    rospy.init_node("lste_target_mission_probe", anonymous=False)
    logger.event(
        "INFO",
        "run_start",
        run_timestamp=run_timestamp,
        timeout_seconds=args.timeout,
        transaction_id=args.transaction_id,
        target_track_id=args.target_track_id,
        goal=[args.goal_x, args.goal_y],
        require_frontier_takeover=args.require_frontier_takeover,
    )
    probe = TargetMissionProbe(args, logger)
    passed = False
    try:
        passed = probe.run()
    finally:
        result = {
            "schema_version": 1,
            "artifact_kind": "target_mission_probe_result",
            "run_timestamp": run_timestamp,
            "log_file": str(log_path),
            "result_file": str(result_path),
            "transaction_id": int(args.transaction_id),
            "target_track_id": args.target_track_id,
            "goal": [float(args.goal_x), float(args.goal_y)],
            "required_frontier_takeover": bool(
                args.require_frontier_takeover
            ),
            "passed": bool(passed),
            "events": list(probe.events),
            "timings_seconds": {
                "target_failure": (
                    None
                    if probe._failure_seen_at is None
                    else round(
                        probe._failure_seen_at - probe.started_monotonic, 3
                    )
                ),
                "target_release": (
                    None
                    if probe._release_seen_at is None
                    else round(
                        probe._release_seen_at - probe.started_monotonic, 3
                    )
                ),
                "frontier_takeover": (
                    None
                    if probe._takeover_seen_at is None
                    else round(
                        probe._takeover_seen_at - probe.started_monotonic, 3
                    )
                ),
            },
            "state": {
                "target_failed": bool(probe.target_failed),
                "target_released": bool(probe.target_released),
                "frontier_takeover": bool(probe.frontier_takeover),
                "required_frontier_takeover": bool(
                    args.require_frontier_takeover
                ),
            },
            "failure_payload": probe._failure_payload,
            "release_payload": probe._release_payload,
            "created_at": _wall_timestamp(),
        }
        logger.event(
            "INFO" if passed else "ERROR",
            "run_stop",
            passed=bool(passed),
            result_file=str(result_path),
        )
        logger.close()
        # Publish the completion marker only after the process log contains
        # its terminal event, so a supervising runner cannot observe success
        # and terminate the probe while the log is still open.
        _write_result(result_path, result)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
