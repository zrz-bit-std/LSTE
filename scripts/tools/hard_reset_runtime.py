#!/usr/bin/env python3
"""Synchronously hard-reset the live LSTE runtime for one diagnostic slice."""

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
from std_srvs.srv import Empty

from experiment_reset_contract import (
    EXPERIMENT_BOUNDARY_TOPIC,
    HARD_RESET_ACK_TOPIC,
    HARD_RESET_RELEASE_TOPIC,
    HARD_RESET_TOPIC,
)
from lifecycle_manager import LifecycleManager


def _as_bool(value):
    return bool(value) if isinstance(value, bool) else str(value).strip().lower() in {
        "1", "true", "yes", "on"
    }


def _clean_ack(node, payload, expected_reset_transaction_id=None):
    """Validate the state fields that prove ownership was actually cleared."""
    if expected_reset_transaction_id is not None:
        try:
            acknowledged_reset_transaction_id = int(
                payload["reset_transaction_id"]
            )
        except (KeyError, TypeError, ValueError):
            return False
        if acknowledged_reset_transaction_id != int(
            expected_reset_transaction_id
        ):
            return False
    if str(payload.get("state", "")).upper() != "IDLE":
        return False
    if node == "lste_cmd_vel_mux":
        return _as_bool(payload.get("actuator_zero")) and _as_bool(
            payload.get("hard_reset_hold")
        )
    if node == "lste_global_frontier":
        return (
            int(payload.get("active_route_id", 0) or 0) == 0
            and not _as_bool(payload.get("route_owner"))
            and not _as_bool(payload.get("graph_transaction_active"))
            and not _as_bool(payload.get("graph_route_lease_active"))
        )
    if node == "lste_teb_goal_bridge":
        return (
            not _as_bool(payload.get("action_active"))
            and not _as_bool(payload.get("route_owner"))
            and int(payload.get("active_route_id", 0) or 0) == 0
            and int(payload.get("latest_route_id", 0) or 0) == 0
            and not _as_bool(payload.get("intent_seen"))
            and not _as_bool(payload.get("route_lease_watchdog_active"))
            and _as_bool(payload.get("persistent_commands_cleared", True))
        )
    if node == "lste_goal_manager":
        return not _as_bool(payload.get("active_goal")) and not _as_bool(
            payload.get("route_owner")
        )
    if node == "lste_teb_turn_supervisor":
        return (
            not _as_bool(payload.get("active_action"))
            and int(payload.get("planner_command_sequence", 0) or 0) == 0
            and int(payload.get("planner_transaction_id", 0) or 0) == 0
            and int(payload.get("output_sequence", 0) or 0) == 0
            and _as_bool(payload.get("planner_command_cleared"))
        )
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reset-id", required=True)
    parser.add_argument("--slice-id", default="")
    parser.add_argument("--reason", default="slice_boundary")
    parser.add_argument(
        "--failure-trigger",
        default="",
        help="explicit metrics failure trigger at this trial boundary",
    )
    parser.add_argument(
        "--failure-details",
        default="{}",
        help="compact JSON details for an explicit failure trigger",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--reset-world",
        action="store_true",
        help="reset Gazebo model poses while the actuator hold is engaged",
    )
    parser.add_argument("--world-reset-timeout", type=float, default=5.0)
    parser.add_argument(
        "--require",
        action="append",
        dest="required",
        help="required node identity; repeat for each participant",
    )
    args = parser.parse_args()
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")
    if args.world_reset_timeout <= 0.0:
        raise SystemExit("--world-reset-timeout must be positive")
    required = {
        str(value).strip().lstrip("/")
        for value in (args.required or ())
        if str(value).strip()
    }
    if not required:
        required = {
            "lste_goal_manager",
            "lste_teb_goal_bridge",
            "lste_cmd_vel_mux",
        }

    rospy.init_node("lste_hard_reset_runtime", anonymous=True, disable_signals=True)
    acknowledgements = {}
    release_acknowledgements = {}
    reset_transaction_id = LifecycleManager._new_transaction_id()

    def on_ack(message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        if str(payload.get("event", "")) != "hard_reset_ack":
            if str(payload.get("event", "")) != "hard_reset_release_ack":
                return
            if str(payload.get("reset_id", "")) != str(args.reset_id):
                return
            node = str(payload.get("node", "")).strip().lstrip("/")
            if node:
                release_acknowledgements[node] = payload
            return
        if str(payload.get("reset_id", "")) != str(args.reset_id):
            return
        node = str(payload.get("node", "")).strip().lstrip("/")
        if node:
            acknowledgements[node] = payload

    subscriber = rospy.Subscriber(
        HARD_RESET_ACK_TOPIC, String, on_ack, queue_size=50
    )
    publisher = rospy.Publisher(HARD_RESET_TOPIC, String, queue_size=5)
    boundary_publisher = rospy.Publisher(
        EXPERIMENT_BOUNDARY_TOPIC, String, queue_size=5
    )
    try:
        failure_details = json.loads(args.failure_details or "{}")
    except (TypeError, ValueError, json.JSONDecodeError):
        failure_details = {}
    if not isinstance(failure_details, dict):
        failure_details = {}
    request = {
        "event": "hard_reset",
        "reset_id": str(args.reset_id),
        "transaction_id": int(reset_transaction_id),
        "slice_id": str(args.slice_id or ""),
        "reason": str(args.reason or "slice_boundary"),
        "requested_wall_time": time.time(),
        "failure_trigger": str(args.failure_trigger or "").strip(),
        "failure_details": failure_details,
    }
    message = String(data=json.dumps(request, sort_keys=True))
    boundary_request = {
        "event": "trial_boundary",
        "boundary_id": str(args.reset_id),
        "reset_id": str(args.reset_id),
        "transaction_id": int(reset_transaction_id),
        "slice_id": str(args.slice_id or ""),
        "reason": str(args.reason or "slice_boundary"),
        "failure_trigger": str(args.failure_trigger or "").strip(),
        "failure_details": failure_details,
        "force_failure": bool(str(args.failure_trigger or "").strip()),
        "requested_wall_time": time.time(),
    }
    boundary_message = String(
        data=json.dumps(boundary_request, sort_keys=True)
    )
    # Metrics owns failure IDs and artifacts. Give its subscriber a bounded
    # delivery window before the reset clears the slice-local evidence ring.
    boundary_deadline = time.monotonic() + 1.0
    while (
        not rospy.is_shutdown()
        and boundary_publisher.get_num_connections() == 0
        and time.monotonic() < boundary_deadline
    ):
        time.sleep(0.05)
    boundary_publisher.publish(boundary_message)
    time.sleep(0.15)
    deadline = time.monotonic() + float(args.timeout)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        publisher.publish(message)
        if required.issubset(acknowledgements) and all(
            _clean_ack(
                node,
                acknowledgements[node],
                reset_transaction_id,
            )
            for node in required
        ):
            break
        time.sleep(0.10)

    clean = {
        node: _clean_ack(
            node,
            acknowledgements[node],
            reset_transaction_id,
        )
        for node in sorted(required)
        if node in acknowledgements
    }
    if not (required.issubset(clean) and all(clean.values())):
        result = {
            "event": "hard_reset_verified",
            "reset_id": str(args.reset_id),
            "slice_id": str(args.slice_id or ""),
            "reason": str(args.reason or "slice_boundary"),
            "required_nodes": sorted(required),
            "acknowledged_nodes": sorted(acknowledgements),
            "clean_nodes": sorted(node for node, ok in clean.items() if ok),
            "all_nodes_idle": False,
            "release_acknowledged_nodes": [],
            "actuator_hold_released": False,
            "acks": {
                node: acknowledgements[node]
                for node in sorted(acknowledgements)
                if node in required
            },
        }
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        subscriber.unregister()
        return 2

    world_reset = {
        "requested": bool(args.reset_world),
        "performed": False,
        "service": "/gazebo/reset_world",
    }
    if args.reset_world:
        try:
            rospy.wait_for_service(
                "/gazebo/reset_world", timeout=float(args.world_reset_timeout)
            )
            rospy.ServiceProxy("/gazebo/reset_world", Empty)()
            world_reset["performed"] = True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            result = {
                "event": "hard_reset_verified",
                "reset_id": str(args.reset_id),
                "transaction_id": int(reset_transaction_id),
                "slice_id": str(args.slice_id or ""),
                "reason": str(args.reason or "slice_boundary"),
                "required_nodes": sorted(required),
                "acknowledged_nodes": sorted(acknowledgements),
                "clean_nodes": sorted(node for node, ok in clean.items() if ok),
                "all_nodes_idle": True,
                "release_acknowledged_nodes": [],
                "actuator_hold_released": False,
                "world_reset": world_reset,
                "world_reset_error": "%s:%s" % (type(exc).__name__, exc),
                "acks": {
                    node: acknowledgements[node]
                    for node in sorted(acknowledgements)
                    if node in required
                },
            }
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
            subscriber.unregister()
            return 2

    release_publisher = rospy.Publisher(
        HARD_RESET_RELEASE_TOPIC, String, queue_size=5
    )
    release_message = String(data=json.dumps({
        "event": "hard_reset_release",
        "reset_id": str(args.reset_id),
        "transaction_id": int(reset_transaction_id),
        "slice_id": str(args.slice_id or ""),
        "reason": str(args.reason or "slice_boundary"),
        "requested_wall_time": time.time(),
    }, sort_keys=True))
    release_deadline = time.monotonic() + min(3.0, float(args.timeout))
    while not rospy.is_shutdown() and time.monotonic() < release_deadline:
        release_publisher.publish(release_message)
        mux_ack = release_acknowledgements.get("lste_cmd_vel_mux")
        if mux_ack is not None and not _as_bool(mux_ack.get("hard_reset_hold", True)):
            break
        time.sleep(0.10)
    mux_ack = release_acknowledgements.get("lste_cmd_vel_mux")
    actuator_hold_released = bool(
        mux_ack is not None
        and not _as_bool(mux_ack.get("hard_reset_hold", True))
        and _as_bool(mux_ack.get("actuator_zero"))
    )
    result = {
        "event": "hard_reset_verified",
            "reset_id": str(args.reset_id),
            "transaction_id": int(reset_transaction_id),
        "slice_id": str(args.slice_id or ""),
        "reason": str(args.reason or "slice_boundary"),
        "required_nodes": sorted(required),
        "acknowledged_nodes": sorted(acknowledgements),
        "clean_nodes": sorted(node for node, ok in clean.items() if ok),
        "all_nodes_idle": True,
        "release_acknowledged_nodes": sorted(release_acknowledgements),
        "actuator_hold_released": actuator_hold_released,
        "world_reset": world_reset,
        "acks": {
            node: acknowledgements[node]
            for node in sorted(acknowledgements)
            if node in required
        },
    }
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    subscriber.unregister()
    return 0 if result["actuator_hold_released"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
