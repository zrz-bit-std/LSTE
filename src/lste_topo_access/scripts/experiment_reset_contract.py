#!/usr/bin/env python3
"""Wire contract for a bounded warm-slice hard reset.

The request is a transient JSON message rather than a latched topic.  Every
stateful node acknowledges the same reset id after its timer-owned lifecycle
has cleared its local ownership state.
"""

import json
import time


HARD_RESET_TOPIC = "/lste/experiment/hard_reset"
HARD_RESET_ACK_TOPIC = "/lste/experiment/hard_reset_ack"
HARD_RESET_RELEASE_TOPIC = "/lste/experiment/hard_reset_release"
EXPERIMENT_BOUNDARY_TOPIC = "/lste/experiment/trial_boundary"


def decode_reset_request(message):
    """Return a normalized reset request, or ``None`` for unrelated input."""
    raw = getattr(message, "data", message)
    if isinstance(raw, dict):
        payload = dict(raw)
    else:
        try:
            payload = json.loads(str(raw))
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {"reset_id": str(raw or "").strip()}
    if not isinstance(payload, dict):
        return None
    event = str(payload.get("event", "hard_reset") or "").strip().lower()
    if event not in ("hard_reset", "experiment_hard_reset"):
        return None
    reset_id = str(payload.get("reset_id", "") or "").strip()
    if not reset_id:
        return None
    failure_details = payload.get("failure_details", {})
    if not isinstance(failure_details, dict):
        failure_details = {}
    return {
        "event": "hard_reset",
        "reset_id": reset_id,
        "transaction_id": max(0, int(payload.get("transaction_id", 0) or 0)),
        "reason": str(payload.get("reason", "slice_boundary") or "slice_boundary"),
        "slice_id": str(payload.get("slice_id", "") or ""),
        "failure_trigger": str(payload.get("failure_trigger", "") or "").strip(),
        "failure_details": failure_details,
    }


def reset_ack_payload(node, request, state="IDLE", transaction_id=0, **fields):
    """Build one JSON-safe acknowledgement shared by all participants."""
    payload = {
        "event": "hard_reset_ack",
        "node": str(node),
        "reset_id": str(request.get("reset_id", "")),
        "reason": str(request.get("reason", "slice_boundary")),
        "slice_id": str(request.get("slice_id", "") or ""),
        "reset_transaction_id": int(request.get("transaction_id", 0) or 0),
        "state": str(state),
        "lifecycle_transaction_id": int(transaction_id or 0),
        "wall_time": time.time(),
    }
    payload.update(fields)
    return payload


def publish_reset_ack(publisher, node, request, state="IDLE", transaction_id=0, **fields):
    """Publish an acknowledgement without imposing a ROS message dependency."""
    from std_msgs.msg import String

    payload = reset_ack_payload(
        node,
        request,
        state=state,
        transaction_id=transaction_id,
        **fields
    )
    publisher.publish(String(data=json.dumps(payload, sort_keys=True)))
    return payload


__all__ = [
    "HARD_RESET_ACK_TOPIC",
    "HARD_RESET_RELEASE_TOPIC",
    "HARD_RESET_TOPIC",
    "EXPERIMENT_BOUNDARY_TOPIC",
    "decode_reset_request",
    "publish_reset_ack",
    "reset_ack_payload",
]
