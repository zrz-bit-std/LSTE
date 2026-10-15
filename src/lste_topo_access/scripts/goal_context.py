#!/usr/bin/env python3
"""Stable ownership identity for a map-frame navigation command.

The geometry sent to Navfn and TEB is intentionally short lived: online SLAM
may move an endpoint during a valid action.  This small value object carries
the durable owner of that geometry so a consumer can distinguish a replan
from a different Place, WorkItem, or certified doorway action.
"""

import hashlib
import json


GOAL_CONTEXT_SCHEMA_VERSION = 2
GOAL_CONTEXT_ROLES = frozenset((
    "geometry_frontier",
    "semantic_target",
    "place_observation",
    "place_observation_work",
    "portal_probe",
    "certified_portal_crossing",
    "covered_place_transit",
    "place_egress",
))


def _positive_id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _portal_gate(value):
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        return [round(float(value[0]), 4), round(float(value[1]), 4)]
    except (TypeError, ValueError):
        return None


def _stable_text(value):
    """Normalize a scalar task field without leaking arbitrary wire values."""
    return str(value or "").strip()


def task_version_from_task(task):
    """Return a deterministic version for one semantic ``LsteTask`` payload.

    ``task_id`` names a task family, but it is not sufficient to distinguish
    two missions that reuse that name with different attributes or context.
    Hashing the semantic fields keeps the public ROS message unchanged while
    giving the mission layer an immutable identity. JSON is canonicalized when
    available so formatting/order do not create a spurious new mission.
    """
    if task is None:
        return ""
    fields = {
        "task_id": _stable_text(getattr(task, "task_id", "")),
        "target_name": _stable_text(getattr(task, "target_name", "")),
        "target_attributes": [
            _stable_text(value)
            for value in list(getattr(task, "target_attributes", []) or [])
        ],
        "env_related_structures": [
            _stable_text(value)
            for value in list(getattr(task, "env_related_structures", []) or [])
        ],
        "env_type_prior": [
            _stable_text(value)
            for value in list(getattr(task, "env_type_prior", []) or [])
        ],
        "obj_key_objects": [
            _stable_text(value)
            for value in list(getattr(task, "obj_key_objects", []) or [])
        ],
        "obj_negative_clues": [
            _stable_text(value)
            for value in list(getattr(task, "obj_negative_clues", []) or [])
        ],
        "ctx_left": _stable_text(getattr(task, "ctx_left", "")),
        "ctx_right": _stable_text(getattr(task, "ctx_right", "")),
    }
    raw_json = _stable_text(getattr(task, "raw_json", ""))
    if raw_json:
        try:
            fields["raw_json"] = json.loads(raw_json)
        except (TypeError, ValueError, json.JSONDecodeError):
            # Preserve non-JSON task provenance without making parsing a
            # prerequisite for mission identity.
            fields["raw_json"] = raw_json
    encoded = json.dumps(
        fields,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:16]


def default_goal_context(
    exploration_method="legacy",
    task_id="",
    mission_id="",
    task_version="",
    graph_transaction_id=0,
):
    """Return the complete, JSON-safe context for a geometry-only action."""
    return {
        "schema_version": GOAL_CONTEXT_SCHEMA_VERSION,
        "exploration_method": str(exploration_method or "legacy").strip().lower()
        or "legacy",
        # ``task_id`` names the semantic task family. ``mission_id`` and
        # ``task_version`` distinguish a new mission from a reissued geometry
        # command or a reused task label.
        "task_id": _stable_text(task_id),
        "mission_id": _stable_text(mission_id) or _stable_text(task_id),
        "task_version": _stable_text(task_version),
        # Graph planning and mission command allocation are independent
        # identity domains. Zero denotes a direct semantic route.
        "graph_transaction_id": _positive_id(graph_transaction_id) or 0,
        "goal_role": "geometry_frontier",
        "owner_place_id": None,
        "source_place_id": None,
        "work_item_id": None,
        "portal_probe_id": None,
        "portal_probe_phase": "",
        "portal_crossing_certified": False,
        "portal_gate_odom": None,
    }


def normalize_goal_context(value):
    """Normalize untrusted wire JSON without changing its semantic identity."""
    if not isinstance(value, dict):
        return default_goal_context()
    context = default_goal_context(
        value.get("exploration_method", "legacy"),
        value.get("task_id", ""),
        value.get("mission_id", ""),
        value.get("task_version", ""),
        value.get("graph_transaction_id", 0),
    )
    role = str(value.get("goal_role", "")).strip().lower()
    if role not in GOAL_CONTEXT_ROLES:
        return context
    context["goal_role"] = role
    for key in (
        "owner_place_id",
        "source_place_id",
        "work_item_id",
        "portal_probe_id",
    ):
        context[key] = _positive_id(value.get(key))
    context["graph_transaction_id"] = (
        _positive_id(value.get("graph_transaction_id")) or 0
    )
    context["portal_probe_phase"] = str(
        value.get("portal_probe_phase", "") or ""
    ).strip().lower()
    if context["portal_probe_phase"] not in ("", "source", "destination"):
        context["portal_probe_phase"] = ""
    context["portal_crossing_certified"] = bool(
        value.get("portal_crossing_certified", False)
    )
    context["portal_gate_odom"] = _portal_gate(value.get("portal_gate_odom"))
    return context


def goal_context_identity(value):
    """Return the durable identity that must survive map-coordinate updates."""
    context = normalize_goal_context(value)
    gate = context["portal_gate_odom"]
    return (
        context["exploration_method"],
        context["task_id"],
        context["mission_id"],
        context["task_version"],
        context["graph_transaction_id"],
        context["goal_role"],
        context["owner_place_id"],
        context["source_place_id"],
        context["work_item_id"],
        context["portal_probe_id"],
        context["portal_probe_phase"],
        context["portal_crossing_certified"],
        None if gate is None else (gate[0], gate[1]),
    )
