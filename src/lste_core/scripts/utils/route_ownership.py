#!/usr/bin/env python3
"""Ownership classes for controller route commands.

Two ownership domains share the controller wire protocol:

``graph``
    An action owned by the durable place/portal graph: a portal probe, a
    portal crossing, or a durable work-item graph action.  It must present a
    positive ``graph_transaction_id`` and it participates in the graph
    high-water mark.

``observation``
    Geometry/place observation work: a GP work item or a plain frontier
    endpoint.  It owns no graph transaction.  Its ownership token is the
    route/lifecycle identity plus, when present, ``work_item_id`` and
    ``attempt_id``.  A zero ``graph_transaction_id`` is legal and the route
    must never advance the graph high-water mark.

Unknown payloads fail closed as ``graph``: a producer that has not adopted
the ownership field cannot silently bypass the strict graph contract.
"""

GRAPH = "graph"
OBSERVATION = "observation"
VALID = (GRAPH, OBSERVATION)

GRAPH_GOAL_ROLES = frozenset({
    "portal_probe",
    "certified_portal_crossing",
    "portal_crossing",
    "portal_authentication",
    "work_item_graph_action",
})

OBSERVATION_GOAL_ROLES = frozenset({
    "place_observation_work",
    "place_observation",
    "place_egress",
    "covered_place_transit",
    "geometry_frontier",
    "bootstrap_observation",
})

GRAPH_ACTIONS = frozenset({
    "probe_portal",
    "cross_portal",
    "portal_crossing",
})


def normalize_ownership(value):
    text = str(value or "").strip().lower()
    return text if text in VALID else ""


def _int(value):
    try:
        return max(0, int(value or 0))
    except (TypeError, ValueError):
        return 0


def _goal_context(payload):
    context = payload.get("goal_context")
    return context if isinstance(context, dict) else {}


def classify_route_ownership(payload):
    """Return ``graph`` or ``observation`` for one route payload.

    An explicit ``route_ownership`` wins.  Otherwise the goal role decides,
    and only then the presence of a positive graph transaction.  The default
    is ``graph`` (fail closed).
    """
    if not isinstance(payload, dict):
        return GRAPH
    context = _goal_context(payload)
    explicit = normalize_ownership(payload.get("route_ownership"))
    if not explicit:
        explicit = normalize_ownership(context.get("route_ownership"))
    if explicit:
        return explicit
    role = str(
        context.get("goal_role") or payload.get("goal_role") or ""
    ).strip().lower()
    if role in OBSERVATION_GOAL_ROLES:
        return OBSERVATION
    if role in GRAPH_GOAL_ROLES:
        return GRAPH
    action = str(
        payload.get("graph_action") or context.get("graph_action") or ""
    ).strip().lower()
    if action in GRAPH_ACTIONS:
        return GRAPH
    if _int(payload.get("graph_transaction_id")) > 0:
        return GRAPH
    return GRAPH


def observation_identity(payload):
    """Return ``(work_item_id, attempt_id)`` for an observation route."""
    context = _goal_context(payload)
    work_item_id = _int(payload.get("work_item_id"))
    if work_item_id <= 0:
        work_item_id = _int(context.get("work_item_id"))
    attempt_id = _int(payload.get("attempt_id"))
    if attempt_id <= 0:
        attempt_id = _int(context.get("attempt_id"))
    return work_item_id, attempt_id


def observation_identity_error(payload):
    """Return a reason when an observation route lacks its owner token.

    A work item must carry the attempt that owns it.  A plain geometry
    frontier has no work item; its route/lifecycle identity is the owner and
    the epoch checks still apply downstream.
    """
    work_item_id, attempt_id = observation_identity(payload)
    if work_item_id > 0 and attempt_id <= 0:
        return "missing_observation_attempt_id"
    return ""


def graph_route_admission_error(payload, route_id=0, source=""):
    """Return the admission reason for a frontier route, or ``""``.

    Only routes that claim the global-frontier source and a positive route id
    are subject to the ownership contract.  Graph-owned routes must present a
    positive graph transaction; observation routes must present their work
    item attempt when they carry a work item.  A plain geometry frontier has
    no work item and relies on the route/lifecycle identity.
    """
    normalized_source = str(source or "").strip().lower()
    if normalized_source != "global_slam_frontier":
        return ""
    try:
        route_id = int(route_id or 0)
    except (TypeError, ValueError):
        route_id = 0
    if route_id <= 0:
        return ""
    ownership = classify_route_ownership(payload)
    if ownership == GRAPH:
        if _int(payload.get("graph_transaction_id")) <= 0:
            return "missing_graph_transaction_id"
        return ""
    return observation_identity_error(payload)
