"""Pure admission rules for a durable graph-route decision.

The durable graph planner selects an obligation after the current map has
rehydrated its Place/Portal/WorkItem identities.  This module makes that
decision an action contract: a candidate either carries the selected durable
identity or it is not allowed to replace the plan.  It intentionally contains
no ROS, geometry, score, timer, or controller policy.
"""

from global_frontier_graph_route_planner import (
    ACTION_BOOTSTRAP,
    ACTION_CROSS_PORTAL,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    ACTION_REINSPECT_TARGET,
    ACTION_RETRY_VIEWPOINT,
    PLAN_READY,
)


def _positive_id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _route(candidate):
    """Return the legacy route tuple from either candidate representation."""
    return getattr(candidate, "route", candidate)


def _probe_id(route):
    probe = route[15] if route is not None and len(route) > 15 else None
    return _positive_id(getattr(probe, "probe_id", None))


def _candidate_place_id(candidate):
    """Return the explicit Place identity carried by a named candidate."""
    return _positive_id(getattr(candidate, "place_id", None))


def graph_plan_is_exclusive(plan):
    """Whether a ready graph plan owns the next discrete action."""
    return bool(
        plan is not None
        and getattr(plan, "status", None) == PLAN_READY
        and getattr(plan, "action", None)
        in (
            ACTION_BOOTSTRAP,
            ACTION_OBSERVE_LOCAL_WORK,
            ACTION_REINSPECT_TARGET,
            ACTION_RETRY_VIEWPOINT,
            ACTION_PROBE_PORTAL,
            ACTION_CROSS_PORTAL,
        )
    )


def candidate_matches_graph_plan(plan, candidate):
    """Check durable identity compatibility without comparing map coordinates.

    Local work is matched by WorkItem ID.  A source-side probe is matched by
    its PortalProbe ID.  A cross-place action is matched by the explicit
    portal-transition route; the portal ID itself is admitted by the portal
    selector's existing durable preference and certification checks.
    """
    if not graph_plan_is_exclusive(plan):
        return True
    route = _route(candidate)
    if route is None:
        return False
    action = getattr(plan, "action", None)
    obligation_kind = str(getattr(plan, "obligation_kind", "") or "")
    obligation_id = _positive_id(getattr(plan, "obligation_id", None))
    if action in (ACTION_BOOTSTRAP, ACTION_OBSERVE_LOCAL_WORK, ACTION_RETRY_VIEWPOINT):
        if obligation_id is None or len(route) <= 12:
            return False
        candidate_work_item_id = _positive_id(route[12])
        if candidate_work_item_id != obligation_id:
            return False
        planned_place_id = _positive_id(getattr(plan, "current_place_id", None))
        candidate_place_id = _candidate_place_id(candidate)
        if (
            candidate_place_id is not None
            and planned_place_id is not None
            and candidate_place_id != planned_place_id
        ):
            return False
        route_action = route[17] if len(route) > 17 else None
        if action == ACTION_BOOTSTRAP:
            return route_action in (None, ACTION_BOOTSTRAP)
        return route_action in (None, ACTION_OBSERVE_LOCAL_WORK, ACTION_RETRY_VIEWPOINT)
    if action == ACTION_REINSPECT_TARGET:
        # Target reinspection is owned by the current Place.  It may use a
        # generic Place WorkItem as its safe geometric viewpoint, but it may
        # never become a Portal transition or a doorway probe while the target
        # evidence lease is unresolved.
        if len(route) <= 17 or len(route) <= 12:
            return False
        try:
            # Reinspection is a same-Place action. A missing hop fact is not
            # equivalent to zero: accepting it would let a geometry-only
            # candidate bypass the physical Portal boundary.
            local_hops = int(route[9]) == 0
        except (TypeError, ValueError):
            local_hops = False
        route_kind = str(route[10] or "") if len(route) > 10 else ""
        route_action = route[17]
        candidate_place_id = _positive_id(getattr(candidate, "place_id", None))
        planned_place_id = _positive_id(getattr(plan, "current_place_id", None))
        return bool(
            local_hops
            and route_kind == "frontier_endpoint"
            and _positive_id(route[12]) is not None
            and route[15] is None
            and (
                candidate_place_id is None
                or candidate_place_id == planned_place_id
            )
            and route_action in (
                None,
                ACTION_OBSERVE_LOCAL_WORK,
                ACTION_RETRY_VIEWPOINT,
                ACTION_REINSPECT_TARGET,
            )
        )
    if action == ACTION_PROBE_PORTAL:
        if obligation_kind == "portal_probe":
            return obligation_id is not None and _probe_id(route) == obligation_id
        # An unbound Portal has no probe ledger ID yet.  It is materialized as
        # the same portal-transition route selected by the durable portal ID.
        return len(route) > 10 and route[10] == "portal_transition"
    if action == ACTION_CROSS_PORTAL:
        if not (len(route) > 10 and route[10] == "portal_transition"):
            return False
        planned_portal_id = _positive_id(
            getattr(plan, "first_portal_id", None)
        )
        if planned_portal_id is None or len(route) <= 19:
            return True
        selected_portal_id = _positive_id(route[19])
        return selected_portal_id is None or selected_portal_id == planned_portal_id
    return False


def candidate_refines_graph_plan(plan, candidate):
    """Allow an explicit same-place refinement before transaction commit.

    The durable planner may name the first pending WorkItem even when the
    current SLAM snapshot has not rehydrated that item's frontier arc.  A
    different unresolved item in the same Place is still the same *action
    class*, and the transaction layer will replace the plan's obligation ID
    explicitly.  The two outward promotions are likewise admitted only as
    typed route facts; physical proof is checked again at commit time.
    """
    if candidate_matches_graph_plan(plan, candidate):
        return True
    if not graph_plan_is_exclusive(plan):
        return True
    route = _route(candidate)
    if route is None or len(route) <= 17:
        return False
    action = getattr(plan, "action", None)
    actual = route[17]
    if action == ACTION_BOOTSTRAP and actual in (
        ACTION_BOOTSTRAP,
        ACTION_OBSERVE_LOCAL_WORK,
    ):
        candidate_work_item_id = _positive_id(
            route[12] if len(route) > 12 else None
        )
        planned_work_item_id = _positive_id(
            getattr(plan, "obligation_id", None)
        )
        if candidate_work_item_id is None:
            return False
        if actual == ACTION_BOOTSTRAP:
            return True
        # WorkItem rehydration is the concrete map projection of the first
        # bootstrap observation. It may carry the same durable identity, or a
        # different item explicitly named as belonging to the same Place.
        if candidate_work_item_id == planned_work_item_id:
            return True
        candidate_place_id = _candidate_place_id(candidate)
        planned_place_id = _positive_id(getattr(plan, "current_place_id", None))
        return bool(
            candidate_place_id is not None
            and planned_place_id is not None
            and candidate_place_id == planned_place_id
        )
    if action in (ACTION_OBSERVE_LOCAL_WORK, ACTION_RETRY_VIEWPOINT):
        if actual in (ACTION_OBSERVE_LOCAL_WORK, ACTION_RETRY_VIEWPOINT):
            candidate_work_item_id = _positive_id(
                route[12] if len(route) > 12 else None
            )
            candidate_place_id = _candidate_place_id(candidate)
            planned_place_id = _positive_id(
                getattr(plan, "current_place_id", None)
            )
            # A different WorkItem is an allowed reconciliation only when the
            # candidate explicitly proves that it belongs to the planned
            # physical Place. A legacy tuple without that identity cannot
            # safely cross this boundary.
            if candidate_work_item_id is None:
                return False
            if candidate_work_item_id == _positive_id(
                getattr(plan, "obligation_id", None)
            ):
                return True
            return bool(
                candidate_place_id is not None
                and planned_place_id is not None
                and candidate_place_id == planned_place_id
            )
        if actual == ACTION_PROBE_PORTAL:
            return _probe_id(route) is not None
        if actual == ACTION_CROSS_PORTAL:
            return (
                len(route) > 10
                and route[10] == "portal_transition"
                and len(route) > 11
                and route[11] is not None
            )
    if action == ACTION_REINSPECT_TARGET:
        return candidate_matches_graph_plan(plan, candidate)
    return False


__all__ = [
    "candidate_refines_graph_plan",
    "candidate_matches_graph_plan",
    "graph_plan_is_exclusive",
]
