"""Two-phase graph-action transaction for frontier route materialization.

The durable Place/Portal graph chooses an *intent* before the short-lived map
snapshot is converted into a navigation endpoint.  Candidate construction can
discover a more specific fact while it is doing that conversion (for example,
an executable Portal appears in a snapshot that previously only exposed local
WorkItems).  This module makes that refinement explicit:

``prepared intent -> materialized candidate -> committed intent``

An action cannot silently change class.  The only permitted promotion is a
local observation intent to a candidate that carries independent Portal or
probe evidence; the promotion is returned as a new immutable plan and must be
logged by the ROS adapter.  This keeps the graph planner the single writer for
the committed action while preserving the existing Navfn/TEB boundary.
"""

from dataclasses import dataclass, replace

from global_frontier_graph_route_planner import (
    ACTION_BOOTSTRAP,
    ACTION_CROSS_PORTAL,
    ACTION_HOLD,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    ACTION_REINSPECT_TARGET,
    ACTION_RETRY_VIEWPOINT,
    GraphRoutePlan,
    PLAN_READY,
)


_UNSET = object()


@dataclass(frozen=True)
class GraphRouteMaterialization:
    """Result of attempting to commit one map candidate to a graph intent."""

    accepted: bool
    plan: GraphRoutePlan
    candidate_action: str = ""
    candidate_work_item_id: object = None
    candidate_portal_id: object = None
    reconciled: bool = False
    reason: str = ""


@dataclass(frozen=True)
class GraphRouteActionTransaction:
    """Immutable lifecycle record for one graph-intent commit."""

    transaction_id: int
    plan: GraphRoutePlan
    map_epoch: object = None
    origin_epoch: object = None
    projection_epoch: object = None
    phase: str = "prepared"
    materialization: GraphRouteMaterialization = None

    @property
    def graph_transaction_id(self):
        """Expose the graph identity without colliding with mission IDs."""
        return int(self.transaction_id)

    def commit(self, materialization):
        """Return the committed transaction without mutating the old record."""
        if not materialization.accepted:
            return self
        return replace(
            self,
            plan=materialization.plan,
            phase="committed",
            materialization=materialization,
        )

    def as_dict(self):
        """Return a compact JSON-safe transaction report."""
        materialization = self.materialization
        return {
            "transaction_id": int(self.transaction_id),
            "graph_transaction_id": int(self.graph_transaction_id),
            "map_epoch": self.map_epoch,
            "origin_epoch": self.origin_epoch,
            "projection_epoch": self.projection_epoch,
            "phase": str(self.phase),
            "plan": self.plan.as_dict(),
            "materialized": (
                None
                if materialization is None
                else {
                    "accepted": bool(materialization.accepted),
                    "candidate_action": str(materialization.candidate_action),
                    "candidate_work_item_id": materialization.candidate_work_item_id,
                    "candidate_portal_id": materialization.candidate_portal_id,
                    "reconciled": bool(materialization.reconciled),
                    "reason": str(materialization.reason),
                }
            ),
        }


def _positive_id(value):
    """Normalize a durable identity without accepting a zero sentinel."""
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _route(candidate):
    """Read a legacy route tuple from either tuple or named candidate."""
    value = getattr(candidate, "route", candidate)
    return tuple(value or ())


def candidate_action(candidate):
    """Read the graph action carried by an existing route tuple."""
    route = _route(candidate)
    return None if len(route) <= 17 else route[17]


def with_graph_action(candidate, action, reason=None):
    """Return a route carrying the committed graph action.

    Route tuples predate the durable graph and are still consumed by the
    activation/controller boundary.  Once a graph transaction commits an
    action such as target reinspection, that semantic action must be visible
    in the tuple as well; otherwise status logs and lifecycle code observe the
    geometric refinement (usually ``observe_local_work``) instead of the
    action that actually owned the route.
    """
    route = list(_route(candidate))
    while len(route) <= 17:
        route.append(None)
    route[17] = action
    if reason is not None:
        while len(route) <= 18:
            route.append(None)
        route[18] = reason
    return tuple(route)


def candidate_reason(candidate):
    """Read the graph-executive reason carried by a route tuple."""
    route = _route(candidate)
    return "" if len(route) <= 18 else str(route[18] or "")


def candidate_work_item_id(candidate):
    """Read the local WorkItem identity from a route tuple."""
    route = _route(candidate)
    return None if len(route) <= 12 else _positive_id(route[12])


def candidate_place_id(candidate):
    """Read the reliable physical Place identity from a named candidate.

    A legacy route tuple intentionally has no Place field.  It can still
    represent the exact WorkItem selected by the plan, but it cannot prove
    that a *different* WorkItem belongs to the same physical Place.
    """
    return _positive_id(getattr(candidate, "place_id", None))


def candidate_portal_id(candidate):
    """Read a Portal identity appended by the portal selector.

    Probe candidates already carry their Portal probe identity at index 15.
    Forward crossing candidates use the optional trailing index 19.  Keeping
    both paths here lets older geometry-only tuples remain readable while the
    full graph method gains an auditable physical identity.
    """
    route = _route(candidate)
    if len(route) > 19:
        portal_id = _positive_id(route[19])
        if portal_id is not None:
            return portal_id
    if len(route) > 15 and route[15] is not None:
        return _positive_id(getattr(route[15], "probe_id", None))
    return None


def candidate_probe_phase(candidate):
    """Return the explicit phase carried by a PortalProbe candidate."""
    route = _route(candidate)
    if len(route) <= 15 or route[15] is None:
        return ""
    return str(getattr(route[15], "phase", "") or "").strip().lower()


def candidate_place_hops(candidate):
    """Read the route's snapshot-local number of Place boundary crossings."""
    route = _route(candidate)
    if len(route) <= 9:
        return None
    try:
        return int(route[9])
    except (TypeError, ValueError):
        return None


def candidate_route_kind(candidate):
    """Read the execution route kind without interpreting geometry."""
    route = _route(candidate)
    return "" if len(route) <= 10 else str(route[10] or "")


def _candidate_has_portal_proof(candidate):
    """Require the independent physical facts needed for local->crossing."""
    return (
        candidate_route_kind(candidate) == "portal_transition"
        and candidate_place_hops(candidate) is not None
        and candidate_place_hops(candidate) >= 1
        and len(_route(candidate)) > 11
        and _route(candidate)[11] is not None
        and candidate_portal_id(candidate) is not None
    )


def _accepted(
    plan,
    candidate,
    *,
    action=None,
    reason="",
    reconciled=False,
    obligation_kind=None,
    obligation_id=_UNSET,
    target_place_id=_UNSET,
    portal_path=None,
    first_portal_id=None,
):
    """Construct the immutable committed-plan result."""
    actual = candidate_action(candidate) if action is None else action
    updates = {"action": actual}
    if obligation_kind is not None:
        updates["obligation_kind"] = obligation_kind
    if obligation_id is not _UNSET:
        updates["obligation_id"] = obligation_id
    if target_place_id is not _UNSET:
        updates["target_place_id"] = target_place_id
    if portal_path is not None:
        updates["portal_path"] = tuple(portal_path)
    if first_portal_id is not None:
        updates["first_portal_id"] = first_portal_id
    # ``GraphRoutePlan.__post_init__`` derives evidence owner IDs from the
    # durable action fields. Reconciliation changes those fields explicitly;
    # retaining the old evidence tuple would report the predecessor WorkItem
    # as the owner of the newly committed candidate.
    if any(
        key in updates
        for key in (
            "action",
            "obligation_kind",
            "obligation_id",
            "target_place_id",
            "first_portal_id",
        )
    ):
        updates["required_evidence"] = ()
        updates["produces_evidence"] = ()
    committed = replace(plan, reason=reason or plan.reason, **updates)
    return GraphRouteMaterialization(
        accepted=True,
        plan=committed,
        candidate_action=str(actual or ""),
        candidate_work_item_id=candidate_work_item_id(candidate),
        candidate_portal_id=candidate_portal_id(candidate),
        reconciled=bool(reconciled),
        reason=reason or "plan_committed",
    )


def materialize_graph_route_action(plan, candidate, *, branch_first=False):
    """Commit one candidate only if it respects the prepared graph intent.

    ``observe_local_work -> retry_viewpoint`` is a refinement of the same
    WorkItem obligation.  ``observe_local_work -> probe_portal`` and
    ``observe_local_work -> cross_portal`` are explicit evidence promotions;
    the latter additionally requires branch-first policy and physical Portal
    facts.  No reverse promotion is accepted, so a selected Portal cannot be
    replaced by an arbitrary local frontier endpoint.
    """
    if not isinstance(plan, GraphRoutePlan) or plan.status != PLAN_READY:
        return GraphRouteMaterialization(
            accepted=False,
            plan=plan,
            candidate_action=str(candidate_action(candidate) or ""),
            candidate_work_item_id=candidate_work_item_id(candidate),
            candidate_portal_id=candidate_portal_id(candidate),
            reason="graph_plan_not_ready",
        )

    actual = candidate_action(candidate)
    work_item_id = candidate_work_item_id(candidate)
    portal_id = candidate_portal_id(candidate)
    reason = candidate_reason(candidate)
    common = {
        "candidate_action": str(actual or ""),
        "candidate_work_item_id": work_item_id,
        "candidate_portal_id": portal_id,
    }
    if actual is None:
        return GraphRouteMaterialization(
            accepted=False,
            plan=plan,
            reason="candidate_missing_graph_action",
            **common,
        )

    expected = str(plan.action or "")
    if expected == ACTION_HOLD:
        return GraphRouteMaterialization(
            accepted=False,
            plan=plan,
            reason="graph_plan_holds_for_evidence",
            **common,
        )

    if expected == ACTION_BOOTSTRAP:
        if actual not in (ACTION_BOOTSTRAP, ACTION_OBSERVE_LOCAL_WORK):
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="bootstrap_intent_replaced",
                **common,
            )
        planned_work_item = _positive_id(plan.obligation_id)
        candidate_place = candidate_place_id(candidate)
        planned_place_id = _positive_id(plan.current_place_id)
        if work_item_id is None:
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="bootstrap_work_item_identity_missing",
                **common,
            )
        if (
            planned_work_item is not None
            and work_item_id != planned_work_item
            and (
                candidate_place is None
                or planned_place_id is None
                or candidate_place != planned_place_id
            )
        ):
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="bootstrap_work_item_place_identity_missing",
                **common,
            )
        return _accepted(
            plan,
            candidate,
            action=ACTION_BOOTSTRAP,
            reason=reason or "bootstrap_observation_committed",
            reconciled=(
                actual != ACTION_BOOTSTRAP
                or (
                    planned_work_item is not None
                    and work_item_id != planned_work_item
                )
            ),
            obligation_kind="work_item",
            obligation_id=work_item_id,
            target_place_id=plan.current_place_id,
        )

    if expected == ACTION_REINSPECT_TARGET:
        # The target obligation is owned by the current physical Place, while
        # its safe endpoint may reuse an ordinary local WorkItem.  Keep the
        # target action as the committed graph intent and reject every route
        # that would turn reinspection into a Portal/probe departure.
        candidate_place = candidate_place_id(candidate)
        planned_place_id = _positive_id(plan.current_place_id)
        if (
            candidate_route_kind(candidate) != "frontier_endpoint"
            or candidate_place_hops(candidate) != 0
            or candidate_work_item_id(candidate) is None
            or candidate_portal_id(candidate) is not None
            or (
                candidate_place is not None
                and candidate_place != planned_place_id
            )
            or actual not in (
                ACTION_OBSERVE_LOCAL_WORK,
                ACTION_RETRY_VIEWPOINT,
                ACTION_REINSPECT_TARGET,
            )
        ):
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="target_reinspection_requires_local_viewpoint",
                **common,
            )
        return _accepted(
            plan,
            candidate,
            action=ACTION_REINSPECT_TARGET,
            reason=reason or "target_reinspection_committed",
            obligation_kind="target_observation",
            obligation_id=None,
            target_place_id=plan.current_place_id,
        )

    if expected in (ACTION_OBSERVE_LOCAL_WORK, ACTION_RETRY_VIEWPOINT):
        if actual in (ACTION_OBSERVE_LOCAL_WORK, ACTION_RETRY_VIEWPOINT):
            planned_work_item = _positive_id(plan.obligation_id)
            candidate_place = candidate_place_id(candidate)
            planned_place_id = _positive_id(plan.current_place_id)
            if (
                candidate_place is not None
                and planned_place_id is not None
                and candidate_place != planned_place_id
            ):
                return GraphRouteMaterialization(
                    accepted=False,
                    plan=plan,
                    reason="local_work_item_place_changed",
                    **common,
                )
            if planned_work_item is not None and work_item_id is None:
                return GraphRouteMaterialization(
                    accepted=False,
                    plan=plan,
                    reason="local_work_item_identity_missing",
                    **common,
                )
            if (
                planned_work_item is not None
                and work_item_id != planned_work_item
                and (candidate_place is None or planned_place_id is None)
            ):
                return GraphRouteMaterialization(
                    accepted=False,
                    plan=plan,
                    reason="local_work_item_place_identity_missing",
                    **common,
                )
            committed_action = actual
            return _accepted(
                plan,
                candidate,
                action=committed_action,
                reason=(
                    reason
                    or (
                        "local_work_item_reconciled"
                        if planned_work_item is not None
                        and work_item_id != planned_work_item
                        else "local_observation_committed"
                    )
                ),
                reconciled=(
                    committed_action != expected
                    or (
                        planned_work_item is not None
                        and work_item_id != planned_work_item
                    )
                ),
                obligation_kind="work_item" if work_item_id is not None else None,
                obligation_id=work_item_id,
                target_place_id=plan.current_place_id,
            )
        if actual == ACTION_PROBE_PORTAL and portal_id is not None:
            return _accepted(
                plan,
                candidate,
                reason="local_intent_promoted_to_portal_probe",
                reconciled=True,
                obligation_kind="portal_probe",
                obligation_id=portal_id,
                portal_path=(portal_id,),
                first_portal_id=portal_id,
                target_place_id=None,
            )
        if actual == ACTION_CROSS_PORTAL:
            if not branch_first:
                return GraphRouteMaterialization(
                    accepted=False,
                    plan=plan,
                    reason="crossing_promotion_requires_branch_first",
                    **common,
                )
            if not _candidate_has_portal_proof(candidate):
                return GraphRouteMaterialization(
                    accepted=False,
                    plan=plan,
                    reason="crossing_promotion_missing_portal_proof",
                    **common,
                )
            return _accepted(
                plan,
                candidate,
                reason=reason or "certified_portal_materialized",
                reconciled=True,
                obligation_kind="portal_edge",
                obligation_id=portal_id,
                portal_path=(portal_id,),
                first_portal_id=portal_id,
                target_place_id=None,
            )
        return GraphRouteMaterialization(
            accepted=False,
            plan=plan,
            reason="local_intent_replaced_by_illegal_action",
            **common,
        )

    if expected == ACTION_PROBE_PORTAL:
        if actual != ACTION_PROBE_PORTAL:
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="portal_probe_intent_replaced",
                **common,
            )
        if (
            plan.first_portal_id is not None
            and portal_id is not None
            and portal_id != _positive_id(plan.first_portal_id)
        ):
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="portal_probe_identity_changed",
                **common,
            )
        planned_phase = str(
            getattr(plan, "portal_probe_phase", "") or ""
        ).strip().lower()
        actual_phase = candidate_probe_phase(candidate)
        if planned_phase and actual_phase and planned_phase != actual_phase:
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="portal_probe_phase_changed",
                **common,
            )
        return _accepted(
            plan,
            candidate,
            reason=reason or "portal_probe_committed",
            obligation_kind=plan.obligation_kind or "portal_probe",
            obligation_id=plan.obligation_id or portal_id,
            portal_path=plan.portal_path or ((portal_id,) if portal_id else ()),
            first_portal_id=plan.first_portal_id or portal_id,
        )

    if expected == ACTION_CROSS_PORTAL:
        if actual != ACTION_CROSS_PORTAL:
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="portal_crossing_intent_replaced",
                **common,
            )
        if not _candidate_has_portal_proof(candidate):
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="portal_crossing_requires_physical_portal_route",
                **common,
            )
        expected_portal = _positive_id(plan.first_portal_id)
        if (
            expected_portal is not None
            and portal_id is not None
            and portal_id != expected_portal
        ):
            return GraphRouteMaterialization(
                accepted=False,
                plan=plan,
                reason="portal_crossing_identity_changed",
                **common,
            )
        return _accepted(
            plan,
            candidate,
            reason=reason or "portal_crossing_committed",
            obligation_kind=plan.obligation_kind or "portal_edge",
            obligation_id=plan.obligation_id,
            portal_path=plan.portal_path or ((portal_id,) if portal_id else ()),
            first_portal_id=plan.first_portal_id or portal_id,
        )

    return GraphRouteMaterialization(
        accepted=False,
        plan=plan,
        reason="unknown_graph_intent",
        **common,
    )


__all__ = [
    "GraphRouteActionTransaction",
    "GraphRouteMaterialization",
    "candidate_action",
    "candidate_portal_id",
    "candidate_place_id",
    "candidate_probe_phase",
    "candidate_reason",
    "candidate_work_item_id",
    "materialize_graph_route_action",
    "with_graph_action",
]
