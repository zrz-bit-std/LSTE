"""Pure durable-action lease reconciliation.

The map planner and the semantic goal manager can request a new decision while
an older route is still in flight.  Clearing only the route's transient map
fields leaves its durable WorkItem or Portal probe in ``active`` state.  The
next graph pass then sees a permanent lock and every replacement candidate is
rejected.  This module makes the preemption decision explicit and keeps it
independent from ROS callbacks.
"""

from dataclasses import dataclass


LEASE_NOOP = "noop"
LEASE_PREEMPT = "preempt"
LEASE_DEFER = "defer"


@dataclass(frozen=True)
class DurableActionLease:
    """Snapshot of the durable identities owned by one active route."""

    route_id: int = 0
    route_kind: str = ""
    work_item_id: object = None
    work_item_attempt_id: object = None
    portal_probe_id: object = None
    portal_crossing_observed: bool = False
    portal_transaction_active: bool = False

    @property
    def active(self):
        return int(self.route_id or 0) > 0 and bool(
            self.work_item_id is not None
            or self.portal_probe_id is not None
            or self.portal_transaction_active
        )


@dataclass(frozen=True)
class LeaseReplanDecision:
    """Result of reconciling a route lease with a replan request."""

    action: str
    reason: str
    route_id: int = 0
    obligations: tuple = ()


def decide_replan_lease(lease, requested_reason="replan"):
    """Choose whether a replan may preempt the current durable action.

    A route that has physically crossed a Portal, or is still owned by the
    Portal transaction, must finish its arrival commit before another request
    can run.  All other active durable actions are explicitly preemptible: the
    caller must settle their Attempts as failed viewpoints before replacing
    the route.  No timeout or distance threshold participates in this choice.
    """
    if not isinstance(lease, DurableActionLease) or not lease.active:
        return LeaseReplanDecision(LEASE_NOOP, "no_durable_route_lease")
    route_id = int(lease.route_id)
    if lease.portal_transaction_active or lease.portal_crossing_observed:
        return LeaseReplanDecision(
            LEASE_DEFER,
            "portal_arrival_commit_owns_route",
            route_id=route_id,
            obligations=_obligations(lease),
        )
    reason = str(requested_reason or "replan").strip() or "replan"
    return LeaseReplanDecision(
        LEASE_PREEMPT,
        "durable_route_preempted_by_%s" % reason,
        route_id=route_id,
        obligations=_obligations(lease),
    )


def _obligations(lease):
    obligations = []
    if lease.portal_probe_id is not None:
        obligations.append(("portal_probe", lease.portal_probe_id))
    if lease.work_item_id is not None:
        obligations.append(("work_item", lease.work_item_id))
    return tuple(obligations)


__all__ = [
    "DurableActionLease",
    "LEASE_DEFER",
    "LEASE_NOOP",
    "LEASE_PREEMPT",
    "LeaseReplanDecision",
    "decide_replan_lease",
]
