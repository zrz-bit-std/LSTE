"""ROS boundary for atomic durable-route lease reconciliation."""

from global_frontier_durable_action_lease import (
    DurableActionLease,
    LEASE_DEFER,
    LEASE_PREEMPT,
    decide_replan_lease,
)


class GlobalFrontierDurableLeaseLifecycleMixin:
    """End durable Attempts before a route identity is replaced."""

    def active_durable_action_lease(self):
        """Capture the current route's durable ownership before clearing it."""
        return DurableActionLease(
            route_id=int(getattr(self, "active_route_id", 0) or 0),
            route_kind=str(getattr(self, "active_route_kind", "") or ""),
            work_item_id=getattr(self, "active_work_item_id", None),
            work_item_attempt_id=getattr(
                self, "active_work_item_attempt_id", None
            ),
            portal_probe_id=getattr(self, "active_portal_probe_id", None),
            portal_crossing_observed=bool(
                getattr(self, "active_portal_crossing_observed", False)
            ),
            portal_transaction_active=bool(
                getattr(
                    getattr(self, "portal_transaction", None), "active", False
                )
            ),
        )

    def reconcile_active_durable_lease_for_replan(self, reason, now):
        """Atomically preempt or defer the current durable route lease.

        The method is called before the generic replan reset clears active
        route fields.  A preempted Portal probe and WorkItem remain unresolved,
        but their active Attempts are closed, so the next graph snapshot can
        choose the same obligation from a different physical viewpoint.
        """
        lease = self.active_durable_action_lease()
        decision = decide_replan_lease(lease, reason)
        if decision.action == LEASE_DEFER:
            self.publish_status(
                "durable_route_lease_deferred",
                route_id=int(decision.route_id),
                reason=str(decision.reason),
                obligations=[list(item) for item in decision.obligations],
            )
            return decision
        if decision.action != LEASE_PREEMPT:
            return decision

        settled = []
        probe_id = lease.portal_probe_id
        settle_probe = getattr(self, "settle_active_portal_probe", None)
        if probe_id is not None and callable(settle_probe):
            record = settle_probe(
                "failed", now, "preempted_by_%s" % (reason or "replan")
            )
            settled.append(
                {
                    "kind": "portal_probe",
                    "id": int(probe_id),
                    "state": None if record is None else record.get("state"),
                }
            )

        work_item_id = lease.work_item_id
        settle_work = getattr(self, "settle_active_work_item", None)
        if work_item_id is not None and callable(settle_work):
            item = settle_work(
                now,
                "deferred",
                "preempted_by_%s" % (reason or "replan"),
            )
            settled.append(
                {
                    "kind": "work_item",
                    "id": int(work_item_id),
                    "state": None if item is None else item.get("state"),
                }
            )

        self.publish_status(
            "durable_route_lease_reconciled",
            route_id=int(decision.route_id),
            route_kind=str(lease.route_kind),
            reason=str(decision.reason),
            settled=settled,
        )
        return decision


__all__ = ["GlobalFrontierDurableLeaseLifecycleMixin"]
