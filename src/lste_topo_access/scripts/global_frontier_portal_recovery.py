#!/usr/bin/env python3
"""One-recovery execution contract for an online structural portal edge."""

from global_frontier_models import PortalTransitionRetry


class GlobalFrontierPortalRecoveryMixin:
    """Retry a failed doorway only after egress through reached free space."""

    def prepare_portal_transition_retry(self, reason):
        """Reserve the current portal edge for one post-egress retry.

        A second failure of the retried edge is final for this traversal.  The
        selector may then inspect another graph edge, but it must not cycle
        through arbitrary goals while still trapped at the same doorway.
        """
        if (
            self.active_route_kind != "portal_transition"
            or self.active_frontier is None
            or getattr(self, "active_portal_crossing_observed", False)
            or getattr(self, "active_portal_crossing_rejected", False)
            or self.active_portal_retry
            or self.pending_portal_retry is not None
        ):
            return None
        retry = PortalTransitionRetry(
            failed_route_id=int(self.active_route_id),
            source_region_id=self.place_departure.region_id,
            goal_xy=(
                float(self.active_frontier[2]),
                float(self.active_frontier[3]),
            ),
            failure_reason=str(reason),
            portal_gate_xy=getattr(self, "active_portal_gate_xy", None),
        )
        self.pending_portal_retry = retry
        self.publish_status(
            "portal_transition_recovery_prepared",
            failed_route_id=retry.failed_route_id,
            source_region_id=retry.source_region_id,
            portal_goal=[
                round(retry.goal_xy[0], 3),
                round(retry.goal_xy[1], 3),
            ],
            portal_gate=(
                None if retry.portal_gate_xy is None else [
                    round(float(retry.portal_gate_xy[0]), 3),
                    round(float(retry.portal_gate_xy[1]), 3),
                ]
            ),
            reason=retry.failure_reason,
        )
        return retry

    def discard_pending_portal_retry(self, reason):
        """Close a failed portal-recovery transaction without another retry."""
        retry = self.pending_portal_retry
        if retry is None:
            return None
        self.pending_portal_retry = None
        transaction = getattr(self, "portal_transaction", None)
        if transaction is not None and transaction.active:
            transition = transaction.abort(
                "portal_retry_abandoned:%s" % str(reason)
            )
            if transition is not None:
                self.publish_status(
                    "portal_transaction_aborted",
                    transaction_id=int(transition.transaction_id),
                    route_id=int(transition.route_id),
                    state=transition.state,
                    reason=transition.last_reason,
                )
                transaction.finish("portal_transaction_aborted")
        self.publish_status(
            "portal_transition_recovery_abandoned",
            failed_route_id=int(retry.failed_route_id),
            source_region_id=retry.source_region_id,
            portal_goal=[
                round(retry.goal_xy[0], 3),
                round(retry.goal_xy[1], 3),
            ],
            portal_gate=(
                None if retry.portal_gate_xy is None else [
                    round(float(retry.portal_gate_xy[0]), 3),
                    round(float(retry.portal_gate_xy[1]), 3),
                ]
            ),
            reason=str(reason),
        )
        return retry

    def select_pending_portal_retry(self, snapshot):
        """Revalidate the reserved edge before sending its one retry action."""
        retry = self.pending_portal_retry
        if retry is None:
            return None, None, False
        route_graph = snapshot.route_graph
        x, y = retry.goal_xy
        reassociated = self.nearest_reachable_cell(
            snapshot.message, route_graph.route_steps, x, y,
        )
        costmap_distance = self.candidate_costmap_distance(
            route_graph.validation, x, y,
        )
        reachable = self.navfn_goal_reachable(
            snapshot.robot_map,
            (x, y),
            snapshot.message.header.frame_id or "map",
        )
        if reachable is None:
            return None, None, True
        if reassociated is None or reachable is False or (
            route_graph.validation is not None and costmap_distance is None
        ):
            self.discard_pending_portal_retry("post_egress_edge_not_reachable")
            return None, None, False
        row, col = reassociated
        path_distance = (
            float(costmap_distance)
            if costmap_distance is not None
            else float(route_graph.route_steps[row, col])
            * float(snapshot.message.info.resolution)
        )
        component = self.component_at_map_position(
            snapshot.message,
            snapshot.map_context.components,
            snapshot.map_context.known_free,
            x,
            y,
        )
        self.pending_portal_retry = None
        self.publish_status(
            "portal_transition_recovery_selected",
            failed_route_id=int(retry.failed_route_id),
            source_region_id=retry.source_region_id,
            portal_goal=[round(x, 3), round(y, 3)],
            portal_gate=(
                None if retry.portal_gate_xy is None else [
                    round(float(retry.portal_gate_xy[0]), 3),
                    round(float(retry.portal_gate_xy[1]), 3),
                ]
            ),
            path_distance=round(path_distance, 3),
        )
        return (
            row,
            col,
            float(x),
            float(y),
            path_distance,
            0.0,
            0.0,
            0.0,
            component,
            1,
            "portal_transition",
            retry.portal_gate_xy,
        ), "portal_recovery_retry", False
