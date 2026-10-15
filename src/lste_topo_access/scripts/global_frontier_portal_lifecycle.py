"""Portal-arrival lifecycle composition for global-frontier exploration."""

from clock_provider import now_for
from global_frontier_models import PendingPortalArrival
from global_frontier_portal_arrival_commit import (
    GlobalFrontierPortalArrivalCommitMixin,
)
from global_frontier_portal_arrival_resolution import (
    GlobalFrontierPortalArrivalResolutionMixin,
)
from global_frontier_portal_crossing_lifecycle import (
    GlobalFrontierPortalCrossingLifecycleMixin,
)
from global_frontier_portal_live_crossing import (
    GlobalFrontierPortalLiveCrossingMixin,
)
from global_frontier_portal_recovery import GlobalFrontierPortalRecoveryMixin


class GlobalFrontierPortalLifecycleMixin(
    GlobalFrontierPortalLiveCrossingMixin,
    GlobalFrontierPortalArrivalCommitMixin,
    GlobalFrontierPortalArrivalResolutionMixin,
    GlobalFrontierPortalCrossingLifecycleMixin,
    GlobalFrontierPortalRecoveryMixin,
):
    """Turn verified portal terminals into durable destination-place memory."""

    def record_portal_place_arrival(
        self, completed, now=None, evidence_source="route_terminal",
        retry_on_unconfirmed=True,
    ):
        """Commit now when possible, otherwise preserve the verified arrival."""
        if completed is None or self.active_route_kind != "portal_transition":
            return None
        if (
            getattr(self, "active_portal_crossing_observed", False)
            or getattr(self, "active_portal_crossing_rejected", False)
        ):
            return None
        if now is None:
            now = now_for(self)
        crossing = self.confirm_active_portal_crossing()
        if not crossing.verified:
            if retry_on_unconfirmed:
                retry = self.prepare_portal_transition_retry(crossing.reason)
                self.publish_status(
                    "portal_crossing_unconfirmed",
                    route_id=int(self.active_route_id),
                    reason=crossing.reason,
                    source_signed_distance=(
                        None if crossing.source_signed_distance is None
                        else round(float(crossing.source_signed_distance), 3)
                    ),
                    terminal_signed_distance=(
                        None if crossing.terminal_signed_distance is None
                        else round(float(crossing.terminal_signed_distance), 3)
                    ),
                    required_destination_depth=round(
                        float(crossing.required_destination_depth), 3,
                    ),
                    retry_prepared=retry is not None,
                )
            return None
        # This fact belongs to the physical edge, not to the eventual local
        # controller terminal.  It suppresses a later same-edge retry even
        # when map association must wait for one fresh scan.
        self.active_portal_crossing_observed = True
        arrival = PendingPortalArrival(
            route_id=int(self.active_route_id),
            goal_xy=(float(completed[2]), float(completed[3])),
            terminal_time=float(now),
            source_region_id=self.place_departure.region_id,
            portal_gate_xy=getattr(self, "active_portal_gate_xy", None),
            physical_goal_xy=(
                None
                if getattr(self, "pose_odom", None) is None
                else (float(self.pose_odom.x), float(self.pose_odom.y))
            ),
            physical_portal_gate_xy=getattr(
                self, "active_portal_gate_odom_xy", None,
            ),
        )
        component = self.active_frontier_component
        if component is None:
            if self.portal_arrival_can_commit_without_component():
                return self.commit_portal_arrival(
                    arrival,
                    None,
                    now,
                    evidence_source="portal_identity_without_component",
                )
            self.queue_pending_portal_arrival(completed, now)
            return None
        if self.portal_arrival_still_in_source_place(arrival, component):
            self.queue_pending_portal_arrival(completed, now)
            return None
        return self.commit_portal_arrival(
            arrival,
            component,
            now,
            evidence_source=str(evidence_source),
        )
