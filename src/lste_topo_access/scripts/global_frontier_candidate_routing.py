"""Route-level validation for one frontier candidate.

This stage answers one narrow question: can a raw frontier boundary become a
safe endpoint in the current place graph?  It does not score candidates or
change place lifecycle state.
"""

import math

from global_frontier_models import FrontierCandidateRoute
from global_frontier_candidate_place_policy import (
    GlobalFrontierCandidatePlacePolicyMixin,
)
from global_frontier_candidate_portals import GlobalFrontierCandidatePortalMixin
from global_frontier_candidate_graph import candidate_graph_action


class GlobalFrontierCandidateRoutingMixin(
    GlobalFrontierCandidatePortalMixin,
    GlobalFrontierCandidatePlacePolicyMixin,
):
    """Resolve raw frontier cells into safe, graph-bounded endpoints."""

    def _resolve_frontier_candidate_route(
        self, request, frontier_row, frontier_col, approach_cells, context,
        portal_sources,
    ):
        """Turn one raw frontier cell into a validated executable endpoint."""
        approach = self.nearest_safe_approach(
            request.steps,
            frontier_row,
            frontier_col,
            approach_cells,
            preferred_steps=request.preferred_steps,
            preferred_mask=request.preferred_mask,
            selection_tier=request.selection_tier,
            # Compile an observation viewpoint along the certified route. The
            # existing approach distance is the sensor-scale standoff contract;
            # a frontier cell is never a docking target.
            standoff_cells=approach_cells,
        )
        if approach is None:
            return None
        target_row, target_col = approach
        place_action = self._classify_candidate_approach(
            request,
            target_row,
            target_col,
            frontier_row,
            frontier_col,
            context,
            portal_sources,
        )
        if place_action is None:
            return None

        action_tier, place_hops, portal_gate_xy, structural_probe = place_action
        x, y = self.cell_xy(request.message, target_row, target_col)
        costmap_distance = self.candidate_costmap_distance(request.validation, x, y)
        if request.validation is not None and costmap_distance is None:
            return None
        path_distance = (
            costmap_distance
            if costmap_distance is not None
            else request.steps[target_row, target_col]
            * request.message.info.resolution
        )
        score_path_distance = (
            float(request.steps[target_row, target_col])
            * request.message.info.resolution
            if request.score_path_from_steps
            else path_distance
        )
        work_item_id, work_item_match, work_item_support_cells, portal_probe = (
            self.local_observation_work_item(
                request, context, frontier_row, frontier_col, place_hops,
            )
        )
        work_item_normal_xy = (
            None
            if work_item_id is None
            else context.work_item_details.get(work_item_id, {}).get("normal_xy")
        )
        viewpoint_retry = False
        ledger = getattr(self, "place_work_items", None)
        if (
            work_item_id is not None
            and ledger is not None
            and ledger.has_failed_viewpoint(
                work_item_id,
                map_epoch=getattr(
                    getattr(request, "components", None), "epoch", None
                ),
            )
        ):
            # Lifecycle filters the failed standoff; survivors view this boundary anew.
            viewpoint_retry = True
        if structural_probe is not None:
            portal_probe = structural_probe
        if portal_probe is not None:
            action_tier = "probe"
            if portal_gate_xy is None:
                portal_gate_xy = self.cell_xy(
                    request.message, *tuple(portal_probe.opening_cell),
                )
            probe_ledger = getattr(self, "portal_probe_ledger", None)
            if probe_ledger is not None and portal_probe.probe_id is not None:
                probe_ledger.bind_work_item(
                    portal_probe.probe_id, work_item_id,
                )
            if str(getattr(portal_probe, "phase", "source")) == "source":
                # The frontier point that exposed the doorway can already be
                # inside TEB's terminal goal ball. Compile a distinct source-
                # side standoff so the probe represents a new observation
                # action instead of a duplicate goal that immediately stalls.
                source_route = self.compile_source_portal_probe_route(
                    request, portal_probe, portal_gate_xy,
                )
                if source_route is None:
                    return None
                (
                    target_row,
                    target_col,
                    x,
                    y,
                    costmap_distance,
                    path_distance,
                    score_path_distance,
                ) = source_route
        # A probe is a durable information obligation. Its safe viewpoint may
        # lie inside an ordinary coverage footprint, and a failed viewpoint
        # must be replaceable by another point on the same opening. The
        # WorkItem/Probe ledgers, rather than point-radius memory, own those
        # two decisions.
        if portal_probe is None:
            if request.excluded and any(
                math.hypot(x - old_x, y - old_y) < self.completed_radius
                for old_x, old_y in request.excluded
            ):
                return None
            if (
                (
                    getattr(self, "distance_deduplication_enabled", True)
                    and self.frontier_is_completed(x, y)
                )
                or self.frontier_is_rejected(x, y, request.now)
            ):
                return None
        else:
            # Navfn rejection happens after candidate construction.  Keep a
            # rejected probe viewpoint out of the next selection pass too;
            # the durable probe ledger will still admit a different viewpoint
            # for the same physical opening.
            rejected = getattr(self, "frontier_is_rejected", None)
            if callable(rejected) and rejected(x, y, request.now):
                return None

        if portal_probe is not None and not self.portal_probe_viewpoint_is_local(
            portal_gate_xy,
            (x, y),
            resolution=request.message.info.resolution,
        ):
            self.last_portal_probe_viewpoint_rejections = int(
                getattr(self, "last_portal_probe_viewpoint_rejections", 0)
            ) + 1
            return None

        source_region = None
        if context.source_place_id is not None:
            memory = getattr(self, "region_memory", None)
            lookup = None if memory is None else getattr(memory, "by_id", None)
            if lookup is not None:
                source_region = lookup(context.source_place_id)
        work_item_available = False
        if work_item_id is not None and ledger is not None:
            work_item_available = bool(
                ledger.work_item_is_available(work_item_id)
            )
        graph_action = candidate_graph_action(
            self,
            request,
            context,
            place_hops,
            work_item_id,
            work_item_available,
            viewpoint_retry,
            portal_probe,
            source_region,
        )
        self.last_graph_action_kind = graph_action.kind
        self.last_graph_action_reason = graph_action.reason
        if not graph_action.executable:
            self.last_graph_policy_rejections = int(
                getattr(self, "last_graph_policy_rejections", 0)
            ) + 1
            return None
        return FrontierCandidateRoute(
            row=target_row,
            col=target_col,
            action_tier=action_tier,
            place_hops=place_hops,
            x=x,
            y=y,
            path_distance=path_distance,
            score_path_distance=score_path_distance,
            portal_gate_xy=portal_gate_xy,
            work_item_id=work_item_id,
            work_item_match=work_item_match,
            work_item_support_cells=work_item_support_cells,
            portal_observation_probe=portal_probe,
            viewpoint_retry=viewpoint_retry,
            work_item_normal_xy=work_item_normal_xy,
            graph_action=graph_action.kind,
            graph_action_reason=graph_action.reason,
        )
