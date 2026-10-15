"""Place-graph admission policy for one safe frontier approach.

This mixin deliberately owns only the discrete decision about whether an
approach is local, a certified portal action, or a source-side probe.  Route
geometry and candidate scoring remain in their respective stages.
"""

from global_frontier_portal_certification import (
    certified_route_place_transitions,
    portal_certification_window,
)
from global_frontier_topology import place_action_tier, route_place_hops


class GlobalFrontierCandidatePlacePolicyMixin:
    """Keep place-graph admission separate from endpoint construction."""

    def _classify_candidate_approach(
        self, request, target_row, target_col, frontier_row, frontier_col,
        context, portal_sources,
    ):
        """Return the permitted place action for one safe approach."""
        components = request.components
        if (
            context.covered_place_reachable is not None
            and not context.covered_place_reachable[target_row, target_col]
        ):
            target_label = int(components.labels[target_row, target_col])
            counter = (
                "last_observed_place_reentry_skips"
                if target_label in context.observed_reentry_labels
                else "last_covered_place_transit_skips"
            )
            setattr(self, counter, getattr(self, counter) + 1)
            return None

        place_hops = None
        portal_gate_xy = None
        structural_probe = None
        if context.place_graph_ready and getattr(
            self, "certified_portals_enabled", True,
        ):
            known_free = ~(request.unknown | request.occupied)
            throat_radius_cells, window_margin_cells = (
                portal_certification_window(
                    request.message.info.resolution,
                    getattr(self, "region_topology_clearance", 0.50),
                    getattr(self, "place_furniture_max_span_m", 2.5),
                )
            )
            raw_place_hops = route_place_hops(
                components.labels,
                request.steps,
                target_row,
                target_col,
                source_label=context.source_label,
            )
            if raw_place_hops is None:
                self.last_place_graph_unresolved_candidates += 1
                structural_probe = self.unresolved_portal_probe(
                    request, frontier_row, frontier_col,
                )
                return (
                    None if structural_probe is None
                    else ("probe", 0, None, structural_probe)
                )
            transitions = certified_route_place_transitions(
                components.labels,
                request.steps,
                known_free,
                target_row,
                target_col,
                context.source_label,
                throat_radius_cells,
                window_margin_cells=window_margin_cells,
                cache=self._portal_transition_certification_cache,
                **self._portal_wall_support_kwargs(request.message, components)
            )
            place_hops = len(transitions)
            for transition in transitions:
                gate_xy = self.cell_xy(
                    request.message, *transition.portal_cell,
                )
                destination_xy = self.cell_xy(
                    request.message, *transition.destination_cell,
                )
                if self.sealed_portal_reentry(
                    gate_xy,
                    destination_xy,
                    map_to_physical_xy=request.map_to_physical_xy,
                ) is not None:
                    return None
            if transitions:
                portal_gate_xy = self.cell_xy(
                    request.message, *transitions[0].portal_cell,
                )
            # A labelled transition may be absent or uncertified while a
            # wall-bounded unknown opening is already observable. Keep it as a
            # source-side probe instead of discarding the only route to fresh
            # doorway evidence.
            if context.source_place_observed and place_hops in (None, 0):
                structural_probe = self.unresolved_portal_probe(
                    request, frontier_row, frontier_col,
                )
                if structural_probe is not None:
                    return "probe", 0, portal_gate_xy, structural_probe
            if raw_place_hops > place_hops:
                self.last_uncertified_place_transition_skips += (
                    raw_place_hops - place_hops
                )
                if (
                    getattr(components, "structural_occupied", None) is not None
                    and place_hops == 0
                ):
                    structural_probe = self.unresolved_portal_probe(
                        request, frontier_row, frontier_col,
                    )
                    return (
                        None if structural_probe is None
                        else ("probe", 0, None, structural_probe)
                    )
            if (
                (context.source_place_id is not None or context.source_place_observed)
                and place_hops >= 1
            ):
                self.last_cross_place_endpoint_deferrals += 1
                self._queue_first_portal_action(
                    request, transitions, frontier_row, frontier_col, portal_sources,
                )
                return None
            if place_hops > self.place_graph_hop_limit:
                self.last_place_graph_hop_skips += 1
                if request.allow_portal_transitions and not transitions:
                    self.last_portal_missing_transition_goals += 1
                self._queue_first_portal_action(
                    request, transitions, frontier_row, frontier_col, portal_sources,
                )
                return None

        action_tier = place_action_tier(place_hops, context.place_graph_ready)
        return (
            None if action_tier is None
            else (action_tier, place_hops, portal_gate_xy, structural_probe)
        )
