"""Select and record one cacheable successor frontier."""

import math

import rospy

from global_frontier_models import SelectedFrontier
from global_frontier_topology import copy_component_evidence


class GlobalFrontierPrefetchSelectionMixin:
    def prefetch_next_frontier(
        self, message, steps, frontier, unknown, occupied, now, active_xy,
        robot_map, validation=None, heading_reference=None, route_seed=None,
        preferred_steps=None, preferred_mask=None, route_anchor_xy=None,
        transition_basis="robot_bfs_tangent", score_path_from_steps=False,
        max_heading_delta=None, allow_heading_fallback=True,
        transition_preference="unclassified", components=None,
    ):
        """Select a same-place successor and cache it for terminal promotion."""
        if self.prefetched_frontier is not None or self.active_frontier is None:
            return
        route_anchor_xy = active_xy if route_anchor_xy is None else route_anchor_xy
        candidate = self.choose_valid_frontier(
            message,
            steps,
            frontier,
            unknown,
            occupied,
            now,
            robot_map,
            excluded=[active_xy],
            validation=validation,
            heading_reference=heading_reference,
            max_heading_delta=(
                self.heading_hard_limit
                if max_heading_delta is None else max_heading_delta
            ),
            route_seed=route_seed,
            route_anchor_xy=route_anchor_xy,
            score_path_from_steps=score_path_from_steps,
            allow_heading_fallback=allow_heading_fallback,
            preferred_steps=preferred_steps,
            preferred_mask=preferred_mask,
            components=components,
            graph_route_planning=False,
        )
        if candidate is None:
            return False
        selected = SelectedFrontier.from_candidate(candidate)
        if selected.place_hops is not None and int(selected.place_hops) >= 1:
            # A prefetch precedes the endpoint's final scan. Cross-place work
            # must wait until that scan proves the current place is complete.
            self.publish_status(
                "frontier_prefetch_deferred",
                reason="cross_place_action_requires_terminal",
                active_goal=[round(float(active_xy[0]), 3), round(float(active_xy[1]), 3)],
                pending_goal=[round(float(selected.x), 3), round(float(selected.y), 3)],
                pending_place_hops=int(selected.place_hops),
                pending_route_kind=selected.route_kind,
            )
            return False
        return self._cache_prefetched_frontier(
            message,
            steps,
            route_seed,
            route_anchor_xy,
            heading_reference,
            transition_basis,
            transition_preference,
            active_xy,
            selected,
        )

    def _cache_prefetched_frontier(
        self,
        message,
        steps,
        route_seed,
        route_anchor_xy,
        heading_reference,
        transition_basis,
        transition_preference,
        active_xy,
        selected,
    ):
        """Persist one selected candidate and publish its explicit contract."""
        if getattr(self, "place_memory_enabled", True):
            pending_region_tier, pending_region = self.region_memory.candidate_tier(
                selected.x,
                selected.y,
                selected.information,
                component=selected.component,
            )
        else:
            pending_region_tier, pending_region = "geometry", None
        pending_entry_heading = None
        if route_seed is not None:
            pending_entry_heading, _ = self.route_headings(
                message,
                steps,
                route_seed,
                (selected.row, selected.col),
                route_anchor_xy,
            )
        transition_distance = (
            None
            if steps is None or steps[selected.row, selected.col] < 0
            else float(steps[selected.row, selected.col])
            * float(message.info.resolution)
        )
        self.prefetched_frontier = (
            selected.row, selected.col, selected.x, selected.y
        )
        self.prefetched_goal_map = (float(selected.x), float(selected.y))
        self.prefetched_frontier_information = float(selected.information)
        self.prefetched_frontier_component = copy_component_evidence(
            selected.component
        )
        self.prefetched_work_item_id = selected.work_item_id
        self.prefetched_work_item_match = selected.work_item_match
        self.prefetched_work_item_support_cells = int(
            selected.work_item_support_cells or 0
        )
        self.publish_status(
            "frontier_prefetched",
            active_goal=[round(float(active_xy[0]), 3), round(float(active_xy[1]), 3)],
            pending_goal=[round(float(selected.x), 3), round(float(selected.y), 3)],
            pending_route_id=int(self.active_route_id + 1),
            pending_entry_yaw=(
                None
                if pending_entry_heading is None
                else round(float(pending_entry_heading), 4)
            ),
            pending_entry_yaw_basis=(
                transition_basis if pending_entry_heading is not None else None
            ),
            transition_path_distance=(
                None if transition_distance is None
                else round(float(transition_distance), 3)
            ),
            transition_preference=transition_preference,
            pending_region_id=(
                None if pending_region is None else int(pending_region["id"])
            ),
            pending_region_tier=pending_region_tier,
            pending_component_cells=(
                None
                if selected.component is None
                else int(selected.component["cells"])
            ),
            pending_place_hops=selected.place_hops,
            pending_route_kind=selected.route_kind,
            pending_work_item_id=selected.work_item_id,
            pending_work_item_match=selected.work_item_match,
            pending_work_item_support_cells=int(selected.work_item_support_cells or 0),
            place_graph_hop_limit=int(self.place_graph_hop_limit),
        )
        heading_delta = self._candidate_route_heading_delta(
            message,
            steps,
            route_seed,
            selected.row,
            selected.col,
            route_anchor_xy,
            heading_reference,
        )
        rospy.loginfo(
            "Global frontier prefetched next branch active=(%.2f,%.2f) "
            "pending=(%.2f,%.2f) robot_path=%.2fm transition_path=%.2fm "
            "information=%.0f structure=%.0f score=%.2f transition_delta=%.1fdeg",
            active_xy[0],
            active_xy[1],
            selected.x,
            selected.y,
            selected.path_distance,
            float("nan") if transition_distance is None else transition_distance,
            selected.information,
            selected.structure,
            selected.score,
            float("nan") if heading_delta is None else math.degrees(heading_delta),
        )
        return True
