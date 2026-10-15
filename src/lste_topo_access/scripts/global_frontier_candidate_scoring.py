"""Lifecycle-aware scoring for validated frontier endpoints."""

import math
from global_frontier_scoring import (
    candidate_score as score_frontier_candidate,
    record_scored_candidate,
    select_score_pool_candidate,
)
from global_frontier_candidate_lifecycle import GlobalFrontierCandidateLifecycleMixin
from global_frontier_frontier_decision import candidate_from_route
from global_frontier_topology import semantic_hint_is_forward
class GlobalFrontierCandidateScoringMixin(GlobalFrontierCandidateLifecycleMixin):
    """Assign lifecycle buckets and scores without changing route geometry."""
    def _route_heading_penalty(
        self, request, candidate_cell, endpoint_xy,
    ):
        """Score the BFS entry tangent rather than a misleading direct ray."""
        if not getattr(self, "heading_policy_enabled", True):
            return None, 0.0
        endpoint_heading_delta = self.heading_delta(
            endpoint_xy[0],
            endpoint_xy[1],
            request.route_anchor_xy,
            request.heading_reference,
        )
        route_initial_heading = None
        if request.route_seed is not None:
            route_initial_heading, _ = self.route_headings(
                request.message,
                request.steps,
                request.route_seed,
                candidate_cell,
                request.route_anchor_xy,
            )
        route_heading_delta = (
            endpoint_heading_delta
            if route_initial_heading is None or request.heading_reference is None
            else abs(
                self._angle_delta(route_initial_heading, request.heading_reference)
            )
        )
        heading_penalty = (
            0.0
            if route_heading_delta is None
            else self.heading_weight * (1.0 - math.cos(route_heading_delta))
        )
        return route_heading_delta, heading_penalty
    def _frontier_candidate_score(
        self, information, structure, score_path_distance, candidate_xy,
        heading_penalty, semantic_hint,
    ):
        """Apply the deterministic frontier-scoring policy."""
        return score_frontier_candidate(
            information,
            structure,
            score_path_distance,
            candidate_xy,
            heading_penalty,
            semantic_hint,
            self.structure_weight,
            self.semantic_hint_weight,
            self.semantic_hint_max_distance,
        )

    def _score_frontier_endpoint(
        self, request, frontier_row, frontier_col, candidate, context,
    ):
        """Return a ranked endpoint tuple and its score-pool classification."""
        information = self.frontier_information(
            request.unknown, frontier_row, frontier_col,
        )
        pool_data = self._candidate_region_pool(
            request, candidate, context, information,
        )
        if pool_data is None:
            return None
        pool_name, component = pool_data
        structure = (
            self.frontier_structure(request.occupied, frontier_row, frontier_col)
            if getattr(self, "structural_score_enabled", True)
            else 0.0
        )
        route_heading_delta, heading_penalty = self._route_heading_penalty(
            request,
            (candidate.row, candidate.col),
            (candidate.x, candidate.y),
        )
        if (
            request.max_heading_delta is not None
            and route_heading_delta is not None
            and route_heading_delta > request.max_heading_delta
        ):
            return None
        score = self._frontier_candidate_score(
            information,
            structure,
            candidate.score_path_distance,
            (candidate.x, candidate.y),
            heading_penalty,
            request.semantic_hint,
        )
        target_direction = False
        target_belief = getattr(self, "target_belief", None)
        if (
            getattr(self, "task_semantic_value_enabled", False)
            and target_belief is not None
            and context.source_place_id is not None
            and candidate.work_item_normal_xy is not None
        ):
            target_direction = target_belief.direction_matches(
                context.source_place_id,
                candidate.work_item_normal_xy,
            )
        if target_direction:
            self.last_target_direction_candidates += 1
        ranking_action_tier = (
            "viewpoint_retry"
            if candidate.viewpoint_retry
            else "target_direction" if target_direction else candidate.action_tier
        )
        scored_candidate = (
            candidate.row,
            candidate.col,
            candidate.x,
            candidate.y,
            candidate.path_distance,
            information,
            structure,
            score,
            component,
            candidate.place_hops,
            "frontier_endpoint",
            candidate.portal_gate_xy,
            candidate.work_item_id,
            candidate.work_item_match,
            candidate.work_item_support_cells,
            candidate.portal_observation_probe,
            candidate.viewpoint_retry,
            candidate.graph_action,
            candidate.graph_action_reason,
            ranking_action_tier,
        )
        pursuit_candidate = bool(
            request.semantic_pursuit
            and semantic_hint_is_forward(
                request.robot_map, request.semantic_hint, (candidate.x, candidate.y),
            )
        )
        if pursuit_candidate:
            self.last_semantic_pursuit_forward_candidates += 1
        if candidate.viewpoint_retry:
            self.last_work_item_alternative_viewpoints += 1
        return scored_candidate, pool_name, pursuit_candidate, ranking_action_tier
    def _collect_frontier_score_pools(
        self, request, rows, cols, context, pools, portal_sources,
        minimum_steps, approach_cells, probe_candidates=None,
        frontier_decision_candidates=None, graph_obligation_candidates=None,
    ):
        for frontier_row, frontier_col in zip(rows.tolist(), cols.tolist()):
            if getattr(self, "planning_should_preempt", lambda: False)(): return
            candidate = self._resolve_frontier_candidate_route(
                request,
                frontier_row,
                frontier_col,
                approach_cells,
                context,
                portal_sources,
            )
            if candidate is None:
                continue
            scored = self._score_frontier_endpoint(
                request, frontier_row, frontier_col, candidate, context,
            )
            if scored is None:
                continue
            scored_candidate, pool_name, pursuit_candidate, ranking_action_tier = scored
            route_steps = request.steps[candidate.row, candidate.col]
            decision_candidate = candidate_from_route(
                scored_candidate, region_tier=pool_name,
                ranking_category=ranking_action_tier,
                map_epoch=getattr(request.components, "epoch", None),
                place_id=context.source_place_id,
            )
            if frontier_decision_candidates is not None and (
                route_steps >= minimum_steps
                or request.selection_tier == "navfn_observation_recovery"
            ):
                frontier_decision_candidates.append(decision_candidate)
            if (
                graph_obligation_candidates is not None
                and (
                    route_steps >= minimum_steps
                    or request.selection_tier == "navfn_observation_recovery"
                )
            ):
                graph_obligation_candidates.append(decision_candidate)
            record_scored_candidate(
                pools,
                ranking_action_tier,
                scored_candidate,
                pool_name,
                pursuit_candidate,
                request.steps[candidate.row, candidate.col],
                minimum_steps,
                request.selection_tier,
                self.min_structure_cells,
            )
            if (probe_candidates is not None and len(scored_candidate) > 15
                    and scored_candidate[15] is not None):
                probe_candidates.append(
                    (scored_candidate, pool_name, ranking_action_tier)
                )
    def _select_score_pool_candidate(self, pools, action_tiers, request):
        selected, region_tier = select_score_pool_candidate(
            pools,
            action_tiers,
            request.allowed_region_tiers,
            request.semantic_pursuit,
        )
        if selected is not None:
            self.last_frontier_region_tier = region_tier
            self.last_selected_place_hops = selected[9]
            self.last_selected_viewpoint_retry = bool(
                len(selected) > 16 and selected[16]
            )
        return selected
