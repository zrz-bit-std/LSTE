"""Navfn validation wrapper around frontier candidate selection."""

import math

import rospy


class GlobalFrontierSelectionValidationMixin:
    """Reject unreachable candidates without conflating startup races and gaps."""

    def choose_valid_frontier(
        self, message, steps, frontier, unknown, occupied, now,
        robot_map, validation=None, excluded=None, heading_reference=None,
        max_heading_delta=None, route_seed=None, semantic_hint=None,
        semantic_pursuit=False,
        preferred_steps=None, preferred_mask=None,
        allow_observation_recovery=False, route_anchor_xy=None,
        score_path_from_steps=False, allow_heading_fallback=True,
        components=None, allowed_region_tiers=None,
        allow_portal_transitions=False, graph_route_planning=True,
    ):
        """Choose a score-best candidate with an executable Navfn route."""
        self.frontier_validation_budget_exhausted = False
        self.frontier_validation_pending = False
        # A durable graph identity is checked against one immutable map
        # snapshot.  If that projection misses, retrying the complete chooser
        # in the remaining heading/tier slots cannot change the evidence; let
        # the next planning wake observe a new snapshot instead.
        self.graph_route_materialization_pending = False
        self.graph_route_materialization_invalidated = False
        tiers = ["strict_clearance"]
        if allow_observation_recovery:
            tiers.append("navfn_observation_recovery")
        for selection_tier in tiers:
            heading_limit = max_heading_delta
            for _ in range(8):
                miss_count_before = int(
                    getattr(self, "graph_route_materialization_miss_count", 0)
                    or 0
                )
                miss_signature_before = getattr(
                    self, "graph_route_materialization_miss_signature", None
                )
                candidate = self.choose_frontier(
                    message,
                    steps,
                    frontier,
                    unknown,
                    occupied,
                    now,
                    excluded=excluded,
                    validation=validation,
                    robot_map=robot_map,
                    heading_reference=heading_reference,
                    max_heading_delta=heading_limit,
                    route_seed=route_seed,
                    route_anchor_xy=route_anchor_xy,
                    score_path_from_steps=score_path_from_steps,
                    semantic_hint=semantic_hint,
                    semantic_pursuit=semantic_pursuit,
                    preferred_steps=preferred_steps,
                    preferred_mask=preferred_mask,
                    selection_tier=selection_tier,
                    components=components,
                    allowed_region_tiers=allowed_region_tiers,
                    allow_portal_transitions=allow_portal_transitions,
                    graph_route_planning=graph_route_planning,
                )
                if getattr(
                    self, "graph_route_materialization_invalidated", False
                ):
                    # The adapter has already discarded the exhausted
                    # durable obligation.  Do not fall through to heading or
                    # score fallbacks with the stale local candidate.
                    self.graph_route_materialization_invalidated = False
                    return None
                miss_count_after = int(
                    getattr(self, "graph_route_materialization_miss_count", 0)
                    or 0
                )
                miss_signature_after = getattr(
                    self, "graph_route_materialization_miss_signature", None
                )
                graph_plan = getattr(self, "last_graph_route_plan", None)
                materialization_miss_observed = (
                    miss_count_after > miss_count_before
                    or miss_signature_after != miss_signature_before
                )
                if (
                    candidate is None
                    and graph_route_planning
                    and materialization_miss_observed
                    and graph_plan is not None
                    and getattr(graph_plan, "status", None) == "ready"
                ):
                    self.graph_route_materialization_pending = True
                    return None
                if (
                    candidate is None
                    and heading_limit is not None
                    and allow_heading_fallback
                ):
                    rospy.loginfo(
                        "Global frontier has no %s candidate within heading "
                        "limit %.1fdeg; allowing a necessary turn",
                        selection_tier,
                        math.degrees(heading_limit),
                    )
                    heading_limit = None
                    continue
                if candidate is None:
                    break

                reachable = self.navfn_goal_reachable(
                    robot_map,
                    (candidate[2], candidate[3]),
                    message.header.frame_id or "map",
                )
                if reachable is True:
                    self.last_frontier_selection_mode = selection_tier
                    return candidate
                if reachable is None:
                    self.frontier_validation_pending = True
                    return None
                if reachable is False:
                    # Navfn rejection happens before route activation, so it
                    # is not an execution Attempt failure. Record the exact
                    # durable WorkItem viewpoint as unavailable; the next
                    # same-WorkItem projection will choose another viewpoint
                    # from its normal/lateral ladder.
                    record_rejection = getattr(
                        self, "record_work_item_route_rejection", None
                    )
                    if callable(record_rejection):
                        record_rejection(
                            message,
                            candidate,
                            now,
                            "navfn_%s" % getattr(
                                self, "navfn_last_validation_state", "rejected"
                            ),
                            map_epoch=(
                                None
                                if components is None
                                else getattr(components, "epoch", None)
                            ),
                            plan_endpoint=getattr(
                                self, "navfn_last_plan_endpoint", None
                            ),
                        )
                self.rejected_frontiers.append((now, candidate[2], candidate[3]))
                rospy.logwarn(
                    "Global frontier deferred %s Navfn-invalid candidate "
                    "map=(%.2f,%.2f) for %.0fs",
                    selection_tier,
                    candidate[2],
                    candidate[3],
                    self.rejected_timeout,
                )
            else:
                self.frontier_validation_budget_exhausted = True
                return None
        return None
