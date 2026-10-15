"""Per-cycle state and topology policy for frontier selection.

This module contains the state that must be rebuilt for every selection pass:
diagnostic counters, the current structural-place view, and the directional
sealed-portal rule.  It deliberately contains no candidate scoring or ROS
planner calls, so those policies can evolve independently.
"""

import numpy as np

from global_frontier_models import FrontierSelectionContext
from global_frontier_topology import grid_reachable_without_closed_places
from global_frontier_work_item_components import observation_supports


class GlobalFrontierSelectionContextMixin:
    """Build and audit the topology context used by one selection pass."""

    def _reset_frontier_selection_metrics(self):
        """Clear diagnostics published with the next route-selection event."""
        self.last_visibility_coverage_skips = 0
        self.last_covered_place_transit_skips = 0
        self.last_covered_place_footprint_cells = 0
        self.last_covered_place_footprint_anchors = 0
        self.last_place_graph_hop_skips = 0
        self.last_place_graph_unresolved_candidates = 0
        self.last_observed_place_reentry_skips = 0
        self.last_ready_to_exit_reentry_skips = 0
        self.last_ready_place_coverage_skips = 0
        self.last_selected_place_hops = None
        self.last_observation_departure_footprint_cells = 0
        self.last_observation_departure_anchor_count = 0
        self.target_region_claim_cross_region_skips = 0
        self.last_semantic_pursuit_forward_candidates = 0
        self.last_target_direction_candidates = 0
        self.last_portal_sources = 0
        # A source is only a structural candidate. These separate audit
        # counters distinguish candidates seen from candidates that survived
        # every physical and graph-action admission gate.
        self.last_portal_candidates_seen = 0
        self.last_portal_hard_rejected = 0
        self.last_portal_executable_candidates = 0
        self.last_portal_endpoint_depth_rejections = 0
        self.last_portal_missing_transition_goals = 0
        self.last_portal_probe_candidates = 0
        self.last_portal_probe_viewpoint_rejections = 0
        self.last_structural_boundary_candidates = 0
        self.last_structural_boundary_rejections = 0
        self.last_structural_boundary_source_place_rejections = 0
        self.last_portal_probe_value_selection = None
        self.last_portal_decision = None
        self.last_portal_excluded = 0
        self.last_portal_rejected = 0
        self.last_portal_costmap_rejected = 0
        self.last_portal_heading_rejected = 0
        self.last_portal_costmap_fallbacks = 0
        self.last_portal_endpoint = None
        self.last_portal_covered_destination_skips = 0
        self.last_portal_source_side_rejections = 0
        self.last_portal_reverse_egress_count = 0
        self.last_portal_reverse_egress_region_id = None
        self.last_portal_covered_cycle_skips = 0
        self.last_portal_destination_class = None
        self.last_portal_unbound_reused = 0
        self.last_portal_unbound_reprojection_skips = 0
        self.last_sealed_portal_reentry_skips = 0
        self.last_sealed_portal_reentry_region_id = None
        self.last_sealed_portal_reentry_gate = None
        self.last_uncertified_place_transition_skips = 0
        self.last_graph_policy_rejections = 0
        self.last_graph_action_kind = None
        self.last_graph_action_reason = None
        # An observed source place may never dispatch a remote endpoint across
        # a doorway. These counters make the explicit portal conversion
        # visible in route-selection logs and benchmark evidence.
        self.last_cross_place_endpoint_deferrals = 0
        self.last_source_place_observed = False
        self.last_action_tier_order = ("unconstrained",)
        self.last_work_item_created = 0
        self.last_work_item_inherited = 0
        self.last_work_item_resolved_descendant_skips = 0
        self.last_work_item_failed_viewpoint_skips = 0
        self.last_work_item_alternative_viewpoints = 0
        self.last_selected_viewpoint_retry = False
        self.last_work_item_sensor_horizon_m = None
        self.last_work_item_place_scope_skips = 0
        self.last_crossed_portal_support_skips = 0
        # A selection pass evaluates several frontier endpoints along the
        # same BFS doorway. Cache its pure cut verdict for this one map epoch;
        # a later map snapshot deliberately starts from fresh evidence.
        self._portal_transition_certification_cache = {}

    def context_has_pending_local_work(self, context):
        """Return whether the current Place still owns local WorkItems.

        Source-side Portal probes stay in their own durable ledger.  They are
        outward evidence and may be selected before an unqualified doorway,
        but they do not make a completed local observation phase reopen.
        """
        if context is None or not getattr(self, "work_item_memory_enabled", True):
            return False
        if getattr(context, "target_observation_work_pending", False):
            return True
        target_work = getattr(self, "target_observation_work", None)
        if (
            target_work is not None
            and context.source_place_id is not None
            and target_work.has_pending(context.source_place_id)
        ):
            # Target observation is a mission WorkItem, not a detector
            # freshness window. Keep the current Place as the owner until the
            # task itself reports completion, even when ordinary frontier
            # descendants are temporarily absent from the latest SLAM map.
            return True
        ledger = getattr(self, "place_work_items", None)
        if ledger is None or context.source_place_id is None:
            work_pending = False
        else:
            portal_probe_ledger = getattr(self, "portal_probe_ledger", None)
            probe_for_work = (
                None if portal_probe_ledger is None
                else getattr(portal_probe_ledger, "probe_for_work_item", None)
            )
            work_pending = False
            for item_id, detail in (context.work_item_details or {}).items():
                if str(detail.get("state", "")) != "unresolved":
                    continue
                # Doorway WorkItems are settled by Portal evidence, not by
                # ordinary local coverage. Their Attempt may remain active
                # while the source/destination probe ledger advances.
                if callable(probe_for_work) and probe_for_work(item_id) is not None:
                    continue
                if ledger.work_item_is_available(item_id):
                    work_pending = True
                    break
        return work_pending

    def _scope_observation_supports_to_source_place(
        self, supports, components, source_label
    ):
        """Keep local WorkItems owned by the current structural Place.

        ``observation_supports`` is computed from the map-wide frontier mask,
        while WorkItems are durable obligations of one physical Place.  A
        support whose frontier arc belongs to another labelled core must be
        represented by the Portal/graph planner, not attached to the current
        Place.  Zero-labelled throat cells are retained only when the same arc
        also touches the current core; a throat by itself is a Portal concern.
        """
        if not supports or components is None or source_label is None:
            return tuple(supports or ())
        try:
            source_label = int(source_label)
            labels = components.labels
            rows, cols = labels.shape
        except (AttributeError, TypeError, ValueError):
            return tuple(supports or ())
        if source_label <= 0:
            return tuple(supports or ())

        scoped = []
        skipped = 0
        for support in supports:
            positive_labels = set()
            for row, col in getattr(support, "frontier_cells", ()) or ():
                try:
                    row, col = int(row), int(col)
                except (TypeError, ValueError):
                    continue
                if 0 <= row < rows and 0 <= col < cols:
                    label = int(labels[row, col])
                    if label > 0:
                        positive_labels.add(label)
            if positive_labels and positive_labels.issubset({source_label}):
                scoped.append(support)
            else:
                skipped += 1
        self.last_work_item_place_scope_skips += skipped
        return tuple(scoped)

    def _suppress_crossed_portal_supports(self, source_place_id, supports):
        """Remove reverse doorway arcs already represented by a crossed edge.

        ``observation_supports`` is deliberately map-local and can rediscover
        the back of a doorway after a crossing.  Letting that support enter
        the WorkItem ledger creates a false unresolved branch and sends the
        robot back through the same room.  The Portal ledger owns the physical
        identity, so this filter keeps the two ledgers consistent before any
        WorkItem is minted.
        """
        if not supports or source_place_id is None:
            return supports
        portal_ledger = getattr(self, "portal_hypothesis_ledger", None)
        query = (
            None
            if portal_ledger is None
            else getattr(
                portal_ledger, "crossed_reverse_for_destination", None,
            )
        )
        if not callable(query):
            return supports
        memory = getattr(self, "region_memory", None)
        gate_radius = getattr(memory, "portal_entry_merge_radius", None)
        kept = []
        reported = getattr(self, "_crossed_portal_support_reports", None)
        if reported is None:
            reported = set()
            self._crossed_portal_support_reports = reported
        for support in supports:
            edge = query(
                source_place_id,
                getattr(support, "anchor_xy", None),
                getattr(support, "normal_xy", None),
                gate_radius=gate_radius,
            )
            if edge is None:
                kept.append(support)
                continue
            self.last_crossed_portal_support_skips += 1
            signature = (int(source_place_id), int(edge["id"]))
            if signature not in reported:
                reported.add(signature)
                publish = getattr(self, "publish_status", None)
                if callable(publish):
                    publish(
                        "portal_observation_support_suppressed",
                        source_place_id=int(source_place_id),
                        portal_id=int(edge["id"]),
                        anchor=(
                            None
                            if getattr(support, "anchor_xy", None) is None
                            else [
                                round(float(support.anchor_xy[0]), 3),
                                round(float(support.anchor_xy[1]), 3),
                            ]
                        ),
                        normal=(
                            None
                            if getattr(support, "normal_xy", None) is None
                            else [
                                round(float(support.normal_xy[0]), 3),
                                round(float(support.normal_xy[1]), 3),
                            ]
                        ),
                        reason="reverse_side_of_crossed_portal_is_transit",
                    )
        return tuple(kept)

    def sealed_portal_reentry(
        self, gate_xy, destination_xy, map_to_physical_xy=None,
    ):
        """Return a dormant region if this route re-enters through its gate.

        The check is directional: the reverse crossing is a permitted exit
        from a closed room. A confirmed visual target claim remains the sole
        intentional re-entry exception.
        """
        if self.target_region_claim_active:
            return None
        lookup = getattr(self.region_memory, "sealed_portal_entry", None)
        if lookup is None:
            return None
        physical_gate = None
        physical_destination = None
        if map_to_physical_xy is not None:
            physical_gate = map_to_physical_xy(gate_xy[0], gate_xy[1])
            physical_destination = map_to_physical_xy(
                destination_xy[0], destination_xy[1]
            )
        matched = lookup(
            gate_xy,
            destination_xy,
            physical_gate_xy=physical_gate,
            physical_destination_xy=physical_destination,
        )
        if matched is None:
            return None
        region, _entry = matched
        self.last_sealed_portal_reentry_skips += 1
        self.last_sealed_portal_reentry_region_id = int(region["id"])
        self.last_sealed_portal_reentry_gate = [
            round(float(gate_xy[0]), 3), round(float(gate_xy[1]), 3),
        ]
        return region

    def _prepare_frontier_selection_context(
        self, message, steps, frontier, unknown, occupied, route_anchor_xy,
        components, now=0.0, map_to_physical_xy=None,
    ):
        """Build the structural-place view used by one selection pass."""
        self._reset_frontier_selection_metrics()
        known_free = ~(unknown | occupied)
        # Geometry baselines must not accidentally inherit an already-created
        # Place from the shared process. Their only durable state, if any, is
        # the explicitly declared endpoint-radius memory.
        if not getattr(self, "place_memory_enabled", True):
            return FrontierSelectionContext(
                known_free=known_free,
                covered_place_reachable=None,
                source_component=None,
                source_label=None,
                source_place_id=None,
                observed_reentry_labels=frozenset(),
                blocked_place_labels=frozenset(),
                place_graph_ready=False,
                source_place_observed=False,
                action_tiers=("unconstrained",),
                work_item_cells={},
                work_item_details={},
                target_observation_work_pending=False,
            )
        covered_place_reachable = None
        source_component = None
        source_label = None
        source_place_id = None
        source_region = None
        observed_reentry_labels = set()
        blocked_place_labels = set()
        place_graph_ready = False
        source_place_observed = False
        target_observation_work_pending = False
        current_place_id = getattr(self, "current_physical_place_id", None)
        if current_place_id is not None:
            current_region = self.region_memory.by_id(current_place_id)
            if current_region is not None and current_region.get("state") in (
                "open", "ready_to_exit", "dormant", "suspended",
            ):
                source_region = current_region
                source_place_id = int(current_region["id"])
                source_place_observed = int(current_region.get("endpoint_observations", 0)) > 0
        target_work = getattr(self, "target_observation_work", None)
        if target_work is not None and source_place_id is not None:
            target_observation_work_pending = bool(
                target_work.has_pending(source_place_id)
            )
        if components is not None and not self.target_region_claim_active:
            closed_labels = self.region_memory.dormant_component_labels(
                components.epoch
            )
            source_component = self.component_at_map_position(
                message,
                components,
                known_free,
                route_anchor_xy[0],
                route_anchor_xy[1],
            )
            source_label = (
                None if source_component is None else int(source_component["label"])
            )
            place_graph_ready = source_label is not None and source_label > 0
            if place_graph_ready and source_region is None:
                _source_tier, source_region = self.region_memory.candidate_tier(
                    route_anchor_xy[0],
                    route_anchor_xy[1],
                    0.0,
                    component=source_component,
                )
                source_place_id = (
                    None
                    if source_region is None
                    or not hasattr(self.region_memory, "by_id")
                    else int(source_region["id"])
                )
            if source_region is not None:
                source_place_observed = bool(
                    source_region.get("state") in (
                        "open", "ready_to_exit", "dormant", "suspended",
                    )
                    and (
                        source_region.get("state") == "dormant"
                        or source_region.get("entered_at") is not None
                    )
                    and int(source_region.get("endpoint_observations", 0)) > 0
                )
            observed_reentry_labels = self.region_memory.observed_open_component_labels(
                components.epoch
            )
            # The source place is always traversable while the base occupies
            # it, including when it is already dormant.  A dormant source is
            # transit-only, not a local exploration candidate, but masking it
            # as closed would strand the robot before it can reach its next
            # portal. Every *other* observed or dormant place remains a
            # durable explored barrier.
            if place_graph_ready:
                observed_reentry_labels.discard(source_label)
                blocked_place_labels.discard(source_label)
            blocked_place_labels = set(closed_labels) | observed_reentry_labels
            covered_footprint, anchor_count = self.covered_place_observation_footprint(
                message,
                known_free,
                components,
                route_anchor_xy,
                source_component,
            )
            self.last_covered_place_footprint_cells = int(
                np.count_nonzero(covered_footprint)
            )
            self.last_covered_place_footprint_anchors = int(anchor_count)
            if blocked_place_labels or self.last_covered_place_footprint_cells:
                covered_place_reachable = grid_reachable_without_closed_places(
                    components.labels,
                    steps,
                    source_label,
                    blocked_place_labels,
                    closed_footprint=covered_footprint,
                )
        action_tiers = (
            ("adjacent", "probe", "local")
            if place_graph_ready and source_place_observed
            else (("local", "adjacent") if place_graph_ready else ("unconstrained",))
        )
        work_item_cells = {}
        work_item_details = {}
        ledger = getattr(self, "place_work_items", None)
        if (
            getattr(self, "work_item_memory_enabled", True)
            and
            ledger is not None
            and source_place_id is not None
            and frontier is not None
        ):
            horizon = getattr(self, "scan_observation_horizon", None)
            if horizon is None or horizon <= 0.0:
                horizon = getattr(self, "scan_range_max", None)
            self.last_work_item_sensor_horizon_m = horizon
            supports = observation_supports(
                frontier,
                unknown,
                cell_xy=lambda row, col: self.cell_xy(message, row, col),
                map_to_physical_xy=map_to_physical_xy,
                resolution=message.info.resolution,
                sensor_horizon_m=horizon,
            )
            supports = self._scope_observation_supports_to_source_place(
                supports,
                components,
                source_label,
            )
            supports = self._suppress_crossed_portal_supports(
                source_place_id,
                supports,
            )
            work_item_cells, work_item_details = ledger.reconcile(
                source_place_id,
                supports,
                now=now,
            )
            for detail in work_item_details.values():
                if detail["match"] == "created":
                    self.last_work_item_created += 1
                elif detail["match"] == "inherited":
                    self.last_work_item_inherited += 1
                elif detail["match"] == "resolved_descendant":
                    self.last_work_item_resolved_descendant_skips += 1
        pending_entry = getattr(self, "place_entry_rehydration_pending", None)
        if pending_entry is not None:
            try:
                entry_place_id = int(pending_entry)
            except (TypeError, ValueError):
                entry_place_id = None
            if entry_place_id is not None and source_place_id == entry_place_id:
                # This is the architectural handoff from a durable physical
                # transition to snapshot-local observation evidence.  It is
                # deliberately cleared after reconciliation, even when the
                # current snapshot yields zero WorkItems: only then is an
                # empty room an evidence-backed fact rather than a race.
                self.place_entry_rehydration_pending = None
                publish = getattr(self, "publish_status", None)
                if callable(publish):
                    publish(
                        "place_entry_rehydrated",
                        place_id=int(source_place_id),
                        work_item_count=len(work_item_details),
                        portal_probe_count=int(
                            getattr(self, "last_portal_probe_candidates", 0)
                        ),
                        reason="first_snapshot_after_portal_arrival",
                    )
        self.last_source_place_observed = source_place_observed
        self.last_action_tier_order = action_tiers
        return FrontierSelectionContext(
            known_free=known_free,
            covered_place_reachable=covered_place_reachable,
            source_component=source_component,
            source_label=source_label,
            source_place_id=source_place_id,
            observed_reentry_labels=frozenset(observed_reentry_labels),
            blocked_place_labels=frozenset(blocked_place_labels),
            place_graph_ready=place_graph_ready,
            source_place_observed=source_place_observed,
            action_tiers=action_tiers,
            work_item_cells=work_item_cells,
            work_item_details=work_item_details,
            target_observation_work_pending=target_observation_work_pending,
        )
