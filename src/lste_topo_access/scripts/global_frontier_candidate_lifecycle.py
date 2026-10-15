"""Place-ownership and observation-lifecycle checks for endpoint scoring."""

from global_frontier_topology import same_topology_component


class GlobalFrontierCandidateLifecycleMixin:
    """Classify endpoint lifecycle state without computing its numeric score."""

    def local_observation_work_item(
        self, request, context, frontier_row, frontier_col, place_hops,
    ):
        """Return durable local-work metadata for one raw frontier boundary."""
        if place_hops is not None and int(place_hops) >= 1:
            return None, None, 0, None
        if context.source_place_id is None:
            return None, None, 0, None
        item_id = context.work_item_cells.get((int(frontier_row), int(frontier_col)))
        detail = context.work_item_details.get(item_id, {})
        probe = None
        # A doorway WorkItem is owned by the PortalProbe transaction once the
        # probe ledger has bound it.  It must never leak back into the generic
        # local-work pool when the source probe is already active/arrived or
        # when SLAM temporarily hides the opening.  Otherwise the graph layer
        # can plan one identity while the snapshot selector returns another,
        # producing a permanent transaction mismatch.
        probe_ledger = getattr(self, "portal_probe_ledger", None)
        probe_for_work = (
            None
            if probe_ledger is None
            else getattr(probe_ledger, "probe_for_work_item", None)
        )
        bound_probe = (
            None
            if not callable(probe_for_work) or item_id is None
            else probe_for_work(item_id)
        )
        # A pending target-observation transaction owns the current Place.
        # Reuse a safe local viewpoint for that transaction, but do not let
        # the same frontier be upgraded into a Portal probe in this pass. The
        # dedicated probe collector still retains the doorway evidence for a
        # later graph action once target verification settles.
        target_observation_pending = bool(
            getattr(context, "target_observation_work_pending", False)
        )
        if (
            context.source_place_observed
            and item_id is not None
            and not target_observation_pending
        ):
            probe = self.portal_observation_probe_for_frontier(
                request, frontier_row, frontier_col,
            )
            if probe is not None:
                self.last_portal_probe_candidates += 1
        if bound_probe is not None and probe is None:
            # The dedicated rehydration pass can still select this durable
            # probe by physical identity. Returning no local WorkItem here is
            # the typed ownership boundary that prevents a generic endpoint
            # from competing with that pass.
            return None, None, 0, None
        return (
            item_id,
            detail.get("match"),
            int(detail.get("support_cell_count", 0)),
            probe,
        )

    def _candidate_has_durable_place_ownership(
        self, request, candidate, component, context,
    ):
        """Require a current or adjacent place after topology is available."""
        if (
            request.components is None
            or not context.place_graph_ready
            or self.target_region_claim_active
        ):
            return True
        if component is None:
            self.last_place_graph_unresolved_candidates += 1
            return False
        if request.selection_tier != "navfn_observation_recovery":
            return True
        if (
            candidate.action_tier not in ("local", "adjacent")
            or candidate.place_hops not in (0, 1)
        ):
            self.last_place_graph_unresolved_candidates += 1
            return False
        return True

    def _candidate_region_pool(
        self, request, candidate, context, information,
    ):
        """Apply ownership, coverage, and memory rules to one endpoint."""
        is_portal_probe = (
            getattr(candidate, "portal_observation_probe", None) is not None
        )
        is_work_item_rehydration = (
            getattr(candidate, "work_item_match", None)
            == "rehydrated_work_item"
        )
        if not getattr(self, "place_memory_enabled", True):
            # The ordinary and distance-deduplicated frontier baselines do
            # not get implicit benefit from room ownership, observation rays,
            # or SLAM-component re-identification.
            return "new", None
        components = request.components
        component = (
            None
            if components is None
            else self.topology_component_evidence(
                request.message,
                components,
                candidate.row,
                candidate.col,
                steps=request.steps,
            )
        )
        if (
            context.place_graph_ready
            and candidate.place_hops == 0
            and component is not None
            and int(component["label"]) != int(context.source_label)
        ):
            # A certified local bypass proves this is an internal structural
            # split, not a new room. Keep the current place identity.
            component = context.source_component
        if not self._candidate_has_durable_place_ownership(
            request, candidate, component, context,
        ):
            return None

        if self.target_region_claim_active:
            endpoint_component = self.component_at_map_position(
                request.message,
                components,
                context.known_free,
                candidate.x,
                candidate.y,
            )
            if not same_topology_component(
                self.target_region_claim_component, endpoint_component,
            ):
                self.target_region_claim_cross_region_skips += 1
                return None

        coverage_method = getattr(
            self, "candidate_covered_by_covered_place_viewpoint", None
        )
        if coverage_method is None:
            coverage_method = getattr(
                self, "candidate_covered_by_ready_place_viewpoint", None
            )
        coverage = (
            None
            if coverage_method is None
            else coverage_method(
                request.message,
                context.known_free,
                candidate.row,
                candidate.col,
                candidate.x,
                candidate.y,
            )
        )
        if coverage is not None and not (is_portal_probe or is_work_item_rehydration):
            self.last_ready_place_coverage_skips += 1
            return None

        covered_by = self.candidate_covered_by_completed_viewpoint(
            request.message,
            context.known_free,
            components,
            candidate.row,
            candidate.col,
            candidate.x,
            candidate.y,
        )
        if covered_by is not None and not (is_portal_probe or is_work_item_rehydration):
            self.last_visibility_coverage_skips += 1
            return None

        physical_xy = None if request.map_to_physical_xy is None else request.map_to_physical_xy(
            candidate.x, candidate.y,
        )
        source_region = None
        if (
            context.source_place_id is not None
            and candidate.place_hops == 0
            and hasattr(self.region_memory, "by_id")
        ):
            source_region = self.region_memory.by_id(context.source_place_id)
        if source_region is not None:
            # A certified zero-hop path is inside the source physical Place,
            # even if an office desk split its current structural component.
            if (
                source_region.get("state") == "dormant"
                and not (is_portal_probe or is_work_item_rehydration)
            ):
                # The base can still leave this closed Place through a portal,
                # but ordinary frontier geometry must not reopen local work.
                # Explicit durable rehydration is the narrow exception: the
                # current physical Place may still own a WorkItem or a
                # source/destination PortalProbe discovered after it was
                # marked dormant.
                return None
            region_tier, region = "revisit", source_region
            ledger = (
                getattr(self, "place_work_items", None)
                if getattr(self, "work_item_memory_enabled", True)
                else None
            )
            if ledger is not None:
                probe_ledger = getattr(self, "portal_probe_ledger", None)
                probe_for_work = None
                if probe_ledger is not None and candidate.work_item_id is not None:
                    probe_for_work = probe_ledger.probe_for_work_item(
                        candidate.work_item_id
                    )
                if (
                    probe_for_work is not None
                    and probe_for_work.get("state") == "source_arrived"
                    and not is_portal_probe
                ):
                    # The source-side view was reached, but the doorway's
                    # destination evidence is still pending. Do not turn the
                    # same boundary back into a generic local route.
                    return None
                if candidate.work_item_id is not None and is_portal_probe:
                    # Portal probes have their own physical obligation. Keep
                    # only the useful part of WorkItem lineage here: when an
                    # unresolved item already recorded this exact viewpoint
                    # as a failed Attempt, let a different safe point win.
                    if ledger.work_item_is_available(candidate.work_item_id):
                        viewpoint_xy = (
                            (candidate.x, candidate.y)
                            if physical_xy is None else physical_xy
                        )
                        if not ledger.viewpoint_is_available(
                            candidate.work_item_id,
                            viewpoint_xy,
                            map_epoch=getattr(
                                getattr(request, "components", None),
                                "epoch",
                                None,
                            ),
                        ):
                            self.last_work_item_failed_viewpoint_skips += 1
                            return None
                if is_portal_probe:
                    probe_id = getattr(
                        getattr(candidate, "portal_observation_probe", None),
                        "probe_id",
                        None,
                    )
                    probe_ledger = getattr(self, "portal_probe_ledger", None)
                    if probe_id is not None and probe_ledger is not None:
                        viewpoint_xy = (
                            (candidate.x, candidate.y)
                            if physical_xy is None else physical_xy
                        )
                        if not probe_ledger.viewpoint_available(
                            probe_id, viewpoint_xy,
                        ):
                            self.last_work_item_failed_viewpoint_skips += 1
                            return None
                elif candidate.work_item_id is not None:
                    if not ledger.work_item_is_available(candidate.work_item_id):
                        if candidate.work_item_match == "resolved_descendant":
                            self.last_work_item_resolved_descendant_skips += 1
                        return None
                    viewpoint_xy = (
                        (candidate.x, candidate.y)
                        if physical_xy is None else physical_xy
                    )
                    if not ledger.viewpoint_is_available(
                        candidate.work_item_id,
                        viewpoint_xy,
                        map_epoch=getattr(
                            getattr(request, "components", None),
                            "epoch",
                            None,
                        ),
                    ):
                        # The semantic boundary remains unresolved. Only this
                        # previously failed odom-frame viewpoint is excluded,
                        # so another safe approach along the same frontier arc
                        # can be selected on this or a later map snapshot.
                        self.last_work_item_failed_viewpoint_skips += 1
                        return None
                elif not ledger.candidate_is_available(
                    context.source_place_id,
                    (candidate.x, candidate.y) if physical_xy is None else physical_xy,
                ):
                    return None
        else:
            candidate_kwargs = {"component": component}
            if physical_xy is not None:
                candidate_kwargs["physical_xy"] = physical_xy
            region_tier, region = self.region_memory.candidate_tier(
                candidate.x,
                candidate.y,
                information,
                **candidate_kwargs
            )
        if region_tier == "dormant":
            return None
        if region_tier == "ready_to_exit":
            self.last_ready_to_exit_reentry_skips += 1
            return None
        if (
            not (is_work_item_rehydration or is_portal_probe)
            and self.region_memory.is_observed_region_reentry(
                region,
                context.source_component,
                component,
            )
        ):
            # A durable WorkItem is already owned by the current physical
            # Place. Its endpoint may land in a different transient SLAM
            # component after map growth, but that is not a Place re-entry.
            # Applying this guard here discarded valid rehydrated candidates
            # after they had passed the BFS/costmap/ledger checks, leaving the
            # graph planner with a different WorkItem than its report.
            self.last_observed_place_reentry_skips += 1
            return None

        pool_name = "new" if region_tier == "new" else "revisit"
        if (
            request.allowed_region_tiers is not None
            and pool_name not in request.allowed_region_tiers
        ):
            return None
        return pool_name, component
