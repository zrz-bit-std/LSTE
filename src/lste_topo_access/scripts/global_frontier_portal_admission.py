"""Durable destination checks for a proposed cross-place portal action."""

import numpy as np

from global_frontier_place_states import PLACE_DORMANT, PLACE_READY_TO_EXIT
from global_frontier_place_progress import covered_destination_progress
from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_portal_crossing import portal_source_side_is_proven


class GlobalFrontierPortalAdmissionMixin:
    """Reject portal actions that would physically revisit covered space."""

    def _covered_portal_transit_is_allowed(
        self, source_region, destination_region, snapshot_progress=False,
    ):
        """Permit a covered edge only when its Place graph has progress."""
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        if not PortalHypothesisLedger.place_is_covered(
            source_region
        ) or not PortalHypothesisLedger.place_is_covered(destination_region):
            return True
        if ledger is None:
            return False
        # Unknown cells are short-lived SLAM evidence.  They become a valid
        # transit reason only after the destination Place owns a persistent
        # WorkItem or Portal obligation.  This prevents a stale/partial map
        # boundary from authorizing an endless covered-room loop.
        if snapshot_progress:
            progress = covered_destination_progress(
                destination_region,
                work_item_ledger=getattr(self, "place_work_items", None),
                portal_probe_ledger=getattr(self, "portal_probe_ledger", None),
            )
            snapshot_progress = bool(
                progress is not None and progress.has_durable_progress
            )
        return ledger.allows_covered_transit(
            source_region,
            destination_region,
            place_memory=getattr(self, "region_memory", None),
            work_item_ledger=getattr(self, "place_work_items", None),
            portal_probe_ledger=getattr(self, "portal_probe_ledger", None),
            snapshot_progress=snapshot_progress,
        )

    def _portal_reverse_egress(
        self, request, portal_gate_xy, destination_xy, physical_gate,
        physical_destination,
    ):
        """Return the documented reverse entry edge, if this is one."""
        lookup = getattr(self.region_memory, "portal_exit_from_entry", None)
        if lookup is None or portal_gate_xy is None:
            return None
        physical_source = (
            None
            if request.map_to_physical_xy is None
            else request.map_to_physical_xy(
                request.route_anchor_xy[0], request.route_anchor_xy[1],
            )
        )
        if (
            physical_gate is None
            or physical_source is None
            or physical_destination is None
        ):
            return lookup(
                portal_gate_xy,
                request.route_anchor_xy,
                destination_xy,
            )
        return lookup(
            portal_gate_xy,
            request.route_anchor_xy,
            destination_xy,
            physical_gate_xy=physical_gate,
            physical_source_xy=physical_source,
            physical_destination_xy=physical_destination,
        )

    @staticmethod
    def _destination_has_live_unknown(request, component):
        """Use the current grid to prove unknown space beyond a covered Place."""
        unknown = getattr(request, "unknown", None)
        components = getattr(request, "components", None)
        labels = getattr(components, "labels", None)
        if unknown is None or labels is None or component is None:
            return False
        if labels.shape != unknown.shape:
            return False
        try:
            label = int(component["label"])
        except (KeyError, TypeError, ValueError):
            return False
        if label <= 0:
            return False
        touching = np.zeros_like(unknown, dtype=bool)
        touching[1:, :] |= unknown[:-1, :]
        touching[:-1, :] |= unknown[1:, :]
        touching[:, 1:] |= unknown[:, :-1]
        touching[:, :-1] |= unknown[:, 1:]
        return bool(np.any((labels == label) & touching))

    def portal_destination_is_admissible(
        self, request, portal_gate_xy, destination_xy, component, information,
        hypothesis=None,
    ):
        """Accept an unseen destination or the exact reverse of an entry edge.

        A structural component may temporarily split around a desk even though
        its map coordinates differ from every saved component label.  The
        physical observation viewpoint is therefore a separate *admission*
        proof: it never merges region identities, but it does prevent a
        doorway action from being sent back into already observed floor space.
        """
        physical_source = (
            None
            if request.map_to_physical_xy is None
            else request.map_to_physical_xy(
                request.route_anchor_xy[0], request.route_anchor_xy[1],
            )
        )
        physical_destination = (
            None
            if request.map_to_physical_xy is None
            else request.map_to_physical_xy(
                destination_xy[0], destination_xy[1],
            )
        )
        physical_gate = (
            None
            if portal_gate_xy is None or request.map_to_physical_xy is None
            else request.map_to_physical_xy(portal_gate_xy[0], portal_gate_xy[1])
        )
        # A portal is a directed edge, not merely a narrow map cell. Prefer
        # the physical snapshot when available; otherwise map coordinates from
        # this one snapshot preserve the same signed side under map->odom's
        # rigid transform. Reject it before dispatch unless its current source
        # lies opposite the certified gate from its destination. Waiting until
        # the terminal to learn this leaves the robot in a completed room and
        # is the direct cause of repeated door retries and long dwell.
        direction_source = physical_source
        direction_gate = physical_gate
        direction_destination = physical_destination
        if any(value is None for value in (
            direction_source, direction_gate, direction_destination,
        )):
            direction_source = request.route_anchor_xy
            direction_gate = portal_gate_xy
            direction_destination = destination_xy
        if not portal_source_side_is_proven(
            direction_source, direction_gate, direction_destination,
        ):
            self.last_portal_source_side_rejections = (
                int(getattr(self, "last_portal_source_side_rejections", 0)) + 1
            )
            return False
        candidate_kwargs = {"component": component}
        if physical_destination is not None:
            candidate_kwargs["physical_xy"] = physical_destination
        durable_entry = getattr(
            self.region_memory, "portal_entry_region", None,
        )
        known_entry = None
        if durable_entry is not None:
            known_entry = durable_entry(
                portal_gate_xy,
                destination_xy,
                component=component,
                physical_gate_xy=physical_gate,
                physical_destination_xy=physical_destination,
            )
        if known_entry is not None:
            destination_region = known_entry[0]
            destination_tier = str(
                destination_region.get("state", "revisit")
            )
        else:
            destination_tier, destination_region = self.region_memory.candidate_tier(
                destination_xy[0],
                destination_xy[1],
                information,
                **candidate_kwargs
            )
        # Once a Portal has crossed, its destination Place is durable. Do not
        # let a fresh SLAM component relabel that same physical endpoint as a
        # new room and thereby bypass the graph's re-entry rules.
        if hypothesis is not None:
            try:
                bound_destination = hypothesis.get("destination_place_id")
                if bound_destination is not None:
                    durable_region = self.region_memory.by_id(
                        int(bound_destination)
                    )
                    if durable_region is not None:
                        destination_region = durable_region
                        destination_tier = str(
                            durable_region.get("state", destination_tier)
                        )
            except (AttributeError, TypeError, ValueError):
                pass

        covered_region = destination_region
        physical_coverage = getattr(
            self.region_memory, "physical_coverage_region", None,
        )
        if physical_coverage is not None and physical_destination is not None:
            observed = physical_coverage(
                physical_destination,
                self.completed_radius,
            )
            if observed is not None:
                covered_region = observed

        is_covered_destination = (
            covered_region is not None
            and (
                destination_tier in ("dormant", "ready_to_exit")
                or covered_region.get("state") in (
                    PLACE_DORMANT,
                    PLACE_READY_TO_EXIT,
                )
                or int(covered_region.get("endpoint_observations", 0)) > 0
            )
        )
        self.last_portal_destination_class = (
            "covered_transit" if is_covered_destination else "new_place"
        )
        if not is_covered_destination:
            return True

        reverse_exit = self._portal_reverse_egress(
            request,
            portal_gate_xy,
            destination_xy,
            physical_gate,
            physical_destination,
        )
        if reverse_exit is None:
            # Discovering this only at the action terminal costs an entire
            # room traversal. Admission is the point where the route can be
            # rejected without moving the base.
            self.last_portal_covered_destination_skips = (
                int(getattr(self, "last_portal_covered_destination_skips", 0))
                + 1
            )
            return False

        if not self._covered_portal_transit_is_allowed(
            reverse_exit[0],
            covered_region,
            snapshot_progress=self._destination_has_live_unknown(
                request, component,
            ),
        ):
            self.last_portal_covered_destination_skips = (
                int(getattr(self, "last_portal_covered_destination_skips", 0))
                + 1
            )
            self.last_portal_covered_cycle_skips = (
                int(getattr(self, "last_portal_covered_cycle_skips", 0)) + 1
            )
            return False

        self.last_portal_reverse_egress_count = (
            int(getattr(self, "last_portal_reverse_egress_count", 0)) + 1
        )
        self.last_portal_reverse_egress_region_id = int(reverse_exit[0]["id"])
        return True
