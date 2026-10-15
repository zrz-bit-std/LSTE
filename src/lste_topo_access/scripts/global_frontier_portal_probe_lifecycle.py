"""ROS boundary for the durable source-side Portal probe ledger.

The probe ledger deliberately stops at an observation fact.  This module is
also the boundary where that fact is promoted into a graph Portal: a
destination-side view is enough to create a *certified hypothesis*, but it is
not a crossing.  Keeping the promotion here prevents the planner from
confusing a camera/lidar observation with a physical Place transition.
"""

import math

from global_frontier_portal_crossing import portal_crossing_depth

from global_frontier_portal_probes import PortalObservationProbe
from global_frontier_directional_branch_coverage import (
    DirectionalBranchKey,
    EVIDENCE_DESTINATION_VIEW,
)


class GlobalFrontierPortalProbeLifecycleMixin:
    """Bind map-derived probe hypotheses to route Attempts."""

    @staticmethod
    def _directional_branch_key(source_place_id, portal_id):
        """Build the stable coverage identity for one directed Portal."""
        return DirectionalBranchKey(
            int(source_place_id), "portal:%d" % int(portal_id)
        )

    def _record_directional_branch_view(
        self, record, portal_id, now, *, event_id=None,
    ):
        """Record destination-view coverage without changing route state."""
        coverage = getattr(self, "directional_branch_coverage", None)
        if coverage is None or not isinstance(record, dict):
            return None
        try:
            source_place_id = int(record["source_place_id"])
            portal_id = int(portal_id)
        except (KeyError, TypeError, ValueError):
            return None
        key = self._directional_branch_key(source_place_id, portal_id)
        frontier_xy = record.get("map_gate_xy")
        normal_xy = record.get("normal_xy")
        if frontier_xy is None:
            snapshot = coverage.register(key, now=now)
        else:
            snapshot = coverage.observe_frontier(
                key,
                frontier_xy,
                direction_xy=normal_xy,
                map_epoch=record.get("last_map_epoch"),
                now=now,
            )
        return coverage.complete_branch(
            key,
            evidence_kind=EVIDENCE_DESTINATION_VIEW,
            source="portal_probe",
            event_id=(
                event_id
                or "portal_probe:%d:destination:%d"
                % (int(record.get("id", 0)), int(record.get("attempt_count", 0)))
            ),
            now=now,
        )

    def mark_directional_branch_transit(
        self, source_place_id, portal_id, destination_place_id, now,
        *, event_id=None,
    ):
        """Commit crossing as transit while retaining the branch lineage."""
        coverage = getattr(self, "directional_branch_coverage", None)
        if coverage is None:
            return None
        try:
            key = self._directional_branch_key(source_place_id, portal_id)
        except (TypeError, ValueError):
            return None
        coverage.register(key, now=now)
        return coverage.mark_transit(
            key,
            int(destination_place_id),
            source="portal_crossing",
            event_id=(
                event_id
                or "portal:%d:crossing:%d"
                % (int(portal_id), int(destination_place_id))
            ),
            now=now,
        )

    def register_portal_probe(self, request, probe):
        """Attach a physical identity to a map opening when possible."""
        ledger = getattr(self, "portal_probe_ledger", None)
        source_place_id = getattr(self, "current_physical_place_id", None)
        if ledger is None or source_place_id is None:
            return probe
        try:
            map_gate = self.cell_xy(
                request.message, *tuple(probe.opening_cell),
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            return None
        if map_gate is None:
            return None
        physical_gate = map_gate
        projector = getattr(request, "map_to_physical_xy", None)
        if projector is not None:
            physical_gate = projector(map_gate[0], map_gate[1])
        if physical_gate is None:
            return None
        # ``PortalObservationProbe.normal`` is a grid direction (row, col),
        # while the durable ledgers store physical/map vectors as (x, y).
        # Convert exactly at this boundary so wall geometry and viewpoint
        # compilation cannot silently rotate a probe by 90 degrees.
        try:
            normal_xy = (
                float(probe.normal[1]), float(probe.normal[0])
            )
        except (IndexError, TypeError, ValueError):
            return None
        # Once a physical edge has been crossed, its reverse boundary is
        # transit evidence, not a new exploration obligation. The strict gate
        # merge radius comes from Place memory; it is intentionally smaller
        # than the coarse map-association radius used for SLAM drift.
        hypothesis_ledger = getattr(self, "portal_hypothesis_ledger", None)
        reverse_edge = None
        suppression_reason = "reverse_side_of_crossed_portal_is_transit"
        reverse_query = (
            None
            if hypothesis_ledger is None
            else getattr(
                hypothesis_ledger,
                "crossed_reverse_for_destination",
                None,
            )
        )
        if callable(reverse_query):
            memory = getattr(self, "region_memory", None)
            gate_radius = getattr(memory, "portal_entry_merge_radius", None)
            reverse_edge = reverse_query(
                source_place_id,
                physical_gate,
                normal_xy,
                gate_radius=gate_radius,
            )
            # A wide doorway can be observed at a different jamb and with a
            # transiently flipped normal after SLAM updates.  If the strict
            # half-plane test misses it, the durable gate-band identity still
            # makes this a transit boundary, never a new probe.
            if reverse_edge is None:
                band_query = getattr(
                    hypothesis_ledger,
                    "crossed_gate_for_destination",
                    None,
                )
                if callable(band_query):
                    gate_radius = getattr(
                        memory, "portal_entry_match_radius", gate_radius,
                    )
                    reverse_edge = band_query(
                        source_place_id,
                        physical_gate,
                        gate_radius=gate_radius,
                    )
                    if reverse_edge is not None:
                        suppression_reason = (
                            "same_wide_gate_as_crossed_portal_is_transit"
                        )
            if reverse_edge is None:
                physical_gate_query = getattr(
                    hypothesis_ledger,
                    "crossed_gate_at_physical_gate",
                    None,
                )
                if callable(physical_gate_query):
                    reverse_edge = physical_gate_query(physical_gate)
                    if reverse_edge is not None:
                        suppression_reason = (
                            "same_physical_gate_as_crossed_portal"
                        )
        if reverse_edge is not None:
            reports = getattr(self, "_portal_probe_suppression_reports", None)
            if reports is None:
                reports = set()
                self._portal_probe_suppression_reports = reports
            signature = (int(source_place_id), int(reverse_edge["id"]))
            if signature not in reports:
                reports.add(signature)
                self.publish_status(
                    "portal_probe_suppressed",
                    source_place_id=int(source_place_id),
                    portal_id=int(reverse_edge["id"]),
                    gate=[
                        round(float(physical_gate[0]), 3),
                        round(float(physical_gate[1]), 3),
                    ],
                    reason=suppression_reason,
                )
            return None
        observe_kwargs = {
            "map_gate_xy": map_gate,
            "opening_cell": probe.opening_cell,
            "map_epoch": getattr(
                getattr(request, "components", None), "epoch", None
            ),
            "now": request.now,
            "observation_source": getattr(
                probe, "observation_source", "frontier"
            ),
        }
        try:
            record = ledger.observe(
                source_place_id,
                physical_gate,
                normal_xy,
                **observe_kwargs,
            )
        except TypeError:
            # Keep injected/older ledgers source-compatible while production
            # uses the explicit provenance field.
            observe_kwargs.pop("observation_source", None)
            record = ledger.observe(
                source_place_id,
                physical_gate,
                normal_xy,
                **observe_kwargs,
            )
        if record is None or not ledger.available(record["id"]):
            # Registration is a source-side identity binding.  Once the
            # durable probe reaches ``source_arrived``, the destination phase
            # is owned by the dedicated rehydration selector; registering the
            # same transient frontier again would create a second owner.
            return None
        state = str(record.get("state", "")).strip().lower()
        return PortalObservationProbe(
            tuple(probe.opening_cell),
            tuple(probe.normal),
            int(record["id"]),
            "destination" if state == "source_arrived" else "source",
            str(record.get("observation_source", "frontier")),
        )

    def register_portal_hypothesis_probe(
        self, request, portal_id, portal_gate_xy, destination_xy,
    ):
        """Create one source-side probe for a durable unbound Portal.

        A Portal hypothesis and its source-side observation are different
        facts.  Binding them when the hypothesis is first certified prevents a
        later planner pass from treating the same doorway as a direct crossing
        when the graph still requires a probe.
        """
        ledger = getattr(self, "portal_probe_ledger", None)
        source_place_id = getattr(self, "current_physical_place_id", None)
        if ledger is None or source_place_id is None:
            return None
        try:
            gate = (float(portal_gate_xy[0]), float(portal_gate_xy[1]))
            destination = (float(destination_xy[0]), float(destination_xy[1]))
        except (IndexError, TypeError, ValueError):
            return None
        normal_map = (destination[0] - gate[0], destination[1] - gate[1])
        projector = getattr(request, "map_to_physical_xy", None)
        physical_gate = gate
        physical_destination = destination
        if projector is not None:
            physical_gate = projector(gate[0], gate[1])
            physical_destination = projector(destination[0], destination[1])
        if physical_gate is None or physical_destination is None:
            return None
        try:
            normal = (
                float(physical_destination[0]) - float(physical_gate[0]),
                float(physical_destination[1]) - float(physical_gate[1]),
            )
            if normal[0] * normal[0] + normal[1] * normal[1] <= 1e-9:
                normal = normal_map
        except (IndexError, TypeError, ValueError):
            normal = normal_map
        opening_cell = None
        xy_to_cell = getattr(self, "xy_to_grid_cell", None)
        if xy_to_cell is not None:
            try:
                opening_cell = xy_to_cell(request.message, gate[0], gate[1])
            except (AttributeError, TypeError, ValueError):
                opening_cell = None
        record = ledger.observe(
            source_place_id,
            physical_gate,
            normal,
            map_gate_xy=gate,
            opening_cell=opening_cell,
            map_epoch=getattr(getattr(request, "components", None), "epoch", None),
            now=request.now,
        )
        if record is None:
            return None
        prior_portal_id = record.get("portal_id")
        bound = getattr(ledger, "bind_portal", None)
        if bound is not None:
            record = bound(record["id"], portal_id)
        if (
            prior_portal_id != int(portal_id)
            and record is not None
        ):
            self.publish_status(
                "portal_probe_bound",
                probe_id=int(record["id"]),
                portal_id=int(portal_id),
                source_place_id=int(source_place_id),
                reason="unbound_portal_requires_source_probe",
            )
        return record

    def start_active_portal_probe(self, selection, now, route_id=None):
        """Reserve the selected probe at route activation time."""
        self.active_portal_probe_id = None
        self.active_portal_probe_phase = ""
        probe = getattr(selection, "portal_observation_probe", None)
        if probe is None:
            return None
        ledger = getattr(self, "portal_probe_ledger", None)
        probe_id = getattr(probe, "probe_id", None)
        if ledger is None or probe_id is None:
            return None
        viewpoint = (float(selection.x), float(selection.y))
        map_message = getattr(self, "map_msg", None)
        transform = getattr(self, "transform_xy", None)
        if map_message is not None and transform is not None:
            map_frame = getattr(
                getattr(map_message, "header", None), "frame_id", "map",
            ) or "map"
            projected = transform("odom", map_frame, viewpoint[0], viewpoint[1])
            if projected is not None:
                viewpoint = projected
        probe_phase = str(getattr(probe, "phase", "source") or "source")
        component = getattr(selection, "component", None)
        map_epoch = (
            component.get("epoch")
            if isinstance(component, dict) else getattr(component, "epoch", None)
        )
        try:
            viewpoint_available = ledger.viewpoint_available(
                probe_id,
                viewpoint,
                phase=probe_phase,
                map_epoch=map_epoch,
            )
        except TypeError:
            # Keep test doubles and older injected ledgers source-compatible.
            viewpoint_available = ledger.viewpoint_available(probe_id, viewpoint)
        if not viewpoint_available:
            reject = getattr(ledger, "reject_viewpoint", None)
            if callable(reject):
                reject(
                    probe_id,
                    viewpoint,
                    phase=probe_phase,
                    reason="viewpoint_already_rejected",
                    now=now,
                )
            self.publish_status(
                "portal_probe_activation_rejected",
                probe_id=int(probe_id),
                reason="viewpoint_already_attempted",
                phase=probe_phase,
                viewpoint=(
                    None if viewpoint is None else [
                        round(float(viewpoint[0]), 3),
                        round(float(viewpoint[1]), 3),
                    ]
                ),
            )
            return None
        destination_start = getattr(ledger, "start_destination", None)
        probe_available = getattr(ledger, "available", lambda _probe_id: True)
        if callable(destination_start) and not probe_available(probe_id):
            try:
                record = destination_start(
                    probe_id,
                    now=now,
                    viewpoint_xy=viewpoint,
                    map_epoch=map_epoch,
                )
            except TypeError:
                record = destination_start(
                    probe_id, now=now, viewpoint_xy=viewpoint,
                )
        else:
            try:
                record = ledger.start(
                    probe_id,
                    now=now,
                    viewpoint_xy=viewpoint,
                    map_epoch=map_epoch,
                )
            except TypeError:
                record = ledger.start(
                    probe_id, now=now, viewpoint_xy=viewpoint,
                )
        if record is None:
            reject = getattr(ledger, "reject_viewpoint", None)
            if callable(reject):
                reject(
                    probe_id,
                    viewpoint,
                    phase=probe_phase,
                    reason="probe_lease_unavailable",
                    now=now,
                )
            self.publish_status(
                "portal_probe_activation_rejected",
                probe_id=int(probe_id),
                reason="probe_lease_unavailable",
                phase=probe_phase,
            )
            return None
        self.active_portal_probe_id = int(record["id"])
        self.active_portal_probe_phase = str(
            record.get("active_phase") or getattr(probe, "phase", "source")
            or "source"
        ).strip().lower()
        self.publish_status(
            "portal_probe_started",
            probe_id=int(record["id"]),
            source_place_id=int(record["source_place_id"]),
            opening_cell=record.get("opening_cell"),
            normal=record.get("normal_xy"),
            phase=record.get("active_phase") or "source",
            portal_probe_phase=self.active_portal_probe_phase,
            route_id=int(
                getattr(self, "active_route_id", 0)
                if route_id is None else route_id
            ),
        )
        return record

    def settle_active_portal_probe(self, result, now, reason):
        """Finish the current probe Attempt without changing Portal legality."""
        probe_id = getattr(self, "active_portal_probe_id", None)
        ledger = getattr(self, "portal_probe_ledger", None)
        if probe_id is None or ledger is None:
            return None
        record = ledger.finish(probe_id, result, now=now, reason=reason)
        if record is not None:
            phase = str(
                record.get("last_phase") or record.get("active_phase") or "source"
            ).strip().lower()
            promoted = None
            if (
                str(result).strip().lower() == "observed"
                and phase == "destination"
            ):
                # A destination view is the second independent fact needed to
                # promote an opening into a durable Portal hypothesis.  It is
                # intentionally separate from ``crossed``: the next action
                # still has to pass Navfn/TEB and the odometry gate proof.
                promoted = self.promote_destination_view(record, now)
                if promoted is not None:
                    refreshed = ledger.get(probe_id)
                    if refreshed is not None:
                        record = refreshed
            self.publish_status(
                "portal_probe_settled",
                probe_id=int(record["id"]),
                source_place_id=int(record["source_place_id"]),
                state=str(record["state"]),
                attempt_count=int(record["attempt_count"]),
                result=str(result),
                reason=str(reason),
                phase=phase,
                portal_probe_phase=phase,
                route_id=int(getattr(self, "active_route_id", 0)),
            )
            if str(result).strip().lower() == "observed" and phase == "destination":
                self.publish_status(
                    "portal_destination_view_observed",
                    portal_probe_id=int(record["id"]),
                    portal_id=record.get("portal_id"),
                    source_place_id=int(record["source_place_id"]),
                    route_id=int(getattr(self, "active_route_id", 0)),
                    portal_probe_phase=phase,
                    reason=str(reason),
                    portal_hypothesis_id=(
                        None if promoted is None else int(promoted["id"])
                    ),
                )
            if str(result).strip().lower() in ("certified", "observed"):
                self._settle_portal_probe_work_item(
                    record,
                    now,
                    (
                        "portal_probe_certified"
                        if str(result).strip().lower() == "certified"
                        else "destination_viewpoint_observed"
                    ),
                )
        self.active_portal_probe_id = None
        self.active_portal_probe_phase = ""
        return record

    def promote_destination_view(self, record, now):
        """Create/reuse the graph Portal proved by a two-view probe.

        ``record`` is a physical gate identity, so the promotion remains
        stable across SLAM map corrections.  The destination point is a
        directed geometric witness derived from the existing portal crossing
        contract; it is not treated as the robot having crossed the gate.
        """
        if not isinstance(record, dict):
            return None
        hypothesis_ledger = getattr(self, "portal_hypothesis_ledger", None)
        probe_ledger = getattr(self, "portal_probe_ledger", None)
        if hypothesis_ledger is None:
            return None
        probe_id = record.get("id")
        existing_id = record.get("portal_id")
        if existing_id not in (None, ""):
            try:
                existing = hypothesis_ledger.get(int(existing_id))
            except (TypeError, ValueError):
                existing = None
            if existing is not None:
                self.last_portal_hypothesis_id = int(existing["id"])
                self._record_directional_branch_view(
                    record,
                    int(existing["id"]),
                    now,
                )
                return existing

        try:
            gate = (
                float(record["physical_gate_xy"][0]),
                float(record["physical_gate_xy"][1]),
            )
            normal = (
                float(record["normal_xy"][0]),
                float(record["normal_xy"][1]),
            )
        except (KeyError, IndexError, TypeError, ValueError):
            return None
        length = math.hypot(normal[0], normal[1])
        if not math.isfinite(length) or length <= 1e-9:
            return None
        normal = normal[0] / length, normal[1] / length
        resolution = 0.0
        message = getattr(self, "map_msg", None)
        if message is not None:
            try:
                resolution = max(0.0, float(message.info.resolution))
            except (AttributeError, TypeError, ValueError):
                resolution = 0.0
        depth = portal_crossing_depth(getattr(self, "clearance", 0.0))
        # Half a cell makes the physical witness independent of which side of
        # a rasterized gate supplied the frontier observation.  This reuses an
        # existing geometric contract rather than adding a tuning parameter.
        depth += 0.5 * resolution
        destination = (
            gate[0] + depth * normal[0],
            gate[1] + depth * normal[1],
        )
        map_gate = record.get("map_gate_xy")
        try:
            map_gate = (
                float(map_gate[0]), float(map_gate[1]),
            ) if map_gate is not None else None
        except (IndexError, TypeError, ValueError):
            map_gate = None
        hypothesis, created = hypothesis_ledger.certify(
            int(record["source_place_id"]),
            gate,
            destination,
            now=now,
            task_version=str(getattr(self, "current_task_version", "") or ""),
            map_gate_xy=map_gate,
        )
        if hypothesis is None:
            return None
        if probe_ledger is not None and probe_id is not None:
            probe_ledger.bind_portal(probe_id, int(hypothesis["id"]))
        self.last_portal_hypothesis_id = int(hypothesis["id"])
        self._record_directional_branch_view(
            record,
            int(hypothesis["id"]),
            now,
        )
        if created:
            self.publish_status(
                "portal_hypothesis_certified",
                portal_id=int(hypothesis["id"]),
                portal_probe_id=int(record["id"]),
                source_place_id=int(record["source_place_id"]),
                gate=[round(gate[0], 3), round(gate[1], 3)],
                destination=[round(destination[0], 3), round(destination[1], 3)],
                evidence_source="two_view_destination_probe",
            )
        self.publish_status(
            "portal_probe_promoted",
            portal_probe_id=int(record["id"]),
            portal_id=int(hypothesis["id"]),
            source_place_id=int(record["source_place_id"]),
            evidence="source_view+destination_view",
            crossing_required=True,
        )
        return hypothesis

    def _settle_portal_probe_work_item(self, record, now, reason):
        """Resolve the generic boundary only after Portal evidence is final."""
        work_item_id = record.get("work_item_id")
        work_items = getattr(self, "place_work_items", None)
        if work_item_id is None or work_items is None:
            return None
        resolved = work_items.resolve(work_item_id, now, reason)
        if resolved is not None:
            self.publish_status(
                "portal_probe_work_item_settled",
                probe_id=int(record["id"]),
                work_item_id=int(work_item_id),
                place_id=int(record["source_place_id"]),
                reason=str(reason),
            )
        return resolved

    def certify_portal_probe(
        self, request, portal_gate_xy, destination_xy, portal_id, now,
    ):
        """Close a source-arrived probe when its directed Portal is proven."""
        ledger = getattr(self, "portal_probe_ledger", None)
        source_place_id = getattr(self, "current_physical_place_id", None)
        if ledger is None or source_place_id is None:
            return None
        projector = getattr(request, "map_to_physical_xy", None)
        physical_gate = portal_gate_xy
        physical_destination = destination_xy
        if projector is not None:
            physical_gate = projector(portal_gate_xy[0], portal_gate_xy[1])
            physical_destination = projector(
                destination_xy[0], destination_xy[1],
            )
        if physical_gate is None or physical_destination is None:
            return None
        direction = (
            float(physical_destination[0]) - float(physical_gate[0]),
            float(physical_destination[1]) - float(physical_gate[1]),
        )
        record = ledger.certify_matching(
            source_place_id,
            physical_gate,
            direction,
            portal_id,
            now=now,
        )
        if record is None:
            return None
        self.publish_status(
            "portal_probe_certified",
            probe_id=int(record["id"]),
            portal_id=int(portal_id),
            source_place_id=int(source_place_id),
            normal=record.get("normal_xy"),
            reason="directed_portal_certified",
        )
        self._settle_portal_probe_work_item(
            record, now, "portal_probe_certified",
        )
        return record
