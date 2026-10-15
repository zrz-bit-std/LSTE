"""Build certified portal actions from frontier and place-graph evidence."""

import math

import numpy as np

from global_frontier_models import FrontierCandidateRoute, PortalSource
from global_frontier_portal_certification import (
    certified_adjacent_place_transitions,
    portal_certification_window,
    portal_transition_verified_exit_cell,
)
from global_frontier_portal_crossing import portal_crossing_depth
from global_frontier_portal_probes import (
    PortalObservationProbe,
    portal_observation_probe,
)
from global_frontier_structural_boundaries import (
    structural_boundary_candidates,
)


class GlobalFrontierCandidatePortalMixin:
    """Own portal endpoint construction; portal ranking lives elsewhere."""

    def source_portal_probe_viewpoint_from_map_normal(
        self, request, gate_xy, normal_xy,
    ):
        """Compile a safe source-side standoff from a map-frame normal."""
        if gate_xy is None or normal_xy is None:
            return None
        try:
            normal_x, normal_y = float(normal_xy[0]), float(normal_xy[1])
            length = math.hypot(normal_x, normal_y)
            if length <= 1e-9:
                return None
            normal_x, normal_y = normal_x / length, normal_y / length
            resolution = max(1e-6, float(request.message.info.resolution))
            depth = (
                portal_crossing_depth(getattr(self, "clearance", 0.0))
                + float(getattr(self, "teb_xy_goal_tolerance", 0.35))
                + 0.5 * resolution
            )
            desired = (
                float(gate_xy[0]) - depth * normal_x,
                float(gate_xy[1]) - depth * normal_y,
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            return None

        # Keep the endpoint on the same strict route/costmap layer as every
        # other candidate. ``nearest_reachable_cell`` is a physical-map
        # reassociation helper, not a permission to enter a low-clearance cell.
        # The fallback keeps this small geometry helper independently usable in
        # ROS-free adapters and tests that only provide the portal mixin.
        reassociate = getattr(self, "nearest_reachable_cell", None)
        if callable(reassociate):
            target = reassociate(
                request.message, request.steps, desired[0], desired[1],
            )
        else:
            target = self._grid_cell_if_reachable(
                request.message, request.steps, desired[0], desired[1],
            )
        if target is None or request.steps[target] < 0:
            return None
        preferred_steps = getattr(request, "preferred_steps", None)
        if (
            request.selection_tier == "strict_clearance"
            and (
                preferred_steps is not None
                and preferred_steps[target] < 0
            )
        ):
            return None
        return target

    @staticmethod
    def _grid_cell_if_reachable(message, steps, x, y):
        """Map one world point to itself when no reassociation helper exists."""
        if steps is None or not hasattr(steps, "shape") or len(steps.shape) != 2:
            return None
        try:
            resolution = float(message.info.resolution)
            origin = message.info.origin.position
            if resolution <= 0.0:
                return None
            col = int(math.floor((float(x) - float(origin.x)) / resolution))
            row = int(math.floor((float(y) - float(origin.y)) / resolution))
        except (AttributeError, TypeError, ValueError, ZeroDivisionError):
            return None
        if not (0 <= row < steps.shape[0] and 0 <= col < steps.shape[1]):
            return None
        return row, col

    def source_portal_probe_viewpoint(self, request, probe, gate_xy):
        """Return a safe source-side standoff cell for one Portal probe.

        ``PortalObservationProbe.normal`` is a grid (row, col) direction;
        convert it before compiling the physical/map viewpoint. The
        destination phase keeps its own viewpoint ladder in the durable-probe
        adapter.
        """
        if probe is None or str(getattr(probe, "phase", "source")) != "source":
            return None
        try:
            normal_xy = (float(probe.normal[1]), float(probe.normal[0]))
        except (AttributeError, IndexError, TypeError, ValueError):
            return None
        return self.source_portal_probe_viewpoint_from_map_normal(
            request, gate_xy, normal_xy,
        )

    def compile_source_portal_probe_route(self, request, probe, gate_xy):
        """Return route geometry for a source-side probe standoff."""
        if probe is None or str(getattr(probe, "phase", "source")) != "source":
            return None
        viewpoint = self.source_portal_probe_viewpoint(request, probe, gate_xy)
        if viewpoint is None:
            return None
        target_row, target_col = viewpoint
        x, y = self.cell_xy(request.message, target_row, target_col)
        costmap_distance = self.candidate_costmap_distance(
            request.validation, x, y,
        )
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
        return (
            target_row,
            target_col,
            x,
            y,
            costmap_distance,
            path_distance,
            score_path_distance,
        )

    def portal_probe_viewpoint_is_local(
        self, gate_xy, viewpoint_xy, resolution=0.0,
    ):
        """Require a probe endpoint to remain in the doorway neighborhood.

        A rehydrated probe may find a frontier cell near the physical gate but
        no safe approach cell there.  Falling back to the robot-rooted nearest
        cell would turn a doorway observation into an unrelated room route.
        The bound reuses existing observation geometry; it is not a new tuning
        parameter.
        """
        if gate_xy is None or viewpoint_xy is None:
            return False
        try:
            distance = math.hypot(
                float(gate_xy[0]) - float(viewpoint_xy[0]),
                float(gate_xy[1]) - float(viewpoint_xy[1]),
            )
            approach = float(
                getattr(self, "frontier_approach_distance", 0.0)
            )
            completed = float(getattr(self, "completed_radius", 0.0))
            resolution = max(0.0, float(resolution))
        except (TypeError, ValueError, IndexError):
            return False
        return distance <= max(approach, completed) + resolution

    @staticmethod
    def _finite_xy(value):
        """Return a finite planar pair for a durable portal projection."""
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            return None
        try:
            result = float(value[0]), float(value[1])
        except (TypeError, ValueError):
            return None
        return result if all(math.isfinite(item) for item in result) else None

    def _unbound_portal_map_xy(self, request, record, key):
        """Project a durable odom portal fact into the current map frame."""
        physical = self._finite_xy(record.get("physical_" + key))
        header = getattr(request.message, "header", None)
        map_frame = getattr(header, "frame_id", "map") or "map"
        transform = getattr(self, "transform_xy", None)
        if physical is not None and transform is not None:
            try:
                projected = self._finite_xy(
                    transform(map_frame, "odom", physical[0], physical[1])
                )
            except Exception:
                projected = None
            if projected is not None:
                return projected
        # A map projection is the last coherent snapshot when TF is
        # temporarily unavailable. Never treat odom as map coordinates here.
        return self._finite_xy(record.get("map_" + key))

    @staticmethod
    def _portal_map_cell(message, steps, xy):
        """Return an in-bounds grid cell for one projected portal point."""
        if xy is None or steps is None or len(steps.shape) != 2:
            return None
        try:
            resolution = float(message.info.resolution)
            origin = message.info.origin.position
            col = int(math.floor((xy[0] - float(origin.x)) / resolution))
            row = int(math.floor((xy[1] - float(origin.y)) / resolution))
        except (AttributeError, TypeError, ValueError, ZeroDivisionError):
            return None
        if not (0 <= row < steps.shape[0] and 0 <= col < steps.shape[1]):
            return None
        return row, col

    def _portal_source_for_unbound_hypothesis(self, request, record):
        """Rebuild one PortalSource from durable identity and current TF."""
        gate_xy = self._unbound_portal_map_xy(request, record, "gate")
        destination_xy = self._unbound_portal_map_xy(
            request, record, "destination",
        )
        gate_cell = self._portal_map_cell(request.message, request.steps, gate_xy)
        destination_cell = self._portal_map_cell(
            request.message, request.steps, destination_xy,
        )
        if gate_cell is None or destination_cell is None:
            return None
        nearest_reachable = getattr(self, "nearest_reachable_cell", None)
        endpoint = (
            None
            if nearest_reachable is None
            else nearest_reachable(
                request.message,
                request.steps,
                destination_xy[0],
                destination_xy[1],
            )
        )
        if nearest_reachable is None and request.steps[destination_cell] >= 0:
            endpoint = destination_cell
        if endpoint is None or request.steps[endpoint] < 0:
            return None
        return PortalSource(
            frontier_row=int(destination_cell[0]),
            frontier_col=int(destination_cell[1]),
            identity_row=int(destination_cell[0]),
            identity_col=int(destination_cell[1]),
            gate_row=int(gate_cell[0]),
            gate_col=int(gate_cell[1]),
            endpoint_row=int(endpoint[0]),
            endpoint_col=int(endpoint[1]),
            hypothesis_id=int(record["id"]),
        )

    def _populate_unbound_portal_sources(
        self, request, context, portal_sources,
    ):
        """Reproject still-unbound durable Portals for this source Place."""
        if context.source_place_id is None:
            return
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        if ledger is None:
            return
        query = getattr(ledger, "unbound_for_source", None)
        if query is None:
            return
        map_epoch = getattr(getattr(request, "components", None), "epoch", None)
        try:
            pending = query(context.source_place_id, map_epoch=map_epoch)
        except TypeError:
            # Keep the helper compatible with older injected ledgers that
            # expose only the source-place positional argument.
            pending = query(context.source_place_id)
        for record in pending:
            source = self._portal_source_for_unbound_hypothesis(request, record)
            if source is None:
                self.last_portal_unbound_reprojection_skips = int(
                    getattr(self, "last_portal_unbound_reprojection_skips", 0)
                ) + 1
                reject = getattr(ledger, "mark_unbound_rejected", None)
                if reject is not None:
                    reject(record["id"], map_epoch, "endpoint_not_reachable")
                continue
            refresh = getattr(ledger, "refresh_projection", None)
            if refresh is not None:
                refresh(
                    record["id"],
                    map_gate_xy=self._unbound_portal_map_xy(
                        request, record, "gate",
                    ),
                    map_destination_xy=self._unbound_portal_map_xy(
                        request, record, "destination",
                    ),
                    now=request.now,
                    task_version=str(
                        getattr(self, "current_task_version", "") or ""
                    ),
                )
            portal_sources.setdefault(("hypothesis", int(record["id"])), source)
            self.last_portal_unbound_reused = int(
                getattr(self, "last_portal_unbound_reused", 0)
            ) + 1

    def _populate_preferred_crossed_portal_source(
        self, request, context, portal_sources,
    ):
        """Reproject the graph-selected forward edge from durable evidence.

        A crossed Portal is normally recovered from the current structural
        labels.  That is only a snapshot-local convenience: after a SLAM
        correction the doorway can temporarily have no matching component at
        all.  When the graph planner owns a specific forward edge, its stored
        physical gate/destination are the stronger identity and can rebuild a
        ``PortalSource`` without inventing a new Portal or choosing another
        edge.  Reverse traversal remains owned by durable egress.
        """
        preferred = getattr(self, "graph_route_portal_id", None)
        if preferred is None or context.source_place_id is None:
            return
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        if ledger is None:
            return
        try:
            preferred = int(preferred)
        except (TypeError, ValueError):
            return
        get_record = getattr(ledger, "get", None)
        if get_record is None:
            get_record = getattr(ledger, "_by_id", None)
        if get_record is None:
            return
        record = get_record(preferred)
        if not isinstance(record, dict):
            return
        try:
            same_source = int(record.get("source_place_id")) == int(
                context.source_place_id
            )
            destination = int(record.get("destination_place_id"))
        except (TypeError, ValueError):
            return
        if (
            not same_source
            or destination <= 0
            or str(record.get("state", "")).strip().lower() != "crossed"
        ):
            return
        source = self._portal_source_for_unbound_hypothesis(request, record)
        if source is None:
            self.last_preferred_portal_reprojection_skips = int(
                getattr(self, "last_preferred_portal_reprojection_skips", 0)
            ) + 1
            return
        portal_sources.setdefault(("hypothesis", preferred), source)
        self.last_preferred_portal_reprojected = int(
            getattr(self, "last_preferred_portal_reprojected", 0)
        ) + 1

    def unresolved_portal_probe(self, request, frontier_row, frontier_col):
        """Turn an uncertified structural boundary into local probe evidence."""
        probe = self.portal_observation_probe_for_frontier(
            request, frontier_row, frontier_col,
        )
        if probe is not None:
            self.last_portal_probe_candidates += 1
        return probe

    def pending_portal_probe_candidates(
        self, request, context, frontier, approach_cells,
    ):
        """Rehydrate pending physical probes when frontier lineage moved.

        The ordinary candidate path starts from a current frontier arc.  After
        a SLAM correction that arc can disappear even though the physical
        doorway remains unresolved.  Reprojecting the durable probe gate and
        selecting the nearest current frontier keeps the information task
        alive without minting a new WorkItem or Place.
        """
        ledger = getattr(self, "portal_probe_ledger", None)
        source_place_id = getattr(context, "source_place_id", None)
        if ledger is None or source_place_id is None or frontier is None:
            return []
        pending_for_source = getattr(ledger, "pending_for_source", None)
        if pending_for_source is None:
            return []
        try:
            records = pending_for_source(
                source_place_id, include_source_arrived=True,
            )
        except TypeError:
            # Keep compatibility with injected ledgers from older callers.
            records = pending_for_source(source_place_id)
        if not records:
            return []
        frontier_cells = np.argwhere(frontier)
        if frontier_cells.size == 0:
            return []
        map_frame = getattr(
            getattr(request.message, "header", None), "frame_id", "map",
        ) or "map"
        transform = getattr(self, "transform_xy", None)
        map_epoch = getattr(
            getattr(request, "components", None), "epoch", None,
        )
        candidates = []
        for record in records:
            if getattr(self, "planning_should_preempt", lambda: False)():
                return []
            map_gate = self._finite_xy(record.get("map_gate_xy"))
            physical_gate = self._finite_xy(record.get("physical_gate_xy"))
            if physical_gate is not None and transform is not None:
                projected = self._finite_xy(
                    transform(map_frame, "odom", physical_gate[0], physical_gate[1])
                )
                if projected is not None:
                    map_gate = projected
            gate_cell = self._portal_map_cell(request.message, request.steps, map_gate)
            if gate_cell is None:
                continue
            distances = (
                (frontier_cells[:, 0] - gate_cell[0]) ** 2
                + (frontier_cells[:, 1] - gate_cell[1]) ** 2
            )
            for index in np.argsort(distances):
                if getattr(self, "planning_should_preempt", lambda: False)():
                    return []
                frontier_row, frontier_col = frontier_cells[int(index)]
                approach = self.nearest_safe_approach(
                    request.steps,
                    int(frontier_row),
                    int(frontier_col),
                    approach_cells,
                    preferred_steps=request.preferred_steps,
                    preferred_mask=request.preferred_mask,
                    selection_tier=request.selection_tier,
                )
                if approach is None:
                    continue
                target_row, target_col = approach
                x, y = self.cell_xy(request.message, target_row, target_col)
                if not self.portal_probe_viewpoint_is_local(
                    map_gate,
                    (x, y),
                    resolution=request.message.info.resolution,
                ):
                    self.last_portal_probe_viewpoint_rejections = int(
                        getattr(
                            self,
                            "last_portal_probe_viewpoint_rejections",
                            0,
                        )
                    ) + 1
                    continue
                if str(record.get("state", "")).strip().lower() != "source_arrived":
                    source_viewpoint = self.source_portal_probe_viewpoint_from_map_normal(
                        request,
                        map_gate,
                        record.get("normal_xy"),
                    )
                    if source_viewpoint is None:
                        continue
                    target_row, target_col = source_viewpoint
                    x, y = self.cell_xy(request.message, target_row, target_col)
                    if not self.portal_probe_viewpoint_is_local(
                        map_gate,
                        (x, y),
                        resolution=request.message.info.resolution,
                    ):
                        continue
                costmap_distance = self.candidate_costmap_distance(
                    request.validation, x, y,
                )
                if request.validation is not None and costmap_distance is None:
                    continue
                probe = PortalObservationProbe(
                    tuple(record.get("opening_cell") or gate_cell),
                    tuple(record.get("normal_xy") or (0.0, 0.0)),
                    int(record["id"]),
                    "destination"
                    if str(record.get("state", "")).strip().lower()
                    == "source_arrived"
                    else "source",
                    str(record.get("observation_source", "frontier")),
                )
                destination_view = (
                    str(record.get("state", "")).strip().lower()
                    == "source_arrived"
                )
                physical_xy = (
                    None
                    if request.map_to_physical_xy is None
                    else request.map_to_physical_xy(x, y)
                )
                if not ledger.viewpoint_available(
                    record["id"],
                    physical_xy,
                    phase=("destination" if destination_view else "source"),
                    map_epoch=map_epoch,
                ):
                    continue
                candidates.append(
                    (
                        int(frontier_row),
                        int(frontier_col),
                        FrontierCandidateRoute(
                            row=int(target_row),
                            col=int(target_col),
                            action_tier="probe",
                            place_hops=0,
                            x=float(x),
                            y=float(y),
                            path_distance=(
                                float(costmap_distance)
                                if costmap_distance is not None
                                else float(request.steps[target_row, target_col])
                                * float(request.message.info.resolution)
                            ),
                            score_path_distance=(
                                float(request.steps[target_row, target_col])
                                * float(request.message.info.resolution)
                            ),
                            portal_gate_xy=map_gate,
                            work_item_id=record.get("work_item_id"),
                            work_item_match="rehydrated_portal_probe",
                            portal_observation_probe=probe,
                            work_item_normal_xy=tuple(record["normal_xy"]),
                            graph_action="probe_portal",
                            graph_action_reason="rehydrated_pending_portal_probe",
                            viewpoint_retry=(
                                record.get("work_item_id") is not None
                                and getattr(self, "place_work_items", None) is not None
                                and self.place_work_items.has_failed_viewpoint(
                                    record["work_item_id"],
                                    map_epoch=getattr(
                                        getattr(request, "components", None),
                                        "epoch",
                                        None,
                                    ),
                                )
                            ),
                        ),
                    )
                )
                break
        return candidates

    def structural_boundary_probe_candidates(
        self, request, context, approach_cells,
    ):
        """Compile known-free architectural openings into probe candidates.

        This is the bridge for a doorway whose far side is already visible.
        Such an opening is not a frontier anymore, but it is still an
        unverified graph boundary.  The returned route remains source-side and
        carries a durable probe identity; it cannot authorize a crossing.
        """
        if (
            context is None
            or not getattr(context, "source_place_observed", False)
            or getattr(context, "source_place_id", None) is None
        ):
            return []
        components = request.components
        structural_occupied = getattr(components, "structural_occupied", None)
        labels = getattr(components, "labels", None)
        if structural_occupied is None or labels is None:
            return []
        ledger = getattr(self, "portal_probe_ledger", None)
        register = getattr(self, "register_portal_probe", None)
        if ledger is None or not callable(register):
            return []
        resolution = max(1e-6, float(request.message.info.resolution))
        throat_radius_cells, window_margin_cells = portal_certification_window(
            resolution,
            getattr(self, "region_topology_clearance", 0.50),
            getattr(self, "place_furniture_max_span_m", 2.5),
        )
        known_free = ~(request.unknown | request.occupied)
        boundaries = structural_boundary_candidates(
            known_free,
            structural_occupied,
            request.steps,
            unknown=request.unknown,
            labels=labels,
            source_label=context.source_label,
            support_radius=throat_radius_cells,
            # Use the same clearance-scale wall support as Portal
            # certification.  A two-cell support is only the thickness of a
            # rasterized wall at the usual map resolution; accepting it here
            # makes the two walls of an ordinary corridor look like a
            # doorway.  The source probe is still weaker than certification,
            # but it must describe an architectural wall run rather than a
            # local obstacle edge.
            minimum_wall_span_cells=throat_radius_cells,
            wall_search_radius=window_margin_cells,
        )
        candidates = []
        for boundary in boundaries:
            if getattr(self, "planning_should_preempt", lambda: False)():
                return []
            approach = self.nearest_safe_approach(
                request.steps,
                *boundary.source_cell,
                approach_cells,
                preferred_steps=request.preferred_steps,
                preferred_mask=request.preferred_mask,
                selection_tier=request.selection_tier,
            )
            if approach is None:
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            target_row, target_col = approach
            x, y = self.cell_xy(request.message, target_row, target_col)
            gate_xy = self.cell_xy(
                request.message, *boundary.opening_cell,
            )
            if not self.portal_probe_viewpoint_is_local(
                gate_xy,
                (x, y),
                resolution=request.message.info.resolution,
            ):
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            costmap_distance = self.candidate_costmap_distance(
                request.validation, x, y,
            )
            if request.validation is not None and costmap_distance is None:
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            # A structural boundary is an active information action, not a
            # command to revisit an arbitrary opening retained in the global
            # SLAM map.  Its wall evidence must still be inside the current
            # lidar observation horizon; otherwise stale map fragments (or a
            # map edge) can win the probe selection before the nearby branch
            # is inspected.  The horizon is an existing sensor contract, not
            # a new planner tuning threshold.
            observation_horizon = getattr(
                self, "scan_observation_horizon", None,
            )
            if observation_horizon is not None:
                try:
                    observation_horizon = float(observation_horizon)
                except (TypeError, ValueError):
                    observation_horizon = None
            if (
                observation_horizon is not None
                and observation_horizon > 0.0
                and float(request.steps[target_row, target_col]) * resolution
                > observation_horizon + resolution
            ):
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            probe_template = PortalObservationProbe(
                tuple(boundary.opening_cell),
                tuple(boundary.normal),
                None,
                "source",
                "structural_boundary",
            )
            source_viewpoint = self.source_portal_probe_viewpoint(
                request, probe_template, gate_xy,
            )
            if source_viewpoint is None:
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            target_row, target_col = source_viewpoint
            try:
                source_label = int(context.source_label)
                viewpoint_label = int(labels[target_row, target_col])
            except (AttributeError, IndexError, TypeError, ValueError):
                source_label = None
                viewpoint_label = None
            if (
                source_label is not None
                and source_label > 0
                and viewpoint_label != source_label
            ):
                # A source-side probe is an observation action, not a hidden
                # crossing. If its only safe cell lies in another structural
                # Place, the route would pass through a doorway without the
                # Portal transaction that owns physical Place identity.
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                self.last_structural_boundary_source_place_rejections = int(
                    getattr(
                        self,
                        "last_structural_boundary_source_place_rejections",
                        0,
                    )
                ) + 1
                continue
            probe = register(request, probe_template)
            if probe is None or getattr(probe, "probe_id", None) is None:
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            x, y = self.cell_xy(request.message, target_row, target_col)
            costmap_distance = self.candidate_costmap_distance(
                request.validation, x, y,
            )
            if request.validation is not None and costmap_distance is None:
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            physical_xy = (
                (x, y)
                if request.map_to_physical_xy is None
                else request.map_to_physical_xy(x, y)
            )
            if not ledger.viewpoint_available(probe.probe_id, physical_xy):
                self.last_structural_boundary_rejections = int(
                    getattr(self, "last_structural_boundary_rejections", 0)
                ) + 1
                continue
            path_distance = (
                costmap_distance
                if costmap_distance is not None
                else float(request.steps[target_row, target_col]) * resolution
            )
            candidates.append(
                (
                    int(boundary.opening_cell[0]),
                    int(boundary.opening_cell[1]),
                    FrontierCandidateRoute(
                        row=int(target_row),
                        col=int(target_col),
                        action_tier="probe",
                        place_hops=0,
                        x=float(x),
                        y=float(y),
                        path_distance=float(path_distance),
                        score_path_distance=float(path_distance),
                        portal_gate_xy=gate_xy,
                        work_item_id=None,
                        work_item_match="structural_boundary_probe",
                        portal_observation_probe=probe,
                        work_item_normal_xy=(
                            float(boundary.normal[1]),
                            float(boundary.normal[0]),
                        ),
                        graph_action="probe_portal",
                        graph_action_reason="structural_boundary_evidence",
                    ),
                )
            )
        self.last_structural_boundary_candidates += len(candidates)
        return candidates

    def _portal_transition_execution_cell(
        self, request, transition, continuation_cell=None, next_transition=None,
    ):
        """Derive a one-edge endpoint that TEB cannot complete too early."""
        resolution = max(1e-6, float(request.message.info.resolution))
        crossing_depth = portal_crossing_depth(
            getattr(self, "clearance", 0.0)
        )
        # TEB reports success inside its XY tolerance, whereas the topology
        # layer needs the terminal past a physical gate plane.  Put the goal
        # beyond both constraints and add half a grid cell for quantisation.
        required_depth = (
            crossing_depth
            + float(getattr(self, "teb_xy_goal_tolerance", 0.35))
            + 0.5 * resolution
        )
        return portal_transition_verified_exit_cell(
            request.steps,
            transition,
            minimum_signed_depth_cells=(required_depth / resolution),
            continuation_cell=continuation_cell,
            next_transition=next_transition,
        )

    def _queue_first_portal_action(
        self, request, transitions, frontier_row, frontier_col, portal_sources,
    ):
        """Represent a remote frontier by its first certified doorway."""
        if not request.allow_portal_transitions or not transitions:
            return
        transition = transitions[0]
        source = self._portal_source_for_transition(
            request,
            transition,
            frontier_row,
            frontier_col,
            next_transition=(transitions[1] if len(transitions) > 1 else None),
        )
        if source is None:
            self.last_portal_endpoint_depth_rejections += 1
            return
        self.last_portal_sources += 1
        portal_sources.setdefault(
            (source.identity_row, source.identity_col), source,
        )

    def _portal_source_for_transition(
        self, request, transition, frontier_row, frontier_col, next_transition=None,
    ):
        """Create the named identity, gate, and execution contract for a door."""
        exit_cell = self._portal_transition_execution_cell(
            request,
            transition,
            (frontier_row, frontier_col),
            next_transition=next_transition,
        )
        if exit_cell is None:
            return None
        exit_row, exit_col = exit_cell
        return PortalSource(
            frontier_row=int(frontier_row),
            frontier_col=int(frontier_col),
            identity_row=int(transition.destination_cell[0]),
            identity_col=int(transition.destination_cell[1]),
            gate_row=int(transition.portal_cell[0]),
            gate_col=int(transition.portal_cell[1]),
            endpoint_row=int(exit_row),
            endpoint_col=int(exit_col),
        )

    def portal_observation_probe_for_frontier(self, request, frontier_row, frontier_col):
        """Recognize a source-side doorway observation without crossing it."""
        components = request.components
        structural_occupied = getattr(components, "structural_occupied", None)
        if structural_occupied is None:
            return None
        throat_radius_cells, window_margin_cells = portal_certification_window(
            request.message.info.resolution,
            getattr(self, "region_topology_clearance", 0.50),
            getattr(self, "place_furniture_max_span_m", 2.5),
        )
        # A source-side probe is intentionally weaker than Portal
        # certification. Use the current scan horizon to inspect partially
        # observed wall runs, and require only the fixed two-cell structural
        # support needed to distinguish a wall-bounded opening from a lone
        # obstacle. No user-facing threshold is introduced.
        resolution = max(1e-6, float(request.message.info.resolution))
        observed_horizon = getattr(self, "scan_observation_horizon", None)
        if observed_horizon is not None and observed_horizon > 0.0:
            throat_radius_cells = max(
                throat_radius_cells,
                int(math.ceil(float(observed_horizon) / resolution)),
            )
        probe = portal_observation_probe(
            request.unknown,
            structural_occupied,
            (frontier_row, frontier_col),
            support_radius=throat_radius_cells,
            minimum_wall_span_cells=2,
            wall_search_radius=window_margin_cells,
        )
        register = getattr(self, "register_portal_probe", None)
        if probe is not None and register is not None:
            probe = register(request, probe)
        return probe

    def _portal_wall_support_kwargs(self, message, components):
        """Adapt structural occupancy into portal-certifier wall evidence."""
        structural_occupied = getattr(components, "structural_occupied", None)
        if structural_occupied is None:
            return {}
        resolution = max(1e-6, float(message.info.resolution))
        # Wall support and furniture scale are different evidence domains.
        # ``furniture_max_span_m`` controls which compact occupied components
        # may be removed while constructing the structural place map; it does
        # not mean that each wall jamb must be longer than a desk.  Requiring
        # that much wall run rejected valid short doors in the focused world
        # (and made every cross-place transition look like a local WorkItem).
        # A clearance-radius run on each side is sufficient to distinguish an
        # architectural opening from a one-cell obstacle, while preserving the
        # raw occupancy and the separate structural-map furniture filter.
        clearance = max(
            0.10,
            float(getattr(self, "region_topology_clearance", 0.50)),
        )
        throat_radius_cells = max(1, int(math.ceil(clearance / resolution)))
        return {
            "structural_occupied": structural_occupied,
            "minimum_wall_span_cells": throat_radius_cells,
        }

    @staticmethod
    def _labels_with_unknown_boundary(labels, unknown):
        """Return covered structural labels that still border current unknown space."""
        if labels is None or unknown is None or labels.shape != unknown.shape:
            return set()
        touching = np.zeros_like(unknown, dtype=bool)
        touching[1:, :] |= unknown[:-1, :]
        touching[:-1, :] |= unknown[1:, :]
        touching[:, 1:] |= unknown[:, :-1]
        touching[:, :-1] |= unknown[:, 1:]
        return {
            int(label)
            for label in np.unique(labels[touching & (labels > 0)])
            if int(label) > 0
        }

    def _populate_adjacent_portal_sources(self, request, context, portal_sources):
        """Find adjacent certified portals when no remote frontier exposed one.

        The graph may already contain a valid outgoing doorway although all
        remote frontier endpoints were filtered by coverage or lifecycle
        rules. In that case, expose only the short destination-side endpoint
        for the first doorway; ranking and publishing still happen later.
        """
        # A graph-selected identity must be added even when the transient
        # snapshot already exposed other doorway candidates.  Otherwise the
        # normal selector can observe only the wrong edges and the graph lease
        # has no way to recover the selected one.
        self._populate_preferred_crossed_portal_source(
            request, context, portal_sources,
        )
        if portal_sources:
            return
        # A SLAM correction can erase every current component label while the
        # durable source Place and its doorway remain known. Rehydrate those
        # identities before asking the transient grid for a new portal.
        self._populate_unbound_portal_sources(request, context, portal_sources)
        if portal_sources or not context.place_graph_ready:
            return
        blocked_labels = set(context.blocked_place_labels)
        if context.covered_place_reachable is not None:
            blocked_labels |= {
                int(label)
                for label in np.unique(request.components.labels)
                if int(label) > 0
                and not np.any(
                    context.covered_place_reachable
                    & (request.components.labels == int(label))
                )
            }
        # A completed Place may still be a necessary transit vertex. Keep its
        # label open only when this snapshot proves that unknown space remains
        # on that side; fully observed covered labels stay closed.
        blocked_labels -= self._labels_with_unknown_boundary(
            request.components.labels,
            request.unknown,
        )
        throat_radius_cells, window_margin_cells = portal_certification_window(
            request.message.info.resolution,
            getattr(self, "region_topology_clearance", 0.50),
            getattr(self, "place_furniture_max_span_m", 2.5),
        )
        for transition in certified_adjacent_place_transitions(
                request.components.labels,
                request.steps,
                context.known_free,
                context.source_label,
                throat_radius_cells,
                closed_labels=blocked_labels,
                window_margin_cells=window_margin_cells,
                cache=self._portal_transition_certification_cache,
                **self._portal_wall_support_kwargs(
                    request.message, request.components,
                )
        ):
            source = self._portal_source_for_transition(
                request,
                transition,
                transition.destination_cell[0],
                transition.destination_cell[1],
            )
            if source is None:
                self.last_portal_endpoint_depth_rejections += 1
                continue
            portal_sources.setdefault(
                (source.identity_row, source.identity_col), source,
            )
