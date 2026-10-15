"""Durable reverse-egress actions for places whose SLAM labels merged.

An online map can temporarily erase the structural split that originally
certified a doorway.  The physical Place memory still contains the directed
entry edge, so a completed room must be able to leave through that edge instead
of being reported as globally exhausted.  This module turns that remembered
edge into one Navfn-validated portal action; it never creates a new Place or a
local observation WorkItem.
"""

import math
from collections import Counter

import numpy as np

from global_frontier_portal_crossing import portal_crossing_depth
from global_frontier_portal_crossing import portal_source_side_is_proven
from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_durable_portal_route import (
    crossing_goal,
    project_record,
    signed_distance,
)
from global_frontier_portal_probes import (
    PortalObservationProbe,
    portal_verification_viewpoints,
    source_verification_viewpoints,
)


# These failures are properties of the current SLAM/costmap projection. They
# are deliberately remembered only for one map epoch: a later structural
# update may make the same physical doorway executable again.
_MAP_EPOCH_REJECTION_REASONS = frozenset(
    (
        "portal_projection_unavailable",
        "portal_direction_unavailable",
        "destination_side_not_reachable",
        "costmap_route_unreachable",
        "physical_source_side_unproven",
        "portal_source_place_mismatch",
    )
)


def _positive_id(value):
    """Normalize an optional durable ledger identity."""
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


class GlobalFrontierPortalEgressMixin:
    """Build a reverse portal route from durable Place entry evidence."""

    def _destination_side_route_cell(
        self, message, route_steps, projection, goal_xy, minimum_depth,
    ):
        """Choose a reachable cell that satisfies the directed gate depth."""
        if route_steps is None or not hasattr(route_steps, "shape"):
            return None
        cells = np.argwhere(route_steps >= 0)
        if cells.size == 0:
            return None
        resolution = max(1e-6, float(message.info.resolution))
        origin = message.info.origin.position
        dx = projection.destination_xy[0] - projection.gate_xy[0]
        dy = projection.destination_xy[1] - projection.gate_xy[1]
        direction_length = math.hypot(dx, dy)
        if direction_length <= 1e-6:
            return None
        # This gate is evaluated on every post-terminal graph decision. Keep
        # the exact historical ordering, but evaluate the half-plane and goal
        # distance in one NumPy pass instead of a Python loop over the whole
        # global grid.
        rows = cells[:, 0].astype(np.int64, copy=False)
        cols = cells[:, 1].astype(np.int64, copy=False)
        x = float(origin.x) + (cols.astype(np.float64) + 0.5) * resolution
        y = float(origin.y) + (rows.astype(np.float64) + 0.5) * resolution
        signed = (
            (x - projection.gate_xy[0]) * dx
            + (y - projection.gate_xy[1]) * dy
        ) / direction_length
        accepted = signed + 1e-9 >= float(minimum_depth)
        if not np.any(accepted):
            return None
        rows = rows[accepted]
        cols = cols[accepted]
        distances = np.hypot(x[accepted] - goal_xy[0], y[accepted] - goal_xy[1])
        path_steps = route_steps[rows, cols].astype(np.int64, copy=False)
        # np.lexsort uses the last key as primary, matching min((distance,
        # steps, row, col)) exactly.
        order = np.lexsort((cols, rows, path_steps, distances))
        index = int(order[0])
        return int(rows[index]), int(cols[index])

    def select_durable_portal_crossing(self, snapshot):
        """Compile an unbound certified Portal into a direct crossing route.

        A Portal with source and destination evidence is a graph action even
        when the current SLAM snapshot has no frontier cell for that doorway.
        This path deliberately bypasses frontier candidate construction while
        retaining the same Navfn/costmap validation and route transaction.
        """
        if getattr(self, "target_region_claim_active", False):
            return None
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        portal_id = getattr(self, "graph_route_portal_id", None)
        if ledger is None or portal_id is None:
            return None
        try:
            portal_id = int(portal_id)
        except (TypeError, ValueError):
            return None
        record = getattr(ledger, "get", lambda _value: None)(portal_id)
        if not isinstance(record, dict):
            return None
        state = str(record.get("state", "")).strip().lower()
        if record.get("destination_place_id") is not None or state not in (
            "certified", "selected",
        ):
            return None

        # Branch-first crossing requires completed destination-view evidence.
        # Without that typed proof, the planner must keep this as a probe.
        probe_ledger = getattr(self, "portal_probe_ledger", None)
        if probe_ledger is not None:
            probes = getattr(probe_ledger, "snapshot", lambda: ())()
            destination_evidence = False
            for probe in probes:
                if not isinstance(probe, dict):
                    continue
                try:
                    bound_portal_id = int(probe.get("portal_id"))
                except (TypeError, ValueError):
                    continue
                if (
                    bound_portal_id == portal_id
                    and str(probe.get("state", "")).strip().lower()
                    in ("observed", "certified")
                ):
                    destination_evidence = True
                    break
            if not destination_evidence:
                return None

        message = snapshot.message
        map_epoch = None
        map_context = getattr(snapshot, "map_context", None)
        components = getattr(map_context, "components", None)
        if components is not None:
            map_epoch = getattr(components, "epoch", None)
        if map_epoch is None:
            validation = getattr(getattr(snapshot, "route_graph", None), "validation", None)
            map_epoch = getattr(validation, "epoch", None)
        try:
            map_epoch = None if map_epoch is None else int(map_epoch)
        except (TypeError, ValueError):
            pass
        self._durable_portal_crossing_map_epoch = map_epoch
        rejected_epoch = record.get("rejected_map_epoch")
        try:
            rejected_for_epoch = (
                map_epoch is not None
                and rejected_epoch is not None
                and int(rejected_epoch) == int(map_epoch)
            )
        except (TypeError, ValueError):
            rejected_for_epoch = (
                map_epoch is not None
                and str(rejected_epoch).strip() == str(map_epoch).strip()
            )
        if rejected_for_epoch:
            # The current snapshot already produced negative evidence for this
            # Portal. Do not recompute the same geometry until a new epoch
            # arrives; the adapter will account for the retained obligation.
            self.last_durable_portal_crossing_unavailable = {
                "portal_id": portal_id,
                "map_epoch": map_epoch,
                "reason": str(
                    record.get("last_rejection_reason")
                    or "portal_materialization_unavailable"
                ),
            }
            return None
        # The graph adapter consumes this marker as epoch-scoped negative
        # evidence. It keeps the durable lease until the miss budget decides
        # whether to retry on a newer map or invalidate the obligation.
        self.last_durable_portal_crossing_unavailable = None
        map_frame = getattr(getattr(message, "header", None), "frame_id", "map") or "map"
        projector_factory = getattr(self, "planar_xy_projector", None)
        project = (
            None
            if projector_factory is None
            else projector_factory(map_frame, "odom")
        )
        projection = project_record(record, project)
        if projection is None:
            self._report_durable_portal_crossing_unavailable(
                portal_id, "portal_projection_unavailable",
            )
            return None
        if int(projection.source_place_id) != int(
            getattr(self, "current_physical_place_id", -1)
        ):
            self._report_durable_portal_crossing_unavailable(
                portal_id, "portal_source_place_mismatch",
            )
            return None
        resolution = max(1e-6, float(message.info.resolution))
        required_depth = (
            portal_crossing_depth(getattr(self, "clearance", 0.0))
            + float(getattr(self, "teb_xy_goal_tolerance", 0.35))
            + 0.5 * resolution
        )
        source_signed = signed_distance(snapshot.robot_map, projection)
        # Destination-side probing intentionally leaves the robot beyond the
        # gate before the crossing action is compiled. Preserve the earlier
        # source-side pose as durable evidence for this same physical Portal;
        # otherwise a valid two-view probe is rejected simply because the
        # latest pose is already on the far side.
        source_history_proven = False
        if probe_ledger is not None:
            physical_gate = record.get("physical_gate")
            physical_destination = record.get("physical_destination")
            for probe in probe_ledger.snapshot():
                if not isinstance(probe, dict):
                    continue
                try:
                    bound_portal_id = int(probe.get("portal_id"))
                except (TypeError, ValueError):
                    continue
                if bound_portal_id != portal_id:
                    continue
                for viewpoint in probe.get("viewpoint_history", ()):
                    if portal_source_side_is_proven(
                        viewpoint,
                        physical_gate,
                        physical_destination,
                    ):
                        source_history_proven = True
                        break
                if source_history_proven:
                    break
        self.last_portal_source_side_proven = bool(
            source_signed is not None
            and source_signed < 0.0
        ) or source_history_proven
        self.last_portal_source_side_proven_id = portal_id
        preobserved = bool(
            source_signed is not None and source_signed >= required_depth
        )
        if not preobserved and not self.last_portal_source_side_proven:
            self._report_durable_portal_crossing_unavailable(
                portal_id, "physical_source_side_unproven",
            )
            return None
        goal = crossing_goal(projection, required_depth)
        if goal is None:
            self._report_durable_portal_crossing_unavailable(
                portal_id, "portal_direction_unavailable",
            )
            return None
        route_steps = snapshot.route_graph.route_steps
        cell = self._destination_side_route_cell(
            message,
            route_steps,
            projection,
            goal,
            required_depth,
        )
        if cell is None or route_steps[cell] < 0:
            self._report_durable_portal_crossing_unavailable(
                portal_id, "destination_side_not_reachable",
            )
            return None
        row, col = cell
        x, y = self.cell_xy(message, row, col)
        costmap_distance = self.candidate_costmap_distance(
            snapshot.route_graph.validation, x, y,
        )
        if snapshot.route_graph.validation is not None and costmap_distance is None:
            self._report_durable_portal_crossing_unavailable(
                portal_id, "costmap_route_unreachable",
            )
            return None
        path_distance = (
            float(costmap_distance)
            if costmap_distance is not None
            else float(route_steps[row, col]) * resolution
        )
        selected = ledger.select(portal_id, now=snapshot.now)
        if selected is None:
            return None
        refresh = getattr(ledger, "refresh_projection", None)
        if callable(refresh):
            refresh(
                portal_id,
                map_gate_xy=projection.gate_xy,
                map_destination_xy=projection.destination_xy,
                now=snapshot.now,
                task_version=str(getattr(self, "current_task_version", "") or ""),
            )
        self.last_portal_hypothesis_id = portal_id
        self.publish_status(
            "durable_portal_crossing_selected",
            portal_id=portal_id,
            source_place_id=int(projection.source_place_id),
            gate=[round(float(value), 3) for value in projection.gate_xy],
            goal=[round(float(x), 3), round(float(y), 3)],
            path_distance=round(path_distance, 3),
            reason="destination_view_promoted_to_portal",
        )
        return (
            int(row), int(col), float(x), float(y), path_distance,
            0.0, 0.0, -path_distance, None, 1, "portal_transition",
            projection.gate_xy, None, None, 0, None, False,
            "cross_portal", "durable_portal_crossing", portal_id,
            preobserved,
        )

    def _report_durable_portal_crossing_unavailable(
        self, portal_id, reason, map_epoch=None,
    ):
        """Publish one deduplicated projection audit for a graph-owned edge."""
        if map_epoch is None:
            map_epoch = getattr(
                self, "_durable_portal_crossing_map_epoch", None
            )
        try:
            map_epoch = None if map_epoch is None else int(map_epoch)
        except (TypeError, ValueError):
            pass
        report = {"portal_id": int(portal_id), "reason": str(reason)}
        if map_epoch is not None:
            report["map_epoch"] = map_epoch
        identity_builder = getattr(
            self, "_graph_route_materialization_identity", None
        )
        if callable(identity_builder):
            identity = identity_builder(
                plan=getattr(self, "last_graph_route_plan", None),
                map_epoch=map_epoch,
            )
            identity.update(report)
            report = identity
        # Keep the full reason even when the status publication is
        # de-duplicated. The graph adapter needs an explicit lifecycle marker,
        # not another timer tick, to release the stale route lease.
        self.last_durable_portal_crossing_unavailable = dict(report)
        if (
            map_epoch is not None
            and str(reason) in _MAP_EPOCH_REJECTION_REASONS
        ):
            ledger = getattr(self, "portal_hypothesis_ledger", None)
            mark_rejected = getattr(ledger, "mark_unbound_rejected", None)
            if callable(mark_rejected):
                mark_rejected(int(portal_id), map_epoch, str(reason))
            # The graph adapter owns the durable miss budget.  Keep its lease
            # alive across this epoch-local rejection so the same Portal edge
            # can be retried on a newer map and eventually invalidated by the
            # graph boundary instead of being replaced after one miss.
        if report == getattr(self, "last_durable_portal_crossing_report", None):
            return
        self.last_durable_portal_crossing_report = report
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish("durable_portal_crossing_unavailable", **report)

    def _report_durable_portal_egress_unavailable(
        self, snapshot, portal_id, reason, **fields
    ):
        """Publish one snapshot-scoped failure for a bound reverse edge.

        Forward crossing already exposes this marker to the graph adapter. A
        reverse egress used to return ``None`` without an equivalent marker,
        so a bound Portal could remain leased forever after its destination
        side became unexecutable. The graph adapter consumes this marker and
        owns the bounded miss/invalidation/release transition.
        """
        try:
            portal_id = int(portal_id)
        except (TypeError, ValueError):
            return
        if portal_id <= 0:
            return
        components = getattr(
            getattr(snapshot, "map_context", None), "components", None
        )
        map_epoch = getattr(components, "epoch", None)
        try:
            map_epoch = None if map_epoch is None else int(map_epoch)
        except (TypeError, ValueError):
            map_epoch = None
        report = {
            "portal_id": portal_id,
            "reason": str(reason or "portal_egress_unavailable"),
        }
        if map_epoch is not None:
            report["map_epoch"] = map_epoch
        source_place_id = getattr(self, "current_physical_place_id", None)
        if source_place_id is not None:
            try:
                report["source_place_id"] = int(source_place_id)
            except (TypeError, ValueError):
                pass
        report.update(fields)
        identity_builder = getattr(
            self, "_graph_route_materialization_identity", None
        )
        if callable(identity_builder):
            identity = identity_builder(
                plan=getattr(self, "last_graph_route_plan", None),
                map_epoch=map_epoch,
                transaction=getattr(
                    self, "graph_route_action_transaction", None
                ),
            )
            report = {**identity, **report}
        self.last_durable_portal_egress_unavailable = dict(report)
        if report == getattr(
            self, "last_durable_portal_egress_report", None
        ):
            return
        self.last_durable_portal_egress_report = dict(report)
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish("durable_portal_egress_unavailable", **report)

    def _covered_portal_transit_is_allowed(self, source_region, destination_region):
        """Permit covered transit only when the durable graph has progress."""
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        if not PortalHypothesisLedger.place_is_covered(
            source_region
        ) or not PortalHypothesisLedger.place_is_covered(destination_region):
            return True
        if ledger is None:
            # A covered-to-covered route without the identity ledger has no
            # auditable destination graph and must fail closed.  Uncovered
            # endpoints are ordinary exploration and remain admissible.
            return False
        return ledger.allows_covered_transit(
            source_region,
            destination_region,
            place_memory=getattr(self, "region_memory", None),
            work_item_ledger=getattr(self, "place_work_items", None),
            portal_probe_ledger=getattr(self, "portal_probe_ledger", None),
        )

    @staticmethod
    def _entry_xy(entry, key):
        value = entry.get(key) if isinstance(entry, dict) else None
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            return None
        try:
            result = float(value[0]), float(value[1])
        except (TypeError, ValueError):
            return None
        return result if all(math.isfinite(item) for item in result) else None

    def _durable_egress_endpoint(self, message, route_steps, gate_xy, inside_xy):
        """Sample a free destination-side cell beyond a remembered gate."""
        vector_x = float(gate_xy[0]) - float(inside_xy[0])
        vector_y = float(gate_xy[1]) - float(inside_xy[1])
        length = math.hypot(vector_x, vector_y)
        if length <= 1e-6:
            return None
        resolution = max(1e-6, float(message.info.resolution))
        depth = (
            portal_crossing_depth(getattr(self, "clearance", 0.0))
            + float(getattr(self, "teb_xy_goal_tolerance", 0.35))
            + 0.5 * resolution
        )
        target = (
            gate_xy[0] + vector_x / length * depth,
            gate_xy[1] + vector_y / length * depth,
        )
        cell = self.nearest_reachable_cell(
            message, route_steps, target[0], target[1],
        )
        if cell is None:
            return None
        row, col = cell
        if route_steps[row, col] < 0:
            return None
        return row, col, self.cell_xy(message, row, col)

    def _durable_egress_physical_projection(self, message, robot_map, gate_xy, goal_xy):
        """Capture one physical transform for directional admission/identity."""
        map_frame = getattr(getattr(message, "header", None), "frame_id", "map") or "map"
        transform = getattr(self, "transform_xy", None)
        if transform is None:
            return robot_map, gate_xy, goal_xy
        source = transform("odom", map_frame, robot_map[0], robot_map[1])
        gate = transform("odom", map_frame, gate_xy[0], gate_xy[1])
        goal = transform("odom", map_frame, goal_xy[0], goal_xy[1])
        if source is None or gate is None or goal is None:
            return robot_map, gate_xy, goal_xy
        return source, gate, goal

    def _remember_durable_egress_hypothesis(
        self, message, gate_xy, goal_xy, physical_gate, physical_goal, now,
        destination_place_id=None,
    ):
        """Reuse the graph edge for reverse action when one is selected.

        ``PortalHypothesisLedger`` records the direction in which a doorway
        was first certified.  Reverse execution is still the same physical
        edge, not a new hypothesis whose source happens to be the current
        Place.  Creating that second ID breaks graph-route ownership because
        the planner names the original crossed edge.
        """
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        place_id = getattr(self, "current_physical_place_id", None)
        if ledger is None or place_id is None:
            return None
        preferred = getattr(self, "graph_route_portal_id", None)
        try:
            preferred = None if preferred is None else int(preferred)
        except (TypeError, ValueError):
            preferred = None
        if preferred is not None and preferred > 0:
            existing = ledger.get(preferred)
            if isinstance(existing, dict) and str(
                existing.get("state", "")
            ).strip().lower() == "crossed":
                try:
                    same_destination = int(
                        existing.get("source_place_id")
                    ) == int(destination_place_id)
                except (TypeError, ValueError):
                    same_destination = False
                existing_gate = existing.get("physical_gate")
                try:
                    gate_matches = (
                        existing_gate is not None
                        and physical_gate is not None
                        and math.hypot(
                            float(existing_gate[0]) - float(physical_gate[0]),
                            float(existing_gate[1]) - float(physical_gate[1]),
                        ) <= float(getattr(ledger, "match_radius", 0.30)) * 2.0
                    )
                except (IndexError, TypeError, ValueError):
                    gate_matches = False
                if same_destination and gate_matches:
                    self.last_portal_hypothesis_id = preferred
                    return existing
        # A wide doorway may place the reverse egress goal at the opposite
        # jamb.  Reuse the incoming crossed edge by its durable gate plane
        # before certifying anything from the current Place; otherwise the
        # reverse route would mint a new source=current Place hypothesis and
        # can later be committed as a self-loop.
        crossed_gate_query = getattr(
            ledger, "crossed_gate_for_destination", None,
        )
        if callable(crossed_gate_query) and physical_gate is not None:
            memory = getattr(self, "region_memory", None)
            gate_radius = getattr(
                memory, "portal_entry_match_radius", None,
            )
            incoming = crossed_gate_query(
                place_id,
                physical_gate,
                gate_radius=gate_radius,
            )
            if incoming is not None:
                try:
                    incoming_source = int(incoming.get("source_place_id"))
                    destination_id = int(destination_place_id)
                except (TypeError, ValueError):
                    incoming_source = None
                    destination_id = None
                if (
                    incoming_source is not None
                    and destination_id == incoming_source
                ):
                    self.last_portal_hypothesis_id = int(incoming["id"])
                    return incoming
        hypothesis, created = ledger.certify(
            place_id,
            physical_gate,
            physical_goal,
            now=now,
            task_version=str(getattr(self, "current_task_version", "") or ""),
            map_gate_xy=gate_xy,
            map_destination_xy=goal_xy,
        )
        if hypothesis is None:
            return None
        if str(hypothesis.get("state", "")).strip().lower() == "failed":
            self.publish_status(
                "durable_portal_egress_skipped",
                source_place_id=int(place_id),
                portal_id=int(hypothesis["id"]),
                reason="portal_negative_evidence",
            )
            return None
        if created:
            self.publish_status(
                "portal_hypothesis_certified",
                portal_id=int(hypothesis["id"]),
                source_place_id=int(place_id),
                gate=[round(float(gate_xy[0]), 3), round(float(gate_xy[1]), 3)],
                destination=[round(float(goal_xy[0]), 3), round(float(goal_xy[1]), 3)],
                direction="reverse_egress",
            )
        selected = ledger.select(hypothesis["id"], now=now)
        if selected is not None:
            self.last_portal_hypothesis_id = int(selected["id"])
        return selected

    def _entry_matches_graph_portal(self, entry):
        """Match a reverse entry to the planner's selected physical edge."""
        preferred = getattr(self, "graph_route_portal_id", None)
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        if preferred is None or ledger is None:
            return True
        try:
            record = ledger.get(int(preferred))
        except (TypeError, ValueError):
            return False
        if not isinstance(record, dict) or not isinstance(entry, dict):
            return False
        try:
            if int(entry.get("source_place_id", -1)) != int(
                record.get("source_place_id", -2)
            ):
                return False
        except (TypeError, ValueError):
            return False
        entry_gates = (
            self._entry_xy(entry, "physical_gate"),
            self._entry_xy(entry, "gate"),
        )
        record_gates = (
            self._entry_xy(record, "physical_gate"),
            self._entry_xy(record, "map_gate"),
        )
        radius = float(getattr(ledger, "match_radius", 0.30)) * 2.0
        return any(
            left is not None
            and right is not None
            and math.hypot(left[0] - right[0], left[1] - right[1]) <= radius
            for left in entry_gates
            for right in record_gates
        )

    def select_durable_portal_egress(self, snapshot):
        """Return one remembered reverse egress candidate, if executable."""
        self.last_durable_portal_egress_unavailable = None
        if getattr(self, "target_region_claim_active", False):
            return None
        place_id = getattr(self, "current_physical_place_id", None)
        if place_id is None:
            return None
        region = self.region_memory.by_id(place_id)
        if region is None:
            return None
        excluded_portal_ids = set()
        exclusion_reader = getattr(
            self, "_graph_route_materialization_exclusions", None
        )
        if callable(exclusion_reader):
            excluded_portal_ids = {
                value for value in (
                    _positive_id(item)
                    for item in exclusion_reader(
                        getattr(
                            getattr(getattr(snapshot, "map_context", None),
                                    "components", None),
                            "epoch", None,
                        )
                    )[0]
                ) if value is not None
            }
        entries = [
            entry for entry in region.get("entry_portals", [])
            if isinstance(entry, dict)
            and self._entry_xy(entry, "gate") is not None
            and self._entry_xy(entry, "inside") is not None
        ]
        preferred_portal_id = _positive_id(
            getattr(self, "graph_route_portal_id", None)
        )
        if not entries:
            if preferred_portal_id is not None:
                self._report_durable_portal_egress_unavailable(
                    snapshot,
                    preferred_portal_id,
                    "reverse_entry_missing",
                )
            return None

        # Prefer the physically closest remembered entrance. This is a graph
        # query, not a new score weight, and keeps an office with two doors on
        # the shortest legal egress edge.
        robot = snapshot.robot_map
        entries.sort(
            key=lambda entry: math.hypot(
                self._entry_xy(entry, "inside")[0] - robot[0],
                self._entry_xy(entry, "inside")[1] - robot[1],
            )
        )
        last_unavailable_reason = None
        last_unavailable_fields = {}
        for entry in entries:
            entry_portal_id = next(
                (
                    _positive_id(entry.get(key))
                    for key in ("portal_id", "id", "hypothesis_id")
                    if _positive_id(entry.get(key)) is not None
                ),
                None,
            )
            if entry_portal_id in excluded_portal_ids:
                continue
            if not self._entry_matches_graph_portal(entry):
                last_unavailable_reason = "portal_entry_identity_mismatch"
                last_unavailable_fields = {}
                continue
            gate_xy = self._entry_xy(entry, "gate")
            inside_xy = self._entry_xy(entry, "inside")
            endpoint = self._durable_egress_endpoint(
                snapshot.message,
                snapshot.route_graph.route_steps,
                gate_xy,
                inside_xy,
            )
            if endpoint is None:
                last_unavailable_reason = "destination_side_not_reachable"
                last_unavailable_fields = {}
                continue
            row, col, goal_xy = endpoint
            physical_source, physical_gate, physical_goal = (
                self._durable_egress_physical_projection(
                    snapshot.message,
                    robot,
                    gate_xy,
                    goal_xy,
                )
            )
            reverse_exit = self.region_memory.portal_exit_from_entry(
                gate_xy,
                robot,
                goal_xy,
                physical_gate_xy=physical_gate,
                physical_source_xy=physical_source,
                physical_destination_xy=physical_goal,
            )
            if reverse_exit is None:
                last_unavailable_reason = "reverse_entry_not_found"
                last_unavailable_fields = {}
                continue
            destination_region = reverse_exit[0]
            if not self._covered_portal_transit_is_allowed(
                region, destination_region,
            ):
                self.last_portal_covered_cycle_skips = int(
                    getattr(self, "last_portal_covered_cycle_skips", 0)
                ) + 1
                self.publish_status(
                    "durable_portal_egress_skipped",
                    source_place_id=int(place_id),
                    destination_place_id=(
                        None
                        if not isinstance(destination_region, dict)
                        else destination_region.get("id")
                    ),
                    reason="covered_to_covered_without_graph_progress",
                )
                last_unavailable_reason = "covered_to_covered_without_graph_progress"
                last_unavailable_fields = {
                    "destination_place_id": (
                        None
                        if not isinstance(destination_region, dict)
                        else destination_region.get("id")
                    ),
                }
                continue
            costmap_distance = self.candidate_costmap_distance(
                snapshot.route_graph.validation,
                goal_xy[0],
                goal_xy[1],
            )
            if (
                snapshot.route_graph.validation is not None
                and costmap_distance is None
            ):
                last_unavailable_reason = "costmap_route_unreachable"
                last_unavailable_fields = {
                    "destination_place_id": int(destination_region["id"]),
                }
                continue
            path_distance = (
                float(costmap_distance)
                if costmap_distance is not None
                else float(snapshot.route_graph.route_steps[row, col])
                * float(snapshot.message.info.resolution)
            )
            hypothesis = self._remember_durable_egress_hypothesis(
                snapshot.message,
                gate_xy,
                goal_xy,
                physical_gate,
                physical_goal,
                snapshot.now,
                destination_place_id=reverse_exit[0]["id"],
            )
            if hypothesis is None:
                last_unavailable_reason = "portal_identity_unavailable"
                last_unavailable_fields = {
                    "destination_place_id": int(destination_region["id"]),
                }
                continue
            self.last_durable_portal_egress_unavailable = None
            self.publish_status(
                "durable_portal_egress_selected",
                place_id=int(place_id),
                portal_id=int(hypothesis["id"]),
                gate=[round(float(gate_xy[0]), 3), round(float(gate_xy[1]), 3)],
                goal=[round(float(goal_xy[0]), 3), round(float(goal_xy[1]), 3)],
                path_distance=round(float(path_distance), 3),
                destination_place_id=int(reverse_exit[0]["id"]),
            )
            return (
                int(row),
                int(col),
                float(goal_xy[0]),
                float(goal_xy[1]),
                float(path_distance),
                0.0,
                0.0,
                -float(path_distance),
                None,
                1,
                "portal_transition",
                gate_xy,
                None,
                None,
                0,
                None,
                False,
                "cross_portal",
                "durable_portal_egress",
                int(hypothesis["id"]),
            )
        if preferred_portal_id is not None and last_unavailable_reason:
            self._report_durable_portal_egress_unavailable(
                snapshot,
                preferred_portal_id,
                last_unavailable_reason,
                **last_unavailable_fields
            )
        return None

    def select_durable_portal_probe(self, snapshot):
        """Rehydrate a pending doorway probe without requiring a live frontier.

        SLAM may turn the unknown arc that created a probe into known space
        before the probe is executed. The physical gate ledger is stronger
        evidence than that transient frontier cell, so route to the nearest
        current safe cell on the correct side of the gate and reobserve the
        same probe identity. This is an information action, not a new room or
        a distance-based retry.
        """
        ledger = getattr(self, "portal_probe_ledger", None)
        place_id = getattr(self, "current_physical_place_id", None)
        if ledger is None or place_id is None:
            return None
        preferred_probe_id = getattr(self, "graph_route_probe_id", None)
        try:
            preferred_probe_id = (
                None if preferred_probe_id is None else int(preferred_probe_id)
            )
        except (TypeError, ValueError):
            preferred_probe_id = None
        pending_for_source = getattr(ledger, "pending_for_source", None)
        if pending_for_source is None:
            return None
        map_epoch = getattr(
            getattr(getattr(snapshot, "map_context", None), "components", None),
            "epoch",
            None,
        )
        try:
            records = pending_for_source(
                place_id,
                include_source_arrived=True,
                map_epoch=map_epoch,
            )
        except TypeError:
            try:
                records = pending_for_source(
                    place_id, include_source_arrived=True,
                )
            except TypeError:
                records = pending_for_source(place_id)
        excluded_probe_ids = set()
        exclusion_reader = getattr(
            self, "_graph_route_materialization_exclusions", None
        )
        if callable(exclusion_reader):
            excluded_probe_ids = {
                value for value in (
                    _positive_id(item)
                    for item in exclusion_reader(map_epoch)[2]
                ) if value is not None
            }
            if preferred_probe_id in excluded_probe_ids:
                preferred_probe_id = None
            records = [
                record for record in records
                if _positive_id(record.get("id")) not in excluded_probe_ids
            ]
        # A graph-owned source-arrived probe is an immediate two-phase
        # transaction: the source observation has just completed, so its
        # destination view must remain materializable even when a
        # map-epoch-aware ledger query temporarily filters the record.  Do
        # not apply this recovery to a probe whose last phase was already a
        # destination attempt; that case is intentionally gated on newer map
        # evidence (or an explicit controller-failure release).
        preferred_record = (
            None
            if preferred_probe_id is None
            else ledger.get(preferred_probe_id)
        )
        preferred_state = (
            None
            if not isinstance(preferred_record, dict)
            else str(preferred_record.get("state", "")).strip().lower()
        )
        preferred_last_phase = (
            None
            if not isinstance(preferred_record, dict)
            else str(preferred_record.get("last_phase", "")).strip().lower()
        )
        source_phase_ready = (
            preferred_state == "source_arrived"
            and preferred_last_phase not in ("destination",)
        )
        if source_phase_ready and not any(
            int(record.get("id", -1)) == preferred_probe_id
            for record in records
            if isinstance(record, dict)
        ):
            records = [preferred_record] + list(records)
        if not records:
            preferred = (
                None if preferred_probe_id is None else ledger.get(preferred_probe_id)
            )
            report = {
                "source_place_id": int(place_id),
                "map_epoch": map_epoch,
                "candidate_count": 0,
                "preferred_probe_id": preferred_probe_id,
                "preferred_probe_state": (
                    None if not isinstance(preferred, dict)
                    else str(preferred.get("state", ""))
                ),
                "preferred_last_phase": (
                    None if not isinstance(preferred, dict)
                    else preferred.get("last_phase")
                ),
                "preferred_destination_attempt_epoch": (
                    None if not isinstance(preferred, dict)
                    else preferred.get("destination_attempt_epoch")
                ),
                "preferred_last_map_epoch": (
                    None if not isinstance(preferred, dict)
                    else preferred.get("last_map_epoch")
                ),
                "rejection_reasons": {
                    "no_rehydratable_probe_for_source": 1,
                },
                "viewpoint_audit": [],
            }
            self._publish_durable_portal_probe_report(report)
            return None
        rejection_reasons = Counter()
        message = snapshot.message
        route_steps = snapshot.route_graph.route_steps
        map_frame = getattr(getattr(message, "header", None), "frame_id", "map") or "map"
        transform = getattr(self, "transform_xy", None)
        projector_factory = getattr(self, "planar_xy_projector", None)
        map_to_physical = (
            None
            if projector_factory is None
            else projector_factory("odom", map_frame)
        )
        if map_to_physical is None and transform is not None:
            map_to_physical = lambda x, y: transform(
                "odom", map_frame, x, y
            )
        physical_to_map = (
            None
            if projector_factory is None
            else projector_factory(map_frame, "odom")
        )
        resolution = max(1e-6, float(message.info.resolution))
        depth = (
            portal_crossing_depth(getattr(self, "clearance", 0.0))
            + float(getattr(self, "teb_xy_goal_tolerance", 0.35))
            + 0.5 * resolution
        )

        def grid_cell_for_map_xy(point):
            """Convert map coordinates to the current route-grid cell."""
            if point is None or not hasattr(route_steps, "shape"):
                return None
            try:
                col = int(math.floor(
                    (float(point[0])
                     - float(message.info.origin.position.x)) / resolution
                ))
                row = int(math.floor(
                    (float(point[1])
                     - float(message.info.origin.position.y)) / resolution
                ))
            except (AttributeError, IndexError, TypeError, ValueError):
                return None
            if not (
                0 <= row < int(route_steps.shape[0])
                and 0 <= col < int(route_steps.shape[1])
            ):
                return None
            return row, col

        viewpoint_audit = []
        source_viewpoint_cell = None
        projected_cells = []
        destination_ladder_candidate_count = 0
        selected_candidate = None
        for record in records:
            if (
                preferred_probe_id is not None
                and int(record.get("id", -1)) != preferred_probe_id
            ):
                continue
            gate = self._entry_xy(record, "map_gate_xy")
            physical_gate = self._entry_xy(record, "physical_gate_xy")
            if physical_gate is not None and (
                physical_to_map is not None or transform is not None
            ):
                projected = (
                    physical_to_map(physical_gate[0], physical_gate[1])
                    if physical_to_map is not None
                    else transform(
                        map_frame, "odom", physical_gate[0], physical_gate[1]
                    )
                )
                if projected is not None:
                    gate = projected
            normal = self._entry_xy(record, "normal_xy")
            if gate is None or normal is None:
                rejection_reasons["gate_or_normal_missing"] += 1
                continue
            normal_length = math.hypot(normal[0], normal[1])
            if normal_length <= 1e-6:
                rejection_reasons["normal_degenerate"] += 1
                continue
            normal = normal[0] / normal_length, normal[1] / normal_length
            destination_view = record.get("state") == "source_arrived"
            if destination_view:
                # ``last_viewpoint_xy`` is durable physical/odom evidence,
                # while the ladder below is projected into this map.  The
                # comparison must therefore happen in the current grid, not
                # in raw coordinates that SLAM may have changed.
                source_viewpoint = self._entry_xy(
                    record, "last_viewpoint_xy",
                )
                source_map_viewpoint = None
                if source_viewpoint is not None:
                    if physical_to_map is not None:
                        source_map_viewpoint = physical_to_map(
                            source_viewpoint[0], source_viewpoint[1],
                        )
                    elif transform is not None:
                        source_map_viewpoint = transform(
                            map_frame, "odom",
                            source_viewpoint[0], source_viewpoint[1],
                        )
                    else:
                        # Existing no-TF fixtures and deployments already
                        # treat physical/map coordinates as coincident when
                        # no projector exists. Keep that fallback explicit;
                        # a missing source viewpoint remains uncomparable.
                        source_map_viewpoint = source_viewpoint
                source_viewpoint_cell = grid_cell_for_map_xy(
                    source_map_viewpoint,
                )
                viewpoint_ladder = portal_verification_viewpoints(
                    gate, normal, depth,
                )
            else:
                viewpoint_ladder = source_verification_viewpoints(
                    gate, normal, depth,
                )
            for viewpoint_strategy, desired in viewpoint_ladder:
                audit = {
                    "strategy": str(viewpoint_strategy),
                    "desired": [
                        round(float(desired[0]), 3),
                        round(float(desired[1]), 3),
                    ],
                    "cell": None,
                    "map_viewpoint": None,
                    "physical_viewpoint": None,
                }
                if destination_view:
                    destination_ladder_candidate_count += 1
                cell = self.nearest_reachable_cell(
                    message, route_steps, desired[0], desired[1],
                )
                if cell is None or route_steps[cell] < 0:
                    if destination_view:
                        projected_cells.append(None)
                    rejection_reasons["no_reachable_gate_side_cell"] += 1
                    audit["reason"] = "no_reachable_gate_side_cell"
                    viewpoint_audit.append(audit)
                    continue
                row, col = cell
                x, y = self.cell_xy(message, row, col)
                audit["cell"] = [int(row), int(col)]
                audit["map_viewpoint"] = [round(float(x), 3), round(float(y), 3)]
                if destination_view:
                    projected_cells.append([int(row), int(col)])
                physical_viewpoint = (
                    (x, y)
                    if map_to_physical is None
                    else map_to_physical(x, y)
                )
                if physical_viewpoint is not None:
                    audit["physical_viewpoint"] = [
                        round(float(physical_viewpoint[0]), 3),
                        round(float(physical_viewpoint[1]), 3),
                    ]
                if (
                    destination_view
                    and source_viewpoint_cell is not None
                    and (int(row), int(col)) == source_viewpoint_cell
                ):
                    rejection_reasons[
                        "projection_collapsed_to_source_viewpoint"
                    ] += 1
                    audit["reason"] = (
                        "projection_collapsed_to_source_viewpoint"
                    )
                    viewpoint_audit.append(audit)
                    continue
                if not self.portal_probe_viewpoint_is_local(
                    gate, (x, y), resolution=resolution,
                ):
                    rejection_reasons["viewpoint_not_local_to_gate"] += 1
                    audit["reason"] = "viewpoint_not_local_to_gate"
                    viewpoint_audit.append(audit)
                    continue
                if snapshot.route_graph.validation is not None:
                    costmap_distance = self.candidate_costmap_distance(
                        snapshot.route_graph.validation, x, y,
                    )
                    if costmap_distance is None:
                        rejection_reasons["costmap_route_unreachable"] += 1
                        audit["reason"] = "costmap_route_unreachable"
                        viewpoint_audit.append(audit)
                        continue
                    path_distance = float(costmap_distance)
                else:
                    path_distance = float(route_steps[row, col]) * resolution
                audit["path_distance"] = round(float(path_distance), 3)
                try:
                    viewpoint_available = ledger.viewpoint_available(
                        int(record["id"]),
                        physical_viewpoint,
                        phase=(
                            "destination" if destination_view else "source"
                        ),
                        map_epoch=map_epoch,
                    )
                except TypeError:
                    # Keep injected/older ledgers compatible while production
                    # uses explicit phase and map-epoch provenance.
                    viewpoint_available = ledger.viewpoint_available(
                        int(record["id"]), physical_viewpoint,
                    )
                if not viewpoint_available:
                    rejection_reasons["viewpoint_already_attempted"] += 1
                    prior = list(record.get("viewpoint_history", ()))
                    prior.extend(record.get("failed_viewpoints", ()))
                    distances = []
                    for previous in prior:
                        try:
                            distances.append(round(
                                math.hypot(
                                    float(physical_viewpoint[0]) - float(previous[0]),
                                    float(physical_viewpoint[1]) - float(previous[1]),
                                ),
                                3,
                            ))
                        except (IndexError, TypeError, ValueError):
                            continue
                    audit["history_distances"] = distances
                    audit["gate_match_radius"] = round(
                        float(getattr(ledger, "match_radius", 0.0)), 3,
                    )
                    audit["viewpoint_match_radius"] = round(
                        float(getattr(
                            ledger, "viewpoint_match_radius",
                            getattr(ledger, "match_radius", 0.0),
                        )),
                        3,
                    )
                    audit["reason"] = "viewpoint_already_attempted"
                    viewpoint_audit.append(audit)
                    continue
                audit["reason"] = "executable"
                viewpoint_audit.append(audit)
                probe = PortalObservationProbe(
                    tuple(record.get("opening_cell") or (row, col)),
                    tuple(normal),
                    int(record["id"]),
                    "destination" if destination_view else "source",
                    str(record.get("observation_source", "frontier")),
                )
                if selected_candidate is None:
                    selected_candidate = {
                        "probe_id": int(record["id"]),
                        "phase": "destination" if destination_view else "source",
                        "strategy": str(viewpoint_strategy),
                        "gate": gate,
                        "row": int(row),
                        "col": int(col),
                        "x": float(x),
                        "y": float(y),
                        "path_distance": path_distance,
                        "observation_count": float(
                            record.get("observation_count", 0.0)
                        ),
                        "work_item_id": record.get("work_item_id"),
                        "probe": probe,
                    }

        projection_collapsed = bool(
            destination_ladder_candidate_count > 0
            and source_viewpoint_cell is not None
            and len(projected_cells) == destination_ladder_candidate_count
            and all(
                cell is not None
                and tuple(cell) == tuple(source_viewpoint_cell)
                for cell in projected_cells
            )
        )
        if selected_candidate is not None:
            selected = selected_candidate
            self.publish_status(
                "durable_portal_probe_selected",
                probe_id=selected["probe_id"],
                source_place_id=int(place_id),
                phase=selected["phase"],
                viewpoint_strategy=selected["strategy"],
                gate=[round(float(value), 3) for value in selected["gate"]],
                goal=[round(selected["x"], 3), round(selected["y"], 3)],
                path_distance=round(selected["path_distance"], 3),
                viewpoint_audit=viewpoint_audit,
                source_viewpoint_cell=(
                    None
                    if source_viewpoint_cell is None
                    else [
                        int(source_viewpoint_cell[0]),
                        int(source_viewpoint_cell[1]),
                    ]
                ),
                projected_cells=projected_cells,
                projection_collapsed=projection_collapsed,
                projection_collapse_reason=(
                    "ladder_collapsed_to_source_viewpoint"
                    if projection_collapsed else None
                ),
                destination_ladder_candidate_count=(
                    destination_ladder_candidate_count
                ),
            )
            self.last_durable_portal_probe_report = None
            return (
                selected["row"], selected["col"],
                selected["x"], selected["y"], selected["path_distance"],
                selected["observation_count"], 0.0,
                -selected["path_distance"], None, 0, "frontier_endpoint",
                selected["gate"], selected["work_item_id"],
                "durable_probe", 0, selected["probe"], False,
                "probe_portal", "durable_portal_probe_rehydrated",
            )
        report = {
            "source_place_id": int(place_id),
            "map_epoch": map_epoch,
            "candidate_count": len(records),
            "preferred_probe_id": preferred_probe_id,
            "rejection_reasons": dict(sorted(rejection_reasons.items())),
            "viewpoint_audit": viewpoint_audit,
            "source_viewpoint_cell": (
                None
                if source_viewpoint_cell is None
                else [
                    int(source_viewpoint_cell[0]),
                    int(source_viewpoint_cell[1]),
                ]
            ),
            "projected_cells": projected_cells,
            "projection_collapsed": projection_collapsed,
            "projection_collapse_reason": (
                "ladder_collapsed_to_source_viewpoint"
                if projection_collapsed else None
            ),
            "destination_ladder_candidate_count": (
                destination_ladder_candidate_count
            ),
        }
        if preferred_probe_id is not None:
            # The graph plan owns one probe identity. Once the current map
            # cannot materialize that identity, park it in the durable ledger
            # so the next planning pass can consider a different evidence
            # action instead of replaying the same failed projection.
            mark_unavailable = getattr(
                ledger, "mark_projection_unavailable", None
            )
            if callable(mark_unavailable) and any(
                int(record.get("id", -1)) == preferred_probe_id
                for record in records
            ):
                reasons = ",".join(
                    "%s=%s" % (key, value)
                    for key, value in sorted(rejection_reasons.items())
                ) or "no_executable_viewpoint"
                mark_unavailable(
                    preferred_probe_id,
                    map_epoch=map_epoch,
                    reason=reasons,
                )
        self._publish_durable_portal_probe_report(report)
        return None

    def _publish_durable_portal_probe_report(self, report):
        """Publish one deduplicated audit for a non-materialized probe."""
        identity_builder = getattr(
            self, "_graph_route_materialization_identity", None
        )
        if callable(identity_builder):
            identity = identity_builder(
                plan=getattr(self, "last_graph_route_plan", None),
                map_epoch=report.get("map_epoch"),
            )
            identity.update(report)
            report = identity
        if report == getattr(self, "last_durable_portal_probe_report", None):
            return
        self.last_durable_portal_probe_report = report
        publish = getattr(self, "publish_status", None)
        if publish is not None:
            publish("durable_portal_probe_unavailable", **report)
