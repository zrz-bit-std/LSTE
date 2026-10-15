"""Persistent identity for certified but not-yet-crossed portals.

Portal geometry is recomputed from every SLAM snapshot.  The physical doorway
itself should not be.  This small ledger keeps a stable hypothesis keyed by
the source Place and physical gate/destination observations, while leaving
certification and route execution to the existing topology and Navfn/TEB
layers.
"""

import math


def _xy(value):
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def _distance(first, second):
    first, second = _xy(first), _xy(second)
    if first is None or second is None:
        return float("inf")
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _unit(value):
    point = _xy(value)
    if point is None:
        return None
    length = math.hypot(point[0], point[1])
    if length <= 1e-9:
        return None
    return point[0] / length, point[1] / length


class PortalHypothesisLedger:
    """Keep one physical identity for each source-place doorway."""

    def __init__(self, match_radius=0.30, limit=512):
        self.match_radius = max(0.05, float(match_radius))
        self.limit = max(16, int(limit))
        self._next_id = 1
        self._records = {}

    def _matching_record(self, source_place_id, gate_xy, destination_xy):
        gate_xy = _xy(gate_xy)
        destination_xy = _xy(destination_xy)
        if gate_xy is None or destination_xy is None:
            return None
        direction = _unit((
            destination_xy[0] - gate_xy[0],
            destination_xy[1] - gate_xy[1],
        ))
        if direction is None:
            return None
        best = None
        best_distance = float("inf")
        for record in self._records.values():
            if int(record["source_place_id"]) != int(source_place_id):
                continue
            prior_direction = _unit((
                float(record.get("physical_destination", (0.0, 0.0))[0])
                - float(record.get("physical_gate", (0.0, 0.0))[0]),
                float(record.get("physical_destination", (0.0, 0.0))[1])
                - float(record.get("physical_gate", (0.0, 0.0))[1]),
            ))
            # A doorway's directed side is part of its physical identity.
            # Without this check, repeated small SLAM projections can move a
            # single record through the gate and eventually reverse its
            # destination direction while retaining the same Portal ID.
            if prior_direction is None or (
                prior_direction[0] * direction[0]
                + prior_direction[1] * direction[1]
                <= 0.0
            ):
                continue
            gate_distance = _distance(record.get("physical_gate"), gate_xy)
            destination_distance = _distance(
                record.get("physical_destination"), destination_xy
            )
            distance = gate_distance + destination_distance
            if (
                gate_distance <= self.match_radius
                and destination_distance <= self.match_radius * 2.0
                and distance < best_distance
            ):
                best = record
                best_distance = distance
        return best

    def certify(
        self,
        source_place_id,
        gate_xy,
        destination_xy,
        now=0.0,
        task_version="",
        map_gate_xy=None,
        map_destination_xy=None,
    ):
        """Record a certified physical doorway and return ``(record, new)``."""
        gate_xy = _xy(gate_xy)
        destination_xy = _xy(destination_xy)
        if gate_xy is None or destination_xy is None:
            return None, False
        try:
            source_place_id = int(source_place_id)
        except (TypeError, ValueError):
            return None, False
        if source_place_id <= 0:
            return None, False
        record = self._matching_record(
            source_place_id, gate_xy, destination_xy,
        )
        created = record is None
        if record is not None and str(record.get("state", "")).strip().lower() == "failed":
            # Negative physical evidence is durable. A later SLAM snapshot
            # may move the projection, but it must not silently reopen the
            # same edge as a fresh certified Portal.
            return record, False
        if record is None:
            if len(self._records) >= self.limit:
                # Keep the newest bounded physical memory.  A record can only
                # be evicted after it is no longer the active graph edge.
                oldest = min(
                    self._records.values(),
                    key=lambda value: float(value.get("last_seen_at", 0.0)),
                )
                self._records.pop(int(oldest["id"]), None)
            record = {
                "id": int(self._next_id),
                "source_place_id": source_place_id,
                # Filled only after the physical crossing commits. Before
                # that point the destination is still an unowned hypothesis.
                "destination_place_id": None,
                "physical_gate": [gate_xy[0], gate_xy[1]],
                "physical_destination": [destination_xy[0], destination_xy[1]],
                "map_gate": None,
                "map_destination": None,
                "state": "certified",
                "certification_count": 0,
                "selection_count": 0,
                "crossing_count": 0,
                "failure_count": 0,
                "task_versions": set(),
                "first_seen_at": float(now),
                "last_seen_at": float(now),
                "last_selected_at": None,
                "last_crossed_at": None,
                "last_failed_at": None,
            }
            self._next_id += 1
            self._records[int(record["id"])] = record
        # ``physical_gate`` and ``physical_destination`` are the canonical
        # odometry-frame witness for this doorway.  They are immutable after
        # creation; only the map-frame projection below may follow SLAM.  A
        # transient map candidate must never rewrite the physical edge and
        # gradually reverse its direction across successive snapshots.
        map_gate_xy = _xy(map_gate_xy)
        map_destination_xy = _xy(map_destination_xy)
        if map_gate_xy is not None:
            record["map_gate"] = [map_gate_xy[0], map_gate_xy[1]]
        if map_destination_xy is not None:
            record["map_destination"] = [
                map_destination_xy[0], map_destination_xy[1]
            ]
        record["certification_count"] += 1
        record["last_seen_at"] = float(now)
        if task_version:
            record["task_versions"].add(str(task_version))
        # A failed edge is durable negative evidence for this physical gate.
        # It can be retried only through the explicit recovery transaction or
        # rediscovered as a genuinely different physical hypothesis; repeated
        # snapshots must not silently reopen the same failed edge.
        return record, created

    def _by_id(self, portal_id):
        try:
            return self._records.get(int(portal_id))
        except (TypeError, ValueError):
            return None

    def select(self, portal_id, now=0.0):
        """Mark one certified hypothesis as the selected graph action."""
        record = self._by_id(portal_id)
        if record is None:
            return None
        if str(record.get("state", "")).strip().lower() == "failed":
            # Failure is durable negative evidence for this physical edge.
            # A fresh map snapshot may create a different hypothesis, but it
            # must not resurrect this exact record by coordinate equality.
            return None
        record["state"] = "selected"
        record["selection_count"] += 1
        record["last_selected_at"] = float(now)
        return record

    def crossed(self, portal_id, now=0.0):
        """Commit a physical crossing without changing the hypothesis ID."""
        record = self._by_id(portal_id)
        if record is None:
            return None
        if str(record.get("state", "")).strip().lower() == "failed":
            # Durable negative evidence cannot be resurrected by a delayed
            # controller terminal or stale arrival callback.
            return None
        record["state"] = "crossed"
        record["crossing_count"] += 1
        record["last_crossed_at"] = float(now)
        return record

    def bind_destination(self, portal_id, destination_place_id, now=0.0):
        """Attach the durable destination Place after a verified crossing."""
        record = self._by_id(portal_id)
        if record is None:
            return None
        try:
            destination_place_id = int(destination_place_id)
        except (TypeError, ValueError):
            return record
        if destination_place_id <= 0:
            return record
        existing_destination = record.get("destination_place_id")
        if existing_destination is not None:
            try:
                return (
                    record
                    if int(existing_destination) == destination_place_id
                    else None
                )
            except (TypeError, ValueError):
                return None
        # A physical crossing must change Place identity.  Treating a
        # destination that equals the source as valid would create a graph
        # self-loop and turn the same doorway into an endless exploration
        # action.  Keep the crossed evidence for diagnosis, but refuse the
        # invalid binding so callers can fail closed.
        try:
            if int(record.get("source_place_id")) == destination_place_id:
                record["self_loop_rejection_count"] = int(
                    record.get("self_loop_rejection_count", 0)
                ) + 1
                record["last_binding_rejection_reason"] = (
                    "source_destination_place_equal"
                )
                return None
        except (TypeError, ValueError):
            return None
        record["destination_place_id"] = destination_place_id
        record["last_crossed_at"] = float(now)
        return record

    def crossed_reverse_for_destination(
        self, destination_place_id, physical_gate_xy, normal_xy,
        gate_radius=None,
    ):
        """Find a crossed edge whose reverse side is this Place's doorway.

        A newly entered Place sees the back of the doorway that was just
        crossed. That boundary is already represented by the undirected
        transit edge and must not become a second exploration Probe. Matching
        uses the durable physical gate, while the direction check keeps an
        adjacent doorway with the same coarse area from being suppressed.
        """
        try:
            destination_place_id = int(destination_place_id)
        except (TypeError, ValueError):
            return None
        gate = _xy(physical_gate_xy)
        normal = _unit(normal_xy)
        if destination_place_id <= 0 or gate is None or normal is None:
            return None
        radius = self.match_radius
        if gate_radius is not None:
            try:
                radius = max(0.05, float(gate_radius))
            except (TypeError, ValueError):
                radius = self.match_radius
        matches = []
        for record in self._records.values():
            if str(record.get("state", "")).strip().lower() != "crossed":
                continue
            try:
                if int(record.get("destination_place_id")) != destination_place_id:
                    continue
            except (TypeError, ValueError):
                continue
            prior_gate = _xy(record.get("physical_gate"))
            destination = _xy(record.get("physical_destination"))
            if prior_gate is None or destination is None:
                continue
            distance = _distance(prior_gate, gate)
            if distance > radius:
                continue
            direction = _unit((
                destination[0] - prior_gate[0],
                destination[1] - prior_gate[1],
            ))
            # The new observation must point back toward the source side.
            if direction is None or direction[0] * normal[0] + direction[1] * normal[1] >= 0.0:
                continue
            matches.append((distance, int(record["id"]), record))
        if not matches:
            return None
        return min(matches, key=lambda value: value[:2])[2]

    def crossed_gate_for_destination(
        self, destination_place_id, physical_gate_xy, gate_radius=None,
        candidate_direction_xy=None,
    ):
        """Find a crossed doorway band on a Place boundary.

        A structural opening can be wider than one grid cell.  Successive
        snapshots may therefore report points at opposite jambs of the same
        doorway, and the instantaneous opening normal may even flip.  The
        strict reverse query above is useful when the normal is trustworthy;
        this query is the fail-closed fallback: match the durable gate plane
        across the doorway's tangent span, while keeping the candidate close
        to that plane in the crossing direction.

        ``gate_radius`` is a geometric span derived by the caller from Place
        memory.  It is not a temporal retry threshold and does not authorize a
        crossing; it only identifies an already-crossed physical boundary.
        """
        try:
            destination_place_id = int(destination_place_id)
        except (TypeError, ValueError):
            return None
        gate = _xy(physical_gate_xy)
        candidate_direction = _unit(candidate_direction_xy)
        if destination_place_id <= 0 or gate is None:
            return None
        try:
            tangent_radius = (
                self.match_radius
                if gate_radius is None
                else max(self.match_radius, float(gate_radius))
            )
        except (TypeError, ValueError):
            tangent_radius = self.match_radius
        # Physical odometry is the durable frame.  Keep the normal offset
        # tight so a different doorway on the same wall cannot be swallowed by
        # the wider tangent span used for a wide opening.
        normal_radius = self.match_radius
        matches = []
        for record in self._records.values():
            if str(record.get("state", "")).strip().lower() != "crossed":
                continue
            try:
                if int(record.get("destination_place_id")) != destination_place_id:
                    continue
            except (TypeError, ValueError):
                continue
            prior_gate = _xy(record.get("physical_gate"))
            destination = _xy(record.get("physical_destination"))
            if prior_gate is None or destination is None:
                continue
            direction = _unit((
                destination[0] - prior_gate[0],
                destination[1] - prior_gate[1],
            ))
            if direction is None:
                continue
            if candidate_direction is not None:
                # The fallback still identifies a wide physical gate, but a
                # caller that has directional evidence must be on the source
                # side of the crossed edge.  Without this signed check a
                # same-side opening can be swallowed by the tangent span.
                if direction[0] * candidate_direction[0] + direction[1] * candidate_direction[1] >= 0.0:
                    continue
            delta = (gate[0] - prior_gate[0], gate[1] - prior_gate[1])
            normal_offset = abs(
                delta[0] * direction[0] + delta[1] * direction[1]
            )
            tangent_offset = abs(
                delta[0] * direction[1] - delta[1] * direction[0]
            )
            if (
                normal_offset > normal_radius
                or tangent_offset > tangent_radius
            ):
                continue
            matches.append(
                (
                    normal_offset + tangent_offset,
                    tangent_offset,
                    int(record["id"]),
                    record,
                )
            )
        if not matches:
            return None
        return min(matches, key=lambda value: value[:3])[3]

    def crossed_gate_at_physical_gate(self, physical_gate_xy):
        """Find any crossed edge sharing the same physical gate band.

        Place IDs are semantic observations and can split when SLAM revisits a
        doorway from the opposite side.  A crossed gate is nevertheless a
        single undirected transit boundary.  This narrow physical lookup is
        used only to suppress a duplicate structural hypothesis; it does not
        bind a destination Place or authorize a crossing.
        """
        gate = _xy(physical_gate_xy)
        if gate is None:
            return None
        matches = []
        for record in self._records.values():
            if str(record.get("state", "")).strip().lower() != "crossed":
                continue
            prior_gate = _xy(record.get("physical_gate"))
            if prior_gate is None:
                continue
            distance = _distance(prior_gate, gate)
            if distance <= self.match_radius:
                matches.append((distance, int(record["id"]), record))
        if not matches:
            return None
        return min(matches, key=lambda value: value[:2])[2]

    def incoming_for_place(self, destination_place_id, gate_xy=None):
        """Return forward hypotheses that arrived at one Place."""
        try:
            destination_place_id = int(destination_place_id)
        except (TypeError, ValueError):
            return []
        records = []
        for record in self._records.values():
            if int(record.get("destination_place_id") or -1) != destination_place_id:
                continue
            if gate_xy is not None and _distance(record.get("physical_gate"), gate_xy) > self.match_radius * 2.0:
                continue
            records.append(record)
        return sorted(records, key=lambda value: int(value["id"]))

    def unbound_for_source(self, source_place_id, map_epoch=None):
        """Return pending edges not already rejected in this map epoch."""
        try:
            source_place_id = int(source_place_id)
        except (TypeError, ValueError):
            return []
        if source_place_id <= 0:
            return []
        epoch = None
        try:
            epoch = None if map_epoch is None else int(map_epoch)
        except (TypeError, ValueError):
            pass
        records = []
        for record in self._records.values():
            try:
                source = int(record.get("source_place_id"))
            except (TypeError, ValueError):
                continue
            if (
                source == source_place_id
                and record.get("destination_place_id") is None
                and str(record.get("state", "")).strip().lower()
                in {"certified", "selected"}
                and not (
                    epoch is not None
                    and record.get("rejected_map_epoch") == epoch
                )
            ):
                records.append(record)
        return sorted(records, key=lambda value: int(value["id"]))

    # Compatibility alias; callers may use either spelling.
    unbound_portals_for_source = unbound_for_source

    def mark_unbound_rejected(self, portal_id, map_epoch, reason=""):
        """Suppress one invalid pending edge until the map epoch changes."""
        record = self._by_id(portal_id)
        if record is None:
            return None
        try:
            record["rejected_map_epoch"] = int(map_epoch)
        except (TypeError, ValueError):
            record["rejected_map_epoch"] = None
        record["last_rejection_reason"] = str(reason or "")
        return record

    def refresh_projection(
        self, portal_id, map_gate_xy=None, map_destination_xy=None,
        now=None, task_version="",
    ):
        """Refresh map-frame coordinates without changing Portal identity."""
        record = self._by_id(portal_id)
        if record is None or record.get("destination_place_id") is not None:
            return None
        if str(record.get("state", "")).strip().lower() not in (
            "certified",
            "selected",
        ):
            return None
        map_gate_xy = _xy(map_gate_xy)
        map_destination_xy = _xy(map_destination_xy)
        if map_gate_xy is not None:
            record["map_gate"] = [map_gate_xy[0], map_gate_xy[1]]
        if map_destination_xy is not None:
            record["map_destination"] = [
                map_destination_xy[0], map_destination_xy[1]
            ]
        if now is not None:
            record["last_seen_at"] = float(now)
        if task_version:
            record.setdefault("task_versions", set()).add(str(task_version))
        return record

    @staticmethod
    def place_is_covered(region):
        """Return whether a Place has already supplied observation evidence.

        ``dormant`` and ``ready_to_exit`` are explicit lifecycle states.  An
        ``open`` Place with a reached endpoint is covered as well, even if the
        current map has not yet allowed the normal close transition.  Keeping
        this predicate here gives portal admission and durable egress the same
        discrete interpretation of a covered graph vertex.
        """
        if not isinstance(region, dict):
            return False
        if str(region.get("state", "")).strip().lower() in (
            "dormant",
            "ready_to_exit",
        ):
            return True
        try:
            return int(region.get("endpoint_observations", 0)) > 0
        except (TypeError, ValueError):
            return False

    def _place_graph_adjacency(self):
        """Build undirected transit adjacency from certified crossings.

        A ledger record is directional because it records the observed source
        and destination.  Once that crossing is committed, the physical gate
        is a legal transit edge in both directions; the reverse execution is
        still admitted by the gate-side geometry check.  This graph is only a
        progress query and never authorizes an action by itself.
        """
        adjacency = {}
        for record in self._records.values():
            if str(record.get("state", "")).strip().lower() != "crossed":
                # A bound destination is not enough to make an edge physical.
                # The graph may only use a doorway after its crossing evidence
                # has committed; certification/selection remain hypotheses.
                continue
            try:
                source = int(record.get("source_place_id"))
                destination = int(record.get("destination_place_id"))
            except (TypeError, ValueError):
                continue
            if source <= 0 or destination <= 0 or source == destination:
                continue
            adjacency.setdefault(source, set()).add(destination)
            adjacency.setdefault(destination, set()).add(source)
        return adjacency

    def has_unresolved_path(
        self, start_place_id, place_memory=None, work_item_ledger=None,
        portal_probe_ledger=None,
    ):
        """Return whether a covered Place can transit to graph progress.

        The search is intentionally over discrete Place/Portal identities, not
        coordinates or distance thresholds.  A path is useful when it reaches
        an unobserved Place, a Place with unresolved observation work, or an
        active certified Portal whose destination has not been bound yet.
        A missing Place record is treated conservatively as no progress: the
        graph must have an auditable durable vertex before it can justify a
        covered-to-covered transit.
        """
        try:
            start_place_id = int(start_place_id)
        except (TypeError, ValueError):
            return False
        if start_place_id <= 0 or place_memory is None:
            return False
        lookup = getattr(place_memory, "by_id", None)
        if lookup is None:
            return False

        adjacency = self._place_graph_adjacency()
        queue = [start_place_id]
        visited = set()
        pending_states = {"certified", "selected"}
        while queue:
            place_id = queue.pop(0)
            if place_id in visited:
                continue
            visited.add(place_id)
            region = lookup(place_id)
            if region is None:
                continue
            if not self.place_is_covered(region):
                return True
            if work_item_ledger is not None:
                unresolved = getattr(work_item_ledger, "unresolved_count", None)
                if unresolved is not None:
                    try:
                        if int(unresolved(place_id)) > 0:
                            return True
                    except (TypeError, ValueError):
                        pass
            if portal_probe_ledger is not None:
                unresolved_probes = getattr(
                    portal_probe_ledger, "unresolved_count", None,
                )
                if unresolved_probes is not None:
                    try:
                        if int(unresolved_probes(place_id)) > 0:
                            return True
                    except (TypeError, ValueError):
                        pass

            # An unbound certified edge is an information-bearing doorway. It
            # is the graph equivalent of a frontier, but unlike a frontier
            # point it retains its source identity across SLAM updates.
            for record in self._records.values():
                try:
                    source = int(record.get("source_place_id"))
                except (TypeError, ValueError):
                    continue
                if (
                    source == place_id
                    and record.get("destination_place_id") is None
                    and str(record.get("state", "")).strip().lower()
                    in pending_states
                ):
                    return True

            for neighbour in sorted(adjacency.get(place_id, ())):
                if neighbour not in visited:
                    queue.append(neighbour)
        return False

    def allows_covered_transit(
        self, source_region, destination_region, place_memory=None,
        work_item_ledger=None, portal_probe_ledger=None,
        snapshot_progress=False,
    ):
        """Allow a covered-to-covered edge only when it leads to progress.

        An outward edge from an unobserved/open Place is ordinary exploration
        and does not need this guard.  Once both endpoints are covered, a
        direct backtrack is useful only if the destination vertex can reach an
        unresolved Place, WorkItem, or certified unbound Portal in the durable
        graph.  Equal Place IDs are never progress.
        """
        if not self.place_is_covered(source_region) or not self.place_is_covered(
            destination_region
        ):
            return True
        if snapshot_progress:
            # The current occupancy snapshot contains a live unknown boundary
            # in the destination Place. This is direct information progress,
            # even if the durable graph has not certified the next doorway yet.
            return True
        try:
            source_id = int(source_region.get("id"))
            destination_id = int(destination_region.get("id"))
        except (AttributeError, TypeError, ValueError):
            return False
        if source_id <= 0 or destination_id <= 0 or source_id == destination_id:
            return False
        return self.has_unresolved_path(
            destination_id,
            place_memory=place_memory,
            work_item_ledger=work_item_ledger,
            portal_probe_ledger=portal_probe_ledger,
        )

    def failed(self, portal_id, now=0.0):
        """Record one failed attempt while keeping the doorway discoverable."""
        record = self._by_id(portal_id)
        if record is None:
            return None
        if str(record.get("state", "")).strip().lower() == "failed":
            return record
        record["state"] = "failed"
        record["failure_count"] += 1
        record["last_failed_at"] = float(now)
        return record

    def execution_failed(self, portal_id, now=0.0):
        """Record controller failure without invalidating a crossed Portal.

        A failed source-side probe is negative evidence about an un-crossed
        doorway and belongs in :meth:`failed`.  Once the physical edge has
        crossed successfully, a later reverse-egress/controller failure says
        nothing about doorway validity; preserve ``state=crossed`` so the graph
        can still use that edge as transit after an explicit recovery route.
        """
        record = self._by_id(portal_id)
        if record is None:
            return None
        record["execution_failure_count"] = int(
            record.get("execution_failure_count", 0)
        ) + 1
        record["last_execution_failed_at"] = float(now)
        return record

    def get(self, portal_id):
        return self._by_id(portal_id)

    def snapshot(self):
        """Return JSON-safe records for diagnostics and experiment logs."""
        result = []
        for portal_id in sorted(self._records):
            record = dict(self._records[portal_id])
            record["task_versions"] = sorted(record["task_versions"])
            result.append(record)
        return result
