"""Durable source-side Portal observation obligations.

An unknown opening is initially only an information hypothesis.  This ledger
gives it a physical identity before a structural Place transition is visible,
so a changing SLAM grid cannot recreate the same probe as a new frontier.
The ledger deliberately contains no ROS or motion-planner policy.
"""

import math


PROBE_PENDING = "pending"
PROBE_ACTIVE = "active"
PROBE_SOURCE_ARRIVED = "source_arrived"
PROBE_DESTINATION_ACTIVE = "destination_active"
PROBE_AWAITING_PROJECTION = "awaiting_projection"
PROBE_CERTIFIED = "certified"
PROBE_OBSERVED = "observed"
PROBE_REJECTED = "rejected"


def _xy(value):
    if value is None:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def _unit(value):
    point = _xy(value)
    if point is None:
        return None
    length = math.hypot(point[0], point[1])
    if not math.isfinite(length) or length <= 1e-9:
        return None
    return point[0] / length, point[1] / length


def _distance(first, second):
    first, second = _xy(first), _xy(second)
    if first is None or second is None:
        return float("inf")
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _epoch(value):
    """Normalize an optional topology epoch without inventing one."""
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _phase(value, default="source"):
    """Normalize the two observation phases used by a probe Attempt."""
    value = str(value or "").strip().lower()
    return value if value in ("source", "destination") else default


def _record_point(record):
    """Read a viewpoint from a typed record, tolerating older snapshots."""
    if not isinstance(record, dict):
        return None
    return _xy(record.get("xy", record.get("viewpoint_xy")))


def _epoch_matches(record_epoch, requested_epoch):
    """Keep known history scoped to the requested map epoch.

    ``None`` means the record predates epoch provenance.  Such history remains
    conservative and matches any explicitly requested epoch, while records
    with provenance can be reused after a map-version transition.
    """
    requested_epoch = _epoch(requested_epoch)
    if requested_epoch is None:
        return True
    record_epoch = _epoch(record_epoch)
    return record_epoch is None or record_epoch == requested_epoch


def destination_reobserve_available(record, map_epoch=None):
    """Return whether a source-arrived probe may acquire a destination view.

    A route failure is different from an inconclusive observation.  The former
    permits an immediate alternative safe viewpoint in the same map epoch,
    while the latter waits for new map evidence so a fast timer cannot replay
    the same physical observation forever.  Legacy records without phase
    metadata remain compatible and are treated as initially actionable.
    """
    if not isinstance(record, dict):
        return False
    if str(record.get("state", "")).strip().lower() != PROBE_SOURCE_ARRIVED:
        return False
    if bool(record.get("destination_retry_allowed", False)):
        return True
    if "last_phase" not in record:
        return True
    if str(record.get("last_phase") or "").strip().lower() != "destination":
        return True
    attempted = record.get("destination_attempt_epoch")
    current = (
        record.get("last_map_epoch")
        if map_epoch is None else map_epoch
    )
    if attempted is None or current is None:
        return False
    try:
        return int(current) > int(attempted)
    except (TypeError, ValueError):
        return False


DEFAULT_VIEWPOINT_MATCH_RADIUS = 0.35


class PortalProbeLedger:
    """Store one directional probe obligation per physical doorway.

    ``match_radius`` identifies the doorway across changing map projections;
    it is deliberately independent from the much smaller radius used to
    deduplicate observation poses.  A room-sized gate association radius must
    never make every viewpoint in that room look like the same observation.
    """

    def __init__(
        self, match_radius=0.30, limit=512, viewpoint_match_radius=None,
    ):
        self.match_radius = max(0.05, float(match_radius))
        if viewpoint_match_radius is None:
            viewpoint_match_radius = min(
                self.match_radius, DEFAULT_VIEWPOINT_MATCH_RADIUS,
            )
        self.viewpoint_match_radius = max(
            0.05, float(viewpoint_match_radius),
        )
        self.limit = max(16, int(limit))
        self._next_id = 1
        self._records = {}

    def _matching(self, source_place_id, physical_gate_xy, normal_xy):
        gate = _xy(physical_gate_xy)
        normal = _unit(normal_xy)
        if gate is None or normal is None:
            return None
        best = None
        best_distance = float("inf")
        for record in self._records.values():
            if int(record["source_place_id"]) != int(source_place_id):
                continue
            prior_normal = _unit(record.get("normal_xy"))
            if prior_normal is None:
                continue
            # Opposite directions are different information requests even at
            # the same physical gate.
            if prior_normal[0] * normal[0] + prior_normal[1] * normal[1] <= 0.0:
                continue
            distance = _distance(record.get("physical_gate_xy"), gate)
            if distance <= self.match_radius and distance < best_distance:
                best, best_distance = record, distance
        return best

    def observe(
        self,
        source_place_id,
        physical_gate_xy,
        normal_xy,
        *,
        map_gate_xy=None,
        opening_cell=None,
        map_epoch=None,
        now=0.0,
        observation_source="frontier",
    ):
        """Associate one map observation with a durable physical probe."""
        try:
            source_place_id = int(source_place_id)
        except (TypeError, ValueError):
            return None
        if source_place_id <= 0:
            return None
        gate = _xy(physical_gate_xy)
        normal = _unit(normal_xy)
        if gate is None or normal is None:
            return None
        record = self._matching(source_place_id, gate, normal)
        if record is None:
            if len(self._records) >= self.limit:
                # Do not evict an active obligation.  Prefer the oldest
                # terminal record; a full ledger is a diagnostic condition,
                # not permission to duplicate a live probe.
                terminals = [
                    item for item in self._records.values()
                    if item.get("state") != PROBE_ACTIVE
                ]
                if not terminals:
                    return None
                oldest = min(
                    terminals,
                    key=lambda item: float(item.get("last_seen_at", 0.0)),
                )
                self._records.pop(int(oldest["id"]), None)
            record = {
                "id": int(self._next_id),
                "source_place_id": source_place_id,
                "physical_gate_xy": [gate[0], gate[1]],
                "normal_xy": [normal[0], normal[1]],
                "map_gate_xy": None,
                "opening_cell": None,
                "work_item_id": None,
                "portal_id": None,
                "state": PROBE_PENDING,
                "attempt_count": 0,
                "observation_count": 0,
                "first_seen_at": float(now),
                "last_seen_at": float(now),
                "last_started_at": None,
                "last_finished_at": None,
                "last_reason": "opening_discovered",
                "active_viewpoint_xy": None,
                "last_viewpoint_xy": None,
                "viewpoint_history": [],
                # The legacy lists above remain part of the public status
                # shape.  Typed records are the source of truth for phase and
                # map-version scoped deduplication.
                "viewpoint_records": [],
                "active_phase": None,
                "active_map_epoch": None,
                "failed_viewpoints": [],
                "last_map_epoch": None,
                "destination_attempt_epoch": None,
                "destination_retry_allowed": False,
                "projection_resume_state": None,
                "projection_blocked_map_epoch": None,
                "projection_blocked_reason": "",
                "viewpoint_match_radius": float(self.viewpoint_match_radius),
                "observation_source": str(observation_source or "frontier"),
            }
            self._next_id += 1
            self._records[int(record["id"])] = record
        if record.get("state") == PROBE_AWAITING_PROJECTION:
            # A projection gap is reopened by a newer topology snapshot, not
            # by the same physical observation being replayed.  This is an
            # evidence-version transition, so the fast planning loop cannot
            # turn one unmaterializable viewpoint into repeated attempts.
            blocked_epoch = _epoch(record.get("projection_blocked_map_epoch"))
            observed_epoch = _epoch(map_epoch)
            has_new_epoch = (
                blocked_epoch is None
                or (
                    observed_epoch is not None
                    and observed_epoch > blocked_epoch
                )
            )
            if has_new_epoch:
                record["state"] = record.pop(
                    "projection_resume_state", PROBE_PENDING,
                ) or PROBE_PENDING
                record["projection_resume_state"] = None
                record["projection_blocked_map_epoch"] = None
                record["projection_blocked_reason"] = ""
        record["physical_gate_xy"] = [gate[0], gate[1]]
        map_gate = _xy(map_gate_xy)
        if map_gate is not None:
            record["map_gate_xy"] = [map_gate[0], map_gate[1]]
        if opening_cell is not None:
            try:
                record["opening_cell"] = [int(opening_cell[0]), int(opening_cell[1])]
            except (IndexError, TypeError, ValueError):
                pass
        if observation_source:
            record["observation_source"] = str(observation_source)
        try:
            if map_epoch is not None:
                record["last_map_epoch"] = int(map_epoch)
        except (TypeError, ValueError):
            pass
        record["observation_count"] += 1
        record["last_seen_at"] = float(now)
        return self.view(record)

    def bind_work_item(self, probe_id, work_item_id):
        """Associate the doorway hypothesis with its place-owned WorkItem."""
        record = self.get(probe_id)
        if record is None or work_item_id is None:
            return record
        try:
            work_item_id = int(work_item_id)
        except (TypeError, ValueError):
            return record
        if work_item_id > 0:
            record["work_item_id"] = work_item_id
        return self.view(record)

    def bind_portal(self, probe_id, portal_id):
        """Associate a source-side probe with its durable Portal hypothesis."""
        record = self.get(probe_id)
        if record is None:
            return None
        try:
            portal_id = int(portal_id)
        except (TypeError, ValueError):
            return self.view(record)
        if portal_id > 0:
            record["portal_id"] = portal_id
        return self.view(record)

    def refresh_projection(self, probe_id, *, map_gate_xy=None, opening_cell=None, now=None):
        """Refresh only the transient map projection of a physical probe."""
        record = self.get(probe_id)
        if record is None:
            return None
        map_gate = _xy(map_gate_xy)
        if map_gate is not None:
            record["map_gate_xy"] = [map_gate[0], map_gate[1]]
        if opening_cell is not None:
            try:
                record["opening_cell"] = [int(opening_cell[0]), int(opening_cell[1])]
            except (IndexError, TypeError, ValueError):
                pass
        if now is not None:
            record["last_seen_at"] = float(now)
        return self.view(record)

    def get(self, probe_id):
        try:
            return self._records.get(int(probe_id))
        except (TypeError, ValueError):
            return None

    def available(self, probe_id):
        record = self.get(probe_id)
        return record is not None and record.get("state") == PROBE_PENDING

    def mark_projection_unavailable(
        self, probe_id, map_epoch=None, reason="",
    ):
        """Park a probe until new map evidence reprojects its physical gate.

        This is a transient evidence state, not a route failure or negative
        doorway observation.  Keeping it in the ledger prevents the planner
        from redispatching the same unmaterializable viewpoint on every fast
        timer tick while preserving the physical identity for a later map
        update.
        """
        record = self.get(probe_id)
        if record is None:
            return None
        state = str(record.get("state", "")).strip().lower()
        if state not in (PROBE_PENDING, PROBE_SOURCE_ARRIVED):
            return self.view(record)
        record["projection_resume_state"] = state
        try:
            record["projection_blocked_map_epoch"] = (
                None if map_epoch is None else int(map_epoch)
            )
        except (TypeError, ValueError):
            record["projection_blocked_map_epoch"] = None
        record["projection_blocked_reason"] = str(reason or "")
        record["state"] = PROBE_AWAITING_PROJECTION
        record["last_reason"] = "projection_unavailable"
        return self.view(record)

    def destination_available(
        self, probe_id, viewpoint_xy=None, map_epoch=None,
    ):
        """Return whether a source-arrived probe can acquire another view."""
        record = self.get(probe_id)
        return (
            record is not None
            and record.get("state") == PROBE_SOURCE_ARRIVED
            and self._destination_reobserve_available(record, map_epoch)
            and self.viewpoint_available(
                probe_id,
                viewpoint_xy,
                phase="destination",
                map_epoch=map_epoch,
            )
        )

    @staticmethod
    def _destination_reobserve_available(record, map_epoch=None):
        """Apply the shared destination-view lifecycle rule."""
        return destination_reobserve_available(record, map_epoch=map_epoch)

    def unresolved_count(self, source_place_id):
        """Count pending source-side probes owned by one physical Place."""
        try:
            source_place_id = int(source_place_id)
        except (TypeError, ValueError):
            return 0
        return sum(
            1
            for record in self._records.values()
            if int(record.get("source_place_id", -1)) == source_place_id
            and record.get("state") in (
                PROBE_PENDING,
                PROBE_ACTIVE,
                PROBE_SOURCE_ARRIVED,
                PROBE_DESTINATION_ACTIVE,
                PROBE_AWAITING_PROJECTION,
            )
        )

    def pending_for_source(
        self, source_place_id, include_source_arrived=True, map_epoch=None,
    ):
        """Return source obligations that can be rehydrated after map updates."""
        try:
            source_place_id = int(source_place_id)
        except (TypeError, ValueError):
            return []
        records = []
        for record in sorted(
            self._records.values(), key=lambda item: int(item["id"])
        ):
            if int(record.get("source_place_id", -1)) != source_place_id:
                continue
            if record.get("state") == PROBE_AWAITING_PROJECTION:
                # A newer map is the explicit retry boundary for a parked
                # physical probe.  Reopen the same durable record here so the
                # graph adapter can retry it without creating a new identity.
                blocked_epoch = _epoch(
                    record.get("projection_blocked_map_epoch")
                )
                requested_epoch = _epoch(map_epoch)
                if (
                    blocked_epoch is None
                    or (
                        requested_epoch is not None
                        and requested_epoch > blocked_epoch
                    )
                ):
                    record["state"] = record.pop(
                        "projection_resume_state", PROBE_PENDING
                    ) or PROBE_PENDING
                    record["projection_resume_state"] = None
                    record["projection_blocked_map_epoch"] = None
                    record["projection_blocked_reason"] = ""
            allowed_states = (
                (PROBE_PENDING, PROBE_SOURCE_ARRIVED)
                if include_source_arrived else (PROBE_PENDING,)
            )
            if record.get("state") in allowed_states:
                records.append(self.view(record))
        if not include_source_arrived:
            return records
        return [
            record for record in records
            if record.get("state") != PROBE_SOURCE_ARRIVED
            or self._destination_reobserve_available(record, map_epoch)
        ]

    def probe_for_work_item(self, work_item_id):
        """Return the durable probe currently owning one WorkItem."""
        try:
            work_item_id = int(work_item_id)
        except (TypeError, ValueError):
            return None
        for record in self._records.values():
            if record.get("work_item_id") == work_item_id:
                return self.view(record)
        return None

    def certify_matching(
        self, source_place_id, physical_gate_xy, direction_xy, portal_id,
        now=0.0,
    ):
        """Promote a source-arrived probe when a directed Portal is certified."""
        gate = _xy(physical_gate_xy)
        direction = _unit(direction_xy)
        if gate is None or direction is None:
            return None
        try:
            source_place_id = int(source_place_id)
        except (TypeError, ValueError):
            return None
        matches = []
        for record in self._records.values():
            if (
                int(record.get("source_place_id", -1)) != source_place_id
                or record.get("state") != PROBE_SOURCE_ARRIVED
            ):
                continue
            prior_direction = _unit(record.get("normal_xy"))
            if prior_direction is None:
                continue
            if prior_direction[0] * direction[0] + prior_direction[1] * direction[1] <= 0.0:
                continue
            distance = _distance(record.get("physical_gate_xy"), gate)
            if distance <= self.match_radius:
                matches.append((distance, int(record["id"]), record))
        if not matches:
            return None
        _distance_value, _record_id, record = min(matches, key=lambda value: value[:2])
        record["state"] = PROBE_CERTIFIED
        record["portal_id"] = int(portal_id)
        record["last_finished_at"] = float(now)
        record["last_reason"] = "directed_portal_certified"
        return self.view(record)

    def start(self, probe_id, now=0.0, viewpoint_xy=None, map_epoch=None):
        """Reserve one probe when its executable route is published."""
        record = self.get(probe_id)
        if record is None or record.get("state") != PROBE_PENDING:
            return None
        record["state"] = PROBE_ACTIVE
        record["attempt_count"] += 1
        record["last_started_at"] = float(now)
        record["last_reason"] = "probe_route_started"
        viewpoint = _xy(viewpoint_xy)
        record["active_viewpoint_xy"] = (
            None if viewpoint is None else [viewpoint[0], viewpoint[1]]
        )
        record["active_phase"] = "source"
        record["active_map_epoch"] = _epoch(
            record.get("last_map_epoch") if map_epoch is None else map_epoch
        )
        return self.view(record)

    def start_destination(
        self, probe_id, now=0.0, viewpoint_xy=None, map_epoch=None,
    ):
        """Start a destination-evidence Attempt for a source-arrived probe."""
        record = self.get(probe_id)
        viewpoint = _xy(viewpoint_xy)
        if not self.destination_available(
            probe_id, viewpoint, map_epoch=map_epoch,
        ):
            return None
        record["state"] = PROBE_DESTINATION_ACTIVE
        record["attempt_count"] += 1
        record["last_started_at"] = float(now)
        record["last_reason"] = "destination_evidence_route_started"
        record["active_viewpoint_xy"] = (
            None if viewpoint is None else [viewpoint[0], viewpoint[1]]
        )
        record["active_phase"] = "destination"
        record["active_map_epoch"] = _epoch(
            record.get("last_map_epoch") if map_epoch is None else map_epoch
        )
        record["destination_attempt_epoch"] = record.get("active_map_epoch")
        record["destination_retry_allowed"] = False
        return self.view(record)

    def viewpoint_available(
        self, probe_id, viewpoint_xy, *, phase=None, map_epoch=None,
    ):
        """Return whether a probe may use a new physical viewpoint.

        Source and destination views are different evidence facts.  A source
        stance therefore cannot suppress the first destination-side attempt,
        even when SLAM projects both stances onto the same coordinates.  When
        an epoch is supplied, only history from that phase and epoch is a
        duplicate; this keeps map churn from turning an old projection into a
        permanent lock.
        """
        record = self.get(probe_id)
        viewpoint = _xy(viewpoint_xy)
        if record is None or viewpoint is None:
            return True
        requested_phase = _phase(
            phase,
            default=(
                "destination"
                if record.get("state") in (
                    PROBE_SOURCE_ARRIVED,
                    PROBE_DESTINATION_ACTIVE,
                ) else "source"
            ),
        )
        typed_records = record.get("viewpoint_records", ())
        prior_viewpoints = []
        if typed_records:
            for prior in typed_records:
                if not isinstance(prior, dict):
                    continue
                if _phase(prior.get("phase")) != requested_phase:
                    continue
                if not _epoch_matches(prior.get("map_epoch"), map_epoch):
                    continue
                point = _record_point(prior)
                if point is not None:
                    prior_viewpoints.append(point)
        elif requested_phase == "source":
            # Legacy lists have no phase provenance.  Interpreting them as
            # source-side history preserves the pre-migration behavior while
            # avoiding false destination suppression.
            prior_viewpoints.extend(record.get("failed_viewpoints", ()))
            prior_viewpoints.extend(record.get("viewpoint_history", ()))
        return all(
            _distance(viewpoint, previous) > self.viewpoint_match_radius
            for previous in prior_viewpoints
        )

    def reject_viewpoint(
        self, probe_id, viewpoint_xy, *, phase=None, reason="", now=0.0,
    ):
        """Persist a failed lease admission for one physical viewpoint.

        A route can be rejected before ``start`` when its durable probe is
        already owned by another Attempt, or when a stale projection no longer
        satisfies the phase contract.  Without a durable record of that fact,
        the next SLAM snapshot can keep publishing the same map endpoint.  The
        rejection is intentionally map-epoch independent: it is evidence that
        this physical stance cannot acquire the requested probe lease, not a
        negative observation about the doorway.
        """
        record = self.get(probe_id)
        viewpoint = _xy(viewpoint_xy)
        if record is None or viewpoint is None:
            return None
        requested_phase = _phase(
            phase,
            default=(
                "destination"
                if record.get("state") in (
                    PROBE_SOURCE_ARRIVED,
                    PROBE_DESTINATION_ACTIVE,
                ) else "source"
            ),
        )
        # Keep the rejection idempotent across repeated planning snapshots.
        for prior in record.get("viewpoint_records", ()):
            if not isinstance(prior, dict):
                continue
            if _phase(prior.get("phase")) != requested_phase:
                continue
            prior_point = _record_point(prior)
            if (
                str(prior.get("outcome", "")) == "activation_rejected"
                and prior_point is not None
                and _distance(prior_point, viewpoint)
                <= self.viewpoint_match_radius
            ):
                record["last_reason"] = str(reason or "viewpoint_rejected")
                record["last_finished_at"] = float(now)
                return self.view(record)
        record.setdefault("failed_viewpoints", []).append(
            [viewpoint[0], viewpoint[1]]
        )
        record.setdefault("viewpoint_history", []).append(
            [viewpoint[0], viewpoint[1]]
        )
        record.setdefault("viewpoint_records", []).append({
            "xy": [viewpoint[0], viewpoint[1]],
            "phase": requested_phase,
            # ``None`` is a deliberate wildcard in ``_epoch_matches``: this
            # physical admission failure must not be replayed after SLAM
            # relabels the same doorway.
            "map_epoch": None,
            "outcome": "activation_rejected",
        })
        record["last_viewpoint_xy"] = [viewpoint[0], viewpoint[1]]
        record["last_finished_at"] = float(now)
        record["last_phase"] = requested_phase
        record["last_reason"] = str(reason or "viewpoint_rejected")
        return self.view(record)

    def finish(self, probe_id, result, now=0.0, reason=""):
        """Close or release a probe Attempt using an explicit outcome."""
        record = self.get(probe_id)
        if record is None:
            return None
        result = str(result or "").strip().lower()
        phase = _phase(record.get("active_phase"))
        attempt_epoch = _epoch(
            record.get("active_map_epoch")
            if record.get("active_map_epoch") is not None
            else record.get("last_map_epoch")
        )
        if result == "source_arrived":
            record["state"] = PROBE_SOURCE_ARRIVED
        elif result == "certified":
            record["state"] = PROBE_CERTIFIED
        elif result == "observed":
            record["state"] = PROBE_OBSERVED
        elif result == "rejected":
            record["state"] = PROBE_REJECTED
        elif result in ("failed", "blocked"):
            # Route failure is not physical negative evidence.  Releasing the
            # obligation lets WorkItem viewpoint lineage choose another view.
            record["state"] = (
                PROBE_SOURCE_ARRIVED
                if phase == "destination" else PROBE_PENDING
            )
            failed_viewpoint = _xy(record.get("active_viewpoint_xy"))
            if failed_viewpoint is not None:
                record.setdefault("failed_viewpoints", []).append(
                    [failed_viewpoint[0], failed_viewpoint[1]]
                )
            # A controller failure did not produce a new observation. Release
            # the destination epoch gate so the next selection can try a
            # different physical viewpoint immediately.
            record["destination_retry_allowed"] = phase == "destination"
        else:
            raise ValueError("unknown portal probe result: %s" % result)
        if result not in ("failed", "blocked"):
            record["destination_retry_allowed"] = False
        record["last_finished_at"] = float(now)
        record["last_reason"] = str(reason or result)
        last_viewpoint = _xy(record.get("active_viewpoint_xy"))
        record["last_viewpoint_xy"] = (
            None if last_viewpoint is None
            else [last_viewpoint[0], last_viewpoint[1]]
        )
        if last_viewpoint is not None:
            record.setdefault("viewpoint_history", []).append(
                [last_viewpoint[0], last_viewpoint[1]]
            )
            record.setdefault("viewpoint_records", []).append({
                "xy": [last_viewpoint[0], last_viewpoint[1]],
                "phase": phase,
                "map_epoch": attempt_epoch,
                "outcome": result,
            })
        record["last_phase"] = phase
        record["active_phase"] = None
        record["active_map_epoch"] = None
        record["active_viewpoint_xy"] = None
        return self.view(record)

    @staticmethod
    def view(record):
        """Return a JSON-safe copy for status events and tests."""
        return {
            "id": int(record["id"]),
            "source_place_id": int(record["source_place_id"]),
            "physical_gate_xy": list(record["physical_gate_xy"]),
            "normal_xy": list(record["normal_xy"]),
            "map_gate_xy": (
                None if record.get("map_gate_xy") is None
                else list(record["map_gate_xy"])
            ),
            "opening_cell": (
                None if record.get("opening_cell") is None
                else list(record["opening_cell"])
            ),
            "work_item_id": record.get("work_item_id"),
            "portal_id": record.get("portal_id"),
            "state": str(record["state"]),
            "attempt_count": int(record["attempt_count"]),
            "observation_count": int(record["observation_count"]),
            "first_seen_at": float(record["first_seen_at"]),
            "last_seen_at": float(record["last_seen_at"]),
            "last_started_at": record.get("last_started_at"),
            "last_finished_at": record.get("last_finished_at"),
            "last_reason": str(record.get("last_reason", "")),
            "last_viewpoint_xy": (
                None if record.get("last_viewpoint_xy") is None
                else list(record["last_viewpoint_xy"])
            ),
            "viewpoint_history": [
                list(viewpoint)
                for viewpoint in record.get("viewpoint_history", ())
            ],
            "viewpoint_records": [
                {
                    "xy": list(point),
                    "phase": _phase(item.get("phase")),
                    "map_epoch": _epoch(item.get("map_epoch")),
                    "outcome": str(item.get("outcome", "")),
                }
                for item in record.get("viewpoint_records", ())
                if isinstance(item, dict)
                for point in [_record_point(item)]
                if point is not None
            ],
            "last_phase": record.get("last_phase"),
            "active_phase": record.get("active_phase"),
            "active_map_epoch": _epoch(record.get("active_map_epoch")),
            "last_map_epoch": record.get("last_map_epoch"),
            "destination_attempt_epoch": record.get("destination_attempt_epoch"),
            "destination_retry_allowed": bool(
                record.get("destination_retry_allowed", False)
            ),
            "projection_resume_state": record.get("projection_resume_state"),
            "projection_blocked_map_epoch": record.get(
                "projection_blocked_map_epoch"
            ),
            "projection_blocked_reason": str(
                record.get("projection_blocked_reason", "")
            ),
            "viewpoint_match_radius": float(record.get(
                "viewpoint_match_radius", DEFAULT_VIEWPOINT_MATCH_RADIUS,
            )),
            "observation_source": str(
                record.get("observation_source", "frontier")
            ),
            "failed_viewpoints": [
                list(viewpoint)
                for viewpoint in record.get("failed_viewpoints", ())
            ],
        }

    def snapshot(self):
        return [self.view(self._records[key]) for key in sorted(self._records)]


__all__ = [
    "PROBE_ACTIVE",
    "PROBE_AWAITING_PROJECTION",
    "PROBE_CERTIFIED",
    "PROBE_DESTINATION_ACTIVE",
    "PROBE_OBSERVED",
    "PROBE_PENDING",
    "PROBE_REJECTED",
    "PROBE_SOURCE_ARRIVED",
    "PortalProbeLedger",
]
