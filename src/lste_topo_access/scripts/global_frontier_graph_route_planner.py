"""Pure graph-level route planning for Place/Portal/WorkItem exploration.

The occupancy grid is a short-lived geometric observation.  This module uses
only durable identities and therefore cannot be destabilised by a new SLAM
component label or by a frontier moving a few cells.  It chooses the next
*discrete* graph action and leaves endpoint construction, Navfn validation,
and TEB control to the existing ROS layers.
"""

from collections import defaultdict, deque
from dataclasses import dataclass

from global_frontier_evidence_contract import action_contract_for

from global_frontier_evidence_contract import (
    EVIDENCE_DESTINATION_VIEW,
    OWNER_PORTAL_PROBE,
    next_evidence_gap,
)
from global_frontier_portal_probe_ledger import destination_reobserve_available


PLAN_READY = "ready"
PLAN_COMPLETE = "complete"
PLAN_BLOCKED = "blocked"

ACTION_BOOTSTRAP = "bootstrap_observation"
ACTION_OBSERVE_LOCAL_WORK = "observe_local_work"
ACTION_RETRY_VIEWPOINT = "retry_viewpoint"
ACTION_REINSPECT_TARGET = "reinspect_target"
ACTION_PROBE_PORTAL = "probe_portal"
ACTION_CROSS_PORTAL = "cross_portal"
ACTION_HOLD = "hold"

_COVERED_STATES = {"dormant", "ready_to_exit", "suspended", "transit"}
_PORTAL_TRANSIT_STATES = {"crossed"}
_UNBOUND_PORTAL_STATES = {"certified", "selected"}
_AVAILABLE_PROBE_STATES = {"pending", "source_arrived"}


@dataclass(frozen=True)
class GraphRoutePlan:
    """Immutable result of one durable graph planning pass."""

    status: str
    action: str
    # The graph decision is valid only for this immutable map snapshot.
    map_epoch: object = None
    current_place_id: object = None
    target_place_id: object = None
    obligation_kind: str = ""
    obligation_id: object = None
    portal_path: tuple = ()
    first_portal_id: object = None
    reason: str = ""
    required_evidence: tuple = ()
    produces_evidence: tuple = ()
    # ``source`` and ``destination`` are separate information obligations for
    # one physical PortalProbe.  The phase is immutable for this route lease.
    portal_probe_phase: str = ""

    def __post_init__(self):
        """Attach the evidence boundary without changing route geometry."""
        contract = action_contract_for(
            self.action,
            current_place_id=self.current_place_id,
            obligation_kind=self.obligation_kind,
            obligation_id=self.obligation_id,
            target_place_id=self.target_place_id,
            first_portal_id=self.first_portal_id,
            portal_probe_phase=self.portal_probe_phase,
        )
        if not self.required_evidence and contract.requires:
            object.__setattr__(self, "required_evidence", tuple(contract.requires))
        if not self.produces_evidence and contract.produces:
            object.__setattr__(self, "produces_evidence", tuple(contract.produces))

    def as_dict(self):
        """Return a JSON-safe event payload without exposing mutable state."""
        def evidence_payload(items):
            return [
                item.as_dict() if callable(getattr(item, "as_dict", None)) else item
                for item in items
            ]

        return {
            "status": str(self.status),
            "action": str(self.action),
            "map_epoch": self.map_epoch,
            "current_place_id": self.current_place_id,
            "target_place_id": self.target_place_id,
            "obligation_kind": str(self.obligation_kind),
            "obligation_id": self.obligation_id,
            "portal_path": [int(portal_id) for portal_id in self.portal_path],
            "first_portal_id": self.first_portal_id,
            "reason": str(self.reason),
            "required_evidence": evidence_payload(self.required_evidence),
            "produces_evidence": evidence_payload(self.produces_evidence),
            "portal_probe_phase": str(self.portal_probe_phase or ""),
        }

    def signature(self):
        """Return stable fields suitable for event de-duplication."""
        return (
            self.status,
            self.action,
            self.map_epoch,
            self.current_place_id,
            self.target_place_id,
            self.obligation_kind,
            self.obligation_id,
            tuple(self.portal_path),
            self.first_portal_id,
            self.reason,
            tuple(
                item.key() if callable(getattr(item, "key", None)) else repr(item)
                for item in self.required_evidence
            ),
            tuple(
                item.key() if callable(getattr(item, "key", None)) else repr(item)
                for item in self.produces_evidence
            ),
            str(self.portal_probe_phase or ""),
        )

    def durable_obligation_key(self):
        """Return the topology identity that survives map re-projection.

        ``signature`` deliberately describes one immutable plan and therefore
        includes map/semantic details that may change on every snapshot.  A
        materialization miss has a different lifetime: it belongs to the
        durable WorkItem, PortalProbe, or Portal obligation.  Keep this key
        free of map epochs and planner reasons so a moving SLAM projection
        cannot reset the miss budget for the same topology obligation.
        """
        action = str(self.action or "")
        obligation_kind = str(self.obligation_kind or "")
        obligation_id = _positive_id(self.obligation_id)
        if obligation_id is not None:
            return ("obligation", obligation_kind or action, obligation_id)

        portal_id = _positive_id(self.first_portal_id)
        if portal_id is not None:
            return ("portal", portal_id)

        path = tuple(
            value
            for value in (_positive_id(item) for item in self.portal_path)
            if value is not None
        )
        current_place = _positive_id(self.current_place_id)
        target_place = _positive_id(self.target_place_id)
        return (
            "logical",
            action,
            obligation_kind,
            current_place,
            target_place,
            path,
        )


def _records(value):
    """Read a ledger snapshot while keeping the planner side-effect free."""
    if value is None:
        return ()
    snapshot = getattr(value, "snapshot", None)
    if callable(snapshot):
        value = snapshot()
    if isinstance(value, dict):
        value = value.values()
    try:
        return tuple(item for item in value if isinstance(item, dict))
    except TypeError:
        return ()


def _positive_id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _work_item_has_direction(item):
    """Whether a durable WorkItem contains a usable unknown-side normal.

    A normal is the minimum durable geometry needed to rehydrate a viewpoint
    after the frontier arc disappears.  Items without one remain unresolved
    long-term obligations, but cannot be treated as an executable local route
    unless the current snapshot explicitly projects their identity.
    """
    if not isinstance(item, dict):
        return False
    value = item.get("normal_xy")
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return False
    return x == x and y == y and (x * x + y * y) > 1e-12


def _covered(place):
    if not isinstance(place, dict):
        return False
    if str(place.get("state", "")).strip().lower() in _COVERED_STATES:
        return True
    try:
        return int(place.get("endpoint_observations", 0)) > 0
    except (TypeError, ValueError):
        return False


def _available_work(records, excluded_ids=(), visible_ids=None):
    excluded_ids = {
        item_id for item_id in (_positive_id(value) for value in excluded_ids)
        if item_id is not None
    }
    by_place = defaultdict(list)
    any_unresolved = defaultdict(list)
    for item in records:
        place_id = _positive_id(item.get("place_id"))
        item_id = _positive_id(item.get("id"))
        if place_id is None or item_id is None:
            continue
        if item_id in excluded_ids:
            continue
        if str(item.get("state", "")).strip().lower() != "unresolved":
            continue
        any_unresolved[place_id].append(item)
        # An active Attempt is already owned by another route.  It remains an
        # obligation, but must not be dispatched a second time.
        if item.get("active_attempt_id") is None:
            by_place[place_id].append(item)
    for values in by_place.values():
        values.sort(key=lambda item: _positive_id(item.get("id")) or 0)
    for values in any_unresolved.values():
        values.sort(key=lambda item: _positive_id(item.get("id")) or 0)
    return by_place, any_unresolved


def _branch_coverage_state(branch_coverage, probe):
    """Read coverage state for a probe bound to a durable Portal."""
    if branch_coverage is None or not isinstance(probe, dict):
        return None
    source_place_id = _positive_id(probe.get("source_place_id"))
    portal_id = _positive_id(probe.get("portal_id"))
    if source_place_id is None or portal_id is None:
        return None
    try:
        branch = branch_coverage.get(
            (source_place_id, "portal:%d" % portal_id)
        )
    except (TypeError, ValueError, AttributeError):
        return None
    if branch is None:
        return None
    return str(getattr(branch, "coverage_state", "") or "").strip().lower()


def _available_probes(records, branch_coverage=None):
    by_place = defaultdict(list)
    for probe in records:
        place_id = _positive_id(probe.get("source_place_id"))
        probe_id = _positive_id(probe.get("id"))
        if place_id is None or probe_id is None:
            continue
        state = str(probe.get("state", "")).strip().lower()
        if state in _AVAILABLE_PROBE_STATES and (
            state == "pending" or destination_reobserve_available(probe)
        ):
            # Destination-view evidence closes the directional branch even if
            # an older probe record remains source-arrived after replay.  The
            # edge can still be selected as transit; only another observation
            # attempt is suppressed here.
            if _branch_coverage_state(branch_coverage, probe) in {
                "completed", "transit",
            }:
                continue
            by_place[place_id].append(probe)
    for values in by_place.values():
        values.sort(key=lambda item: _positive_id(item.get("id")) or 0)
    return by_place


def _available_structural_boundary_probes(
    records, branch_coverage=None, excluded_ids=()
):
    """Return source probes compiled from known-free structural openings."""
    excluded = {
        probe_id
        for probe_id in (_positive_id(value) for value in excluded_ids)
        if probe_id is not None
    }
    by_place = defaultdict(list)
    for probe in records:
        if str(probe.get("observation_source", "")).strip().lower() != (
            "structural_boundary"
        ):
            continue
        place_id = _positive_id(probe.get("source_place_id"))
        probe_id = _positive_id(probe.get("id"))
        if place_id is None or probe_id is None:
            continue
        if probe_id in excluded:
            continue
        state = str(probe.get("state", "")).strip().lower()
        if state not in _AVAILABLE_PROBE_STATES:
            continue
        if _branch_coverage_state(branch_coverage, probe) in {
            "completed", "transit",
        }:
            continue
        by_place[place_id].append(probe)
    for values in by_place.values():
        values.sort(key=lambda item: _positive_id(item.get("id")) or 0)
    return by_place


def _bound_probe_for_portal(
    records, source_place_id, portal_id, branch_coverage=None,
    excluded_ids=(),
):
    """Resolve a Portal hypothesis to its executable source-side probe.

    A durable Portal hypothesis and a PortalProbe deliberately have different
    identity domains.  The former is a graph fact (``portal.id``); the latter
    is an information action (``probe.id``).  Once the probe ledger binds the
    two facts, a planner must use the probe identity for a source-side action,
    even when the current map did not rehydrate that probe into a visible
    frontier candidate.
    """
    source = _positive_id(source_place_id)
    portal = _positive_id(portal_id)
    if source is None or portal is None:
        return None
    excluded = {
        probe_id
        for probe_id in (_positive_id(value) for value in excluded_ids)
        if probe_id is not None
    }
    candidates = []
    for record in records:
        if _positive_id(record.get("source_place_id")) != source:
            continue
        probe_id = _positive_id(record.get("id"))
        if _positive_id(record.get("portal_id")) != portal:
            continue
        if probe_id in excluded:
            continue
        state = str(record.get("state", "")).strip().lower()
        if state not in _AVAILABLE_PROBE_STATES:
            continue
        if _branch_coverage_state(branch_coverage, record) in {
            "completed", "transit",
        }:
            continue
        candidates.append(record)
    if not candidates:
        return None
    # There should be one probe per physical doorway/direction.  Sorting keeps
    # recovery deterministic if an older ledger contains duplicate bindings.
    return min(candidates, key=lambda item: _positive_id(item.get("id")) or 0)


def _portal_probe_state(records, source_place_id, portal_id):
    """Return the bound probe state for one Portal, if it has one.

    A Portal hypothesis and its source-side probe are separate durable facts.
    The probe may be temporarily non-executable while its physical gate is
    waiting for a fresh map projection.  Exposing that state to the planner
    lets it distinguish a parked obligation from an unbound doorway and avoid
    turning the same missing projection into a new action every cycle.
    """
    source = _positive_id(source_place_id)
    portal = _positive_id(portal_id)
    if source is None or portal is None:
        return None
    matches = [
        record for record in records
        if _positive_id(record.get("source_place_id")) == source
        and _positive_id(record.get("portal_id")) == portal
    ]
    if not matches:
        return None
    record = min(matches, key=lambda item: _positive_id(item.get("id")) or 0)
    return str(record.get("state", "")).strip().lower()


def _unresolved_probe_places(records, branch_coverage=None, excluded_ids=()):
    """Keep active destination attempts visible to the completion decision."""
    excluded = {
        probe_id
        for probe_id in (_positive_id(value) for value in excluded_ids)
        if probe_id is not None
    }
    by_place = defaultdict(list)
    terminal_states = {"observed", "rejected"}
    for probe in records:
        place_id = _positive_id(probe.get("source_place_id"))
        probe_id = _positive_id(probe.get("id"))
        if place_id is None:
            continue
        if probe_id in excluded:
            continue
        if str(probe.get("state", "")).strip().lower() in terminal_states:
            continue
        if _branch_coverage_state(branch_coverage, probe) in {
            "completed", "transit",
        }:
            continue
        by_place[place_id].append(probe)
    return by_place


def _incoming_crossed_portal(
    current_place_id, portal_records, excluded_portal_ids=()
):
    """Return the durable edge that brought the robot into this Place.

    A covered Place can still own local frontier WorkItems after its entry
    observation.  When no outward Portal evidence is executable, consuming all
    of those transient viewpoints before returning to the graph corridor
    serializes exploration room by room.  The reverse action must retain the
    original crossed Portal identity so the adapter can materialize its durable
    egress instead of minting a new edge.
    """
    current = _positive_id(current_place_id)
    if current is None:
        return None
    excluded = {
        portal_id
        for portal_id in (
            _positive_id(value) for value in excluded_portal_ids
        )
        if portal_id is not None
    }
    candidates = []
    for portal in _records(portal_records):
        if not isinstance(portal, dict):
            continue
        portal_id = _positive_id(portal.get("id"))
        source = _positive_id(portal.get("source_place_id"))
        destination = _positive_id(portal.get("destination_place_id"))
        state = str(portal.get("state", "")).strip().lower()
        if (
            portal_id is None
            or source is None
            or destination != current
            or source == current
            or portal_id in excluded
            or state not in _PORTAL_TRANSIT_STATES
        ):
            continue
        try:
            crossed_at = float(portal.get("last_crossed_at"))
        except (TypeError, ValueError):
            crossed_at = float("-inf")
        candidates.append((-crossed_at, portal_id, source))
    if not candidates:
        return None
    _order, portal_id, source = min(candidates)
    return source, (portal_id,), "portal_edge", portal_id


class GraphRoutePlanner:
    """Plan over certified graph identities with deterministic BFS ordering.

    A crossed Portal is the only edge that may be used as a known transit
    connection.  An unbound certified Portal is retained as an information
    obligation, never as a fabricated destination Place.  The planner returns
    the complete topological path, while the runtime materializes only its
    first edge and replans after the corresponding physical crossing.
    """

    def plan(
        self,
        current_place_id,
        *,
        places,
        portals,
        work_items=(),
        probes=(),
        visible_work_item_ids=None,
        visible_probe_ids=None,
        visible_portal_ids=None,
        branch_first=False,
        target_place_id=None,
        target_observation_pending=False,
        resume_place_id=None,
        branch_coverage=None,
        structural_boundary_first=False,
        map_epoch=None,
        excluded_portal_ids=(),
        excluded_work_item_ids=(),
        excluded_probe_ids=(),
    ):
        current = _positive_id(current_place_id)
        resume_place = _positive_id(resume_place_id)
        excluded_portals = {
            value for value in (_positive_id(item) for item in excluded_portal_ids)
            if value is not None
        }
        excluded_work = {
            value for value in (_positive_id(item) for item in excluded_work_item_ids)
            if value is not None
        }
        excluded_probes = {
            value for value in (_positive_id(item) for item in excluded_probe_ids)
            if value is not None
        }
        place_records = _records(places)
        place_by_id = {
            place_id: place
            for place in place_records
            if (place_id := _positive_id(place.get("id"))) is not None
        }
        if current is None or current not in place_by_id:
            return GraphRoutePlan(
                PLAN_BLOCKED,
                ACTION_HOLD,
                current_place_id=current,
                reason="current_place_not_in_durable_graph",
            )

        probe_records = _records(probes)
        # A source-side probe and its unbound Portal are two representations
        # of one physical doorway.  Excluding only the probe would still let
        # the Portal branch reintroduce that doorway after invalidation.
        if excluded_probes:
            for probe in probe_records:
                probe_id = _positive_id(probe.get("id"))
                if probe_id not in excluded_probes:
                    continue
                portal_id = _positive_id(probe.get("portal_id"))
                if portal_id is not None:
                    excluded_portals.add(portal_id)
        probe_by_place = _available_probes(probe_records, branch_coverage)
        if excluded_probes:
            for place_id, values in tuple(probe_by_place.items()):
                probe_by_place[place_id] = [
                    item
                    for item in values
                    if _positive_id(item.get("id")) not in excluded_probes
                ]
        structural_boundary_by_place = _available_structural_boundary_probes(
            probe_records, branch_coverage, excluded_ids=excluded_probes,
        )
        unresolved_probe_places = _unresolved_probe_places(
            probe_records, branch_coverage, excluded_ids=excluded_probes,
        )
        active_probe_by_place = defaultdict(list)
        for probe in probe_records:
            place_id = _positive_id(probe.get("source_place_id"))
            probe_id = _positive_id(probe.get("id"))
            if place_id is None:
                continue
            if probe_id in excluded_probes:
                continue
            if str(probe.get("state", "")).strip().lower() in {
                "active",
                "destination_active",
            }:
                active_probe_by_place[place_id].append(probe)
        probe_work_ids = {
            probe.get("work_item_id")
            for probe in probe_records
            if probe.get("work_item_id") is not None
            and _positive_id(probe.get("id")) not in excluded_probes
            and str(probe.get("state", "")).strip().lower()
            in {"pending", "active", "source_arrived", "destination_active"}
        }
        work, unresolved_work = _available_work(
            _records(work_items), excluded_ids=probe_work_ids,
        )
        if excluded_work:
            for place_id, values in tuple(work.items()):
                work[place_id] = [
                    item
                    for item in values
                    if _positive_id(item.get("id")) not in excluded_work
                ]
        # A durable WorkItem may outlive the frontier arc that currently
        # projects it.  It remains an unresolved obligation, but it is not an
        # executable local action until the current snapshot rehydrates its
        # physical support.  Remote places stay in ``work`` so a certified
        # graph path can still route to them.
        if visible_work_item_ids is not None:
            visible = {
                item_id
                for item_id in (_positive_id(value) for value in visible_work_item_ids)
                if item_id is not None
            }
            visible_current_work = [
                item for item in work.get(current, ())
                if _positive_id(item.get("id")) in visible
            ]
            # An explicit set is the completed projection result, including
            # the empty set. Never resurrect a durable local item merely
            # because branch-first has no current candidate.
            work[current] = visible_current_work
        if visible_probe_ids is not None:
            # Probe visibility is a snapshot-local geometry fact. An explicit
            # set, including an empty set, is the completed projection result
            # for the current Place; remote probes remain durable obligations.
            visible = {
                probe_id
                for probe_id in (_positive_id(value) for value in visible_probe_ids)
                if probe_id is not None
            }
            probe_by_place[current] = [
                probe for probe in probe_by_place.get(current, ())
                if _positive_id(probe.get("id")) in visible
            ]
            # Apply the same snapshot boundary to structural probe records.
            # Keeping a rejected/hidden pending record in this map made an
            # otherwise executable bound-Portal egress wait forever even
            # though no current-map structural probe could be dispatched.
            structural_boundary_by_place[current] = [
                probe for probe in structural_boundary_by_place.get(current, ())
                if _positive_id(probe.get("id")) in visible
            ]
        adjacency = defaultdict(list)
        unbound_by_place = defaultdict(list)
        portal_records = _records(portals)
        for portal in portal_records:
            portal_id = _positive_id(portal.get("id"))
            source = _positive_id(portal.get("source_place_id"))
            destination = _positive_id(portal.get("destination_place_id"))
            state = str(portal.get("state", "")).strip().lower()
            if portal_id is None or source is None:
                continue
            if destination is None:
                if state in _UNBOUND_PORTAL_STATES and source in place_by_id:
                    if portal_id in excluded_portals:
                        continue
                    # A projection failure is negative evidence for this
                    # *map snapshot*, not for the physical doorway.  The
                    # portal ledger records the epoch so a changed SLAM
                    # projection can retry it, while repeated planning on
                    # the same snapshot moves on to another obligation.
                    rejected_epoch = portal.get("rejected_map_epoch")
                    try:
                        rejected_for_epoch = (
                            map_epoch is not None
                            and rejected_epoch is not None
                            and int(rejected_epoch) == int(map_epoch)
                        )
                    except (TypeError, ValueError):
                        rejected_for_epoch = (
                            map_epoch is not None
                            and str(rejected_epoch).strip()
                            == str(map_epoch).strip()
                        )
                    if rejected_for_epoch:
                        continue
                    # A source-side probe that cannot currently be projected
                    # is still an unresolved obligation, but it is not a
                    # legal action.  Do not fall back to the Portal ID here:
                    # that would replay the same physical doorway on every
                    # planning pass until SLAM happens to expose it again.
                    if _portal_probe_state(
                        probe_records, source, portal_id,
                    ) == "awaiting_projection":
                        continue
                    unbound_by_place[source].append(portal)
                continue
            if (
                state not in _PORTAL_TRANSIT_STATES
                or destination not in place_by_id
                or source == destination
            ):
                continue
            # Once crossed, a physical doorway is legal transit in either
            # direction.  Sorting makes the plan invariant to ledger order.
            adjacency[source].append((destination, portal_id))
            adjacency[destination].append((source, portal_id))
        for values in adjacency.values():
            values.sort(key=lambda edge: (edge[1], edge[0]))
        for values in unbound_by_place.values():
            values.sort(key=lambda item: _positive_id(item.get("id")) or 0)
        if visible_portal_ids is not None:
            visible = {
                portal_id
                for portal_id in (
                    _positive_id(value) for value in visible_portal_ids
                )
                if portal_id is not None
            }
            unbound_by_place[current] = [
                portal for portal in unbound_by_place.get(current, ())
                if (
                    _positive_id(portal.get("id")) in visible
                    # Once both sides of a physical doorway have been
                    # observed, its crossing obligation is durable graph
                    # evidence. Do not require the transient frontier arc to
                    # remain visible before selecting the same Portal edge.
                    or _portal_probe_state(
                        probe_records,
                        current,
                        portal.get("id"),
                    ) in {"observed", "certified"}
                )
            ]

        current_place = place_by_id[current]
        current_covered = _covered(current_place)
        local_work = work.get(current, ())
        local_probe = probe_by_place.get(current, ())
        local_unbound = unbound_by_place.get(current, ())

        # A newly entered Place must be observed before it can issue an
        # outward graph action.  This is a state invariant, not a score.
        if not current_covered:
            if local_work:
                return self._bootstrap(
                    current, local_work[0], "place_observation_required"
                )
            if local_probe:
                return self._probe(current, local_probe[0], "place_observation_required")
            if local_unbound:
                return self._unbound_portal(
                    current,
                    local_unbound[0],
                    "unbound_portal_requires_observation",
                )
            return GraphRoutePlan(
                PLAN_BLOCKED,
                ACTION_HOLD,
                current_place_id=current,
                reason="place_observation_required_before_graph_route",
            )

        # A stale active Attempt is an ownership fact, not permission to pick
        # another room. Wait for its terminal event so one WorkItem/probe is
        # never concurrently owned by two routes.
        if any(
            item.get("active_attempt_id") is not None
            for item in unresolved_work.get(current, ())
        ):
            return GraphRoutePlan(
                PLAN_BLOCKED,
                ACTION_HOLD,
                current_place_id=current,
                reason="local_work_attempt_active",
            )
        if active_probe_by_place.get(current):
            return GraphRoutePlan(
                PLAN_BLOCKED,
                ACTION_HOLD,
                current_place_id=current,
                reason="portal_probe_attempt_active",
            )

        # Target reinspection is a first-class information action. It owns
        # the current Place until its evidence transaction resolves; ordinary
        # local WorkItems and doorway probes are not equivalent substitutes.
        # Geometry still comes from the current map below, but the discrete
        # action class is fixed here so the graph cannot silently depart.
        if target_observation_pending:
            return self._target(current, "target_observation_required")

        # A branch-first Portal can turn out to be a second doorway into an
        # already-known Place.  Preserve the source Place as a one-shot graph
        # resume obligation so the destination's local work cannot steal the
        # next decision and create a covered-room loop.  The adapter clears
        # this marker when the source Place is reached; this planner remains
        # pure and only follows certified adjacency.
        if branch_first and resume_place is not None and resume_place != current:
            resume_pending = bool(
                work.get(resume_place)
                or probe_by_place.get(resume_place)
                or unbound_by_place.get(resume_place)
            )
            if resume_pending:
                resume_path = self._path_to_place(
                    current,
                    resume_place,
                    adjacency,
                    excluded_portal_ids=excluded_portals,
                )
                if resume_path is not None:
                    return self._cross(
                        current,
                        resume_path,
                        "resume_source_after_covered_portal_arrival",
                    )

        # A source-arrived Portal probe is an active evidence contract.  Its
        # destination view must be acquired before ordinary local WorkItems;
        # otherwise a fragmented frontier can keep the robot in the source
        # Place forever and the Place graph never gains a new edge.  This is a
        # lifecycle phase rule, not a numeric priority or a retry heuristic.
        available_probe_ids = {
            _positive_id(probe.get("id"))
            for probes_at_place in probe_by_place.values()
            for probe in probes_at_place
        }
        evidence_probe_records = tuple(
                probe for probe in probe_records
                if _branch_coverage_state(branch_coverage, probe)
                not in {"completed", "transit"}
                and _positive_id(probe.get("id")) not in excluded_probes
                and (
                # ``source_arrived`` is a durable phase boundary.  Its
                # destination-view obligation must survive disappearance of
                # the transient frontier arc; otherwise local WorkItems can
                # starve the second half of the same Portal transaction.
                str(probe.get("state", "")).strip().lower()
                == "source_arrived"
                or str(probe.get("state", "")).strip().lower()
                not in _AVAILABLE_PROBE_STATES
                or _positive_id(probe.get("id")) in available_probe_ids
            )
        )
        evidence_gap = next_evidence_gap(
            current,
            work_items=unresolved_work.get(current, ()),
            probes=evidence_probe_records,
        )
        if (
            evidence_gap is not None
            and evidence_gap.owner_kind == OWNER_PORTAL_PROBE
            and evidence_gap.kind == EVIDENCE_DESTINATION_VIEW
        ):
            probe = next(
                (
                    item for item in evidence_probe_records
                    if _positive_id(item.get("id")) == evidence_gap.owner_id
                ),
                None,
            )
            if probe is not None:
                return self._probe(current, probe, evidence_gap.reason)

        # A two-view probe has already supplied the source and destination
        # observations for an opening. Under branch-first policy the next
        # graph action is the physical crossing itself, before any other
        # structural probe from the source Place. This makes the graph edge
        # executable as soon as its evidence contract is complete.
        explicit_target = _positive_id(target_place_id)
        if branch_first and local_unbound:
            promoted = next(
                (
                    portal for portal in local_unbound
                    if _portal_probe_state(
                        probe_records,
                        current,
                        portal.get("id"),
                    ) in {"observed", "certified"}
                ),
                None,
            )
            if promoted is not None:
                portal_id = _positive_id(promoted.get("id"))
                return self._cross(
                    current,
                    (None, (portal_id,), "portal_edge", portal_id),
                    "destination_view_promoted_to_portal",
                )

        # A durable unbound Portal with a registered source-side probe is an
        # active branch obligation.  In branch-first mode, execute that probe
        # before another local WorkItem can starve the doorway.  The probe
        # identity is kept separate from the Portal identity so the source
        # observation still follows the normal two-phase contract.
        if branch_first and local_unbound:
            for portal in local_unbound:
                probe = _bound_probe_for_portal(
                    probe_records,
                    current,
                    portal.get("id"),
                    branch_coverage=branch_coverage,
                    excluded_ids=excluded_probes,
                )
                if probe is not None:
                    return self._probe(
                        current,
                        probe,
                        "unbound_portal_source_probe_priority",
                    )

        # A known-free structural opening is a first-class information
        # obligation in the full branch-first method.  It is deliberately
        # checked after target and destination-view obligations, but before
        # ordinary local WorkItems can hide the only route to a new Place.
        if structural_boundary_first:
            boundary_probe = structural_boundary_by_place.get(current, ())
            if boundary_probe:
                return self._probe(
                    current,
                    boundary_probe[0],
                    "structural_boundary_requires_observation",
                )

        # A covered Place reached through a crossed Portal is allowed to yield
        # back to the graph corridor once it has no executable outward Portal
        # evidence. Before taking the incoming edge, however, use any already
        # certified path to an unobserved Place. Taking the reverse egress
        # first makes a newly entered room immediately re-enter its source and
        # can starve a different unexplored branch behind that source.
        if (
            branch_first
            and not local_probe
            and not local_unbound
            and not structural_boundary_by_place.get(current)
            # A destination Place may have no currently projected local
            # WorkItems or Portal evidence while its durable obligation is
            # still unresolved.  Use the unfiltered ledger here: returning
            # through the incoming edge before re-materialization treats a
            # newly entered Place as transit-only and sends it back to its
            # source before its local work can be attempted.
            and not unresolved_work.get(current)
        ):
            unobserved_path = self._best_path(
                current,
                place_by_id,
                adjacency,
                work,
                probe_by_place,
                unresolved_work,
                unbound_by_place,
                only_unobserved=True,
                explicit_target=explicit_target,
                excluded_portal_ids=excluded_portals,
            )
            if unobserved_path is not None:
                return self._cross(
                    current,
                    unobserved_path,
                    "branch_first_from_covered_place_to_unobserved_place",
                )
            egress_path = _incoming_crossed_portal(
                current,
                portal_records,
                excluded_portal_ids=excluded_portals,
            )
            if egress_path is not None:
                return self._cross(
                    current,
                    egress_path,
                    "bound_portal_egress_required",
                )

        # Branch-first may suspend a local WorkItem once a certified edge leads
        # to an unobserved Place. The WorkItem remains durable and resumable;
        # requiring every rehydratable viewpoint to finish before crossing
        # serializes the whole building into room-sized detours and defeats the
        # graph policy's explicit branch-first contract.
        local_work_has_rehydratable_view = any(
            _work_item_has_direction(item)
            for item in unresolved_work.get(current, ())
        )
        local_work_projection_missing = bool(
            branch_first
            and visible_work_item_ids is not None
            and unresolved_work.get(current)
            and not local_work
        )
        if local_work_projection_missing:
            # An explicit empty visibility set is a completed snapshot result,
            # not an instruction to wait forever.  Preserve the local durable
            # WorkItems, but use a certified path to a genuinely unobserved
            # Place when one exists.  This prevents a transient projection gap
            # from pinning the robot in a covered room and still forbids a
            # return to another already observed Place.
            path = self._best_path(
                current,
                place_by_id,
                adjacency,
                work,
                probe_by_place,
                unresolved_work,
                unbound_by_place,
                only_unobserved=True,
                explicit_target=explicit_target,
                excluded_portal_ids=excluded_portals,
            )
            if path is not None:
                return self._cross(
                    current,
                    path,
                    "branch_first_after_local_projection_miss",
                )
        if (
            branch_first
            and local_work
            and not self._target_work_pending(
                explicit_target, current, local_work,
            )
        ):
            path = self._best_path(
                current,
                place_by_id,
                adjacency,
                work,
                probe_by_place,
                unresolved_work,
                unbound_by_place,
                only_unobserved=True,
                explicit_target=explicit_target,
                excluded_portal_ids=excluded_portals,
            )
            if path is not None:
                return self._cross(current, path, "branch_first_to_unobserved_place")

        if local_work:
            return self._local(current, local_work[0], "local_work_pending")

        # A source-side probe is itself an information obligation.  It cannot
        # be replaced by a generic frontier or silently cleared by a route
        # failure.
        if local_probe:
            return self._probe(current, local_probe[0], "portal_probe_pending")
        if local_unbound:
            # A Portal hypothesis may already own a source-side probe even
            # though SLAM has not projected that probe into this snapshot's
            # frontier set.  Convert the graph identity to the action identity
            # before returning a plan; passing the Portal ID downstream would
            # make the adapter search for a Portal transition and strand the
            # robot at the current Place.
            for portal in local_unbound:
                probe = _bound_probe_for_portal(
                    probe_records,
                    current,
                    portal.get("id"),
                    branch_coverage=branch_coverage,
                    excluded_ids=excluded_probes,
                )
                if probe is not None:
                    return self._probe(
                        current,
                        probe,
                        "unbound_portal_promoted_to_source_probe",
                    )
            return self._unbound_portal(
                current, local_unbound[0], "unbound_portal_pending"
            )

        # In the full graph-first method, a current-place obligation is a hard
        # barrier even when its transient endpoint is missing. Leaving through
        # a known Portal to satisfy a remote WorkItem would re-enter an already
        # observed Place and turn one projection gap into a room loop. The
        # rehydration selector gets the next map snapshot; until then, hold.
        if branch_first and (
            work.get(current)
            or local_work_has_rehydratable_view
            or unresolved_probe_places.get(current)
            or local_unbound
        ):
            return GraphRoutePlan(
                PLAN_BLOCKED,
                ACTION_HOLD,
                current_place_id=current,
                reason="current_place_obligation_not_materialized",
            )

        path = self._best_path(
            current,
            place_by_id,
            adjacency,
            work,
            probe_by_place,
            unresolved_work,
            unbound_by_place,
            only_unobserved=False,
            explicit_target=explicit_target,
            excluded_portal_ids=excluded_portals,
        )
        if path is not None:
            return self._cross(current, path, "transit_to_pending_graph_obligation")

        # An unresolved Attempt may be active or a durable obligation may be
        # disconnected from the known graph.  Neither condition is completion.
        if (
            any(unresolved_work.values())
            or any(unresolved_probe_places.values())
            or any(unbound_by_place.values())
        ):
            return GraphRoutePlan(
                PLAN_BLOCKED,
                ACTION_HOLD,
                current_place_id=current,
                reason="durable_obligation_has_no_certified_graph_path",
            )
        if any(not _covered(place) for place in place_by_id.values()):
            return GraphRoutePlan(
                PLAN_BLOCKED,
                ACTION_HOLD,
                current_place_id=current,
                reason="unobserved_place_has_no_certified_graph_path",
            )
        return GraphRoutePlan(
            PLAN_COMPLETE,
            ACTION_HOLD,
            current_place_id=current,
            reason="all_durable_graph_obligations_settled",
        )

    @staticmethod
    def _path_to_place(
        start, target, adjacency, excluded_portal_ids=()
    ):
        """Return a certified graph path to one explicitly resumed Place."""
        start = _positive_id(start)
        target = _positive_id(target)
        if start is None or target is None or start == target:
            return None
        excluded = {
            portal_id
            for portal_id in (
                _positive_id(value) for value in excluded_portal_ids
            )
            if portal_id is not None
        }
        queue = deque([start])
        paths = {start: ()}
        while queue:
            place_id = queue.popleft()
            for neighbour, portal_id in adjacency.get(place_id, ()):
                portal_id = _positive_id(portal_id)
                if portal_id in excluded or neighbour in paths:
                    continue
                paths[neighbour] = paths[place_id] + (portal_id,)
                if neighbour == target:
                    path = paths[neighbour]
                    return target, path, "portal_edge", path[-1]
                queue.append(neighbour)
        return None

    @staticmethod
    def _target_work_pending(explicit_target, current, local_work):
        """Keep a target-owned local obligation ahead of branch-first."""
        return explicit_target is not None and explicit_target == current and bool(local_work)

    @staticmethod
    def _bootstrap(place_id, item, reason):
        """Name the first observation as a phase transition, not local work."""
        item_id = _positive_id(item.get("id"))
        return GraphRoutePlan(
            PLAN_READY,
            ACTION_BOOTSTRAP,
            current_place_id=place_id,
            target_place_id=place_id,
            obligation_kind="work_item",
            obligation_id=item_id,
            reason=reason,
        )

    @staticmethod
    def _local(place_id, item, reason):
        item_id = _positive_id(item.get("id"))
        return GraphRoutePlan(
            PLAN_READY,
            ACTION_OBSERVE_LOCAL_WORK,
            current_place_id=place_id,
            target_place_id=place_id,
            obligation_kind="work_item",
            obligation_id=item_id,
            reason=reason,
        )

    @staticmethod
    def _target(place_id, reason):
        """Represent one unresolved target-place evidence transaction."""
        return GraphRoutePlan(
            PLAN_READY,
            ACTION_REINSPECT_TARGET,
            current_place_id=place_id,
            target_place_id=place_id,
            obligation_kind="target_observation",
            obligation_id=None,
            reason=reason,
        )

    @staticmethod
    def _probe(place_id, item, reason):
        probe_id = _positive_id(item.get("id"))
        state = str(item.get("state", "")).strip().lower()
        phase = "destination" if state in {
            "source_arrived", "destination_active",
        } else "source"
        return GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=place_id,
            # The source Place is already known; an unbound opening has no
            # destination identity yet and must never be represented as one.
            target_place_id=None,
            obligation_kind="portal_probe",
            obligation_id=probe_id,
            portal_path=(probe_id,) if probe_id is not None else (),
            first_portal_id=probe_id,
            reason=reason,
            portal_probe_phase=phase,
        )

    @staticmethod
    def _unbound_portal(place_id, portal, reason):
        portal_id = _positive_id(portal.get("id"))
        return GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=place_id,
            target_place_id=None,
            obligation_kind="unbound_portal",
            obligation_id=portal_id,
            portal_path=(portal_id,) if portal_id is not None else (),
            first_portal_id=portal_id,
            reason=reason,
        )

    @staticmethod
    def _cross(place_id, path, reason):
        target, portal_path, obligation_kind, obligation_id = path
        return GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=place_id,
            target_place_id=target,
            obligation_kind=obligation_kind,
            obligation_id=obligation_id,
            portal_path=tuple(portal_path),
            first_portal_id=portal_path[0] if portal_path else None,
            reason=reason,
        )

    @staticmethod
    def _best_path(
        start,
        place_by_id,
        adjacency,
        work,
        probes,
        unresolved_work,
        unbound,
        *,
        only_unobserved,
        explicit_target,
        excluded_portal_ids=(),
    ):
        """Find a shortest stable path to the next graph obligation."""
        queue = deque([start])
        paths = {start: ()}
        while queue:
            place_id = queue.popleft()
            for neighbour, portal_id in adjacency.get(place_id, ()):
                if neighbour in paths:
                    continue
                paths[neighbour] = paths[place_id] + (portal_id,)
                queue.append(neighbour)

        candidates = []
        for place_id, portal_path in paths.items():
            if place_id == start:
                continue
            if portal_path and portal_path[0] in excluded_portal_ids:
                continue
            place = place_by_id[place_id]
            is_unobserved = not _covered(place)
            if only_unobserved and not is_unobserved:
                continue
            obligation_kind = None
            obligation_id = None
            priority = 9
            if explicit_target is not None and place_id == explicit_target and (
                is_unobserved or work.get(place_id) or probes.get(place_id)
            ):
                priority = 0
                obligation_kind = "target_place"
            elif is_unobserved:
                priority = 1
                obligation_kind = "unobserved_place"
            elif work.get(place_id):
                priority = 2
                obligation_kind = "work_item"
                obligation_id = _positive_id(work[place_id][0].get("id"))
            elif probes.get(place_id):
                priority = 3
                obligation_kind = "portal_probe"
                obligation_id = _positive_id(probes[place_id][0].get("id"))
            elif unbound.get(place_id):
                priority = 4
                obligation_kind = "unbound_portal"
                obligation_id = _positive_id(unbound[place_id][0].get("id"))
            if obligation_kind is None:
                continue
            candidates.append(
                (
                    len(portal_path),
                    priority,
                    place_id,
                    tuple(portal_path),
                    obligation_kind,
                    obligation_id,
                )
            )
        if not candidates:
            return None
        # Semantic novelty is the primary graph objective.  Comparing path
        # length first makes a nearby covered Place's WorkItem beat a slightly
        # farther unobserved Place, which creates room re-entry and starves
        # topology expansion.  Keep shortest certified transit as the tie-break
        # within the same obligation class.
        _priority, _distance, target, portal_path, kind, obligation_id = min(
            (
                priority,
                distance,
                place_id,
                portal_path,
                obligation_kind,
                obligation_id,
            )
            for distance, priority, place_id, portal_path, obligation_kind,
            obligation_id in candidates
        )
        return target, portal_path, kind, obligation_id


__all__ = [
    "ACTION_BOOTSTRAP",
    "ACTION_CROSS_PORTAL",
    "ACTION_HOLD",
    "ACTION_OBSERVE_LOCAL_WORK",
    "ACTION_REINSPECT_TARGET",
    "ACTION_PROBE_PORTAL",
    "ACTION_RETRY_VIEWPOINT",
    "GraphRoutePlan",
    "GraphRoutePlanner",
    "PLAN_BLOCKED",
    "PLAN_COMPLETE",
    "PLAN_READY",
]
