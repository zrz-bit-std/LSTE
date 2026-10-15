"""Small, replayable evidence contracts for graph-level exploration.

The geometric stack observes a changing map, while the exploration executive
owns durable Place, Portal, and WorkItem identities.  This module is the
boundary between those time scales.  It records which evidence is still
missing and which evidence an action is allowed to produce; it does not score
frontiers or control the robot.

The reducer is intentionally immutable.  Declaring requirements and applying
observations are set operations, so replaying the same event or delivering
events in a different order produces the same contract state.
"""

from dataclasses import dataclass, replace


REQUIREMENT_PENDING = "pending"
REQUIREMENT_RESOLVED = "resolved"
REQUIREMENT_BLOCKED = "blocked"

OWNER_PLACE = "place"
OWNER_PORTAL = "portal"
OWNER_PORTAL_PROBE = "portal_probe"
OWNER_WORK_ITEM = "work_item"
OWNER_TARGET = "target"

EVIDENCE_PLACE_VIEW = "place_view"
EVIDENCE_WORK_ITEM_OPEN = "work_item_open"
EVIDENCE_WORK_ITEM_VIEWPOINT = "work_item_viewpoint"
EVIDENCE_PORTAL_CERTIFIED = "portal_certified"
EVIDENCE_PORTAL_SOURCE_VIEW = "portal_source_view"
EVIDENCE_PORTAL_CROSSING = "portal_crossing"
EVIDENCE_PORTAL_DESTINATION_VIEW = "portal_destination_view"
EVIDENCE_TARGET_VERIFY = "target_verify"
EVIDENCE_TARGET_VIEWPOINT = "target_viewpoint"
EVIDENCE_TARGET_VERIFIED = "target_verified"

# Names used by the first Portal-probe slice.  Keep them as aliases of the
# graph vocabulary so a probe phase cannot create a second evidence ontology.
EVIDENCE_SOURCE_VIEW = EVIDENCE_PORTAL_SOURCE_VIEW
EVIDENCE_DESTINATION_VIEW = EVIDENCE_PORTAL_DESTINATION_VIEW
EVIDENCE_LOCAL_OBSERVATION = EVIDENCE_WORK_ITEM_VIEWPOINT

ACTION_BOOTSTRAP = "bootstrap_observation"
ACTION_OBSERVE_LOCAL_WORK = "observe_local_work"
ACTION_RETRY_VIEWPOINT = "retry_viewpoint"
ACTION_REINSPECT_TARGET = "reinspect_target"
ACTION_PROBE_PORTAL = "probe_portal"
ACTION_CROSS_PORTAL = "cross_portal"
ACTION_HOLD = "hold"

_KNOWN_EVENTS = frozenset(
    (
        "source_viewpoint_arrived",
        "portal_probe_certified",
        "portal_crossing_verified",
        "portal_hypothesis_crossed",
        "frontier_place_boundary_crossed",
        "portal_destination_view_observed",
        "destination_view_observed",
        "portal_hypothesis_certified",
        "portal_probe_promoted",
        "portal_place_entered",
        "portal_place_covered_arrival",
        "portal_place_entry_observed",
        "frontier_endpoint_observed",
        "physical_place_bootstrapped",
        "frontier_place_closed",
        "target_task_completed",
        "task_done",
    )
)


def _text(value):
    return str(value or "").strip()


def _owner_id(value):
    """Keep common numeric identities numeric and all other IDs stable."""
    if value is None or value == "":
        return ""
    if isinstance(value, bool):
        return str(value).lower()
    try:
        number = int(value)
    except (TypeError, ValueError):
        return _text(value)
    return number if number > 0 else _text(value)


def _owner_kind(value):
    return _text(value).lower().replace("-", "_")


def _requirement_key(owner_kind, owner_id, kind):
    return _owner_kind(owner_kind), _owner_id(owner_id), _text(kind).lower()


def _sort_key(value):
    parts = value.key()
    reason = getattr(value, "reason", "")
    return tuple(
        (type(part).__name__, repr(part)) for part in parts + (reason,)
    )


def _gap_sort_key(value):
    """Order lifecycle phases before stable owner identity."""
    priority = {
        EVIDENCE_PORTAL_CROSSING: 0,
        EVIDENCE_PORTAL_DESTINATION_VIEW: 1,
        EVIDENCE_PORTAL_SOURCE_VIEW: 2,
        EVIDENCE_WORK_ITEM_VIEWPOINT: 3,
        EVIDENCE_TARGET_VERIFY: 4,
        EVIDENCE_PLACE_VIEW: 5,
    }
    return priority.get(value.kind, 99), _sort_key(value)


@dataclass(frozen=True)
class EvidenceRequirement:
    """One durable evidence gap owned by a graph object."""

    owner_kind: str
    owner_id: object
    kind: str
    state: str = REQUIREMENT_PENDING
    reason: str = ""

    def __post_init__(self):
        object.__setattr__(self, "owner_kind", _owner_kind(self.owner_kind))
        object.__setattr__(self, "owner_id", _owner_id(self.owner_id))
        object.__setattr__(self, "kind", _text(self.kind).lower())
        object.__setattr__(self, "state", _text(self.state).lower() or REQUIREMENT_PENDING)
        object.__setattr__(self, "reason", _text(self.reason))

    def key(self):
        return _requirement_key(self.owner_kind, self.owner_id, self.kind)

    def as_dict(self):
        return {
            "owner_kind": self.owner_kind,
            "owner_id": self.owner_id,
            "kind": self.kind,
            "state": self.state,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class EvidenceFact:
    """One observed fact, retaining enough provenance for replay audits."""

    owner_kind: str
    owner_id: object
    kind: str
    event: str = ""
    route_id: object = None
    viewpoint_id: object = None

    def __post_init__(self):
        object.__setattr__(self, "owner_kind", _owner_kind(self.owner_kind))
        object.__setattr__(self, "owner_id", _owner_id(self.owner_id))
        object.__setattr__(self, "kind", _text(self.kind).lower())
        object.__setattr__(self, "event", _text(self.event).lower())
        object.__setattr__(self, "route_id", _owner_id(self.route_id) or None)
        object.__setattr__(self, "viewpoint_id", _owner_id(self.viewpoint_id) or None)

    def key(self):
        return (
            self.owner_kind,
            self.owner_id,
            self.kind,
            self.event,
            self.route_id,
            self.viewpoint_id,
        )

    def requirement_key(self):
        return _requirement_key(self.owner_kind, self.owner_id, self.kind)

    def as_dict(self):
        return {
            "owner_kind": self.owner_kind,
            "owner_id": self.owner_id,
            "kind": self.kind,
            "event": self.event,
            "route_id": self.route_id,
            "viewpoint_id": self.viewpoint_id,
        }


@dataclass(frozen=True)
class EvidenceObservation:
    """An event that may satisfy one evidence requirement."""

    event: str
    owner_kind: str
    owner_id: object
    evidence_kind: str = ""
    route_id: object = None
    viewpoint_id: object = None

    def __post_init__(self):
        object.__setattr__(self, "event", _text(self.event).lower())
        object.__setattr__(self, "owner_kind", _owner_kind(self.owner_kind))
        object.__setattr__(self, "owner_id", _owner_id(self.owner_id))
        object.__setattr__(self, "route_id", _owner_id(self.route_id) or None)
        object.__setattr__(self, "viewpoint_id", _owner_id(self.viewpoint_id) or None)
        object.__setattr__(self, "evidence_kind", _text(self.evidence_kind).lower())

    def fact(self):
        return EvidenceFact(
            self.owner_kind,
            self.owner_id,
            self.evidence_kind,
            event=self.event,
            route_id=self.route_id,
            viewpoint_id=self.viewpoint_id,
        )


@dataclass(frozen=True)
class EvidenceBlock:
    """A durable reason an obligation cannot currently be executed."""

    owner_kind: str
    owner_id: object
    kind: str
    reason: str

    def __post_init__(self):
        object.__setattr__(self, "owner_kind", _owner_kind(self.owner_kind))
        object.__setattr__(self, "owner_id", _owner_id(self.owner_id))
        object.__setattr__(self, "kind", _text(self.kind).lower())
        object.__setattr__(self, "reason", _text(self.reason))

    def key(self):
        return _requirement_key(self.owner_kind, self.owner_id, self.kind)

    def as_dict(self):
        return {
            "owner_kind": self.owner_kind,
            "owner_id": self.owner_id,
            "kind": self.kind,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class EvidenceAction:
    """An action contract selected from one evidence gap."""

    kind: str
    reason: str
    owner_kind: str = ""
    owner_id: object = ""
    requires: tuple = ()
    produces: tuple = ()

    @property
    def executable(self):
        return self.kind != ACTION_HOLD

    def as_dict(self):
        return {
            "kind": self.kind,
            "reason": self.reason,
            "owner_kind": self.owner_kind,
            "owner_id": self.owner_id,
            "requires": [item.as_dict() for item in self.requires],
            "produces": [item.as_dict() for item in self.produces],
        }


@dataclass(frozen=True)
class EvidenceContractSnapshot:
    """Canonical immutable state of outstanding and observed evidence."""

    requirements: tuple = ()
    resolved: tuple = ()
    blocked: tuple = ()
    revision: int = 0

    def pending(self):
        """Return pending gaps in stable evidence-first order."""
        return tuple(
            sorted(self.requirements, key=lambda item: _gap_sort_key(item))
        )

    @property
    def pending_requirements(self):
        """Property form for callers that treat the snapshot as a value."""
        return tuple(self.requirements)

    def signature(self):
        return (
            tuple(item.key() + (item.reason,) for item in self.requirements),
            tuple(item.key() for item in self.resolved),
            tuple(item.key() + (item.reason,) for item in self.blocked),
        )

    def is_satisfied(self, requirement):
        requirement = _coerce_requirement(requirement)
        if requirement is None:
            return False
        key = requirement.key()
        return any(item.requirement_key() == key for item in self.resolved)

    def declare(self, requirements):
        """Add gaps without reviving a fact already observed."""
        candidates = list(self.requirements)
        for value in requirements or ():
            requirement = _coerce_requirement(value)
            if requirement is None:
                continue
            if self.is_satisfied(requirement):
                continue
            if any(item.key() == requirement.key() for item in candidates):
                continue
            candidates.append(requirement)
        blocked_keys = {item.key() for item in self.blocked}
        canonical = tuple(
            sorted(
                (
                    item
                    for item in candidates
                    if item.key() not in blocked_keys
                ),
                key=lambda item: _sort_key(item),
            )
        )
        if canonical == self.requirements:
            return self
        return replace(self, requirements=canonical, revision=self.revision + 1)

    def observe(self, observation):
        """Apply one evidence fact idempotently and close its matching gap."""
        observation = _coerce_observation(observation)
        if observation is None or not observation.evidence_kind:
            return self
        fact = observation.fact()
        known_requirement = any(
            item.key() == fact.requirement_key() for item in self.requirements
        ) or any(
            item.requirement_key() == fact.requirement_key()
            for item in self.resolved
        )
        if not known_requirement and observation.event not in _KNOWN_EVENTS:
            return self
        facts = set(self.resolved)
        before = tuple(sorted(facts, key=lambda item: _sort_key(item)))
        facts.add(fact)
        after = tuple(sorted(facts, key=lambda item: _sort_key(item)))
        req_key = fact.requirement_key()
        requirements = tuple(
            item for item in self.requirements if item.key() != req_key
        )
        blocked = tuple(item for item in self.blocked if item.key() != req_key)
        if after == before and requirements == self.requirements and blocked == self.blocked:
            return self
        return replace(
            self,
            requirements=requirements,
            resolved=after,
            blocked=blocked,
            revision=self.revision + 1,
        )

    def block(self, owner_kind, owner_id, kind, reason):
        """Record an explicit execution block without inventing a retry knob."""
        block = EvidenceBlock(owner_kind, owner_id, kind, reason)
        if any(item.key() == block.key() and item.reason == block.reason for item in self.blocked):
            return self
        blocked = tuple(
            sorted(
                set(self.blocked).union((block,)),
                key=lambda item: _sort_key(item),
            )
        )
        requirements = tuple(item for item in self.requirements if item.key() != block.key())
        return replace(
            self,
            requirements=requirements,
            blocked=blocked,
            revision=self.revision + 1,
        )

    def as_dict(self):
        return {
            "revision": int(self.revision),
            "requirements": [item.as_dict() for item in self.requirements],
            "resolved": [item.as_dict() for item in self.resolved],
            "blocked": [item.as_dict() for item in self.blocked],
        }


def _coerce_requirement(value):
    if isinstance(value, EvidenceRequirement):
        return value
    if not isinstance(value, dict):
        return None
    return EvidenceRequirement(
        value.get("owner_kind", ""),
        value.get("owner_id"),
        value.get("kind", ""),
        value.get("state", REQUIREMENT_PENDING),
        value.get("reason", ""),
    )


def _coerce_observation(value):
    if isinstance(value, EvidenceObservation):
        return value
    if not isinstance(value, dict):
        return None
    return EvidenceObservation(
        value.get("event", ""),
        value.get("owner_kind", ""),
        value.get("owner_id"),
        evidence_kind=value.get("evidence_kind", value.get("kind", "")),
        route_id=value.get("route_id"),
        viewpoint_id=value.get("viewpoint_id"),
    )


def apply_observation(snapshot, observation):
    """Functional reducer wrapper used by ROS-free callers and tests."""
    if not isinstance(snapshot, EvidenceContractSnapshot):
        snapshot = EvidenceContractSnapshot()
    return snapshot.observe(observation)


def _record(value):
    if value is None:
        return None
    if isinstance(value, dict):
        return value
    as_dict = getattr(value, "as_dict", None)
    if callable(as_dict):
        result = as_dict()
        return result if isinstance(result, dict) else None
    return None


def _bool(value):
    if isinstance(value, bool):
        return value
    return _text(value).lower() in {"1", "true", "yes", "observed", "verified", "succeeded"}


def _id_from(record, *names):
    for name in names:
        value = record.get(name)
        if value not in (None, ""):
            return _owner_id(value)
    return ""


def _append_requirement(output, owner_kind, owner_id, kind, reason=""):
    if owner_id in (None, "") or not kind:
        return
    output.append(EvidenceRequirement(owner_kind, owner_id, kind, reason=reason))


def requirements_for(place=None, portal=None, work_item=None, target=None):
    """Derive evidence gaps from durable records, without using coordinates."""
    output = []

    place_record = _record(place)
    if place_record is not None:
        place_id = _id_from(place_record, "id", "place_id")
        state = _text(place_record.get("state")).lower()
        covered = (
            _bool(place_record.get("observed"))
            or _bool(place_record.get("covered"))
            or state in {"observed", "closed", "dormant", "ready_to_exit", "suspended", "transit"}
        )
        try:
            covered = covered or int(place_record.get("endpoint_observations", 0)) > 0
        except (TypeError, ValueError):
            pass
        if not covered:
            _append_requirement(output, OWNER_PLACE, place_id, EVIDENCE_PLACE_VIEW)

    work_record = _record(work_item)
    if work_record is not None:
        work_id = _id_from(work_record, "id", "work_item_id")
        if _text(work_record.get("state")).lower() in {"unresolved", "pending", "open"}:
            _append_requirement(output, OWNER_WORK_ITEM, work_id, EVIDENCE_WORK_ITEM_VIEWPOINT)

    portal_record = _record(portal)
    if portal_record is not None:
        portal_id = _id_from(portal_record, "id", "portal_id")
        state = _text(portal_record.get("state")).lower()
        if state not in {"failed", "rejected", "closed"}:
            phase = _text(portal_record.get("phase")).lower()
            source_view = _bool(portal_record.get("source_view_observed")) or phase in {
                "source_view", "source_view_observed", "crossing", "crossed", "destination_view"
            }
            crossing = _bool(portal_record.get("crossing_verified")) or phase in {
                "crossing", "crossed", "destination_view"
            }
            destination_view = _bool(portal_record.get("destination_view_observed")) or phase == "destination_view"
            if not source_view:
                _append_requirement(output, OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_SOURCE_VIEW)
            elif not crossing:
                _append_requirement(output, OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_CROSSING)
            elif portal_record.get("destination_place_id") not in (None, "") and not destination_view:
                _append_requirement(output, OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_DESTINATION_VIEW)

    target_record = _record(target)
    if target_record is not None:
        target_id = _id_from(target_record, "id", "target_id", "place_id", "task_version")
        state = _text(target_record.get("state")).lower()
        pending = target_record.get("pending")
        if pending is None:
            pending = state in {"unresolved", "pending", "approaching", "reobserving", "active"}
        if _bool(pending):
            _append_requirement(output, OWNER_TARGET, target_id, EVIDENCE_TARGET_VERIFY)

    unique = {item.key(): item for item in output}
    return tuple(sorted(unique.values(), key=lambda item: _sort_key(item)))


def _probe_requirements(probe):
    """Map a probe lifecycle state to its next evidence obligation."""
    record = _record(probe)
    if record is None:
        return ()
    probe_id = _id_from(record, "id", "probe_id", "portal_probe_id")
    state = _text(record.get("state")).lower()
    if state in {"failed", "rejected", "observed", "settled", "closed"}:
        return ()
    if state in {"source_arrived", "destination_active", "crossing"}:
        return (
            EvidenceRequirement(
                OWNER_PORTAL_PROBE,
                probe_id,
                EVIDENCE_PORTAL_DESTINATION_VIEW,
            ),
        )
    return (
        EvidenceRequirement(
            OWNER_PORTAL_PROBE,
            probe_id,
            EVIDENCE_PORTAL_SOURCE_VIEW,
        ),
    )


def build_contract(
    *, places=(), portals=(), work_items=(), probes=(), target=None,
):
    """Build a contract snapshot from durable ledger records.

    This adapter deliberately treats a source-side probe and its Portal
    hypothesis as separate owners.  A source arrival therefore asks for a
    destination view, while a pending probe only asks for a source view after
    local work has had a chance to run.
    """
    requirements = []
    for place in places or ():
        requirements.extend(requirements_for(place=place))
    for work_item in work_items or ():
        requirements.extend(requirements_for(work_item=work_item))
    for portal in portals or ():
        record = _record(portal)
        if record is None:
            continue
        portal_id = _id_from(record, "id", "portal_id")
        state = _text(record.get("state")).lower()
        if state in {"certified", "selected", "crossing"}:
            requirements.append(
                EvidenceRequirement(
                    OWNER_PORTAL,
                    portal_id,
                    EVIDENCE_PORTAL_CROSSING,
                )
            )
        elif state == "crossed" and record.get("destination_place_id") not in (None, ""):
            if not _bool(record.get("destination_view_observed")):
                requirements.append(
                    EvidenceRequirement(
                        OWNER_PORTAL,
                        portal_id,
                        EVIDENCE_PORTAL_DESTINATION_VIEW,
                    )
                )
    for probe in probes or ():
        requirements.extend(_probe_requirements(probe))
    if target is not None:
        requirements.extend(requirements_for(target=target))
    unique = {item.key(): item for item in requirements}
    return EvidenceContractSnapshot(
        requirements=tuple(
            sorted(unique.values(), key=lambda item: _sort_key(item))
        )
    )


def next_evidence_gap(
    current_place_id, *, work_items=(), probes=(), portals=(), target=None,
):
    """Return the next executable gap for one current Place.

    A source-arrived probe is an active Portal transaction and takes
    precedence.  A merely pending probe is parked behind local WorkItems so a
    newly discovered doorway cannot starve ordinary observation.
    """
    current = _owner_id(current_place_id)
    candidates = []
    for work_item in work_items or ():
        record = _record(work_item)
        if record is None or _id_from(record, "place_id") != current:
            continue
        if _text(record.get("state")).lower() not in {"unresolved", "pending", "open"}:
            continue
        candidates.append(
            (
                2,
                _id_from(record, "id", "work_item_id"),
                EvidenceRequirement(
                    OWNER_WORK_ITEM,
                    _id_from(record, "id", "work_item_id"),
                    EVIDENCE_LOCAL_OBSERVATION,
                ),
            )
        )
    for probe in probes or ():
        record = _record(probe)
        if record is None or _id_from(record, "source_place_id") != current:
            continue
        state = _text(record.get("state")).lower()
        probe_id = _id_from(record, "id", "probe_id", "portal_probe_id")
        if state in {"source_arrived", "destination_active", "crossing"}:
            priority = 0
            kind = EVIDENCE_DESTINATION_VIEW
        elif state in {"pending", "active", "awaiting_projection"}:
            priority = 3
            kind = EVIDENCE_SOURCE_VIEW
        else:
            continue
        candidates.append(
            (
                priority,
                probe_id,
                EvidenceRequirement(OWNER_PORTAL_PROBE, probe_id, kind),
            )
        )
    for portal in portals or ():
        record = _record(portal)
        if record is None or _id_from(record, "source_place_id") != current:
            continue
        state = _text(record.get("state")).lower()
        if state in {"certified", "selected", "crossing"}:
            portal_id = _id_from(record, "id", "portal_id")
            candidates.append(
                (
                    1,
                    portal_id,
                    EvidenceRequirement(OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_CROSSING),
                )
            )
    if target is not None:
        candidates.extend(
            (0, item.owner_id, item)
            for item in requirements_for(target=target)
            if item.kind == EVIDENCE_TARGET_VERIFY
        )
    if not candidates:
        return None
    return min(candidates, key=lambda item: (item[0], item[1], _sort_key(item[2])))[2]


def action_contract_for(
    action,
    *,
    current_place_id=None,
    obligation_kind="",
    obligation_id=None,
    target_place_id=None,
    first_portal_id=None,
    portal_probe_phase=None,
):
    """Describe one existing graph action's evidence boundary.

    ``portal_probe_phase`` is deliberately part of the contract rather than a
    numeric route hint.  A source-side view and a destination-side view are
    different facts for the same physical opening, so a route may not satisfy
    one by accidentally executing the other.
    """
    action = _text(action)
    place_id = _owner_id(current_place_id)
    portal_id = _owner_id(first_portal_id or (obligation_id if obligation_kind in {"portal_probe", "portal_edge", "unbound_portal"} else None))
    target_id = _owner_id(target_place_id or (obligation_id if obligation_kind == "target_observation" else None))
    work_id = _owner_id(obligation_id if obligation_kind == "work_item" else None)

    if action == ACTION_BOOTSTRAP:
        return EvidenceAction(
            action,
            "place_identity_available",
            OWNER_PLACE,
            place_id,
            (EvidenceRequirement(OWNER_PLACE, place_id, "place_identity"),),
            (EvidenceRequirement(OWNER_PLACE, place_id, EVIDENCE_PLACE_VIEW, REQUIREMENT_RESOLVED),),
        )
    if action in (ACTION_OBSERVE_LOCAL_WORK, ACTION_RETRY_VIEWPOINT):
        return EvidenceAction(
            action,
            "owned_work_item_viewpoint",
            OWNER_WORK_ITEM,
            work_id,
            (
                EvidenceRequirement(OWNER_PLACE, place_id, EVIDENCE_PLACE_VIEW),
                EvidenceRequirement(OWNER_WORK_ITEM, work_id, EVIDENCE_WORK_ITEM_OPEN),
            ),
            (EvidenceRequirement(OWNER_WORK_ITEM, work_id, EVIDENCE_WORK_ITEM_VIEWPOINT, REQUIREMENT_RESOLVED),),
        )
    if action == ACTION_REINSPECT_TARGET:
        return EvidenceAction(
            action,
            "target_evidence_gap",
            OWNER_TARGET,
            target_id,
            (EvidenceRequirement(OWNER_TARGET, target_id, EVIDENCE_TARGET_VERIFY),),
            (EvidenceRequirement(OWNER_TARGET, target_id, EVIDENCE_TARGET_VIEWPOINT, REQUIREMENT_RESOLVED),),
        )
    if action == ACTION_PROBE_PORTAL:
        if obligation_kind == "portal_probe":
            phase = _text(portal_probe_phase).lower() or "source"
            evidence_kind = (
                EVIDENCE_PORTAL_DESTINATION_VIEW
                if phase == "destination"
                else EVIDENCE_PORTAL_SOURCE_VIEW
            )
            return EvidenceAction(
                action,
                "%s_view_required" % phase,
                OWNER_PORTAL_PROBE,
                _owner_id(obligation_id),
                (
                    EvidenceRequirement(
                        OWNER_PLACE, place_id, EVIDENCE_PLACE_VIEW,
                    ),
                ),
                (
                    EvidenceRequirement(
                        OWNER_PORTAL_PROBE,
                        _owner_id(obligation_id),
                        evidence_kind,
                        REQUIREMENT_RESOLVED,
                    ),
                ),
            )
        return EvidenceAction(
            action,
            "portal_source_view_required",
            OWNER_PORTAL,
            portal_id,
            (
                EvidenceRequirement(OWNER_PLACE, place_id, EVIDENCE_PLACE_VIEW),
                EvidenceRequirement(OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_CERTIFIED),
            ),
            (EvidenceRequirement(OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_SOURCE_VIEW, REQUIREMENT_RESOLVED),),
        )
    if action == ACTION_CROSS_PORTAL:
        return EvidenceAction(
            action,
            "certified_portal_crossing",
            OWNER_PORTAL,
            portal_id,
            (
                EvidenceRequirement(OWNER_PLACE, place_id, EVIDENCE_PLACE_VIEW),
                EvidenceRequirement(OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_CERTIFIED),
            ),
            (EvidenceRequirement(OWNER_PORTAL, portal_id, EVIDENCE_PORTAL_CROSSING, REQUIREMENT_RESOLVED),),
        )
    return EvidenceAction(ACTION_HOLD, "no_evidence_action")


def action_for_gap(snapshot):
    """Select an action class from the first unresolved evidence category."""
    if not isinstance(snapshot, EvidenceContractSnapshot):
        return EvidenceAction(ACTION_HOLD, "invalid_evidence_contract")
    priority = {
        EVIDENCE_TARGET_VERIFY: 0,
        EVIDENCE_PLACE_VIEW: 1,
        EVIDENCE_WORK_ITEM_VIEWPOINT: 2,
        EVIDENCE_PORTAL_SOURCE_VIEW: 3,
        EVIDENCE_PORTAL_CROSSING: 4,
        EVIDENCE_PORTAL_DESTINATION_VIEW: 5,
    }
    pending = sorted(
        snapshot.pending(),
        key=lambda item: (priority.get(item.kind, 99), _sort_key(item)),
    )
    if not pending:
        return EvidenceAction(ACTION_HOLD, "no_evidence_gap")
    gap = pending[0]
    if gap.kind == EVIDENCE_TARGET_VERIFY:
        return action_contract_for(
            ACTION_REINSPECT_TARGET,
            target_place_id=gap.owner_id,
            obligation_kind="target_observation",
        )
    if gap.kind == EVIDENCE_PLACE_VIEW:
        return action_contract_for(ACTION_BOOTSTRAP, current_place_id=gap.owner_id)
    if gap.kind == EVIDENCE_WORK_ITEM_VIEWPOINT:
        return action_contract_for(
            ACTION_OBSERVE_LOCAL_WORK,
            current_place_id=None,
            obligation_kind="work_item",
            obligation_id=gap.owner_id,
        )
    if gap.kind == EVIDENCE_PORTAL_SOURCE_VIEW:
        return action_contract_for(
            ACTION_PROBE_PORTAL,
            obligation_kind="portal_probe",
            obligation_id=gap.owner_id,
            first_portal_id=gap.owner_id,
            portal_probe_phase="source",
        )
    if gap.kind == EVIDENCE_PORTAL_CROSSING:
        return action_contract_for(
            ACTION_CROSS_PORTAL,
            obligation_kind="portal_edge",
            obligation_id=gap.owner_id,
            first_portal_id=gap.owner_id,
        )
    if gap.kind == EVIDENCE_PORTAL_DESTINATION_VIEW:
        return action_contract_for(
            ACTION_PROBE_PORTAL,
            obligation_kind="portal_probe",
            obligation_id=gap.owner_id,
            first_portal_id=gap.owner_id,
            portal_probe_phase="destination",
        )
    return EvidenceAction(ACTION_HOLD, "unknown_evidence_gap")


_EVENT_EVIDENCE = {
    "physical_place_bootstrapped": (OWNER_PLACE, EVIDENCE_PLACE_VIEW),
    "frontier_place_closed": (OWNER_PLACE, EVIDENCE_PLACE_VIEW),
    "frontier_endpoint_observed": (OWNER_WORK_ITEM, EVIDENCE_WORK_ITEM_VIEWPOINT),
    # Probe and Portal IDs are separate namespaces. A probe certification
    # closes the probe's source-view requirement; the graph Portal is closed
    # by ``portal_hypothesis_certified`` below.
    "portal_probe_certified": (OWNER_PORTAL_PROBE, EVIDENCE_PORTAL_SOURCE_VIEW),
    "portal_crossing_verified": (OWNER_PORTAL, EVIDENCE_PORTAL_CROSSING),
    "portal_hypothesis_crossed": (OWNER_PORTAL, EVIDENCE_PORTAL_CROSSING),
    "frontier_place_boundary_crossed": (OWNER_PORTAL, EVIDENCE_PORTAL_CROSSING),
    "portal_hypothesis_certified": (OWNER_PORTAL, EVIDENCE_PORTAL_CERTIFIED),
    "portal_probe_promoted": (OWNER_PORTAL, EVIDENCE_PORTAL_CERTIFIED),
    "portal_place_entered": (OWNER_PORTAL, EVIDENCE_PORTAL_CROSSING),
    "portal_place_covered_arrival": (OWNER_PORTAL, EVIDENCE_PORTAL_CROSSING),
    "portal_place_entry_observed": (OWNER_PLACE, EVIDENCE_PLACE_VIEW),
    "portal_destination_view_observed": (OWNER_PORTAL_PROBE, EVIDENCE_PORTAL_DESTINATION_VIEW),
    "destination_view_observed": (OWNER_PORTAL_PROBE, EVIDENCE_PORTAL_DESTINATION_VIEW),
    "target_task_completed": (OWNER_TARGET, EVIDENCE_TARGET_VERIFIED),
    "task_done": (OWNER_TARGET, EVIDENCE_TARGET_VERIFIED),
}


def observation_for_event(event, fields=None):
    """Translate an existing lifecycle event into explicit evidence, if any."""
    fields = fields if isinstance(fields, dict) else {}
    name = _text(event).lower()
    configured = fields.get("evidence_kind")
    inferred = _EVENT_EVIDENCE.get(name)
    # ``portal_probe_settled`` carries a phase/result pair rather than one
    # fixed fact. Resolve it here so replay consumers never infer a Portal
    # crossing from an ordinary source-side attempt.
    if name == "portal_probe_settled":
        result = _text(fields.get("result") or fields.get("evidence")).lower()
        phase = _text(
            fields.get("phase") or fields.get("portal_probe_phase")
        ).lower()
        if result == "source_arrived" or phase == "source_arrived":
            inferred = (OWNER_PORTAL_PROBE, EVIDENCE_PORTAL_SOURCE_VIEW)
        elif result == "observed" and phase == "destination":
            inferred = (OWNER_PORTAL_PROBE, EVIDENCE_PORTAL_DESTINATION_VIEW)
        elif result == "certified":
            inferred = (OWNER_PORTAL_PROBE, EVIDENCE_PORTAL_SOURCE_VIEW)
    if configured:
        owner_kind = _owner_kind(fields.get("evidence_owner_kind") or (inferred[0] if inferred else ""))
        kind = _text(configured).lower()
    elif inferred is not None:
        owner_kind, kind = inferred
    else:
        return None

    context = fields.get("goal_context")
    context = context if isinstance(context, dict) else {}
    if owner_kind == OWNER_PLACE:
        owner_id = fields.get("place_id", fields.get("active_region_id", context.get("owner_place_id")))
    elif owner_kind == OWNER_WORK_ITEM:
        owner_id = fields.get("work_item_id", fields.get("active_work_item_id", context.get("work_item_id")))
    elif owner_kind == OWNER_PORTAL:
        owner_id = fields.get("portal_id", fields.get("portal_probe_id", fields.get("active_portal_probe_id")))
    elif owner_kind == OWNER_PORTAL_PROBE:
        owner_id = fields.get(
            "portal_probe_id",
            fields.get("active_portal_probe_id", fields.get("probe_id")),
        )
    else:
        owner_id = fields.get("target_id", fields.get("target_place_id", fields.get("task_version")))
    if owner_id in (None, ""):
        return None
    return EvidenceObservation(
        name,
        owner_kind,
        owner_id,
        route_id=fields.get("route_id"),
        viewpoint_id=fields.get("viewpoint_id", fields.get("active_work_item_attempt_id")),
        evidence_kind=kind,
    )


__all__ = [
    "ACTION_BOOTSTRAP",
    "ACTION_CROSS_PORTAL",
    "ACTION_HOLD",
    "ACTION_OBSERVE_LOCAL_WORK",
    "ACTION_PROBE_PORTAL",
    "ACTION_REINSPECT_TARGET",
    "ACTION_RETRY_VIEWPOINT",
    "EVIDENCE_PORTAL_CERTIFIED",
    "EVIDENCE_PORTAL_CROSSING",
    "EVIDENCE_PORTAL_DESTINATION_VIEW",
    "EVIDENCE_PORTAL_SOURCE_VIEW",
    "EVIDENCE_DESTINATION_VIEW",
    "EVIDENCE_LOCAL_OBSERVATION",
    "EVIDENCE_SOURCE_VIEW",
    "EVIDENCE_PLACE_VIEW",
    "EVIDENCE_TARGET_VERIFY",
    "EVIDENCE_TARGET_VERIFIED",
    "EVIDENCE_TARGET_VIEWPOINT",
    "EVIDENCE_WORK_ITEM_VIEWPOINT",
    "EvidenceAction",
    "EvidenceBlock",
    "EvidenceContractSnapshot",
    "EvidenceFact",
    "EvidenceObservation",
    "EvidenceRequirement",
    "OWNER_PLACE",
    "OWNER_PORTAL",
    "OWNER_PORTAL_PROBE",
    "OWNER_PORTAL_PROBE",
    "OWNER_TARGET",
    "OWNER_WORK_ITEM",
    "REQUIREMENT_BLOCKED",
    "REQUIREMENT_PENDING",
    "REQUIREMENT_RESOLVED",
    "action_contract_for",
    "action_for_gap",
    "apply_observation",
    "build_contract",
    "next_evidence_gap",
    "observation_for_event",
    "requirements_for",
]
