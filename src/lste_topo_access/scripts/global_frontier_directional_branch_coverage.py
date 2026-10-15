"""Persistent coverage state for directional exploration branches.

The online SLAM grid is a projection that may move, split, or disappear on
every update.  It is therefore not a suitable identity for an exploration
obligation.  This module keeps the identity of a physical directional branch
separate from its latest frontier projection.

The caller supplies ``physical_branch_id`` from a durable source (normally a
Portal or a portal-probe ledger).  Frontier coordinates are only refreshed
metadata.  A completed branch has no active ObservationWorkItem and may be
marked as transit after a verified crossing.  A normal map update or a normal
evidence fact cannot reopen it; reopening requires an explicitly contradictory
fact.  The model is intentionally ROS-free so it can be replayed and tested
without a running map or controller.
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Set, Tuple


BRANCH_OPEN = "open"
BRANCH_COMPLETED = "completed"
BRANCH_TRANSIT = "transit"

EVIDENCE_FRONTIER_SEEN = "frontier_seen"
EVIDENCE_BRANCH_COVERED = "branch_coverage_verified"
EVIDENCE_SOURCE_VIEW = "source_view"
EVIDENCE_DESTINATION_VIEW = "destination_view"
EVIDENCE_CROSSING_VERIFIED = "crossing_verified"


def _xy(value, name="xy"):
    """Normalize a finite two-dimensional point."""
    if value is None:
        return None
    try:
        point = (float(value[0]), float(value[1]))
    except (IndexError, TypeError, ValueError):
        raise ValueError("%s must contain two numeric values" % name)
    if not all(math.isfinite(value) for value in point):
        raise ValueError("%s must contain finite values" % name)
    return point


def _unit(value, name="direction_xy"):
    """Normalize a direction while rejecting a zero vector."""
    point = _xy(value, name)
    if point is None:
        return None
    length = math.hypot(point[0], point[1])
    if length <= 1e-9:
        raise ValueError("%s must not be zero" % name)
    return point[0] / length, point[1] / length


@dataclass(frozen=True)
class DirectionalBranchKey:
    """Stable identity of one physical branch leaving a Place.

    ``physical_branch_id`` is deliberately not derived from the current
    frontier coordinate.  It may be a durable Portal ID, a probe ID, or an
    upstream physical gate token.  The source Place is part of the identity so
    the same physical-looking direction in two Places remains distinct.
    """

    source_place_id: int
    physical_branch_id: str

    def __post_init__(self):
        try:
            place_id = int(self.source_place_id)
        except (TypeError, ValueError):
            raise ValueError("source_place_id must be a positive integer")
        if place_id <= 0:
            raise ValueError("source_place_id must be a positive integer")
        if self.physical_branch_id is None:
            raise ValueError("physical_branch_id must not be empty")
        branch_id = str(self.physical_branch_id).strip()
        if not branch_id:
            raise ValueError("physical_branch_id must not be empty")
        object.__setattr__(self, "source_place_id", place_id)
        object.__setattr__(self, "physical_branch_id", branch_id)

    @property
    def stable_id(self):
        """Return a deterministic key suitable for logs and WorkItem IDs."""
        return "%d:%s" % (self.source_place_id, self.physical_branch_id)

    def as_dict(self):
        return {
            "source_place_id": int(self.source_place_id),
            "physical_branch_id": self.physical_branch_id,
            "stable_id": self.stable_id,
        }


@dataclass(frozen=True)
class EvidenceFact:
    """One replayable fact about a directional branch.

    ``resolves`` closes the branch's observation obligation.  ``contradictory``
    invalidates an earlier closure.  They are mutually exclusive because a
    single event must not both prove and invalidate the same branch.
    """

    kind: str
    source: str = ""
    event_id: Optional[str] = None
    resolves: bool = False
    contradictory: bool = False

    def __post_init__(self):
        kind = str(self.kind).strip()
        if not kind:
            raise ValueError("evidence kind must not be empty")
        if self.resolves and self.contradictory:
            raise ValueError("evidence cannot both resolve and contradict")
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "source", str(self.source or ""))
        if self.event_id is not None:
            object.__setattr__(self, "event_id", str(self.event_id))

    def as_dict(self):
        return {
            "kind": self.kind,
            "source": self.source,
            "event_id": self.event_id,
            "resolves": bool(self.resolves),
            "contradictory": bool(self.contradictory),
        }


@dataclass(frozen=True)
class ObservationWorkItem:
    """Stable observation obligation for one branch.

    Reopening increments ``generation`` but keeps ``id`` unchanged.  This
    preserves lineage and prevents a contradictory observation from creating
    a second logical task for the same physical branch.
    """

    id: str
    branch: DirectionalBranchKey
    generation: int

    def as_dict(self):
        return {
            "id": self.id,
            "branch": self.branch.as_dict(),
            "generation": int(self.generation),
        }


@dataclass(frozen=True)
class DirectionalBranchSnapshot:
    """Read-only state returned by the ledger after every mutation."""

    key: DirectionalBranchKey
    state: str
    transit: bool
    destination_place_id: Optional[int]
    work_item: Optional[ObservationWorkItem]
    stable_work_item_id: str
    evidence: Tuple[EvidenceFact, ...]
    contradictions: Tuple[EvidenceFact, ...]
    frontier_xy: Optional[Tuple[float, float]]
    direction_xy: Optional[Tuple[float, float]]
    map_epoch: Optional[int]
    observation_count: int
    evidence_revision: int
    reopen_count: int
    updated_at: float

    @property
    def coverage_state(self):
        """Return open/completed state without the transit projection."""
        return BRANCH_COMPLETED if self.transit else self.state

    def allows_transit(self):
        return bool(self.transit)

    def as_dict(self):
        return {
            "key": self.key.as_dict(),
            "state": self.state,
            "coverage_state": self.coverage_state,
            "transit": bool(self.transit),
            "destination_place_id": self.destination_place_id,
            "work_item": (
                None if self.work_item is None else self.work_item.as_dict()
            ),
            # Keep the stable lineage visible even while no active WorkItem is
            # allowed for a completed/transit branch.
            "stable_work_item_id": self.stable_work_item_id,
            "evidence": [fact.as_dict() for fact in self.evidence],
            "contradictions": [
                fact.as_dict() for fact in self.contradictions
            ],
            "frontier_xy": self.frontier_xy,
            "direction_xy": self.direction_xy,
            "map_epoch": self.map_epoch,
            "observation_count": int(self.observation_count),
            "evidence_revision": int(self.evidence_revision),
            "reopen_count": int(self.reopen_count),
            "updated_at": float(self.updated_at),
        }


@dataclass
class _BranchRecord:
    key: DirectionalBranchKey
    stable_work_item_id: str
    state: str = BRANCH_OPEN
    transit: bool = False
    destination_place_id: Optional[int] = None
    generation: int = 0
    evidence: Dict[str, EvidenceFact] = None
    contradictions: Dict[str, EvidenceFact] = None
    applied_event_ids: Set[str] = None
    frontier_xy: Optional[Tuple[float, float]] = None
    direction_xy: Optional[Tuple[float, float]] = None
    map_epoch: Optional[int] = None
    observation_count: int = 0
    evidence_revision: int = 0
    reopen_count: int = 0
    updated_at: float = 0.0

    def __post_init__(self):
        if self.evidence is None:
            self.evidence = {}
        if self.contradictions is None:
            self.contradictions = {}
        if self.applied_event_ids is None:
            self.applied_event_ids = set()


class DirectionalBranchCoverage:
    """Ledger for persistent directional branch coverage and transit.

    The API intentionally has no distance, time, or confidence thresholds.
    Identity comes from ``DirectionalBranchKey`` and lifecycle changes come
    from explicit evidence facts.  A map coordinate can therefore change
    arbitrarily without creating a new branch or WorkItem.
    """

    def __init__(self):
        self._records = {}

    @staticmethod
    def _coerce_key(key):
        if isinstance(key, DirectionalBranchKey):
            return key
        if isinstance(key, (list, tuple)) and len(key) >= 2:
            return DirectionalBranchKey(key[0], key[1])
        if isinstance(key, dict):
            return DirectionalBranchKey(
                key.get("source_place_id"), key.get("physical_branch_id")
            )
        raise ValueError(
            "key must be DirectionalBranchKey, (source_place_id, branch_id), "
            "or a matching dict"
        )

    @staticmethod
    def _work_item_id(key):
        return "branch_work_item:%s" % key.stable_id

    def _record(self, key, now=0.0):
        key = self._coerce_key(key)
        record = self._records.get(key)
        if record is None:
            record = _BranchRecord(
                key=key,
                stable_work_item_id=self._work_item_id(key),
                updated_at=float(now),
            )
            self._records[key] = record
        return record

    @staticmethod
    def _fact(fact, *, source="", event_id=None, resolves=False,
              contradictory=False):
        if isinstance(fact, EvidenceFact):
            if any((source, event_id, resolves, contradictory)):
                raise ValueError(
                    "EvidenceFact cannot be combined with evidence options"
                )
            return fact
        return EvidenceFact(
            kind=fact,
            source=source,
            event_id=event_id,
            resolves=bool(resolves),
            contradictory=bool(contradictory),
        )

    @staticmethod
    def _epoch(value):
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            raise ValueError("map_epoch must be an integer")

    @staticmethod
    def _active_work_item(record):
        if record.state != BRANCH_OPEN or record.transit:
            return None
        return ObservationWorkItem(
            id=record.stable_work_item_id,
            branch=record.key,
            generation=int(record.generation),
        )

    def _snapshot(self, record):
        return DirectionalBranchSnapshot(
            key=record.key,
            state=(BRANCH_TRANSIT if record.transit else record.state),
            transit=bool(record.transit),
            destination_place_id=record.destination_place_id,
            work_item=self._active_work_item(record),
            stable_work_item_id=record.stable_work_item_id,
            evidence=tuple(
                record.evidence[name] for name in sorted(record.evidence)
            ),
            contradictions=tuple(
                record.contradictions[name]
                for name in sorted(record.contradictions)
            ),
            frontier_xy=record.frontier_xy,
            direction_xy=record.direction_xy,
            map_epoch=record.map_epoch,
            observation_count=int(record.observation_count),
            evidence_revision=int(record.evidence_revision),
            reopen_count=int(record.reopen_count),
            updated_at=float(record.updated_at),
        )

    def register(self, key, now=0.0):
        """Create or return a branch without inventing a map projection."""
        return self._snapshot(self._record(key, now=now))

    def observe_frontier(
        self,
        key,
        frontier_xy,
        *,
        direction_xy=None,
        map_epoch=None,
        now=0.0,
    ):
        """Refresh a transient frontier projection for a durable branch.

        This operation never changes completion or transit state.  In
        particular, a completed branch remains completed when the next SLAM
        update produces a different endpoint coordinate.
        """
        record = self._record(key, now=now)
        record.frontier_xy = _xy(frontier_xy, "frontier_xy")
        if direction_xy is not None:
            record.direction_xy = _unit(direction_xy)
        if map_epoch is not None:
            record.map_epoch = self._epoch(map_epoch)
        record.observation_count += 1
        record.updated_at = float(now)
        return self._snapshot(record)

    # Explicit alias for callers that use map-reconciliation terminology.
    refresh_projection = observe_frontier

    def get(self, key):
        """Return a read-only snapshot, or ``None`` for an unknown key."""
        try:
            key = self._coerce_key(key)
        except ValueError:
            return None
        record = self._records.get(key)
        return None if record is None else self._snapshot(record)

    def work_item_for(self, key):
        """Return the one active WorkItem, never a duplicate for a branch."""
        try:
            key = self._coerce_key(key)
        except ValueError:
            return None
        record = self._records.get(key)
        return None if record is None else self._active_work_item(record)

    def pending_work_items(self, source_place_id=None):
        """Return active WorkItems in deterministic branch order."""
        place_id = None
        if source_place_id is not None:
            try:
                place_id = int(source_place_id)
            except (TypeError, ValueError):
                return ()
        items = []
        for key in sorted(self._records, key=lambda item: item.stable_id):
            if place_id is not None and key.source_place_id != place_id:
                continue
            item = self._active_work_item(self._records[key])
            if item is not None:
                items.append(item)
        return tuple(items)

    def record_evidence(
        self,
        key,
        fact,
        *,
        source="",
        event_id=None,
        resolves=False,
        contradictory=False,
        now=0.0,
    ):
        """Apply one explicit fact to a branch.

        A regular fact can add information or resolve an open branch.  It
        cannot reopen a completed/transit branch.  Only ``contradictory=True``
        reopens such a branch, and reopening retains the original WorkItem ID
        while incrementing its generation.
        """
        record = self._record(key, now=now)
        fact = self._fact(
            fact,
            source=source,
            event_id=event_id,
            resolves=resolves,
            contradictory=contradictory,
        )
        if fact.event_id is not None and fact.event_id in record.applied_event_ids:
            return self._snapshot(record)
        if fact.event_id is not None:
            record.applied_event_ids.add(fact.event_id)

        record.evidence_revision += 1
        record.updated_at = float(now)
        if fact.contradictory:
            record.contradictions[fact.kind] = fact
            if record.state == BRANCH_COMPLETED or record.transit:
                record.state = BRANCH_OPEN
                record.transit = False
                record.destination_place_id = None
                record.generation += 1
                record.reopen_count += 1
            return self._snapshot(record)

        record.evidence[fact.kind] = fact
        if fact.resolves:
            record.state = BRANCH_COMPLETED
            # A newly completed branch is not transit until a crossing fact is
            # separately committed through mark_transit().  Do not demote an
            # already committed transit edge when a later positive fact is
            # replayed.
            if not record.transit:
                record.destination_place_id = None
        return self._snapshot(record)

    def complete_branch(
        self,
        key,
        *,
        evidence_kind=EVIDENCE_BRANCH_COVERED,
        source="",
        event_id=None,
        now=0.0,
    ):
        """Resolve branch observation from an explicit completion fact."""
        return self.record_evidence(
            key,
            evidence_kind,
            source=source,
            event_id=event_id,
            resolves=True,
            now=now,
        )

    def mark_transit(
        self,
        key,
        destination_place_id,
        *,
        source="",
        event_id=None,
        now=0.0,
    ):
        """Commit a verified crossing and expose the branch as transit.

        Crossing is itself a resolving physical fact.  It first closes the
        branch WorkItem (if needed), then marks the edge as graph transit.
        """
        try:
            destination_place_id = int(destination_place_id)
        except (TypeError, ValueError):
            raise ValueError("destination_place_id must be a positive integer")
        if destination_place_id <= 0:
            raise ValueError("destination_place_id must be a positive integer")
        record = self._record(key, now=now)
        # A replay of an old crossing event must not undo a later explicit
        # contradiction.  New crossings use a new event identity.
        if event_id is not None and str(event_id) in record.applied_event_ids:
            return self._snapshot(record)
        self.record_evidence(
            key,
            EVIDENCE_CROSSING_VERIFIED,
            source=source,
            event_id=event_id,
            resolves=True,
            now=now,
        )
        record = self._record(key, now=now)
        record.transit = True
        record.state = BRANCH_COMPLETED
        record.destination_place_id = destination_place_id
        record.updated_at = float(now)
        return self._snapshot(record)

    def allows_transit(self, key):
        snapshot = self.get(key)
        return snapshot is not None and snapshot.allows_transit()

    def snapshots(self, source_place_id=None):
        """Return all branch snapshots in deterministic order."""
        place_id = None
        if source_place_id is not None:
            try:
                place_id = int(source_place_id)
            except (TypeError, ValueError):
                return ()
        return tuple(
            self._snapshot(self._records[key])
            for key in sorted(self._records, key=lambda item: item.stable_id)
            if place_id is None or key.source_place_id == place_id
        )


__all__ = [
    "BRANCH_COMPLETED",
    "BRANCH_OPEN",
    "BRANCH_TRANSIT",
    "EVIDENCE_BRANCH_COVERED",
    "EVIDENCE_CROSSING_VERIFIED",
    "EVIDENCE_DESTINATION_VIEW",
    "EVIDENCE_FRONTIER_SEEN",
    "EVIDENCE_SOURCE_VIEW",
    "DirectionalBranchCoverage",
    "DirectionalBranchKey",
    "DirectionalBranchSnapshot",
    "EvidenceFact",
    "ObservationWorkItem",
]
