"""Identity-bound navigation options for one visual target.

The target detector supplies an observation, while Navfn and TEB execute one
short viewpoint at a time.  Those are different facts.  This module keeps the
candidate-level lifecycle between them so a controller failure consumes only
the current option instead of terminating the semantic target transaction.

The ledger is deliberately ROS-free.  A goal or a plan may therefore be a ROS
message in production and a tuple or a small fake object in policy tests.
"""

from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Tuple


ROUTE_PENDING = "route_pending"
ROUTE_REJECTED = "route_rejected"
ROUTE_READY = "route_ready"
VIEWPOINT_ACTIVE = "active"
VIEWPOINT_ARRIVED = "arrived"
VIEWPOINT_OBSERVED = "observed"
VIEWPOINT_CONTROLLER_FAILED = "controller_failed"

_CLOSED_STATUSES = frozenset(
    (
        ROUTE_REJECTED,
        VIEWPOINT_ARRIVED,
        VIEWPOINT_OBSERVED,
        VIEWPOINT_CONTROLLER_FAILED,
    )
)
_OPEN_STATUSES = frozenset((ROUTE_PENDING, ROUTE_READY))


def _xy(value):
    """Return a JSON-friendly planar goal representation when possible."""
    if value is None:
        return None
    try:
        position = value.pose.position
        return [round(float(position.x), 4), round(float(position.y), 4)]
    except AttributeError:
        pass
    try:
        if len(value) >= 2:
            return [round(float(value[0]), 4), round(float(value[1]), 4)]
    except (TypeError, ValueError, IndexError):
        pass
    return str(value)


@dataclass
class TargetViewpointCandidate:
    """One semantic viewpoint option and its execution provenance."""

    candidate_id: str
    index: int
    target_track_id: str
    target_epoch: int = 0
    requested_goal: object = None
    validated_goal: object = None
    route_plan: object = None
    map_epoch: object = None
    route_status: str = ROUTE_PENDING
    attempt_id: str = ""
    attempt_count: int = 0
    failure_reason: str = ""
    last_event_at: object = None
    history: list = field(default_factory=list)

    @property
    def dispatchable(self):
        return self.route_status == ROUTE_READY

    def to_dict(self):
        return {
            "candidate_id": self.candidate_id,
            "index": int(self.index),
            "target_track_id": self.target_track_id,
            "target_epoch": int(self.target_epoch),
            "requested_goal": _xy(self.requested_goal),
            "validated_goal": _xy(self.validated_goal),
            "route_status": self.route_status,
            "map_epoch": self.map_epoch,
            "attempt_id": self.attempt_id,
            "attempt_count": int(self.attempt_count),
            "failure_reason": self.failure_reason,
            "last_event_at": self.last_event_at,
            "route_plan_size": (
                None
                if self.route_plan is None
                else len(self.route_plan)
                if hasattr(self.route_plan, "__len__")
                else None
            ),
            "history": list(self.history),
        }


@dataclass(frozen=True)
class TargetViewpointLedgerSnapshot:
    """Serializable view of target-owned viewpoint options."""

    track_id: str
    target_epoch: int
    generation: int
    state: str
    active_candidate_id: str
    active_attempt_id: str
    candidates: Tuple[dict, ...]

    def to_dict(self):
        return {
            "track_id": self.track_id,
            "target_epoch": int(self.target_epoch),
            "generation": int(self.generation),
            "state": self.state,
            "active_candidate_id": self.active_candidate_id,
            "active_attempt_id": self.active_attempt_id,
            "candidates": list(self.candidates),
        }


class TargetViewpointAttemptLedger:
    """Keep target identity stable while cycling through local viewpoints."""

    def __init__(self):
        self.clear("initial")

    def clear(self, reason="cleared"):
        self.track_id = ""
        self.target_epoch = 0
        self.generation = 0
        self.state = "idle"
        self.last_reason = str(reason or "cleared")
        self._attempt_sequence = 0
        self._candidates: Dict[str, TargetViewpointCandidate] = {}

    @property
    def active(self):
        return bool(self.track_id) and self.state == "active"

    @property
    def active_candidate(self) -> Optional[TargetViewpointCandidate]:
        for candidate in self._candidates.values():
            if candidate.route_status == VIEWPOINT_ACTIVE:
                return candidate
        return None

    def begin(self, track_id, target_epoch=0):
        """Open a target option set, preserving it across re-observation."""
        track_id = str(track_id or "").strip()
        if not track_id:
            return False
        target_epoch = int(target_epoch or 0)
        same_target = (
            self.track_id == track_id and self.target_epoch == target_epoch
        )
        if not same_target:
            self.clear("new_target")
            self.track_id = track_id
            self.target_epoch = target_epoch
            self.generation += 1
        self.state = "active"
        self.last_reason = "target_open"
        return not same_target

    def candidate_id(self, index: int) -> str:
        if not self.track_id:
            return ""
        return "%s::epoch:%d::viewpoint:%d" % (
            self.track_id,
            self.target_epoch,
            int(index),
        )

    def ensure_candidate(
        self,
        index: int,
        requested_goal=None,
        *,
        map_epoch=None,
    ) -> Optional[TargetViewpointCandidate]:
        """Register one logical ladder rung without resurrecting a failure."""
        if not self.active:
            return None
        index = int(index)
        candidate_id = self.candidate_id(index)
        candidate = self._candidates.get(candidate_id)
        if candidate is None:
            candidate = TargetViewpointCandidate(
                candidate_id=candidate_id,
                index=index,
                target_track_id=self.track_id,
                target_epoch=self.target_epoch,
            )
            self._candidates[candidate_id] = candidate
        if (
            candidate.route_status == ROUTE_REJECTED
            and map_epoch is not None
            and candidate.map_epoch != map_epoch
        ):
            candidate.route_status = ROUTE_PENDING
            candidate.failure_reason = ""
        if candidate.route_status in _CLOSED_STATUSES or (
            candidate.route_status == VIEWPOINT_ACTIVE
        ):
            return candidate
        if requested_goal is not None:
            candidate.requested_goal = requested_goal
        if map_epoch is not None:
            candidate.map_epoch = map_epoch
        return candidate

    def mark_route_rejected(
        self, index: int, reason="navfn_empty_plan", *, map_epoch=None, now=None
    ):
        candidate = self.ensure_candidate(index, map_epoch=map_epoch)
        if candidate is None:
            return None
        # A fresh map snapshot is a new route hypothesis, but a controller
        # failure remains a durable negative result for this physical option.
        if (
            candidate.route_status == ROUTE_REJECTED
            and map_epoch is not None
            and candidate.map_epoch != map_epoch
        ):
            candidate.route_status = ROUTE_PENDING
        if candidate.route_status in (
            VIEWPOINT_CONTROLLER_FAILED,
            VIEWPOINT_ARRIVED,
            VIEWPOINT_OBSERVED,
        ):
            return candidate
        candidate.route_status = ROUTE_REJECTED
        candidate.validated_goal = None
        candidate.route_plan = None
        candidate.map_epoch = map_epoch
        candidate.failure_reason = str(reason or "route_rejected")
        candidate.last_event_at = now
        candidate.history.append(("route_rejected", candidate.failure_reason))
        return candidate

    def mark_route_ready(
        self,
        index: int,
        requested_goal,
        validated_goal,
        route_plan=None,
        *,
        map_epoch=None,
        now=None,
    ):
        candidate = self.ensure_candidate(
            index, requested_goal, map_epoch=map_epoch
        )
        if candidate is None:
            return None
        if candidate.route_status in (
            VIEWPOINT_CONTROLLER_FAILED,
            VIEWPOINT_ARRIVED,
            VIEWPOINT_OBSERVED,
            VIEWPOINT_ACTIVE,
        ):
            return candidate
        candidate.requested_goal = requested_goal
        candidate.validated_goal = validated_goal
        candidate.route_plan = route_plan
        candidate.map_epoch = map_epoch
        candidate.route_status = ROUTE_READY
        candidate.failure_reason = ""
        candidate.last_event_at = now
        candidate.history.append(("route_ready", map_epoch))
        return candidate

    def begin_attempt(self, candidate_id, now=None):
        """Lease one route option to the controller exactly once."""
        candidate = self._candidates.get(str(candidate_id or ""))
        if candidate is None or not self.active:
            return None
        if candidate.route_status == VIEWPOINT_ACTIVE:
            return candidate
        if not candidate.dispatchable:
            return None
        current = self.active_candidate
        if current is not None and current.candidate_id != candidate.candidate_id:
            return None
        self._attempt_sequence += 1
        candidate.attempt_count += 1
        candidate.attempt_id = "%s::attempt:%d" % (
            candidate.candidate_id,
            self._attempt_sequence,
        )
        candidate.route_status = VIEWPOINT_ACTIVE
        candidate.last_event_at = now
        candidate.history.append(("attempt_started", candidate.attempt_id))
        self.last_reason = "viewpoint_attempt_started"
        return candidate

    def mark_arrived(self, candidate_id=None, attempt_id=None, now=None):
        candidate = self._matching_attempt(candidate_id, attempt_id)
        if candidate is None:
            return None
        if candidate.route_status == VIEWPOINT_ACTIVE:
            candidate.route_status = VIEWPOINT_ARRIVED
            candidate.last_event_at = now
            candidate.history.append(("arrived", candidate.attempt_id))
            self.last_reason = "viewpoint_arrived"
        return candidate

    def mark_observed(self, candidate_id=None, attempt_id=None, now=None):
        candidate = self._matching_attempt(candidate_id, attempt_id)
        if candidate is None:
            return None
        if candidate.route_status in (VIEWPOINT_ACTIVE, VIEWPOINT_ARRIVED):
            candidate.route_status = VIEWPOINT_OBSERVED
            candidate.last_event_at = now
            candidate.history.append(("observed", candidate.attempt_id))
            self.last_reason = "viewpoint_observed"
        return candidate

    def mark_controller_failed(
        self, candidate_id=None, attempt_id=None, reason="controller_failed", now=None
    ):
        """Close only the failed viewpoint and retain the target option set."""
        candidate = self._matching_attempt(candidate_id, attempt_id)
        if candidate is None:
            return None
        if candidate.route_status not in (VIEWPOINT_ACTIVE, VIEWPOINT_ARRIVED):
            return None
        candidate.route_status = VIEWPOINT_CONTROLLER_FAILED
        candidate.failure_reason = str(reason or "controller_failed")
        candidate.last_event_at = now
        candidate.history.append(("controller_failed", candidate.failure_reason))
        self.last_reason = "viewpoint_attempt_failed"
        return candidate

    def _matching_attempt(self, candidate_id=None, attempt_id=None):
        candidate_id = str(candidate_id or "").strip()
        attempt_id = str(attempt_id or "").strip()
        candidate = self._candidates.get(candidate_id) if candidate_id else None
        if candidate_id and candidate is None:
            return None
        if candidate is None and attempt_id:
            candidate = next(
                (
                    item
                    for item in self._candidates.values()
                    if item.attempt_id == attempt_id
                ),
                None,
            )
            if candidate is None:
                return None
        if candidate is None and not candidate_id and not attempt_id:
            candidate = self.active_candidate
        if candidate is None or candidate.target_track_id != self.track_id:
            return None
        if attempt_id and candidate.attempt_id != attempt_id:
            return None
        return candidate

    def get(self, candidate_id):
        return self._candidates.get(str(candidate_id or ""))

    def has_open_alternative(self, excluding=None):
        excluding = str(excluding or "")
        return any(
            candidate.candidate_id != excluding
            and candidate.route_status in _OPEN_STATUSES
            for candidate in self._candidates.values()
        )

    def alternatives_exhausted(self, candidate_ids: Optional[Iterable[str]] = None):
        """Return true only when all known options are closed.

        ``route_pending`` is intentionally open: it means the map/planner has
        not decided yet, not that the semantic target is impossible.
        """
        if candidate_ids is None:
            candidates = list(self._candidates.values())
        else:
            candidates = [
                self._candidates[item]
                for item in candidate_ids
                if item in self._candidates
            ]
        return bool(candidates) and all(
            candidate.route_status in _CLOSED_STATUSES for candidate in candidates
        )

    def snapshot(self):
        active = self.active_candidate
        return TargetViewpointLedgerSnapshot(
            track_id=self.track_id,
            target_epoch=int(self.target_epoch),
            generation=int(self.generation),
            state=self.state,
            active_candidate_id="" if active is None else active.candidate_id,
            active_attempt_id="" if active is None else active.attempt_id,
            candidates=tuple(
                candidate.to_dict()
                for candidate in sorted(
                    self._candidates.values(), key=lambda item: item.index
                )
            ),
        )


__all__ = [
    "ROUTE_PENDING",
    "ROUTE_REJECTED",
    "ROUTE_READY",
    "VIEWPOINT_ACTIVE",
    "VIEWPOINT_ARRIVED",
    "VIEWPOINT_OBSERVED",
    "VIEWPOINT_CONTROLLER_FAILED",
    "TargetViewpointCandidate",
    "TargetViewpointLedgerSnapshot",
    "TargetViewpointAttemptLedger",
]
