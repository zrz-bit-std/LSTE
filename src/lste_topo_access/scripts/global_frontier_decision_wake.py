"""Event-driven wake boundary for the slow exploration executive.

The map, costmap, SLAM transform and controller run at their own rates.  They
must not all be interpreted as permission to choose a new exploration goal.
This module turns durable facts into a small decision queue:

    fast execution -> observe fact -> one slow decision wake -> Navfn/TEB

It deliberately has no ROS dependency, timer, distance threshold, score, or
controller parameter.  A map fingerprint is used only to distinguish a new
geometric observation from a repeated publication of the same observation; it
is not a place or portal identity.
"""

from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import json


_STRUCTURAL_STATUS_EVENTS = frozenset(
    (
        "semantic_task_updated",
        "semantic_place_evidence",
        "target_belief_updated",
        "target_track_started",
        "target_segment_committed",
        "target_reinspection_requested",
        "target_loss_certified",
        "target_task_completed",
        "task_done",
        "route_command",
        "route_terminal",
        "route_invalidated",
        "execution_terminal_failure",
        "terminal_replan_requested",
        "frontier_endpoint_observed",
        "frontier_place_boundary_crossed",
        "frontier_place_closed",
        "frontier_place_departure_prepared",
        "frontier_place_transited",
        "frontier_place_suspended",
        "physical_place_bootstrapped",
        "portal_hypothesis_certified",
        "portal_hypothesis_selected",
        "portal_hypothesis_crossed",
        "portal_hypothesis_failed",
        "portal_hypothesis_destination_bound",
        "portal_transaction_started",
        "portal_transaction_finished",
        "portal_transaction_aborted",
        "portal_probe_started",
        "portal_probe_settled",
        "portal_probe_certified",
        "portal_probe_work_item_settled",
        "portal_probe_promoted",
        "portal_place_entered",
        "portal_place_covered_arrival",
        "frontier_action_selected",
        "graph_recovery_requested",
        "graph_exploration_blocked",
        "graph_route_materialization_miss",
        "graph_route_materialization_negative_evidence",
        "graph_route_materialization_invalidated",
        "graph_route_next_action",
        "frontier_route_unavailable",
        "frontier_exhausted",
        "replan_acknowledged",
        "navigation_readiness",
    )
)

_ROUTE_TERMINAL_EVENTS = frozenset(
    (
        "route_terminal",
        "route_invalidated",
        "execution_terminal_failure",
        "frontier_route_unavailable",
    )
)


@dataclass(frozen=True)
class DecisionWake:
    """One claim on the slow decision queue."""

    sequence: int
    reasons: tuple

    @property
    def reason(self):
        """Return the first reason for concise lifecycle logging."""
        return self.reasons[0] if self.reasons else "unknown"

    def as_dict(self):
        """Return a JSON-safe representation for status and experiment logs."""
        return {
            "sequence": int(self.sequence),
            "reasons": list(self.reasons),
            "reason": self.reason,
        }


def _positive_id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return 0
    return value if value > 0 else 0


def _message_fingerprint(message):
    """Hash map-like data while ignoring publication sequence and timestamps.

    ``OccupancyGrid`` messages are intentionally handled by attributes rather
    than importing ROS message classes.  This keeps the scheduler replayable
    in unit tests and in the offline benchmark reducer.
    """
    if message is None:
        return None
    data = getattr(message, "data", None)
    info = getattr(message, "info", None)
    if data is not None and info is not None:
        origin = getattr(info, "origin", None)
        position = getattr(origin, "position", None)
        geometry = (
            getattr(info, "width", None),
            getattr(info, "height", None),
            getattr(info, "resolution", None),
            getattr(position, "x", None),
            getattr(position, "y", None),
            getattr(position, "z", None),
        )
        digest = hashlib.blake2b(digest_size=16)
        digest.update(repr(geometry).encode("utf-8"))
        values = tuple(data)
        for start in range(0, len(values), 4096):
            # Occupancy grids use signed int8 values, while costmaps commonly
            # use unsigned values. Mapping both to one byte preserves values
            # without depending on the concrete ROS serialization type.
            digest.update(
                bytes((int(value) + 256) % 256 for value in values[start:start + 4096])
            )
        return digest.hexdigest()
    if isinstance(message, (dict, list, tuple, str, int, float, bool)):
        try:
            encoded = json.dumps(
                message, sort_keys=True, separators=(",", ":"),
            ).encode("utf-8")
        except (TypeError, ValueError):
            encoded = repr(message).encode("utf-8")
        return hashlib.blake2b(encoded, digest_size=16).hexdigest()
    return hashlib.blake2b(repr(message).encode("utf-8"), digest_size=16).hexdigest()


class DecisionWakeScheduler:
    """Coordinate fast execution and event-driven graph deliberation.

    The scheduler is intentionally conservative: a new fact can request a
    wake, but no fact can preempt a live route.  Terminal events release the
    route and expose the queued wake to the next planning cycle.  Reasons are
    de-duplicated while preserving their first-seen order, making event logs
    stable under latched ROS messages.
    """

    def __init__(self):
        self._pending = OrderedDict()
        self._seen_status_facts = OrderedDict()
        self._sequence = 0
        self._active_route_id = 0
        self._halted = False
        self._map_fingerprint = None
        self._costmap_fingerprint = None
        self._pose_available = False
        self._last_wake = None
        self._startup_requested = True
        self.request("startup")

    @property
    def active_route_id(self):
        return int(self._active_route_id)

    @property
    def halted(self):
        return bool(self._halted)

    @property
    def pending(self):
        return bool(self._pending)

    def request(self, reason):
        """Queue one reason unless the mission has explicitly halted."""
        reason = str(reason or "unknown").strip().lower() or "unknown"
        if self._halted:
            return False
        if reason in self._pending:
            return False
        self._pending[reason] = None
        return True

    @staticmethod
    def _status_fact_key(name, fields):
        """Build a durable event key without transient map coordinates."""
        explicit = fields.get("event_id")
        if explicit not in (None, ""):
            return (name, "event_id", str(explicit))
        values = (
            fields.get("route_id"),
            fields.get("place_id", fields.get("active_region_id")),
            fields.get("source_place_id"),
            fields.get("work_item_id", fields.get("active_work_item_id")),
            fields.get("portal_id", fields.get("portal_probe_id")),
            fields.get("attempt_id"),
            fields.get("transaction_id"),
            fields.get("request_id", fields.get("replan_request_id")),
            fields.get("state"),
            fields.get("phase"),
            fields.get("reason"),
            fields.get("status"),
            fields.get("status_text"),
            fields.get("ready"),
            fields.get("decision_epoch"),
            fields.get("candidate_id"),
            fields.get("evidence_revision"),
            fields.get("target_track_id"),
            fields.get("task_version"),
        )
        try:
            normalized = json.dumps(
                values, sort_keys=True, separators=(",", ":"), default=str,
            )
        except (TypeError, ValueError):
            normalized = repr(values)
        return (name, normalized)

    def _accept_status_fact(self, name, fields):
        """Accept one lifecycle fact once during this process lifetime."""
        key = self._status_fact_key(name, fields)
        if key in self._seen_status_facts:
            return False
        self._seen_status_facts[key] = None
        if len(self._seen_status_facts) > 4096:
            self._seen_status_facts.popitem(last=False)
        return True

    def observe_status(self, event, fields=None):
        """Translate one lifecycle event into a wake or route transition."""
        name = str(event or "").strip().lower()
        fields = fields if isinstance(fields, dict) else {}
        if name == "task_done":
            self.halt()
            return False
        if name in _STRUCTURAL_STATUS_EVENTS and not self._accept_status_fact(
            name, fields
        ):
            return False
        if name == "route_command":
            route_id = _positive_id(fields.get("route_id"))
            if route_id:
                self.start_route(route_id)
            return bool(route_id)
        if name in _ROUTE_TERMINAL_EVENTS:
            route_id = _positive_id(fields.get("route_id"))
            if route_id and self._active_route_id not in (0, route_id):
                # A late callback for an old route is diagnostic only. It must
                # not wake a new route and cause an unrelated replan.
                return False
            self.finish_route(route_id or self._active_route_id, name)
            return True
        if name in _STRUCTURAL_STATUS_EVENTS:
            return self.request(name)
        return False

    def observe_map(self, message=None, *, fingerprint=None):
        """Request a wake only when the map geometry actually changes."""
        value = fingerprint if fingerprint is not None else _message_fingerprint(message)
        if value is None or value == self._map_fingerprint:
            return False
        first = self._map_fingerprint is None
        self._map_fingerprint = value
        return self.request("map_available" if first else "map_changed")

    def observe_costmap(self, message=None, *, fingerprint=None):
        """Request a wake on the first or changed costmap observation."""
        value = fingerprint if fingerprint is not None else _message_fingerprint(message)
        if value is None or value == self._costmap_fingerprint:
            return False
        first = self._costmap_fingerprint is None
        self._costmap_fingerprint = value
        return self.request("costmap_available" if first else "costmap_changed")

    def observe_pose(self, available=True):
        """Wake once when the pose dependency becomes available again."""
        available = bool(available)
        changed = available and not self._pose_available
        self._pose_available = available
        return self.request("pose_available") if changed else False

    def start_route(self, route_id):
        """Commit the fast route and consume stale slow-layer reasons."""
        route_id = _positive_id(route_id)
        if not route_id or self._halted:
            return False
        self._active_route_id = route_id
        self._pending.clear()
        self._startup_requested = False
        return True

    def finish_route(self, route_id=0, reason="route_terminal"):
        """Release a route and queue exactly one successor decision."""
        route_id = _positive_id(route_id)
        if route_id and self._active_route_id not in (0, route_id):
            return False
        self._active_route_id = 0
        return self.request(reason)

    def halt(self):
        """Stop deliberation after a terminal mission fact such as task_done."""
        self._halted = True
        self._active_route_id = 0
        self._pending.clear()

    def resume(self):
        """Resume only when the outer mission explicitly resets task_done."""
        self._halted = False
        return self.request("mission_resumed")

    def claim(self, *, active_route_id=0):
        """Claim one wake if no route currently owns the actuator."""
        if self._halted or _positive_id(active_route_id) or self._active_route_id:
            return None
        if not self._pending:
            return None
        self._sequence += 1
        reasons = tuple(self._pending)
        self._pending.clear()
        self._last_wake = DecisionWake(self._sequence, reasons)
        return self._last_wake

    def report(self):
        """Return compact scheduler state for lifecycle logs and replay."""
        return {
            "pending": list(self._pending),
            "active_route_id": int(self._active_route_id),
            "halted": bool(self._halted),
            "decision_sequence": int(self._sequence),
            "last_wake": (
                None if self._last_wake is None else self._last_wake.as_dict()
            ),
        }


__all__ = [
    "DecisionWake",
    "DecisionWakeScheduler",
]
