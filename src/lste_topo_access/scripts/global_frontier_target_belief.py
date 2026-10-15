"""Persistent task evidence attached to the physical place graph.

The detector stream is short-lived and may lose an object while the robot is
moving.  This ledger keeps the task-specific fact separate from the generic
semantic label count: a target observation remains attached to the current
task, Place, and (when known) WorkItem or Portal.  It is intentionally a
qualitative state machine, so perception does not become another collection
of tunable scores and decay constants.
"""

from collections import Counter
import math


BELIEF_UNOBSERVED = "unobserved"
BELIEF_CONTEXT_SUPPORTED = "context_supported"
BELIEF_TARGET_SUPPORTED = "target_supported"
BELIEF_TARGET_CONFIRMED = "target_confirmed"
BELIEF_CONTRADICTED = "contradicted"


def _xy(value):
    if value is None:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return None
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


def _unit(value):
    point = _xy(value)
    if point is None:
        return None
    length = math.hypot(point[0], point[1])
    if not math.isfinite(length) or length <= 1e-9:
        return None
    return point[0] / length, point[1] / length


class TargetBeliefLedger:
    """Accumulate target, context, and negative evidence by physical owner."""

    def __init__(self, task_version=""):
        self._task_version = str(task_version or "")
        self._records = {}

    @property
    def task_version(self):
        return self._task_version

    def reset(self, task_version):
        """Start a task epoch without erasing the physical Place ledger."""
        self._task_version = str(task_version or "")
        self._records = {}

    @staticmethod
    def _place_id(value):
        try:
            value = int(value)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    @staticmethod
    def _optional_id(value):
        try:
            value = int(value)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    def _key(self, place_id, work_item_id, portal_id, track_id):
        place_id = self._place_id(place_id)
        if place_id is None:
            return None
        return (
            place_id,
            self._optional_id(work_item_id),
            self._optional_id(portal_id),
            str(track_id or "").strip(),
        )

    def _record(self, key):
        record = self._records.get(key)
        if record is None:
            record = {
                "task_version": self._task_version,
                "place_id": key[0],
                "work_item_id": key[1],
                "portal_id": key[2],
                "track_id": key[3] or None,
                "target_hits": 0,
                "context_hits": 0,
                "negative_hits": 0,
                "target_labels": Counter(),
                "context_labels": Counter(),
                "negative_labels": Counter(),
                "state": BELIEF_UNOBSERVED,
                "bearing_xy": None,
                "origin_xy": None,
                "last_observed_at": None,
            }
            self._records[key] = record
        return record

    @staticmethod
    def _state(record):
        if record["target_hits"] and record.get("confirmed"):
            return BELIEF_TARGET_CONFIRMED
        if record["target_hits"]:
            return BELIEF_TARGET_SUPPORTED
        if record["context_hits"]:
            return BELIEF_CONTEXT_SUPPORTED
        if record["negative_hits"]:
            return BELIEF_CONTRADICTED
        return BELIEF_UNOBSERVED

    def _observe(
        self, kind, task_version, place_id, labels=(), now=0.0,
        work_item_id=None, portal_id=None, track_id="", confirmed=False,
    ):
        if str(task_version or "") != self._task_version:
            return None
        key = self._key(place_id, work_item_id, portal_id, track_id)
        if key is None:
            return None
        record = self._record(key)
        labels = [str(label or "").strip().lower() for label in labels]
        labels = [label for label in labels if label]
        counter_name = "%s_labels" % kind
        hit_name = "%s_hits" % kind
        record[hit_name] += 1
        record[counter_name].update(labels)
        if confirmed:
            record["confirmed"] = True
        record["state"] = self._state(record)
        record["last_observed_at"] = float(now)
        return self._record_view(record)

    def observe_target(
        self, task_version, place_id, *, labels=(), now=0.0,
        work_item_id=None, portal_id=None, track_id="", confirmed=False,
    ):
        """Record one target-bearing detector or navigation-confirmation event."""
        return self._observe(
            "target", task_version, place_id, labels, now,
            work_item_id, portal_id, track_id, confirmed,
        )

    def observe_context(
        self, task_version, place_id, *, labels=(), now=0.0,
        work_item_id=None, portal_id=None, track_id="",
    ):
        return self._observe(
            "context", task_version, place_id, labels, now,
            work_item_id, portal_id, track_id,
        )

    def observe_negative(
        self, task_version, place_id, *, labels=(), now=0.0,
        work_item_id=None, portal_id=None, track_id="",
    ):
        return self._observe(
            "negative", task_version, place_id, labels, now,
            work_item_id, portal_id, track_id,
        )

    def bind_observation_geometry(
        self, task_version, place_id, *, bearing_xy=None, origin_xy=None,
        work_item_id=None, portal_id=None, track_id="", now=0.0,
        confirmed=False,
    ):
        """Attach a world bearing after Goal Manager commits a target segment."""
        if str(task_version or "") != self._task_version:
            return None
        key = self._key(place_id, work_item_id, portal_id, track_id)
        if key is None:
            return None
        record = self._record(key)
        bearing = _unit(bearing_xy)
        origin = _xy(origin_xy)
        if bearing is not None:
            record["bearing_xy"] = bearing
        if origin is not None:
            record["origin_xy"] = origin
        if confirmed:
            record["confirmed"] = True
            if record["target_hits"] == 0:
                record["target_hits"] = 1
        record["state"] = self._state(record)
        record["last_observed_at"] = float(now)
        return self._record_view(record)

    @staticmethod
    def _record_view(record):
        return {
            "task_version": record["task_version"],
            "place_id": int(record["place_id"]),
            "work_item_id": record["work_item_id"],
            "portal_id": record["portal_id"],
            "track_id": record["track_id"],
            "target_hits": int(record["target_hits"]),
            "context_hits": int(record["context_hits"]),
            "negative_hits": int(record["negative_hits"]),
            "target_labels": dict(record["target_labels"]),
            "context_labels": dict(record["context_labels"]),
            "negative_labels": dict(record["negative_labels"]),
            "state": record["state"],
            "bearing_xy": record["bearing_xy"],
            "origin_xy": record["origin_xy"],
            "last_observed_at": record["last_observed_at"],
        }

    def evidence(self, place_id):
        """Aggregate all task evidence for one Place, retaining no stale task."""
        place_id = self._place_id(place_id)
        if place_id is None:
            return None
        records = [
            record for key, record in self._records.items()
            if key[0] == place_id
        ]
        if not records:
            return None
        target_labels = Counter()
        context_labels = Counter()
        negative_labels = Counter()
        for record in records:
            target_labels.update(record["target_labels"])
            context_labels.update(record["context_labels"])
            negative_labels.update(record["negative_labels"])
        confirmed = any(record.get("confirmed") for record in records)
        target_hits = sum(record["target_hits"] for record in records)
        context_hits = sum(record["context_hits"] for record in records)
        negative_hits = sum(record["negative_hits"] for record in records)
        state = (
            BELIEF_TARGET_CONFIRMED if target_hits and confirmed
            else BELIEF_TARGET_SUPPORTED if target_hits
            else BELIEF_CONTEXT_SUPPORTED if context_hits
            else BELIEF_CONTRADICTED if negative_hits
            else BELIEF_UNOBSERVED
        )
        latest = max(
            records,
            key=lambda record: (
                -float("inf")
                if record["last_observed_at"] is None
                else float(record["last_observed_at"])
            ),
        )
        return {
            "task_version": self._task_version,
            "place_id": place_id,
            "records": len(records),
            "target_hits": target_hits,
            "context_hits": context_hits,
            "negative_hits": negative_hits,
            "target_labels": dict(target_labels),
            "context_labels": dict(context_labels),
            "negative_labels": dict(negative_labels),
            "state": state,
            "bearing_xy": latest["bearing_xy"],
            "origin_xy": latest["origin_xy"],
            "track_ids": sorted(
                record["track_id"] for record in records if record["track_id"]
            ),
            "last_observed_at": latest["last_observed_at"],
        }

    def record_for_track(self, place_id, track_id):
        place_id = self._place_id(place_id)
        track_id = str(track_id or "").strip()
        if place_id is None or not track_id:
            return None
        for key, record in self._records.items():
            if key[0] == place_id and key[3] == track_id:
                return self._record_view(record)
        return None

    def direction_matches(self, place_id, normal_xy):
        """Return whether a WorkItem points into a remembered target bearing."""
        evidence = self.evidence(place_id)
        target_bearing = None if evidence is None else evidence.get("bearing_xy")
        target_bearing = _unit(target_bearing)
        normal = _unit(normal_xy)
        if target_bearing is None or normal is None:
            return False
        # The sign is a topological half-plane relation, not an angle knob:
        # opposite directions are different information requests.
        return target_bearing[0] * normal[0] + target_bearing[1] * normal[1] > 0.0

    def snapshot(self):
        """Return stable JSON-safe records for experiment logs."""
        return [
            self._record_view(self._records[key])
            for key in sorted(self._records, key=lambda value: tuple(
                "" if item is None else str(item) for item in value
            ))
        ]
