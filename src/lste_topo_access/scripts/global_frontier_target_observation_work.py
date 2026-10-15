"""Mission-level observation obligations for a visually discovered target.

Detector frames are ephemeral.  A target-bearing frame therefore creates a
single task WorkItem owned by the durable physical Place, rather than granting
the current frame permanent navigation ownership.  The obligation remains
unresolved through detector gaps, SLAM relabelling and controller recovery; it
is settled only by an explicit task completion event or a new task epoch.

This is intentionally ROS-free and has no score, timeout or confidence
threshold.  It complements (rather than replaces) ``TargetBeliefLedger``:
the belief stores evidence, while this ledger stores the mission obligation.
"""

from collections import Counter


WORK_UNRESOLVED = "unresolved"
WORK_COMPLETED = "completed"
WORK_RELEASED = "released"


class TargetObservationWorkLedger:
    """Keep one durable target-observation obligation per physical Place."""

    def __init__(self, task_version=""):
        self._task_version = str(task_version or "")
        self._records = {}

    @property
    def task_version(self):
        return self._task_version

    def reset(self, task_version):
        """Start a new mission epoch without carrying target work across tasks."""
        self._task_version = str(task_version or "")
        self._records = {}

    @staticmethod
    def _place_id(value):
        try:
            value = int(value)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    def _record(self, place_id):
        place_id = self._place_id(place_id)
        if place_id is None:
            return None
        record = self._records.get(place_id)
        if record is None:
            record = {
                "task_version": self._task_version,
                "place_id": place_id,
                "state": WORK_UNRESOLVED,
                "target_hits": 0,
                "target_labels": Counter(),
                "track_ids": set(),
                "latest_track_id": "",
                "first_observed_at": None,
                "last_observed_at": None,
                "completed_at": None,
                "completion_reason": None,
            }
            self._records[place_id] = record
        return record

    def observe_target(
        self, task_version, place_id, *, labels=(), track_id="", now=0.0,
    ):
        """Create or refresh the unresolved obligation for one Place."""
        if str(task_version or "") != self._task_version:
            return None
        record = self._record(place_id)
        if record is None or record["state"] == WORK_COMPLETED:
            return None if record is None else self._view(record)
        if record["state"] == WORK_RELEASED:
            # A later detector identity is a new mission observation, not a
            # stale continuation of the released track.
            record["state"] = WORK_UNRESOLVED
            record["completed_at"] = None
            record["completion_reason"] = None
        stamp = float(now)
        record["target_hits"] += 1
        record["target_labels"].update(
            str(label or "").strip().lower()
            for label in labels
            if str(label or "").strip()
        )
        track_id = str(track_id or "").strip()
        if track_id:
            record["track_ids"].add(track_id)
            record["latest_track_id"] = track_id
        if record["first_observed_at"] is None:
            record["first_observed_at"] = stamp
        record["last_observed_at"] = stamp
        return self._view(record)

    def bind_track(self, task_version, place_id, track_id):
        """Attach Goal Manager's confirmed identity to existing Place work."""
        if str(task_version or "") != self._task_version:
            return None
        record = self._record(place_id)
        track_id = str(track_id or "").strip()
        if record is None or not track_id:
            return None
        record["track_ids"].add(track_id)
        record["latest_track_id"] = track_id
        return self._view(record)

    def complete(self, task_version, place_id=None, *, now=0.0, reason="task_done"):
        """Settle target work explicitly; no detector timeout settles it."""
        if str(task_version or "") != self._task_version:
            return []
        ids = (
            [self._place_id(place_id)]
            if place_id is not None
            else sorted(self._records)
        )
        completed = []
        for current_id in ids:
            record = self._records.get(current_id)
            if record is None or record["state"] == WORK_COMPLETED:
                continue
            record["state"] = WORK_COMPLETED
            record["completed_at"] = float(now)
            record["completion_reason"] = str(reason or "task_done")
            completed.append(self._view(record))
        return completed

    def release(
        self, task_version, place_id=None, *, track_id="", now=0.0,
        reason="target_lost",
    ):
        """Release an obligation only on an explicit target-loss decision."""
        if str(task_version or "") != self._task_version:
            return []
        ids = (
            [self._place_id(place_id)]
            if place_id is not None
            else sorted(self._records)
        )
        track_id = str(track_id or "").strip()
        released = []
        for current_id in ids:
            record = self._records.get(current_id)
            if record is None or record["state"] != WORK_UNRESOLVED:
                continue
            if track_id and (
                track_id not in record["track_ids"]
                or (
                    record.get("latest_track_id")
                    and track_id != record["latest_track_id"]
                )
            ):
                continue
            record["state"] = WORK_RELEASED
            record["completed_at"] = float(now)
            record["completion_reason"] = str(reason or "target_lost")
            released.append(self._view(record))
        return released

    def has_pending(self, place_id):
        """Return whether this Place still owns unresolved target work."""
        place_id = self._place_id(place_id)
        record = self._records.get(place_id)
        return bool(record is not None and record["state"] == WORK_UNRESOLVED)

    def pending_place_ids(self):
        return tuple(
            place_id
            for place_id in sorted(self._records)
            if self.has_pending(place_id)
        )

    def evidence(self, place_id):
        place_id = self._place_id(place_id)
        record = self._records.get(place_id)
        return None if record is None else self._view(record)

    @staticmethod
    def _view(record):
        return {
            "task_version": record["task_version"],
            "place_id": int(record["place_id"]),
            "state": record["state"],
            "target_hits": int(record["target_hits"]),
            "target_labels": dict(record["target_labels"]),
            "track_ids": sorted(record["track_ids"]),
            "latest_track_id": record.get("latest_track_id", ""),
            "first_observed_at": record["first_observed_at"],
            "last_observed_at": record["last_observed_at"],
            "completed_at": record["completed_at"],
            "completion_reason": record["completion_reason"],
        }

    def snapshot(self):
        return [self._view(self._records[place_id]) for place_id in sorted(self._records)]


__all__ = [
    "WORK_COMPLETED",
    "WORK_RELEASED",
    "WORK_UNRESOLVED",
    "TargetObservationWorkLedger",
]
