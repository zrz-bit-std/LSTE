"""Pure lifecycle for a confirmed visual-target approach.

The previous implementation used ``target_cache_max_advances`` as an implicit
mission boundary.  That conflated a controller handoff optimization with the
semantic question of whether the target has been observed, completed, or
actually lost.  This transaction keeps those facts explicit and leaves route
geometry to the existing Navfn/TEB adapter.
"""

from dataclasses import dataclass

from goal_manager_target_viewpoint_attempt_ledger import (
    TargetViewpointAttemptLedger,
    VIEWPOINT_ARRIVED,
)


STATE_IDLE = "idle"
STATE_APPROACHING = "approaching"
STATE_REOBSERVING = "reobserving"
STATE_COMPLETED = "completed"
STATE_LOST = "lost"
STATE_FAILED = "failed"


@dataclass(frozen=True)
class TargetApproachSnapshot:
    state: str
    track_id: str
    segment_count: int
    last_evidence_at: object
    last_terminal_at: object
    last_reason: str
    target_epoch: int = 0
    viewpoint_ledger: object = None


class TargetApproachTransaction:
    """Keep one confirmed target track alive until an explicit terminal fact."""

    def __init__(self):
        self.viewpoint_ledger = TargetViewpointAttemptLedger()
        self.clear("initial")

    @property
    def active(self):
        return self.state in (STATE_APPROACHING, STATE_REOBSERVING)

    def clear(self, reason="cleared"):
        self.state = STATE_IDLE
        self.track_id = ""
        self.segment_count = 0
        self.last_evidence_at = None
        self.last_terminal_at = None
        self.last_reason = str(reason or "cleared")
        self.target_epoch = 0
        self.last_viewpoint_candidate_id = ""
        self.last_viewpoint_attempt_id = ""
        self.viewpoint_ledger.clear(reason)

    def begin(self, track_id, now, target_epoch=0):
        """Start a track once; a new identity replaces an old terminal one."""
        track_id = str(track_id or "").strip()
        if not track_id:
            return False
        if self.track_id == track_id and self.active:
            return False
        target_epoch = int(target_epoch or 0)
        # A preserved room obligation can outlive a lost approach transaction.
        # Re-confirming that same semantic track must create a new viewpoint
        # episode; otherwise the ledger's deliberately durable closed rungs
        # make every reinspection immediately exhaust the same portfolio.
        if self.track_id == track_id and self.target_epoch > 0:
            target_epoch = max(target_epoch, self.target_epoch + 1)
        self.state = STATE_APPROACHING
        self.track_id = track_id
        self.target_epoch = target_epoch
        self.segment_count = 0
        self.last_evidence_at = float(now)
        self.last_terminal_at = None
        self.last_reason = "track_confirmed"
        self.viewpoint_ledger.begin(track_id, self.target_epoch)
        if not self.viewpoint_ledger.get(self.last_viewpoint_candidate_id):
            self.last_viewpoint_candidate_id = ""
            self.last_viewpoint_attempt_id = ""
        return True

    def observe(self, track_id, now, close=False):
        """Refresh evidence without changing route ownership."""
        if str(track_id or "").strip() != self.track_id or not self.track_id:
            return False
        if self.state in (STATE_COMPLETED, STATE_LOST, STATE_FAILED):
            return False
        self.last_evidence_at = float(now)
        self.state = STATE_REOBSERVING if close else STATE_APPROACHING
        self.last_reason = "close_evidence" if close else "target_evidence"
        arrived = self.viewpoint_ledger.get(self.last_viewpoint_candidate_id)
        if arrived is not None and arrived.route_status == VIEWPOINT_ARRIVED:
            self.viewpoint_ledger.mark_observed(
                arrived.candidate_id, arrived.attempt_id, now
            )
        return True

    def segment_committed(
        self, track_id, now, candidate_id=None, attempt_id=None
    ):
        """Record one validated target viewpoint route."""
        if str(track_id or "").strip() != self.track_id or not self.track_id:
            return False
        if self.state in (STATE_COMPLETED, STATE_LOST, STATE_FAILED):
            return False
        self.segment_count += 1
        self.state = STATE_APPROACHING
        self.last_reason = "segment_committed"
        if candidate_id and attempt_id:
            self.last_viewpoint_candidate_id = str(candidate_id)
            self.last_viewpoint_attempt_id = str(attempt_id)
            candidate = self.viewpoint_ledger.get(candidate_id)
            if candidate is not None and candidate.attempt_id == attempt_id:
                self.last_reason = "viewpoint_attempt_committed"
        return True

    def segment_arrived(
        self, track_id, now, candidate_id=None, attempt_id=None
    ):
        """Enter an observation boundary after the controller reaches a view."""
        if str(track_id or "").strip() != self.track_id or not self.track_id:
            return False
        if self.state in (STATE_COMPLETED, STATE_LOST, STATE_FAILED):
            return False
        self.state = STATE_REOBSERVING
        self.last_terminal_at = float(now)
        self.last_reason = "segment_arrived_waiting_observation"
        if candidate_id or attempt_id:
            self.last_viewpoint_candidate_id = str(candidate_id or "")
            self.last_viewpoint_attempt_id = str(attempt_id or "")
            self.viewpoint_ledger.mark_arrived(
                candidate_id, attempt_id, now
            )
        return True

    def mark_viewpoint_failed(
        self,
        track_id,
        now,
        candidate_id=None,
        attempt_id=None,
        reason="controller_failed",
    ):
        """Consume one controller option without closing the target track."""
        if str(track_id or "").strip() != self.track_id or not self.track_id:
            return None
        if self.state in (STATE_COMPLETED, STATE_LOST, STATE_FAILED):
            return None
        candidate = self.viewpoint_ledger.mark_controller_failed(
            candidate_id, attempt_id, reason, now
        )
        if candidate is None:
            return None
        self.state = STATE_APPROACHING
        self.last_reason = "viewpoint_attempt_failed"
        return candidate

    def viewpoint_alternative_available(self, excluding=None):
        return self.viewpoint_ledger.has_open_alternative(excluding)

    def viewpoint_alternatives_exhausted(self):
        return self.viewpoint_ledger.alternatives_exhausted()

    def mark_completed(self, now, reason="task_done"):
        if not self.track_id:
            return False
        self.state = STATE_COMPLETED
        self.last_reason = str(reason or "task_done")
        active = self.viewpoint_ledger.get(self.last_viewpoint_candidate_id)
        if active is None:
            active = self.viewpoint_ledger.active_candidate
        if active is not None:
            self.viewpoint_ledger.mark_observed(
                active.candidate_id, active.attempt_id, now
            )
        self.viewpoint_ledger.state = "completed"
        return True

    def mark_lost(self, now, reason="target_evidence_expired"):
        if not self.track_id:
            return False
        self.state = STATE_LOST
        self.last_reason = str(reason or "target_evidence_expired")
        self.viewpoint_ledger.state = "lost"
        return True

    def mark_failed(self, now, reason="target_route_failed"):
        if not self.track_id:
            return False
        self.state = STATE_FAILED
        self.last_reason = str(reason or "target_route_failed")
        self.viewpoint_ledger.state = "failed"
        return True

    def snapshot(self):
        return TargetApproachSnapshot(
            state=self.state,
            track_id=self.track_id,
            segment_count=int(self.segment_count),
            last_evidence_at=self.last_evidence_at,
            last_terminal_at=self.last_terminal_at,
            last_reason=self.last_reason,
            target_epoch=int(self.target_epoch),
            viewpoint_ledger=self.viewpoint_ledger.snapshot().to_dict(),
        )


__all__ = [
    "STATE_APPROACHING",
    "STATE_COMPLETED",
    "STATE_FAILED",
    "STATE_IDLE",
    "STATE_LOST",
    "STATE_REOBSERVING",
    "TargetApproachSnapshot",
    "TargetApproachTransaction",
]
