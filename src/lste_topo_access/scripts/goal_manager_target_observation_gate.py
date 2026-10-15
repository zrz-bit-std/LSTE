"""Event-driven ownership gate for a confirmed visual target.

The detector is an asynchronous source of observations, not the owner of a
Place.  A missing target box therefore records a gap but cannot release a
target observation WorkItem by itself.  Release becomes possible only after a
target approach reaches an observation boundary and the detector supplies a
small, consecutive negative observation episode at that boundary.

This module deliberately has no ROS, wall-clock timeout, score threshold, or
map dependency.  The Goal Manager may still use a timer for scheduling and for
the controller's zero-velocity hold, but semantic target loss is decided by
this event state machine.
"""

from dataclasses import dataclass


STATE_IDLE = "idle"
STATE_TRACKING = "tracking"
STATE_REOBSERVING = "reobserving"
STATE_LOSS_CERTIFIED = "loss_certified"

# A post-terminal negative episode is a structural observation contract.  Two
# empty frames are the existing post-arrival barrier; the third frame prevents
# one detector miss from releasing a Place obligation.  This is intentionally
# fixed here rather than exposed as another runtime tuning parameter.
MIN_NEGATIVE_REOBSERVATIONS = 3


@dataclass(frozen=True)
class TargetObservationGateSnapshot:
    """Immutable evidence state suitable for diagnostics and replay."""

    state: str
    track_id: str
    detector_epoch: int
    reobserve_epoch: int
    positive_observations: int
    negative_observations: int
    negative_streak: int
    negative_viewpoint_count: int
    last_observed_at: object
    last_reason: str


class TargetObservationGate:
    """Separate detector freshness from semantic target ownership."""

    def __init__(self, min_negative_observations=MIN_NEGATIVE_REOBSERVATIONS):
        # The default is an architecture constant.  The optional argument is
        # useful for deterministic policy tests and offline research variants,
        # but the live Goal Manager does not load it from ROS parameters.
        self.min_negative_observations = max(1, int(min_negative_observations))
        self.clear("initial")

    @property
    def active(self):
        return self.state in (STATE_TRACKING, STATE_REOBSERVING)

    def clear(self, reason="cleared"):
        self.state = STATE_IDLE
        self.track_id = ""
        self.detector_epoch = 0
        self.reobserve_epoch = 0
        self.positive_observations = 0
        self.negative_observations = 0
        self.negative_streak = 0
        self.negative_viewpoints = set()
        self.last_observed_at = None
        self.last_reason = str(reason or "cleared")

    def confirm(self, track_id, detector_epoch, now):
        """Give one geometrically confirmed track Place ownership."""
        track_id = str(track_id or "").strip()
        if not track_id:
            return False
        if self.track_id == track_id and self.active:
            return False
        self.state = STATE_TRACKING
        self.track_id = track_id
        self.detector_epoch = int(detector_epoch)
        self.reobserve_epoch = int(detector_epoch)
        self.positive_observations = 0
        self.negative_observations = 0
        self.negative_streak = 0
        self.last_observed_at = float(now)
        self.last_reason = "track_confirmed"
        return True

    def begin_reobserve(self, track_id, detector_epoch, now):
        """Open a negative-evidence window after a reached target segment."""
        track_id = str(track_id or "").strip()
        if (
            not track_id
            or track_id != self.track_id
            or self.state in (STATE_IDLE, STATE_LOSS_CERTIFIED)
        ):
            return False
        # If the preceding observation was already negative, a deliberate
        # reinspection is part of the same evidence episode. Preserve its
        # negative viewpoints across the new route terminal so one missed view
        # cannot be promoted to target loss merely because the robot moved to
        # the next viewpoint. A positive frame returns the gate to TRACKING and
        # starts a fresh episode on the next terminal.
        continue_episode = self.state == STATE_REOBSERVING
        self.state = STATE_REOBSERVING
        self.detector_epoch = int(detector_epoch)
        self.reobserve_epoch = int(detector_epoch)
        if not continue_episode:
            self.negative_observations = 0
            self.negative_streak = 0
            self.negative_viewpoints = set()
        self.last_observed_at = float(now)
        self.last_reason = "reobserve_boundary"
        return True

    def observe(
        self, track_id, detector_epoch, target_visible, now, viewpoint_key=None,
    ):
        """Consume one distinct detector message.

        Empty messages before an observation boundary are only a detector gap.
        Empty messages after the boundary count toward explicit negative
        evidence.  A positive target frame cancels the negative episode and
        returns the track to ordinary approach ownership.
        """
        track_id = str(track_id or "").strip()
        if not self.active or track_id != self.track_id:
            return False
        epoch = int(detector_epoch)
        if epoch <= self.detector_epoch:
            return False
        self.detector_epoch = epoch
        self.last_observed_at = float(now)
        if bool(target_visible):
            self.positive_observations += 1
            self.negative_streak = 0
            if self.state == STATE_REOBSERVING:
                self.state = STATE_TRACKING
            self.last_reason = "target_reobserved"
            return True
        if self.state != STATE_REOBSERVING or epoch <= self.reobserve_epoch:
            self.last_reason = "detector_gap"
            return True
        self.negative_observations += 1
        self.negative_streak += 1
        if viewpoint_key is not None:
            try:
                self.negative_viewpoints.add(tuple(viewpoint_key))
            except TypeError:
                self.negative_viewpoints.add(str(viewpoint_key))
        if (
            self.negative_streak >= self.min_negative_observations
            and len(self.negative_viewpoints) >= 2
        ):
            self.state = STATE_LOSS_CERTIFIED
            self.last_reason = "negative_reobserve_consensus"
        else:
            self.last_reason = "negative_reobserve_observation"
        return True

    def loss_certified(self, track_id=None):
        """Return whether this exact track may release its Place obligation."""
        if track_id is not None and str(track_id or "").strip() != self.track_id:
            return False
        return self.state == STATE_LOSS_CERTIFIED

    @property
    def needs_viewpoint_reinspection(self):
        """Whether one view was negative but absence is not yet proven."""
        return (
            self.state == STATE_REOBSERVING
            and self.negative_observations > 0
            and not self.loss_certified()
        )

    def snapshot(self):
        return TargetObservationGateSnapshot(
            state=self.state,
            track_id=self.track_id,
            detector_epoch=int(self.detector_epoch),
            reobserve_epoch=int(self.reobserve_epoch),
            positive_observations=int(self.positive_observations),
            negative_observations=int(self.negative_observations),
            negative_streak=int(self.negative_streak),
            negative_viewpoint_count=len(self.negative_viewpoints),
            last_observed_at=self.last_observed_at,
            last_reason=self.last_reason,
        )


__all__ = [
    "MIN_NEGATIVE_REOBSERVATIONS",
    "STATE_IDLE",
    "STATE_LOSS_CERTIFIED",
    "STATE_REOBSERVING",
    "STATE_TRACKING",
    "TargetObservationGate",
    "TargetObservationGateSnapshot",
]
