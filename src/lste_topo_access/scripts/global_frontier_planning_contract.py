"""Optimistic planning/commit contracts for the slow exploration layer.

The frontier planner consumes a coherent map snapshot, but candidate
enumeration can take materially longer than a ROS callback.  A planning
result therefore needs an identity of its own.  This module keeps that
identity independent from map coordinates and from the controller's route
lease, so an execution terminal can invalidate an old result without waiting
for the expensive selector to finish.

There is deliberately no timeout, score, or controller setting here.  The
contract is a small optimistic-concurrency protocol:

    begin snapshot -> compute proposal -> validate current lease -> commit

Only the owner of the runtime state decides what a proposal means; this
module only answers whether the proposal is still eligible to be committed.
"""

from dataclasses import dataclass
import threading


@dataclass(frozen=True)
class PlanningToken:
    """Identity captured before a slow planning pass starts."""

    generation: int
    active_route_id: int
    wake_sequence: object = None


@dataclass(frozen=True)
class PlanningProposal:
    """A candidate selected from one immutable planning snapshot."""

    token: PlanningToken
    candidate: object
    selection_mode: str = ""
    wait_for_validation: bool = False


@dataclass(frozen=True)
class ProposalDecision:
    """Result of checking a proposal at the execution commit boundary."""

    accepted: bool
    reason: str


class PlanningProposalGate:
    """Thread-safe generation gate for optimistic planner commits.

    ``begin`` replaces the previous snapshot owner.  ``invalidate`` is cheap
    enough for a ROS callback and makes every older proposal stale.  The
    runtime still performs its normal route/graph checks; this gate is the
    additional ownership check that prevents a slow result from winning after
    a terminal or another planner cycle has changed the mission.
    """

    def __init__(self):
        self._lock = threading.RLock()
        self._generation = 0
        self._current = None
        self._last_invalidation = ""

    def begin(self, active_route_id=0, wake_sequence=None):
        """Claim a fresh snapshot generation and return its token."""
        with self._lock:
            self._generation += 1
            token = PlanningToken(
                generation=int(self._generation),
                active_route_id=int(active_route_id or 0),
                wake_sequence=wake_sequence,
            )
            self._current = token
            self._last_invalidation = ""
            return token

    def invalidate(self, reason):
        """Invalidate the current snapshot without acquiring runtime locks."""
        with self._lock:
            self._generation += 1
            self._current = None
            self._last_invalidation = str(reason or "invalidated")

    def check(self, token, active_route_id=0):
        """Check whether ``token`` still owns the current route boundary."""
        with self._lock:
            if token is None or self._current != token:
                return ProposalDecision(False, self._last_invalidation or "stale_snapshot")
            if int(active_route_id or 0) != int(token.active_route_id):
                return ProposalDecision(False, "active_route_changed")
            return ProposalDecision(True, "current_snapshot")

    def commit_if_current(self, token, active_route_id, commit):
        """Run one short activation callback under the proposal boundary.

        A terminal callback invalidates through the same internal lock.  It
        therefore either wins before this method, making the proposal stale,
        or waits until the already-authorized activation has completed.  The
        latter is important: an old controller terminal is then matched
        against the newly committed route instead of interrupting a half
        published transition.
        """
        with self._lock:
            decision = self.check(token, active_route_id)
            if not decision.accepted:
                return decision
            try:
                committed = commit()
            finally:
                if self._current == token:
                    self._current = None
            if committed is False:
                return ProposalDecision(False, "activation_rejected")
            return ProposalDecision(True, "committed")

    def finish(self, token):
        """Release a completed proposal if it is still the current owner."""
        with self._lock:
            if self._current == token:
                self._current = None

    @property
    def generation(self):
        with self._lock:
            return int(self._generation)

    @property
    def last_invalidation(self):
        with self._lock:
            return self._last_invalidation


__all__ = [
    "PlanningProposal",
    "PlanningProposalGate",
    "PlanningToken",
    "ProposalDecision",
]
