"""Pure lifecycle contract for one physical Portal transaction.

The occupancy grid and SLAM labels are transient observations.  This record
is the durable execution lease that connects those observations to one
physical doorway.  A normal frontier route cannot replace the lease between
source-side probing and destination-place commit.
"""

from dataclasses import dataclass
import math


PORTAL_TX_IDLE = "idle"
PORTAL_TX_SOURCE_PROBE = "source_probe"
PORTAL_TX_THROAT = "throat"
PORTAL_TX_DESTINATION_STANDOFF = "destination_standoff"
PORTAL_TX_CROSSING_VERIFIED = "crossing_verified"
PORTAL_TX_PLACE_COMMIT = "place_commit"
PORTAL_TX_ABORTED = "aborted"

_ACTIVE_STATES = frozenset((
    PORTAL_TX_SOURCE_PROBE,
    PORTAL_TX_THROAT,
    PORTAL_TX_DESTINATION_STANDOFF,
    PORTAL_TX_CROSSING_VERIFIED,
))


def _xy(value):
    if value is None:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return (x, y)


@dataclass(frozen=True)
class PortalTransactionSnapshot:
    """Serializable state of one physical edge execution lease."""

    state: str
    transaction_id: int
    route_id: int
    portal_id: object
    source_place_id: object
    gate_xy: object
    destination_xy: object
    retry_count: int
    # Source-side evidence belongs to the physical edge transaction, not to
    # one controller route.  A recovery route may start at the doorway after
    # the first route has already proved the source half-plane.
    source_side_proven: bool
    source_signed_distance: object
    last_reason: str
    last_transition: str


class PortalTransaction:
    """Enforce the Portal phase order and route ownership invariant."""

    def __init__(self):
        self._next_transaction_id = 1
        self._record = None
        self._last = self._snapshot(
            state=PORTAL_TX_IDLE,
            transaction_id=0,
            route_id=0,
            portal_id=None,
            source_place_id=None,
            gate_xy=None,
            destination_xy=None,
            retry_count=0,
            source_side_proven=False,
            source_signed_distance=None,
            last_reason="none",
            last_transition="initial",
        )

    @property
    def state(self):
        return self._last.state

    @property
    def active(self):
        return self.state in _ACTIVE_STATES

    def snapshot(self):
        """Return the latest immutable state, including terminal history."""
        return self._last

    def _snapshot(self, **values):
        return PortalTransactionSnapshot(**values)

    def _set(
        self, state, *, route_id=None, retry_count=None, reason, transition,
    ):
        prior = self._last
        current = self._snapshot(
            state=state,
            transaction_id=prior.transaction_id,
            route_id=(prior.route_id if route_id is None else int(route_id)),
            portal_id=prior.portal_id,
            source_place_id=prior.source_place_id,
            gate_xy=prior.gate_xy,
            destination_xy=prior.destination_xy,
            retry_count=(
                prior.retry_count if retry_count is None else int(retry_count)
            ),
            source_side_proven=bool(prior.source_side_proven),
            source_signed_distance=prior.source_signed_distance,
            last_reason=str(reason),
            last_transition=str(transition),
        )
        self._record = current if state in _ACTIVE_STATES else None
        self._last = current
        return current

    def _route_matches(self, route_id):
        try:
            return int(route_id) == int(self._last.route_id)
        except (TypeError, ValueError):
            return False

    def commit_authorized(
        self, *, route_id=None, portal_id=None, source_place_id=None,
    ):
        """Check the immutable identity of the pending arrival transaction.

        A controller terminal is asynchronous and may arrive after a newer
        route has been selected.  The graph must therefore validate the
        transaction identity before mutating Place memory.  ``None`` keeps
        compatibility with old pure fixtures; production callers pass every
        identity available in the arrival record.
        """
        if self.state not in (
            PORTAL_TX_CROSSING_VERIFIED,
            PORTAL_TX_PLACE_COMMIT,
        ):
            return False
        if route_id is not None and not self._route_matches(route_id):
            return False
        if portal_id is not None:
            try:
                if int(portal_id) != int(self._last.portal_id):
                    return False
            except (TypeError, ValueError):
                return False
        if source_place_id is not None:
            try:
                if int(source_place_id) != int(self._last.source_place_id):
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def start(
        self, route_id, portal_id=None, source_place_id=None,
        gate_xy=None, destination_xy=None, now=None,
        source_side_proven=False, source_signed_distance=None,
    ):
        """Open a Portal lease at the source-side probing phase.

        ``now`` is accepted for callers that use one event clock; lifecycle
        ordering deliberately does not depend on elapsed time.
        """
        del now
        try:
            route_id = int(route_id)
        except (TypeError, ValueError):
            return None
        if route_id <= 0:
            return None
        if self.active:
            if self._route_matches(route_id) and (
                portal_id is None or portal_id == self._last.portal_id
            ):
                return self._last
            return None
        try:
            normalized_portal_id = (
                None if portal_id is None else int(portal_id)
            )
        except (TypeError, ValueError):
            normalized_portal_id = None
        try:
            normalized_source_id = (
                None if source_place_id is None else int(source_place_id)
            )
        except (TypeError, ValueError):
            normalized_source_id = None
        transaction_id = self._next_transaction_id
        self._next_transaction_id += 1
        self._last = self._snapshot(
            state=PORTAL_TX_SOURCE_PROBE,
            transaction_id=transaction_id,
            route_id=route_id,
            portal_id=normalized_portal_id,
            source_place_id=normalized_source_id,
            gate_xy=_xy(gate_xy),
            destination_xy=_xy(destination_xy),
            retry_count=0,
            source_side_proven=bool(source_side_proven),
            source_signed_distance=source_signed_distance,
            last_reason="portal_selected",
            last_transition="source_probe",
        )
        self._record = self._last
        return self._last

    def observe_source_side(
        self, signed_distance, *, route_id=None, reason="source_side_observed",
    ):
        """Persist source-half-plane evidence across controller route leases.

        The controller may need a recovery route whose start pose is already
        at the doorway.  Replacing the transaction's historical source proof
        with that retry pose would make a valid crossing impossible to certify.
        Only the evidence fact is retained; route geometry remains transient.
        """
        if not self.active:
            return None
        if route_id is not None and not self._route_matches(route_id):
            return None
        try:
            signed_distance = float(signed_distance)
        except (TypeError, ValueError):
            return self._last
        if not math.isfinite(signed_distance):
            return self._last
        prior = self._last
        best = signed_distance
        if prior.source_signed_distance is not None:
            try:
                best = min(float(prior.source_signed_distance), signed_distance)
            except (TypeError, ValueError):
                best = signed_distance
        self._last = self._snapshot(
            state=prior.state,
            transaction_id=prior.transaction_id,
            route_id=prior.route_id,
            portal_id=prior.portal_id,
            source_place_id=prior.source_place_id,
            gate_xy=prior.gate_xy,
            destination_xy=prior.destination_xy,
            retry_count=prior.retry_count,
            source_side_proven=bool(prior.source_side_proven or signed_distance < 0.0),
            source_signed_distance=best,
            last_reason=(reason if signed_distance < 0.0 else prior.last_reason),
            last_transition=(
                "source_side_observed" if signed_distance < 0.0
                else prior.last_transition
            ),
        )
        self._record = self._last
        return self._last

    def bind_route(self, route_id, reason="portal_route_retry"):
        """Move the same edge lease to a fresh action route after egress."""
        if not self.active:
            return None
        try:
            route_id = int(route_id)
        except (TypeError, ValueError):
            return None
        if route_id <= 0:
            return None
        if self.state not in (PORTAL_TX_SOURCE_PROBE, PORTAL_TX_THROAT):
            return None
        return self._set(
            self.state,
            route_id=route_id,
            reason=reason,
            transition="route_rebound",
        )

    def gate_reached(self, route_id, reason="source_gate_reached"):
        """Advance source_probe to the physical doorway throat."""
        if not self.active or not self._route_matches(route_id):
            return None
        if self.state == PORTAL_TX_SOURCE_PROBE:
            return self._set(
                PORTAL_TX_THROAT,
                reason=reason,
                transition="source_probe_to_throat",
            )
        if self.state == PORTAL_TX_THROAT:
            return self._last
        return None

    def destination_standoff(self, route_id, reason="destination_standoff_reached"):
        """Record the endpoint beyond the gate before committing a Place."""
        if not self.active or not self._route_matches(route_id):
            return None
        if self.state == PORTAL_TX_SOURCE_PROBE:
            self.gate_reached(route_id, reason="implicit_throat_before_standoff")
        if self.state == PORTAL_TX_THROAT:
            return self._set(
                PORTAL_TX_DESTINATION_STANDOFF,
                reason=reason,
                transition="throat_to_destination_standoff",
            )
        if self.state == PORTAL_TX_DESTINATION_STANDOFF:
            return self._last
        return None

    def crossing_verified(self, route_id, reason="physical_crossing_verified"):
        """Advance to crossing_verified using directional physical evidence."""
        if not self.active or not self._route_matches(route_id):
            return None
        if self.state in (PORTAL_TX_SOURCE_PROBE, PORTAL_TX_THROAT):
            self.destination_standoff(
                route_id, reason="implicit_standoff_from_crossing_evidence",
            )
        if self.state == PORTAL_TX_DESTINATION_STANDOFF:
            return self._set(
                PORTAL_TX_CROSSING_VERIFIED,
                reason=reason,
                transition="destination_standoff_to_crossing_verified",
            )
        if self.state == PORTAL_TX_CROSSING_VERIFIED:
            return self._last
        return None

    def place_commit(
        self, reason="destination_place_committed", *, route_id=None,
        portal_id=None, source_place_id=None,
    ):
        """Allow a destination Place commit only after crossing evidence."""
        if not self.commit_authorized(
            route_id=route_id,
            portal_id=portal_id,
            source_place_id=source_place_id,
        ):
            return None
        if self.state == PORTAL_TX_PLACE_COMMIT:
            return self._last
        if self.state != PORTAL_TX_CROSSING_VERIFIED:
            return None
        return self._set(
            PORTAL_TX_PLACE_COMMIT,
            reason=reason,
            transition="crossing_verified_to_place_commit",
        )

    def retry(self, reason="portal_retry_after_egress"):
        """Reset an un-crossed edge to source_probe without changing identity."""
        if self.state not in (PORTAL_TX_SOURCE_PROBE, PORTAL_TX_THROAT):
            return None
        return self._set(
            PORTAL_TX_SOURCE_PROBE,
            retry_count=self._last.retry_count + 1,
            reason=reason,
            transition="retry_to_source_probe",
        )

    def abort(self, reason="portal_transaction_aborted"):
        """Close a transaction only through an explicit lifecycle decision."""
        if not self.active:
            return self._last
        return self._set(
            PORTAL_TX_ABORTED,
            reason=reason,
            transition="aborted",
        )

    def finish(self, reason="portal_transaction_finished"):
        """Release the lease after place_commit or an explicit abort."""
        if self.active:
            return None
        prior = self._last
        self._last = self._snapshot(
            state=PORTAL_TX_IDLE,
            transaction_id=prior.transaction_id,
            route_id=0,
            portal_id=prior.portal_id,
            source_place_id=prior.source_place_id,
            gate_xy=prior.gate_xy,
            destination_xy=prior.destination_xy,
            retry_count=prior.retry_count,
            source_side_proven=bool(prior.source_side_proven),
            source_signed_distance=prior.source_signed_distance,
            last_reason=str(reason),
            last_transition="finished",
        )
        self._record = None
        return self._last

    def route_allowed(self, route_id, route_kind):
        """Check route ownership without letting ordinary frontier preempt it."""
        if not self.active:
            return True
        if self._route_matches(route_id):
            return True
        # A local egress is the only temporary action allowed to recover the
        # source side before this same edge is rebound for retry.
        return str(route_kind or "").strip().lower() == "local_egress" and (
            self.state in (PORTAL_TX_SOURCE_PROBE, PORTAL_TX_THROAT)
        )

    def preemption_allowed(self, request_kind, route_id=None):
        """Return whether an asynchronous request may replace this lease."""
        if not self.active:
            return True
        if route_id is not None and self._route_matches(route_id):
            return True
        # This is intentionally a semantic contract, not a time/score gate.
        # A target request, replan, or ordinary frontier can be retried after
        # the physical edge has committed; none may interrupt the edge.
        return str(request_kind or "").strip().lower() in (
            "task_done",
            "shutdown",
        )


__all__ = [
    "PORTAL_TX_ABORTED",
    "PORTAL_TX_CROSSING_VERIFIED",
    "PORTAL_TX_DESTINATION_STANDOFF",
    "PORTAL_TX_IDLE",
    "PORTAL_TX_PLACE_COMMIT",
    "PORTAL_TX_SOURCE_PROBE",
    "PORTAL_TX_THROAT",
    "PortalTransaction",
    "PortalTransactionSnapshot",
]
