"""Graph-level recovery and terminal decisions for frontier exploration.

``CompletionGate`` answers a narrow question: are all durable graph
obligations settled? It deliberately does not answer what to do when an
obligation is still pending but the current occupancy snapshot contains no
executable viewpoint. Without that second decision, the planner can emit a
correct ``not complete`` result forever while never asking for another
observation or reporting an explicit blocked outcome.

This module is independent of ROS and navigation parameters. It models
recovery as a small finite-state protocol over graph facts:

``searching -> waiting -> recovering -> blocked``

or, when all obligations are settled, ``searching -> complete``. The
transition from ``recovering`` to ``blocked`` is keyed by the same durable
obligation/evidence signature, not by elapsed time or a retry-count threshold.
New graph evidence changes the signature and reopens recovery.
"""

from dataclasses import dataclass


STATE_SEARCHING = "searching"
STATE_WAITING = "waiting"
STATE_RECOVERING = "recovering"
STATE_BLOCKED = "blocked"
STATE_COMPLETE = "complete"

ACTION_CONTINUE = "continue"
ACTION_WAIT = "wait_for_evidence"
ACTION_REOBSERVE = "reobserve_current_place"
ACTION_TRANSIT = "transit_to_pending_place"
ACTION_BLOCK = "blocked_unresolved_obligations"
ACTION_COMPLETE = "complete_exploration"


@dataclass(frozen=True)
class GraphCompletionDecision:
    """One deterministic graph-level decision."""

    state: str
    action: str
    reason: str
    terminal: bool
    transition: bool
    obligation_counts: dict


def _counts(gate):
    counts = getattr(gate, "counts", {}) or {}
    normalized = []
    for key, value in counts.items():
        try:
            value = int(value)
        except (TypeError, ValueError):
            value = 0
        normalized.append((str(key), value))
    return tuple(sorted(normalized))


class GraphCompletionStateMachine:
    """Turn completion audits into recovery or terminal state transitions.

    The caller supplies facts from the current planning snapshot. In
    particular, ``has_executable_viewpoint`` must mean that a candidate passed
    the existing graph, costmap and Navfn admission checks. The state machine
    never invents a goal and never relaxes those checks.
    """

    def __init__(self):
        self.state = STATE_SEARCHING
        self._last_signature = None
        self._last_decision = None

    @staticmethod
    def _signature(
        gate,
        *,
        graph_ready,
        active_transaction,
        validation_pending,
        has_executable_viewpoint,
        structural_candidate_pending,
        evidence_signature,
    ):
        return (
            bool(getattr(gate, "complete", False)),
            tuple(getattr(gate, "reasons", ()) or ()),
            _counts(gate),
            bool(graph_ready),
            bool(active_transaction),
            bool(validation_pending),
            bool(has_executable_viewpoint),
            bool(structural_candidate_pending),
            evidence_signature,
        )

    def decide(
        self,
        gate,
        *,
        graph_ready=True,
        active_transaction=False,
        validation_pending=False,
        has_executable_viewpoint=False,
        local_reobserve_available=False,
        graph_transit_available=False,
        structural_candidate_pending=False,
        evidence_signature=None,
    ):
        """Advance the protocol from one immutable planning audit.

        ``local_reobserve_available`` and ``graph_transit_available`` are
        capabilities already proven by the caller. If neither is available,
        the first audit still requests one fresh observation pass. Repeating
        the same audit then produces an explicit blocked terminal instead of a
        silent infinite wait. No elapsed time, distance, score or retry count
        is consulted.
        """
        signature = self._signature(
            gate,
            graph_ready=graph_ready,
            active_transaction=active_transaction,
            validation_pending=validation_pending,
            has_executable_viewpoint=has_executable_viewpoint,
            structural_candidate_pending=structural_candidate_pending,
            evidence_signature=evidence_signature,
        )
        previous_state = self.state

        if not graph_ready:
            state, action, reason, terminal = (
                STATE_WAITING,
                ACTION_WAIT,
                "graph_not_ready",
                False,
            )
        elif active_transaction:
            state, action, reason, terminal = (
                STATE_WAITING,
                ACTION_WAIT,
                "portal_transaction_owned",
                False,
            )
        elif validation_pending:
            state, action, reason, terminal = (
                STATE_WAITING,
                ACTION_WAIT,
                "navigation_validation_pending",
                False,
            )
        elif bool(getattr(gate, "complete", False)) and not structural_candidate_pending:
            state, action, reason, terminal = (
                STATE_COMPLETE,
                ACTION_COMPLETE,
                "all_durable_obligations_settled",
                True,
            )
        elif has_executable_viewpoint:
            state, action, reason, terminal = (
                STATE_SEARCHING,
                ACTION_CONTINUE,
                "executable_graph_action_available",
                False,
            )
        else:
            # A graph-level transit is stronger than asking a local Place to
            # rediscover an already-owned boundary. Prefer it when the caller
            # has proved a path to another pending Place.
            if graph_transit_available:
                recovery_action = ACTION_TRANSIT
                recovery_reason = "pending_place_reachable_through_graph"
            elif local_reobserve_available:
                recovery_action = ACTION_REOBSERVE
                recovery_reason = "alternate_viewpoint_evidence_available"
            else:
                recovery_action = ACTION_REOBSERVE
                recovery_reason = (
                    "unresolved_obligation_requires_fresh_place_evidence"
                )

            # Recovery is a one-shot transition for one unchanged evidence
            # signature. The next identical audit is terminally blocked; a
            # changed signature (new topology, ledger state, or caller
            # supplied evidence identity) can request recovery again.
            if signature != self._last_signature:
                state, action, reason, terminal = (
                    STATE_RECOVERING,
                    recovery_action,
                    recovery_reason,
                    False,
                )
            else:
                state, action, reason, terminal = (
                    STATE_BLOCKED,
                    ACTION_BLOCK,
                    "no_executable_viewpoint_for_unresolved_obligation",
                    True,
                )

        self.state = state
        transition = (
            state != previous_state
            or signature != self._last_signature
            or action != (
                None if self._last_decision is None else self._last_decision.action
            )
        )
        decision = GraphCompletionDecision(
            state=state,
            action=action,
            reason=reason,
            terminal=terminal,
            transition=transition,
            obligation_counts=dict(_counts(gate)),
        )
        self._last_signature = signature
        self._last_decision = decision
        return decision

    def reset(self):
        """Forget a completed/blocked episode when a new task starts."""
        self.state = STATE_SEARCHING
        self._last_signature = None
        self._last_decision = None


# A descriptive alias keeps callers free to use either terminology.
CompletionRecoveryStateMachine = GraphCompletionStateMachine


__all__ = [
    "ACTION_BLOCK",
    "ACTION_COMPLETE",
    "ACTION_CONTINUE",
    "ACTION_REOBSERVE",
    "ACTION_TRANSIT",
    "ACTION_WAIT",
    "CompletionRecoveryStateMachine",
    "GraphCompletionDecision",
    "GraphCompletionStateMachine",
    "STATE_BLOCKED",
    "STATE_COMPLETE",
    "STATE_RECOVERING",
    "STATE_SEARCHING",
    "STATE_WAITING",
]
