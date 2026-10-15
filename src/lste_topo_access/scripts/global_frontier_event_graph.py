"""Event-driven evidence graph for the slow exploration executive.

The reactive stack (SLAM, costmap, Navfn, and TEB) produces frequent status
updates.  A deliberative explorer should wake only when a durable fact changes:
a Place is observed, a Portal is certified/crossed, a WorkItem is resolved, or
an action reaches a terminal state.  This module is a small, ROS-free
projection of those facts.  It is deliberately an audit and scheduling
boundary; it does not invent a goal, alter a controller, or use a timer.

The projection is event-sourced from the existing status stream, so a recorded
run can be replayed deterministically and compared with a baseline without
re-running Gazebo.
"""

from collections import deque
from dataclasses import dataclass

from global_frontier_evidence_contract import (
    EvidenceContractSnapshot,
    apply_observation,
    observation_for_event,
)


_STRUCTURAL_EVENTS = frozenset(
    (
        "semantic_task_updated",
        "route_command",
        "route_terminal",
        "route_invalidated",
        "execution_terminal_failure",
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
        "portal_place_entry_observed",
        "frontier_action_selected",
        "work_item_viewpoint_route_rejected",
        "target_track_started",
        "target_segment_committed",
        "target_reinspection_requested",
        "target_loss_certified",
        "target_task_completed",
        "task_done",
        "graph_recovery_requested",
        "graph_exploration_blocked",
        "graph_route_materialization_miss",
        "graph_route_materialization_negative_evidence",
        "graph_route_materialization_invalidated",
        "graph_route_next_action",
        "frontier_route_unavailable",
        "frontier_exhausted",
    )
)


@dataclass(frozen=True)
class GraphEvent:
    """One normalized status event and its slow-layer wake decision."""

    sequence: int
    event: str
    structural: bool
    wake: bool
    reason: str
    route_id: int = 0
    place_id: int = 0
    work_item_id: int = 0
    portal_id: int = 0
    task_version: str = ""
    evidence_revision: int = 0


def _id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return 0
    return value if value > 0 else 0


def _text(value):
    return str(value or "").strip()


class EvidenceEventGraph:
    """Replayable structural projection with a one-shot deliberation wake."""

    def __init__(self, history_limit=512):
        self.history_limit = max(32, int(history_limit))
        self.sequence = 0
        self.structural_revision = 0
        self._wake_pending = False
        self._last_wake = None
        self._seen_structural_keys = set()
        self._history = deque(maxlen=self.history_limit)
        self.active_route_id = 0
        self.active_route_kind = ""
        self.completed_routes = set()
        self.place_states = {}
        self.work_states = {}
        self.portal_states = {}
        self.target_state = ""
        self.task_version = ""
        self.evidence_contract = EvidenceContractSnapshot()

    @staticmethod
    def is_structural(event):
        """Return whether ``event`` changes durable graph meaning."""
        return _text(event).lower() in _STRUCTURAL_EVENTS

    @staticmethod
    def _key(event, fields, route_id, place_id, work_item_id, portal_id):
        """Build an identity key without relying on transient coordinates."""
        explicit = fields.get("event_id")
        if explicit not in (None, ""):
            return ("event_id", _text(event), _text(explicit))
        return (
            _text(event).lower(),
            route_id,
            place_id,
            work_item_id,
            portal_id,
            _id(fields.get("request_id")) or _id(fields.get("replan_request_id")),
            _text(fields.get("state")),
            _text(fields.get("reason")),
        )

    def ingest(self, event, fields=None):
        """Append one status event and update the structural projection.

        Repeated latched status messages with the same durable identity do not
        wake the slow layer twice.  Distinct route IDs, Portal IDs, or WorkItem
        states remain distinct even when their map coordinates are identical.
        """
        fields = fields if isinstance(fields, dict) else {}
        name = _text(event).lower()
        context = fields.get("goal_context")
        context = context if isinstance(context, dict) else {}
        route_id = _id(fields.get("route_id"))
        place_id = _id(
            fields.get(
                "place_id",
                fields.get(
                    "active_region_id",
                    fields.get(
                        "source_place_id",
                        context.get("owner_place_id", context.get("source_place_id")),
                    ),
                ),
            )
        )
        work_item_id = _id(
            fields.get(
                "work_item_id",
                fields.get(
                    "active_work_item_id",
                    context.get("work_item_id"),
                ),
            )
        )
        portal_id = _id(
            fields.get(
                "portal_id",
                fields.get(
                    "portal_probe_id",
                    fields.get(
                        "active_portal_probe_id",
                        context.get("portal_probe_id"),
                    ),
                ),
            )
        )
        task_version = _text(fields.get("task_version", context.get("task_version")))
        self._reduce_evidence(name, fields)
        structural = self.is_structural(name)
        key = self._key(
            name, fields, route_id, place_id, work_item_id, portal_id,
        )
        changed = structural and key not in self._seen_structural_keys
        self.sequence += 1
        if changed:
            self.structural_revision += 1
            self._wake_pending = True
            self._last_wake = GraphEvent(
                sequence=self.sequence,
                event=name,
                structural=True,
                wake=True,
                reason="new_structural_fact",
                route_id=route_id,
                place_id=place_id,
                work_item_id=work_item_id,
                portal_id=portal_id,
                task_version=task_version,
                evidence_revision=self.evidence_contract.revision,
            )
        graph_event = GraphEvent(
            sequence=self.sequence,
            event=name,
            structural=structural,
            wake=changed,
            reason=("new_structural_fact" if changed else "duplicate_or_reactive"),
            route_id=route_id,
            place_id=place_id,
            work_item_id=work_item_id,
            portal_id=portal_id,
            task_version=task_version,
            evidence_revision=self.evidence_contract.revision,
        )
        self._history.append(graph_event)
        if task_version:
            self.task_version = task_version
        self._apply(name, fields, route_id, place_id, work_item_id, portal_id)
        if structural:
            self._seen_structural_keys.add(key)
            if len(self._seen_structural_keys) > self.history_limit * 2:
                # The history is intentionally bounded. The active graph
                # facts remain, while a very old replay may wake again.
                self._seen_structural_keys.pop()
        return graph_event

    def _reduce_evidence(self, event, fields):
        """Project explicit contract fields and known evidence events.

        A route command can declare an evidence boundary, but it cannot satisfy
        that boundary merely by being published.  Only an explicit evidence
        observation, or one of the lifecycle events with a well-defined
        mapping, closes a requirement.  This keeps route terminals from being
        confused with observation proof.
        """
        requirements = fields.get("evidence_requirements")
        # Route commands carry the contract in the self-contained graph plan
        # nested inside the status payload.  Project that declaration at the
        # event boundary so replay sees the same obligations as the live
        # planner, without teaching the ROS publisher about reducer internals.
        if not requirements:
            plan = fields.get("graph_route_plan")
            if not isinstance(plan, dict):
                transaction = fields.get("graph_route_action_transaction")
                transaction = transaction if isinstance(transaction, dict) else {}
                plan = transaction.get("plan")
            if isinstance(plan, dict):
                requirements = plan.get("required_evidence")
        if requirements:
            self.evidence_contract = self.evidence_contract.declare(requirements)
        observation = observation_for_event(event, fields)
        if observation is not None:
            self.evidence_contract = apply_observation(
                self.evidence_contract, observation,
            )
        for value in fields.get("evidence_observations", ()) or ():
            self.evidence_contract = apply_observation(
                self.evidence_contract, value,
            )

    def _apply(self, event, fields, route_id, place_id, work_item_id, portal_id):
        """Project stable IDs and lifecycle labels for replay diagnostics."""
        if event == "semantic_task_updated":
            self.task_version = _text(fields.get("task_version"))
        elif event == "route_command":
            if route_id:
                self.active_route_id = route_id
                self.active_route_kind = _text(fields.get("route_kind"))
            if place_id:
                self.place_states.setdefault(place_id, "active")
            if work_item_id:
                self.work_states.setdefault(work_item_id, "selected")
        elif event in (
            "route_terminal",
            "route_invalidated",
            "execution_terminal_failure",
            "frontier_route_unavailable",
        ):
            if route_id:
                self.completed_routes.add(route_id)
                if route_id == self.active_route_id:
                    self.active_route_id = 0
                    self.active_route_kind = ""
        elif event.startswith("frontier_place_") or event == "physical_place_bootstrapped":
            if place_id:
                self.place_states[place_id] = _text(
                    fields.get("state", event.replace("frontier_place_", ""))
                )
        elif event.startswith("portal_"):
            if portal_id:
                self.portal_states[portal_id] = event
        elif event in ("frontier_action_selected",):
            if work_item_id:
                self.work_states[work_item_id] = _text(
                    fields.get("category", "selected")
                )
        elif event == "work_item_viewpoint_route_rejected":
            if work_item_id:
                self.work_states[work_item_id] = "route_rejected"
        elif event.startswith("target_") or event == "task_done":
            self.target_state = event

    def consume_wake(self):
        """Consume one pending slow-layer wake without clearing graph facts."""
        if not self._wake_pending:
            return None
        self._wake_pending = False
        return self._last_wake

    @property
    def wake_pending(self):
        return bool(self._wake_pending)

    def report(self):
        """Return a compact JSON-safe state for status messages and logs."""
        return {
            "sequence": int(self.sequence),
            "structural_revision": int(self.structural_revision),
            "wake_pending": bool(self._wake_pending),
            "last_wake_event": (
                None if self._last_wake is None else self._last_wake.event
            ),
            "active_route_id": int(self.active_route_id),
            "active_route_kind": self.active_route_kind,
            "completed_route_count": len(self.completed_routes),
            "place_count": len(self.place_states),
            "work_item_count": len(self.work_states),
            "portal_count": len(self.portal_states),
            "target_state": self.target_state,
            "task_version": self.task_version,
            "evidence_contract": self.evidence_contract.as_dict(),
        }

    def history(self):
        """Return the bounded normalized event history for deterministic replay."""
        return tuple(self._history)


__all__ = ["EvidenceEventGraph", "GraphEvent"]
