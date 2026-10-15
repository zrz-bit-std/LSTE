"""Causal envelopes for Place/Portal lifecycle events.

The live map and the durable graph advance at different times.  A portal
arrival can therefore be reported while the graph plan still describes the
source Place, and a late controller failure can arrive after a newer Place has
become current.  This module gives those events an explicit causal identity so
replay code never has to infer a transition from the mutable current snapshot.

The helper is ROS-free and intentionally has no timing or geometric policy.
"""


_TRANSITION_EVENTS = frozenset(
    (
        "frontier_place_departure_prepared",
        "frontier_place_departure_waiting",
        "frontier_place_boundary_crossed",
        "frontier_place_closed",
        "frontier_place_suspended",
        "frontier_place_transited",
        "portal_hypothesis_selected",
        "portal_hypothesis_crossed",
        "portal_hypothesis_failed",
        "portal_execution_failure_observed",
        "portal_hypothesis_destination_bound",
        "portal_transaction_started",
        "portal_transaction_phase",
        "portal_transaction_finished",
        "portal_transaction_aborted",
        "portal_place_entered",
        "portal_place_covered_arrival",
        "portal_place_arrival_rejected",
        "frontier_route_unavailable",
    )
)

_ARRIVAL_EVENTS = frozenset(
    (
        "portal_place_entered",
        "portal_place_covered_arrival",
        "portal_place_arrival_rejected",
        "portal_hypothesis_destination_bound",
    )
)


def _id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _text(value):
    return str(value or "").strip()


def _transaction(fields, portal_id):
    """Use the embedded transaction only when it owns this Portal event."""
    transaction = fields.get("portal_transaction")
    if not isinstance(transaction, dict):
        return {}
    embedded_portal = _id(transaction.get("portal_id"))
    if portal_id is not None and embedded_portal not in (None, portal_id):
        # ``publish_status`` includes the latest transaction report on every
        # event.  A late failure for another Portal must not inherit that
        # report's source Place or transaction ID.
        return {}
    return transaction


def build_transition_envelope(event, fields=None):
    """Return one JSON-safe causal envelope, or ``None`` for other events."""
    fields = fields if isinstance(fields, dict) else {}
    name = _text(event).lower()
    if name not in _TRANSITION_EVENTS and not any(
        key in fields for key in ("source_place_id", "destination_place_id")
    ):
        return None

    explicit_portal = _id(fields.get("portal_id"))
    transaction = _transaction(fields, explicit_portal)
    portal_id = explicit_portal or _id(transaction.get("portal_id"))
    source = _id(fields.get("source_place_id"))
    if source is None:
        source = _id(fields.get("from_place_id"))
    if source is None:
        source = _id(transaction.get("source_place_id"))
    if source is None and name not in _ARRIVAL_EVENTS:
        source = _id(fields.get("region_id"))
        if source is None:
            source = _id(fields.get("place_id"))

    destination = _id(fields.get("destination_place_id"))
    if destination is None and name in _ARRIVAL_EVENTS:
        destination = _id(fields.get("region_id"))
        if destination is None:
            destination = _id(fields.get("place_id"))

    route_id = _id(fields.get("route_id")) or _id(
        transaction.get("route_id")
    )
    transaction_id = _id(fields.get("transaction_id")) or _id(
        transaction.get("transaction_id")
    )
    phase = _text(fields.get("phase")) or _text(fields.get("state"))
    if not phase:
        phase = _text(transaction.get("state"))
    evidence = _text(
        fields.get("evidence_source")
        or fields.get("departure_basis")
        or fields.get("reason")
    )

    if all(value is None for value in (source, destination, portal_id, route_id, transaction_id)):
        return None
    return {
        "from_place_id": source,
        "to_place_id": destination,
        "portal_id": portal_id,
        "transaction_id": transaction_id,
        "route_id": route_id,
        "phase": phase,
        "event": name,
        "evidence": evidence,
    }


__all__ = ["build_transition_envelope"]
