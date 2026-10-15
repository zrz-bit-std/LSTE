"""Pure place-phase and durable-progress contracts for graph-first exploration.

The occupancy grid is a short-lived observation.  It can expose an unknown
cell without telling us whether that cell belongs to a new physical place or
is simply a stale boundary inside a place that has already been inspected.
This module keeps that distinction explicit and gives the graph executive one
small, auditable phase decision.

No distance, confidence, timeout, or controller parameter is used here.  The
strict Place phase leaves local observation only after its durable WorkItems
are clear. The branch-first policy is the explicit exception: it may suspend
an observed Place for a certified edge to an unobserved Place, while preserving
the WorkItems for a later graph return. Source-side Portal obligations remain
durable outward evidence, and an unknown cell alone is never a progress
certificate for re-entering a covered Place.
"""

from dataclasses import dataclass


PLACE_PHASE_BOOTSTRAP = "bootstrap"
PLACE_PHASE_OBSERVE = "observe"
PLACE_PHASE_EXIT = "exit"
PLACE_PHASE_TRANSIT = "transit"


@dataclass(frozen=True)
class PlaceProgress:
    """Immutable facts used to admit the next graph action."""

    place_id: object
    phase: str
    observed: bool
    covered: bool
    unresolved_work_items: int
    unresolved_portals: int
    reason: str

    @property
    def local_observation_complete(self):
        """Whether the place can legally issue an outward Portal action."""
        return self.phase == PLACE_PHASE_EXIT

    @property
    def has_durable_progress(self):
        """Whether a covered place has an owned obligation worth transiting to."""
        return bool(
            self.unresolved_work_items > 0 or self.unresolved_portals > 0
        )


def _count(value):
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return 0


def derive_place_progress(
    *,
    place_id=None,
    state=None,
    observed=False,
    unresolved_work_items=0,
    unresolved_portals=0,
):
    """Derive one discrete phase from durable place facts.

    ``observed`` is the endpoint/observation fact owned by Place memory.  The
    two unresolved counts are owned by the WorkItem and Portal ledgers.  Their
    values are facts, not weights; changing a detector or map resolution cannot
    silently change this policy.
    """
    work_items = _count(unresolved_work_items)
    portals = _count(unresolved_portals)
    normalized_state = str(state or "").strip().lower()
    if place_id is None:
        return PlaceProgress(
            place_id=None,
            phase=PLACE_PHASE_BOOTSTRAP,
            observed=False,
            covered=False,
            unresolved_work_items=work_items,
            unresolved_portals=portals,
            reason="physical_place_not_bootstrapped",
        )
    if normalized_state == PLACE_PHASE_TRANSIT or normalized_state == "dormant":
        return PlaceProgress(
            place_id=place_id,
            phase=PLACE_PHASE_TRANSIT,
            observed=True,
            covered=True,
            unresolved_work_items=work_items,
            unresolved_portals=portals,
            reason="covered_place_is_transit_only",
        )
    if not bool(observed):
        return PlaceProgress(
            place_id=place_id,
            phase=PLACE_PHASE_OBSERVE,
            observed=False,
            covered=False,
            unresolved_work_items=work_items,
            unresolved_portals=portals,
            reason="place_observation_required_before_crossing",
        )
    if work_items > 0:
        reason = "local_work_pending_before_crossing"
    elif portals > 0:
        # A source-side probe is an outward information action. It remains
        # durable for later re-observation, but it must not prevent a place
        # whose own ObservationWorkItems are complete from using an already
        # certified exit to reach new graph progress.
        reason = "place_observation_complete_with_portal_evidence_pending"
    else:
        reason = "place_observation_complete"
    return PlaceProgress(
        place_id=place_id,
        phase=(PLACE_PHASE_OBSERVE if work_items else PLACE_PHASE_EXIT),
        observed=True,
        covered=True,
        unresolved_work_items=work_items,
        unresolved_portals=portals,
        reason=reason,
    )


def covered_destination_progress(
    destination_region, *, work_item_ledger=None, portal_probe_ledger=None,
):
    """Return durable progress facts for a covered destination Place.

    This intentionally does not inspect the current unknown mask.  The mask is
    allowed to change as SLAM updates and is not a physical identity.  A
    covered destination can justify transit only when its persistent graph
    record still owns unresolved observation or portal work.
    """
    if not isinstance(destination_region, dict):
        return None
    place_id = destination_region.get("id")
    observed = str(destination_region.get("state", "")).strip().lower() in {
        "dormant",
        "ready_to_exit",
    }
    try:
        observed = observed or int(
            destination_region.get("endpoint_observations", 0)
        ) > 0
    except (TypeError, ValueError):
        pass
    if not observed:
        return PlaceProgress(
            place_id=place_id,
            phase=PLACE_PHASE_OBSERVE,
            observed=False,
            covered=False,
            unresolved_work_items=0,
            unresolved_portals=0,
            reason="destination_place_not_observed",
        )
    unresolved_work = 0
    if work_item_ledger is not None:
        count = getattr(work_item_ledger, "unresolved_count", None)
        if callable(count):
            unresolved_work = _count(count(place_id))
    unresolved_portals = 0
    if portal_probe_ledger is not None:
        count = getattr(portal_probe_ledger, "unresolved_count", None)
        if callable(count):
            unresolved_portals = _count(count(place_id))
    return derive_place_progress(
        place_id=place_id,
        state=destination_region.get("state"),
        observed=True,
        unresolved_work_items=unresolved_work,
        unresolved_portals=unresolved_portals,
    )


__all__ = [
    "PLACE_PHASE_BOOTSTRAP",
    "PLACE_PHASE_OBSERVE",
    "PLACE_PHASE_EXIT",
    "PLACE_PHASE_TRANSIT",
    "PlaceProgress",
    "covered_destination_progress",
    "derive_place_progress",
]
