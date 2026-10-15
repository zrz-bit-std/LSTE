"""Pure graph-first action policy for unknown-space exploration.

The frontier score answers *which legal viewpoint is preferable*.  It must not
answer whether a viewpoint is allowed to create work in the current physical
place.  This module is the small architectural boundary between those two
questions.  It has no ROS, map, or numeric tuning parameter.

The policy is deliberately finite-state:

``bootstrap -> observe_work -> retry_viewpoint``

and, after the first observation, the only outward actions are
``probe_portal`` or ``cross_portal``.  The branch-first method may cross a
certified Portal to an unobserved Place before local descendants are exhausted;
covered transit remains gated by the strict local-work rule. A candidate that
cannot be assigned one of these meanings is held for a fresh topology snapshot
instead of being silently downgraded to a generic frontier. A mission-level
target observation WorkItem is stronger still: branch-first cannot suspend its
owning Place while that obligation remains unresolved.
"""

from dataclasses import dataclass

from global_frontier_place_states import (
    PLACE_DORMANT,
    PLACE_READY_TO_EXIT,
)
from global_frontier_place_progress import (
    PLACE_PHASE_OBSERVE,
    derive_place_progress,
)


ACTION_BOOTSTRAP = "bootstrap_observation"
ACTION_LOCAL_WORK = "observe_local_work"
ACTION_VIEWPOINT_RETRY = "retry_viewpoint"
ACTION_PORTAL_PROBE = "probe_portal"
ACTION_PORTAL_CROSSING = "cross_portal"
ACTION_GEOMETRIC_FRONTIER = "geometry_frontier"
ACTION_HOLD = "hold"


@dataclass(frozen=True)
class GraphAction:
    """An auditable action class selected before numeric frontier scoring."""

    kind: str
    reason: str
    source_place_id: object = None
    work_item_id: object = None
    place_hops: object = None

    @property
    def executable(self):
        """Whether the candidate may proceed to route/cost scoring."""
        return self.kind != ACTION_HOLD


def _positive_id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def choose_graph_action(
    *,
    graph_ready,
    source_place_id=None,
    source_place_state=None,
    source_place_observed=False,
    place_hops=None,
    work_item_id=None,
    work_item_available=False,
    viewpoint_retry=False,
    portal_probe=False,
    portal_certified=False,
    work_item_required=False,
    local_work_pending=False,
    must_complete_local_work=False,
    portal_branch_priority=False,
    portal_destination_unobserved=False,
    target_work_pending=False,
):
    """Classify one candidate using durable graph facts.

    ``place_hops`` is only a snapshot-local hint.  A positive hop count is
    accepted as a portal action only when the caller supplies the independent
    ``portal_certified`` proof.  In particular, a missing or zero hop count
    never grants local ownership after an observed Place has lost its
    WorkItem association.

    ``portal_destination_unobserved`` is a graph fact, not a score bonus.  A
    branch-first method may suspend residual local WorkItems only when the
    certified edge leads to a genuinely unobserved Place.  Covered transit
    remains subject to the local-work gate, so branch-first cannot become a
    shortcut for room re-entry.

    The function intentionally does not decide *which* local WorkItem wins;
    that remains the existing deterministic frontier scorer inside a legal
    action class.
    """
    source_place_id = _positive_id(source_place_id)
    work_item_id = _positive_id(work_item_id)

    if not graph_ready:
        return GraphAction(
            ACTION_GEOMETRIC_FRONTIER,
            "place_graph_not_ready",
            source_place_id=source_place_id,
            work_item_id=work_item_id,
            place_hops=place_hops,
        )

    try:
        hops = None if place_hops is None else int(place_hops)
    except (TypeError, ValueError):
        hops = None

    if hops is None:
        return GraphAction(
            ACTION_HOLD,
            "topology_transition_unresolved",
            source_place_id=source_place_id,
            work_item_id=work_item_id,
            place_hops=place_hops,
        )

    place_progress = derive_place_progress(
        place_id=source_place_id,
        state=source_place_state,
        observed=source_place_observed,
        unresolved_work_items=(1 if local_work_pending else 0),
    )

    if hops >= 1:
        # A newly entered Place is not an exit point.  The first labelled
        # component after a Portal crossing is only an identity anchor; it is
        # not permission to immediately take another edge.  This discrete
        # phase gate is what prevents a robot from bouncing through a pair of
        # doors before it has acquired any observation evidence in the new
        # Place.
        # A newly entered Place still needs one observation before it can
        # leave. After that first observation, the branch-first method permits
        # a certified Portal to become the next graph action even when other
        # local frontier descendants are unresolved. Those WorkItems remain
        # owned by the source Place and can never be recreated from a map cell.
        if work_item_required and not source_place_observed:
            return GraphAction(
                ACTION_HOLD,
                "place_observation_required_before_crossing",
                source_place_id=source_place_id,
                work_item_id=work_item_id,
                place_hops=hops,
            )
        if (
            source_place_observed
            and work_item_required
            and local_work_pending
            and target_work_pending
        ):
            return GraphAction(
                ACTION_HOLD,
                "target_observation_required_before_crossing",
                source_place_id=source_place_id,
                work_item_id=work_item_id,
                place_hops=hops,
            )
        if (
            source_place_observed
            and work_item_required
            and local_work_pending
            and not (
                portal_branch_priority and portal_destination_unobserved
            )
        ):
            return GraphAction(
                ACTION_HOLD,
                "local_work_pending_before_crossing",
                source_place_id=source_place_id,
                work_item_id=work_item_id,
                place_hops=hops,
            )
        if (
            portal_certified
            and portal_branch_priority
            and portal_destination_unobserved
            and local_work_pending
        ):
            return GraphAction(
                ACTION_PORTAL_CROSSING,
                "certified_portal_branch_to_unobserved_place",
                source_place_id=source_place_id,
                work_item_id=work_item_id,
                place_hops=hops,
            )
        if portal_certified:
            return GraphAction(
                ACTION_PORTAL_CROSSING,
                "certified_portal_transition",
                source_place_id=source_place_id,
                work_item_id=work_item_id,
                place_hops=hops,
            )
        return GraphAction(
            ACTION_HOLD,
            "cross_place_route_without_portal_certificate",
            source_place_id=source_place_id,
            work_item_id=work_item_id,
            place_hops=hops,
        )

    if source_place_id is None:
        return GraphAction(
            ACTION_BOOTSTRAP,
            "physical_place_not_bootstrapped",
            work_item_id=work_item_id,
            place_hops=hops,
        )

    if source_place_state in (PLACE_DORMANT, PLACE_READY_TO_EXIT):
        return GraphAction(
            ACTION_HOLD,
            "covered_place_is_transit_only",
            source_place_id=source_place_id,
            work_item_id=work_item_id,
            place_hops=hops,
        )

    if portal_probe:
        return GraphAction(
            ACTION_PORTAL_PROBE,
            "unknown_side_is_wall_bounded",
            source_place_id=source_place_id,
            work_item_id=work_item_id,
            place_hops=hops,
        )

    if not source_place_observed:
        return GraphAction(
            ACTION_BOOTSTRAP,
            "first_observation_in_open_place",
            source_place_id=source_place_id,
            work_item_id=work_item_id,
            place_hops=hops,
        )

    if work_item_required and work_item_id is None:
        return GraphAction(
            ACTION_HOLD,
            "observed_place_local_candidate_without_work_item",
            source_place_id=source_place_id,
            place_hops=hops,
        )

    if work_item_id is None:
        return GraphAction(
            ACTION_LOCAL_WORK,
            "legacy_local_work_without_lineage",
            source_place_id=source_place_id,
            place_hops=hops,
        )

    if not work_item_available:
        return GraphAction(
            ACTION_HOLD,
            "work_item_not_available",
            source_place_id=source_place_id,
            work_item_id=work_item_id,
            place_hops=hops,
        )

    return GraphAction(
        ACTION_VIEWPOINT_RETRY if viewpoint_retry else ACTION_LOCAL_WORK,
        "failed_viewpoint_retry" if viewpoint_retry else "unresolved_work_item",
        source_place_id=source_place_id,
        work_item_id=work_item_id,
        place_hops=hops,
    )


__all__ = [
    "ACTION_BOOTSTRAP",
    "ACTION_LOCAL_WORK",
    "ACTION_VIEWPOINT_RETRY",
    "ACTION_PORTAL_PROBE",
    "ACTION_PORTAL_CROSSING",
    "ACTION_GEOMETRIC_FRONTIER",
    "ACTION_HOLD",
    "GraphAction",
    "choose_graph_action",
]
