"""Compose runtime facts into one graph-executive action."""

from global_frontier_graph_executive import (
    ACTION_GEOMETRIC_FRONTIER,
    ACTION_PORTAL_PROBE,
    GraphAction,
    choose_graph_action,
)


def candidate_graph_action(
    host,
    request,
    context,
    place_hops,
    work_item_id,
    work_item_available,
    viewpoint_retry,
    portal_probe,
    source_region,
):
    """Return the legal graph action for one already-routed endpoint."""
    local_work_pending = False
    if context.source_place_id is not None:
        pending_method = getattr(host, "context_has_pending_local_work", None)
        if pending_method is not None:
            local_work_pending = bool(pending_method(context))
    try:
        portal_certified = place_hops is not None and int(place_hops) >= 1
    except (TypeError, ValueError):
        portal_certified = False
    graph_action = choose_graph_action(
        graph_ready=context.place_graph_ready,
        source_place_id=context.source_place_id,
        source_place_state=(
            None if source_region is None else source_region.get("state")
        ),
        source_place_observed=context.source_place_observed,
        place_hops=place_hops,
        work_item_id=work_item_id,
        work_item_available=work_item_available,
        viewpoint_retry=viewpoint_retry,
        portal_probe=portal_probe is not None,
        portal_certified=portal_certified,
        work_item_required=bool(
            getattr(host, "work_item_memory_enabled", False)
        ),
        local_work_pending=local_work_pending,
        must_complete_local_work=bool(
            getattr(host, "work_item_memory_enabled", False)
        ),
        portal_branch_priority=bool(
            getattr(host, "branch_first_enabled", False)
        ),
        target_work_pending=bool(
            getattr(context, "target_observation_work_pending", False)
        ),
    )
    # A durable probe identity outranks a transient missing Place label.
    if portal_probe is not None and graph_action.kind == ACTION_GEOMETRIC_FRONTIER:
        graph_action = GraphAction(
            ACTION_PORTAL_PROBE,
            "durable_probe_identity_survives_snapshot_gap",
            source_place_id=context.source_place_id,
            work_item_id=work_item_id,
            place_hops=place_hops,
        )
    return graph_action


__all__ = ["candidate_graph_action"]
