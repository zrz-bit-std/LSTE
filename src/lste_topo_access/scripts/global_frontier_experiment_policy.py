"""Named method boundaries for reproducible exploration comparisons.

This module deliberately contains no ROS or map code.  A method name is a
research contract, not a bundle of incidental score weights: all methods keep
the same SLAM, Navfn, TEB, perception, world, and task interfaces while this
policy states which persistent exploration memory is available.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ExplorationMethodCapabilities:
    """The state abstractions an experimental method is allowed to use."""

    name: str
    place_memory: bool
    work_items: bool
    distance_deduplication: bool
    certified_portals: bool
    structural_score: bool
    heading_policy: bool
    semantic_hint: bool
    # Full method only: task-conditioned Place/Portal value layer.  Keeping
    # this capability explicit makes the benchmark ablation reproducible.
    task_semantic_value: bool = False
    # Branch-first execution is a state-policy capability, not a score weight.
    # Once a Place has produced its first observation, a certified Portal may
    # bypass only local candidates that have no durable viewpoint geometry.
    # Rehydratable WorkItems are place-closure obligations and remain ahead of
    # the crossing; this prevents a later return to the same physical Place.
    branch_first: bool = False
    # Local ObservationWorkItem selection policy. Baselines retain the
    # historical scalar rank; the graph method uses category-first evidence.
    frontier_action_policy: str = "legacy_scalar"
    # Durable graph routing is shared by the Place/Portal/WorkItem arms. It
    # turns an existing multi-hop graph path into one next-edge intent; the
    # geometry and controller layers remain unchanged.
    graph_route_planner: bool = False
    # The full graph method separates reactive execution from slow graph
    # deliberation.  Baselines keep their historical timer-driven selector so
    # this architectural change remains an explicit ablation.
    event_driven_deliberation: bool = False


_METHODS = {
    # A conventional online frontier selector: information gain and path cost
    # only.  It intentionally has neither durable completion memory nor a
    # structural room/door interpretation.
    "frontier": ExplorationMethodCapabilities(
        "frontier", False, False, False, False, False, False, False,
    ),
    # The common stronger frontier baseline.  A reached endpoint is suppressed
    # by a Euclidean radius, but a SLAM-shifted frontier can still appear as a
    # new task and rooms have no durable identity.
    "frontier_distance_dedup": ExplorationMethodCapabilities(
        "frontier_distance_dedup", False, False, True, False, False, False, False,
    ),
    # Ablation of ObservationWorkItem lineage.  Physical Places and certified
    # portal transitions remain, so it isolates whether work ownership rather
    # than merely a room-level closure prevents repeated local exploration.
    "place_portal": ExplorationMethodCapabilities(
        "place_portal", True, False, True, True, True, True, True,
    ),
    # Full method: durable Place identity, physical directed portal proof, and
    # place-owned observation work that survives SLAM frontier movement.
    "place_portal_workitem": ExplorationMethodCapabilities(
        "place_portal_workitem", True, True, True, True, True, True, True,
        task_semantic_value=True,
        branch_first=True,
        frontier_action_policy="event_pareto",
        graph_route_planner=True,
        event_driven_deliberation=True,
    ),
    # Selector-only ablation: keep the same durable graph and branch-first
    # transition, but retain the scalar local rank to isolate this paper's
    # decision-boundary contribution.
    "place_portal_workitem_legacy_rank": ExplorationMethodCapabilities(
        "place_portal_workitem_legacy_rank", True, True, True, True, True, True, True,
        task_semantic_value=True,
        branch_first=True,
        frontier_action_policy="legacy_scalar",
        graph_route_planner=True,
    ),
    # Controlled ablation of branch-first: retain the same Place, Portal,
    # WorkItem and semantic-value capabilities, but require local WorkItems to
    # clear before crossing an outward edge.
    "place_portal_workitem_strict": ExplorationMethodCapabilities(
        "place_portal_workitem_strict", True, True, True, True, True, True, True,
        task_semantic_value=True,
        branch_first=False,
        frontier_action_policy="event_pareto",
        graph_route_planner=True,
        event_driven_deliberation=True,
    ),
}


def method_names():
    """Return stable method identifiers suitable for config validation."""
    return tuple(_METHODS)


def resolve_exploration_method(value):
    """Return one exact capability contract or raise a useful error."""
    name = str(value or "place_portal_workitem").strip().lower().replace("-", "_")
    try:
        return _METHODS[name]
    except KeyError as exc:
        raise ValueError(
            "unknown exploration_method=%r; choose one of: %s"
            % (value, ", ".join(method_names()))
        ) from exc
