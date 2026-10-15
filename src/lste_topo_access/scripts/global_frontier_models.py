"""Named data contracts shared by global-frontier planning stages.

The ROS node owns mutable runtime state.  These immutable records describe
one planning snapshot or selected route so methods can exchange meaningful
objects instead of growing positional tuples.
"""

from dataclasses import dataclass


@dataclass
class ActiveRouteObservation:
    """Map and progress facts collected for one active frontier tick."""

    distance: float
    information: float
    component: object
    route_distance: float


@dataclass
class ActiveRouteWatchdogState:
    """Execution-state decisions kept separate from frontier selection."""

    turn_phase_active: bool
    recovery_pending: bool
    post_turn_goal_matches: bool
    post_turn_elapsed: float
    post_turn_translation: object
    post_turn_stalled: bool
    region_stagnant: bool
    stalled: bool
    waypoint_distance: float
    waypoint_reached: bool
    portal_edge_expired: bool
    expired: bool


@dataclass
class FrontierSelectionContext:
    """Map-wide facts shared while scoring one frontier snapshot."""

    known_free: object
    covered_place_reachable: object
    source_component: object
    source_label: object
    # The physical Place remains stable while a SLAM snapshot may split its
    # high-clearance cells around furniture.  It is the owner of local work.
    source_place_id: object
    observed_reentry_labels: object
    blocked_place_labels: object
    place_graph_ready: bool
    # A reached endpoint has already certified one viewpoint in the source
    # place. The scheduler may now prefer a certified adjacent action before
    # spending another route on same-place revisits.
    source_place_observed: bool
    action_tiers: tuple
    # Mapping from each current frontier cell to the persistent local
    # observation boundary it belongs to. Remote candidates intentionally do
    # not receive an item here; they must become explicit portal actions.
    work_item_cells: object
    work_item_details: object
    # Mission-level target observation is a Place-owned obligation. It is
    # separate from ordinary local frontier lineage and therefore survives a
    # detector gap and a SLAM component relabel.
    target_observation_work_pending: bool = False


@dataclass
class FrontierCandidateRoute:
    """A candidate endpoint after route and basic lifecycle validation."""

    row: int
    col: int
    action_tier: str
    place_hops: object
    x: float
    y: float
    path_distance: float
    score_path_distance: float
    # The first certified doorway crossed by this route, represented in map
    # coordinates.  It remains optional for local and unanchored routes.
    portal_gate_xy: object = None
    # Local ObservationWorkItem identity. It is created from the frontier arc,
    # never reconstructed from this temporary safe endpoint.
    work_item_id: object = None
    work_item_match: object = None
    work_item_support_cells: int = 0
    # A failed viewpoint does not fail the unknown-side observation task.
    # This marks a distinct safe viewpoint for the same unresolved WorkItem.
    viewpoint_retry: bool = False
    # A source-side doorway inspection. It remains a normal local endpoint
    # for execution, and cannot create a physical place transition.
    portal_observation_probe: object = None
    # Direction of the unknown side for the durable WorkItem. It is optional
    # for legacy callers and used only by task-conditioned action ordering.
    work_item_normal_xy: object = None
    # The graph executive's finite-state decision. Numeric frontier scoring is
    # only a tie-break inside this already legal action class.
    graph_action: object = None
    graph_action_reason: str = ""
    # Destination-view probing may itself carry the base beyond a certified
    # gate. This fact lets terminal verification close the same edge without
    # pretending that the crossing started on the source side.
    portal_crossing_preobserved: bool = False


@dataclass(frozen=True)
class PortalSource:
    """One certified portal and the remote frontier that justified it.

    The destination core is used for graph identity and fallback path costs.
    The endpoint is a short known-free cell beyond the gate, which is the only
    coordinate sent to the navigation controller.
    """

    frontier_row: int
    frontier_col: int
    identity_row: int
    identity_col: int
    gate_row: int
    gate_col: int
    endpoint_row: int
    endpoint_col: int
    # Durable identity for a certified portal whose destination Place has not
    # been bound yet.  Fresh map-derived candidates leave this unset.
    hypothesis_id: object = None


@dataclass(frozen=True)
class PortalCrossingEvidence:
    """Immutable directional proof evaluated in the stable odom frame."""

    verified: bool
    reason: str
    source_signed_distance: object
    terminal_signed_distance: object
    required_destination_depth: float


@dataclass(frozen=True)
class FrontierSelectionRequest:
    """Stable inputs for one candidate-selection pass.

    Keeping this request named prevents the route, scoring, and portal stages
    from exchanging long positional argument lists.  It intentionally has no
    mutable planning state; the host node remains the owner of diagnostics and
    lifecycle state.
    """

    message: object
    steps: object
    unknown: object
    occupied: object
    now: float
    excluded: object
    validation: object
    robot_map: tuple
    heading_reference: object
    max_heading_delta: object
    route_seed: object
    semantic_hint: object
    semantic_pursuit: bool
    preferred_steps: object
    preferred_mask: object
    selection_tier: str
    route_anchor_xy: tuple
    score_path_from_steps: bool
    components: object
    allowed_region_tiers: object
    allow_portal_transitions: bool
    # A closure captured from the current ``map -> odom`` TF relation. It is
    # used only for place identity; all selected goals remain map-frame.
    map_to_physical_xy: object = None


@dataclass(frozen=True)
class FrontierMapContext:
    """One occupancy-grid snapshot and its structural-place interpretation."""

    unknown: object
    known_free: object
    occupied: object
    components: object


@dataclass(frozen=True)
class FrontierRouteGraph:
    """Route masks and BFS fields rooted at the current robot position."""

    strict_free: object
    strict_steps: object
    frontier_free: object
    frontier: object
    validation: object
    route_steps: object
    seed: object


@dataclass(frozen=True)
class FrontierPlanningSnapshot:
    """All immutable inputs consumed by one frontier planning cycle."""

    message: object
    map_context: FrontierMapContext
    route_graph: FrontierRouteGraph
    robot_map: tuple
    robot_yaw_map: object
    now: float


@dataclass(frozen=True)
class ActiveRouteCommand:
    """One immutable command derived from an active frontier route.

    ``route_kind`` is the short-lived controller phase (for example a
    ``frontier_connector``). ``mission_route_kind`` is the immutable route
    lifecycle contract selected by the topology planner.
    """

    mission_xy: tuple
    command_xy: tuple
    command_yaw: object
    route_kind: str
    remaining_path: float
    mission_route_kind: str = "frontier_endpoint"


@dataclass(frozen=True)
class SelectedFrontier:
    """Candidate facts retained when a frontier becomes an active route."""

    row: int
    col: int
    x: float
    y: float
    path_distance: float
    information: float
    structure: float
    score: float
    component: object
    place_hops: object
    route_kind: str
    # Preserve the selected doorway through route activation and terminal
    # commit so place memory can record a physical entrance, not just a core.
    portal_gate_xy: object = None
    work_item_id: object = None
    work_item_match: object = None
    work_item_support_cells: int = 0
    portal_observation_probe: object = None
    viewpoint_retry: bool = False
    graph_action: object = None
    graph_action_reason: str = ""
    # Destination-view probing may itself carry the base beyond a certified
    # gate. This fact lets terminal verification close the same edge without
    # pretending that the crossing started on the source side.
    portal_crossing_preobserved: bool = False

    @classmethod
    def from_candidate(cls, candidate):
        """Convert the internal candidate tuple into a stable named contract.

        Candidate selection evolves independently from route activation and
        prefetch.  Centralizing the positional boundary here prevents a new
        optional candidate field from silently breaking one of those consumers.
        """
        return cls(
            row=candidate[0],
            col=candidate[1],
            x=candidate[2],
            y=candidate[3],
            path_distance=candidate[4],
            information=candidate[5] if len(candidate) > 5 else 0.0,
            structure=candidate[6] if len(candidate) > 6 else 0.0,
            score=candidate[7] if len(candidate) > 7 else 0.0,
            component=candidate[8] if len(candidate) > 8 else None,
            place_hops=candidate[9] if len(candidate) > 9 else None,
            route_kind=(
                candidate[10] if len(candidate) > 10 else "frontier_endpoint"
            ),
            portal_gate_xy=candidate[11] if len(candidate) > 11 else None,
            work_item_id=candidate[12] if len(candidate) > 12 else None,
            work_item_match=candidate[13] if len(candidate) > 13 else None,
            work_item_support_cells=candidate[14] if len(candidate) > 14 else 0,
            portal_observation_probe=candidate[15] if len(candidate) > 15 else None,
            viewpoint_retry=bool(candidate[16]) if len(candidate) > 16 else False,
            graph_action=candidate[17] if len(candidate) > 17 else None,
            graph_action_reason=(candidate[18] if len(candidate) > 18 else ""),
            portal_crossing_preobserved=(
                bool(candidate[20]) if len(candidate) > 20 else False
            ),
        )


@dataclass(frozen=True)
class ObservationDepartureSource:
    """A reached local observation region awaiting a possible outward route."""

    region_id: int
    anchor_map: tuple
    route_id: int


@dataclass(frozen=True)
class PendingPortalArrival:
    """A verified doorway crossing awaiting fresh structural map evidence.

    The action terminal proves the physical crossing immediately.  A current
    structural component can temporarily be absent while SLAM incorporates
    the terminal scan, so the memory commit is retried from later snapshots
    instead of guessing through a wall or dropping the arrival fact.
    """

    route_id: int
    goal_xy: tuple
    terminal_time: float
    source_region_id: object
    portal_gate_xy: object = None
    physical_goal_xy: object = None
    physical_portal_gate_xy: object = None


@dataclass(frozen=True)
class PortalTransitionRetry:
    """One map-validated retry of a doorway edge after local egress.

    This is an edge transaction, not a generic goal retry.  It preserves the
    original adjacent-place destination through one proved-safe retreat, then
    lets a fresh map decide whether that exact portal can still be traversed.
    """

    failed_route_id: int
    source_region_id: object
    goal_xy: tuple
    failure_reason: str
    # A retry must preserve the certified doorway. Retrying only a remote
    # endpoint loses the edge identity and turns the action into an
    # indistinguishable room re-entry.
    portal_gate_xy: object = None
