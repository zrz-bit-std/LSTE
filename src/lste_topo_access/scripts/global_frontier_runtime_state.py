#!/usr/bin/env python3

"""Mutable runtime state for the online frontier explorer."""

import collections

import rospy
from nav_msgs.srv import GetPlan

from global_frontier_place_lifecycle import (
    LocalEgressPlaceLease,
    PlaceDepartureTransaction,
)
from global_frontier_place_memory import FrontierRegionMemory
from global_frontier_completion_recovery import GraphCompletionStateMachine
from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_route_history import ReachedRouteHistory
from global_frontier_semantic_belief import SemanticPlaceBelief
from global_frontier_target_belief import TargetBeliefLedger
from global_frontier_target_observation_work import TargetObservationWorkLedger
from global_frontier_portal_probe_ledger import PortalProbeLedger
from global_frontier_portal_transaction import PortalTransaction
from global_frontier_work_items import PlaceWorkItemLedger
from global_frontier_event_graph import EvidenceEventGraph
from global_frontier_decision_wake import DecisionWakeScheduler
from global_frontier_graph_route_planner import GraphRoutePlanner
from global_frontier_directional_branch_coverage import DirectionalBranchCoverage


class GlobalFrontierRuntimeStateMixin:
    def _initialize_runtime_state(self):
        """Create mutable route, map, and topology state for this process."""
        gp = rospy.get_param
        self.map_msg = None
        self.costmap_msg = None
        # Keep the most recent authoritative full grid separately from the
        # mutable delta reconstruction.  A coalesced update can invalidate the
        # latter even though the last full grid remains a valid reset baseline.
        self._last_full_costmap = None
        self.costmap_message_count = 0
        self.costmap_last_receive_wall = 0.0
        # A hard-reset snapshot and the first live pose can straddle one
        # callback cycle. Do not invalidate the restored full grid on that
        # single handoff sample; later physical motion follows the normal
        # stale-costmap rule.
        self._costmap_rehydrated_pose_grace = False
        # A coalesced costmap delta can arrive immediately after the reset
        # snapshot and legitimately request a resync. Keep the authoritative
        # reset full-grid as a bounded baseline until a newer full grid wins;
        # otherwise the resync marker erases the only planner input we just
        # restored and the warm slice remains costmap-blind forever.
        self._rehydration_costmap_fallback = None
        self._rehydration_costmap_fallback_active = False
        self._applying_rehydration_costmap = False
        self.costmap_stationary_pose_epsilon = max(
            0.0, float(gp("~costmap_stationary_pose_epsilon", 0.01))
        )
        self.costmap_stationary = False
        self.costmap_stationary_since = None
        self._costmap_explicitly_invalid = False
        # A full costmap BFS is useful for candidate validation, but it is not
        # a control-cycle operation.  Reuse the last connected mask briefly;
        # the local costmap/TEB remains responsible for immediate obstacles.
        self.costmap_validation_period = max(
            0.2, float(gp("~costmap_validation_period", 2.0))
        )
        self.cached_costmap_validation = None
        self.cached_costmap_validation_wall = 0.0
        navfn_validation = gp("~navfn_plan_validation", True)
        self.navfn_plan_validation = str(navfn_validation).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.navfn_make_plan_service = gp(
            "~navfn_make_plan_service", "/move_base/NavfnROS/make_plan"
        )
        self.navfn_make_plan_timeout = max(
            0.01, float(gp("~navfn_make_plan_timeout", 0.05))
        )
        # Kept as a visible launch parameter for compatibility. The execution
        # contract deliberately validates exact endpoints, so its value must
        # never relax the GetPlan request.
        self.navfn_make_plan_tolerance = 0.0
        # A ROS service can appear before move_base has populated its global
        # costmap. During that short bootstrap window Navfn legally replies
        # with an empty plan for every goal. Track the first live response so
        # this lifecycle state is not mistaken for a real unreachable branch.
        self.navfn_empty_warmup_seconds = max(
            0.0, float(gp("~navfn_empty_warmup_seconds", 5.0))
        )
        # Do not start selecting frontiers merely because the Navfn service
        # has registered.  During online-SLAM startup that service can exist
        # before GMapping owns map->odom and before the global costmap has a
        # connected free cell around the base.  A short, costmap-derived
        # nearby plan proves the complete map/TF/costmap/Navfn chain before
        # any real frontier is allowed to become a mission transaction.
        self.navfn_startup_probe_distance = max(
            0.10, float(gp("~navfn_startup_probe_distance", 0.40))
        )
        self.navigation_stack_ready = not self.navfn_plan_validation
        self.navigation_readiness_state = (
            "navfn_validation_disabled"
            if self.navigation_stack_ready
            else "waiting_for_map_tf_costmap_navfn"
        )
        self.navfn_first_response_wall = None
        # A live make_plan service can still have no usable global route while
        # online SLAM is constructing its initial costmap. An empty response
        # cannot prove a candidate unreachable until Navfn has first produced
        # at least one non-empty plan in this process lifetime.
        self.navfn_first_nonempty_response_wall = None
        # The selector needs to distinguish a live-but-cold Navfn response
        # from a service/TF failure. Only the former may use the conservative
        # costmap-connected bootstrap route below.
        self.navfn_last_validation_state = "unavailable"
        self.navfn_last_plan_endpoint = None
        self.navfn_service = rospy.ServiceProxy(
            self.navfn_make_plan_service, GetPlan
        )
        self.pose_odom = None
        # Recovery arbitration uses the forward lidar arc, not the global
        # minimum: a corridor wall close to either side is normal, while an
        # obstacle directly in front means a new global endpoint cannot make
        # the base move until local recovery has created some clearance.
        self.scan_forward_minimum = float("nan")
        # Retain the actual sensor horizon for viewpoint-coverage reasoning.
        # This is an instrument property, not an exploration-distance knob.
        self.scan_range_max = None
        # The hardware max range is often much larger than the office area
        # that this scan can actually see. Local ObservationWorkItems use the
        # robust horizon of real returns below, while coverage retains the
        # full instrument capability above.
        self.scan_observation_horizon = None
        self.task_done = False
        # Semantic evidence is indexed by durable Place identity.  It is kept
        # separate from the SLAM grid so map relabels cannot erase what a
        # detector already observed in a physical room.
        self.latest_task = None
        self.current_task_id = ""
        self.current_task_version = ""
        self.current_mission_id = ""
        self.semantic_place_belief = SemanticPlaceBelief()
        # Target evidence has a stricter identity than generic scene labels:
        # it is bound to the task epoch and may later be attached to a
        # WorkItem/Portal bearing received from Goal Manager.
        self.target_belief = TargetBeliefLedger()
        # A target-bearing detector frame creates a mission obligation owned
        # by the physical Place. It is settled only by task_done, not by a
        # transient detector gap or a stale target track timeout.
        self.target_observation_work = TargetObservationWorkLedger()
        self.pending_semantic_observations = []
        self.pending_target_belief_observations = []
        self.pending_target_track_ids = []
        self._target_belief_last_report = {}
        self._target_belief_geometry_reports = set()
        self._semantic_last_report = {}
        self.active_frontier = None
        # Component labels are snapshot-local, so retain a compact map-frame
        # signature with the execution lease. Terminal and failure callbacks
        # otherwise fall back to geometry and can retire a neighbouring room.
        self.active_frontier_component = None
        # The certified doorway used to enter the selected place. It survives
        # route execution so the terminal can commit a directional portal
        # ledger entry even if SLAM relabels the room core.
        self.active_portal_gate_xy = None
        self.active_portal_gate_odom_xy = None
        self.active_portal_destination_odom_xy = None
        # The edge action begins only at its physical gate. A selected route
        # may otherwise spend most of its time safely approaching the door
        # through a corridor.
        self.active_portal_gate_approached_at = None
        # Set from continuous odom evidence as soon as the base has crossed
        # the active directed portal. A later local-controller stall must not
        # convert that completed graph edge into a same-door retry.
        self.active_portal_crossing_observed = False
        # Destination-view probing can itself place the base beyond the gate.
        # Keep this separate from crossing_observed until the route terminal
        # or fresh structural map commits the Place transition.
        self.active_portal_crossing_preobserved = False
        # A physical gate result can be proven to resolve back to the source
        # Place.  This is durable negative evidence for the active route, not a
        # successful crossing; keep it separate so later pose callbacks cannot
        # repeatedly reopen the rejected transaction.
        self.active_portal_crossing_rejected = False
        # A verified doorway crossing can precede the map snapshot that
        # separates the destination room. Keep those facts until fresh
        # structural evidence can commit them without geometry guessing.
        self.pending_portal_arrivals = []
        # Bind every active route to the exact region selected with it. SLAM
        # may later move the endpoint or replace its component evidence, but a
        # terminal/failure event must never guess a different room from that.
        self.active_frontier_region_id = None
        # Physical place ownership changes only after a verified portal
        # arrival.  It prevents snapshot-local component splits from minting
        # a second room while the base is still in the first one.
        self.current_physical_place_id = None
        # A portal arrival commits the durable Place identity before the next
        # map snapshot has rehydrated its local WorkItems/Portal probes.  Keep
        # this one-shot boundary explicit so successor planning cannot mistake
        # an empty, not-yet-reconciled ledger for an empty room and immediately
        # select reverse transit.
        self.place_entry_rehydration_pending = None
        # A branch-first Portal may be discovered to lead back into an
        # already-known Place. Keep the interrupted source obligation as a
        # one-shot graph resume until its certified route is reached.
        self.pending_graph_resume_place_id = None
        self.pending_graph_resume_portal_id = None
        self.pending_graph_resume_destination_place_id = None
        self.active_work_item_id = None
        self.active_work_item_attempt_id = None
        self.active_work_item_place_id = None
        self.active_work_item_goal = None
        self.active_work_item_route_kind = None
        self.active_work_item_dispatch_announced = False
        self.active_portal_probe_id = None
        # Explicit information phase for the active physical doorway probe.
        # This is separate from the controller route kind (which remains a
        # map-frame endpoint for Navfn/TEB).
        self.active_portal_probe_phase = ""
        # Boundary identity and failed-viewpoint identity have different
        # spatial semantics.  The former follows the place completion radius;
        # the latter follows the 0.25 m physical support lattice used by
        # observation WorkItems, allowing a real alternate view after a stall.
        self.place_work_items = PlaceWorkItemLedger(
            self.completed_radius,
            viewpoint_merge_radius=0.25,
        )
        # Keep the selected topological boundary independently from the route
        # kind. ``frontier_endpoint`` may be the first safe endpoint inside an
        # adjacent place and therefore still represents a physical crossing.
        self.active_place_hops = None
        # A region may stay open across several endpoint actions. Stagnation
        # belongs to one physical observation session, never to the region's
        # lifetime, otherwise a prior endpoint can expire a new long route.
        self.active_observation_session_started_at = None
        # A cross-place action is a two-phase graph transaction. It owns the
        # pending state so route callbacks cannot accidentally close a place
        # before the base has actually passed through the doorway.
        self.place_departure = PlaceDepartureTransaction()
        # A successful endpoint yields a durable local observation submap even
        # when the current structural-place labels have not yet separated the
        # doorway.  Keep its exact region identity until a later selected route
        # proves it exits that observed free-space footprint.
        self.observation_departure_source = None
        self.last_observation_departure_footprint_cells = 0
        self.last_observation_departure_anchor_count = 0
        self.active_since = 0.0
        # Keep the physical mission endpoint distinct from a grid cell that is
        # re-associated after each online-SLAM update.  The latter is useful
        # for route validation, but it is not a stable identity for measuring
        # whether the robot is approaching the command sent to move_base.
        self.active_best_distance = None
        self.active_best_goal_distance = None
        self.active_best_path_distance = None
        self.active_progress_time = 0.0
        self.active_last_progress_signal = "none"
        self.active_last_robot_xy = None
        # Map-space distances are authoritative for the endpoint, but online
        # SLAM can legitimately require an initial detour that increases them.
        # Keep an odom-relative start only as secondary evidence of one-way
        # physical transit through that detour; it is never used as a goal.
        self.active_start_odom_xy = None
        self.active_best_detour_odom_distance = 0.0
        self.active_visited_odom_cells = set()
        # A global endpoint can be valid while its first local passage becomes
        # temporarily impassable.  Preserve the positions actually traversed
        # by this lease so a failure can recover through proven free space
        # instead of immediately trying another distant endpoint from the same
        # local trap.
        self.active_route_history = ReachedRouteHistory(max_points=32)
        # ``map`` may change as gmapping corrects scan alignment.  Preserve
        # physical history in odom whenever it is available, and transform a
        # selected anchor only when preparing the next egress command.
        self.active_route_history_frame = None
        self.pending_local_egress = None
        self.local_egress_place_lease = LocalEgressPlaceLease()
        # Portal traversal is an edge transaction: a failed edge receives one
        # reached-history egress and one fresh-map retry, never an unbounded
        # stream of unrelated long-distance replacement goals.
        self.pending_portal_retry = None
        self.active_portal_retry = False
        self.active_local_egress_resumes_portal = False
        self.active_unreachable_since = None
        # Route points stay in the SLAM frame.  The old ``*_odom`` contract
        # made progress checks disagree with move_base after a SLAM correction.
        self.active_last_waypoint_map = None
        # The last command may be a semantic turn connector.  Keep its
        # orientation with the position so a map refresh cannot replace a
        # turn-in-place goal with a pose whose yaw silently resets to zero.
        self.active_last_waypoint_yaw = None
        # The mission kind persists while a connector is replaced from a
        # fresh SLAM snapshot; the route kind below is only the live phase.
        self.active_mission_route_kind = "frontier_endpoint"
        self.active_route_kind = "frontier_endpoint"
        # Configuration is loaded before runtime state.  Do not overwrite the
        # persistent-stream authority here: during startup the bridge may need
        # time to acquire the MoveBase action server, and the global watchdog
        # must not manufacture a route failure in that transport gap.
        persistent_execution = bool(
            getattr(self, "persistent_execution", False)
        )
        self.route_failure_authority = (
            "controller_terminal" if persistent_execution else "global_watchdog"
        )
        self.controller_owned_route_failure = persistent_execution
        self.last_route_stagnation_reported_route_id = 0
        # Geometry ownership and controller ownership have different
        # lifetimes in persistent execution.  After a logical terminal the
        # frontier pointer is released so the graph can deliberate, while
        # the bridge may still hold the old MoveBase action until its action
        # callback reaches DONE.  Keep that short-lived identity tombstone so
        # a delayed controller failure is still matched to the correct route
        # without reviving the old frontier geometry.
        self.last_released_route_id = 0
        self.last_released_route_kind = ""
        self.last_released_route_terminal_received = False
        self.last_released_route_controller_pending = False
        self.last_released_route_transaction_id = 0
        self.last_released_route_lifecycle_transaction_id = 0
        self.last_released_route_action_generation = 0
        self.last_released_route_graph_transaction_id = 0
        self.last_released_route_map_epoch = None
        self.last_released_route_origin_epoch = None
        self.last_released_route_projection_epoch = None
        # A terminal releases semantic ownership first.  Successor planning is
        # held until the bridge confirms that the transport lease is released.
        self.active_terminal_contract = None
        # One scalar cannot represent a delayed predecessor ACK after a
        # successor has already become active. Keep contracts keyed by their
        # route identity; the legacy scalar is maintained as a report alias.
        self.pending_controller_lease_releases = {}
        self.pending_controller_lease_release = None
        self.awaiting_controller_lease_release_ack = False
        self.last_controller_lease_release_ack = None
        self.last_controller_lease_wait_report_wall = 0.0
        # Bridge terminal failures are accepted only after this process has
        # observed the matching action dispatch contract.
        self._bridge_dispatch_contracts = {}
        # Terminal and dispatch use independent ROS topics. Keep a bounded
        # out-of-order terminal until the matching dispatch contract arrives.
        self._pending_terminals_waiting_for_dispatch = {}
        # Stable identity for an exploration route transaction. It changes
        # only when a new BFS branch is selected; an early handoff preserves
        # it only after the discrete path-prefix test proves continuity.
        self.active_route_id = 0
        self.active_route_map_epoch = None
        # Capture the canonical lifecycle identity when a route is activated.
        # Later failure/recovery transitions must not rewrite the predecessor
        # contract that the controller is still draining.
        self.active_route_lifecycle_transaction_id = 0
        # Freeze the route-side predecessor identity before a later graph
        # snapshot can replace ``graph_route_action_transaction``.  Mission
        # transaction/action generation are filled when Bridge adoption is
        # observed; route/lifecycle/epoch identity is available at activation.
        self.active_route_dispatch_contract = None
        # A graph materialization failure can be observed after the physical
        # predecessor has already released its controller lease. Preserve the
        # exact predecessor contract until the next route is activated so the
        # graph failure, release ACK, and successor adoption remain one chain.
        self.pending_graph_successor_predecessor_contract = None
        # Route id, rather than a bare Boolean, makes a late recovery message
        # harmless after the selector has already promoted a new branch.
        self.recovery_pending_route_id = 0
        self.recovery_pending_behavior = ""
        self.recovery_pending_reason = ""
        self.recovery_pending_wall = 0.0
        # In production each frontier endpoint is an atomic move_base action.
        # Physical proximity is useful for prefetching, but it must not retire
        # the active route before the matching action reports success.
        self.active_terminal_received = False
        # A turn connector is a committed execution phase. Keep its pose and
        # tangent frozen until the supervisor reports completion.
        self.turn_connector_released = True
        self.turn_supervisor_state = "UNKNOWN"
        # The supervisor owns a short execution phase for endpoint/portal
        # alignment.  Keep its route label with the state so the watchdog can
        # distinguish an active turn from a stale latched status message.
        self.turn_supervisor_route_kind = ""
        self.turn_supervisor_turn_phase = ""
        self.turn_supervisor_yaw_error = None
        self.turn_supervisor_target_yaw = None
        # The turn supervisor does not own frontier route ids, so retain both
        # route identity and endpoint. A late completion from a superseded
        # action must never affect a newer route.
        self.active_turn_completed_route_id = 0
        self.active_turn_completed_goal = None
        self.active_turn_completed_wall = 0.0
        self.active_turn_completed_odom_xy = None
        self.active_turn_completed_translation = 0.0
        self.active_turn_completed_launched = False
        self.last_status_command_map = None
        self.last_status_command_yaw = None
        self.last_status_mission_map = None
        self.prefetched_frontier = None
        self.prefetched_goal_map = None
        self.prefetched_frontier_information = None
        self.prefetched_frontier_component = None
        self.prefetched_work_item_id = None
        self.prefetched_work_item_match = None
        self.prefetched_work_item_support_cells = 0
        # Endpoint-rooted transition BFS is an advisory tangent for the next
        # action. It is invalidated by route identity or grid-shape changes,
        # not by every high-rate SLAM/costmap update.
        self.active_transition_topology_cache = None
        # Reaching the last safe unknown-space boundary is not a local-planner
        # failure.  Remember this state so the explorer emits one lifecycle
        # event instead of letting the persistent move_base lease recover
        # forever against an endpoint for which no successor exists.
        self.frontier_exhausted = False
        # Every published endpoint is a new destination, but not every update
        # is a disruptive path change. Keep the BFS verdict with the route
        # transaction so metrics can distinguish a prefix extension from a
        # necessary branch selected after inspecting the old endpoint.
        self.active_transition_kind = "initial"
        self.active_predecessor_route_id = 0
        self.active_transition_distance = None
        # A replan request is issued when target execution relinquishes
        # exploration ownership. The following endpoint must be selected from
        # the robot's then-current map pose, not recovered from this cache.
        self.pending_replan_request_id = 0
        self.pending_replan_reason = ""
        self.pending_semantic_hint_map = None
        # Semantic replans are asynchronous requests.  A physical Portal
        # transaction owns the actuator until its crossing/place commit, so a
        # target update is retained and replayed after that transaction ends.
        self.deferred_replan_request = None
        # A target-route failure has a stronger contract than an ordinary
        # semantic tie-break: select a safe frontier that progresses toward the
        # directly observed target whenever such a frontier exists.
        self.pending_semantic_pursuit = False
        # A direct target detection claims the current map-derived room until
        # that room has no remaining executable observation boundary.  This is
        # intentionally separate from the soft semantic direction hint: a
        # missing monitor or a failed short visual segment must not make the
        # selector cross a doorway and abandon the only room with target
        # evidence.
        self.target_region_claim_active = False
        self.target_region_claim_anchor_map = None
        self.target_region_claim_component = None
        self.target_region_claim_track_id = ""
        self.target_region_claim_request_id = 0
        self.target_region_claim_waiting_reported = False
        self.target_region_claim_bound_reported = False
        self.target_region_claim_cross_region_skips = 0
        # A target reinspection request is an event-level graph obligation.
        # Keep it independent from detector timing and from whether the
        # asynchronous target WorkItem has already been bound to this Place.
        self.target_reinspection_pending = False
        # A target observation can remain an unresolved mission obligation even
        # when the current map has no executable viewpoint for it.  Remember
        # that bounded materialization outcome per Place so the graph planner
        # can continue other legal work instead of rebuilding the same
        # reinspection transaction on every map epoch.  New target evidence or
        # a new task explicitly clears the deferral.
        self.target_observation_materialization_deferred_places = set()
        self.last_semantic_pursuit_forward_candidates = 0
        self.last_target_direction_candidates = 0
        self.last_planning_wall = 0.0
        self.last_selection_context = None
        self.last_completion_gate_signature = None
        # Completion is a graph protocol, not a Boolean derived from the
        # current frontier array.  The state machine prevents unresolved
        # ledger entries from becoming a silent infinite wait.
        self.graph_completion_state = GraphCompletionStateMachine()
        # Fast reactive navigation and slow graph deliberation share this
        # replayable structural projection. It is an audit/scheduling layer;
        # Navfn and TEB remain the only motion authorities.
        self.event_graph = EvidenceEventGraph()
        # The event graph is the durable evidence projection; this scheduler
        # is its execution boundary.  It never chooses a goal itself.  The
        # named experiment method decides whether the slow wake is enforced.
        self.decision_wake_scheduler = DecisionWakeScheduler()
        # The durable graph planner chooses a complete Place/Portal route
        # between slow evidence updates. Geometry and TEB still own execution.
        self.graph_route_planner = GraphRoutePlanner()
        self.last_graph_route_plan = None
        self.last_graph_route_plan_signature = None
        self.graph_route_action_transaction_sequence = 0
        self.graph_route_action_transaction = None
        self.graph_route_portal_id = None
        self.graph_route_probe_id = None
        self.graph_route_work_item_id = None
        self.graph_route_work_item_filter = None
        # De-duplicate the event for one unchanged blocked graph decision;
        # changing evidence or an action terminal clears it in the selector.
        self.last_graph_blocked_signature = None
        self.last_graph_candidate_unavailable_signature = None
        # A ready graph obligation may be visible in the durable ledger while
        # its current map projection has no executable edge.  Deduplicate the
        # controller handoff event for that exact route identity and reason.
        self.last_graph_route_unavailable_signature = None
        self.last_portal_source_side_proven = False
        self.last_portal_source_side_proven_id = None
        # A ready graph action remains the durable owner while its transient
        # map projection is unavailable.  Without this lease, visibility
        # filtering can alternate between two legal obligations on successive
        # snapshots even though neither obligation has changed state.
        self.graph_route_plan_lease_active = False
        self.graph_route_plan_lease_signature = None
        self.last_graph_materialization_wait_signature = None
        self.graph_route_fsm_state = "SEARCHING"
        self.graph_route_materialization_miss_signature = None
        self.graph_route_materialization_miss_count = 0
        self.graph_route_materialization_epochs = []
        self.graph_route_materialization_last_event = None
        # Keep materialization budgets by durable obligation.  The legacy
        # scalar fields below are only a view of the currently loaded bucket.
        self.graph_route_materialization_trackers = {}
        self.graph_route_materialization_obligation_key = None
        self.graph_route_materialization_last_observation = None
        self.graph_route_materialization_first_epoch = None
        self.graph_route_materialization_last_epoch = None
        self.graph_route_materialization_invalidated_obligations = set()
        self.graph_route_materialization_invalidated_identity_map = {}
        self.last_graph_route_materialization_invalidation = None
        self.graph_route_materialization_invalidated = False
        self.graph_route_materialization_pending = False
        self.graph_route_materialization_exclusions = {}
        self.last_graph_route_materialization_negative_evidence = None
        self.last_graph_rehydration_signature = None
        self.last_graph_snapshot_projection_signature = None
        self.last_graph_snapshot_projection_complete = False
        self.last_graph_snapshot_visible_work_item_ids = None
        self.last_graph_snapshot_visible_probe_ids = None
        self.last_graph_snapshot_visible_portal_ids = None
        self.last_graph_snapshot_candidate_count = 0
        self.last_graph_snapshot_epoch = None
        self.last_map_epoch = None
        # The hard-reset handler re-runs this initializer, so no graph route
        # identity or map snapshot epoch can leak into the next slice.
        # The graph method records the category/Pareto decision separately
        # from the legacy scalar diagnostics so experiment replay can prove
        # which policy actually selected a route.
        self.last_frontier_decision = None
        # Named graph candidate retained through the tuple-based activation
        # boundary so same-Place refinements keep their physical identity.
        self.last_graph_selected_candidate = None
        self.last_durable_portal_probe_report = None
        # One snapshot-scoped negative result from the durable crossing
        # materializer. The graph adapter consumes it to release the stale
        # action lease and replan; it is cleared after that handoff.
        self.last_durable_portal_crossing_unavailable = None
        # Reverse traversal has the same graph-owned materialization boundary
        # as forward crossing. Keep its last snapshot-scoped miss available to
        # the graph adapter instead of silently returning an empty candidate.
        self.last_durable_portal_egress_unavailable = None
        self.last_durable_portal_egress_report = None
        # The lifecycle tick owns all route-state computation.  Slow planning
        # compatibility fields remain observable for old diagnostics/tests,
        # but the live node does not use a second lock or proposal state
        # machine to coordinate callbacks.
        # The lifecycle manager's tick is the only production compute owner.
        # These legacy locks/gates remain optional compatibility hooks for
        # isolated tests and older compositions, but are absent in the live
        # node so they cannot become a second state machine.
        self.planning_cycle_lock = None
        self.planning_cycle_active = False
        self.planning_cycle_started_wall = 0.0
        self.planning_cycle_skipped = 0
        self.planning_proposal_gate = None
        self.planning_cycle_token = None
        self.planning_cycle_proposal = None
        self.planning_lock = None
        # Terminal ingress is owned by LifecycleManager in the live node.
        # The following fields are optional compatibility state for older
        # compositions and are intentionally not initialized here.
        self.terminal_ingress_lock = None
        self.pending_execution_terminals = None
        self.terminal_ingress_dropped = 0
        # A terminal invalidates any snapshot-local candidate computation that
        # is still running. The callback only flips this atomic fact; the
        # planner consumes it at safe enumeration boundaries.
        self.planning_preempt_requested = False
        self.planning_preempt_reason = ""
        self.rejected_frontiers = collections.deque(maxlen=24)
        self.completed_frontiers = collections.deque(maxlen=self.completed_limit)
        # The topology labels are regenerated on every map snapshot. Cache a
        # completed viewpoint's current label only for that one snapshot so
        # candidate visibility checks remain bounded without carrying stale
        # SLAM labels across updates.
        self.completed_viewpoint_component_cache = {}
        self.last_visibility_coverage_skips = 0
        self.last_covered_place_transit_skips = 0
        self.last_covered_place_footprint_cells = 0
        self.last_covered_place_footprint_anchors = 0
        self.last_observed_place_reentry_skips = 0
        # This is a mission architecture invariant, rather than another
        # frontier-score weight: normal exploration may stay in its current
        # structural place or cross exactly one observed portal.  A later
        # planning cycle then makes the next adjacent-place decision from the
        # new location.  Keeping the horizon at one prevents a distant
        # information boundary from pulling the robot through a room it has
        # already inspected.
        self.place_graph_hop_limit = 1
        # A new route can be chosen before its first control cycle moves the
        # base. Requiring this much physical motion from a completed place's
        # departure anchor prevents a SLAM-grid shift from falsely proving a
        # doorway crossing at the same pose. Both terms are existing execution
        # geometry, not a frontier-score tuning parameter.
        self.place_departure_min_travel_distance = max(
            self.waypoint_release_radius,
            self.endpoint_terminal_wait_radius,
        )
        self.last_place_graph_hop_skips = 0
        self.last_place_graph_unresolved_candidates = 0
        self.last_uncertified_place_transition_skips = 0
        self.last_selected_place_hops = None
        self.place_graph_waiting_for_portal = False
        self.region_memory = FrontierRegionMemory(
            radius=self.region_memory_radius,
            information_delta=self.region_information_delta,
            stagnation_timeout=self.region_stagnation_timeout,
            failure_limit=self.region_failure_limit,
            limit=self.region_memory_limit,
        )
        # Portal geometry is recomputed from each SLAM snapshot, but its
        # physical doorway identity persists across those snapshots.
        self.portal_hypothesis_ledger = PortalHypothesisLedger(
            match_radius=self.region_memory.portal_entry_match_radius,
        )
        self.portal_probe_ledger = PortalProbeLedger(
            match_radius=self.region_memory.portal_entry_match_radius,
        )
        # Directional branch coverage is a durable semantic view over Portal
        # identity. It keeps completed doorway observations out of the probe
        # pool while allowing the same edge to remain graph transit.
        self.directional_branch_coverage = DirectionalBranchCoverage()
        # A Portal route is an edge transaction, not another frontier point.
        # Keep its phase/ownership independent from snapshot-local map labels.
        self.portal_transaction = PortalTransaction()
        self.last_portal_hypothesis_id = 0
        # A structural-place core uses a little more clearance than the actual
        # navigation route. This cuts a normal doorway into a topological gate
        # while preserving a wide room interior. The structural map separately
        # ignores compact observed furniture, so that furniture cannot split
        # the place identity used by exploration.
        self.region_topology_clearance = self.clearance + min(
            0.20, max(0.10, self.frontier_clearance / 2.0)
        )
        self.region_topology_association_radius = max(
            self.frontier_approach_distance,
            self.region_topology_clearance * 2.0,
        )
        # Labels inside a TopologicalFreeSpaceComponents instance are
        # snapshot-local.
        # This monotonically increasing epoch makes accidental reuse across a
        # new SLAM grid impossible, while the memory class bridges legitimate
        # updates by conservative world-space component signatures.
        self.topology_component_epoch = 0
        self.last_frontier_region_tier = "new"
