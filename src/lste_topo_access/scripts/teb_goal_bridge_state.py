#!/usr/bin/env python3
"""Runtime state construction for the TEB goal bridge."""

from contextlib import nullcontext

import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction

from goal_context import default_goal_context


def initialize_bridge_state(bridge):
    """Create state in small, named groups before any ROS callback can run."""
    # LifecycleManager.tick is the live serialization boundary. Keep a
    # no-op context for legacy helpers that still spell their critical
    # sections as ``with self.lock``.
    bridge.lock = nullcontext()
    _initialize_action_state(bridge)
    _initialize_frontier_state(bridge)
    _initialize_mission_state(bridge)
    _initialize_teb_state(bridge)
    bridge.tf_listener = tf.TransformListener()
    bridge.action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    bridge.terminal_dispatch_timer = None


def _initialize_action_state(bridge):
    bridge.latest_goal = None
    bridge.last_dispatched_goal = None
    # Geometry alone cannot identify a mission action: a successor route may
    # deliberately reuse the same endpoint.  Keep the dispatch contract
    # separate from the pose so idle retry logic can distinguish a new route
    # from a duplicate publication of the current one.
    bridge.last_dispatch_identity = None
    bridge.last_terminal_goal = None
    bridge.last_dispatch_monotonic = 0.0
    bridge.last_result_monotonic = 0.0
    bridge.last_result_status = None
    bridge.task_done = False
    # GoalManager may intentionally hold the actuator while collecting target
    # evidence.  The bridge owns this sampled gate so its progress watchdog
    # cannot turn an expected zero command into a route failure.
    bridge.navigation_hold = False
    bridge.navigation_hold_started_monotonic = 0.0
    bridge.navigation_hold_last_change_monotonic = 0.0
    bridge.action_active = False
    bridge.action_server_seen = False
    bridge.deferred_goal_updates = 0
    bridge.deferred_goal_log_wall = 0.0
    bridge.deferred_signature = None
    bridge.bridge_events = 0
    bridge.dispatch_count = 0
    bridge.terminal_count = 0
    bridge.action_generation = 0
    bridge.canonical_lifecycle_high_water = 0
    bridge.canonical_route_high_water = 0
    bridge.canonical_graph_high_water = 0
    bridge.canonical_map_epoch_high_water = 0
    bridge.canonical_origin_epoch_high_water = 0
    bridge.canonical_projection_epoch_high_water = 0
    # A prepared terminal revokes the old action generation immediately. The
    # old contract remains as a tombstone until Global Frontier acknowledges
    # the release, so a late callback cannot reopen the route.
    bridge.pending_terminal_prepare = None
    bridge.pending_lease_release_contract = None
    # One bounded table is shared by stall, terminal, watchdog, and explicit
    # cancel paths. Its key is (route_id, action_generation), not geometry.
    bridge.terminal_action_tombstones = {}
    bridge.released_action_tombstones = {}
    # A successor may be dispatched only after the graph's release ACK has
    # matched this exact predecessor transport contract.
    bridge.pending_successor_release_ack = None
    # Global Frontier can publish a semantic release just before the matching
    # mission command reaches this bridge. Retain that one bounded request and
    # replay it after the route contract is adopted.
    bridge.deferred_frontier_lease_release = None
    bridge.pending_dispatch_generation = 0
    # Semantic ownership may be released before actionlib finishes cancelling
    # the old MoveBase goal. Keep this transport gate until the old goal reaches
    # a terminal state so a successor cannot be sent while it is PREEMPTING.
    bridge.transport_handoff_pending = None
    bridge.transport_handoff_last_report_monotonic = 0.0
    bridge.planner_contract_sequence = 0
    bridge.planner_contract_identity = None
    bridge.planner_contract_state = None
    bridge.planner_contract_reason = "startup"
    bridge.planner_contract_wall = 0.0
    bridge.planner_contract_awaiting_feedback = True
    bridge.planner_contract_producer = ""
    bridge.active_action_contract = None
    # A terminal route remains identifiable for a bounded handoff window.
    # The timer callback only wakes the lifecycle queue; cleanup itself stays
    # inside the single lifecycle tick owner.
    bridge.route_lease_watchdog = None
    bridge.route_lease_watchdog_timer = None
    bridge.latest_frontier_map_epoch = None
    bridge.latest_frontier_map_route_id = 0
    bridge.latest_frontier_graph_transaction_id = 0
    bridge.latest_route_map_epoch = None
    bridge.latest_origin_epoch = None
    bridge.latest_projection_epoch = None
    bridge.latest_graph_transaction_id = 0
    bridge.active_frontier_map_epoch = None
    bridge.active_route_map_epoch = None
    bridge.active_origin_epoch = None
    bridge.active_projection_epoch = None
    bridge.active_graph_transaction_id = 0
    bridge.active_goal_global = None
    # A failed action releases controller-scoped health metrics but keeps the
    # durable route lease identity until Global Frontier publishes a successor.
    bridge.failed_route_id = 0
    bridge.failed_route_source = "unknown"
    bridge.failed_route_priority = 0
    bridge.failed_route_kind = ""
    # A graph route can become unavailable after Global Frontier has released
    # its local lease but while MoveBase still owns the old action.  Keep the
    # route identity as a tombstone so a delayed copy of that command cannot
    # resurrect the controller lease.
    bridge.frontier_lease_released_route_id = 0
    bridge.frontier_lease_released_reason = ""
    # Portal transitions are frozen in odom at activation. Persistent path
    # adoption may replace ``last_dispatched_goal`` with a map-frame plan, so
    # retain the original physical endpoint for the terminal contract.
    bridge.active_portal_source_goal = None
    bridge.active_feedback_distance = None
    bridge.active_feedback_pose = None
    bridge.active_feedback_frame = ""
    bridge.last_feedback_pose_global = None
    bridge.feedback_transform_failures = 0
    bridge.active_best_distance = None
    bridge.active_progress_monotonic = 0.0
    bridge.active_motion_reference = None
    bridge.active_motion_progress_monotonic = 0.0
    bridge.active_navfn_plan_points = []
    bridge.active_navfn_plan_endpoint = None
    bridge.active_navfn_remaining = None
    bridge.active_navfn_best_remaining = None
    bridge.active_navfn_progress_monotonic = 0.0
    bridge.last_feedback_monotonic = 0.0
    bridge.handoff_requested = False
    bridge.handoff_count = 0
    bridge.priority_handoff_count = 0
    bridge.target_segment_handoff_count = 0
    bridge.frontier_segment_handoff_count = 0


def _initialize_frontier_state(bridge):
    bridge.frontier_observation_completion_count = 0
    bridge.frontier_continuous_prefetch_handoff_count = 0
    bridge.frontier_continuous_prefetch_handoff_fallback_count = 0
    bridge.frontier_prefetch_requires_turn_pairs = set()
    bridge.persistent_frontier_prefetch_promoted_pairs = set()
    bridge.persistent_frontier_endpoint_terminal_keys = set()
    # A persistent frontier endpoint has completed semantically while the
    # single MoveBase action remains the transport lease.  The pending record
    # is cleared only when a newer route is adopted or a forced release wins.
    bridge.persistent_frontier_endpoint_pending = None
    bridge.frontier_observation_completion_pending = None
    bridge.frontier_continuous_prefetch_handoff_pending = None
    bridge.prefetched_frontier_goal = None
    bridge.prefetched_frontier_route_id = 0
    bridge.prefetched_frontier_entry_yaw = None
    bridge.prefetched_frontier_entry_yaw_basis = ""
    bridge.frontier_portal_wait = False
    bridge.frontier_portal_wait_route_id = 0
    bridge.frontier_stale_recovery_count = 0
    bridge.frontier_stale_wait_started_monotonic = 0.0
    bridge.handoff_log_monotonic = 0.0


def _initialize_mission_state(bridge):
    bridge.target_failure_latched = False
    bridge.target_failure_goal = None
    bridge.target_failure_epoch = 0
    bridge.target_failure_track_id = ""
    bridge.target_failure_generation = 0
    bridge.target_failure_count = 0
    # A failed target transaction is a durable input tombstone.  The goal and
    # intent topics are latched independently, so an old target command can be
    # delivered after the bridge has already released the controller lease.
    # Keep this identity until a newer target transaction is explicitly
    # accepted; coordinates alone cannot distinguish a replay from a retry.
    bridge.target_lease_tombstone_transaction_id = 0
    bridge.target_lease_tombstone_epoch = 0
    bridge.target_lease_tombstone_track_id = ""
    bridge.latest_intent_source = "unknown"
    bridge.latest_intent_priority = 0
    # The mission command is the canonical lifecycle source for every
    # persistent planner command. Do not derive it from an unrelated local
    # status event after a route has been released.
    bridge.latest_lifecycle_transaction_id = 0
    bridge.latest_route_kind = ""
    bridge.latest_mission_route_kind = ""
    bridge.latest_graph_action = ""
    bridge.latest_graph_obligation_kind = ""
    bridge.latest_route_id = 0
    bridge.latest_route_map_epoch = None
    bridge.latest_target_epoch = 0
    bridge.latest_target_track_id = ""
    bridge.latest_target_viewpoint_candidate_id = ""
    bridge.latest_target_viewpoint_attempt_id = ""
    bridge.latest_goal_context = default_goal_context()
    bridge.latest_predecessor_contract = None
    bridge.latest_goal_transaction_id = 0
    bridge.persistent_installed_target_goal = None
    bridge.persistent_installed_target_transaction = 0
    bridge.persistent_target_pending_transaction = 0
    bridge.persistent_target_pending_goal = None
    bridge.persistent_target_request_transaction = 0
    bridge.persistent_target_approach_reported_transaction = 0
    # Persistent execution reports a semantic target terminal before the
    # single MoveBase lease closes. This transaction marks that ownership
    # boundary so a waiting frontier can become the next route owner.
    bridge.persistent_target_terminal_boundary_transaction = 0
    bridge.persistent_target_republish_transaction = 0
    bridge.latest_intent_goal = None
    bridge.intent_seen = False
    bridge.active_intent_source = "unknown"
    bridge.active_intent_priority = 0
    # Semantic transaction currently driving the persistent route. The action
    # generation may stay stable across stream adoption, while its contract is
    # refreshed to this semantic lease before a terminal callback arrives.
    bridge.active_goal_transaction_id = 0
    bridge.active_route_kind = ""
    bridge.active_mission_route_kind = ""
    bridge.active_graph_action = ""
    bridge.active_graph_obligation_kind = ""
    bridge.active_route_id = 0
    bridge.active_route_map_epoch = None
    bridge.active_target_epoch = 0
    bridge.active_frontier_map_epoch = None
    bridge.active_target_track_id = ""
    bridge.active_target_viewpoint_candidate_id = ""
    bridge.active_target_viewpoint_attempt_id = ""
    bridge.active_goal_context = default_goal_context()
    bridge.active_predecessor_contract = None
    bridge.turn_supervisor_state = "UNKNOWN"
    bridge.turn_supervisor_last_event = "unknown"
    bridge.move_base_terminal_pending = False
    bridge.turn_transition_ready = False


def _initialize_teb_state(bridge):
    bridge.latest_teb_selected_linear = None
    bridge.latest_teb_selected_angular = None
    bridge.latest_teb_feedback_monotonic = 0.0
    # TEB feedback has no route identity on the wire.  Keep an explicit local
    # validity gate so a sample from the previous owner cannot extend the
    # current route's health clock across a zero command or handoff.
    bridge.teb_feedback_valid = False
    bridge.teb_feedback_invalid_reason = "no_feedback"
    bridge.teb_feedback_requires_fresh_planner_command = True
    bridge.teb_feedback_generation = 0
    bridge.teb_feedback_route_id = 0
    bridge.teb_feedback_transaction_id = 0
    bridge.teb_feedback_map_epoch = None
    bridge.teb_feedback_graph_transaction_id = 0
    bridge.teb_feedback_invalidation_count = 0
    bridge.latest_teb_planner_linear = None
    bridge.latest_teb_planner_angular = None
    bridge.latest_teb_planner_command_monotonic = 0.0
    bridge.teb_planner_stationary_since = 0.0
    bridge.local_costmap = None
    bridge.local_costmap_received_monotonic = 0.0
    bridge.persistent_prefetch_admission_cache = None
    bridge.persistent_prefetch_admission_last_report_monotonic = 0.0
    bridge.odom_yaw = None
    bridge.odom_pose_monotonic = 0.0
    bridge.teb_reorientation_started_monotonic = 0.0
    bridge.teb_reorientation_reference_yaw = None
    bridge.teb_reorientation_last_yaw_progress_monotonic = 0.0
    bridge.teb_reorientation_total_yaw = 0.0
    bridge.teb_reorientation_deferrals = 0
    bridge.teb_reorientation_last_status_monotonic = 0.0
