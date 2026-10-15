"""Mutable counters and observations for navigation telemetry."""

import collections
from collections import deque

import rospy
from geometry_msgs.msg import Twist


class NavigationMetricsObserverStateMixin:
    """Initialize state used by command, planner, and lifecycle observers."""

    def _initialize_navigation_observer_state(self):
        self._initialize_control_and_planner_observer_state()
        self._initialize_goal_lifecycle_observer_state()
        self._initialize_smoothness_observer_state()
        self._initialize_failure_evidence_state()

    def _initialize_control_and_planner_observer_state(self):
        self.subgoal = None
        self.command = Twist()
        self.teb_command = Twist()
        self.teb_planner_command = Twist()
        self.teb_turn_supervisor_status = None
        self.teb_turn_supervisor_events = 0
        self.teb_turn_supervisor_last_event = "unknown"
        # A bounded pass-through event where TEB feedback remains forward but
        # its raw cmd_vel has one zero scheduler tick. Keep this separate from
        # safety brakes and route terminal stops so a smoothness result can be
        # audited instead of inferred from the visual behavior.
        self.teb_trajectory_continuity_events = 0
        self.controller_mode = "unknown"
        self.controller_status = "not_available"
        self.controller_source = "unknown"
        self.controller_reason = "not_available"
        self.controller_requested = (float("nan"), float("nan"))
        self.controller_action = (float("nan"), float("nan"))
        self.controller_predicted_clearance = float("nan")
        self.controller_status_changes = 0
        # The mux is the actuator boundary. Its structured status is the
        # authoritative cause for a governor cap or a forced zero; scan-based
        # classification below remains a compatibility fallback only.
        self.cmd_vel_mux_status = None
        self.cmd_vel_mux_status_wall = None
        self.last_odom_wall = None
        self.last_scan_wall = None
        self.last_global_costmap_wall = None
        self.last_local_costmap_wall = None
        self.last_navfn_plan_wall = None
        self.last_global_planner_plan_wall = None
        self.last_teb_global_plan_wall = None
        self.last_teb_local_plan_wall = None
        self.last_move_base_feedback_wall = None
        self.last_teb_cmd_wall = None
        self.last_teb_planner_cmd_wall = None
        self.mux_governor_limited_events = 0
        self.mux_forced_zero_events = 0
        self.mux_status_reason_counts = {}
        self.bridge_events = 0
        self.bridge_deferred_goal_updates = 0
        self.bridge_dispatches = 0
        self.bridge_terminal_events = 0
        self.bridge_endpoint_detections = 0
        self.bridge_successor_dispatches = 0
        self.bridge_priority_handoffs = 0
        self.bridge_target_retries = 0
        self.bridge_target_segment_handoffs = 0
        self.bridge_frontier_segment_handoffs = 0
        self.bridge_frontier_observation_completions = 0
        self.bridge_frontier_terminal_settle_completions = 0
        self.bridge_frontier_continuous_prefetch_handoffs = 0
        self.bridge_frontier_continuous_prefetch_fallbacks = 0
        self.bridge_frontier_prefetch_requires_turn = 0
        # Persistent RouteCorridor handoffs keep the same move_base lease.
        # Track their local-costmap/Navfn admission separately from legacy
        # actionlib handoffs, otherwise a zero-stop improvement would have no
        # evidence that the successor route was actually checked.
        self.bridge_persistent_lookahead_handoffs = 0
        self.bridge_persistent_curve_handoffs = 0
        self.bridge_persistent_lookahead_admission_deferred = 0
        self.bridge_persistent_lookahead_admission_reasons = {}
        self.bridge_frontier_sharp_replacements = 0
        self.bridge_goal_replacements = 0
        self.bridge_priority_goal_replacements = 0
        self.bridge_target_goal_replacements = 0
        self.bridge_active = False
        self.bridge_last_event = "unknown"
        self.bridge_active_intent_source = "unknown"
        self.bridge_latest_intent_source = "unknown"
        # A persistent prefetch may be rejected because the successor begins
        # with a proven sharp BFS tangent.  The current endpoint is then
        # intentionally allowed to complete before TEB rotates for the next
        # branch. Retain that narrow route contract so its terminal stop is
        # not misreported as an unexplained clear-space brake.
        self.pending_terminal_native_reorientation = None
        # Endpoint stops and local-planner recoveries are both visible as a
        # zero command, but demand very different fixes. Keep their lifecycle
        # evidence separate and measure how quickly a successful action is
        # replaced by its successor.
        self.move_base_recovery_events = 0
        self.pending_action_terminal_wall = None
        self.pending_action_terminal_source = ""
        self.terminal_to_dispatch_count = 0
        self.terminal_to_dispatch_total = 0.0
        self.terminal_to_dispatch_max = 0.0
        self.terminal_to_dispatch_last = None
        self.safety_override_events = 0
        # ``safety_override_events`` is a state-transition count kept for
        # backwards-compatible dashboards. These counters describe the actual
        # control stream more precisely: an intervention sample is one where
        # the guard source or requested/applied action differs materially.
        self.safety_intervention_samples = 0
        self.controller_action_delta = 0.0
        self.hard_stop_events = 0
        self.linear_brake_events = 0
        # A local planner deliberately modulates speed for curvature and
        # endpoint approach.  Keep those non-zero changes separate from an
        # actual motion interruption, otherwise a normal 0.50 -> 0.37 m/s
        # turn is reported as an "emergency brake".
        self.speed_modulation_events = 0
        self.teb_speed_modulation_events = 0
        self.turn_only_events = 0
        self.turn_only_start_wall = None
        self.turn_only_duration_total = 0.0
        self.last_brake_wall = 0.0
        # Raw velocity changes have no inherent cause.  Retain recent action
        # lifecycle events so post-run analysis can separate a legitimate
        # terminal/turn boundary from an unexplained clear-path brake.
        self.lifecycle_event_wall = {}
        self.last_move_base_status = "UNKNOWN"
        self.brake_reason_counts = {}
        self.stop_reason_counts = {}
        # A zero command usually arrives one scheduler tick before move_base
        # publishes its SUCCEEDED status. Keep those provisional records long
        # enough to correct their cause when the terminal status follows.
        self.recent_discontinuities = deque()
        self.discontinuity_sequence = 0
        self.last_status_text = ""
        self.last_status_signature = None
        self.teb_status = "not_available"
        self.teb_feedback_state = None
        # TEB feedback and raw cmd_vel use separate ROS callback paths. Keep
        # the receive time so a one-tick raw-zero can be distinguished from a
        # real planner stop when the selected trajectory is still forward.
        self.teb_feedback_wall = None
        # FeedbackMsg carries no route identity. Keep the bridge/scheduler
        # invalidation contract alongside the raw sample so diagnosis cannot
        # mistake a completed route's feedback for live planner output.
        self.teb_feedback_valid = False
        self.teb_feedback_invalid_reason = "no_feedback"
        self.teb_feedback_requires_fresh_planner_command = True
        self.teb_feedback_identity = None
        self.teb_feedback_owner_identity = None
        self.teb_feedback_invalidation_count = 0
        # Typed planner/terminal contracts are observed independently from
        # identity-free TEB feedback. Keep the latest validation result and a
        # bounded set of contracts that arrived before their bridge dispatch
        # status due to ROS topic scheduling.
        self.planner_contract_identity = None
        self.planner_contract_state = None
        self.planner_contract_validation = "unobserved"
        self.planner_contract_reason = "startup"
        self.planner_contract_authorized = 0
        self.planner_contract_rejections = 0
        self.planner_contract_sequence_by_producer = {}
        self.pending_planner_contract = None
        self.terminal_contract_identity = None
        self.terminal_contract_validation = "unobserved"
        self.terminal_contract_rejections = 0
        self.terminal_contract_count = 0
        self.pending_terminal_contracts = deque(maxlen=16)
        self.bridge_dispatch_contracts = {}
        self.bridge_latest_dispatch_contract = None
        self.bridge_active_contract = None
        self.teb_control_cycle_gap_max_feedback_age = max(
            0.05,
            float(rospy.get_param("~teb_control_cycle_gap_max_feedback_age", 0.60)),
        )
        self.teb_control_cycle_gap_min_goal_distance = max(
            0.05,
            float(rospy.get_param("~teb_control_cycle_gap_min_goal_distance", 0.90)),
        )
        self.move_base_feedback_state = None
        self.recovery_state = None
        self.global_costmap_stats = None
        self.local_costmap_stats = None
        # Keep the latest grids only for a bounded failure-context window.  The
        # normal metrics stream still records counts; failure samples extract
        # a small robot-centred patch instead of serializing a full costmap.
        self.global_costmap_message = None
        self.local_costmap_message = None
        self.navfn_plan_stats = None
        self.global_planner_plan_stats = None
        self.teb_global_plan_stats = None
        self.teb_local_plan_stats = None
        # Keep the active TEB global path geometry only for evaluation. The
        # metric uses it to separate intentional route bends from left/right
        # corrections while the planned route is locally straight.
        self.teb_global_plan_geometry = None
        self.teb_local_plan_geometry = None
        self.last_teb_feedback_log_wall = 0.0
        # A valid TEB trajectory whose first velocity remains nearly zero is
        # materially different from an invalid trajectory or a safety-layer
        # stop. Capture its geometry once per plateau so an offline diagnosis
        # can distinguish a blocked endpoint from optimizer degeneration.
        self.teb_zero_velocity_start_wall = None
        self.teb_zero_velocity_snapshot_wall = 0.0
        self.teb_zero_velocity_snapshot_min_duration = 0.70
        self.last_plan_log_wall = {}
        self.state = "unknown"
        self.goal_diagnostic = None
        self.task_done = False
        self.navigation_hold = False
        self.navigation_hold_events = 0
        self.navigation_hold_start_wall = None
        self.navigation_hold_duration_total = 0.0
        self.scan_minimum = float("nan")
        self.scan_forward_minimum = float("nan")
        self.scan_left_minimum = float("nan")
        self.scan_right_minimum = float("nan")
        # A forward laser arc alone is not a collision certificate for a
        # circular base that may be turning. Use the physical footprint plus
        # TEB's hard obstacle clearance when classifying a command brake.
        self.discontinuity_obstacle_clearance = max(
            0.05,
            float(rospy.get_param("/move_base/local_costmap/robot_radius", 0.30))
            + float(rospy.get_param("/move_base/TebLocalPlannerROS/min_obstacle_dist", 0.22))
            + 0.05,
        )
        self.target = None
        self.scores = None
        self.map_stats = None

    def _initialize_goal_lifecycle_observer_state(self):
        self.path_length = 0.0
        self.last_pose_xy = None
        self.goal_messages = 0
        self.goal_changes = 0
        self.goal_delta_sum = 0.0
        self.goal_delta_max = 0.0
        self.goal_last_change_ros = None
        self.goal_last_change_wall = None
        self.last_goal_xy = None
        self.dispatch_count = 0
        self.dispatch_last_xy = None
        self.status_seen = set()
        self.move_base_goal_ids = set()
        self.status_counts = {}
        self.preemptions = 0
        self.frontier_observation_preemptions = 0
        self.frontier_terminal_settle_preemptions = 0
        self.frontier_continuous_prefetch_preemptions = 0
        self.frontier_segment_preemptions = 0
        self.target_segment_preemptions = 0
        self.target_terminal_observation_preemptions = 0
        self.priority_preemptions = 0
        self.task_done_preemptions = 0
        self.unexpected_preemptions = 0
        self.pending_frontier_observation_preemptions = 0
        self.pending_frontier_terminal_settle_preemptions = 0
        self.pending_frontier_continuous_prefetch_preemptions = 0
        self.pending_frontier_segment_preemptions = 0
        self.pending_target_segment_preemptions = 0
        self.pending_target_terminal_observation_preemptions = 0
        self.pending_priority_preemptions = 0
        self.pending_task_done_preemptions = 0
        # A warm-slice hard reset intentionally cancels the active MoveBase
        # goal. Keep that expected PREEMPTED separate from route recovery and
        # expire it if the transport never reports the cancellation.
        self.pending_hard_reset_preemptions = 0
        self.hard_reset_preemption_deadline_wall = 0.0
        # A frontier route can be intentionally cancelled after the graph
        # invalidates its geometric projection. Keep this lifecycle separate
        # from unexplained PREEMPTED controller failures.
        self.route_recovery_preemptions = 0
        self.pending_route_recovery_preemptions = 0
        self.route_recovery_preemption_reasons = {}
        self.expected_route_recovery_preemptions = deque(maxlen=8)
        self.consumed_route_recovery_route_ids = deque(maxlen=16)
        self.aborts = 0
        self.successes = 0
        self.cmd_messages = 0
        self.teb_cmd_messages = 0
        self.angular_sign_flips = 0
        self.last_nonzero_angular_sign = 0
        self.strong_angular_sign_flips = 0
        # Unlike the legacy count above, this counts only direct left/right
        # changes while both adjacent commands are forward motion. It is the
        # metric for actual corridor wobble, excluding route-boundary turns.
        self.forward_steering_sign_flips = 0
        self.teb_angular_sign_flips = 0
        self.teb_last_nonzero_angular_sign = 0
        self.teb_strong_angular_sign_flips = 0
        self.teb_forward_steering_sign_flips = 0
        self.teb_strong_angular_threshold = max(
            0.0, float(rospy.get_param("~teb_strong_angular_threshold", 0.12))
        )
        self.teb_linear_brake_events = 0
        self.teb_planner_linear_brake_events = 0
        self.teb_planner_speed_modulation_events = 0
        self.stop_events = 0
        self.zero_start_wall = None
        self.zero_duration_total = 0.0
        self.stop_duration_count = 0
        self.max_stop_duration = 0.0
        self.last_stop_duration = None
        self.detector_messages = 0
        self.target_messages = 0
        self.target_first_seen_ros = None
        self.target_follow_confirmed_ros = None
        self.target_close_confirmation_started_ros = None
        self.target_close_confirmed_ros = None
        self.legacy_state_locked_ros = None
        self.target_lock_ros = None
        self.task_done_ros = None
        self.task_done_completion_evidence = None
        # Keep target evidence by its mission identity. A detector may lose a
        # weak track and later confirm another one during the same task; the
        # timestamps must never be combined across those tracks.
        self.target_lifecycle_sessions = {}
        self.target_lifecycle_last_completed_key = None
        self.goal_source = "unknown"
        self.goal_transaction_id = 0
        self.mission_goal_messages = 0
        self.goal_transition_kind = "unknown"
        self.goal_predecessor_route_id = 0
        self.goal_transition_distance = None
        self.last_goal_transition_wall = None
        self.persistent_plan_received = 0
        self.persistent_plan_equivalent_retained = 0
        self.persistent_plan_installed = 0
        self.persistent_plan_last_event = "unknown"
        self.persistent_plan_route_version = 0
        self.persistent_plan_geometry_hash = None
        self.continuous_goal_transitions = 0
        self.divergent_goal_transitions = 0
        self.terminal_goal_transitions = 0
        self.unknown_goal_transitions = 0
        self.transition_brake_events = collections.Counter()
        self.last_goal_publish_wall = None
        self.target_goal_changes = 0
        self.target_route_accepts = 0
        self.target_route_rejections = 0
        self.target_route_deferrals = 0
        self.target_route_holds = 0
        self.target_route_semantic_replans = 0
        self.target_route_failures = 0
        self.target_route_releases = 0
        self.target_approach_terminals = 0
        self.target_segments_committed = 0
        self.target_continuous_handoffs_prepared = 0
        self.last_detection_stamp = None
        self.min_clearance = float("inf")
        self.sample_count = 0

    def _initialize_smoothness_observer_state(self):
        # Smoothness diagnostics.  ``forward_angular_energy`` accumulates how
        # much the robot steers while it is travelling at a meaningful forward
        # speed, normalised by forward distance so runs of different length are
        # comparable.  A straight-line wobble shows up as a high value per
        # metre even when the mean angular velocity is near zero.  Brake events
        # are split by whether a real obstacle occupied the forward lidar arc
        # at the moment of the brake: a brake with plenty of clearance is a
        # system-side jitter stop, not an obstacle avoidance response.
        self.forward_distance = 0.0
        self.forward_angular_energy = 0.0
        self.straight_path_distance = 0.0
        self.straight_path_angular_energy = 0.0
        self.straight_path_steering_sign_flips = 0
        self.straight_path_last_nonzero_sign = 0
        self.straight_path_last_nonzero_sign_wall = 0.0
        self.straight_path_sign_flip_window = 1.0
        self.straight_path_samples = 0
        self.straight_path_state = None
        self.straight_path_last_eval_wall = 0.0
        self.straight_path_eval_period = max(
            0.05, float(rospy.get_param("~straight_path_eval_period", 0.20))
        )
        self.straight_path_lookahead = max(
            0.30, float(rospy.get_param("~straight_path_lookahead", 1.00))
        )
        self.straight_path_max_curvature = max(
            0.01, float(rospy.get_param("~straight_path_max_curvature", 0.14))
        )
        self.straight_path_max_heading_error = max(
            0.01, float(rospy.get_param("~straight_path_max_heading_error", 0.20))
        )
        self.last_cmd_wall = None
        self.brake_events_clear = 0
        self.brake_events_near = 0
        self.brake_events_unknown_clearance = 0
        # Forward speed threshold (m/s) above which steering counts as
        # "straight-line" steering energy.  Derived from the TEB speed limit
        # so a slower comparison controller is not unfairly penalised.
        try:
            self.forward_speed_threshold = 0.30 * float(
                self._resolved_startup_params().get("teb_max_vel_x") or 0.50
            )
        except Exception:
            self.forward_speed_threshold = 0.15
        self.forward_speed_threshold = max(0.08, self.forward_speed_threshold)
