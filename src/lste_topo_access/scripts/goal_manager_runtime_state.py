#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Runtime-state initialization for the Goal Manager ROS node."""

import math
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D, PoseStamped, Twist, Vector3Stamped
from image_geometry import PinholeCameraModel
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from lste_msgs.msg import (
    LsteDetection,
    LsteDetections,
    LsteFrontiers,
    LsteScores,
    LsteState,
    LsteTask,
)
from goal_context import default_goal_context
from goal_manager_modes import EXPLORE_PASS_MODE, STATE_PASS
from goal_manager_target_approach_transaction import TargetApproachTransaction
from goal_manager_target_observation_gate import TargetObservationGate

class GoalManagerRuntimeStateMixin:
    def _initialize_runtime_state(self, gp):
        # --- 状态缓存 ---
        self.current_state = STATE_PASS
        self.current_subtype = ""
        self.last_state = None
        self.last_goal: Optional[PoseStamped] = None
        self.next_update_time = 0.0
        self.start_pose: Optional[Pose2D] = None

        self.latest_pose: Optional[Pose2D] = None
        self.latest_state_msg: Optional[LsteState] = None
        self.latest_task: Optional[LsteTask] = None
        self.latest_dets: Optional[LsteDetections] = None
        self.latest_scores: Optional[LsteScores] = None
        self.latest_frontiers: Optional[LsteFrontiers] = None
        self.latest_frontier: Optional[Vector3Stamped] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_cmd_vel: Optional[Twist] = None
        self.latest_global_frontier_goal: Optional[PoseStamped] = None
        # Target routes are validated against the live Navfn/map snapshot even
        # when the exploration executive is disabled. Keep a local monotonic
        # epoch so direct target goals carry the same snapshot identity as
        # frontier routes.
        # Map epochs are issued by Global Frontier for canonical routes.
        # Keep the legacy value only for fixed-goal/non-frontier experiments.
        self.navigation_map_epoch = 1
        self.navigation_map_last_stamp = 0.0
        self.canonical_lifecycle_high_water = 0
        self.canonical_route_high_water = 0
        self.canonical_graph_high_water = 0
        self.canonical_map_epoch_high_water = 0
        self.canonical_origin_epoch_high_water = 0
        self.canonical_projection_epoch_high_water = 0
        # ``action_generation`` is allocated by TEB Bridge, not GoalManager.
        # Keep the last observed transport identity for arbitration logs; zero
        # means that the current semantic decision has not been dispatched.
        self.bridge_action_generation = 0
        self.bridge_action_identity = None
        # The frontier planner labels whether the current goal is a validated
        # route connector or the final approach point.  This semantic is
        # forwarded with the goal intent so the TEB bridge can perform an
        # atomic handoff only for a continuous map route.
        self.global_frontier_route_kind = "frontier_endpoint"
        # The topology-selected route contract is distinct from the temporary
        # connector currently sent to MoveBase.
        self.global_frontier_mission_route_kind = "frontier_endpoint"
        # Route ids come from the frontier planner's BFS transaction. A goal
        # coordinate alone cannot tell the TEB bridge whether a map update is
        # a continuous path extension or an unrelated branch.
        self.global_frontier_route_id = 0
        # Lifecycle identity received with the current Global Frontier route.
        # Keep it explicit so downstream mission publication does not infer a
        # route's identity from an unrelated local callback.
        self.global_frontier_lifecycle_transaction_id = 0
        self.global_frontier_map_epoch = None
        self.global_frontier_origin_epoch = None
        self.global_frontier_projection_epoch = None
        # Identity of the graph planner transaction.  This is deliberately
        # separate from GoalManager's mission transaction_id.
        self.global_frontier_graph_transaction_id = 0
        self.global_frontier_graph_action = ""
        self.global_frontier_graph_obligation_kind = ""
        self.global_frontier_transition_kind = "initial"
        self.global_frontier_predecessor_route_id = 0
        self.global_frontier_predecessor_contract = None
        self.global_frontier_transition_distance = None
        # A map-frame coordinate is only a short-lived execution point.  The
        # companion context identifies its durable Place/WorkItem or portal
        # role and is forwarded unchanged to the mission command log.
        self.global_frontier_goal_context = default_goal_context()
        # The last published context, rather than its temporary coordinates,
        # determines whether a map update is still the same mission action.
        self.last_frontier_goal_context = default_goal_context()
        self.global_frontier_route_ids_by_goal = {}
        # Keep the last map-connected waypoint separate from the currently
        # published visual/context segment. This lets a stale target segment
        # release cleanly after handoff instead of leaving the controller
        # stopped on an obsolete target pose.
        self.last_frontier_goal: Optional[PoseStamped] = None
        # A frontier terminal is a one-way boundary.  The cached endpoint
        # remains useful for diagnostics, but it must not be republished as a
        # new mission transaction until Global Frontier supplies a higher
        # route ID.
        self.last_published_frontier_route_id = 0
        # Set only when TEB reports a terminal event for the current frontier
        # pose. It is consumed by the next successful publish, so one finished
        # action can release exactly one held waypoint.
        self.teb_terminal_goal: Optional[PoseStamped] = None
        # The bridge may coalesce several frontier publications while one
        # action is executing.  Keep the small committed history so a terminal
        # result is matched to the action that actually finished, rather than
        # to whichever frontier happened to be published most recently.
        self.teb_frontier_goal_history: List[PoseStamped] = []
        self.headings: Optional[List[float]] = None
        self.last_dir_idx: Optional[int] = None
        self.goal_source = "uninitialized"
        self.last_goal_source = ""
        self.frontier_debug = "unavailable"
        # Access-Topo 覆盖
        self.access_mode = 0
        self.access_backtrack_goal: Optional[PoseStamped] = None
        # follow 模式内部计时与缓存
        self.target_last_seen: Optional[float] = None
        self.target_last_goal: Optional[PoseStamped] = None
        self.target_last_update: float = 0.0
        self.target_last_heading: Optional[float] = None
        self.target_last_navigation_heading: Optional[float] = None
        # A visual ray is valid at the camera exposure time, not when the
        # detector finishes. Keep this alongside the accepted world heading
        # so every committed segment can be traced to its source image.
        self.target_last_heading_source_stamp: Optional[float] = None
        # Preserve the robot pose that observed the target bearing. A later
        # visual-route failure may occur after the base has moved around a desk;
        # this origin keeps the recovery hint tied to direct target evidence.
        self.target_last_observation_odom: Optional[Tuple[float, float]] = None
        # A static target point is estimated only when multiple source-stamped
        # visual rays provide physical parallax.  It is separate from the
        # latest image bearing, which remains the fallback for one-ray tracks.
        self.target_hypothesis_xy: Optional[Tuple[float, float]] = None
        self.target_hypothesis_residual: Optional[float] = None
        self.target_hypothesis_ray_count = 0
        # The ray estimator remains diagnostic until camera/world calibration
        # has an independent benchmark contract.  Bearing-only pursuit and
        # active parallax are production-safe; a static point may be enabled
        # by a future calibrated experiment without changing this state model.
        self.target_hypothesis_navigation_enabled = False
        self.target_filtered_cx: Optional[float] = None
        self.target_filtered_cy: Optional[float] = None
        self.target_filtered_heading: Optional[float] = None
        self.target_last_detection_stamp = None
        # Counts every distinct /lste/detections message, including an empty
        # result.  Target observations use a separate epoch because an empty
        # detector message is still meaningful after a reached target segment.
        self.target_detector_frame_epoch = 0
        self.target_detector_last_stamp = None
        # Fallback receipt identity for legacy messages with all-zero header
        # fields when a caller bypasses ``record_detector_frame``.
        self.target_detector_receipt_epoch = 0
        self.target_goal_detection_stamp = None
        # Detection evidence and navigation actions have different lifecycles.
        # Keep the current visual-servo segment committed until its move_base
        # action reaches a terminal result; detector frames only update this
        # track state and cannot replace an active TEB action mid-segment.
        self.target_segment_terminal_ready = False
        # A target action terminal is an observation boundary. The target can
        # be much larger and at a different bearing after a short approach,
        # while the first post-terminal detector frame is often a weak or
        # partially occluded box. Keep ownership for fresh evidence instead
        # of immediately handing one rejected image ray back to the frontier.
        self.target_terminal_reobserve_pending = False
        self.target_terminal_reobserve_epoch = 0
        self.target_terminal_reobserve_min_epoch = 0
        self.target_terminal_reobserve_until = 0.0
        # One explicit ownership handoff cancels the old TEB terminal action
        # while post-arrival target evidence is being collected.
        self.target_terminal_observation_intent_sent = False
        # The successor target transaction is held until the TEB bridge
        # confirms that the previous controller lease was released.
        self.target_terminal_observation_pending_transaction = 0
        self.target_terminal_observation_ack_transaction = 0
        self.target_track_label = ""
        # A detector epoch changes for every inference frame. This id remains
        # stable for one continuous visual target track.
        self.target_track_sequence = 0
        self.target_track_id = ""
        # This is deliberately separate from continuous handoffs.  The budget
        # limits only post-terminal movement without a newly committed visual
        # successor; fresh detector-backed handoffs must not consume it.
        self.target_terminal_blind_advances = 0
        self.target_cache_advances = 0
        self.frontier_goal_sent_at: Optional[float] = None
        self.target_observation_hold_goal: Optional[PoseStamped] = None
        self.target_candidate: Optional[LsteDetection] = None
        self.target_candidate_last_seen: Optional[float] = None
        self.target_candidate_source_stamp = None
        self.target_candidate_hits = 0
        self.target_candidate_anchor_cx: Optional[float] = None
        self.target_candidate_anchor_cy: Optional[float] = None
        self.target_candidate_viewpoint_anchor_odom: Optional[Tuple[float, float]] = None
        self.target_candidate_viewpoint_anchor_yaw: Optional[float] = None
        self.target_candidate_viewpoint_translation = 0.0
        self.target_candidate_viewpoint_yaw_delta = 0.0
        self.target_candidate_viewpoint_diverse = False
        self.target_candidate_score_sum = 0.0
        # Source-stamped visual rays are retained only for the current target
        # candidate. They become navigation evidence only after geometric
        # parallax supports a static world point.
        self.target_ray_history = []
        # One direct target frame is not enough to drive a monocular approach,
        # but it is enough to claim the current structural place for another
        # safe observation. This keeps the mission from leaving a room merely
        # because context boxes flicker while avoiding a fixed stationary hold.
        self.target_candidate_room_claim_requested = False
        self.target_reinspection_pending = False
        self.target_follow_confirmed = False
        # Before a weak target can own navigation, the manager may compile one
        # lateral, Navfn-validated observation move.  This is an active
        # information action, not a target pursuit segment.
        self.target_parallax_goal: Optional[PoseStamped] = None
        self.target_parallax_attempts = 0
        self.target_parallax_completed = False
        self.target_parallax_active_side = ""
        self.target_parallax_failed_sides = []
        # A confirmed track owns a semantic approach transaction until an
        # explicit completion, route failure, or evidence-loss transition.
        # This prevents the visual cache-advance optimization from ending the
        # mission after an arbitrary number of viewpoint segments.
        self.target_approach_transaction = TargetApproachTransaction()
        # Semantic target loss is event-driven.  The gate keeps this Place
        # obligation through detector gaps and releases it only after an
        # explicit post-terminal negative observation episode.
        self.target_observation_gate = TargetObservationGate()
        # Target navigation is an explicit mission state, separate from raw
        # detector evidence.  A failed visual route becomes BLOCKED and can be
        # reconsidered only after frontier progress; this prevents perception
        # from reclaiming the controller on every detector frame.
        self.target_execution_state = "TARGET_CANDIDATE"
        self.target_observation_epoch = 0
        self.target_segment_commit_epoch = 0
        # The semantic target owns a stable option identity.  These fields are
        # copied into goal_intent so a controller failure can close the exact
        # viewpoint attempt that was dispatched, even if its coordinates were
        # later transformed or replaced by Navfn.
        self.target_viewpoint_candidate_id = ""
        self.target_viewpoint_attempt_id = ""
        self.target_viewpoint_map_epoch = None
        self.target_blocked = False
        self.target_blocked_goal: Optional[PoseStamped] = None
        self.target_blocked_since: Optional[float] = None
        self.target_blocked_reason = ""
        self.target_failure_count = 0
        self.target_observation_hold_until = 0.0
        self.target_route_validation_next_time = 0.0
        self.target_route_validation_failures = 0
        self.target_route_validation_last_result = "not_checked"
        self.target_route_validation_last_goal = None
        # A non-empty Navfn response can end on the request tolerance boundary
        # rather than the requested visual-ray point. Keep that returned pose
        # with the validation result so the mission and persistent planner use
        # precisely the same endpoint.
        self.target_route_validation_last_endpoint: Optional[PoseStamped] = None
        # This plan is diagnostic/selection input only. Streaming Navfn and
        # TEB remain the execution authorities after the goal is committed.
        self.target_route_validation_last_plan: List[PoseStamped] = []
        self.target_route_validation_last_start_heading: Optional[float] = None
        # In doorway-entry mode, this is the one Navfn-proven task route to
        # the doorway. It remains owned by the task until task_done or an
        # explicit controller failure releases it.
        self.target_room_entry_goal: Optional[PoseStamped] = None
        self.target_room_entry_retry_blocked = False
        self.target_room_entry_blocked_route_id = 0
        self.frontier_terminal_count = 0
        self.target_route_continuity_deferred = False
        self.target_route_continuity_deferred_goal: Optional[PoseStamped] = None
        self.target_route_hold_last_emit = 0.0
        self.navigation_hold_active = False
        self.target_reacquire_goal: Optional[PoseStamped] = None
        self.target_reacquire_started: Optional[float] = None
        self.target_reacquire_attempts = 0
        self.locked_enter_time: Optional[float] = None
        self.target_close_since: Optional[float] = None
        self.target_close_hits = 0
        self.target_close_last_stamp = None
        # Wall time of the most recent close-box detector frame.  A small
        # object at the frame edge can flicker below the box threshold for one
        # detector cycle without leaving the scene; the completion counter must
        # survive that dip instead of resetting to zero.
        self.target_close_last_seen: Optional[float] = None
        self.target_close_wait_reported = False
        # A target can be visually close at a validated terminal and then
        # flicker one pixel below the box-size rule. Keep that fact as part of
        # the same terminal observation transaction; it is reset for every
        # new target viewpoint and never survives target-memory clearing.
        self.target_terminal_close_candidate_seen = False
        # Exhausting the currently compiled viewpoint ledger is not proof that
        # the semantic target was reached.  It opens one explicit revalidation
        # transaction so a false or distant track gets new local observations
        # instead of publishing task_done or retrying the same coordinates.
        self.target_portfolio_revalidation_requested = False
        # A confirmed, close direct target can complete at the current safe
        # viewpoint.  This keeps the camera still while the detector supplies
        # the remaining independent close-confirmation frames instead of
        # forcing a blind last metre through the desk that holds the object.
        self.target_direct_close_hold_reported = False
        self.target_completed_segments = 0
        # Completion evidence belongs to one visual track.  A later target
        # candidate must never inherit a previous track's arrival terminal.
        self.target_approach_track_id = ""
        self.task_done_published = False
        # One-way benchmark acceptance state. It is reset on process/hard-task
        # boundaries, not on visual-track loss, so entering the target room
        # remains a valid terminal fact even if perception is unavailable.
        self.target_room_entry_seen_outside = False
        self.target_room_entry_inside_since = None
        self.target_room_entry_last_pose = None
        self.target_room_entry_event_reported = False
        # Monotonic within this GoalManager process. It is diagnostic identity
        # for one execution transaction, not a frontier route id.
        self.goal_command_id = 0
        self.current_task_id = ""
        # ``task_id`` is a human/task-family label.  Keep a deterministic
        # content version so reusing that label for a new mission resets the
        # old target evidence and cannot inherit its navigation ownership.
        self.current_task_version = ""
        self.current_mission_id = ""
        self.ctx_start_time: Optional[float] = None
        self.ctx_state_hold_until = 0.0
        self.ctx_cooldown_until = 0.0
        self.ctx_last_goal: Optional[PoseStamped] = None
        self.ctx_last_update: float = 0.0
        # Context detections (for example monitor + monitor) are a visual
        # direction cue, not a continuously moving setpoint.  Keep one short
        # inspection segment until the robot reaches it or the bearing has
        # changed materially.
        self.ctx_goal_reached_radius = max(
            0.20, float(gp("~context_goal_reached_radius", 0.55))
        )
        self.ctx_goal_heading_update_threshold = math.radians(max(
            1.0, float(gp("~context_goal_heading_update_threshold_deg", 35.0))
        ))

        # 模式：base_mode 直接由 state/subtype 映射得到；后续可在 effective_mode 上做 override
        self.base_mode = EXPLORE_PASS_MODE
        self.effective_mode = self.base_mode

        # 深度/相机
        self.bridge = CvBridge()
        self.depth_image: Optional[Image] = None
        self.depth_stamp: Optional[rospy.Time] = None
        self.camera_info: Optional[CameraInfo] = None
        self.camera_frame: Optional[str] = None
        self.camera_model = PinholeCameraModel()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_route_validation_client = rospy.ServiceProxy(
            self.target_route_validation_service, GetPlan
        )
