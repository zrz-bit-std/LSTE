#!/usr/bin/env python3

"""Exploration, place-map, and candidate-selection configuration."""

import math

import rospy

from global_frontier_experiment_policy import resolve_exploration_method


class GlobalFrontierExplorationConfigurationMixin:
    """Load topology memory and frontier-selection policy settings."""

    def _load_exploration_parameters(self, gp):
        """Resolve settings for coverage, structural places, and planning rate."""
        requested_method = gp("~exploration_method", "place_portal_workitem")
        try:
            self.exploration_capabilities = resolve_exploration_method(
                requested_method
            )
        except ValueError as exc:
            # A benchmark arm is a reproducibility contract. Falling back to
            # the full method would silently change the method being measured.
            rospy.logerr("Global frontier configuration rejected: %s", exc)
            raise
        self.exploration_method = self.exploration_capabilities.name
        self.place_memory_enabled = self.exploration_capabilities.place_memory
        self.work_item_memory_enabled = self.exploration_capabilities.work_items
        self.distance_deduplication_enabled = (
            self.exploration_capabilities.distance_deduplication
        )
        self.certified_portals_enabled = (
            self.exploration_capabilities.certified_portals
        )
        self.structural_score_enabled = (
            self.exploration_capabilities.structural_score
        )
        self.heading_policy_enabled = self.exploration_capabilities.heading_policy
        self.semantic_hint_enabled = self.exploration_capabilities.semantic_hint
        self.task_semantic_value_enabled = bool(
            getattr(self.exploration_capabilities, "task_semantic_value", False)
        )
        self.branch_first_enabled = bool(
            getattr(self.exploration_capabilities, "branch_first", False)
        )
        self.graph_route_planner_enabled = bool(
            getattr(self.exploration_capabilities, "graph_route_planner", False)
        )
        self.event_driven_deliberation_enabled = bool(
            getattr(
                self.exploration_capabilities,
                "event_driven_deliberation",
                False,
            )
        )
        # The named method owns this policy boundary. Do not expose another
        # score/weight switch: baselines and the graph method must remain
        # comparable by changing one declared method contract.
        self.frontier_action_policy = str(
            getattr(
                self.exploration_capabilities,
                "frontier_action_policy",
                "legacy_scalar",
            )
        ).strip().lower()
        # This capability is intentionally derived from the named experiment
        # contract. There is no independent semantic score knob to tune.
        # A frontier reached by the robot has already contributed its camera
        # observation and local lidar scan.  Keep that coverage memory for the
        # whole task, rather than forgetting it after the short retry timeout
        # used for a temporarily blocked route.  As SLAM reveals more space,
        # the true unknown boundary moves beyond this radius and remains a
        # valid candidate.
        self.completed_radius = max(0.2, float(gp("~completed_radius", 1.25)))
        self.completed_limit = max(16, int(gp("~completed_limit", 256)))
        self.candidate_limit = max(32, int(gp("~candidate_limit", 512)))
        self.active_reassociation_radius = max(
            0.4, float(gp("~active_reassociation_radius", 1.0))
        )
        self.info_radius = max(1, int(gp("~info_radius_cells", 8)))
        # Frontier points are not places. A place can yield several safe
        # approach points as scans resolve its geometry, so retain a small
        # persistent place ledger above point-level completion. Its identity
        # comes from ``StructuralPlaceMap`` below, not from raw obstacle
        # clearance: desks and chairs must remain navigation obstacles without
        # becoming architectural boundaries.
        self.region_memory_radius = max(
            self.completed_radius * 2.0,
            float(gp(
                "~region_memory_radius",
                max(self.completed_radius * 2.0, self.frontier_approach_distance * 2.0),
            )),
        )
        self.region_information_delta = max(
            1.0,
            float(gp("~region_information_delta", max(8, self.info_radius))),
        )
        self.region_stagnation_timeout = max(
            0.4,
            float(gp(
                "~region_stagnation_timeout",
                self.stall_timeout,
            )),
        )
        self.region_failure_limit = max(
            1, int(gp("~region_failure_limit", 2))
        )
        self.region_memory_limit = max(
            16, int(gp("~region_memory_limit", 128))
        )
        # This is a structural interpretation bound, not a motion-planner
        # clearance or exploration reward. Obstacles no larger than a normal
        # office furnishing are ignored only while constructing the separate
        # place map; Navfn, costmaps, and TEB still receive the raw obstacle.
        self.place_furniture_max_span_m = max(
            0.25, float(gp("~place_furniture_max_span_m", 2.5))
        )
        # A lidar max-range frontier in an empty open area is often just the
        # edge of a scan, not an entrance worth searching. Doorways, corridor
        # branches and room boundaries have occupied cells nearby. Rewarding
        # that local structure makes coverage spend time in discoverable indoor
        # space before drifting into unconstrained open floor.
        self.structure_radius = max(1, int(gp("~structure_radius_cells", 10)))
        self.structure_weight = max(0.0, float(gp("~structure_weight", 0.16)))
        self.min_structure_cells = max(0, int(gp("~min_structure_cells", 3)))
        # Dead-end detection: when the robot is within this distance of the
        # active frontier and no unknown remains within ``dead_end_unknown_cells``
        # of the frontier cell, the route ends at a resolved wall with no
        # opening.  Mark it inspected and choose a real passage instead of
        # driving into the known wall.
        self.dead_end_check_distance = max(
            0.0, float(gp("~dead_end_check_distance", 4.0))
        )
        self.dead_end_unknown_cells = max(
            2, int(gp("~dead_end_unknown_cells", 15))
        )
        # Exploration still rewards information and doorway-like structure,
        # but a branch whose *route's first tangent* is behind the robot
        # should not win by a small score margin and force TEB to brake and
        # make an abrupt U-turn.  Endpoint bearing is not sufficient here: a
        # reachable endpoint in front of the robot may require a doorway
        # detour that starts behind it. This is a soft continuity cost rather
        # than a hard direction filter, so a necessary turn remains valid.
        self.heading_weight = max(0.0, float(gp("~heading_weight", 2.5)))
        self.heading_hard_limit = math.radians(max(
            0.0, float(gp("~heading_hard_limit_deg", 115.0))
        ))
        # Successor selection is a route-transition decision, not merely a
        # weighted frontier score. Reuse the bridge's three execution
        # envelopes: first seek a smooth continuation, then a curve that TEB
        # may stream, and only then accept a terminal reorientation branch.
        # The final hard limit still permits necessary indoor turns.
        self.successor_smooth_heading_limit = math.radians(max(
            0.0, float(gp("~successor_smooth_heading_limit_deg", 45.0))
        ))
        self.successor_curve_heading_limit = math.radians(min(
            math.degrees(self.heading_hard_limit),
            max(
                math.degrees(self.successor_smooth_heading_limit),
                float(gp("~successor_curve_heading_limit_deg", 65.0)),
            ),
        ))
        # TEB normally owns route turns itself. The retired connector path is
        # retained solely for controlled comparisons: it inserts a separate
        # MoveBase action and an external velocity owner, which creates an
        # action boundary and can make the frontier watchdog see no
        # translational progress during a valid rotation.
        self.turn_execution_mode = str(
            gp("~turn_execution_mode", "native_teb")
        ).strip().lower()
        if self.turn_execution_mode not in ("native_teb", "legacy_connector"):
            rospy.logwarn(
                "Global frontier invalid turn_execution_mode=%r; using native_teb",
                self.turn_execution_mode,
            )
            self.turn_execution_mode = "native_teb"
        # Used only by the legacy connector comparison mode.
        self.explicit_turn_connector_threshold = math.radians(max(
            90.0,
            min(
                180.0,
                float(gp("~explicit_turn_connector_threshold_deg", 135.0)),
            ),
        ))
        # This must exceed move_base's 0.35 m XY terminal tolerance. The
        # connector is sampled from the already validated BFS/Navfn route, so
        # it exists solely to keep the action active for the supervised yaw
        # phase and is never an arbitrary free-space waypoint.
        self.turn_connector_distance = max(
            0.45, float(gp("~turn_connector_distance", 0.50))
        )
        # A confirmed target can be visible beyond a mapped wall.  Its visual
        # ray is not an executable goal in that case, but it is useful evidence
        # for choosing the next *reachable* information boundary.  This is a
        # soft score term applied only to the one replan requested by Goal
        # Manager after Navfn rejects that ray; ordinary coverage is unchanged.
        self.semantic_hint_weight = max(
            0.0, float(gp("~semantic_hint_weight", 0.35))
        )
        self.semantic_hint_max_distance = max(
            0.5, float(gp("~semantic_hint_max_distance", 12.0))
        )
        # A turn connector is complete when its route tangent is inside the
        # same orientation tolerance used by TEB's action goal. This is a
        # route-state contract, not a velocity/controller tuning knob.
        self.turn_yaw_tolerance = 0.35
        # Full map/costmap BFS is useful near a frontier, but it is not needed
        # while the active endpoint is metres away and its route is healthy.
        # Throttling that work leaves more CPU headroom for Gazebo, SLAM and
        # move_base without changing the committed goal.
        self.planning_period = max(0.2, float(gp("~planning_period", 1.5)))
