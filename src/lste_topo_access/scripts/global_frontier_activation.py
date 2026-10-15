"""Frontier selection activation and route-lifecycle bookkeeping.

This mixin contains the small transaction that turns a selected candidate
into an active route. Selection and execution remain separate modules; this
file only commits the lifecycle state and emits the associated status events.
"""

import math

import rospy

from lifecycle_manager import State
from global_frontier_models import SelectedFrontier
from global_frontier_topology import copy_component_evidence
from global_frontier_portal_crossing import portal_signed_distance


class GlobalFrontierActivationMixin:
    @staticmethod
    def selected_frontier_from_cell(active_cell):
        """Convert the positional candidate tuple into named route facts."""
        return SelectedFrontier.from_candidate(active_cell)

    def prepare_selected_frontier_lifecycle(
        self, message, components, known_free, robot_map, now, selection,
        selection_mode,
    ):
        """Open the destination region and prepare any source-place departure."""
        self.place_graph_waiting_for_portal = False
        if selection.route_kind == "local_egress":
            # A recovery anchor is a previously reached pose, not a new
            # observation candidate or a transition out of a place.
            return None, None
        departed_place = None
        if self.route_crosses_place_boundary(selection.place_hops):
            departed_place = self.prepare_place_departure(
                message,
                components,
                known_free,
                robot_map,
                selection.place_hops,
            )
        if (
            departed_place is None
            and self.route_crosses_place_boundary(selection.place_hops)
        ):
            # Observation-footprint evidence can recover a temporarily
            # unlabeled doorway, but it is still an *outward* action. A local
            # endpoint outside the first viewpoint's ray is new room coverage,
            # not proof that the robot should close the current place.
            departed_place = self.prepare_observation_place_departure(
                message,
                known_free,
                selection,
            )
        if (
            departed_place is None
            and selection.route_kind == "portal_transition"
            and self.route_crosses_place_boundary(selection.place_hops)
        ):
            departed_place = self.prepare_covered_place_transit(
                robot_map, selection.place_hops,
            )
        if (
            departed_place is not None
            and selection.route_kind == "portal_transition"
        ):
            self.place_departure.require_physical_gate_crossing()
            if (
                getattr(self, "branch_first_enabled", False)
                and getattr(selection, "graph_action_reason", "")
                == "certified_portal_branch_to_unobserved_place"
            ):
                self.place_departure.suspend_local_work_on_commit()
        region = None
        if (
            selection.route_kind != "portal_transition"
            and getattr(self, "place_memory_enabled", True)
        ):
            physical_place_id = (
                getattr(self, "current_physical_place_id", None)
                if selection.place_hops == 0 else None
            )
            probe = getattr(selection, "portal_observation_probe", None)
            current_place_durable_reobserve = bool(
                physical_place_id is not None
                and (
                    probe is not None
                    or getattr(selection, "work_item_match", None)
                    == "rehydrated_work_item"
                )
            )
            if current_place_durable_reobserve:
                # A durable PortalProbe or WorkItem belongs to the current
                # physical Place even when its memory is dormant/transit-only.
                # Keep the explicit rehydration route executable without
                # reopening the Place for arbitrary frontier geometry.
                region = self.region_memory.by_id(physical_place_id)
                if region is not None:
                    region["route_dispatches"] = int(
                        region.get("route_dispatches", 0)
                    ) + 1
                    region["last_selected"] = float(now)
                    region["last_association"] = "current_physical_place_owner"
                    self.last_frontier_region_tier = "reobserve"
            else:
                region = self.activate_frontier_region(
                    selection.x,
                    selection.y,
                    selection.information,
                    now,
                    selection_mode,
                    component=selection.component,
                    physical_place_id=physical_place_id,
                )
            if region is None:
                # An immediate terminal/recovery callback can make a candidate
                # dormant after selection. Keep the active route untouched and
                # retry from fresh map evidence.
                return None
            if selection.place_hops == 0:
                self.current_physical_place_id = int(region["id"])
        elif not getattr(self, "place_memory_enabled", True):
            # Geometry baselines never acquire a hidden Place simply because
            # a frontier has been accepted by Navfn. Their durable state is
            # limited to the declared endpoint-distance memory.
                self.current_physical_place_id = None
        return region, departed_place

    def _rollback_failed_activation(self, now, reason, selection=None):
        """Undo a partially installed route before the next graph wake."""
        graph_rejection_recorded = False
        record_rejection = getattr(
            self, "_record_graph_route_activation_rejection", None
        )
        if callable(record_rejection):
            graph_rejection_recorded = bool(
                record_rejection(selection, reason=reason)
            )
        settle_probe = getattr(self, "settle_active_portal_probe", None)
        if (
            callable(settle_probe)
            and getattr(self, "active_portal_probe_id", None) is not None
        ):
            settle_probe("failed", now, reason)

        release = getattr(self, "release_active_frontier", None)
        if callable(release) and getattr(self, "active_frontier", None) is not None:
            release(discard_prefetch=True)
        else:
            self.active_portal_probe_id = None
            self.active_portal_probe_phase = ""

        clear_lease = getattr(self, "_clear_graph_route_plan_lease", None)
        if callable(clear_lease) and not graph_rejection_recorded:
            clear_lease("activation_rejected:%s" % str(reason))

        lifecycle = getattr(self, "lifecycle_manager", None)
        if (
            lifecycle is not None
            and getattr(lifecycle, "current_state", None) == State.DISPATCHED
        ):
            lifecycle.begin_transaction(State.IDLE, now=now)

    def initialize_active_frontier_route(
        self, selection, region, now, robot_map, probe_record=None,
    ):
        """Install the selected candidate as a new route lifecycle lease."""
        lifecycle = getattr(self, "lifecycle_manager", None)
        if lifecycle is not None:
            lifecycle.begin_transaction(State.DISPATCHED, now=now)
            latest = getattr(lifecycle, "latest_transaction_id", None)
            self.active_route_lifecycle_transaction_id = int(
                latest() if callable(latest) else lifecycle.current_transaction_id
            )
        else:
            self.active_route_lifecycle_transaction_id = 0
        self.frontier_exhausted = False
        self.active_frontier = (
            selection.row,
            selection.col,
            selection.x,
            selection.y,
        )
        self.active_frontier_component = copy_component_evidence(selection.component)
        portal_gate_xy = getattr(selection, "portal_gate_xy", None)
        self.active_portal_gate_xy = (
            None
            if portal_gate_xy is None
            else (
                float(portal_gate_xy[0]),
                float(portal_gate_xy[1]),
            )
        )
        map_message = getattr(self, "map_msg", None)
        map_header = None if map_message is None else getattr(map_message, "header", None)
        map_frame = getattr(map_header, "frame_id", "map") or "map"
        transform_xy = getattr(self, "transform_xy", None)
        self.active_portal_gate_odom_xy = (
            None
            if self.active_portal_gate_xy is None or transform_xy is None
            else transform_xy(
                "odom",
                map_frame,
                self.active_portal_gate_xy[0],
                self.active_portal_gate_xy[1],
            )
        )
        self.active_portal_destination_odom_xy = (
            None
            if self.active_portal_gate_xy is None or transform_xy is None
            else transform_xy(
                "odom",
                map_frame,
                float(selection.x),
                float(selection.y),
            )
        )
        self.active_portal_gate_approached_at = None
        self.active_portal_crossing_observed = False
        self.active_portal_crossing_preobserved = bool(
            getattr(selection, "portal_crossing_preobserved", False)
        )
        self.active_portal_crossing_rejected = False
        self.active_frontier_region_id = (
            None if region is None else int(region["id"])
        )
        # Route kind describes how the controller executes a goal. Place hops
        # describe the stronger topological fact: whether that goal crosses a
        # certified room boundary. A normal frontier endpoint can do both.
        self.active_place_hops = getattr(selection, "place_hops", None)
        if selection.route_kind == "local_egress":
            if not self.active_local_egress_resumes_portal:
                self.local_egress_place_lease.activate()
        self.active_observation_session_started_at = None
        if not self.place_departure.active:
            self._clear_active_place_departure()
        self.active_route_id += 1
        pending_predecessor = getattr(
            self, "pending_graph_successor_predecessor_contract", None
        )
        if isinstance(pending_predecessor, dict):
            try:
                pending_predecessor_route_id = int(
                    pending_predecessor.get("route_id", 0) or 0
                )
            except (TypeError, ValueError):
                pending_predecessor_route_id = 0
            if pending_predecessor_route_id >= int(self.active_route_id):
                pending_predecessor = None
                pending_predecessor_route_id = 0
        else:
            pending_predecessor_route_id = 0
        if pending_predecessor is not None:
            pending_predecessor = dict(pending_predecessor)
            pending_predecessor_route_id = int(
                pending_predecessor.get("route_id", 0) or 0
            )
        self.pending_graph_successor_predecessor_contract = None
        graph_transaction = getattr(
            self, "graph_route_action_transaction", None
        )
        route_epoch = (
            getattr(graph_transaction, "map_epoch", None)
            if graph_transaction is not None
            else getattr(self, "last_map_epoch", None)
        )
        try:
            self.active_route_map_epoch = max(1, int(route_epoch or 0))
        except (TypeError, ValueError):
            self.active_route_map_epoch = max(
                1, int(getattr(self, "topology_component_epoch", 1) or 1)
            )
        # A strictly newer graph route supersedes any delayed terminal from
        # the previous controller lease.  Clearing the tombstone here keeps
        # late callbacks from being attributed to the new route.
        if int(getattr(self, "last_released_route_id", 0) or 0) < int(
            self.active_route_id
        ):
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
        self.active_transition_kind = "initial"
        self.active_predecessor_route_id = pending_predecessor_route_id
        self.active_transition_distance = None
        self.recovery_pending_route_id = 0
        self.recovery_pending_behavior = ""
        self.recovery_pending_reason = ""
        self.active_since = now
        self.active_best_distance = math.hypot(
            selection.x - robot_map[0], selection.y - robot_map[1]
        )
        self.active_best_goal_distance = self.active_best_distance
        self.active_best_path_distance = float(selection.path_distance)
        self.active_progress_time = now
        self.active_last_progress_signal = "route_and_goal_initialized"
        self.active_last_robot_xy = (robot_map[0], robot_map[1])
        self.begin_active_route_history(robot_map)
        self.active_start_odom_xy = (
            float(self.pose_odom.x), float(self.pose_odom.y)
        )
        self.active_best_detour_odom_distance = 0.0
        self._reset_active_odom_coverage()
        self.active_unreachable_since = None
        self.active_last_waypoint_map = None
        self.active_route_kind = selection.route_kind
        self.active_mission_route_kind = selection.route_kind
        self.active_portal_probe_phase = ""
        self.active_portal_retry = False
        self.active_terminal_received = False
        self.active_terminal_contract = None
        graph_transaction_id = int(
            getattr(graph_transaction, "graph_transaction_id", 0) or 0
        )
        route_origin_epoch = getattr(graph_transaction, "origin_epoch", None)
        route_projection_epoch = getattr(
            graph_transaction, "projection_epoch", None
        )
        if route_origin_epoch is None:
            route_origin_epoch = self.active_route_map_epoch
        if route_projection_epoch is None:
            route_projection_epoch = self.active_route_map_epoch
        # This record is the route-side source of truth while the asynchronous
        # Bridge adoption event is still in flight.  Do not read the mutable
        # graph transaction again during a later failure/replan callback.
        self.active_route_dispatch_contract = {
            "route_id": int(self.active_route_id),
            "transaction_id": 0,
            "lifecycle_transaction_id": int(
                self.active_route_lifecycle_transaction_id
            ),
            "action_generation": 0,
            "graph_transaction_id": graph_transaction_id,
            "map_epoch": self.active_route_map_epoch,
            "origin_epoch": route_origin_epoch,
            "projection_epoch": route_projection_epoch,
            "route_kind": str(self.active_route_kind or ""),
            "mission_route_kind": str(self.active_mission_route_kind or ""),
            "source": "global_slam_frontier",
            "priority": 0,
        }
        if pending_predecessor is not None:
            self.active_route_dispatch_contract["predecessor_contract"] = dict(
                pending_predecessor
            )
            self.publish_status(
                "frontier_successor_predecessor_bound",
                route_id=int(self.active_route_id),
                predecessor_route_id=int(pending_predecessor_route_id),
                predecessor_contract=dict(pending_predecessor),
                lifecycle_transaction_id=int(
                    self.active_route_lifecycle_transaction_id
                ),
                graph_transaction_id=graph_transaction_id,
                map_epoch=self.active_route_map_epoch,
                origin_epoch=route_origin_epoch,
                projection_epoch=route_projection_epoch,
                reason="graph_materialization_failure_successor",
            )
        start_probe = getattr(self, "start_active_portal_probe", None)
        if start_probe is not None and probe_record is None:
            probe_record = start_probe(selection, now)
        if (
            getattr(selection, "portal_observation_probe", None) is not None
            and probe_record is None
        ):
            self.publish_status(
                "portal_probe_route_rejected",
                route_id=int(self.active_route_id),
                reason="durable_probe_lease_unavailable",
            )
            return False
        if probe_record is not None or getattr(
            selection, "portal_observation_probe", None
        ) is not None:
            # The controller still receives a normal map endpoint. The
            # mission contract is an explicit two-phase evidence action and
            # must survive the GoalManager/TEB bridge boundary.
            self.active_mission_route_kind = "portal_probe"
            self.active_portal_probe_phase = str(
                (probe_record or {}).get("active_phase")
                or getattr(
                    getattr(selection, "portal_observation_probe", None),
                    "phase",
                    "source",
                )
                or "source"
            ).strip().lower()
        self.turn_connector_released = True
        self.last_status_command_map = None
        self.last_status_command_yaw = None
        self.last_status_mission_map = None

    def publish_selected_frontier_status(
        self, message, route_steps, seed, robot_map, robot_yaw_map, selection,
        region, departed_place, selection_mode,
    ):
        """Emit the complete selection event without modifying route state."""
        heading_delta = self._candidate_route_heading_delta(
            message,
            route_steps,
            seed,
            selection.row,
            selection.col,
            robot_map,
            robot_yaw_map,
        )
        rospy.loginfo(
            "Global frontier selected mode=%s map=(%.2f,%.2f) path=%.2fm "
            "information=%.0f structure=%.0f score=%.2f "
            "route_heading_delta=%.1fdeg",
            selection_mode,
            selection.x,
            selection.y,
            selection.path_distance,
            selection.information,
            selection.structure,
            selection.score,
            math.degrees(heading_delta)
            if heading_delta is not None else float("nan"),
        )
        self.publish_status(
            "route_selected",
            exploration_method=getattr(
                self, "exploration_method", "place_portal_workitem",
            ),
            frontier_action_policy=getattr(
                self, "frontier_action_policy", "legacy_scalar",
            ),
            graph_route_planner=bool(
                getattr(self, "graph_route_planner_enabled", False)
            ),
            frontier_decision=getattr(self, "last_frontier_decision", None),
            selection_mode=selection_mode,
            goal=[round(float(selection.x), 3), round(float(selection.y), 3)],
            path_distance=round(float(selection.path_distance), 3),
            goal_distance=round(float(self.active_best_goal_distance), 3),
            information=round(float(selection.information), 3),
            structure=round(float(selection.structure), 3),
            route_kind=selection.route_kind,
            branch_first=bool(getattr(self, "branch_first_enabled", False)),
            graph_action=getattr(selection, "graph_action", None),
            graph_action_reason=getattr(selection, "graph_action_reason", ""),
            region_id=(None if region is None else int(region["id"])),
            region_tier=self.last_frontier_region_tier,
            region_visits=(None if region is None else int(region["visits"])),
            region_route_dispatches=(
                None
                if region is None
                else int(region.get("route_dispatches", 0))
            ),
            region_physical_entry_count=(
                None
                if region is None
                else int(region.get("physical_entry_count", 0))
            ),
            region_observation_sessions=(
                None
                if region is None
                else int(region.get("observation_sessions", 0))
            ),
            region_component_cells=(
                None if region is None or region.get("component") is None
                else int(region["component"]["cells"])
            ),
            region_association=(
                "portal_edge"
                if region is None
                else region.get("last_association", "unknown")
            ),
            departed_place_id=(
                None if departed_place is None else int(departed_place["id"])
            ),
            departed_place_state=(
                None if departed_place is None else departed_place["state"]
            ),
            visibility_covered_candidates=int(self.last_visibility_coverage_skips),
            observed_place_reentry_skips=int(
                self.last_observed_place_reentry_skips
            ),
            covered_place_transit_skips=int(self.last_covered_place_transit_skips),
            covered_place_footprint_cells=int(self.last_covered_place_footprint_cells),
            covered_place_footprint_anchors=int(self.last_covered_place_footprint_anchors),
            observation_departure_footprint_cells=int(
                self.last_observation_departure_footprint_cells
            ),
            observation_departure_anchor_count=int(
                self.last_observation_departure_anchor_count
            ),
            place_graph_hops=selection.place_hops,
            place_graph_hop_limit=int(self.place_graph_hop_limit),
            source_place_observed=bool(
                getattr(self, "last_source_place_observed", False)
            ),
            action_tier_order=list(
                getattr(self, "last_action_tier_order", ())
            ),
            place_graph_hop_skips=int(self.last_place_graph_hop_skips),
            place_graph_unresolved_candidates=int(
                self.last_place_graph_unresolved_candidates
            ),
            uncertified_place_transition_skips=int(
                self.last_uncertified_place_transition_skips
            ),
            cross_place_endpoint_deferrals=int(
                self.last_cross_place_endpoint_deferrals
            ),
            graph_policy_rejections=int(
                getattr(self, "last_graph_policy_rejections", 0)
            ),
            route_heading_delta_deg=(
                None if heading_delta is None
                else round(math.degrees(heading_delta), 2)
            ),
            semantic_hint_map=(
                None if self.pending_semantic_hint_map is None
                else [
                    round(float(self.pending_semantic_hint_map[0]), 3),
                    round(float(self.pending_semantic_hint_map[1]), 3),
                ]
            ),
            semantic_pursuit=bool(self.pending_semantic_pursuit),
            target_region_claim=bool(self.target_region_claim_active),
            target_reinspection_pending=bool(
                getattr(self, "target_reinspection_pending", False)
            ),
            target_region_claim_component=(
                None if self.target_region_claim_component is None else {
                    "epoch": int(self.target_region_claim_component["epoch"]),
                    "label": int(self.target_region_claim_component["label"]),
                }
            ),
            semantic_pursuit_forward_candidates=int(
                self.last_semantic_pursuit_forward_candidates
            ),
            target_direction_candidates=int(
                getattr(self, "last_target_direction_candidates", 0)
            ),
            portal_covered_destination_skips=int(
                self.last_portal_covered_destination_skips
            ),
            portal_covered_cycle_skips=int(
                getattr(self, "last_portal_covered_cycle_skips", 0)
            ),
            portal_destination_class=getattr(
                self, "last_portal_destination_class", None
            ),
            portal_source_side_rejections=int(
                self.last_portal_source_side_rejections
            ),
            portal_endpoint_depth_rejections=int(
                self.last_portal_endpoint_depth_rejections
            ),
            portal_reverse_egress_count=int(
                self.last_portal_reverse_egress_count
            ),
            portal_reverse_egress_region_id=(
                self.last_portal_reverse_egress_region_id
            ),
            sealed_portal_reentry_skips=int(
                self.last_sealed_portal_reentry_skips
            ),
            sealed_portal_reentry_region_id=self.last_sealed_portal_reentry_region_id,
            sealed_portal_reentry_gate=self.last_sealed_portal_reentry_gate,
            crossed_portal_support_skips=int(
                getattr(self, "last_crossed_portal_support_skips", 0)
            ),
            portal_gate=(
                None
                if getattr(selection, "portal_gate_xy", None) is None
                else [
                    round(float(selection.portal_gate_xy[0]), 3),
                    round(float(selection.portal_gate_xy[1]), 3),
                ]
            ),
            portal_gate_physical=(
                None if self.active_portal_gate_odom_xy is None else [
                    round(float(self.active_portal_gate_odom_xy[0]), 3),
                    round(float(self.active_portal_gate_odom_xy[1]), 3),
                ]
            ),
            work_item_id=getattr(selection, "work_item_id", None),
            work_item_match=getattr(selection, "work_item_match", None),
            work_item_support_cells=int(
                getattr(selection, "work_item_support_cells", 0) or 0
            ),
            work_item_created=int(getattr(self, "last_work_item_created", 0)),
            work_item_inherited=int(getattr(self, "last_work_item_inherited", 0)),
            work_item_sensor_horizon_m=(
                None
                if getattr(self, "last_work_item_sensor_horizon_m", None) is None
                else round(float(self.last_work_item_sensor_horizon_m), 3)
            ),
            work_item_place_scope_skips=int(
                getattr(self, "last_work_item_place_scope_skips", 0)
            ),
            resolved_work_item_descendant_skips=int(
                getattr(self, "last_work_item_resolved_descendant_skips", 0)
            ),
            failed_work_item_viewpoint_skips=int(
                getattr(self, "last_work_item_failed_viewpoint_skips", 0)
            ),
            alternative_work_item_viewpoints=int(
                getattr(self, "last_work_item_alternative_viewpoints", 0)
            ),
            viewpoint_retry=bool(getattr(selection, "viewpoint_retry", False)),
            portal_probe_candidates=int(
                getattr(self, "last_portal_probe_candidates", 0)
            ),
            portal_probe_viewpoint_rejections=int(
                getattr(self, "last_portal_probe_viewpoint_rejections", 0)
            ),
            structural_boundary_candidates=int(
                getattr(self, "last_structural_boundary_candidates", 0)
            ),
            structural_boundary_rejections=int(
                getattr(self, "last_structural_boundary_rejections", 0)
            ),
            portal_observation_probe=(
                None
                if getattr(selection, "portal_observation_probe", None) is None
                else {
                    "opening_cell": list(selection.portal_observation_probe.opening_cell),
                    "normal": list(selection.portal_observation_probe.normal),
                    "probe_id": getattr(
                        selection.portal_observation_probe, "probe_id", None,
                    ),
                    "observation_source": getattr(
                        selection.portal_observation_probe,
                        "observation_source",
                        "frontier",
                    ),
                }
            ),
            portal_probe_value_selection=getattr(
                self, "last_portal_probe_value_selection", None
            ),
            portal_decision=getattr(self, "last_portal_decision", None),
            target_observation_work_pending=(
                bool(
                    getattr(
                        getattr(self, "last_selection_context", None),
                        "target_observation_work_pending",
                        False,
                    )
                )
            ),
            portal_probe_ledger=self.portal_probe_report(),
        )
        if getattr(selection, "portal_observation_probe", None) is not None:
            self.publish_status(
                "portal_observation_probe_selected",
                route_id=int(self.active_route_id),
                region_id=(None if region is None else int(region["id"])),
                opening_cell=list(selection.portal_observation_probe.opening_cell),
                normal=list(selection.portal_observation_probe.normal),
                observation_source=getattr(
                    selection.portal_observation_probe,
                    "observation_source",
                    "frontier",
                ),
                work_item_id=getattr(selection, "work_item_id", None),
            )
        if selection_mode == "navfn_observation_recovery":
            self.publish_status(
                "navfn_observation_recovery_selected",
                goal=[round(float(selection.x), 3), round(float(selection.y), 3)],
                path_distance=round(float(selection.path_distance), 3),
                strict_clearance=round(float(self.clearance), 3),
                observation_clearance=round(float(self.frontier_clearance), 3),
                validation_state=self.navfn_last_validation_state,
            )

    def complete_pending_replan(self, selection):
        """Acknowledge the replan request that selected this route, if any."""
        if self.pending_replan_request_id <= 0:
            return
        self.publish_status(
            "replan_ready",
            replan_request_id=self.pending_replan_request_id,
            reason=self.pending_replan_reason,
            goal=[round(float(selection.x), 3), round(float(selection.y), 3)],
            path_distance=round(float(selection.path_distance), 3),
            semantic_hint_map=(
                None if self.pending_semantic_hint_map is None
                else [
                    round(float(self.pending_semantic_hint_map[0]), 3),
                    round(float(self.pending_semantic_hint_map[1]), 3),
                ]
            ),
            target_pursuit=bool(self.pending_semantic_pursuit),
        )
        rospy.loginfo(
            "Global frontier replan ready id=%d goal=(%.2f,%.2f)",
            self.pending_replan_request_id,
            selection.x,
            selection.y,
        )
        self.pending_replan_request_id = 0
        self.pending_replan_reason = ""
        self.pending_semantic_hint_map = None
        self.pending_semantic_pursuit = False

    def activate_selected_frontier(
        self, message, active_cell, components, known_free, now, robot_map,
        robot_yaw_map, route_steps, seed, selection_mode,
    ):
        """Commit a chosen frontier as the next active route transaction."""
        if self.active_frontier is not None:
            return True
        selection = self.selected_frontier_from_cell(active_cell)
        transaction = getattr(self, "portal_transaction", None)
        prospective_route_id = int(getattr(self, "active_route_id", 0)) + 1
        transaction_rebind = (
            transaction is not None
            and transaction.active
            and selection.route_kind == "portal_transition"
            and selection_mode == "portal_recovery_retry"
            and transaction.state in ("source_probe", "throat")
        )
        if transaction is not None and transaction.active and not (
            transaction_rebind
            or transaction.route_allowed(prospective_route_id, selection.route_kind)
        ):
            self.publish_status(
                "portal_transaction_route_rejected",
                requested_route_id=prospective_route_id,
                requested_route_kind=selection.route_kind,
                state=transaction.state,
                reason="portal_transaction_owned",
            )
            return False
        # A PortalProbe is a durable information Attempt, not merely a point
        # on the current grid. Reserve its lease before mutating Place state or
        # publishing a controller goal.  The old order allowed a failed
        # reservation to fall through as a generic frontier endpoint, which
        # replayed the same doorway forever without producing new evidence.
        probe_record = None
        if getattr(selection, "portal_observation_probe", None) is not None:
            start_probe = getattr(self, "start_active_portal_probe", None)
            if not callable(start_probe):
                self.publish_status(
                    "portal_probe_route_rejected",
                    route_id=prospective_route_id,
                    reason="durable_probe_lease_api_missing",
                )
                self._rollback_failed_activation(
                    now, "durable_probe_lease_api_missing", selection
                )
                return False
            try:
                probe_record = start_probe(
                    selection, now, route_id=prospective_route_id,
                )
            except TypeError:
                # Compatibility with narrow test doubles and older injected
                # lifecycle adapters that do not expose the optional route ID.
                probe_record = start_probe(selection, now)
            if probe_record is None:
                self.publish_status(
                    "portal_probe_route_rejected",
                    route_id=prospective_route_id,
                    reason="durable_probe_lease_unavailable",
                )
                self._rollback_failed_activation(
                    now, "durable_probe_lease_unavailable", selection
                )
                return False
        lifecycle = self.prepare_selected_frontier_lifecycle(
            message,
            components,
            known_free,
            robot_map,
            now,
            selection,
            selection_mode,
        )
        if lifecycle is None:
            self._rollback_failed_activation(
                now, "route_lifecycle_rejected", selection
            )
            return False
        region, departed_place = lifecycle
        initialized = self.initialize_active_frontier_route(
            selection,
            region,
            now,
            robot_map,
            probe_record=probe_record,
        )
        if initialized is False:
            self._rollback_failed_activation(
                now, "route_initialization_rejected", selection
            )
            return False
        if transaction is not None and selection.route_kind == "portal_transition":
            source_place_id = getattr(self.place_departure, "region_id", None)
            if source_place_id is None:
                source_place_id = getattr(self, "current_physical_place_id", None)
            if transaction_rebind:
                transaction_snapshot = transaction.bind_route(
                    self.active_route_id,
                    reason="portal_retry_route_bound",
                )
                transaction_event = "portal_transaction_rebound"
            else:
                source_signed_distance = portal_signed_distance(
                    getattr(self, "active_start_odom_xy", None),
                    getattr(self, "active_portal_gate_odom_xy", None),
                    getattr(self, "active_portal_destination_odom_xy", None),
                )
                transaction_snapshot = transaction.start(
                    self.active_route_id,
                    portal_id=getattr(self, "last_portal_hypothesis_id", None),
                    source_place_id=source_place_id,
                    gate_xy=getattr(self, "active_portal_gate_odom_xy", None),
                    destination_xy=getattr(
                        self, "active_portal_destination_odom_xy", None
                    ),
                    source_side_proven=(
                        (
                            source_signed_distance is not None
                            and source_signed_distance < 0.0
                        )
                        or (
                            getattr(
                                self,
                                "last_portal_source_side_proven_id",
                                None,
                            )
                            == getattr(
                                self,
                                "last_portal_hypothesis_id",
                                None,
                            )
                            and bool(
                                getattr(
                                    self,
                                    "last_portal_source_side_proven",
                                    False,
                                )
                            )
                        )
                    ),
                    source_signed_distance=source_signed_distance,
                    now=now,
                )
                transaction_event = "portal_transaction_started"
            if transaction_snapshot is None:
                self.publish_status(
                    "portal_transaction_route_rejected",
                    requested_route_id=int(self.active_route_id),
                    requested_route_kind=selection.route_kind,
                    state=transaction.state,
                    reason="invalid_portal_transaction_transition",
                )
                self._rollback_failed_activation(
                    now, "invalid_portal_transaction_transition", selection
                )
                return False
            self.publish_status(
                transaction_event,
                transaction_id=int(transaction_snapshot.transaction_id),
                route_id=int(transaction_snapshot.route_id),
                portal_id=transaction_snapshot.portal_id,
                source_place_id=transaction_snapshot.source_place_id,
                state=transaction_snapshot.state,
                gate=transaction_snapshot.gate_xy,
                destination=transaction_snapshot.destination_xy,
                retry_count=int(transaction_snapshot.retry_count),
            )
        dispatch = getattr(self, "dispatch_active_work_item", None)
        # Portal probes are their own physical Attempts. Their associated
        # generic WorkItem is settled by the probe ledger, so dispatching it
        # here would resurrect stale viewpoint state and keep the Place open
        # forever after a successful probe.
        if dispatch is not None and not (
            getattr(selection, "graph_action", None) == "probe_portal"
        ):
            dispatch(message, selection, region, now)
        # Only the transaction boundary knows the selection mode. Mark one
        # post-egress portal action as a retry so a second failure cannot
        # restart the same edge-recovery loop.
        self.active_portal_retry = selection_mode == "portal_recovery_retry"
        self.publish_selected_frontier_status(
            message,
            route_steps,
            seed,
            robot_map,
            robot_yaw_map,
            selection,
            region,
            departed_place,
            selection_mode,
        )
        clear_lease = getattr(self, "_clear_graph_route_plan_lease", None)
        if callable(clear_lease):
            clear_lease("route_activated")
        self.complete_pending_replan(selection)
        return True
