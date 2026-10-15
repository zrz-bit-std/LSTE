"""Keep the GoalManager module split visible and installable."""

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


class GoalManagerModuleLayoutTest(unittest.TestCase):
    def test_detection_state_machine_has_stage_boundaries(self):
        source = (SCRIPTS / "goal_manager_detection.py").read_text(encoding="utf-8")
        for name in (
            "handle_detections",
            "_record_candidate",
            "_update_target_heading",
            "_apply_confirmation_policy",
        ):
            self.assertIn("def %s(" % name, source)

    def test_route_validation_is_a_separate_module(self):
        source = (SCRIPTS / "goal_manager_route_validation.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("def validate_target_route(", source)

    def test_projection_helpers_are_a_separate_module(self):
        source = (SCRIPTS / "goal_manager_projection.py").read_text(encoding="utf-8")
        for name in ("det_heading_world", "clip_distance", "transform_point", "lookup_transform"):
            self.assertIn("def %s(" % name, source)

    def test_ros_callbacks_separate_inputs_from_teb_transactions(self):
        """Keep ROS input updates apart from TEB action-result state changes."""
        facade = (SCRIPTS / "goal_manager_ros_callbacks.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GoalManagerRosCallbacksMixin", facade)
        self.assertLessEqual(len(facade.splitlines()), 50)
        for filename, mixin, methods in (
            (
                "goal_manager_input_callbacks.py",
                "GoalManagerInputCallbacksMixin",
                ("on_state", "on_dets", "on_task", "on_fixed_goal_click"),
            ),
            (
                "goal_manager_teb_callbacks.py",
                "GoalManagerTebCallbacksMixin",
                ("on_teb_goal_terminal", "on_teb_goal_failure"),
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            for method in methods:
                self.assertIn("def %s(" % method, source)

    def test_parameter_loading_is_a_separate_module(self):
        source = (SCRIPTS / "goal_manager_config.py").read_text(encoding="utf-8")
        for name in ("_load_parameters", "_load_target_parameters", "_load_navigation_parameters", "_load_scan_parameters"):
            self.assertIn("def %s(" % name, source)
        self.assertLessEqual(len(source.splitlines()), 100)
        for filename, mixin, method in (
            (
                "goal_manager_config_target_tracking.py",
                "GoalManagerTargetTrackingConfigMixin",
                "_load_target_tracking_parameters",
            ),
            (
                "goal_manager_config_target_routing.py",
                "GoalManagerTargetRoutingConfigMixin",
                "_load_target_routing_parameters",
            ),
            (
                "goal_manager_config_target_completion.py",
                "GoalManagerTargetCompletionConfigMixin",
                "_load_target_completion_parameters",
            ),
            (
                "goal_manager_config_navigation.py",
                "GoalManagerNavigationConfigMixin",
                "_load_frontier_integration_parameters",
            ),
        ):
            module = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, module)
            self.assertIn("def %s(" % method, module)

    def test_viewpoint_selection_is_a_separate_module(self):
        source = (SCRIPTS / "goal_manager_viewpoint.py").read_text(encoding="utf-8")
        self.assertIn("class TargetViewpointSelection", source)
        self.assertIn("def select_target_viewpoint(", source)

    def test_extracted_goal_modules_import_their_own_runtime_symbols(self):
        """Prevent missing imports that only fail after a target is detected."""
        import goal_manager_goal_arbitration as arbitration
        import goal_manager_target_follow as follow
        import goal_manager_target_segments as segments

        self.assertTrue(callable(segments.select_target_viewpoint))
        self.assertTrue(callable(segments.wrap_angle))
        self.assertTrue(callable(arbitration.quaternion_from_euler))
        self.assertTrue(callable(arbitration.wrap_angle))
        self.assertEqual(arbitration.STATE_LOCKED, 2)
        self.assertTrue(callable(follow.String))

    def test_target_follow_policy_is_a_separate_module(self):
        source = (SCRIPTS / "goal_manager_target_follow.py").read_text(encoding="utf-8")
        self.assertIn("class GoalManagerTargetFollowMixin", source)
        self.assertIn("def goal_from_target_follow(", source)
        # The entry point coordinates lifecycle stages only. Keeping terminal
        # observation and recovery in named helpers prevents the function from
        # growing into an unreviewable state machine again.
        for name in (
            "_resolve_target_follow_preconditions",
            "_start_confirmed_target_segment",
            "_follow_active_target_segment",
            "_handle_target_segment_terminal",
            "_handle_blocked_terminal_target_route",
            "_hold_close_target_confirmation",
        ):
            self.assertIn("def %s(" % name, source)

    def test_target_observation_ownership_gate_is_ros_free_and_installable(self):
        source = (SCRIPTS / "goal_manager_target_observation_gate.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class TargetObservationGate", source)
        self.assertNotIn("import rospy", source)
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn(
            "scripts/goal_manager_target_observation_gate.py", cmake
        )

    def test_target_segments_are_split_by_lifecycle_responsibility(self):
        """Keep route policy, commit mutation, and recovery independently small."""
        facade = (SCRIPTS / "goal_manager_target_segments.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GoalManagerTargetSegmentsMixin", facade)
        self.assertLessEqual(len(facade.splitlines()), 40)
        for filename, mixin, methods in (
            (
                "goal_manager_target_route_continuity.py",
                "GoalManagerTargetRouteContinuityMixin",
                (
                    "target_route_continuity",
                    "should_defer_sharp_target_takeover",
                    "can_prepare_target_continuous_handoff",
                ),
            ),
            (
                "goal_manager_target_segment_commit.py",
                "GoalManagerTargetSegmentCommitMixin",
                ("commit_target_segment", "_publish_target_segment_commit"),
            ),
            (
                "goal_manager_target_recovery.py",
                "GoalManagerTargetRecoveryMixin",
                ("target_semantic_hint_map", "release_target_follow_to_frontier"),
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            for method in methods:
                self.assertIn("def %s(" % method, source)

    def test_place_memory_lifecycle_is_split_by_evidence_type(self):
        """Keep topology facts apart from coverage and state transitions."""
        facade = (SCRIPTS / "global_frontier_place_memory_lifecycle.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class FrontierRegionLifecycleMixin", facade)
        self.assertLessEqual(len(facade.splitlines()), 35)
        for filename, mixin, methods in (
            (
                "global_frontier_place_memory_portals.py",
                "FrontierRegionPortalMixin",
                ("record_entry_portal", "enter"),
            ),
            (
                "global_frontier_place_memory_observation.py",
                "FrontierRegionObservationMixin",
                ("observe", "stagnant", "endpoint_observed"),
            ),
            (
                "global_frontier_place_memory_state.py",
                "FrontierRegionStateMixin",
                ("activate", "close", "dormant"),
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            for method in methods:
                self.assertIn("def %s(" % method, source)

    def test_frontier_route_ownership_is_a_separate_module(self):
        source = (SCRIPTS / "goal_manager_frontier.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GoalManagerFrontierMixin", source)
        for name in (
            "on_global_frontier",
            "on_global_frontier_command",
            "on_global_frontier_status",
            "request_global_frontier_replan",
        ):
            self.assertIn("def %s(" % name, source)

        main_source = (SCRIPTS / "lste_goal_manager.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("GoalManagerFrontierMixin", main_source)
        self.assertNotIn("    def on_global_frontier(", main_source)
        self.assertNotIn("    def request_global_frontier_replan(", main_source)

    def test_target_utilities_are_a_separate_module(self):
        source = (SCRIPTS / "goal_manager_target_utils.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GoalManagerTargetUtilsMixin", source)
        for name in (
            "goal_intent_priority",
            "target_detection_for_track",
            "pick_ctx_pair",
            "log_goal_diagnostic",
        ):
            self.assertIn("def %s(" % name, source)
        main_source = (SCRIPTS / "lste_goal_manager.py").read_text(encoding="utf-8")
        self.assertIn("GoalManagerTargetUtilsMixin", main_source)
        self.assertNotIn("def target_detection_for_track(", main_source)

    def test_target_identity_policy_is_ros_free_and_installable(self):
        source = (SCRIPTS / "target_track_identity.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("def labels_compatible(", source)
        self.assertIn("class TargetTrackIdentity", source)
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/target_track_identity.py", cmake)

    def test_goal_manager_entrypoint_only_composes_modules(self):
        source = (SCRIPTS / "lste_goal_manager.py").read_text(encoding="utf-8")
        self.assertIn("GoalManagerRosCallbacksMixin", source)
        self.assertIn("GoalManagerSchedulingMixin", source)
        self.assertIn("GoalManagerTargetSegmentsMixin", source)
        self.assertIn("GoalManagerTargetCompletionMixin", source)
        self.assertIn("GoalManagerGoalOutputMixin", source)
        self.assertNotIn("    def on_teb_goal_failure(", source)
        self.assertNotIn("    def publish_goal(", source)

    def test_runtime_state_imports_the_mode_it_initializes(self):
        source = (SCRIPTS / "goal_manager_runtime_state.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("EXPLORE_PASS_MODE", source)
        self.assertIn("from goal_manager_modes import EXPLORE_PASS_MODE, STATE_PASS", source)

    def test_cmake_installs_extracted_modules(self):
        source = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for name in (
            "scripts/goal_manager_detection.py",
            "scripts/target_track_identity.py",
            "scripts/goal_manager_route_validation.py",
            "scripts/goal_manager_projection.py",
            "scripts/goal_manager_config.py",
            "scripts/goal_manager_config_navigation.py",
            "scripts/goal_manager_config_target_completion.py",
            "scripts/goal_manager_config_target_routing.py",
            "scripts/goal_manager_config_target_tracking.py",
            "scripts/goal_manager_goal_arbitration.py",
            "scripts/goal_manager_goal_output.py",
            "scripts/goal_manager_legacy_goals.py",
            "scripts/goal_manager_modes.py",
            "scripts/goal_manager_input_callbacks.py",
            "scripts/goal_manager_ros_callbacks.py",
            "scripts/goal_manager_teb_callbacks.py",
            "scripts/goal_manager_ros_interfaces.py",
            "scripts/goal_manager_runtime_state.py",
            "scripts/goal_manager_scheduling.py",
            "scripts/goal_manager_target_completion.py",
            "scripts/goal_manager_target_segments.py",
            "scripts/goal_manager_target_route_continuity.py",
            "scripts/goal_manager_target_segment_commit.py",
            "scripts/goal_manager_target_recovery.py",
            "scripts/global_frontier_place_memory_state.py",
            "scripts/global_frontier_place_memory_portals.py",
            "scripts/global_frontier_place_memory_observation.py",
            "scripts/goal_manager_frontier.py",
            "scripts/goal_manager_viewpoint.py",
            "scripts/goal_manager_target_utils.py",
            "scripts/navigation_metrics_goal_events.py",
            "scripts/navigation_metrics_logging.py",
            "scripts/navigation_metrics_callbacks.py",
            "scripts/navigation_metrics_observer_state.py",
            "scripts/navigation_metrics_pose.py",
            "scripts/navigation_metrics_ros_interfaces.py",
            "scripts/navigation_metrics_runtime.py",
            "scripts/navigation_metrics_setup.py",
            "scripts/navigation_metrics_snapshot.py",
            "scripts/navigation_metrics_target_state.py",
            "scripts/teb_goal_bridge_parameters.py",
            "scripts/teb_goal_bridge_ros.py",
            "scripts/teb_goal_bridge_state.py",
            "scripts/teb_goal_bridge_status.py",
        ):
            self.assertIn(name, source)

    def test_navigation_metrics_target_evaluation_is_a_separate_module(self):
        facade = (SCRIPTS / "navigation_metrics_target_eval.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class NavigationMetricsTargetEvaluationMixin", facade)
        self.assertLessEqual(len(facade.splitlines()), 50)
        for filename, mixin, method in (
            (
                "navigation_metrics_target_eval_configuration.py",
                "NavigationMetricsTargetEvaluationConfigurationMixin",
                "_target_eval_configuration",
            ),
            (
                "navigation_metrics_target_eval_projection.py",
                "NavigationMetricsTargetEvaluationProjectionMixin",
                "_target_eval_projected_box_locked",
            ),
            (
                "navigation_metrics_target_eval_depth.py",
                "NavigationMetricsTargetEvaluationDepthMixin",
                "_target_eval_depth_visibility_locked",
            ),
            (
                "navigation_metrics_target_eval_episodes.py",
                "NavigationMetricsTargetEvaluationEpisodesMixin",
                "_target_eval_snapshot_locked",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, facade)

        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsTargetEvaluationMixin", main_source)
        self.assertNotIn("def _target_eval_projected_box_locked(", main_source)

    def test_navigation_metrics_detection_is_a_separate_module(self):
        source = (SCRIPTS / "navigation_metrics_detection.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class NavigationMetricsDetectionMixin", source)
        self.assertIn("def _evaluate_target_detection_locked(", source)
        self.assertIn("def on_detections(", source)
        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("def _evaluate_target_detection_locked(", main_source)

    def test_navigation_metrics_goal_events_are_a_separate_module(self):
        source = (SCRIPTS / "navigation_metrics_goal_events.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class NavigationMetricsGoalEventsMixin", source)
        for name in ("on_goal", "on_mission_goal"):
            self.assertIn("def %s(" % name, source)

        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsGoalEventsMixin", main_source)
        self.assertNotIn("    def on_goal(", main_source)
        self.assertNotIn("    def on_mission_goal(", main_source)

    def test_navigation_metrics_pose_and_logging_are_separate_modules(self):
        pose_source = (SCRIPTS / "navigation_metrics_pose.py").read_text(
            encoding="utf-8"
        )
        logging_source = (SCRIPTS / "navigation_metrics_logging.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class NavigationMetricsPoseMixin", pose_source)
        self.assertIn("def _pose_xy_in_frame_locked(", pose_source)
        self.assertIn("def on_gazebo_model_states(", pose_source)
        self.assertIn("class NavigationMetricsLoggingMixin", logging_source)
        self.assertIn("def _record_target_lifecycle_locked(", logging_source)

        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsPoseMixin", main_source)
        self.assertIn("NavigationMetricsLoggingMixin", main_source)
        self.assertNotIn("    def _pose_xy_in_frame_locked(", main_source)
        self.assertNotIn("    def _record_target_lifecycle_locked(", main_source)

    def test_navigation_metrics_runtime_is_a_separate_module(self):
        source = (SCRIPTS / "navigation_metrics_runtime.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class NavigationMetricsRuntimeMixin", source)
        for name in (
            "_resolve_path",
            "_resolved_startup_params",
            "_run_start_context",
            "_create_run_dir",
            "_rotate_point",
        ):
            self.assertIn("def %s(" % name, source)
        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsRuntimeMixin", main_source)
        self.assertNotIn("def _run_start_context(", main_source)

    def test_navigation_metrics_entrypoint_only_composes_observers(self):
        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        for name in (
            "NavigationMetricsSetupMixin",
            "NavigationMetricsTargetStateMixin",
            "NavigationMetricsObserverStateMixin",
            "NavigationMetricsRosInterfacesMixin",
            "NavigationMetricsCallbacksMixin",
            "NavigationMetricsSnapshotMixin",
        ):
            self.assertIn(name, main_source)
        self.assertLessEqual(len(main_source.splitlines()), 100)
        for method in (
            "_initialize_navigation_observer_state",
            "_setup_ros_interfaces",
            "on_goal_arbitration",
            "_snapshot",
        ):
            self.assertNotIn("    def %s(" % method, main_source)

    def test_navigation_metrics_state_groups_are_explicit(self):
        source = (SCRIPTS / "navigation_metrics_observer_state.py").read_text(
            encoding="utf-8"
        )
        for name in (
            "_initialize_navigation_observer_state",
            "_initialize_control_and_planner_observer_state",
            "_initialize_goal_lifecycle_observer_state",
            "_initialize_smoothness_observer_state",
        ):
            self.assertIn("def %s(" % name, source)

    def test_navigation_metrics_planner_observation_is_a_separate_module(self):
        facade = (SCRIPTS / "navigation_metrics_planner.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin, name in (
            (
                "navigation_metrics_planner_teb_feedback.py",
                "NavigationMetricsTebFeedbackMixin",
                "on_teb_feedback",
            ),
            (
                "navigation_metrics_planner_move_base.py",
                "NavigationMetricsMoveBaseMixin",
                "on_recovery",
            ),
            (
                "navigation_metrics_planner_paths.py",
                "NavigationMetricsPathMixin",
                "_path_straightness_locked",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % name, source)
            self.assertIn(mixin, facade)
        self.assertLessEqual(len(facade.splitlines()), 35)

        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsPlannerMixin", main_source)
        self.assertNotIn("def _path_straightness_locked(", main_source)

    def test_navigation_metrics_action_lifecycle_is_a_separate_module(self):
        source = (SCRIPTS / "navigation_metrics_action_lifecycle.py").read_text(
            encoding="utf-8"
        )
        for name in (
            "on_turn_supervisor_status",
            "on_status",
            "_record_terminal_to_dispatch_locked",
            "_recent_lifecycle_event",
        ):
            self.assertIn("def %s(" % name, source)

        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsActionLifecycleMixin", main_source)
        self.assertNotIn("    def on_turn_supervisor_status(", main_source)
        self.assertNotIn("    def on_status(", main_source)

    def test_navigation_metrics_execution_events_are_a_separate_module(self):
        source = (SCRIPTS / "navigation_metrics_execution_events.py").read_text(
            encoding="utf-8"
        )
        for name in (
            "on_scan",
            "on_bridge_status",
            "on_global_frontier_status",
            "on_controller_status",
        ):
            self.assertIn("def %s(" % name, source)

        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsExecutionEventsMixin", main_source)
        self.assertNotIn("    def on_bridge_status(", main_source)
        self.assertNotIn("    def on_controller_status(", main_source)

    def test_navigation_metrics_command_quality_is_a_separate_module(self):
        facade = (SCRIPTS / "navigation_metrics_command_quality.py").read_text(
            encoding="utf-8"
        )
        events = (SCRIPTS / "navigation_metrics_command_events.py").read_text(
            encoding="utf-8"
        )
        stream = (SCRIPTS / "navigation_metrics_command_stream.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsCommandEventsMixin", facade)
        self.assertIn("NavigationMetricsCommandStreamMixin", facade)
        self.assertIn("def on_teb_cmd(", events)
        self.assertIn("def _command_discontinuity_reason_locked(", events)
        self.assertIn("def on_cmd(", stream)
        self.assertIn("def on_cmd_vel_mux_status(", stream)

        main_source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("NavigationMetricsCommandQualityMixin", main_source)
        self.assertNotIn("    def on_cmd_vel_mux_status(", main_source)

    def test_teb_bridge_action_health_is_a_separate_module(self):
        source = (SCRIPTS / "teb_goal_bridge_action_health.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class TebGoalBridgeActionHealthMixin", source)
        for name in (
            "cancel_locked",
            "_pending_goal_delta_locked",
            "_latch_target_failure_locked",
        ):
            self.assertIn("def %s(" % name, source)
        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("def _latch_target_failure_locked(", main_source)

    def test_teb_bridge_mission_lifecycle_is_a_separate_module(self):
        facade = (SCRIPTS / "teb_goal_bridge_mission_lifecycle.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class TebGoalBridgeMissionLifecycleMixin", facade)
        components = {
            "teb_goal_bridge_mission_input.py": "on_goal_command",
            "teb_goal_bridge_mission_runtime.py": "_adopt_persistent_mission_goal_locked",
            "teb_goal_bridge_mission_control.py": "on_task_done",
            "teb_goal_bridge_persistent_target_result.py": "on_persistent_target_plan_result",
        }
        for filename, method in components.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("def %s(" % method, source)

        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("TebGoalBridgeMissionLifecycleMixin", main_source)
        self.assertNotIn("    def on_goal_command(", main_source)
        self.assertNotIn("    def on_persistent_target_plan_result(", main_source)

    def test_teb_bridge_persistent_lifecycles_are_separate_modules(self):
        expected = {
            "teb_goal_bridge_persistent_frontier_endpoint.py": (
                "TebGoalBridgePersistentFrontierEndpointMixin",
                "on_persistent_frontier_endpoint_reached",
            ),
            "teb_goal_bridge_persistent_target_approach.py": (
                "TebGoalBridgePersistentTargetApproachMixin",
                "_handle_persistent_target_approach_locked",
            ),
            "teb_goal_bridge_persistent_frontier_handoff.py": (
                "TebGoalBridgePersistentFrontierHandoffMixin",
                "_start_frontier_continuous_prefetch_handoff_locked",
            ),
            "teb_goal_bridge_persistent_execution.py": (
                "TebGoalBridgePersistentExecutionMixin",
                "_complete_frontier_observation_locked",
            ),
        }
        for filename, (mixin, method) in expected.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)

        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        for mixin, _method in expected.values():
            self.assertIn(mixin, main_source)
        self.assertNotIn("    def on_persistent_frontier_endpoint_reached(", main_source)

    def test_teb_bridge_installs_every_persistent_lifecycle_module(self):
        cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text(encoding="utf-8")
        for filename in (
            "teb_goal_bridge_persistent_execution.py",
            "teb_goal_bridge_persistent_frontier_endpoint.py",
            "teb_goal_bridge_persistent_frontier_handoff.py",
            "teb_goal_bridge_persistent_target_approach.py",
        ):
            self.assertIn("scripts/%s" % filename, cmake)

    def test_teb_bridge_handoff_policy_is_a_separate_module(self):
        facade = (SCRIPTS / "teb_goal_bridge_handoff.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class TebGoalBridgeHandoffMixin", facade)
        components = {
            "teb_goal_bridge_handoff_stall.py": "maybe_handoff_locked",
            "teb_goal_bridge_handoff_segment.py": "maybe_segment_handoff_locked",
        }
        for filename, method in components.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("def %s(" % method, source)

        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("TebGoalBridgeHandoffMixin", main_source)
        self.assertNotIn("    def maybe_handoff_locked(", main_source)

    def test_teb_bridge_action_client_stages_are_separate_modules(self):
        direct_components = {
            "teb_goal_bridge_action_dispatch.py": (
                "TebGoalBridgeActionDispatchMixin",
                "dispatch_locked",
            ),
            "teb_goal_bridge_prefetch_admission.py": (
                "TebGoalBridgePrefetchAdmissionMixin",
                "_admit_persistent_frontier_prefetch_locked",
            ),
        }
        nested_components = {
            "teb_goal_bridge_action_active_dispatch.py": (
                "TebGoalBridgeActionActiveDispatchMixin",
                "_dispatch_active_action_locked",
            ),
            "teb_goal_bridge_action_retry_policy.py": (
                "TebGoalBridgeActionRetryPolicyMixin",
                "_dispatch_inactive_action_locked",
            ),
            "teb_goal_bridge_action_client.py": (
                "TebGoalBridgeActionClientMixin",
                "_send_goal_locked",
            ),
            "teb_goal_bridge_action_terminal.py": (
                "TebGoalBridgeActionTerminalMixin",
                "on_done",
            ),
            "teb_goal_bridge_action_feedback.py": (
                "TebGoalBridgeActionFeedbackMixin",
                "on_feedback",
            ),
        }
        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        for filename, (mixin, method) in direct_components.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, main_source)
        for filename, (mixin, method) in nested_components.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
        dispatch_source = (SCRIPTS / "teb_goal_bridge_action_dispatch.py").read_text(
            encoding="utf-8"
        )
        for mixin, _method in (
            nested_components["teb_goal_bridge_action_active_dispatch.py"],
            nested_components["teb_goal_bridge_action_retry_policy.py"],
            nested_components["teb_goal_bridge_action_client.py"],
        ):
            self.assertIn(mixin, dispatch_source)
        callbacks_source = (SCRIPTS / "teb_goal_bridge_action_callbacks.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class TebGoalBridgeActionCallbacksMixin", callbacks_source)
        self.assertIn("TebGoalBridgeActionTerminalMixin", callbacks_source)
        self.assertIn("TebGoalBridgeActionFeedbackMixin", callbacks_source)
        self.assertNotIn("    def dispatch_locked(", main_source)
        self.assertNotIn("    def on_done(", main_source)

    def test_teb_bridge_action_helpers_are_installed(self):
        cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text(encoding="utf-8")
        for filename in (
            "teb_goal_bridge_action_active_dispatch.py",
            "teb_goal_bridge_action_client.py",
            "teb_goal_bridge_action_feedback.py",
            "teb_goal_bridge_action_retry_policy.py",
            "teb_goal_bridge_action_terminal.py",
        ):
            self.assertIn("scripts/%s" % filename, cmake)

    def test_teb_bridge_keeps_filesystem_and_ros_path_types_distinct(self):
        # ``pathlib.Path`` is needed while the catkin relay is importing
        # sibling helpers; ``nav_msgs.msg.Path`` is used only by the ROS
        # wiring module. Keeping them in separate files makes the former
        # import-name collision impossible at the node entry point.
        bridge_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        ros_source = (SCRIPTS / "teb_goal_bridge_ros.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("from pathlib import Path as FilePath", bridge_source)
        self.assertIn("SCRIPT_DIR = FilePath(__file__).resolve().parent", bridge_source)
        self.assertIn("from nav_msgs.msg import OccupancyGrid, Path", ros_source)

    def test_teb_bridge_setup_is_split_by_responsibility(self):
        modules = {
            "teb_goal_bridge_parameters.py": "def configure_bridge_parameters(",
            "teb_goal_bridge_state.py": "def initialize_bridge_state(",
            "teb_goal_bridge_ros.py": "def connect_bridge_ros(",
            "teb_goal_bridge_status.py": "class TebGoalBridgeStatusMixin",
        }
        for filename, contract in modules.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn(contract, source)

        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        for imported_name in (
            "configure_bridge_parameters",
            "initialize_bridge_state",
            "connect_bridge_ros",
            "TebGoalBridgeStatusMixin",
        ):
            self.assertIn(imported_name, main_source)

    def test_teb_bridge_persistent_target_policy_is_a_separate_module(self):
        source = (SCRIPTS / "teb_goal_bridge_persistent_target.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class TebGoalBridgePersistentTargetMixin", source)
        for name in (
            "_install_persistent_target_locked",
            "_request_persistent_target_locked",
            "_clear_persistent_target_request_locked",
            "_publish_persistent_mission_goal_locked",
        ):
            self.assertIn("def %s(" % name, source)
        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("def _install_persistent_target_locked(", main_source)

    def test_teb_bridge_route_monitoring_is_a_separate_module(self):
        source = (SCRIPTS / "teb_goal_bridge_route_monitoring.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class TebGoalBridgeRouteMonitoringMixin", source)
        for name in (
            "_path_remaining_distance",
            "_update_navfn_path_progress_locked",
            "on_navfn_plan",
            "on_local_costmap",
        ):
            self.assertIn("def %s(" % name, source)
        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("def _path_remaining_distance(", main_source)

    def test_teb_bridge_entrypoint_delegates_geometry_and_live_callbacks(self):
        modules = {
            "teb_goal_bridge_geometry.py": (
                "TebGoalBridgeGeometryMixin",
                "_goal_in_global_frame",
            ),
            "teb_goal_bridge_intent.py": (
                "TebGoalBridgeIntentMixin",
                "on_intent",
            ),
            "teb_goal_bridge_frontier_status.py": (
                "TebGoalBridgeFrontierStatusMixin",
                "on_frontier_status",
            ),
            "teb_goal_bridge_teb_runtime.py": (
                "TebGoalBridgeTebRuntimeMixin",
                "on_teb_feedback",
            ),
        }
        for filename, (mixin, method) in modules.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)

        main_source = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(
            encoding="utf-8"
        )
        for mixin in (contract[0] for contract in modules.values()):
            self.assertIn(mixin, main_source)
        for method in (
            "_goal_in_global_frame",
            "on_intent",
            "on_frontier_status",
            "on_teb_feedback",
            "_teb_reorientation_progressing_locked",
        ):
            self.assertNotIn("    def %s(" % method, main_source)
        self.assertLessEqual(len(main_source.splitlines()), 120)

    def test_teb_bridge_install_rules_include_extracted_modules(self):
        cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text(encoding="utf-8")
        for filename in (
            "teb_goal_bridge_geometry.py",
            "teb_goal_bridge_intent.py",
            "teb_goal_bridge_frontier_status.py",
            "teb_goal_bridge_teb_runtime.py",
        ):
            self.assertIn("scripts/%s" % filename, cmake)


if __name__ == "__main__":
    unittest.main()
