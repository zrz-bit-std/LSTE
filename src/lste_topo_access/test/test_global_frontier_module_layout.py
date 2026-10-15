"""Guard the global-frontier decomposition against accidental re-growth."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


class GlobalFrontierModuleLayoutTest(unittest.TestCase):
    def test_planning_cycle_has_its_own_entry_point(self):
        facade = (SCRIPTS / "global_frontier_planning_cycle.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierPlanningCycleMixin", facade)
        for filename, mixin, name in (
            (
                "global_frontier_planning_runtime.py",
                "GlobalFrontierPlanningRuntimeMixin",
                "on_timer",
            ),
            (
                "global_frontier_planning_snapshot.py",
                "GlobalFrontierPlanningSnapshotMixin",
                "build_frontier_planning_snapshot",
            ),
            (
                "global_frontier_planning_selection.py",
                "GlobalFrontierPlanningSelectionMixin",
                "select_next_active_frontier",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % name, source)
            self.assertIn(mixin, facade)

        main_source = (SCRIPTS / "lste_global_frontier_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("GlobalFrontierPlanningCycleMixin", main_source)
        self.assertNotIn("    def build_frontier_planning_snapshot(", main_source)

    def test_planning_delegates_costmap_navfn_cache_and_route_helpers(self):
        facade = (SCRIPTS / "global_frontier_planning.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin, method in (
            (
                "global_frontier_planning_costmap.py",
                "GlobalFrontierPlanningCostmapMixin",
                "cached_costmap_steps",
            ),
            (
                "global_frontier_planning_navfn.py",
                "GlobalFrontierPlanningNavfnMixin",
                "navfn_goal_reachable",
            ),
            (
                "global_frontier_planning_routes.py",
                "GlobalFrontierPlanningRouteMixin",
                "nearest_reachable_cell",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, facade)
        prefetch_facade = (SCRIPTS / "global_frontier_planning_prefetch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierPlanningPrefetchMixin", prefetch_facade)
        self.assertIn("GlobalFrontierPlanningPrefetchMixin", facade)
        for filename, mixin, method in (
            (
                "global_frontier_prefetch_selection.py",
                "GlobalFrontierPrefetchSelectionMixin",
                "prefetch_next_frontier",
            ),
            (
                "global_frontier_prefetch_validation.py",
                "GlobalFrontierPrefetchValidationMixin",
                "validated_prefetched_frontier",
            ),
            (
                "global_frontier_prefetch_activation.py",
                "GlobalFrontierPrefetchActivationMixin",
                "promote_prefetched_frontier",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, prefetch_facade)
        self.assertLessEqual(len(facade.splitlines()), 50)

    def test_configuration_delegates_topics_navigation_and_exploration(self):
        """Keep configuration domains readable without changing its facade."""
        facade = (SCRIPTS / "global_frontier_config.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin, method in (
            (
                "global_frontier_config_topics.py",
                "GlobalFrontierTopicConfigurationMixin",
                "_load_topic_parameters",
            ),
            (
                "global_frontier_config_navigation.py",
                "GlobalFrontierNavigationConfigurationMixin",
                "_load_navigation_parameters",
            ),
            (
                "global_frontier_config_exploration.py",
                "GlobalFrontierExplorationConfigurationMixin",
                "_load_exploration_parameters",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, facade)
            self.assertIn("self.%s(gp)" % method, facade)
        self.assertLessEqual(len(facade.splitlines()), 45)

    def test_place_memory_separates_association_queries_and_lifecycle(self):
        facade = (SCRIPTS / "global_frontier_place_memory.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin, method in (
            (
                "global_frontier_place_memory_association.py",
                "FrontierRegionAssociationMixin",
                "candidate_tier",
            ),
            (
                "global_frontier_place_memory_queries.py",
                "FrontierRegionQueryMixin",
                "refresh_components",
            ),
            (
                "global_frontier_place_memory_lifecycle.py",
                "FrontierRegionLifecycleMixin",
                None,
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            if method is not None:
                self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, facade)
        lifecycle = (SCRIPTS / "global_frontier_place_memory_lifecycle.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin, method in (
            (
                "global_frontier_place_memory_portals.py",
                "FrontierRegionPortalMixin",
                "enter",
            ),
            (
                "global_frontier_place_memory_observation.py",
                "FrontierRegionObservationMixin",
                "endpoint_observed",
            ),
            (
                "global_frontier_place_memory_state.py",
                "FrontierRegionStateMixin",
                "activate",
            ),
            (
                "global_frontier_place_memory_physical.py",
                None,
                "project_points",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            if mixin is not None:
                self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            if mixin is not None:
                self.assertIn(mixin, lifecycle)
        self.assertLessEqual(len(lifecycle.splitlines()), 35)
        self.assertLessEqual(len(facade.splitlines()), 100)

    def test_activation_has_its_own_route_transaction(self):
        source = (SCRIPTS / "global_frontier_activation.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierActivationMixin", source)
        for name in (
            "selected_frontier_from_cell",
            "initialize_active_frontier_route",
            "activate_selected_frontier",
        ):
            self.assertIn("def %s(" % name, source)

        main_source = (SCRIPTS / "lste_global_frontier_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("GlobalFrontierActivationMixin", main_source)
        self.assertNotIn("    def initialize_active_frontier_route(", main_source)

    def test_cmake_installs_frontier_mixins(self):
        source = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for name in (
            "scripts/global_frontier_config.py",
            "scripts/global_frontier_config_topics.py",
            "scripts/global_frontier_config_navigation.py",
            "scripts/global_frontier_config_exploration.py",
            "scripts/global_frontier_graph_executive.py",
            "scripts/global_frontier_place_progress.py",
            "scripts/global_frontier_portal_transaction.py",
            "scripts/global_frontier_action_policy.py",
            "scripts/global_frontier_completion_gate.py",
            "scripts/global_frontier_completion_recovery.py",
            "scripts/global_frontier_event_graph.py",
            "scripts/global_frontier_decision_wake.py",
            "scripts/global_frontier_evidence_contract.py",
            "scripts/global_frontier_transition.py",
            "scripts/global_frontier_durable_action_lease.py",
            "scripts/global_frontier_durable_lease_lifecycle.py",
            "scripts/global_frontier_work_items.py",
            "scripts/global_frontier_semantic_belief.py",
            "scripts/global_frontier_portal_belief.py",
            "scripts/global_frontier_portal_egress.py",
            "scripts/global_frontier_portal_probes.py",
            "scripts/global_frontier_structural_boundaries.py",
            "scripts/global_frontier_portal_probe_ledger.py",
            "scripts/global_frontier_portal_probe_lifecycle.py",
            "scripts/global_frontier_runtime_state.py",
            "scripts/global_frontier_ros_interfaces.py",
            "scripts/global_frontier_reporting.py",
            "scripts/global_frontier_route_state.py",
            "scripts/global_frontier_event_callbacks.py",
            "scripts/global_frontier_geometry.py",
            "scripts/global_frontier_observation.py",
            "scripts/global_frontier_planning_cycle.py",
            "scripts/global_frontier_planning_prefetch.py",
            "scripts/global_frontier_prefetch_selection.py",
            "scripts/global_frontier_prefetch_validation.py",
            "scripts/global_frontier_prefetch_activation.py",
            "scripts/global_frontier_activation.py",
            "scripts/global_frontier_terminal_lifecycle.py",
            "scripts/global_frontier_candidate_lifecycle.py",
            "scripts/global_frontier_candidate_portals.py",
            "scripts/global_frontier_candidate_routing.py",
            "scripts/global_frontier_candidate_scoring.py",
            "scripts/global_frontier_selection_planner.py",
            "scripts/global_frontier_selection_validation.py",
            "scripts/global_frontier_execution_observation.py",
            "scripts/global_frontier_execution_prefetch.py",
            "scripts/global_frontier_execution_resolution.py",
            "scripts/global_frontier_execution_routes.py",
            "scripts/global_frontier_execution_egress.py",
            "scripts/global_frontier_execution_active_route.py",
            "scripts/global_frontier_execution_watchdogs.py",
            "scripts/global_frontier_portal_selection.py",
            "scripts/global_frontier_portal_lifecycle.py",
            "scripts/global_frontier_portal_arrival_resolution.py",
            "scripts/global_frontier_portal_arrival_commit.py",
            "scripts/global_frontier_portal_recovery.py",
            "scripts/global_frontier_route_command_resolution.py",
            "scripts/global_frontier_route_command_geometry.py",
            "scripts/global_frontier_route_command_publication.py",
            "scripts/global_frontier_terminal_observation.py",
        ):
            self.assertIn(name, source)

    def test_route_commands_separate_resolution_from_ros_publication(self):
        facade = (SCRIPTS / "global_frontier_route_commands.py").read_text(
            encoding="utf-8"
        )
        geometry_source = (
            SCRIPTS / "global_frontier_route_command_geometry.py"
        ).read_text(encoding="utf-8")
        self.assertIn("class GlobalFrontierRouteCommandGeometryMixin", geometry_source)
        self.assertIn("def select_route_command_cell(", geometry_source)

        for filename, mixin, method in (
            (
                "global_frontier_route_command_resolution.py",
                "GlobalFrontierRouteCommandResolutionMixin",
                "resolve_active_route_command",
            ),
            (
                "global_frontier_route_command_publication.py",
                "GlobalFrontierRouteCommandPublicationMixin",
                "publish_active_route_command",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, facade)
        resolution_source = (
            SCRIPTS / "global_frontier_route_command_resolution.py"
        ).read_text(encoding="utf-8")
        self.assertIn("GlobalFrontierRouteCommandGeometryMixin", resolution_source)
        self.assertLessEqual(len(facade.splitlines()), 60)

    def test_portal_arrivals_separate_evidence_and_memory_commit(self):
        facade = (SCRIPTS / "global_frontier_portal_lifecycle.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin, method in (
            (
                "global_frontier_portal_arrival_resolution.py",
                "GlobalFrontierPortalArrivalResolutionMixin",
                "portal_arrival_component_from_snapshot",
            ),
            (
                "global_frontier_portal_arrival_commit.py",
                "GlobalFrontierPortalArrivalCommitMixin",
                "commit_portal_arrival",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % method, source)
            self.assertIn(mixin, facade)
        self.assertLessEqual(len(facade.splitlines()), 120)

    def test_portal_recovery_has_its_own_edge_transaction_module(self):
        source = (SCRIPTS / "global_frontier_portal_recovery.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierPortalRecoveryMixin", source)
        for name in (
            "prepare_portal_transition_retry",
            "select_pending_portal_retry",
            "discard_pending_portal_retry",
        ):
            self.assertIn("def %s(" % name, source)

    def test_reporting_owns_all_external_route_messages(self):
        source = (SCRIPTS / "global_frontier_reporting.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierReportingMixin", source)
        for name in (
            "active_region_report",
            "publish_status",
            "publish_route_command",
        ):
            self.assertIn("def %s(" % name, source)

        main_source = (SCRIPTS / "lste_global_frontier_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("GlobalFrontierReportingMixin", main_source)
        for name in ("publish_status", "publish_route_command"):
            self.assertNotIn("    def %s(" % name, main_source)

    def test_terminal_lifecycle_has_its_own_module(self):
        source = (SCRIPTS / "global_frontier_terminal_lifecycle.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierTerminalLifecycleMixin", source)
        for module, mixin, name in (
            (
                "global_frontier_terminal_validation.py",
                "GlobalFrontierTerminalValidationMixin",
                "matching_execution_terminal",
            ),
            (
                "global_frontier_terminal_recording.py",
                "GlobalFrontierTerminalRecordingMixin",
                "record_execution_terminal",
            ),
            (
                "global_frontier_terminal_replan.py",
                "GlobalFrontierTerminalReplanMixin",
                "schedule_terminal_replan",
            ),
            (
                "global_frontier_terminal_prefetch.py",
                "GlobalFrontierTerminalPrefetchMixin",
                "promote_prefetched_terminal",
            ),
            (
                "global_frontier_terminal_connector.py",
                "GlobalFrontierTerminalConnectorMixin",
                "consume_connector_terminal",
            ),
        ):
            module_source = (SCRIPTS / module).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, module_source)
            self.assertIn("def %s(" % name, module_source)

        main_source = (SCRIPTS / "lste_global_frontier_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("GlobalFrontierTerminalLifecycleMixin", main_source)
        self.assertIn("GlobalFrontierPortalLifecycleMixin", main_source)
        self.assertNotIn("    def matching_execution_terminal(", main_source)

    def test_observation_policy_has_its_own_module(self):
        facade_source = (SCRIPTS / "global_frontier_observation.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierObservationMixin", facade_source)
        for filename, mixin, name in (
            (
                "global_frontier_observation_components.py",
                "GlobalFrontierObservationComponentsMixin",
                "build_topology_components",
            ),
            (
                "global_frontier_observation_viewpoints.py",
                "GlobalFrontierObservationViewpointMixin",
                "candidate_covered_by_completed_viewpoint",
            ),
            (
                "global_frontier_observation_departure.py",
                "GlobalFrontierObservationDepartureMixin",
                "commit_departed_place_after_boundary_crossing",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, source)
            self.assertIn("def %s(" % name, source)

        main_source = (SCRIPTS / "lste_global_frontier_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("GlobalFrontierObservationMixin", main_source)
        self.assertNotIn("    def build_topology_components(", main_source)

    def test_composition_root_keeps_setup_and_callbacks_out_of_the_node(self):
        main_source = (SCRIPTS / "lste_global_frontier_node.py").read_text(
            encoding="utf-8"
        )
        expected_modules = {
            "global_frontier_config.py": "GlobalFrontierConfigurationMixin",
            "global_frontier_runtime_state.py": "GlobalFrontierRuntimeStateMixin",
            "global_frontier_ros_interfaces.py": "GlobalFrontierRosInterfacesMixin",
            "global_frontier_reporting.py": "GlobalFrontierReportingMixin",
            "global_frontier_route_state.py": "GlobalFrontierRouteStateMixin",
            "global_frontier_event_callbacks.py": "GlobalFrontierEventCallbacksMixin",
            "global_frontier_geometry.py": "GlobalFrontierGeometryMixin",
        }
        for filename, mixin in expected_modules.items():
            module_source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, module_source)
            self.assertIn(mixin, main_source)

        for method in (
            "_load_parameters",
            "_initialize_runtime_state",
            "_setup_ros_interfaces",
            "publish_status",
            "publish_route_command",
            "on_costmap_update",
            "release_active_frontier",
            "on_replan_request",
            "transform_xy",
        ):
            self.assertNotIn("    def %s(" % method, main_source)
        self.assertLessEqual(len(main_source.splitlines()), 220)

    def test_selection_delegates_route_scoring_and_portal_concerns(self):
        source = (SCRIPTS / "global_frontier_selection.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin in (
            (
                "global_frontier_selection_context.py",
                "GlobalFrontierSelectionContextMixin",
            ),
            ("global_frontier_candidate_routing.py", "GlobalFrontierCandidateRoutingMixin"),
            ("global_frontier_candidate_scoring.py", "GlobalFrontierCandidateScoringMixin"),
            ("global_frontier_portal_selection.py", "GlobalFrontierPortalSelectionMixin"),
            ("global_frontier_selection_planner.py", "GlobalFrontierSelectionPlannerMixin"),
            ("global_frontier_selection_validation.py", "GlobalFrontierSelectionValidationMixin"),
        ):
            module_source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, module_source)
            self.assertIn(mixin, source)

        self.assertLessEqual(len(source.splitlines()), 40)

    def test_candidate_selection_separates_portals_and_lifecycle(self):
        routing = (SCRIPTS / "global_frontier_candidate_routing.py").read_text(
            encoding="utf-8"
        )
        portals = (SCRIPTS / "global_frontier_candidate_portals.py").read_text(
            encoding="utf-8"
        )
        scoring = (SCRIPTS / "global_frontier_candidate_scoring.py").read_text(
            encoding="utf-8"
        )
        lifecycle = (
            SCRIPTS / "global_frontier_candidate_lifecycle.py"
        ).read_text(encoding="utf-8")
        self.assertIn("GlobalFrontierCandidatePortalMixin", routing)
        self.assertIn("PortalSource", portals)
        self.assertIn("_populate_adjacent_portal_sources", portals)
        self.assertIn("GlobalFrontierCandidateLifecycleMixin", scoring)
        self.assertIn("_candidate_region_pool", lifecycle)
        self.assertLessEqual(len(routing.splitlines()), 230)
        self.assertLessEqual(len(scoring.splitlines()), 220)

    def test_portal_certification_separates_routes_cuts_and_queries(self):
        """Keep the topology policy readable without changing public imports."""
        facade = (SCRIPTS / "global_frontier_portal_certification.py").read_text(
            encoding="utf-8"
        )
        for filename, symbol in (
            (
                "global_frontier_portal_certification_models.py",
                "RoutePlaceTransition",
            ),
            (
                "global_frontier_portal_certification_routes.py",
                "route_place_transition_candidates",
            ),
            (
                "global_frontier_portal_certification_cut.py",
                "is_structural_cut",
            ),
            (
                "global_frontier_portal_certification_transitions.py",
                "certified_route_place_transitions",
            ),
        ):
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn(symbol, source)

        self.assertIn("certified_adjacent_place_transitions", facade)
        self.assertLessEqual(len(facade.splitlines()), 70)

    def test_execution_delegates_each_route_lifecycle_stage(self):
        source = (SCRIPTS / "global_frontier_execution.py").read_text(
            encoding="utf-8"
        )
        for filename, mixin in (
            ("global_frontier_execution_observation.py", "GlobalFrontierExecutionObservationMixin"),
            ("global_frontier_execution_watchdogs.py", "GlobalFrontierExecutionWatchdogMixin"),
            ("global_frontier_execution_prefetch.py", "GlobalFrontierExecutionPrefetchMixin"),
            ("global_frontier_execution_resolution.py", "GlobalFrontierExecutionResolutionMixin"),
            ("global_frontier_execution_routes.py", "GlobalFrontierExecutionRouteMixin"),
            ("global_frontier_execution_egress.py", "GlobalFrontierExecutionEgressMixin"),
            ("global_frontier_execution_active_route.py", "GlobalFrontierExecutionActiveRouteMixin"),
        ):
            module_source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn("class %s" % mixin, module_source)
            self.assertIn(mixin, source)

        self.assertLessEqual(len(source.splitlines()), 80)

    def test_terminal_observation_closure_is_separate_from_terminal_matching(self):
        source = (SCRIPTS / "global_frontier_terminal_observation.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("class GlobalFrontierTerminalObservationMixin", source)
        self.assertIn("def close_stagnant_place_after_terminal(", source)
        recording = (SCRIPTS / "global_frontier_terminal_recording.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "self.close_stagnant_place_after_terminal(completed)",
            recording,
        )


if __name__ == "__main__":
    unittest.main()
