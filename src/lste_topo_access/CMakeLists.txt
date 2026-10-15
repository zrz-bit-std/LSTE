cmake_minimum_required(VERSION 3.0.2)
project(lste_topo_access)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  costmap_2d
  gazebo_msgs
  geometry_msgs
  lste_msgs
  message_generation
  move_base_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  teb_local_planner
  tf
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FrontierExecutionTerminal.msg
  PlannerCommandContract.msg
  PersistentGoalCommand.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lste_persistent_navigation_plugins
  CATKIN_DEPENDS costmap_2d gazebo_msgs geometry_msgs message_runtime nav_core navfn pluginlib roscpp std_msgs teb_local_planner tf tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(lste_persistent_navigation_plugins
  src/persistent_navigation_plugins.cpp
)
target_link_libraries(lste_persistent_navigation_plugins
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(lste_persistent_navigation_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lste_topo_access_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/gp_subgoals_sim_topo.py
  scripts/lste_global_frontier_node.py
  scripts/lste_navigation_metrics.py
  scripts/lste_teb_goal_bridge.py
  scripts/lste_teb_turn_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# The Python nodes import these sibling mixins/helpers at runtime. Install them
# beside the executables so an install-space launch has the same module layout
# as a devel-space launch.
install(FILES
  scripts/clock_provider.py
  scripts/experiment_reset_contract.py
  scripts/lifecycle_manager.py
  scripts/global_frontier_lifecycle.py
  scripts/goal_manager_lifecycle.py
  scripts/teb_goal_bridge_lifecycle.py
  scripts/teb_turn_supervisor_lifecycle.py
  scripts/gp_frontier_bootstrap.py
  scripts/gp_frontier_common.py
  scripts/gp_frontier_callbacks_config.py
  scripts/gp_frontier_topology.py
  scripts/gp_frontier_anchor_graph.py
  scripts/gp_frontier_backtrack.py
  scripts/gp_frontier_forced_branch.py
  scripts/gp_frontier_junction_logging.py
  scripts/gp_frontier_junction.py
  scripts/gp_frontier_lifecycle.py
  scripts/gp_frontier_model.py
  scripts/gp_frontier_geometry_candidates.py
  scripts/gp_frontier_publication.py
  scripts/gp_frontier_visualization.py
  scripts/gp_frontier_runtime.py
  scripts/gp_frontier_sparse_gp.py
  scripts/global_frontier_grid.py
  scripts/global_frontier_action_policy.py
  scripts/global_frontier_completion_gate.py
  scripts/global_frontier_completion_recovery.py
  scripts/global_frontier_event_graph.py
  scripts/global_frontier_decision_wake.py
  scripts/global_frontier_evidence_contract.py
  scripts/global_frontier_transition.py
  scripts/global_frontier_graph_route_planner.py
  scripts/global_frontier_graph_route_adapter.py
  scripts/global_frontier_graph_route_transaction.py
  scripts/global_frontier_graph_route_gate.py
  scripts/target_ray_hypothesis.py
  scripts/global_frontier_frontier_decision.py
  scripts/global_frontier_graph_executive.py
  scripts/global_frontier_experiment_policy.py
  scripts/global_frontier_models.py
  scripts/global_frontier_execution.py
  scripts/global_frontier_execution_active_route.py
  scripts/global_frontier_execution_egress.py
  scripts/global_frontier_execution_observation.py
  scripts/global_frontier_execution_prefetch.py
  scripts/global_frontier_execution_resolution.py
  scripts/global_frontier_execution_routes.py
  scripts/global_frontier_execution_watchdogs.py
  scripts/global_frontier_physical_progress.py
  scripts/global_frontier_event_callbacks.py
  scripts/global_frontier_durable_action_lease.py
  scripts/global_frontier_durable_lease_lifecycle.py
  scripts/global_frontier_geometry.py
  scripts/global_frontier_observation.py
  scripts/global_frontier_observation_topology.py
  scripts/global_frontier_observation_components.py
  scripts/global_frontier_observation_place_state.py
  scripts/global_frontier_observation_viewpoints.py
  scripts/global_frontier_observation_coverage.py
  scripts/global_frontier_observation_departure.py
  scripts/global_frontier_planning_cycle.py
  scripts/global_frontier_planning_contract.py
  scripts/global_frontier_planning_runtime.py
  scripts/global_frontier_planning_snapshot.py
  scripts/global_frontier_planning_selection.py
  scripts/global_frontier_activation.py
  scripts/global_frontier_candidate_lifecycle.py
  scripts/global_frontier_candidate_work_items.py
  scripts/global_frontier_candidate_place_policy.py
  scripts/global_frontier_candidate_portals.py
  scripts/global_frontier_candidate_graph.py
  scripts/global_frontier_candidate_routing.py
  scripts/global_frontier_candidate_scoring.py
  scripts/global_frontier_config.py
  scripts/global_frontier_config_topics.py
  scripts/global_frontier_config_navigation.py
  scripts/global_frontier_config_exploration.py
  scripts/global_frontier_planning.py
  scripts/global_frontier_planning_costmap.py
  scripts/global_frontier_planning_navfn.py
  scripts/global_frontier_planning_prefetch.py
  scripts/global_frontier_prefetch_selection.py
  scripts/global_frontier_prefetch_validation.py
  scripts/global_frontier_prefetch_activation.py
  scripts/global_frontier_planning_routes.py
  scripts/global_frontier_reporting.py
  scripts/global_frontier_place_lifecycle.py
  scripts/global_frontier_place_memory.py
  scripts/global_frontier_place_memory_association.py
  scripts/global_frontier_place_memory_queries.py
  scripts/global_frontier_place_memory_lifecycle.py
  scripts/global_frontier_place_memory_state.py
  scripts/global_frontier_place_memory_portals.py
  scripts/global_frontier_place_memory_observation.py
  scripts/global_frontier_place_memory_physical.py
  scripts/global_frontier_place_states.py
  scripts/global_frontier_place_progress.py
  scripts/global_frontier_portal_selection.py
  scripts/global_frontier_portal_probes.py
  scripts/global_frontier_structural_boundaries.py
  scripts/global_frontier_portal_probe_lifecycle.py
  scripts/global_frontier_portal_probe_ledger.py
  scripts/global_frontier_portal_probe_value.py
  scripts/global_frontier_portal_probe_adapter.py
  scripts/global_frontier_portal_decision.py
  scripts/global_frontier_portal_admission.py
  scripts/global_frontier_portal_certification.py
  scripts/global_frontier_portal_certification_models.py
  scripts/global_frontier_portal_certification_routes.py
  scripts/global_frontier_portal_certification_cut.py
  scripts/global_frontier_portal_certification_wall.py
  scripts/global_frontier_portal_certification_transitions.py
  scripts/global_frontier_portal_crossing.py
  scripts/global_frontier_portal_crossing_lifecycle.py
  scripts/global_frontier_portal_lifecycle.py
  scripts/global_frontier_portal_arrival_resolution.py
  scripts/global_frontier_portal_transaction.py
  scripts/global_frontier_portal_arrival_commit.py
  scripts/global_frontier_portal_recovery.py
  scripts/global_frontier_ros_interfaces.py
  scripts/global_frontier_route_commands.py
  scripts/global_frontier_route_command_geometry.py
  scripts/global_frontier_route_command_resolution.py
  scripts/global_frontier_route_command_publication.py
  scripts/global_frontier_route_history.py
  scripts/global_frontier_route_lease.py
  scripts/global_frontier_semantic_belief.py
  scripts/global_frontier_target_belief.py
  scripts/global_frontier_target_observation_work.py
  scripts/global_frontier_portal_belief.py
  scripts/global_frontier_durable_portal_route.py
  scripts/global_frontier_directional_branch_coverage.py
  scripts/global_frontier_portal_egress.py
  scripts/global_frontier_route_state.py
  scripts/global_frontier_runtime_state.py
  scripts/global_frontier_work_items.py
  scripts/global_frontier_work_item_components.py
  scripts/global_frontier_scoring.py
  scripts/global_frontier_selection_context.py
  scripts/global_frontier_selection_planner.py
  scripts/global_frontier_selection_validation.py
  scripts/global_frontier_selection.py
  scripts/global_frontier_terminal_lifecycle.py
  scripts/global_frontier_terminal_connector.py
  scripts/global_frontier_terminal_validation.py
  scripts/global_frontier_terminal_recording.py
  scripts/global_frontier_terminal_replan.py
  scripts/global_frontier_terminal_prefetch.py
  scripts/global_frontier_terminal_observation.py
  scripts/global_frontier_topology.py
  scripts/global_frontier_topology_components.py
  scripts/global_frontier_topology_places.py
  scripts/global_frontier_topology_evidence.py
  scripts/global_frontier_topology_grid.py
  scripts/global_frontier_topology_routes.py
  scripts/global_frontier_topology_policy.py
  scripts/goal_manager_detection.py
  scripts/target_track_identity.py
  scripts/goal_manager_route_validation.py
  scripts/goal_manager_config.py
  scripts/goal_manager_config_navigation.py
  scripts/goal_manager_config_target_completion.py
  scripts/goal_manager_config_target_routing.py
  scripts/goal_manager_config_target_tracking.py
  scripts/goal_manager_goal_arbitration.py
  scripts/goal_manager_goal_output.py
  scripts/goal_context.py
  scripts/goal_manager_legacy_goals.py
  scripts/goal_manager_modes.py
  scripts/goal_manager_input_callbacks.py
  scripts/goal_manager_ros_callbacks.py
  scripts/goal_manager_teb_callbacks.py
  scripts/goal_manager_ros_interfaces.py
  scripts/goal_manager_runtime_state.py
  scripts/goal_manager_scheduling.py
  scripts/goal_manager_target_completion.py
  scripts/goal_manager_target_geometry.py
  scripts/goal_manager_target_segments.py
  scripts/goal_manager_target_route_continuity.py
  scripts/goal_manager_target_segment_commit.py
  scripts/goal_manager_target_recovery.py
  scripts/goal_manager_frontier.py
  scripts/goal_manager_projection.py
  scripts/goal_manager_viewpoint.py
  scripts/goal_manager_target_viewpoint_geometry.py
  scripts/goal_manager_target_follow.py
  scripts/goal_manager_target_approach_transaction.py
  scripts/goal_manager_target_viewpoint_attempt_ledger.py
  scripts/goal_manager_target_observation_gate.py
  scripts/goal_manager_target_utils.py
  scripts/navigation_metrics_target_eval.py
  scripts/navigation_metrics_target_eval_configuration.py
  scripts/navigation_metrics_target_eval_projection.py
  scripts/navigation_metrics_target_eval_depth.py
  scripts/navigation_metrics_target_eval_episodes.py
  scripts/navigation_metrics_benchmark_truth.py
  scripts/office_benchmark_coverage.py
  scripts/navigation_metrics_action_lifecycle.py
  scripts/navigation_metrics_command_events.py
  scripts/navigation_metrics_command_quality.py
  scripts/navigation_metrics_command_stream.py
  scripts/navigation_metrics_callbacks.py
  scripts/navigation_metrics_detection.py
  scripts/navigation_metrics_execution_events.py
  scripts/navigation_metrics_failure_evidence.py
  scripts/navigation_metrics_goal_events.py
  scripts/navigation_metrics_logging.py
  scripts/navigation_metrics_observer_state.py
  scripts/navigation_metrics_planner.py
  scripts/navigation_metrics_planner_teb_feedback.py
  scripts/navigation_metrics_planner_move_base.py
  scripts/navigation_metrics_planner_paths.py
  scripts/navigation_metrics_pose.py
  scripts/navigation_metrics_ros_interfaces.py
  scripts/navigation_metrics_runtime.py
  scripts/navigation_metrics_setup.py
  scripts/navigation_metrics_snapshot.py
  scripts/navigation_metrics_target_state.py
  scripts/teb_goal_bridge_action_health.py
  scripts/teb_goal_bridge_action_callbacks.py
  scripts/teb_goal_bridge_action_client.py
  scripts/teb_goal_bridge_action_dispatch.py
  scripts/teb_goal_bridge_action_active_dispatch.py
  scripts/teb_goal_bridge_action_feedback.py
  scripts/teb_goal_bridge_action_retry_policy.py
  scripts/teb_goal_bridge_action_terminal.py
  scripts/teb_goal_bridge_frontier_status.py
  scripts/teb_goal_bridge_geometry.py
  scripts/teb_goal_bridge_handoff.py
  scripts/teb_goal_bridge_handoff_segment.py
  scripts/teb_goal_bridge_handoff_stall.py
  scripts/teb_goal_bridge_intent.py
  scripts/teb_goal_bridge_mission_lifecycle.py
  scripts/teb_goal_bridge_mission_control.py
  scripts/teb_goal_bridge_mission_input.py
  scripts/teb_goal_bridge_mission_runtime.py
  scripts/teb_goal_bridge_parameters.py
  scripts/teb_goal_bridge_persistent_execution.py
  scripts/teb_goal_bridge_persistent_frontier_endpoint.py
  scripts/teb_goal_bridge_persistent_frontier_handoff.py
  scripts/teb_goal_bridge_prefetch_admission.py
  scripts/teb_goal_bridge_persistent_target.py
  scripts/teb_goal_bridge_persistent_target_approach.py
  scripts/teb_goal_bridge_persistent_target_result.py
  scripts/teb_goal_bridge_ros.py
  scripts/teb_goal_bridge_route_monitoring.py
  scripts/teb_goal_bridge_state.py
  scripts/teb_goal_bridge_status.py
  scripts/teb_goal_bridge_teb_runtime.py
  scripts/teb_turn_supervisor_callbacks.py
  scripts/teb_turn_supervisor_control.py
  scripts/teb_turn_supervisor_contract.py
  scripts/teb_turn_supervisor_parameters.py
  scripts/teb_turn_supervisor_ros.py
  scripts/teb_turn_supervisor_routes.py
  scripts/teb_turn_supervisor_state.py
  scripts/teb_turn_supervisor_turn_lifecycle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS lste_persistent_navigation_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES persistent_navigation_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lste_topo_access.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
