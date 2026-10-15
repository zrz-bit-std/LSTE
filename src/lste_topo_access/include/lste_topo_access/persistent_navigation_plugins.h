#ifndef LSTE_TOPO_ACCESS_PERSISTENT_NAVIGATION_PLUGINS_H
#define LSTE_TOPO_ACCESS_PERSISTENT_NAVIGATION_PLUGINS_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <lste_topo_access/PlannerCommandContract.h>
#include <lste_topo_access/PersistentGoalCommand.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <navfn/navfn_ros.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace lste_topo_access {

// Keeps one move_base action alive while the exploration layer updates the
// global path. Arrival at a temporary frontier is therefore not task success.
class PersistentTebLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  PersistentTebLocalPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

 private:
  void onTaskDone(const std_msgs::BoolConstPtr& message);
  void onMissionCommand(const PersistentGoalCommandConstPtr& message);
  void onInstalledTargetCommand(
      const PersistentGoalCommandConstPtr& message);
  bool isInstalledTargetPlanLocked();
  bool reportTargetApproachIfReady();
  bool reportFrontierEndpointIfReady();
  void publishPlanEvent(
      const char* event,
      const std::vector<geometry_msgs::PoseStamped>& plan) const;
  void publishPlannerCommandContract(
      const geometry_msgs::Twist& command, uint8_t state,
      const std::string& reason);
  static uint64_t geometryHash(
      const std::vector<geometry_msgs::PoseStamped>& plan);
  bool planIsEquivalentLocked(
      const std::vector<geometry_msgs::PoseStamped>& candidate) const;

  bool initialized_;
  bool persistent_execution_;
  std::atomic<bool> task_done_;
  std::mutex plan_mutex_;
  bool has_installed_target_goal_;
  bool has_reported_target_goal_;
  bool has_reported_frontier_goal_;
  // Once the persistent local planner has reported a verified endpoint, keep
  // returning a valid zero command until the graph installs a successor. This
  // is a controller lease state, not MoveBase goal completion; without it,
  // TEB can briefly lose its geometric goal and MoveBase starts recovery while
  // the graph is still committing the next Place/Portal transition.
  std::atomic<bool> terminal_hold_active_;
  // A warm-slice reset holds the local planner at zero until a new Navfn plan
  // is installed. This covers the cancellation-to-clear propagation window.
  std::atomic<bool> hard_reset_hold_;
  geometry_msgs::PoseStamped installed_target_goal_;
  geometry_msgs::PoseStamped reported_target_goal_;
  geometry_msgs::PoseStamped reported_frontier_goal_;
  // A physical endpoint can legitimately reappear after a route retry.  Its
  // identity is the installed TEB path, rather than only its XY coordinate.
  uint64_t reported_frontier_route_version_;
  geometry_msgs::PoseStamped current_plan_goal_;
  std::vector<geometry_msgs::PoseStamped> applied_plan_;
  double target_goal_epsilon_;
  // Frontier endpoints are produced on the Navfn occupancy-grid lattice.
  // Keep their comparison tolerance separate from an exact visual-target
  // contract, otherwise two diagonally adjacent grid cells repeatedly reset
  // the same TEB trajectory.
  double frontier_endpoint_equivalence_distance_;
  double plan_equivalence_distance_;
  double plan_equivalence_prefix_distance_;
  ros::WallTime last_equivalent_plan_event_wall_;
  std::atomic<uint64_t> plan_received_count_;
  std::atomic<uint64_t> plan_equivalent_count_;
  std::atomic<uint64_t> plan_installed_count_;
  std::atomic<uint64_t> route_version_;
  std::atomic<uint64_t> route_geometry_hash_;
  uint32_t installed_target_transaction_;
  uint64_t installed_target_origin_epoch_;
  uint64_t installed_target_projection_epoch_;
  std::atomic<uint32_t> latest_mission_transaction_;
  // The persistent bridge can change route semantics without changing the
  // Navfn cell endpoint (for example, frontier observation -> Portal
  // crossing). A new mission transaction must therefore force one plan
  // installation before the old endpoint may be reported again.
  std::atomic<uint32_t> installed_mission_transaction_;
  ros::Subscriber task_done_subscriber_;
  ros::Subscriber mission_command_subscriber_;
  ros::Subscriber installed_target_command_subscriber_;
  ros::Publisher target_approach_publisher_;
  ros::Publisher target_approach_result_publisher_;
  ros::Publisher frontier_endpoint_publisher_;
  ros::Publisher plan_event_publisher_;
  ros::Publisher planner_command_contract_publisher_;
  teb_local_planner::TebLocalPlannerROS teb_;
  uint32_t latest_route_id_;
  uint64_t latest_graph_transaction_id_;
  uint64_t latest_map_epoch_;
  uint64_t latest_origin_epoch_;
  uint64_t latest_projection_epoch_;
  std::string latest_lifecycle_transaction_id_;
  uint32_t latest_action_generation_;
  std::atomic<uint64_t> planner_command_sequence_;
};

// Routes move_base's one active action to the latest mission goal. It caches
// the Navfn path until mission ownership changes, avoiding map-refresh churn.
class StreamingNavfnPlanner : public nav_core::BaseGlobalPlanner {
 public:
  StreamingNavfnPlanner();

  void initialize(std::string name,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& action_goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  void onMissionCommand(const PersistentGoalCommandConstPtr& message);
  void onTargetCommand(const PersistentGoalCommandConstPtr& message);
  void publishTargetPlanResult(const char* event, uint32_t transaction_id,
                               const geometry_msgs::PoseStamped& goal) const;
  bool shouldReplanLocked(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& selected) const;
  bool transformToGlobalFrame(const geometry_msgs::PoseStamped& source,
                              geometry_msgs::PoseStamped& transformed) const;

  bool initialized_;
  std::mutex mutex_;
  ros::Subscriber mission_command_subscriber_;
  ros::Subscriber target_command_subscriber_;
  ros::Publisher installed_target_goal_publisher_;
  ros::Publisher installed_target_command_publisher_;
  ros::Publisher target_plan_result_publisher_;
  navfn::NavfnROS navfn_;
  std::unique_ptr<tf::TransformListener> transform_listener_;
  std::string global_frame_;
  bool has_mission_goal_;
  bool has_target_goal_;
  bool has_validated_target_plan_;
  // A target remains a target after the bridge promotes its provisional
  // request to a mission command. Replanning that approved mission must keep
  // the exact endpoint contract until another mission supersedes it.
  bool has_active_target_mission_;
  bool plan_dirty_;
  geometry_msgs::PoseStamped mission_goal_;
  geometry_msgs::PoseStamped target_goal_;
  geometry_msgs::PoseStamped validated_target_goal_;
  geometry_msgs::PoseStamped cached_goal_;
  geometry_msgs::PoseStamped last_plan_start_;
  std::vector<geometry_msgs::PoseStamped> cached_plan_;
  std::vector<geometry_msgs::PoseStamped> validated_target_plan_;
  double goal_epsilon_;
  double replan_distance_;
  double max_replan_interval_;
  ros::WallTime last_plan_time_;
  // Bridge transaction IDs are explicit fields in PersistentGoalCommand. A
  // delayed latched delivery must never let an older mission or speculative
  // target overwrite a newer approved route.
  uint32_t latest_mission_sequence_;
  uint64_t latest_mission_origin_epoch_;
  uint64_t latest_mission_projection_epoch_;
  uint32_t latest_target_sequence_;
  uint32_t active_target_mission_sequence_;
  uint32_t validated_target_sequence_;
  uint32_t failed_target_sequence_;
  uint64_t latest_target_map_epoch_;
  uint64_t latest_target_origin_epoch_;
  uint64_t latest_target_projection_epoch_;
  uint64_t latest_target_graph_transaction_id_;
  uint32_t latest_target_route_id_;
  std::string latest_target_lifecycle_transaction_id_;
  uint32_t latest_target_action_generation_;
  std::string latest_target_route_kind_;
  std::string latest_target_mission_route_kind_;
  std::string latest_target_graph_action_;
  std::string latest_target_graph_obligation_kind_;
  uint64_t mission_generation_;
  uint64_t target_failure_reported_generation_;
};

}  // namespace lste_topo_access

#endif
