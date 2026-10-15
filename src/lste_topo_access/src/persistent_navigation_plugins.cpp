#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

#include "lste_topo_access/persistent_navigation_plugins.h"

namespace lste_topo_access {

PersistentTebLocalPlanner::PersistentTebLocalPlanner()
    : initialized_(false),
      persistent_execution_(false),
      task_done_(false),
      has_installed_target_goal_(false),
      has_reported_target_goal_(false),
      has_reported_frontier_goal_(false),
      terminal_hold_active_(false),
      hard_reset_hold_(false),
      reported_frontier_route_version_(0),
      target_goal_epsilon_(0.05),
      frontier_endpoint_equivalence_distance_(0.05),
      plan_equivalence_distance_(0.25),
      plan_equivalence_prefix_distance_(0.35),
      plan_received_count_(0),
      plan_equivalent_count_(0),
      plan_installed_count_(0),
      route_version_(0),
      route_geometry_hash_(0),
      installed_target_transaction_(0),
      installed_target_origin_epoch_(0),
      installed_target_projection_epoch_(0),
      latest_mission_transaction_(0),
      installed_mission_transaction_(0),
      latest_route_id_(0),
      latest_graph_transaction_id_(0),
      latest_map_epoch_(0),
      latest_origin_epoch_(0),
      latest_projection_epoch_(0),
      latest_lifecycle_transaction_id_(),
      latest_action_generation_(0),
      planner_command_sequence_(0) {}

void PersistentTebLocalPlanner::initialize(
    std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("PersistentTebLocalPlanner is already initialized");
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  ros::param::param("/move_base/persistent_execution", persistent_execution_,
                    false);
  std::string task_done_topic;
  std::string installed_target_goal_topic;
  std::string installed_target_command_topic;
  std::string target_approach_topic;
  std::string frontier_endpoint_topic;
  std::string plan_event_topic;
  std::string target_plan_result_topic;
  ros::param::param<std::string>("/move_base/persistent_task_done_topic",
                                 task_done_topic, "/lste/task_done");
  ros::param::param<std::string>(
      "/move_base/persistent_installed_target_goal_topic",
      installed_target_goal_topic,
      "/lste/persistent_execution/installed_target_goal");
  ros::param::param<std::string>(
      "/move_base/persistent_installed_target_command_topic",
      installed_target_command_topic,
      "/lste/persistent_execution/installed_target_command");
  ros::param::param<std::string>(
      "/move_base/persistent_target_approach_topic", target_approach_topic,
      "/lste/persistent_execution/target_approach");
  ros::param::param<std::string>(
      "/move_base/persistent_frontier_endpoint_topic", frontier_endpoint_topic,
      "/lste/persistent_execution/frontier_endpoint_reached");
  ros::param::param<std::string>(
      "/move_base/persistent_plan_event_topic", plan_event_topic,
      "/lste/persistent_execution/plan_event");
  ros::param::param<std::string>(
      "/move_base/persistent_target_plan_result_topic", target_plan_result_topic,
      "/lste/persistent_execution/target_plan_result");
  ros::param::param("/move_base/streaming_goal_epsilon", target_goal_epsilon_,
                    0.05);
  target_goal_epsilon_ = std::max(0.001, target_goal_epsilon_);
  ros::param::param("/move_base/persistent_plan_equivalence_distance",
                    plan_equivalence_distance_, 0.25);
  plan_equivalence_distance_ = std::max(0.02, plan_equivalence_distance_);
  // Navfn rounds an endpoint to occupancy-grid cells. On a 0.1 m map the
  // same geometric frontier can therefore alternate between diagonal cells
  // 0.141 m apart as SLAM updates. A visual target remains exact at
  // target_goal_epsilon_, but frontier plan equivalence must absorb that
  // lattice representation error or it will reset TEB's elastic band every
  // time the global planner recomputes the unchanged corridor.
  const double map_cell_diagonal = std::sqrt(2.0) *
      costmap_ros->getCostmap()->getResolution();
  frontier_endpoint_equivalence_distance_ = std::max(
      target_goal_epsilon_, map_cell_diagonal + 0.01);
  // Navfn begins each refreshed plan at the latest robot pose. The first few
  // decimetres therefore move with normal odometry/SLAM correction even when
  // the route corridor ahead is unchanged. Local TEB owns that immediate
  // horizon, so excluding it from global-route equivalence avoids resetting
  // an elastic band for a mere resample while still comparing every later
  // branch and the exact endpoint.
  ros::param::param("/move_base/persistent_plan_equivalence_prefix_distance",
                    plan_equivalence_prefix_distance_, 0.35);
  plan_equivalence_prefix_distance_ =
      std::max(0.0, plan_equivalence_prefix_distance_);

  // Use the original plugin name: all existing TebLocalPlannerROS parameters
  // remain valid, so this wrapper changes lifecycle semantics only.
  teb_.initialize("TebLocalPlannerROS", tf, costmap_ros);
  task_done_subscriber_ = private_nh.subscribe(
      task_done_topic, 1, &PersistentTebLocalPlanner::onTaskDone, this);
  std::string mission_command_topic;
  ros::param::param<std::string>(
      "/move_base/persistent_mission_command_topic", mission_command_topic,
      "/lste/persistent_execution/mission_command");
  mission_command_subscriber_ = private_nh.subscribe(
      mission_command_topic, 1, &PersistentTebLocalPlanner::onMissionCommand,
      this);
  installed_target_command_subscriber_ = private_nh.subscribe(
      installed_target_command_topic, 1,
      &PersistentTebLocalPlanner::onInstalledTargetCommand, this);
  target_approach_publisher_ = private_nh.advertise<geometry_msgs::PoseStamped>(
      target_approach_topic, 1, false);
  target_approach_result_publisher_ = private_nh.advertise<std_msgs::String>(
      target_plan_result_topic, 10, false);
  frontier_endpoint_publisher_ = private_nh.advertise<geometry_msgs::PoseStamped>(
      frontier_endpoint_topic, 1, false);
  plan_event_publisher_ = private_nh.advertise<std_msgs::String>(
      plan_event_topic, 10, false);
  std::string planner_command_contract_topic;
  ros::param::param<std::string>(
      "/move_base/persistent_planner_command_contract_topic",
      planner_command_contract_topic,
      "/lste/persistent_execution/planner_command_contract");
  planner_command_contract_publisher_ = private_nh.advertise<PlannerCommandContract>(
      planner_command_contract_topic, 20, false);
  initialized_ = true;
  ROS_INFO_STREAM("PersistentTebLocalPlanner initialized: persistent_execution="
                  << (persistent_execution_ ? "true" : "false"));
}

bool PersistentTebLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  plan_received_count_.fetch_add(1, std::memory_order_relaxed);
  bool retained_equivalent = false;
  bool emit_equivalent_event = false;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    // move_base invokes the global planner at its configured rate. In the
    // persistent architecture that can produce a freshly allocated Navfn
    // vector for the exact same remaining route every 50 ms. Feeding each
    // duplicate to TEB resets its elastic band and creates visible zero-vel
    // pulses. Preserve TEB's current band only when the new path is wholly
    // geometrically covered by it; a changed endpoint or a real detour still
    // installs immediately. Local costmap collision checking remains in TEB's
    // 20 Hz compute cycle, independently of this global-path gate.
    const bool mission_transaction_pending = persistent_execution_ &&
        latest_mission_transaction_.load(std::memory_order_acquire) >
            installed_mission_transaction_.load(std::memory_order_acquire);
    if (persistent_execution_ && !mission_transaction_pending &&
        planIsEquivalentLocked(plan)) {
      plan_equivalent_count_.fetch_add(1, std::memory_order_relaxed);
      retained_equivalent = true;
      const ros::WallTime now = ros::WallTime::now();
      if (last_equivalent_plan_event_wall_.isZero() ||
          (now - last_equivalent_plan_event_wall_).toSec() >= 1.0) {
        last_equivalent_plan_event_wall_ = now;
        emit_equivalent_event = true;
      }
    }
  }
  if (retained_equivalent) {
    if (emit_equivalent_event) {
      publishPlanEvent("equivalent_retained", plan);
    }
    return true;
  }
  // Navfn preserves the planning request timestamp on its poses.  In this
  // stack the request can be stamped a millisecond after the latest map->odom
  // transform, making TEB reject an otherwise valid path for one control
  // cycle with a future-extrapolation error.  A global route is geometric
  // data, not a time-indexed trajectory: zero requests the latest transform
  // and keeps the route in the correct current odom frame.
  std::vector<geometry_msgs::PoseStamped> teb_plan = plan;
  for (geometry_msgs::PoseStamped& pose : teb_plan) {
    pose.header.stamp = ros::Time(0);
  }
  const bool accepted = teb_.setPlan(teb_plan);
  if (!accepted) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    applied_plan_ = std::move(teb_plan);
    if (!applied_plan_.empty()) {
      current_plan_goal_ = applied_plan_.back();
    }
    // The endpoint report is scoped to this installed path.  Increment while
    // holding the same mutex used by the reporter, so it cannot associate a
    // new goal with the preceding path version.
    route_version_.fetch_add(1, std::memory_order_relaxed);
    installed_mission_transaction_.store(
        latest_mission_transaction_.load(std::memory_order_acquire),
        std::memory_order_release);
    // A newly installed route owns the controller again. Equivalent Navfn
    // refreshes deliberately retain the hold above and do not reset TEB.
    terminal_hold_active_.store(false, std::memory_order_release);
    hard_reset_hold_.store(false, std::memory_order_release);
  }
  plan_installed_count_.fetch_add(1, std::memory_order_relaxed);
  route_geometry_hash_.store(geometryHash(plan), std::memory_order_relaxed);
  publishPlanEvent("installed", plan);
  return true;
}

void PersistentTebLocalPlanner::publishPlannerCommandContract(
    const geometry_msgs::Twist& command, uint8_t state,
    const std::string& reason) {
  if (!planner_command_contract_publisher_) {
    return;
  }
  PlannerCommandContract message;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    message.transaction_id = latest_mission_transaction_.load(
        std::memory_order_acquire);
    message.route_id = latest_route_id_;
    message.graph_transaction_id = latest_graph_transaction_id_;
    message.map_epoch = latest_map_epoch_;
    message.origin_epoch = latest_origin_epoch_;
    message.projection_epoch = latest_projection_epoch_;
    message.lifecycle_transaction_id = latest_lifecycle_transaction_id_;
    message.action_generation = latest_action_generation_;
  }
  message.planner_sequence = planner_command_sequence_.fetch_add(
      1, std::memory_order_relaxed) + 1;
  message.command = command;
  message.state = state;
  message.reason = reason;
  message.producer = "persistent_teb_local_planner";
  planner_command_contract_publisher_.publish(message);
}

void PersistentTebLocalPlanner::publishPlanEvent(
    const char* event,
    const std::vector<geometry_msgs::PoseStamped>& plan) const {
  if (!plan_event_publisher_) {
    return;
  }
  std::ostringstream payload;
  payload << "{\"event\":\"" << event << "\",\"persistent\":"
          << (persistent_execution_ ? "true" : "false")
          << ",\"received\":"
          << plan_received_count_.load(std::memory_order_relaxed)
          << ",\"equivalent_retained\":"
          << plan_equivalent_count_.load(std::memory_order_relaxed)
          << ",\"installed\":"
          << plan_installed_count_.load(std::memory_order_relaxed)
          << ",\"route_version\":"
          << route_version_.load(std::memory_order_relaxed)
          << ",\"geometry_hash\":\""
          << route_geometry_hash_.load(std::memory_order_relaxed) << "\""
          << ",\"poses\":" << plan.size();
  if (!plan.empty()) {
    const geometry_msgs::PoseStamped& goal = plan.back();
    payload << ",\"goal\":[" << goal.pose.position.x << ","
            << goal.pose.position.y << "]";
  }
  payload << "}";
  std_msgs::String message;
  message.data = payload.str();
  plan_event_publisher_.publish(message);
}

uint64_t PersistentTebLocalPlanner::geometryHash(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  // Quantize to the 0.1 m Navfn grid before hashing. The hash is telemetry,
  // not a safety predicate; its purpose is to reveal meaningful corridor
  // replacements separately from per-message timestamps.
  uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](int64_t value) {
    const uint64_t encoded = static_cast<uint64_t>(value);
    for (size_t byte = 0; byte < sizeof(encoded); ++byte) {
      hash ^= (encoded >> (byte * 8U)) & 0xffU;
      hash *= 1099511628211ULL;
    }
  };
  mix(static_cast<int64_t>(plan.size()));
  for (const geometry_msgs::PoseStamped& pose : plan) {
    mix(static_cast<int64_t>(std::llround(pose.pose.position.x * 10.0)));
    mix(static_cast<int64_t>(std::llround(pose.pose.position.y * 10.0)));
  }
  return hash;
}

bool PersistentTebLocalPlanner::planIsEquivalentLocked(
    const std::vector<geometry_msgs::PoseStamped>& candidate) const {
  if (candidate.empty() || applied_plan_.empty()) {
    return false;
  }
  const geometry_msgs::PoseStamped& candidate_goal = candidate.back();
  const geometry_msgs::PoseStamped& applied_goal = applied_plan_.back();
  if (candidate_goal.header.frame_id != applied_goal.header.frame_id) {
    return false;
  }
  // A target's endpoint is part of its perception-to-navigation identity and
  // must stay exact. Frontier endpoints are grid-derived exploration hints,
  // so use the map-resolution bound above only when the *currently applied*
  // route is not the installed visual target.
  const bool applied_plan_is_target =
      has_installed_target_goal_ &&
      applied_goal.header.frame_id == installed_target_goal_.header.frame_id &&
      std::hypot(
          applied_goal.pose.position.x - installed_target_goal_.pose.position.x,
          applied_goal.pose.position.y - installed_target_goal_.pose.position.y) <=
          target_goal_epsilon_;
  const double endpoint_equivalence_distance = applied_plan_is_target
      ? target_goal_epsilon_
      : frontier_endpoint_equivalence_distance_;
  if (std::hypot(candidate_goal.pose.position.x - applied_goal.pose.position.x,
                 candidate_goal.pose.position.y - applied_goal.pose.position.y) >
      endpoint_equivalence_distance) {
    return false;
  }

  // Navfn paths are normally sampled at the 0.1 m map resolution. Sampling at
  // most 25 deterministic points keeps the comparison cheap at 20 Hz while
  // still rejecting a newly mapped doorway detour instead of treating it as a
  // duplicate of the old corridor route.
  const size_t stride = std::max<size_t>(1, candidate.size() / 24);
  double traversed = 0.0;
  for (size_t i = 0; i < candidate.size(); i += stride) {
    if (i > 0) {
      const geometry_msgs::Point& previous = candidate[i - stride].pose.position;
      const geometry_msgs::Point& current = candidate[i].pose.position;
      traversed += std::hypot(current.x - previous.x, current.y - previous.y);
    }
    if (traversed < plan_equivalence_prefix_distance_) {
      continue;
    }
    const geometry_msgs::Point& point = candidate[i].pose.position;
    double nearest = std::numeric_limits<double>::infinity();
    for (const geometry_msgs::PoseStamped& existing : applied_plan_) {
      nearest = std::min(
          nearest,
          std::hypot(point.x - existing.pose.position.x,
                     point.y - existing.pose.position.y));
    }
    if (nearest > plan_equivalence_distance_) {
      return false;
    }
  }
  return true;
}

bool PersistentTebLocalPlanner::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
  if (hard_reset_hold_.load(std::memory_order_acquire)) {
    cmd_vel = geometry_msgs::Twist();
    publishPlannerCommandContract(
        cmd_vel, PlannerCommandContract::STATE_INVALIDATED, "hard_reset");
    return true;
  }
  if (!persistent_execution_) {
    return teb_.computeVelocityCommands(cmd_vel);
  }
  // Endpoint ownership has already been transferred to the graph. Keep the
  // MoveBase action healthy while the graph commits a successor route. This
  // prevents a transient TEB false result at the old endpoint from entering
  // recovery; a real successor clears the hold in setPlan.
  if (terminal_hold_active_.load(std::memory_order_acquire)) {
    if (isInstalledTargetPlanLocked()) {
      reportTargetApproachIfReady();
    } else {
      reportFrontierEndpointIfReady();
    }
    cmd_vel = geometry_msgs::Twist();
    publishPlannerCommandContract(
        cmd_vel, PlannerCommandContract::STATE_ZERO, "terminal_hold");
    return true;
  }
  // Do not call the underlying TEB while a newer mission transaction is still
  // waiting for its Navfn plan.  TEB has no transaction identity and would
  // otherwise return the predecessor route's command, which downstream nodes
  // could mistake for the successor's first command and use to re-arm stale
  // feedback.  A zero command is the only valid output for this boundary.
  if (latest_mission_transaction_.load(std::memory_order_acquire) >
      installed_mission_transaction_.load(std::memory_order_acquire)) {
    cmd_vel = geometry_msgs::Twist();
    publishPlannerCommandContract(
        cmd_vel, PlannerCommandContract::STATE_ZERO, "mission_plan_pending");
    return true;
  }

  const bool command_available = teb_.computeVelocityCommands(cmd_vel);

  // Mission, terminal and task state can change while TEB is computing.  The
  // second check closes that short callback/control-cycle race before the
  // command reaches move_base's remapped planner output topic.
  if (task_done_.load(std::memory_order_acquire) ||
      terminal_hold_active_.load(std::memory_order_acquire) ||
      latest_mission_transaction_.load(std::memory_order_acquire) >
          installed_mission_transaction_.load(std::memory_order_acquire)) {
    cmd_vel = geometry_msgs::Twist();
    publishPlannerCommandContract(
        cmd_vel, PlannerCommandContract::STATE_ZERO, "mission_boundary");
    return true;
  }
  // TEB returns false at every XY terminal. In the persistent architecture an
  // action is a lease for the whole mission, so both a temporary frontier and
  // a target approach must keep move_base's control loop healthy while the
  // stream supplies the next plan. Returning false here makes move_base start
  // recovery on a perfectly valid reached frontier whenever no successor has
  // been installed in the same control cycle. A target approach additionally
  // emits its one-shot observation event; a frontier simply holds zero.
  if (teb_.isGoalReached()) {
    if (isInstalledTargetPlanLocked()) {
      reportTargetApproachIfReady();
    } else {
      reportFrontierEndpointIfReady();
    }
    cmd_vel = geometry_msgs::Twist();
    publishPlannerCommandContract(
        cmd_vel, PlannerCommandContract::STATE_ZERO, "endpoint_reached");
    return true;
  }
  if (!command_available) {
    // In persistent mode a transient TEB failure is input to the bridge's
    // progress watchdog, not a MoveBase recovery request.  Keep the action
    // server's control loop responsive so an explicit cancel can reach its
    // terminal state and the successor gate can make progress.
    cmd_vel = geometry_msgs::Twist();
  }
  publishPlannerCommandContract(
      cmd_vel,
      (command_available &&
       (std::abs(cmd_vel.linear.x) > 0.001 ||
        std::abs(cmd_vel.angular.z) > 0.01))
          ? PlannerCommandContract::STATE_ACTIVE
          : PlannerCommandContract::STATE_ZERO,
      command_available ? "planner_command" : "planner_no_command");
  return true;
}

bool PersistentTebLocalPlanner::isGoalReached() {
  if (hard_reset_hold_.load(std::memory_order_acquire)) {
    return false;
  }
  if (!persistent_execution_) {
    return teb_.isGoalReached();
  }
  // The action is a controller lease, rather than ownership of one frontier
  // endpoint. Reaching a target must notify GoalManager without ending that
  // lease; otherwise actionlib's terminal callback dispatches a second action
  // and reintroduces the observable stop boundary we are eliminating.
  if (task_done_.load()) {
    return true;
  }
  if (persistent_execution_ &&
      latest_mission_transaction_.load(std::memory_order_acquire) >
          installed_mission_transaction_.load(std::memory_order_acquire)) {
    return false;
  }
  if (terminal_hold_active_.load(std::memory_order_acquire)) {
    // The action remains a mission lease until a successor route is installed.
    return false;
  }
  if (teb_.isGoalReached()) {
    if (isInstalledTargetPlanLocked()) {
      reportTargetApproachIfReady();
    } else {
      reportFrontierEndpointIfReady();
    }
  }
  return false;
}

bool PersistentTebLocalPlanner::reportFrontierEndpointIfReady() {
  if (!frontier_endpoint_publisher_) {
    return false;
  }
  geometry_msgs::PoseStamped endpoint;
  uint64_t route_version = 0;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (latest_mission_transaction_.load(std::memory_order_acquire) >
        installed_mission_transaction_.load(std::memory_order_acquire)) {
      return false;
    }
    if (current_plan_goal_.header.frame_id.empty()) {
      return false;
    }
    route_version = route_version_.load(std::memory_order_relaxed);
    const bool already_reported = has_reported_frontier_goal_ &&
        reported_frontier_route_version_ == route_version;
    if (already_reported) {
      return true;
    }
    reported_frontier_goal_ = current_plan_goal_;
    reported_frontier_route_version_ = route_version;
    has_reported_frontier_goal_ = true;
    endpoint = current_plan_goal_;
    endpoint.header.stamp = ros::Time::now();
    terminal_hold_active_.store(true, std::memory_order_release);
  }
  frontier_endpoint_publisher_.publish(endpoint);
  ROS_INFO_STREAM(
      "PersistentTebLocalPlanner reported frontier endpoint: endpoint=("
      << endpoint.pose.position.x << "," << endpoint.pose.position.y
      << ") route_version=" << route_version);
  return true;
}

bool PersistentTebLocalPlanner::reportTargetApproachIfReady() {
  if (!isInstalledTargetPlanLocked()) {
    return false;
  }
  geometry_msgs::PoseStamped approach;
  uint32_t transaction_id = 0;
  uint64_t origin_epoch = 0;
  uint64_t projection_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    // The installed-target callback can race this control cycle. Recheck the
    // endpoint under the same lock before emitting an approach terminal.
    // ``move_base`` may rewrite PoseStamped.header.seq while transforming a
    // Navfn path, so header metadata is not a reliable cross-plugin identity.
    if (!has_installed_target_goal_ ||
        current_plan_goal_.header.frame_id != installed_target_goal_.header.frame_id ||
        std::hypot(current_plan_goal_.pose.position.x -
                       installed_target_goal_.pose.position.x,
                   current_plan_goal_.pose.position.y -
                       installed_target_goal_.pose.position.y) >
            target_goal_epsilon_) {
      return false;
    }
    const bool already_reported = has_reported_target_goal_ &&
        reported_target_goal_.header.frame_id ==
            installed_target_goal_.header.frame_id &&
        std::hypot(reported_target_goal_.pose.position.x -
                       installed_target_goal_.pose.position.x,
                   reported_target_goal_.pose.position.y -
                       installed_target_goal_.pose.position.y) <=
            target_goal_epsilon_;
    if (!already_reported) {
      reported_target_goal_ = installed_target_goal_;
      has_reported_target_goal_ = true;
    }
    approach = installed_target_goal_;
    approach.header.stamp = ros::Time::now();
    transaction_id = installed_target_transaction_;
    origin_epoch = installed_target_origin_epoch_;
    projection_epoch = installed_target_projection_epoch_;
    terminal_hold_active_.store(true, std::memory_order_release);
    if (already_reported) {
      return true;
    }
  }
  target_approach_publisher_.publish(approach);
  if (transaction_id > 0 && target_approach_result_publisher_) {
    std::ostringstream payload;
    payload << "{\"event\":\"target_approach\",\"transaction_id\":"
            << transaction_id << ",\"frame_id\":\""
            << approach.header.frame_id << "\",\"goal\":["
            << approach.pose.position.x << "," << approach.pose.position.y
            << "],\"origin_epoch\":" << origin_epoch
            << ",\"projection_epoch\":" << projection_epoch << "}";
    std_msgs::String result;
    result.data = payload.str();
    target_approach_result_publisher_.publish(result);
  }
  ROS_INFO_STREAM("PersistentTebLocalPlanner reported target approach: endpoint=("
                  << approach.pose.position.x << "," << approach.pose.position.y
                  << ") transaction=" << transaction_id << " origin_epoch="
                  << origin_epoch << " projection_epoch="
                  << projection_epoch);
  return true;
}

void PersistentTebLocalPlanner::onTaskDone(
    const std_msgs::BoolConstPtr& message) {
  task_done_.store(message->data);
}

void PersistentTebLocalPlanner::onMissionCommand(
    const PersistentGoalCommandConstPtr& message) {
  if (message->kind == PersistentGoalCommand::KIND_CLEAR &&
      message->transaction_id == 0) {
    publishPlannerCommandContract(
        geometry_msgs::Twist(), PlannerCommandContract::STATE_INVALIDATED,
        "hard_reset_command");
    std::lock_guard<std::mutex> lock(plan_mutex_);
    has_installed_target_goal_ = false;
    has_reported_target_goal_ = false;
    has_reported_frontier_goal_ = false;
    installed_target_transaction_ = 0;
    installed_target_origin_epoch_ = 0;
    installed_target_projection_epoch_ = 0;
    reported_frontier_route_version_ = 0;
    latest_route_id_ = 0;
    latest_graph_transaction_id_ = 0;
    latest_map_epoch_ = 0;
    latest_origin_epoch_ = 0;
    latest_projection_epoch_ = 0;
    latest_lifecycle_transaction_id_.clear();
    latest_action_generation_ = 0;
    current_plan_goal_ = geometry_msgs::PoseStamped();
    applied_plan_.clear();
    terminal_hold_active_.store(true, std::memory_order_release);
    hard_reset_hold_.store(true, std::memory_order_release);
    publishPlanEvent("hard_reset_cleared", applied_plan_);
    ROS_INFO_STREAM(
        "PersistentTebLocalPlanner cleared mission route for hard reset "
        << "origin_epoch=" << message->origin_epoch
        << " projection_epoch=" << message->projection_epoch);
    return;
  }
  if (message->kind != PersistentGoalCommand::KIND_MISSION ||
      message->transaction_id == 0) {
    return;
  }
  bool publish_mission_boundary = false;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    const bool same_mission_identity =
        message->transaction_id == latest_mission_transaction_ &&
        message->route_id == latest_route_id_ &&
        message->graph_transaction_id == latest_graph_transaction_id_ &&
        message->map_epoch == latest_map_epoch_ &&
        message->origin_epoch == latest_origin_epoch_ &&
        message->projection_epoch == latest_projection_epoch_ &&
        message->lifecycle_transaction_id == latest_lifecycle_transaction_id_ &&
        message->action_generation == latest_action_generation_;
    if (same_mission_identity) {
      return;
    }
    if (message->transaction_id < latest_mission_transaction_) {
      ROS_WARN_STREAM("PersistentTebLocalPlanner ignored stale mission "
                      << "transaction=" << message->transaction_id << " < "
                      << latest_mission_transaction_);
      return;
    }
    // A new mission transaction may reuse the exact same Navfn geometry (for
    // example, a second Portal probe at the same endpoint).  Path equivalence
    // must not suppress that transaction's endpoint report.
    latest_mission_transaction_ = message->transaction_id;
    latest_route_id_ = message->route_id;
    latest_graph_transaction_id_ = message->graph_transaction_id;
    latest_map_epoch_ = message->map_epoch;
    latest_lifecycle_transaction_id_ = message->lifecycle_transaction_id;
    latest_action_generation_ = message->action_generation;
    latest_origin_epoch_ = message->origin_epoch;
    latest_projection_epoch_ = message->projection_epoch;
    has_reported_frontier_goal_ = false;
    reported_frontier_route_version_ = 0;
    terminal_hold_active_.store(false, std::memory_order_release);
    publish_mission_boundary = true;
  }
  // Publish immediately after installing the new wire identity.  Waiting for
  // the next move_base compute cycle leaves Supervisor holding the predecessor
  // contract while the successor action is being adopted.
  if (publish_mission_boundary) {
    publishPlannerCommandContract(
        geometry_msgs::Twist(), PlannerCommandContract::STATE_ZERO,
        "mission_boundary");
  }
}

void PersistentTebLocalPlanner::onInstalledTargetCommand(
    const PersistentGoalCommandConstPtr& message) {
  if (message->kind == PersistentGoalCommand::KIND_CLEAR) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    const uint32_t sequence = message->transaction_id;
    // A clear is a tombstone for both provisional and already-installed
    // target plans.  It may arrive from the StreamingNavfnPlanner after a
    // target-plan failure, so the local planner must stop classifying the
    // cached route as a semantic target immediately.
    if (sequence > 0 && installed_target_transaction_ > sequence) {
      ROS_DEBUG_STREAM("PersistentTebLocalPlanner ignored stale target clear "
                       << sequence << " < "
                       << installed_target_transaction_);
      return;
    }
    has_installed_target_goal_ = false;
    installed_target_transaction_ = 0;
    installed_target_origin_epoch_ = message->origin_epoch;
    installed_target_projection_epoch_ = message->projection_epoch;
    has_reported_target_goal_ = false;
    if (sequence == 0) {
      current_plan_goal_ = geometry_msgs::PoseStamped();
      applied_plan_.clear();
      terminal_hold_active_.store(true, std::memory_order_release);
      hard_reset_hold_.store(true, std::memory_order_release);
      publishPlanEvent("hard_reset_cleared", applied_plan_);
    } else {
      terminal_hold_active_.store(false, std::memory_order_release);
    }
    ROS_INFO_STREAM("PersistentTebLocalPlanner cleared installed target "
                    << "transaction=" << sequence << " origin_epoch="
                    << message->origin_epoch << " projection_epoch="
                    << message->projection_epoch);
    return;
  }
  if (message->kind != PersistentGoalCommand::KIND_TARGET_INSTALLED ||
      message->transaction_id == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(plan_mutex_);
  const bool changed_target = !has_installed_target_goal_ ||
      installed_target_transaction_ != message->transaction_id ||
      installed_target_goal_.header.frame_id != message->goal.header.frame_id ||
      std::hypot(installed_target_goal_.pose.position.x - message->goal.pose.position.x,
                 installed_target_goal_.pose.position.y - message->goal.pose.position.y) >
          target_goal_epsilon_;
  installed_target_goal_ = message->goal;
  installed_target_transaction_ = message->transaction_id;
  installed_target_origin_epoch_ = message->origin_epoch;
  installed_target_projection_epoch_ = message->projection_epoch;
  has_installed_target_goal_ = true;
  if (changed_target) {
    has_reported_target_goal_ = false;
    terminal_hold_active_.store(false, std::memory_order_release);
  }
}

bool PersistentTebLocalPlanner::isInstalledTargetPlanLocked() {
  std::lock_guard<std::mutex> lock(plan_mutex_);
  if (!has_installed_target_goal_ || current_plan_goal_.header.frame_id.empty()) {
    return false;
  }
  if (current_plan_goal_.header.frame_id != installed_target_goal_.header.frame_id) {
    return false;
  }
  return std::hypot(
             current_plan_goal_.pose.position.x -
                 installed_target_goal_.pose.position.x,
             current_plan_goal_.pose.position.y -
                 installed_target_goal_.pose.position.y) <=
         target_goal_epsilon_;
}

StreamingNavfnPlanner::StreamingNavfnPlanner()
    : initialized_(false),
      has_mission_goal_(false),
      has_target_goal_(false),
      has_validated_target_plan_(false),
      has_active_target_mission_(false),
      plan_dirty_(true),
      goal_epsilon_(0.05),
      replan_distance_(0.50),
      max_replan_interval_(2.0),
      latest_mission_sequence_(0),
      latest_mission_origin_epoch_(0),
      latest_mission_projection_epoch_(0),
      latest_target_sequence_(0),
      active_target_mission_sequence_(0),
      validated_target_sequence_(0),
      failed_target_sequence_(0),
      latest_target_map_epoch_(0),
      latest_target_origin_epoch_(0),
      latest_target_projection_epoch_(0),
      latest_target_graph_transaction_id_(0),
      latest_target_route_id_(0),
      latest_target_lifecycle_transaction_id_(),
      latest_target_action_generation_(0),
      latest_target_route_kind_(),
      latest_target_mission_route_kind_(),
      latest_target_graph_action_(),
      latest_target_graph_obligation_kind_(),
      mission_generation_(0),
      target_failure_reported_generation_(0) {}

void StreamingNavfnPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("StreamingNavfnPlanner is already initialized");
    return;
  }

  std::string mission_command_topic;
  ros::param::param<std::string>(
      "/move_base/streaming_mission_command_topic", mission_command_topic,
      "/lste/persistent_execution/mission_command");
  std::string target_command_topic;
  std::string installed_target_goal_topic;
  std::string installed_target_command_topic;
  std::string target_plan_result_topic;
  ros::param::param<std::string>(
      "/move_base/streaming_target_command_topic", target_command_topic,
      "/lste/persistent_execution/target_command");
  ros::param::param<std::string>(
      "/move_base/persistent_installed_target_goal_topic",
      installed_target_goal_topic,
      "/lste/persistent_execution/installed_target_goal");
  ros::param::param<std::string>(
      "/move_base/persistent_installed_target_command_topic",
      installed_target_command_topic,
      "/lste/persistent_execution/installed_target_command");
  ros::param::param<std::string>(
      "/move_base/persistent_target_plan_result_topic",
      target_plan_result_topic,
      "/lste/persistent_execution/target_plan_result");
  ros::param::param("/move_base/streaming_goal_epsilon", goal_epsilon_, 0.05);
  ros::param::param("/move_base/streaming_replan_distance", replan_distance_,
                    0.50);
  ros::param::param("/move_base/streaming_max_replan_interval",
                    max_replan_interval_, 2.0);
  goal_epsilon_ = std::max(0.001, goal_epsilon_);
  replan_distance_ = std::max(0.05, replan_distance_);
  max_replan_interval_ = std::max(0.10, max_replan_interval_);

  // Keep Navfn's established parameter namespace. The router owns goal
  // streaming, not path-search policy or cost interpretation.
  navfn_.initialize("NavfnROS", costmap_ros);
  global_frame_ = costmap_ros->getGlobalFrameID();
  transform_listener_.reset(new tf::TransformListener());
  ros::NodeHandle nh;
  mission_command_subscriber_ = nh.subscribe(
      mission_command_topic, 1, &StreamingNavfnPlanner::onMissionCommand,
      this);
  target_command_subscriber_ = nh.subscribe(
      target_command_topic, 1, &StreamingNavfnPlanner::onTargetCommand,
      this);
  installed_target_goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>(
      installed_target_goal_topic, 1, true);
  installed_target_command_publisher_ = nh.advertise<PersistentGoalCommand>(
      installed_target_command_topic, 1, true);
  target_plan_result_publisher_ = nh.advertise<std_msgs::String>(
      target_plan_result_topic, 10, false);
  initialized_ = true;
  ROS_INFO_STREAM("StreamingNavfnPlanner initialized: mission_command="
                  << mission_command_topic << " target_command="
                  << target_command_topic
                  << " result_topic=" << target_plan_result_topic
                  << " global_frame=" << global_frame_
                  << " epsilon=" << goal_epsilon_);
}

void StreamingNavfnPlanner::publishTargetPlanResult(
    const char* event, uint32_t transaction_id,
    const geometry_msgs::PoseStamped& goal) const {
  if (!target_plan_result_publisher_) {
    return;
  }
  std::ostringstream payload;
  payload << "{\"event\":\"" << event << "\",\"transaction_id\":"
          << transaction_id << ",\"frame_id\":\"" << goal.header.frame_id
          << "\",\"map_epoch\":" << latest_target_map_epoch_
          << ",\"origin_epoch\":" << latest_target_origin_epoch_
          << ",\"projection_epoch\":" << latest_target_projection_epoch_
          << ",\"graph_transaction_id\":"
          << latest_target_graph_transaction_id_
          << ",\"route_kind\":\"" << latest_target_route_kind_
          << "\",\"mission_route_kind\":\""
          << latest_target_mission_route_kind_
          << "\",\"graph_action\":\"" << latest_target_graph_action_
          << "\",\"graph_obligation_kind\":\""
          << latest_target_graph_obligation_kind_
          << "\",\"goal\":[" << goal.pose.position.x << ","
          << goal.pose.position.y << "]}";
  std_msgs::String message;
  message.data = payload.str();
  target_plan_result_publisher_.publish(message);
}

bool StreamingNavfnPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& action_goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  geometry_msgs::PoseStamped selected_goal;
  geometry_msgs::PoseStamped global_goal;
  bool must_replan = false;
  bool selected_is_target = false;
  bool retained_previous_route = false;
  uint32_t selected_target_sequence = 0;
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A target stream is a two-phase request.  It can temporarily outrank the
    // last bridge-approved mission only while it carries a newer transaction
    // number; once the bridge publishes that same number on the approved
    // mission topic, normal mission ownership resumes.  This prevents a
    // direct /lste/final_goal update from bypassing target-plan confirmation.
    const bool provisional_target = has_target_goal_ &&
        latest_target_sequence_ > 0 &&
        (!has_mission_goal_ || latest_mission_sequence_ == 0 ||
         latest_target_sequence_ > latest_mission_sequence_);
    if (provisional_target) {
      selected_target_sequence = latest_target_sequence_;
      // A rejected transaction stays rejected until GoalManager emits a newer
      // command. Without this guard, each planner tick re-runs Navfn against
      // a target which the bridge has already released back to exploration.
      if (failed_target_sequence_ == selected_target_sequence) {
        if (!cached_plan_.empty()) {
          plan = cached_plan_;
          return true;
        }
        return false;
      }
      // Navfn has already validated this exact target and notified the
      // bridge. Keep executing the last approved route while waiting for the
      // bridge to publish the matching approved mission transaction. This is
      // the actual two-phase boundary: a provisional path never reaches TEB
      // when a previous route is available.
      if (has_validated_target_plan_ &&
          validated_target_sequence_ == selected_target_sequence) {
        if (!cached_plan_.empty()) {
          plan = cached_plan_;
          return true;
        }
        // There is no prior route only during a cold-start visual mission.
        // The candidate was still Navfn-validated exactly before it can be
        // returned, so this is a safe bootstrap exception to the stream lease.
        plan = validated_target_plan_;
        return !plan.empty();
      }
    }
    // After the bridge approves an installed target, its provisional request
    // is deliberately cleared. The approved mission still has to keep the
    // target's exact-endpoint rule on every later Navfn replan; otherwise a
    // map update can silently move the path endpoint to Navfn's tolerance
    // boundary and desynchronize GoalManager, TEB, and Navfn again.
    const bool approved_target_mission = has_active_target_mission_ &&
        has_mission_goal_ &&
        active_target_mission_sequence_ == latest_mission_sequence_;
    if (!provisional_target && approved_target_mission) {
      selected_target_sequence = active_target_mission_sequence_;
    }
    selected_goal = provisional_target ? target_goal_ :
        (has_mission_goal_ ? mission_goal_ : action_goal);
    must_replan = plan_dirty_ || shouldReplanLocked(start, selected_goal);
    if (!must_replan && !cached_plan_.empty()) {
      plan = cached_plan_;
      return true;
    }
    selected_is_target = provisional_target || approved_target_mission;
    generation = mission_generation_;
  }

  if (!transformToGlobalFrame(selected_goal, global_goal)) {
    ROS_WARN_THROTTLE(1.0,
                      "StreamingNavfnPlanner could not transform mission goal to %s",
                      global_frame_.c_str());
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> candidate;
  const bool navfn_success = navfn_.makePlan(start, global_goal, candidate) &&
      !candidate.empty();
  // A target is an exact execution contract.  Navfn can otherwise return a
  // path that terminates at its internal tolerance boundary, which would make
  // the local planner announce arrival at a point GoalManager never approved.
  // Keep the previous frontier path in that case and report a route failure
  // for the original transaction instead.
  const bool target_endpoint_exact = !selected_is_target ||
      (navfn_success && std::hypot(
          candidate.back().pose.position.x - global_goal.pose.position.x,
          candidate.back().pose.position.y - global_goal.pose.position.y) <=
          goal_epsilon_);
  if (!navfn_success || !target_endpoint_exact) {
    bool report_target_failure = false;
    bool retained_previous_plan = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // A target request is speculative until Navfn has found a map-connected
      // route.  Keep a known-good route alive during that decision instead of
      // returning false to move_base: recovery would erase the action lease,
      // while TEB would otherwise sit at the endpoint of the old route with no
      // explanation of why the requested target was not installed.
      if (generation == mission_generation_) {
        if (selected_is_target) {
          has_target_goal_ = false;
          has_validated_target_plan_ = false;
          if (has_active_target_mission_ &&
              active_target_mission_sequence_ == selected_target_sequence) {
            has_active_target_mission_ = false;
          }
          if (failed_target_sequence_ != selected_target_sequence) {
            failed_target_sequence_ = selected_target_sequence;
            target_failure_reported_generation_ = generation;
            report_target_failure = true;
          }
        }
        if (!cached_plan_.empty()) {
          plan = cached_plan_;
          retained_previous_plan = true;
        }
      }
    }
    if (report_target_failure) {
      publishTargetPlanResult("target_plan_failed", selected_target_sequence,
                              global_goal);
    }
    ROS_WARN_THROTTLE(
        1.0,
        "StreamingNavfnPlanner could not install the current mission goal%s%s",
        selected_is_target && navfn_success ? " exactly" : "",
        retained_previous_plan ? "; retaining previous valid path" : "");
    return retained_previous_plan;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != mission_generation_) {
      // A detector update can race Navfn by one callback. Do not make
      // move_base enter recovery for a result we know is obsolete; retain the
      // last valid path and let the next planner tick install the newer one.
      ROS_DEBUG("StreamingNavfnPlanner discarded stale planning result");
      plan_dirty_ = true;
      if (!cached_plan_.empty()) {
        plan = cached_plan_;
        return true;
      }
      return false;
    }
    if (selected_is_target) {
      validated_target_plan_ = candidate;
      validated_target_goal_ = global_goal;
      has_validated_target_plan_ = true;
      validated_target_sequence_ = selected_target_sequence;
      plan_dirty_ = false;
      if (!cached_plan_.empty()) {
        // Do not expose the speculative path to move_base. The bridge will
        // publish the matching mission transaction after it receives the
        // installed-target acknowledgement below.
        plan = cached_plan_;
        retained_previous_route = true;
      } else {
        // See the cold-start exception above. Retain the validated candidate
        // as the initial route so the first action lease can stay alive.
        cached_plan_ = candidate;
        cached_goal_ = selected_goal;
        last_plan_start_ = start;
        last_plan_time_ = ros::WallTime::now();
        plan = cached_plan_;
      }
    } else {
      cached_plan_ = candidate;
      cached_goal_ = selected_goal;
      last_plan_start_ = start;
      last_plan_time_ = ros::WallTime::now();
      plan_dirty_ = false;
      plan = cached_plan_;
    }
  }
  if (selected_is_target) {
    geometry_msgs::PoseStamped installed = candidate.back();
    installed.header.stamp = ros::Time::now();
    installed_target_goal_publisher_.publish(installed);
    PersistentGoalCommand installed_command;
    installed_command.transaction_id = selected_target_sequence;
    installed_command.kind = PersistentGoalCommand::KIND_TARGET_INSTALLED;
    installed_command.route_id = latest_target_route_id_;
    installed_command.lifecycle_transaction_id =
        latest_target_lifecycle_transaction_id_;
    installed_command.action_generation = latest_target_action_generation_;
    installed_command.map_epoch = latest_target_map_epoch_;
    installed_command.origin_epoch = latest_target_origin_epoch_;
    installed_command.projection_epoch = latest_target_projection_epoch_;
    installed_command.graph_transaction_id = latest_target_graph_transaction_id_;
    installed_command.route_kind = latest_target_route_kind_;
    installed_command.mission_route_kind = latest_target_mission_route_kind_;
    installed_command.graph_action = latest_target_graph_action_;
    installed_command.graph_obligation_kind = latest_target_graph_obligation_kind_;
    installed_command.goal = installed;
    installed_target_command_publisher_.publish(installed_command);
    ROS_INFO_STREAM("StreamingNavfnPlanner validated target transaction="
                    << selected_target_sequence << " endpoint=("
                    << installed.pose.position.x << ","
                    << installed.pose.position.y << ") previous_route="
                    << (retained_previous_route ? "retained" : "bootstrap")
                    << " origin_epoch=" << latest_target_origin_epoch_
                    << " projection_epoch=" << latest_target_projection_epoch_);
    publishTargetPlanResult("target_plan_installed", selected_target_sequence,
                            installed);
  }
  return true;
}

void StreamingNavfnPlanner::onMissionCommand(
    const PersistentGoalCommandConstPtr& message) {
  if (message->kind == PersistentGoalCommand::KIND_CLEAR &&
      message->transaction_id == 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    has_mission_goal_ = false;
    has_target_goal_ = false;
    has_validated_target_plan_ = false;
    has_active_target_mission_ = false;
    plan_dirty_ = true;
    mission_goal_ = geometry_msgs::PoseStamped();
    target_goal_ = geometry_msgs::PoseStamped();
    validated_target_goal_ = geometry_msgs::PoseStamped();
    cached_goal_ = geometry_msgs::PoseStamped();
    last_plan_start_ = geometry_msgs::PoseStamped();
    cached_plan_.clear();
    validated_target_plan_.clear();
    latest_mission_sequence_ = 0;
    latest_mission_origin_epoch_ = 0;
    latest_mission_projection_epoch_ = 0;
    latest_target_sequence_ = 0;
    latest_target_map_epoch_ = 0;
    latest_target_origin_epoch_ = 0;
    latest_target_projection_epoch_ = 0;
    latest_target_graph_transaction_id_ = 0;
    latest_target_route_id_ = 0;
    latest_target_lifecycle_transaction_id_.clear();
    latest_target_action_generation_ = 0;
    latest_target_route_kind_.clear();
    latest_target_mission_route_kind_.clear();
    latest_target_graph_action_.clear();
    latest_target_graph_obligation_kind_.clear();
    active_target_mission_sequence_ = 0;
    validated_target_sequence_ = 0;
    failed_target_sequence_ = 0;
    ++mission_generation_;
    ROS_INFO_STREAM(
        "StreamingNavfnPlanner cleared mission and target routes for hard reset "
        << "origin_epoch=" << message->origin_epoch
        << " projection_epoch=" << message->projection_epoch);
    return;
  }
  if (message->kind != PersistentGoalCommand::KIND_MISSION ||
      message->transaction_id == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const uint32_t sequence = message->transaction_id;
  if (sequence > 0 && latest_mission_sequence_ > 0 &&
      sequence < latest_mission_sequence_) {
    ROS_WARN_STREAM("StreamingNavfnPlanner ignored stale approved mission "
                    << sequence << " < " << latest_mission_sequence_);
    return;
  }
  mission_goal_ = message->goal;
  latest_mission_sequence_ = sequence;
  latest_mission_origin_epoch_ = message->origin_epoch;
  latest_mission_projection_epoch_ = message->projection_epoch;
  has_mission_goal_ = true;
  // A target is provisional only while it is strictly newer than the approved
  // mission. Once the bridge repeats its transaction here, consume the exact
  // already-validated path and remove the speculative input from selection.
  if (has_target_goal_ && latest_target_sequence_ > 0 &&
      sequence >= latest_target_sequence_) {
    has_target_goal_ = false;
  }
  const bool approving_validated_target =
      sequence > 0 && has_validated_target_plan_ &&
      sequence == validated_target_sequence_;
  if (approving_validated_target) {
    cached_plan_ = validated_target_plan_;
    cached_goal_ = mission_goal_;
    last_plan_time_ = ros::WallTime::now();
    plan_dirty_ = false;
    has_validated_target_plan_ = false;
    has_active_target_mission_ = true;
    active_target_mission_sequence_ = sequence;
  } else {
    plan_dirty_ = true;
    // A different approved mission takes ownership from the prior visual
    // target. The command protocol has no separate target flag here; only a
    // matching previously validated target transaction may retain it.
    has_active_target_mission_ = false;
    active_target_mission_sequence_ = 0;
  }
  ++mission_generation_;
  ROS_INFO_STREAM("StreamingNavfnPlanner approved mission transaction="
                  << sequence << " endpoint=("
                  << message->goal.pose.position.x << ","
                  << message->goal.pose.position.y << ") origin_epoch="
                  << latest_mission_origin_epoch_ << " projection_epoch="
                  << latest_mission_projection_epoch_);
}

void StreamingNavfnPlanner::onTargetCommand(
    const PersistentGoalCommandConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const uint32_t sequence = message->transaction_id;
  if (message->kind == PersistentGoalCommand::KIND_CLEAR) {
    // The bridge publishes this tombstone after a target transaction resolves
    // or is superseded. It clears the latched request for a hot move_base
    // restart and prevents repeated validation of an old visual ray.
    const uint32_t clear_sequence =
        sequence > 0 ? sequence : latest_target_sequence_;
    if (sequence == 0) {
      has_mission_goal_ = false;
      has_target_goal_ = false;
      has_validated_target_plan_ = false;
      has_active_target_mission_ = false;
      plan_dirty_ = true;
      mission_goal_ = geometry_msgs::PoseStamped();
      target_goal_ = geometry_msgs::PoseStamped();
      validated_target_goal_ = geometry_msgs::PoseStamped();
      cached_goal_ = geometry_msgs::PoseStamped();
      last_plan_start_ = geometry_msgs::PoseStamped();
      cached_plan_.clear();
      validated_target_plan_.clear();
      latest_mission_sequence_ = 0;
      latest_mission_origin_epoch_ = 0;
      latest_mission_projection_epoch_ = 0;
      latest_target_sequence_ = 0;
      latest_target_map_epoch_ = 0;
      latest_target_origin_epoch_ = 0;
      latest_target_projection_epoch_ = 0;
      latest_target_graph_transaction_id_ = 0;
      latest_target_route_id_ = 0;
      latest_target_lifecycle_transaction_id_.clear();
      latest_target_action_generation_ = 0;
      latest_target_route_kind_.clear();
      latest_target_mission_route_kind_.clear();
      latest_target_graph_action_.clear();
      latest_target_graph_obligation_kind_.clear();
      active_target_mission_sequence_ = 0;
      validated_target_sequence_ = 0;
      failed_target_sequence_ = 0;
      ++mission_generation_;
      if (installed_target_command_publisher_) {
        PersistentGoalCommand installed_clear;
      installed_clear.kind = PersistentGoalCommand::KIND_CLEAR;
      installed_clear.transaction_id = 0;
      installed_clear.route_id = message->route_id;
      installed_clear.lifecycle_transaction_id =
          message->lifecycle_transaction_id;
      installed_clear.action_generation = message->action_generation;
      installed_clear.map_epoch = message->map_epoch;
      installed_clear.origin_epoch = message->origin_epoch;
      installed_clear.projection_epoch = message->projection_epoch;
      installed_clear.graph_transaction_id = message->graph_transaction_id;
      installed_clear.route_kind = message->route_kind;
      installed_clear.mission_route_kind = message->mission_route_kind;
      installed_clear.graph_action = message->graph_action;
      installed_clear.graph_obligation_kind = message->graph_obligation_kind;
      installed_clear.goal = message->goal;
        installed_target_command_publisher_.publish(installed_clear);
      }
      ROS_INFO_STREAM(
          "StreamingNavfnPlanner cleared all routes for hard reset "
          << "origin_epoch=" << message->origin_epoch
          << " projection_epoch=" << message->projection_epoch);
      return;
    }
    if (clear_sequence > 0 && latest_target_sequence_ > clear_sequence) {
      ROS_DEBUG_STREAM("StreamingNavfnPlanner ignored stale target clear "
                       << clear_sequence << " < " << latest_target_sequence_);
      return;
    }
    has_target_goal_ = false;
    has_validated_target_plan_ = false;
    validated_target_sequence_ = 0;
    if (clear_sequence == 0 ||
        active_target_mission_sequence_ <= clear_sequence) {
      has_active_target_mission_ = false;
      active_target_mission_sequence_ = 0;
    }
    // Retain the largest rejected sequence as a planner-side tombstone.  A
    // latched target request with this identity can be delivered again after
    // a hot restart, but it must not trigger another Navfn search.
    failed_target_sequence_ = std::max(failed_target_sequence_, clear_sequence);
    latest_target_sequence_ = std::max(latest_target_sequence_, clear_sequence);
    latest_target_origin_epoch_ = message->origin_epoch;
    latest_target_projection_epoch_ = message->projection_epoch;
    // Invalidation is terminal for this target identity. The bridge emits the
    // clear after any installed-target acknowledgement, so retaining the
    // candidate here would allow a delayed planner tick to reselect it before
    // the replacement frontier mission arrives.
    plan_dirty_ = true;
    ++mission_generation_;
    if (installed_target_command_publisher_) {
    PersistentGoalCommand installed_clear;
    installed_clear.kind = PersistentGoalCommand::KIND_CLEAR;
    installed_clear.transaction_id = clear_sequence;
    installed_clear.route_id = message->route_id;
    installed_clear.lifecycle_transaction_id =
        message->lifecycle_transaction_id;
    installed_clear.action_generation = message->action_generation;
    installed_clear.map_epoch = message->map_epoch;
    installed_clear.origin_epoch = message->origin_epoch;
    installed_clear.projection_epoch = message->projection_epoch;
    installed_clear.graph_transaction_id = message->graph_transaction_id;
    installed_clear.route_kind = message->route_kind;
    installed_clear.mission_route_kind = message->mission_route_kind;
    installed_clear.graph_action = message->graph_action;
    installed_clear.graph_obligation_kind = message->graph_obligation_kind;
    installed_clear.goal = message->goal;
      installed_target_command_publisher_.publish(installed_clear);
    }
    ROS_INFO_STREAM("StreamingNavfnPlanner cleared target request transaction="
                    << clear_sequence << " origin_epoch="
                    << message->origin_epoch << " projection_epoch="
                    << message->projection_epoch);
    return;
  }
  if (message->kind != PersistentGoalCommand::KIND_TARGET_REQUEST ||
      sequence == 0) {
    return;
  }
  ROS_INFO_STREAM("StreamingNavfnPlanner received target transaction="
                  << sequence << " approved_mission="
                  << latest_mission_sequence_ << " latest_target="
                  << latest_target_sequence_ << " endpoint=("
                  << message->goal.pose.position.x << ","
                  << message->goal.pose.position.y << ") origin_epoch="
                  << message->origin_epoch << " projection_epoch="
                  << message->projection_epoch);
  if (latest_mission_sequence_ > 0 && sequence <= latest_mission_sequence_) {
    ROS_WARN_STREAM("StreamingNavfnPlanner ignored stale target request "
                    << sequence << " <= approved mission "
                    << latest_mission_sequence_);
    return;
  }
  if (latest_target_sequence_ > 0 && sequence <= latest_target_sequence_) {
    ROS_WARN_STREAM("StreamingNavfnPlanner ignored stale target request "
                    << sequence << " <= " << latest_target_sequence_);
    return;
  }
  target_goal_ = message->goal;
  latest_target_sequence_ = sequence;
  latest_target_map_epoch_ = message->map_epoch;
  latest_target_origin_epoch_ = message->origin_epoch;
  latest_target_projection_epoch_ = message->projection_epoch;
  latest_target_graph_transaction_id_ = message->graph_transaction_id;
  latest_target_route_id_ = message->route_id;
  latest_target_lifecycle_transaction_id_ = message->lifecycle_transaction_id;
  latest_target_action_generation_ = message->action_generation;
  latest_target_route_kind_ = message->route_kind;
  latest_target_mission_route_kind_ = message->mission_route_kind;
  latest_target_graph_action_ = message->graph_action;
  latest_target_graph_obligation_kind_ = message->graph_obligation_kind;
  has_target_goal_ = true;
  has_validated_target_plan_ = false;
  failed_target_sequence_ = 0;
  plan_dirty_ = true;
  ++mission_generation_;
}

bool StreamingNavfnPlanner::shouldReplanLocked(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& selected) const {
  if (cached_plan_.empty()) {
    return true;
  }
  if (selected.header.frame_id != cached_goal_.header.frame_id) {
    return true;
  }
  if (std::hypot(selected.pose.position.x - cached_goal_.pose.position.x,
                 selected.pose.position.y - cached_goal_.pose.position.y) >
      goal_epsilon_) {
    return true;
  }
  if (std::hypot(last_plan_start_.pose.position.x - start.pose.position.x,
                 last_plan_start_.pose.position.y - start.pose.position.y) >
      replan_distance_) {
    return true;
  }
  return !last_plan_time_.isZero() &&
         (ros::WallTime::now() - last_plan_time_).toSec() >
             max_replan_interval_;
}

bool StreamingNavfnPlanner::transformToGlobalFrame(
    const geometry_msgs::PoseStamped& source,
    geometry_msgs::PoseStamped& transformed) const {
  if (source.header.frame_id.empty()) {
    return false;
  }
  if (source.header.frame_id == global_frame_) {
    transformed = source;
    return true;
  }
  try {
    // Mission endpoints are geometric route data, not historical sensor
    // observations. Portal endpoints are intentionally frozen in odom, so a
    // timestamp from activation becomes invalid as soon as the TF buffer
    // advances. Ask TF for the latest transform on every replan instead of
    // repeatedly rejecting an otherwise valid crossing route as stale.
    geometry_msgs::PoseStamped latest = source;
    latest.header.stamp = ros::Time(0);
    transform_listener_->transformPose(global_frame_, latest, transformed);
    return true;
  } catch (const tf::TransformException& error) {
    ROS_WARN_THROTTLE(1.0, "StreamingNavfnPlanner TF transform failed: %s",
                      error.what());
    return false;
  }
}

}  // namespace lste_topo_access

PLUGINLIB_EXPORT_CLASS(lste_topo_access::PersistentTebLocalPlanner,
                       nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(lste_topo_access::StreamingNavfnPlanner,
                       nav_core::BaseGlobalPlanner)
