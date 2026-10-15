"""Navfn reachability validation for visual target approach points.

This module contains the service transaction and cache policy. GoalManager
only supplies the mutable state object and retains the public method name.
"""

import copy
import math

import rospy
from nav_msgs.srv import GetPlanRequest


def validate_target_route(manager, goal, now, force=False):
    """Ask Navfn whether a visual approach point is currently connected.

    Return values are deliberately tri-state: ``True`` is a live route,
    ``False`` is a live empty plan (the target ray is blocked), and
    ``None`` means the planner/TF is not ready.  Both non-true states keep
    the current map route in charge; neither is converted into a blind
    action retry.
    """
    if manager.controller_mode != "teb" or not manager.target_route_validation:
        return True
    if goal is None:
        return False
    goal_frame = (goal.header.frame_id or "odom").strip().lstrip("/") or "odom"
    goal_xy = (
        round(float(goal.pose.position.x), 3),
        round(float(goal.pose.position.y), 3),
    )
    # A cached returned endpoint is valid only for the exact visual
    # request that produced it. Reusing a nearby candidate's endpoint
    # would silently pair two different sides of Navfn's tolerance rule.
    cached_goal_matches = (
        manager.target_route_validation_last_goal is not None
        and math.hypot(
            float(goal_xy[0]) - float(manager.target_route_validation_last_goal[0]),
            float(goal_xy[1]) - float(manager.target_route_validation_last_goal[1]),
        ) <= 0.001
    )
    if (
        not force
        and cached_goal_matches
        and now < manager.target_route_validation_next_time
    ):
        if (
            manager.target_route_validation_last_result == "reachable"
            and manager.target_route_validation_last_endpoint is not None
        ):
            return True
        if manager.target_route_validation_last_result == "blocked":
            return False
        return None
    if manager.latest_pose is None:
        return None
    start_odom = manager.make_goal_pose(
        (manager.latest_pose.x, manager.latest_pose.y, 0.0), manager.latest_pose.theta
    )
    start_map = manager._pose_in_frame(start_odom, "map")
    goal_map = manager._pose_in_frame(goal, "map")
    manager.target_route_validation_next_time = now + manager.target_route_validation_period
    manager.target_route_validation_last_goal = goal_xy
    if start_map is None or goal_map is None:
        manager.target_route_validation_last_result = "unavailable"
        manager.target_route_validation_last_endpoint = None
        manager.target_route_validation_last_plan = []
        manager.target_route_validation_last_start_heading = None
        manager.publish_goal_arbitration(
            "target_route_deferred",
            reason="tf_unavailable",
            goal_frame=goal_frame,
            goal=list(goal_xy),
        )
        return None
    try:
        rospy.wait_for_service(
            manager.target_route_validation_service,
            timeout=manager.target_route_validation_timeout,
        )
    except (rospy.ROSException, rospy.ROSInterruptException):
        manager.target_route_validation_last_result = "unavailable"
        manager.target_route_validation_last_endpoint = None
        manager.target_route_validation_last_plan = []
        manager.target_route_validation_last_start_heading = None
        manager.publish_goal_arbitration(
            "target_route_deferred",
            reason="navfn_service_unavailable",
            service=manager.target_route_validation_service,
            goal=list(goal_xy),
        )
        return None
    request = GetPlanRequest()
    request.start = start_map
    request.goal = goal_map
    request.tolerance = manager.target_route_validation_tolerance
    try:
        response = manager.target_route_validation_client(request)
    except (rospy.ServiceException, rospy.ROSException) as exc:
        manager.target_route_validation_last_result = "unavailable"
        manager.target_route_validation_last_endpoint = None
        manager.target_route_validation_last_plan = []
        manager.target_route_validation_last_start_heading = None
        manager.publish_goal_arbitration(
            "target_route_deferred",
            reason="navfn_service_error",
            service=manager.target_route_validation_service,
            error=str(exc),
            goal=list(goal_xy),
        )
        return None
    reachable = bool(response.plan.poses)
    manager.target_route_validation_last_result = "reachable" if reachable else "blocked"
    manager.target_route_validation_last_plan = []
    manager.target_route_validation_last_start_heading = None
    if reachable:
        # Navfn is queried in map coordinates. Its final plan pose is the
        # only endpoint it has actually proven reachable, including the
        # configured tolerance behaviour. Preserve the visual candidate's
        # desired yaw, while making that reachable point the target pose.
        returned_endpoint = copy.deepcopy(response.plan.poses[-1])
        returned_endpoint.header.frame_id = (
            returned_endpoint.header.frame_id
            or response.plan.header.frame_id
            or goal_map.header.frame_id
            or "map"
        )
        if manager._frame_name(returned_endpoint.header.frame_id) != "map":
            returned_endpoint = manager._pose_in_frame(returned_endpoint, "map")
        if returned_endpoint is None:
            manager.target_route_validation_last_result = "unavailable"
            manager.target_route_validation_last_endpoint = None
            manager.publish_goal_arbitration(
                "target_route_deferred",
                reason="navfn_endpoint_tf_unavailable",
                goal=list(goal_xy),
            )
            return None
        returned_endpoint.header.frame_id = "map"
        returned_endpoint.header.stamp = goal_map.header.stamp
        returned_endpoint.pose.orientation = copy.deepcopy(goal_map.pose.orientation)
        endpoint_adjustment = math.hypot(
            float(returned_endpoint.pose.position.x) - float(goal_map.pose.position.x),
            float(returned_endpoint.pose.position.y) - float(goal_map.pose.position.y),
        )
        manager.target_route_validation_last_endpoint = returned_endpoint
        manager.target_route_validation_last_plan = copy.deepcopy(
            list(response.plan.poses)
        )
        manager.target_route_validation_last_start_heading = manager.yaw_from_pose(
            start_map
        )
        manager.target_route_validation_failures = 0
        manager.publish_goal_arbitration(
            "target_route_accepted",
            requested_goal=list(goal_xy),
            navfn_returned_goal=[
                round(float(returned_endpoint.pose.position.x), 3),
                round(float(returned_endpoint.pose.position.y), 3),
            ],
            navfn_endpoint_adjustment=round(float(endpoint_adjustment), 4),
            plan_poses=len(response.plan.poses),
            plan_frame=response.plan.header.frame_id or "map",
        )
        return True
    manager.target_route_validation_last_endpoint = None
    manager.target_route_validation_failures += 1
    manager.publish_goal_arbitration(
        "target_route_rejected",
        reason="navfn_empty_plan",
        goal=list(goal_xy),
        robot=[
            round(float(start_map.pose.position.x), 3),
            round(float(start_map.pose.position.y), 3),
        ],
        failures=manager.target_route_validation_failures,
    )
    rospy.logwarn(
        "GoalManager: visual target route rejected by Navfn goal=(%.2f,%.2f) "
        "robot=(%.2f,%.2f) failures=%d; retaining map route",
        goal_map.pose.position.x,
        goal_map.pose.position.y,
        start_map.pose.position.x,
        start_map.pose.position.y,
        manager.target_route_validation_failures,
    )
    return False

