"""Live Navfn/costmap admission for persistent frontier prefetches.

A prefetch is accepted only when a fresh Navfn plan and its local-costmap
prefix remain admissible. This module evaluates that contract; it does not
publish route ownership or manipulate the MoveBase action.
"""

import math

import rospy
import tf
from nav_msgs.srv import GetPlan, GetPlanRequest

from clock_provider import now_for


class TebGoalBridgePrefetchAdmissionMixin:
    def _local_costmap_prefix_admission_locked(self, path):
        """Check a Navfn route's near prefix against the current lidar map.

        Navfn validates the global SLAM costmap, which can legitimately lag a
        newly seen wall or doorway.  Before a running TEB band is redirected
        to a prefetched branch, sample the first local horizon in the exact
        rolling costmap that TEB will use.  This is intentionally conservative
        at lethal/unknown cells; inflated but traversable cells remain TEB's
        optimization problem rather than a duplicated obstacle policy here.
        """
        message = self.local_costmap
        now = now_for(self)
        if message is None:
            return False, {"reason": "local_costmap_unavailable"}
        age = now - self.local_costmap_received_monotonic
        if age > self.persistent_frontier_admission_max_costmap_age:
            return False, {
                "reason": "local_costmap_stale",
                "costmap_age_seconds": round(age, 3),
            }
        if len(path.poses) < 2:
            return False, {"reason": "navfn_plan_too_short"}
        local_frame = (message.header.frame_id or "").strip().lstrip("/")
        plan_frame = (
            path.header.frame_id
            or path.poses[0].header.frame_id
            or self.global_frame
        ).strip().lstrip("/")
        if not local_frame or not plan_frame:
            return False, {"reason": "local_costmap_frame_unavailable"}
        try:
            if local_frame == plan_frame:
                translation = (0.0, 0.0, 0.0)
                rotation = (0.0, 0.0, 0.0, 1.0)
            else:
                translation, rotation = self.tf_listener.lookupTransform(
                    local_frame, plan_frame, rospy.Time(0)
                )
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            return False, {
                "reason": "local_costmap_transform_unavailable",
                "error": type(exc).__name__,
                "local_frame": local_frame,
                "plan_frame": plan_frame,
            }
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        previous = None
        traversed = 0.0
        checked_cells = set()
        max_cost = 0
        for pose in path.poses:
            px = float(pose.pose.position.x)
            py = float(pose.pose.position.y)
            if previous is not None:
                traversed += math.hypot(px - previous[0], py - previous[1])
            previous = (px, py)
            if traversed > self.persistent_frontier_admission_horizon:
                break
            local_x = cosine * px - sine * py + translation[0]
            local_y = sine * px + cosine * py + translation[1]
            index = self._local_costmap_cell(message, local_x, local_y)
            if index is None:
                return False, {
                    "reason": "local_horizon_outside_costmap",
                    "checked_distance_m": round(traversed, 3),
                    "local_frame": local_frame,
                }
            if index in checked_cells:
                continue
            checked_cells.add(index)
            cost = int(message.data[index])
            max_cost = max(max_cost, cost)
            if cost < 0 or cost >= self.persistent_frontier_admission_blocked_cost:
                return False, {
                    "reason": "local_prefix_blocked",
                    "checked_distance_m": round(traversed, 3),
                    "cell_cost": cost,
                    "max_cost": max_cost,
                    "blocked_cost": self.persistent_frontier_admission_blocked_cost,
                    "local_frame": local_frame,
                }
        if not checked_cells:
            return False, {"reason": "local_prefix_empty"}
        return True, {
            "reason": "accepted",
            "checked_distance_m": round(
                min(traversed, self.persistent_frontier_admission_horizon), 3
            ),
            "checked_cells": len(checked_cells),
            "max_cost": max_cost,
            "local_frame": local_frame,
            "costmap_age_seconds": round(age, 3),
        }

    def _admit_persistent_frontier_prefetch_locked(self):
        """Return whether the pending branch is safe to stream before stop.

        The cached frontier choice alone is not sufficient: it was selected
        before the latest lidar scan and from the global map.  This transaction
        replans from the live feedback pose through Navfn and requires the
        resulting local prefix to be admissible.  A failure never cancels the
        active route; the normal safe-terminal path remains the fallback.
        """
        if self.active_feedback_pose is None or self.prefetched_frontier_goal is None:
            return False, {"reason": "feedback_or_prefetch_unavailable"}
        route_pair = (int(self.active_route_id), int(self.prefetched_frontier_route_id))
        feedback_xy = (float(self.active_feedback_pose[0]), float(self.active_feedback_pose[1]))
        now = now_for(self)
        cached = self.persistent_prefetch_admission_cache
        if (
            cached is not None
            and cached.get("route_pair") == route_pair
            and now - cached.get("monotonic", 0.0) <= 0.30
            and math.hypot(
                feedback_xy[0] - cached.get("feedback_xy", feedback_xy)[0],
                feedback_xy[1] - cached.get("feedback_xy", feedback_xy)[1],
            ) <= 0.10
        ):
            return bool(cached["accepted"]), dict(cached["details"])

        details = {
            "route_id": route_pair[0],
            "successor_route_id": route_pair[1],
            "feedback": [round(feedback_xy[0], 3), round(feedback_xy[1], 3)],
            "pending_goal": [
                round(float(self.prefetched_frontier_goal[0]), 3),
                round(float(self.prefetched_frontier_goal[1]), 3),
            ],
        }
        # Keep the frontier's BFS tangent as diagnostic evidence only.  The
        # actual transition class is assigned below from the fresh Navfn plan
        # requested from the live feedback pose.
        prefetch_heading_delta, prefetch_heading_basis = (
            self._frontier_prefetch_heading_delta_locked(
                self.last_dispatched_goal,
                self.prefetched_frontier_goal,
            )
        )
        details["prefetch_entry_heading_basis"] = prefetch_heading_basis
        details["prefetch_entry_heading_delta_deg"] = (
            None
            if prefetch_heading_delta is None
            else round(math.degrees(prefetch_heading_delta), 3)
        )
        details["max_entry_heading_delta_deg"] = round(
            math.degrees(self.frontier_early_handoff_max_heading_delta), 3
        )
        details["max_curve_heading_delta_deg"] = round(
            math.degrees(self.persistent_frontier_curve_handoff_max_heading_delta),
            3,
        )
        details["navfn_entry_tangent_sample_distance_m"] = round(
            self.persistent_frontier_entry_tangent_distance, 3
        )
        request = GetPlanRequest()
        request.start.header.frame_id = self.global_frame
        request.start.header.stamp = rospy.Time.now()
        request.start.pose.position.x = feedback_xy[0]
        request.start.pose.position.y = feedback_xy[1]
        request.start.pose.orientation.w = 1.0
        request.goal.header.frame_id = self.global_frame
        request.goal.header.stamp = request.start.header.stamp
        request.goal.pose.position.x = float(self.prefetched_frontier_goal[0])
        request.goal.pose.position.y = float(self.prefetched_frontier_goal[1])
        request.goal.pose.orientation.w = 1.0
        request.tolerance = 0.0
        try:
            rospy.wait_for_service(
                self.navfn_make_plan_service,
                timeout=self.persistent_frontier_admission_service_timeout,
            )
            response = rospy.ServiceProxy(
                self.navfn_make_plan_service, GetPlan
            )(request)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            details.update({
                "reason": "navfn_admission_unavailable",
                "error": type(exc).__name__,
            })
            accepted = False
        else:
            path = response.plan
            if not path.poses:
                details["reason"] = "navfn_admission_empty"
                accepted = False
            else:
                endpoint = path.poses[-1].pose.position
                endpoint_error = math.hypot(
                    float(endpoint.x) - request.goal.pose.position.x,
                    float(endpoint.y) - request.goal.pose.position.y,
                )
                if endpoint_error > self.persistent_frontier_admission_endpoint_epsilon:
                    details.update({
                        "reason": "navfn_admission_endpoint_offset",
                        "endpoint_error_m": round(endpoint_error, 3),
                    })
                    accepted = False
                else:
                    details["navfn_poses"] = len(path.poses)
                    details["endpoint_error_m"] = round(endpoint_error, 3)
                    entry_heading = self._navfn_entry_tangent_locked(
                        path, feedback_xy
                    )
                    if entry_heading is None:
                        details.update({
                            "reason": "navfn_entry_tangent_unavailable",
                            "transition_kind": "terminal_then_native_teb_reorientation",
                        })
                        accepted = False
                    else:
                        entry_delta = abs(self._angle_delta(
                            entry_heading, self.active_feedback_pose[2]
                        ))
                        details["entry_heading_basis"] = "navfn_live_feedback_tangent"
                        details["entry_heading_delta_deg"] = round(
                            math.degrees(entry_delta), 3
                        )
                        if (
                            entry_delta
                            > self.persistent_frontier_curve_handoff_max_heading_delta
                        ):
                            # This is deliberately a terminal fallback, not a
                            # claimed explicit-turn state: production lets the
                            # native TEB controller reorient after the active
                            # endpoint completes.
                            details.update({
                                "reason": "navfn_entry_tangent_too_sharp",
                                "transition_kind": "terminal_then_native_teb_reorientation",
                            })
                            accepted = False
                        else:
                            details["transition_kind"] = (
                                "curve_handoff"
                                if entry_delta
                                > self.frontier_early_handoff_max_heading_delta
                                else "smooth_handoff"
                            )
                            accepted, local = self._local_costmap_prefix_admission_locked(path)
                            details.update(local)
        self.persistent_prefetch_admission_cache = {
            "route_pair": route_pair,
            "feedback_xy": feedback_xy,
            "monotonic": now,
            "accepted": bool(accepted),
            "details": dict(details),
        }
        return bool(accepted), details
