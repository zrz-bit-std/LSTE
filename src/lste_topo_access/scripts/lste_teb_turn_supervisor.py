#!/usr/bin/env python3
"""Adapt TEB execution phases without creating another navigation action.

TEB is the production local planner.  A forward-only base may nevertheless
start a valid Navfn route with its first tangent behind the current heading.
In that case TEB can alternate between rotation and a blocked forward command
near a wall.  This node makes the short *execution phase* explicit: it rotates
to the first Navfn path tangent, then releases the same TEB command stream.
The MoveBaseAction identity and endpoint never change.

The control boundary is deliberately narrow:

* legacy ``frontier_turn_connector`` actions remain supported;
* legacy ``frontier_turn_connector`` actions get an explicit in-place turn;
* online-SLAM ``frontier_endpoint`` alignment is an opt-in comparison mode,
  not a second production local controller;
* visual targets and all other route kinds pass through unchanged;
* the turn velocity is an acceleration-limited angle-closed-loop command,
  derived from TEB's own ``max_vel_theta`` and ``acc_lim_theta`` parameters;
* after the route yaw tolerance is reached, the latest TEB command is released
  on the same output cycle.

The node sits between move_base and the command mux:

    move_base/TEB -> /lste/cmd_vel/teb_planner
                    -> turn supervisor -> /lste/cmd_vel/teb -> mux

It is therefore an execution-state adapter, not a second path planner and not
another PID tuning layer.
"""

from pathlib import Path
import sys

import rospy


# ``catkin_install_python`` can execute this source through a devel-space
# relay. Make sibling modules available in both direct and ``rosrun`` starts.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from teb_turn_supervisor_callbacks import TebTurnSupervisorCallbacksMixin
from teb_turn_supervisor_control import TebTurnSupervisorControlMixin
from teb_turn_supervisor_lifecycle import TebTurnSupervisorLifecycleMixin
from teb_turn_supervisor_parameters import configure_turn_supervisor_parameters
from teb_turn_supervisor_routes import TebTurnSupervisorRoutesMixin
from teb_turn_supervisor_ros import connect_turn_supervisor_ros
from teb_turn_supervisor_state import initialize_turn_supervisor_state
from teb_turn_supervisor_turn_lifecycle import TebTurnSupervisorTurnLifecycleMixin


class TebTurnSupervisor(
    TebTurnSupervisorLifecycleMixin,
    TebTurnSupervisorControlMixin,
    TebTurnSupervisorTurnLifecycleMixin,
    TebTurnSupervisorRoutesMixin,
    TebTurnSupervisorCallbacksMixin,
):
    """Small state machine which owns only explicit in-place turn actions."""

    def __init__(self):
        rospy.init_node("lste_teb_turn_supervisor")
        configure_turn_supervisor_parameters(self)
        initialize_turn_supervisor_state(self)
        self._initialize_lifecycle_manager()
        connect_turn_supervisor_ros(self)
        self.publish_status_locked("startup")
        rospy.loginfo(
            "TEB turn supervisor ready: planner=%s navfn_plan=%s output=%s "
            "frequency=%.1fHz max_theta=%.3f acc_theta=%.3f "
            "yaw_tolerance=%.3f continuity=%s stalled_route_reorientation=%s",
            self.planner_cmd_topic,
            self.navfn_plan_topic,
            self.output_cmd_topic,
            self.command_frequency,
            self.max_vel_theta,
            self.acc_lim_theta,
            self.yaw_goal_tolerance,
            self.trajectory_continuity_enabled,
            self.stalled_route_reorientation_enabled,
        )




if __name__ == "__main__":
    TebTurnSupervisor()
    rospy.spin()
