#!/usr/bin/env python3

"""ROS topic names used by the online frontier explorer."""


class GlobalFrontierTopicConfigurationMixin:
    """Load ROS interfaces without coupling them to route policy."""

    def _load_topic_parameters(self, gp):
        """Resolve all subscribed and published ROS topic names."""
        self.map_topic = gp("~map_topic", "/map")
        # Navfn plans on the inflated global costmap, not directly on the raw
        # SLAM occupancy grid.  Use it as a candidate validator whenever it is
        # available so a frontier that is connected in /map but lethal after
        # costmap inflation is never handed to move_base.
        self.costmap_topic = gp(
            "~costmap_topic", "/move_base/global_costmap/costmap"
        )
        self.costmap_updates_topic = gp(
            "~costmap_updates_topic", "/move_base/global_costmap/costmap_updates"
        )
        self.pose_topic = gp("~pose_topic", "/rbt_pose")
        # The task and detector streams provide semantic evidence for the
        # topological explorer. They never publish geometry or control.
        self.task_topic = gp("~task_topic", "/lste/task")
        self.detections_topic = gp("~detections_topic", "/lste/detections")
        self.goal_arbitration_topic = gp(
            "~goal_arbitration_topic", "/lste/goal_arbitration"
        )
        self.goal_topic = gp("~goal_topic", "/lste/global_frontier_goal")
        # A PoseStamped cannot carry route kind and rospy owns Header.seq.
        # Publish the complete route transaction on one companion topic so
        # consumers never have to correlate independent pose/status messages.
        self.command_topic = gp(
            "~command_topic", "/lste/global_frontier/route_command"
        )
        self.task_done_topic = gp("~task_done_topic", "/lste/task_done")
        self.status_topic = gp(
            "~status_topic", "/lste/global_frontier/status"
        )
        self.replan_request_topic = gp(
            "~replan_request_topic", "/lste/global_frontier/replan_request"
        )
        self.turn_status_topic = gp(
            "~turn_status_topic", "/lste/teb_turn_supervisor/status"
        )
        # The legacy pose terminal remains available to existing diagnostics.
        # The frontier state machine consumes the typed companion contract:
        # its route ID prevents an adjacent old endpoint from terminating a
        # newly selected action.
        self.terminal_topic = gp("~terminal_topic", "/lste/teb_goal_terminal")
        self.terminal_contract_topic = gp(
            "~terminal_contract_topic", "/lste/teb_goal_terminal_contract"
        )
        # Local recovery and action failure are different lifecycle states.
        # A recovery is deliberately local: TEB/move_base may still clear a
        # transient lidar obstacle or rotate into an open doorway. Only a
        # terminal *failed* action proves that the globally validated route
        # must be replaced.
        self.recovery_topic = gp("~recovery_topic", "/move_base/recovery_status")
        self.bridge_status_topic = gp(
            "~bridge_status_topic", "/lste/teb_goal_bridge/status"
        )
        self.scan_topic = gp("~scan_topic", "/pro3/rlscan")
