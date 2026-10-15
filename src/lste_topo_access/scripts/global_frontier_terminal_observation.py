"""Place closure that is permitted only after a verified route terminal."""

import rospy


class GlobalFrontierTerminalObservationMixin:
    """Commit terminal-time observation facts without owning action matching."""

    def close_stagnant_place_after_terminal(self, completed):
        """Close a no-gain place after TEB completes its exact route.

        The active-route watchdog may discover that a reached lidar viewpoint
        has yielded no new map information. That is useful evidence, but it
        cannot cancel or supersede a move_base action. A matching successful
        terminal is the lifecycle boundary at which that evidence becomes a
        place closure.
        """
        if (
            completed is None
            or self.active_route_kind in ("portal_transition", "local_egress")
            or self.target_region_claim_active
            or self.active_observation_session_started_at is None
            or self.active_last_robot_xy is None
        ):
            return None
        now = rospy.get_time()
        region = self.region_memory.stagnant(
            completed[2],
            completed[3],
            self.active_last_robot_xy[0],
            self.active_last_robot_xy[1],
            now,
            component=self.active_frontier_component,
            region_id=self.active_frontier_region_id,
        )
        if region is None or not self.region_memory.dormant(
            region, now, "no_information_progress",
        ):
            return None
        self.publish_status(
            "frontier_region_dormant",
            reason="no_information_progress",
            region_id=int(region["id"]),
            region_visits=int(region["visits"]),
            region_completions=int(region["completions"]),
            region_information=round(float(region["information"]), 3),
            no_gain_elapsed=round(float(now - region["last_gain"]), 3),
            goal=[round(float(completed[2]), 3), round(float(completed[3]), 3)],
        )
        rospy.loginfo(
            "Global frontier closed no-gain region id=%d after terminal "
            "route_id=%d",
            region["id"],
            self.active_route_id,
        )
        return region
