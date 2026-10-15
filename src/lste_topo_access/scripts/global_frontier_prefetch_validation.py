"""Validate a cached successor against the latest map and planners."""

import rospy


class GlobalFrontierPrefetchValidationMixin:
    def validated_prefetched_frontier(self, message, steps, robot_map, validation):
        """Return the cached branch only while both planners still accept it."""
        if self.prefetched_frontier is None:
            return None
        _, _, x, y = self.prefetched_frontier
        if validation is not None and self.candidate_costmap_distance(
            validation, x, y
        ) is None:
            rospy.logwarn(
                "Global frontier discarded prefetched branch map=(%.2f,%.2f): "
                "global costmap has no connected route",
                x,
                y,
            )
            self.clear_prefetched_frontier()
            return None
        reachable = self.navfn_goal_reachable(
            robot_map, (x, y), message.header.frame_id or "map"
        )
        if reachable is False:
            rospy.logwarn(
                "Global frontier discarded prefetched Navfn-empty branch "
                "map=(%.2f,%.2f)", x, y,
            )
            self.clear_prefetched_frontier()
            return None
        reassociated = self.nearest_reachable_cell(message, steps, x, y)
        if reassociated is None:
            rospy.logwarn(
                "Global frontier discarded prefetched branch map=(%.2f,%.2f): "
                "no safe connected cell",
                x,
                y,
            )
            self.clear_prefetched_frontier()
            return None
        return reassociated[0], reassociated[1], x, y
