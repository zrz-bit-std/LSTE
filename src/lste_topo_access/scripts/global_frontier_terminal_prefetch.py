"""Terminal-time policy for precomputed frontier successors.

The policy currently invalidates cached successors. The implementation stays
isolated so a future safe handoff cannot obscure the terminal transaction.
"""

import rospy


class GlobalFrontierTerminalPrefetchMixin:
    """Discard a stale successor hint before planning from fresh map evidence."""

    def promote_prefetched_terminal(self, _terminal):
        """Discard a stale endpoint hint and require a fresh place decision.

        The final scan at an endpoint is exactly when online SLAM changes the
        local frontier graph. Directly promoting a branch scored before that
        scan allowed one physical room to reactivate itself before the place
        lifecycle could freeze it. A terminal is therefore a transaction
        boundary: the next timer builds one current snapshot and decides
        between an unseen local branch and an explicit doorway egress.
        """
        if self.prefetched_frontier is None or self.map_msg is None:
            return False
        _row, _col, x, y = self.prefetched_frontier
        self.publish_status(
            "frontier_prefetch_discarded",
            reason="terminal_requires_fresh_observation_place_decision",
            pending_goal=[round(float(x), 3), round(float(y), 3)],
        )
        self.clear_prefetched_frontier()
        rospy.loginfo(
            "Global frontier discarded terminal prefetch map=(%.2f,%.2f); "
            "replanning from current observation-place state",
            x,
            y,
        )
        return False
