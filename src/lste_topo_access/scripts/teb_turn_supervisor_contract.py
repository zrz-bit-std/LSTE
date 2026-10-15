"""Shared execution-contract primitives for the TEB turn supervisor.

Keep these definitions outside the ROS entrypoint so every supervisor module
uses the same route labels and yaw conversion rules.
"""

import math


TURN_ROUTE_KIND = "frontier_turn_connector"
FRONTIER_ENDPOINT_KIND = "frontier_endpoint"
PORTAL_TRANSITION_KIND = "portal_transition"
LOCAL_EGRESS_KIND = "local_egress"
PORTAL_PROBE_KIND = "portal_probe"
FRONTIER_SOURCE = "global_slam_frontier"
STATE_PASS_THROUGH = "PASS_THROUGH"
STATE_TURNING = "TURNING"
SHARP_ENTRY_CONTINUITY_BLOCK_RAD = math.pi / 2.0


def normalize_angle(angle):
    """Return ``angle`` in [-pi, pi]."""
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def is_managed_frontier_route(route_kind, source):
    """Return whether the execution adapter owns this frontier route phase.

    A portal transition is still a single TEB mission endpoint, but it is a
    graph-edge transaction with the same need for a deterministic entry
    heading as an explicit turn connector. Keeping the membership in one
    contract function prevents one callback from silently treating portals as
    pass-through while another callback treats them as managed actions.
    """
    normalized_route = str(route_kind or "").strip().lower()
    if normalized_route == TURN_ROUTE_KIND:
        # Preserve the legacy explicit-turn contract, whose intent source was
        # historically not restricted to GlobalFrontier.
        return True
    return (
        str(source or "").strip().lower() == FRONTIER_SOURCE
        and normalized_route in (
            FRONTIER_ENDPOINT_KIND,
            PORTAL_TRANSITION_KIND,
            LOCAL_EGRESS_KIND,
            PORTAL_PROBE_KIND,
        )
    )


def angle_from_pose(message):
    """Extract planar yaw from a PoseStamped quaternion."""
    q = message.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
