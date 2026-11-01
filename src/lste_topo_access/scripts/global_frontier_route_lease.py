"""Failure authority for one committed exploration route lease.

The exploration layer and the local navigation controller observe different
facts.  A map snapshot can show no progress while TEB is still recovering or
while SLAM is correcting the frame.  In persistent execution that observation
must not be promoted into a route failure.  The controller already owns the
authoritative action terminal and recovery result.

This module is intentionally ROS-free.  It describes the ownership boundary
without adding a timeout, score, or controller parameter.
"""

from dataclasses import dataclass


AUTHORITY_GLOBAL_WATCHDOG = "global_watchdog"
AUTHORITY_CONTROLLER_TERMINAL = "controller_terminal"


@dataclass(frozen=True)
class RouteLeaseDecision:
    """Whether the global layer is allowed to release a route lease."""

    release: bool
    authority: str
    reason: str


def route_lease_failure_decision(
    *,
    controller_owns_failure=False,
    recovery_pending=False,
    route_stalled=False,
    post_turn_stalled=False,
    portal_edge_expired=False,
    active_timeout=False,
    stalled_for=None,
    controller_terminal_limit=None,
):
    """Return the only admissible failure transition for an active lease.

    ``recovery_pending`` is an externally reported controller terminal.  It is
    therefore authoritative in both modes.  In legacy mode, elapsed/stall
    evidence remains the historical fallback.  In persistent mode those same
    values are diagnostics only: the route stays committed until the local
    controller reports a failure.
    """
    authority = (
        AUTHORITY_CONTROLLER_TERMINAL
        if controller_owns_failure
        else AUTHORITY_GLOBAL_WATCHDOG
    )
    if recovery_pending:
        return RouteLeaseDecision(
            True,
            authority,
            "controller_terminal_failure",
        )
    if controller_owns_failure:
        # The controller is the failure authority, but it is not allowed to
        # withhold its terminal forever.  ``controller_owned_route_failure`` is
        # a mode flag, so the wait is measured from the route's own stalled
        # duration rather than from the flag.
        if (stalled_for is not None and controller_terminal_limit is not None
                and float(stalled_for) > float(controller_terminal_limit)):
            return RouteLeaseDecision(
                True,
                authority,
                "controller_terminal_timeout",
            )
        return RouteLeaseDecision(
            False,
            authority,
            "controller_owns_failure_terminal",
        )
    if portal_edge_expired:
        return RouteLeaseDecision(True, authority, "portal_edge_deadline")
    if post_turn_stalled:
        return RouteLeaseDecision(True, authority, "post_turn_no_progress")
    if active_timeout and route_stalled:
        return RouteLeaseDecision(True, authority, "active_timeout")
    if route_stalled:
        return RouteLeaseDecision(True, authority, "stall")
    return RouteLeaseDecision(False, authority, "active_route_healthy")


__all__ = [
    "AUTHORITY_CONTROLLER_TERMINAL",
    "AUTHORITY_GLOBAL_WATCHDOG",
    "RouteLeaseDecision",
    "route_lease_failure_decision",
]
