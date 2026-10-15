"""Physical, directional evidence for one certified portal crossing.

Map coordinates change while online SLAM refines the pose graph.  A portal
transaction therefore freezes its gate and destination direction in ``odom``
at activation time, then evaluates the base's actual odometry at terminal
time.  This module is ROS-free so the topological invariant is testable.
"""

import math

from global_frontier_models import PortalCrossingEvidence


def portal_crossing_depth(clearance):
    """Return the geometric depth needed to call a doorway crossed.

    ``completed_radius`` describes how much of a *room* has been observed. It
    is intentionally much larger than a doorway and must not decide whether a
    robot crossed that doorway.  A certified portal already guarantees the
    full navigation clearance at its opening.  Once the base centre has moved
    half that clearance beyond the gate plane, it is on the destination side
    rather than in the opening itself.
    """
    return max(0.05, 0.5 * max(0.0, float(clearance)))


def portal_execution_goal(route_kind, map_frame, map_xy, frozen_odom_xy):
    """Return the frame-stable goal for a route command.

    Ordinary frontiers are map observations and remain in the SLAM ``map``
    frame.  A portal is a physical edge transaction: after admission, its
    terminal must not move when gmapping refines ``map -> odom``.  Reusing the
    odom destination frozen at activation preserves exactly that contract.
    """
    if route_kind == "portal_transition" and frozen_odom_xy is not None:
        try:
            x, y = float(frozen_odom_xy[0]), float(frozen_odom_xy[1])
        except (TypeError, ValueError, IndexError):
            pass
        else:
            if math.isfinite(x) and math.isfinite(y):
                return "odom", (x, y)
    return (str(map_frame or "map").strip().lstrip("/") or "map"), (
        float(map_xy[0]), float(map_xy[1]),
    )


def portal_signed_distance(point_xy, gate_xy, destination_xy):
    """Project a physical point onto a portal's directed normal.

    Negative values lie on the source side of the gate; positive values lie
    on the destination side.  ``None`` means the candidate cannot define a
    physical directed edge and must not be treated as a portal transaction.
    """
    if not all((point_xy, gate_xy, destination_xy)):
        return None
    try:
        direction_x = float(destination_xy[0]) - float(gate_xy[0])
        direction_y = float(destination_xy[1]) - float(gate_xy[1])
        point_x, point_y = float(point_xy[0]), float(point_xy[1])
    except (IndexError, TypeError, ValueError):
        return None
    direction_length = math.hypot(direction_x, direction_y)
    if direction_length <= 1e-6:
        return None
    return (
        (point_x - float(gate_xy[0])) * direction_x
        + (point_y - float(gate_xy[1])) * direction_y
    ) / direction_length


def portal_source_side_is_proven(source_xy, gate_xy, destination_xy):
    """Return true only when a candidate begins on its directed source side."""
    signed_distance = portal_signed_distance(
        source_xy, gate_xy, destination_xy,
    )
    return signed_distance is not None and signed_distance < 0.0


def portal_crossing_evidence(
    start_xy, gate_xy, destination_xy, terminal_xy, gate_approached,
    minimum_destination_depth, preobserved=False, source_side_proven=False,
):
    """Return whether the terminal is physically beyond the directed gate.

    A move_base success only says that the base reached a circle around its
    local goal.  It is not doorway evidence.  A crossing needs a source-side
    start and a terminal pose a bounded distance on the destination side.

    ``gate_approached`` remains useful early evidence while the action is
    running, but it must not be a terminal-only requirement.  Odometry is a
    continuous trajectory: when a certified portal action starts with a
    negative signed distance and ends at a positive signed distance, it has
    necessarily crossed that portal's directed plane. Requiring one planning
    tick to also fall inside a gate-centre circle created false retries when
    SLAM or a curved TEB path shifted that sample slightly along the doorway.

    ``source_side_proven`` is the durable transaction-level form of the same
    fact. It is needed when a recovery route starts at the doorway and therefore
    has no route-local source-side sample, while the earlier attempt already
    supplied one.
    """
    if not all((start_xy, gate_xy, destination_xy, terminal_xy)):
        return PortalCrossingEvidence(
            verified=False,
            reason="physical_crossing_evidence_unavailable",
            source_signed_distance=None,
            terminal_signed_distance=None,
            required_destination_depth=float(minimum_destination_depth),
        )
    source_signed = portal_signed_distance(start_xy, gate_xy, destination_xy)
    terminal_signed = portal_signed_distance(
        terminal_xy, gate_xy, destination_xy,
    )
    if source_signed is None or terminal_signed is None:
        return PortalCrossingEvidence(
            verified=False,
            reason="portal_destination_direction_unavailable",
            source_signed_distance=None,
            terminal_signed_distance=None,
            required_destination_depth=float(minimum_destination_depth),
        )
    required_depth = max(0.0, float(minimum_destination_depth))
    # A positive terminal after a negative source is continuous directed-plane
    # evidence.  The candidate route itself was already constrained to one
    # certified doorway, so this is not permission to cross an arbitrary map
    # line.  It simply removes a lossy, sampled duplicate of the same fact.
    directed_plane_crossed = source_signed < 0.0 and terminal_signed >= 0.0
    historical_source_proof = bool(source_side_proven)
    if preobserved and source_signed >= required_depth:
        reason = (
            "portal_crossing_preobserved"
            if terminal_signed >= required_depth
            else "physical_destination_side_not_reached"
        )
    elif not historical_source_proof and source_signed >= 0.0:
        reason = "physical_source_side_unproven"
    elif (
        not historical_source_proof
        and not gate_approached
        and not directed_plane_crossed
    ):
        reason = "physical_gate_not_approached"
    elif terminal_signed < required_depth:
        reason = "physical_destination_side_not_reached"
    else:
        reason = "physical_gate_crossed"
    return PortalCrossingEvidence(
        verified=reason in ("physical_gate_crossed", "portal_crossing_preobserved"),
        reason=reason,
        source_signed_distance=source_signed,
        terminal_signed_distance=terminal_signed,
        required_destination_depth=required_depth,
    )
