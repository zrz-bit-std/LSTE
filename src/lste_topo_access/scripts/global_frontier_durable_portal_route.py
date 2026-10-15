"""Pure geometry for compiling a durable Portal fact into a crossing route.

The Place graph owns the identity of a doorway, while the occupancy grid owns
only its current projection.  Keeping this small conversion pure makes the
important boundary explicit and lets tests exercise it without ROS or Gazebo.
"""

from dataclasses import dataclass
import math


def finite_xy(value):
    """Return a finite planar pair or ``None``."""
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        point = float(value[0]), float(value[1])
    except (TypeError, ValueError):
        return None
    return point if all(math.isfinite(item) for item in point) else None


@dataclass(frozen=True)
class DurablePortalProjection:
    """One coherent map-frame projection of a physical Portal edge."""

    portal_id: int
    source_place_id: int
    gate_xy: tuple
    destination_xy: tuple


def signed_distance(point_xy, projection):
    """Return signed distance along the Portal's directed normal."""
    point = finite_xy(point_xy)
    if point is None or not isinstance(projection, DurablePortalProjection):
        return None
    dx = projection.destination_xy[0] - projection.gate_xy[0]
    dy = projection.destination_xy[1] - projection.gate_xy[1]
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return None
    return (
        (point[0] - projection.gate_xy[0]) * dx
        + (point[1] - projection.gate_xy[1]) * dy
    ) / length


def project_record(record, project_physical_to_map=None):
    """Project a ledger record, preferring current physical TF evidence.

    ``map_gate`` and ``map_destination`` are the last coherent snapshot.  A
    current physical projection is stronger when TF is available, but the
    fallback keeps a short-lived TF outage from changing the Portal identity.
    """
    if not isinstance(record, dict):
        return None
    try:
        portal_id = int(record.get("id"))
        source_place_id = int(record.get("source_place_id"))
    except (TypeError, ValueError):
        return None
    if portal_id <= 0 or source_place_id <= 0:
        return None

    gate = finite_xy(record.get("physical_gate"))
    destination = finite_xy(record.get("physical_destination"))
    if project_physical_to_map is not None and gate is not None and destination is not None:
        try:
            projected_gate = finite_xy(
                project_physical_to_map(gate[0], gate[1])
            )
            projected_destination = finite_xy(
                project_physical_to_map(destination[0], destination[1])
            )
        except Exception:
            projected_gate = projected_destination = None
        if projected_gate is not None and projected_destination is not None:
            gate, destination = projected_gate, projected_destination
        else:
            # The physical values are odom-frame facts.  Do not silently use
            # them as map coordinates when this snapshot has no valid TF.
            gate = destination = None

    if gate is None:
        gate = finite_xy(record.get("map_gate"))
    if destination is None:
        destination = finite_xy(record.get("map_destination"))
    if gate is None or destination is None:
        return None
    if math.hypot(destination[0] - gate[0], destination[1] - gate[1]) <= 1e-6:
        return None
    return DurablePortalProjection(
        portal_id=portal_id,
        source_place_id=source_place_id,
        gate_xy=gate,
        destination_xy=destination,
    )


def crossing_goal(projection, minimum_depth):
    """Return a point at least ``minimum_depth`` beyond the Portal plane."""
    if not isinstance(projection, DurablePortalProjection):
        return None
    try:
        depth = max(0.0, float(minimum_depth))
    except (TypeError, ValueError):
        return None
    dx = projection.destination_xy[0] - projection.gate_xy[0]
    dy = projection.destination_xy[1] - projection.gate_xy[1]
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return None
    distance = max(length, depth)
    return (
        projection.gate_xy[0] + dx / length * distance,
        projection.gate_xy[1] + dy / length * distance,
    )


__all__ = [
    "DurablePortalProjection",
    "crossing_goal",
    "finite_xy",
    "project_record",
    "signed_distance",
]
