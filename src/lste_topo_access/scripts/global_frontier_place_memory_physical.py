"""Small coordinate helpers for SLAM-stable place-memory facts.

``map`` coordinates are execution projections and may move after a SLAM
correction.  These helpers keep physical ``odom`` evidence well-formed while
leaving all navigation commands in the map frame.
"""

import math


def as_xy(value):
    """Return a finite planar pair, or ``None`` for incomplete evidence."""
    if value is None:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (IndexError, TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return x, y


def append_spaced_anchor(anchors, value, minimum_spacing):
    """Append one physical or map anchor when it adds spatial evidence."""
    point = as_xy(value)
    if point is None:
        return False
    for existing in anchors:
        existing_point = as_xy(existing)
        if existing_point is None:
            continue
        if math.hypot(point[0] - existing_point[0], point[1] - existing_point[1]) < minimum_spacing:
            return False
    anchors.append(point)
    return True


def project_points(points, projector):
    """Project valid physical anchors into one current map snapshot."""
    if projector is None:
        return []
    projected = []
    for point in points:
        point = as_xy(point)
        if point is None:
            continue
        value = as_xy(projector(point[0], point[1]))
        if value is not None:
            projected.append(value)
    return projected
