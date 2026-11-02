"""Map-frame angular ownership for WorkItems inside a selected GP branch."""
import math


def build_branch_scope(node_xy, branches, selected_branch_id):
    """Snapshot sibling GP directions in map frame for one selected branch."""
    try:
        origin = (float(node_xy[0]), float(node_xy[1]))
        selected = int(selected_branch_id)
    except (TypeError, ValueError, IndexError):
        return None
    directions = []
    for branch in branches:
        try:
            branch_id = int(branch.get("id"))
            direction = (branch.get("map_geometry") or {}).get("direction")
            dx, dy = float(direction[0]), float(direction[1])
            norm = math.hypot(dx, dy)
        except (AttributeError, TypeError, ValueError, IndexError):
            continue
        if norm <= 1e-9:
            continue
        directions.append({"branch_id": branch_id, "direction_xy": [dx / norm, dy / norm]})
    if not any(item["branch_id"] == selected for item in directions):
        return None
    return {"origin_xy": list(origin), "selected_branch_id": selected, "directions": directions}


def work_item_belongs_to_branch(item, scope):
    """Assign a map-frame anchor to the nearest forward GP sibling direction.

    A connected SLAM Place can span multiple GP arms. The branch fan divides
    its WorkItems without changing the durable Place identity.
    """
    if not isinstance(scope, dict) or not isinstance(item, dict):
        return False
    anchor = item.get("anchor_xy")
    if anchor is None and item.get("x") is not None and item.get("y") is not None:
        anchor = (item.get("x"), item.get("y"))
    try:
        px, py = float(anchor[0]), float(anchor[1])
        ox, oy = float(scope["origin_xy"][0]), float(scope["origin_xy"][1])
    except (TypeError, ValueError, IndexError, KeyError):
        return False
    vx, vy = px - ox, py - oy
    distance = math.hypot(vx, vy)
    if not math.isfinite(distance) or distance <= 1e-6:
        return False
    best = None
    for direction in scope.get("directions", ()):
        try:
            branch_id = int(direction["branch_id"])
            dx, dy = float(direction["direction_xy"][0]), float(direction["direction_xy"][1])
            norm = math.hypot(dx, dy)
        except (TypeError, ValueError, IndexError, KeyError):
            continue
        if norm <= 1e-9:
            continue
        projection = (vx * dx + vy * dy) / (distance * norm)
        if projection <= 0.0:
            continue
        if best is None or projection > best[0]:
            best = (projection, branch_id)
    return best is not None and best[1] == int(scope.get("selected_branch_id", -1))


def best_branch_for_portal(branches, portal_id):
    """Choose the highest semantic-rank GP branch represented by a Portal."""
    try:
        portal_id = int(portal_id)
    except (TypeError, ValueError):
        return None
    candidates = []
    values = branches.values() if isinstance(branches, dict) else branches
    for branch in values:
        try:
            if int(branch.get("canonical_portal_id", -1)) != portal_id:
                continue
            if branch.get("room_invalidated", False) or branch.get("state") == "superseded":
                continue
            rank = int(branch.get("room_priority"))
        except (AttributeError, TypeError, ValueError):
            rank = 10 ** 9
        try:
            branch_id = int(branch.get("id", 0))
        except (AttributeError, TypeError, ValueError):
            continue
        candidates.append((rank, branch_id, branch))
    return min(candidates, default=(None, None, None), key=lambda row: (row[0], row[1]))[2]
