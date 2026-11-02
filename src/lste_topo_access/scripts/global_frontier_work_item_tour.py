"""Kinematic Time-Optimal WorkItem Open TSP Tour Solver.

Optimizes the global visit sequence of WorkItems starting from the robot's
current pose (x0, y0, theta0).  Instead of pure Euclidean distance, this solver
uses a kinematic time metric combining linear translation speed (v_max) and
rotational turning speed (omega_max) to account for heading alignment and avoid
awkward, costly 180-degree turnarounds in the field.
"""

import math
from typing import Any, Iterable, List, Optional, Sequence, Tuple


def normalize_angle(angle: float) -> float:
    """Normalize an angle to (-pi, pi]."""
    a = math.atan2(math.sin(angle), math.cos(angle))
    if a <= -math.pi + 1e-12:
        return math.pi
    return a


def extract_work_item_pose(
    item: Any,
) -> Optional[Tuple[float, float, Optional[float]]]:
    """Safely extract 2D pose (x, y, theta) from a WorkItem, pose-like object, or tuple.

    Returns:
        (x, y, theta) where theta is float heading in radians or None if unoriented.
    """
    if item is None:
        return None

    x: Optional[float] = None
    y: Optional[float] = None
    th: Optional[float] = None

    if isinstance(item, dict):
        anchor = item.get("anchor_xy")
        if (
            isinstance(anchor, (tuple, list))
            and len(anchor) >= 2
            and anchor[0] is not None
            and anchor[1] is not None
        ):
            x, y = float(anchor[0]), float(anchor[1])
        elif item.get("x") is not None and item.get("y") is not None:
            x, y = float(item["x"]), float(item["y"])
        elif item.get("xy") is not None and isinstance(item["xy"], (tuple, list)) and len(item["xy"]) >= 2:
            x, y = float(item["xy"][0]), float(item["xy"][1])

        # Heading angle extraction
        if item.get("theta") is not None:
            th = float(item["theta"])
        elif item.get("yaw") is not None:
            th = float(item["yaw"])
        elif item.get("heading") is not None:
            th = float(item["heading"])
        elif item.get("normal_xy") is not None:
            norm_xy = item["normal_xy"]
            if isinstance(norm_xy, (tuple, list)) and len(norm_xy) >= 2:
                nx, ny = float(norm_xy[0]), float(norm_xy[1])
                if math.hypot(nx, ny) > 1e-6:
                    th = math.atan2(ny, nx)
        elif item.get("viewpoint_yaw") is not None:
            th = float(item["viewpoint_yaw"])

    elif hasattr(item, "anchor_xy") and getattr(item, "anchor_xy", None) is not None:
        anchor = getattr(item, "anchor_xy")
        if isinstance(anchor, (tuple, list)) and len(anchor) >= 2:
            x, y = float(anchor[0]), float(anchor[1])
    elif hasattr(item, "x") and hasattr(item, "y"):
        x_val = getattr(item, "x", None)
        y_val = getattr(item, "y", None)
        if x_val is not None and y_val is not None:
            x, y = float(x_val), float(y_val)
        for attr in ("theta", "yaw", "heading"):
            if hasattr(item, attr) and getattr(item, attr, None) is not None:
                th = float(getattr(item, attr))
                break
        if th is None and hasattr(item, "normal_xy") and getattr(item, "normal_xy", None) is not None:
            norm_xy = getattr(item, "normal_xy")
            if isinstance(norm_xy, (tuple, list)) and len(norm_xy) >= 2:
                nx, ny = float(norm_xy[0]), float(norm_xy[1])
                if math.hypot(nx, ny) > 1e-6:
                    th = math.atan2(ny, nx)

    elif isinstance(item, (tuple, list)) and len(item) >= 2:
        try:
            x, y = float(item[0]), float(item[1])
            if len(item) >= 3 and item[2] is not None:
                th = float(item[2])
        except (TypeError, ValueError):
            return None

    if x is not None and y is not None and math.isfinite(x) and math.isfinite(y):
        if th is not None and not math.isfinite(th):
            th = None
        return (x, y, th)
    return None


def extract_work_item_coordinates(
    item: Any,
) -> Optional[Tuple[float, float]]:
    """Safely extract 2D (x, y) coordinates from a WorkItem or pose-like object."""
    pose = extract_work_item_pose(item)
    if pose is None:
        return None
    return (pose[0], pose[1])


def extract_work_item_spawn_distance(item: Any, default: float = 0.0) -> float:
    """Safely extract spawn_distance (in meters) from a WorkItem or dict."""
    if item is None:
        return default
    d = None
    if isinstance(item, dict):
        d = item.get("spawn_distance")
    elif hasattr(item, "spawn_distance"):
        d = getattr(item, "spawn_distance", None)
    elif isinstance(item, (tuple, list)) and len(item) >= 4:
        d = item[3]
    if d is not None:
        try:
            val = float(d)
            if math.isfinite(val) and val >= 0.0:
                return val
        except (TypeError, ValueError):
            pass
    return default


def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Compute 2D Euclidean distance between two points."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def kinematic_time(
    p1: Tuple[float, float, Optional[float]],
    p2: Tuple[float, float, Optional[float]],
    v_max: float = 0.4,
    omega_max: float = 0.8,
) -> float:
    """Compute kinematic transition time between two oriented poses.

    Formula:
        delta_d = ||p2 - p1||_2
        theta_seg = atan2(y2 - y1, x2 - x1)
        delta_th1 = |normalize_angle(theta_seg - theta1)|
        delta_th2 = |normalize_angle(theta2 - theta_seg)|
        T = delta_d / v_max + (delta_th1 + delta_th2) / omega_max

    If delta_d is near 0, pure in-place turn time is used:
        T = |normalize_angle(theta2 - theta1)| / omega_max
    """
    x1, y1 = p1[0], p1[1]
    th1 = p1[2] if len(p1) > 2 else None
    x2, y2 = p2[0], p2[1]
    th2 = p2[2] if len(p2) > 2 else None

    dx = x2 - x1
    dy = y2 - y1
    dist = math.hypot(dx, dy)

    v = max(1e-6, float(v_max))
    w = max(1e-6, float(omega_max))

    if dist <= 1e-6:
        if th1 is not None and th2 is not None:
            return abs(normalize_angle(th2 - th1)) / w
        return 0.0

    t_trans = dist / v

    dth1 = 0.0
    dth2 = 0.0
    seg_th = math.atan2(dy, dx)

    if th1 is not None:
        dth1 = abs(normalize_angle(seg_th - th1))
    if th2 is not None:
        dth2 = abs(normalize_angle(th2 - seg_th))

    t_rot = (dth1 + dth2) / w
    return t_trans + t_rot


def compute_open_tour_time(
    start_pose: Tuple[float, float, Optional[float]],
    target_poses: Sequence[Tuple[float, float, Optional[float]]],
    tour: Sequence[int],
    v_max: float = 0.4,
    omega_max: float = 0.8,
    spawn_distances: Optional[Sequence[float]] = None,
    alpha: float = 0.0,
    beta: float = 0.0,
) -> float:
    """Compute total kinematic time plus spawn penalty of an open tour from start_pose.

    Formula:
        Cost = T_kin(p0 -> W_t0) + beta * D_spawn(W_t0)
             + sum_{k=0}^{m-2} [ T_kin(W_tk -> W_t(k+1)) + alpha * max(0, D_spawn(W_tk) - D_spawn(W_t(k+1))) ]
    """
    if not tour:
        return 0.0

    t0 = tour[0]
    total = kinematic_time(start_pose, target_poses[t0], v_max, omega_max)
    if spawn_distances is not None and beta > 0.0:
        if t0 < len(spawn_distances) and spawn_distances[t0] is not None:
            total += beta * max(0.0, float(spawn_distances[t0]))

    for k in range(len(tour) - 1):
        idx_curr = tour[k]
        idx_next = tour[k + 1]
        total += kinematic_time(
            target_poses[idx_curr], target_poses[idx_next], v_max, omega_max
        )
        if spawn_distances is not None and alpha > 0.0:
            if (
                idx_curr < len(spawn_distances)
                and idx_next < len(spawn_distances)
                and spawn_distances[idx_curr] is not None
                and spawn_distances[idx_next] is not None
            ):
                d_curr = float(spawn_distances[idx_curr])
                d_next = float(spawn_distances[idx_next])
                if d_curr > d_next:
                    total += alpha * (d_curr - d_next)

    return total


def nearest_neighbor_kinematic_open_tsp(
    start_pose: Tuple[float, float, Optional[float]],
    target_poses: Sequence[Tuple[float, float, Optional[float]]],
    v_max: float = 0.4,
    omega_max: float = 0.8,
    spawn_distances: Optional[Sequence[float]] = None,
    alpha: float = 0.0,
    beta: float = 0.0,
) -> List[int]:
    """Construct an initial Open TSP tour using kinematic time + spawn penalty nearest neighbor."""
    n = len(target_poses)
    if n <= 1:
        return list(range(n))

    unvisited = set(range(n))
    tour: List[int] = []
    curr = start_pose
    prev_idx: Optional[int] = None

    while unvisited:
        best_idx = -1
        best_cost = float("inf")
        for idx in unvisited:
            cost = kinematic_time(curr, target_poses[idx], v_max, omega_max)
            if prev_idx is None:
                if spawn_distances is not None and beta > 0.0:
                    if idx < len(spawn_distances) and spawn_distances[idx] is not None:
                        cost += beta * max(0.0, float(spawn_distances[idx]))
            else:
                if spawn_distances is not None and alpha > 0.0:
                    if (
                        prev_idx < len(spawn_distances)
                        and idx < len(spawn_distances)
                        and spawn_distances[prev_idx] is not None
                        and spawn_distances[idx] is not None
                    ):
                        d_prev = float(spawn_distances[prev_idx])
                        d_next = float(spawn_distances[idx])
                        if d_prev > d_next:
                            cost += alpha * (d_prev - d_next)
            if cost < best_cost:
                best_cost = cost
                best_idx = idx

        tour.append(best_idx)
        unvisited.remove(best_idx)
        curr = target_poses[best_idx]
        prev_idx = best_idx

    return tour


def optimize_kinematic_open_tsp(
    start_pose: Tuple[float, float, Optional[float]],
    target_poses: Sequence[Tuple[float, float, Optional[float]]],
    initial_tour: Optional[Sequence[int]] = None,
    v_max: float = 0.4,
    omega_max: float = 0.8,
    max_iterations: int = 500,
    spawn_distances: Optional[Sequence[float]] = None,
    alpha: float = 0.0,
    beta: float = 0.0,
) -> List[int]:
    """Optimize a fixed-start Open TSP tour using 2-opt and Or-opt relocation.

    Because kinematic time T(i -> j) != T(j -> i) depends on orientation,
    both directional 2-opt segment reversals and node insertions are explored
    to eliminate crossings and awkward turnarounds, evaluated by kinematic time
    plus spawn penalty.
    """
    n = len(target_poses)
    if n <= 1:
        return list(range(n))

    if initial_tour is None:
        tour = nearest_neighbor_kinematic_open_tsp(
            start_pose,
            target_poses,
            v_max=v_max,
            omega_max=omega_max,
            spawn_distances=spawn_distances,
            alpha=alpha,
            beta=beta,
        )
    else:
        tour = list(initial_tour)

    best_time = compute_open_tour_time(
        start_pose,
        target_poses,
        tour,
        v_max=v_max,
        omega_max=omega_max,
        spawn_distances=spawn_distances,
        alpha=alpha,
        beta=beta,
    )

    improved = True
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        # 1. 2-opt subsegment reversals
        for i in range(n - 1):
            for j in range(i + 1, n):
                cand_tour = tour[:i] + tour[i : j + 1][::-1] + tour[j + 1:]
                cand_time = compute_open_tour_time(
                    start_pose,
                    target_poses,
                    cand_tour,
                    v_max=v_max,
                    omega_max=omega_max,
                    spawn_distances=spawn_distances,
                    alpha=alpha,
                    beta=beta,
                )
                if cand_time < best_time - 1e-7:
                    tour = cand_tour
                    best_time = cand_time
                    improved = True
                    break
            if improved:
                break

        if improved:
            continue

        # 2. 1-node Relocation (Shift/Insert)
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                node = tour[i]
                rem = tour[:i] + tour[i + 1:]
                cand_tour = rem[:j] + [node] + rem[j:]
                if cand_tour == tour:
                    continue
                cand_time = compute_open_tour_time(
                    start_pose,
                    target_poses,
                    cand_tour,
                    v_max=v_max,
                    omega_max=omega_max,
                    spawn_distances=spawn_distances,
                    alpha=alpha,
                    beta=beta,
                )
                if cand_time < best_time - 1e-7:
                    tour = cand_tour
                    best_time = cand_time
                    improved = True
                    break
            if improved:
                break

        if improved:
            continue

        # 3. 2-node Block Relocation (Or-opt pair insert)
        if n >= 4:
            for i in range(n - 1):
                block = tour[i : i + 2]
                rem = tour[:i] + tour[i + 2:]
                for j in range(len(rem) + 1):
                    cand_tour = rem[:j] + block + rem[j:]
                    if cand_tour == tour:
                        continue
                    cand_time = compute_open_tour_time(
                        start_pose,
                        target_poses,
                        cand_tour,
                        v_max=v_max,
                        omega_max=omega_max,
                        spawn_distances=spawn_distances,
                        alpha=alpha,
                        beta=beta,
                    )
                    if cand_time < best_time - 1e-7:
                        tour = cand_tour
                        best_time = cand_time
                        improved = True
                        break
                if improved:
                    break

    return tour


def nearest_neighbor_open_tsp(
    start_point: Tuple[float, float],
    target_points: Sequence[Tuple[float, float]],
) -> List[int]:
    """Euclidean nearest neighbor open TSP heuristic (retained for backward compatibility)."""
    p0 = (start_point[0], start_point[1], None)
    pts = [(p[0], p[1], None) for p in target_points]
    return nearest_neighbor_kinematic_open_tsp(p0, pts, v_max=1.0, omega_max=1.0)


def two_opt_open_tsp(
    start_point: Tuple[float, float],
    target_points: Sequence[Tuple[float, float]],
    initial_tour: Optional[Sequence[int]] = None,
    max_iterations: int = 500,
) -> List[int]:
    """Euclidean 2-opt open TSP local search (retained for backward compatibility)."""
    p0 = (start_point[0], start_point[1], None)
    pts = [(p[0], p[1], None) for p in target_points]
    return optimize_kinematic_open_tsp(
        p0, pts, initial_tour=initial_tour, v_max=1.0, omega_max=1.0, max_iterations=max_iterations
    )


def is_target_work_item(item: Any) -> bool:
    """Return whether a WorkItem represents target observation / pursuit."""
    if isinstance(item, dict):
        return bool(item.get("is_target") or item.get("kind") == "target_observation")
    return bool(getattr(item, "is_target", False) or getattr(item, "kind", None) == "target_observation")


def order_work_items_open_tsp(
    robot_pose: Any,
    work_items: Sequence[Any],
    max_iterations: int = 500,
    v_max: float = 0.4,
    omega_max: float = 0.8,
    alpha: float = 4.0,
    beta: float = 3.0,
    spawn_distances: Optional[Sequence[float]] = None,
) -> List[Any]:
    """Order WorkItems in the time-optimal Open TSP sequence starting from robot_pose.

    Balances linear translation distance with turning angles using v_max and
    omega_max, incorporating spawn distance penalties (alpha for backtracking,
    beta for initial spawn proximity). Target WorkItems are given absolute
    precedence and placed strictly at the beginning of the sequence. Items
    without valid 2D coordinates are safely appended at the end preserving
    their original relative order.
    """
    if not work_items:
        return []
    if len(work_items) == 1:
        return list(work_items)

    # Prioritize Target WorkItems: target discovery is the overarching mission
    target_items = [it for it in work_items if is_target_work_item(it)]
    if target_items:
        non_target_items = [it for it in work_items if not is_target_work_item(it)]
        start_pt = extract_work_item_coordinates(robot_pose)
        if start_pt is not None:
            def _target_dist(it):
                coord = extract_work_item_coordinates(it)
                return euclidean_distance(start_pt, coord) if coord is not None else float("inf")
            ordered_targets = sorted(target_items, key=_target_dist)
        else:
            ordered_targets = list(target_items)

        if not non_target_items:
            return ordered_targets

        last_target_pose = extract_work_item_pose(ordered_targets[-1])
        next_start = last_target_pose if last_target_pose is not None else robot_pose
        ordered_non_targets = order_work_items_open_tsp(
            next_start,
            non_target_items,
            max_iterations=max_iterations,
            v_max=v_max,
            omega_max=omega_max,
            alpha=alpha,
            beta=beta,
        )
        return ordered_targets + ordered_non_targets

    start_pose = extract_work_item_pose(robot_pose)
    if start_pose is None:
        def _fallback_key(it):
            sd = extract_work_item_spawn_distance(it, default=-1.0)
            return (0, sd) if sd >= 0.0 else (1, 0.0)
        return sorted(work_items, key=_fallback_key)

    valid_items: List[Any] = []
    valid_poses: List[Tuple[float, float, Optional[float]]] = []
    valid_spawn_distances: List[float] = []
    invalid_items: List[Any] = []

    for idx, item in enumerate(work_items):
        pose = extract_work_item_pose(item)
        if pose is not None:
            valid_items.append(item)
            valid_poses.append(pose)
            if spawn_distances is not None and idx < len(spawn_distances):
                d = spawn_distances[idx]
                valid_spawn_distances.append(float(d) if d is not None else 0.0)
            else:
                valid_spawn_distances.append(extract_work_item_spawn_distance(item))
        else:
            invalid_items.append(item)

    if not valid_poses:
        return list(work_items)
    if len(valid_poses) == 1:
        return valid_items + invalid_items

    tour_indices = optimize_kinematic_open_tsp(
        start_pose,
        valid_poses,
        v_max=v_max,
        omega_max=omega_max,
        max_iterations=max_iterations,
        spawn_distances=valid_spawn_distances,
        alpha=alpha,
        beta=beta,
    )
    ordered_valid = [valid_items[idx] for idx in tour_indices]
    return ordered_valid + invalid_items


def total_open_tour_time(
    start_point: Any,
    work_items_or_poses: Sequence[Any],
    v_max: float = 0.4,
    omega_max: float = 0.8,
    spawn_distances: Optional[Sequence[float]] = None,
    alpha: float = 0.0,
    beta: float = 0.0,
) -> float:
    """Compute total kinematic time of an open tour from start_point."""
    start_pose = extract_work_item_pose(start_point)
    if start_pose is None:
        return 0.0

    poses: List[Tuple[float, float, Optional[float]]] = []
    extracted_spawns: List[float] = []
    for idx, item in enumerate(work_items_or_poses):
        p = extract_work_item_pose(item)
        if p is not None:
            poses.append(p)
            if spawn_distances is not None and idx < len(spawn_distances):
                extracted_spawns.append(float(spawn_distances[idx]))
            else:
                extracted_spawns.append(extract_work_item_spawn_distance(item))

    if not poses:
        return 0.0

    return compute_open_tour_time(
        start_pose,
        poses,
        list(range(len(poses))),
        v_max=v_max,
        omega_max=omega_max,
        spawn_distances=extracted_spawns,
        alpha=alpha,
        beta=beta,
    )


def total_open_tour_length(
    start_point: Any,
    work_items_or_coords: Sequence[Any],
) -> float:
    """Compute total Euclidean path length of an open tour from start_point."""
    start_pt = extract_work_item_coordinates(start_point)
    if start_pt is None:
        return 0.0

    current = start_pt
    length = 0.0
    for item in work_items_or_coords:
        pt = extract_work_item_coordinates(item)
        if pt is not None:
            length += euclidean_distance(current, pt)
            current = pt
    return length


class WorkItemTourSolver:
    """Configurable kinematic time-optimal Open TSP solver for WorkItem planning."""

    def __init__(
        self,
        max_iterations: int = 500,
        v_max: float = 0.4,
        omega_max: float = 0.8,
        alpha: float = 3.0,
        beta: float = 2.0,
    ):
        self.max_iterations = max_iterations
        self.v_max = v_max
        self.omega_max = omega_max
        self.alpha = alpha
        self.beta = beta

    def plan_tour(
        self,
        robot_pose: Any,
        work_items: Sequence[Any],
        spawn_distances: Optional[Sequence[float]] = None,
    ) -> List[Any]:
        return order_work_items_open_tsp(
            robot_pose,
            work_items,
            max_iterations=self.max_iterations,
            v_max=self.v_max,
            omega_max=self.omega_max,
            alpha=self.alpha,
            beta=self.beta,
            spawn_distances=spawn_distances,
        )

    def tour_indices(
        self,
        start_point: Any,
        coords_or_poses: Sequence[Any],
        spawn_distances: Optional[Sequence[float]] = None,
    ) -> List[int]:
        pt = extract_work_item_pose(start_point)
        if pt is None:
            return list(range(len(coords_or_poses)))
        target_poses = [
            extract_work_item_pose(c) or (0.0, 0.0, None) for c in coords_or_poses
        ]
        spawns = spawn_distances
        if spawns is None:
            spawns = [extract_work_item_spawn_distance(c) for c in coords_or_poses]
        return optimize_kinematic_open_tsp(
            pt,
            target_poses,
            v_max=self.v_max,
            omega_max=self.omega_max,
            max_iterations=self.max_iterations,
            spawn_distances=spawns,
            alpha=self.alpha,
            beta=self.beta,
        )

    def tour_time(
        self,
        start_point: Any,
        work_items_or_poses: Sequence[Any],
        spawn_distances: Optional[Sequence[float]] = None,
    ) -> float:
        return total_open_tour_time(
            start_point,
            work_items_or_poses,
            v_max=self.v_max,
            omega_max=self.omega_max,
            spawn_distances=spawn_distances,
            alpha=self.alpha,
            beta=self.beta,
        )

    def tour_length(
        self, start_point: Any, work_items_or_coords: Sequence[Any]
    ) -> float:
        return total_open_tour_length(start_point, work_items_or_coords)
