"""Reachable visual-viewpoint selection for GoalManager.

The selector evaluates the bounded approach ladder but does not mutate the
active target segment.  Committing the selected endpoint remains the
GoalManager's responsibility, so route selection can be tested independently
from publication and controller handoff.
"""

from dataclasses import dataclass
import copy
import math
from typing import Optional

from goal_manager_target_viewpoint_geometry import build_target_viewpoint
from goal_manager_target_viewpoint_attempt_ledger import (
    ROUTE_REJECTED,
    VIEWPOINT_ACTIVE,
    VIEWPOINT_ARRIVED,
    VIEWPOINT_CONTROLLER_FAILED,
    VIEWPOINT_OBSERVED,
)


@dataclass
class TargetViewpointSelection:
    """Result of evaluating one visual target approach ladder."""

    goal: object = None
    requested_goal: object = None
    distance: Optional[float] = None
    index: Optional[int] = None
    entry_heading: Optional[float] = None
    entry_heading_error: Optional[float] = None
    distance_penalty: Optional[float] = None
    continuity_score: Optional[float] = None
    continuity_tier: str = "unknown"
    last_requested_goal: object = None
    rejected_distances: tuple = ()
    route_status: object = None
    navigation_heading: Optional[float] = None
    geometry_mode: str = "bearing_horizon"
    target_range: Optional[float] = None
    target_standoff: Optional[float] = None
    remaining_target_range: Optional[float] = None
    candidate_id: str = ""
    attempt_id: str = ""


def _target_viewpoint_ledger(manager):
    transaction = getattr(manager, "target_approach_transaction", None)
    return None if transaction is None else getattr(
        transaction, "viewpoint_ledger", None
    )


def select_target_viewpoint(manager, now: float) -> Optional[TargetViewpointSelection]:
    """Validate and rank the bounded visual approach candidates."""
    navigation_heading = getattr(manager, "target_navigation_heading", None)
    navigation_heading = (
        manager.target_last_heading
        if navigation_heading is None
        else navigation_heading()
    )
    if manager.latest_pose is None or navigation_heading is None:
        return None

    desired_distance = manager.target_segment_distance()
    reachable = []
    rejected_distances = []
    last_requested_goal = None
    route_status = None
    safe_standoff_geometry = None
    ledger = _target_viewpoint_ledger(manager)
    map_epoch = getattr(manager, "target_viewpoint_map_epoch", None)
    transaction = getattr(manager, "target_approach_transaction", None)
    if ledger is not None and not ledger.active and getattr(
        transaction, "active", False
    ):
        ledger.begin(
            getattr(manager, "target_track_id", ""),
            int(
                getattr(
                    transaction,
                    "target_epoch",
                    0,
                )
                or 0
            ),
        )

    for candidate_index, distance in enumerate(manager.target_segment_distances()):
        geometry = build_target_viewpoint(manager, navigation_heading, distance)
        if geometry is None:
            return None
        if geometry.mode == "safe_standoff":
            # A static target estimate says the robot already has a safe view.
            # Do not turn this into a zero-length Navfn request; the caller
            # owns the observation hold and completion evidence instead.
            safe_standoff_geometry = geometry
            route_status = "safe_standoff_reached"
            break
        requested_odom = manager.make_goal_pose(
            (geometry.x, geometry.y, 0.0), geometry.yaw
        )
        requested_goal = manager._pose_in_frame(requested_odom, "map")
        if requested_goal is None:
            manager.target_execution_state = "TARGET_ROUTE_PENDING"
            route_status = None
            break

        last_requested_goal = copy.deepcopy(requested_goal)
        candidate = None
        if ledger is not None:
            candidate = ledger.ensure_candidate(
                candidate_index,
                requested_goal,
                map_epoch=map_epoch,
            )
            if candidate is not None and candidate.route_status in (
                ROUTE_REJECTED,
                VIEWPOINT_ACTIVE,
                VIEWPOINT_ARRIVED,
                VIEWPOINT_CONTROLLER_FAILED,
                VIEWPOINT_OBSERVED,
            ):
                manager.publish_goal_arbitration(
                    "target_viewpoint_skipped",
                    target_track_id=manager.target_track_id,
                    candidate_id=candidate.candidate_id,
                    candidate_index=int(candidate_index),
                    status=candidate.route_status,
                    attempt_id=candidate.attempt_id,
                    failure_reason=candidate.failure_reason,
                )
                continue
        route_status = manager.validate_target_route(
            requested_goal,
            now,
            force=(candidate_index > 0),
        )
        if route_status is True:
            endpoint = manager.target_route_validation_last_endpoint
            if endpoint is None:
                route_status = None
                break
            if ledger is not None:
                candidate = ledger.mark_route_ready(
                    candidate_index,
                    requested_goal,
                    endpoint,
                    copy.deepcopy(
                        getattr(
                            manager, "target_route_validation_last_plan", None
                        )
                    ),
                    map_epoch=map_epoch,
                    now=now,
                )
            entry_heading, heading_error, distance_penalty, score, tier = (
                manager.target_route_continuity(
                    geometry.distance_from_robot, desired_distance
                )
            )
            reachable.append({
                "goal": copy.deepcopy(endpoint),
                "requested_goal": copy.deepcopy(requested_goal),
                "distance": float(geometry.distance_from_robot),
                "index": int(candidate_index),
                "entry_heading": entry_heading,
                "entry_heading_error": heading_error,
                "distance_penalty": distance_penalty,
                "continuity_score": score,
                "continuity_tier": tier,
                "geometry_mode": geometry.mode,
                "target_range": geometry.target_range,
                "target_standoff": geometry.target_standoff,
                "remaining_target_range": geometry.remaining_target_range,
                "candidate_id": (
                    "" if candidate is None else candidate.candidate_id
                ),
                "attempt_id": (
                    "" if candidate is None else candidate.attempt_id
                ),
            })
            manager.publish_goal_arbitration(
                "target_viewpoint_evaluated",
                target_track_id=manager.target_track_id,
                strategy=manager.target_approach_strategy,
                candidate_index=int(candidate_index),
                candidate_id=(
                    None if candidate is None else candidate.candidate_id
                ),
                distance=round(float(geometry.distance_from_robot), 3),
                navfn_entry_heading=(
                    None if entry_heading is None else round(float(entry_heading), 4)
                ),
                navfn_entry_heading_error=(
                    None if heading_error is None else round(float(heading_error), 4)
                ),
                continuity_distance_penalty=round(float(distance_penalty), 4),
                continuity_score=round(float(score), 4),
                continuity_tier=tier,
                geometry_mode=geometry.mode,
                target_range=(
                    None
                    if geometry.target_range is None
                    else round(float(geometry.target_range), 3)
                ),
                target_standoff=(
                    None
                    if geometry.target_standoff is None
                    else round(float(geometry.target_standoff), 3)
                ),
                remaining_target_range=(
                    None
                    if geometry.remaining_target_range is None
                    else round(float(geometry.remaining_target_range), 3)
                ),
            )
            continue

        if route_status is None:
            break
        if ledger is not None:
            candidate = ledger.mark_route_rejected(
                candidate_index,
                "navfn_empty_plan",
                map_epoch=map_epoch,
                now=now,
            )
        rejected_distances.append(round(float(distance), 3))
        manager.publish_goal_arbitration(
            "target_viewpoint_rejected",
            target_track_id=manager.target_track_id,
            strategy=manager.target_approach_strategy,
            candidate_index=int(candidate_index),
            candidate_id=(None if candidate is None else candidate.candidate_id),
            distance=round(float(distance), 3),
            heading=round(float(manager.target_last_heading), 4),
        )

    if not reachable:
        # ``None`` means that Navfn has not answered yet.  A ledger with no
        # open candidates is a different, terminal fact: every physical
        # viewpoint in this target episode has already been consumed.
        if (
            safe_standoff_geometry is None
            and ledger is not None
            and ledger.alternatives_exhausted()
        ):
            route_status = "viewpoint_portfolio_exhausted"
        return TargetViewpointSelection(
            last_requested_goal=last_requested_goal,
            rejected_distances=tuple(rejected_distances),
            route_status=route_status,
            navigation_heading=float(navigation_heading),
            geometry_mode=(
                "safe_standoff"
                if safe_standoff_geometry is not None
                else "bearing_horizon"
            ),
            target_range=(
                None
                if safe_standoff_geometry is None
                else safe_standoff_geometry.target_range
            ),
            target_standoff=(
                None
                if safe_standoff_geometry is None
                else safe_standoff_geometry.target_standoff
            ),
            remaining_target_range=(
                None
                if safe_standoff_geometry is None
                else safe_standoff_geometry.remaining_target_range
            ),
        )

    if manager.target_route_continuity_enabled:
        selected = min(
            reachable,
            key=lambda item: (
                0 if item["continuity_tier"] == "smooth" else 1,
                float(item["continuity_score"]),
                -float(item["distance"]),
            ),
        )
    else:
        selected = reachable[0]

    return TargetViewpointSelection(
        goal=selected["goal"],
        requested_goal=selected["requested_goal"],
        distance=selected["distance"],
        index=selected["index"],
        entry_heading=selected["entry_heading"],
        entry_heading_error=selected["entry_heading_error"],
        distance_penalty=selected["distance_penalty"],
        continuity_score=selected["continuity_score"],
        continuity_tier=selected["continuity_tier"],
        last_requested_goal=last_requested_goal,
        rejected_distances=tuple(rejected_distances),
        route_status=route_status,
        navigation_heading=float(navigation_heading),
        geometry_mode=selected["geometry_mode"],
        target_range=selected["target_range"],
        target_standoff=selected["target_standoff"],
        remaining_target_range=selected["remaining_target_range"],
        candidate_id=selected.get("candidate_id", ""),
        attempt_id=selected.get("attempt_id", ""),
    )
