"""Persistent, place-owned frontier work for online exploration.

A structural component is a transient description of free space in one SLAM
snapshot.  A work item is instead an observation boundary owned by the
physical place where it was discovered.  That distinction prevents furniture
from turning one office into a stream of newly minted exploration places.
"""

import math

from global_frontier_work_item_components import support_overlap


# A WorkItem answers one semantic question: has this unknown-side boundary
# been observed sufficiently to resolve it?  It intentionally does not carry
# route failure state.  A route reaches a particular safe viewpoint, whereas
# the WorkItem owns the physical observation boundary and can require another
# viewpoint after that route fails.
WORK_UNRESOLVED = "unresolved"
WORK_RESOLVED = "resolved"

# Kept as imports for older callers/tests. They are *not* WorkItem states:
# ``dispatched`` and ``deferred`` are now Attempt lifecycle values.
WORK_UNSEEN = WORK_UNRESOLVED
WORK_DISPATCHED = "dispatched"
WORK_DEFERRED = "deferred"

ATTEMPT_ACTIVE = "active"
ATTEMPT_FAILED = "failed"
ATTEMPT_BLOCKED = "blocked"
ATTEMPT_SUCCEEDED = "succeeded"


class PlaceWorkItemLedger:
    """Keep a small lifecycle ledger for frontier observations per Place."""

    def __init__(self, merge_radius, viewpoint_merge_radius=None):
        self.merge_radius = max(0.05, float(merge_radius))
        # ``merge_radius`` identifies one physical unknown boundary across
        # SLAM snapshots.  It must not also collapse distinct recovery poses:
        # a failed corner view may need a lateral view well inside that
        # long-term identity footprint.  Keep the old behavior for injected
        # callers that omit the optional value, while production passes the
        # observation-lattice scale explicitly.
        if viewpoint_merge_radius is None:
            viewpoint_merge_radius = self.merge_radius
        self.viewpoint_merge_radius = max(
            0.05, min(self.merge_radius, float(viewpoint_merge_radius))
        )
        self._next_id = 1
        self._next_attempt_id = 1
        self._items = {}

    @staticmethod
    def _normalise_support(support_cells):
        """Make support signatures safe to retain across map snapshots."""
        if not support_cells:
            return frozenset()
        return frozenset((int(row), int(col)) for row, col in support_cells)

    @staticmethod
    def _normalise_xy(xy):
        if xy is None:
            return None
        try:
            return float(xy[0]), float(xy[1])
        except (IndexError, TypeError, ValueError):
            return None

    @classmethod
    def _normalise_direction(cls, direction):
        """Return a unit direction, or ``None`` for legacy/malformed data."""
        direction = cls._normalise_xy(direction)
        if direction is None:
            return None
        norm = math.hypot(direction[0], direction[1])
        if not math.isfinite(norm) or norm <= 1e-9:
            return None
        return direction[0] / norm, direction[1] / norm

    @classmethod
    def _directions_compatible(cls, left, right):
        """Require two known normals to point into the same unknown half-plane.

        Missing normals are accepted for compatibility with work created by
        older callers.  Once both snapshots provide geometry, an opposite (or
        orthogonal) direction cannot inherit the same physical observation
        task, even when their coarse support lattices overlap.
        """
        left = cls._normalise_direction(left)
        right = cls._normalise_direction(right)
        if left is None or right is None:
            return True
        return left[0] * right[0] + left[1] * right[1] > 0.0

    def _anchors_compatible(self, left, right):
        """Return whether two known physical anchors can share one item."""
        left = self._normalise_xy(left)
        right = self._normalise_xy(right)
        if left is None or right is None:
            # Older records have no anchor.  Their support overlap remains the
            # compatibility path until a physical anchor is reacquired.
            return True
        return math.hypot(left[0] - right[0], left[1] - right[1]) <= self.merge_radius

    def _new_item(
        self, place_id, support_cells, now, xy=None,
        reason="unknown_boundary_discovered", anchor_xy=None, normal_xy=None,
    ):
        xy = self._normalise_xy(xy)
        item = {
            "id": self._next_id,
            "place_id": int(place_id),
            "x": None if xy is None else xy[0],
            "y": None if xy is None else xy[1],
            "anchor_xy": self._normalise_xy(anchor_xy),
            "normal_xy": self._normalise_direction(normal_xy),
            "support_cells": support_cells,
            # Semantic state: only observation evidence may resolve this.
            "state": WORK_UNRESOLVED,
            "last_reason": str(reason),
            "updated_at": float(now),
            "attempts": [],
            # A route can be rejected before a controller Attempt exists. Keep
            # that geometric admission fact separate from execution history so
            # the same physical boundary can be reprojected from another
            # viewpoint without pretending that the robot already tried it.
            "route_rejections": [],
            "active_attempt_id": None,
        }
        self._next_id += 1
        self._items[int(item["id"])] = item
        return item

    @staticmethod
    def _active_attempt(item):
        active_id = item.get("active_attempt_id")
        if active_id is None:
            return None
        for attempt in item.get("attempts", ()):
            if int(attempt["id"]) == int(active_id):
                return attempt
        # Ledger corruption must not leave a semantic item locked forever.
        item["active_attempt_id"] = None
        return None

    def _attempted_same_viewpoint(self, item, viewpoint_xy):
        """Return a failed attempt at this physical viewpoint, if any.

        The same odom-frame viewpoint after a SLAM correction is still the
        same physical route attempt.  The existing WorkItem merge footprint
        supplies the only tolerance: this is identity association, not a
        new controller tuning knob.
        """
        viewpoint_xy = self._normalise_xy(viewpoint_xy)
        if viewpoint_xy is None:
            return None
        for attempt in item.get("attempts", ()):
            if attempt.get("state") not in (ATTEMPT_FAILED, ATTEMPT_BLOCKED):
                continue
            attempt_xy = self._normalise_xy(attempt.get("viewpoint_xy"))
            if attempt_xy is None:
                continue
            if math.hypot(
                viewpoint_xy[0] - attempt_xy[0],
                viewpoint_xy[1] - attempt_xy[1],
            ) <= self.viewpoint_merge_radius:
                return attempt
        return None

    @staticmethod
    def _normalise_epoch(value):
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _route_rejected_same_viewpoint(
        self, item, viewpoint_xy, map_epoch=None,
    ):
        """Return a pre-dispatch route rejection at this physical viewpoint."""
        viewpoint_xy = self._normalise_xy(viewpoint_xy)
        if viewpoint_xy is None:
            return None
        map_epoch = self._normalise_epoch(map_epoch)
        for rejection in item.get("route_rejections", ()):
            rejection_epoch = self._normalise_epoch(rejection.get("map_epoch"))
            # Route-admission facts belong to one short-lived map snapshot.
            # Keep the record for replay, but never let an older epoch block a
            # newly projected viewpoint after SLAM/costmap evidence changes.
            if (
                rejection_epoch is not None
                and map_epoch is not None
                and rejection_epoch != map_epoch
            ):
                continue
            rejection_xy = self._normalise_xy(rejection.get("viewpoint_xy"))
            if rejection_xy is None:
                continue
            if math.hypot(
                viewpoint_xy[0] - rejection_xy[0],
                viewpoint_xy[1] - rejection_xy[1],
            ) <= self.viewpoint_merge_radius:
                return rejection
        return None

    def reconcile(self, place_id, supports, now):
        """Match current frontier arcs to persistent physical work items.

        A support is an unknown-side region in odom coordinates, not a goal
        pose.  Any later arc overlapping a resolved support is its descendant
        and therefore cannot be dispatched again merely because the frontier
        moved farther into the same unobserved direction.
        """
        if place_id is None:
            return {}, {}
        cell_to_item = {}
        details = {}
        for support in supports:
            support_cells = self._normalise_support(support.support_cells)
            if not support_cells:
                continue
            support_anchor = self._normalise_xy(
                getattr(support, "anchor_xy", None)
            )
            matches = []
            for item in self._items.values():
                if int(item["place_id"]) != int(place_id):
                    continue
                prior_state = item["state"]
                overlap = support_overlap(support_cells, item.get("support_cells", ()))
                if not self._directions_compatible(
                    getattr(support, "normal_xy", None), item.get("normal_xy")
                ):
                    continue
                prior_anchor = self._normalise_xy(item.get("anchor_xy"))
                if prior_anchor is not None and support_anchor is not None:
                    # Physical anchors are the durable identity.  Support
                    # cells belong to a transient SLAM snapshot and may be
                    # completely disjoint after map relabelling.
                    anchor_match = self._anchors_compatible(
                        prior_anchor, support_anchor
                    )
                    # A resolved support is already physical evidence that
                    # this unknown patch was observed.  A later frontier that
                    # overlaps that patch is its descendant even when the
                    # frontier anchor has advanced beyond the identity radius.
                    if not anchor_match and not (
                        overlap and prior_state == WORK_RESOLVED
                    ):
                        continue
                    identity_match = anchor_match or bool(overlap)
                    match_quality = 2 if anchor_match else 1
                else:
                    # Compatibility for pre-anchor records is intentionally
                    # conservative: only support overlap can inherit them.
                    identity_match = bool(overlap)
                    match_quality = 0
                if identity_match:
                    matches.append((
                        match_quality,
                        overlap,
                        -int(item["id"]),
                        item,
                    ))
            if matches:
                _anchor_match, _overlap, _negative_id, item = max(
                    matches, key=lambda value: value[:3]
                )
                prior_state = item["state"]
                item["support_cells"] = frozenset(item["support_cells"] | support_cells)
                support_anchor = self._normalise_xy(
                    getattr(support, "anchor_xy", None)
                )
                if support_anchor is not None:
                    item["anchor_xy"] = support_anchor
                support_direction = self._normalise_direction(
                    getattr(support, "normal_xy", None)
                )
                if item.get("normal_xy") is None and support_direction is not None:
                    item["normal_xy"] = support_direction
                item["updated_at"] = float(now)
                match = (
                    "resolved_descendant"
                    if prior_state == WORK_RESOLVED
                    else "inherited"
                )
            else:
                item = self._new_item(
                    place_id,
                    support_cells,
                    now,
                    anchor_xy=getattr(support, "anchor_xy", None),
                    normal_xy=getattr(support, "normal_xy", None),
                )
                match = "created"
            item_id = int(item["id"])
            detail = {
                "id": item_id,
                "state": item["state"],
                "match": match,
                "support_cell_count": len(support_cells),
                "anchor_xy": item.get("anchor_xy"),
                "normal_xy": item.get("normal_xy"),
            }
            details[item_id] = detail
            for cell in support.frontier_cells:
                cell_to_item[(int(cell[0]), int(cell[1]))] = item_id
        return cell_to_item, details

    def work_item_is_available(self, item_id):
        """Return whether an unresolved boundary has no active Attempt."""
        item = self._items.get(item_id)
        return (
            item is not None
            and item["state"] == WORK_UNRESOLVED
            and self._active_attempt(item) is None
        )

    def viewpoint_is_available(self, item_id, viewpoint_xy, map_epoch=None):
        """Return whether this is a new safe viewpoint for an unresolved item."""
        item = self._items.get(item_id)
        if item is None or not self.work_item_is_available(item_id):
            return False
        return (
            self._attempted_same_viewpoint(item, viewpoint_xy) is None
            and self._route_rejected_same_viewpoint(
                item, viewpoint_xy, map_epoch=map_epoch,
            ) is None
        )

    def route_rejection_for_viewpoint(
        self, item_id, viewpoint_xy, map_epoch=None,
    ):
        """Return the current-epoch pre-dispatch rejection, if any."""
        item = self._items.get(item_id)
        if item is None:
            return None
        return self._route_rejected_same_viewpoint(
            item, viewpoint_xy, map_epoch=map_epoch,
        )

    def has_failed_viewpoint(self, item_id, map_epoch=None):
        """Return whether an unresolved item needs a different viewpoint.

        This is WorkItem state, not a retry counter. The selector may promote
        another map-safe viewpoint after either an executed viewpoint ended in
        ``failed``/``blocked`` or Navfn rejected the viewpoint before dispatch.
        """
        item = self._items.get(item_id)
        if item is None or item["state"] != WORK_UNRESOLVED:
            return False
        return any(
            attempt.get("state") in (ATTEMPT_FAILED, ATTEMPT_BLOCKED)
            for attempt in item.get("attempts", ())
        ) or any(
            self._route_rejected_same_viewpoint(
                item, rejection.get("viewpoint_xy"), map_epoch=map_epoch,
            ) is not None
            for rejection in item.get("route_rejections", ())
        )

    def record_route_rejection(
        self, item_id, now, viewpoint_xy, map_goal=None, reason="",
        map_epoch=None, plan_endpoint=None,
    ):
        """Record a route-admission failure without opening an Attempt.

        Navfn/costmap validation happens before route activation, so it is not
        an execution failure and must not consume an Attempt ID. The durable
        WorkItem still needs to remember that exact physical standoff until a
        different viewpoint on the same unknown boundary is compiled.
        """
        item = self._items.get(item_id)
        if item is None or item.get("state") != WORK_UNRESOLVED:
            return None, None
        if self._active_attempt(item) is not None:
            return item, None
        viewpoint_xy = self._normalise_xy(viewpoint_xy)
        if viewpoint_xy is None:
            return item, None
        existing = self._route_rejected_same_viewpoint(
            item, viewpoint_xy, map_epoch=map_epoch,
        )
        if existing is not None:
            return item, existing
        rejection = {
            "viewpoint_xy": viewpoint_xy,
            "map_goal": self._normalise_xy(map_goal),
            "map_epoch": self._normalise_epoch(map_epoch),
            "plan_endpoint": self._normalise_xy(plan_endpoint),
            "reason": str(reason),
            "recorded_at": float(now),
        }
        item.setdefault("route_rejections", []).append(rejection)
        item["last_reason"] = str(reason)
        item["updated_at"] = float(now)
        return item, rejection

    def active_attempt_id(self, item_id):
        item = self._items.get(item_id)
        if item is None:
            return None
        attempt = self._active_attempt(item)
        return None if attempt is None else int(attempt["id"])

    def dispatch_existing(
        self, item_id, now, viewpoint_xy=None, map_goal=None, route_kind=None,
        map_epoch=None,
    ):
        """Start one Attempt without changing the WorkItem semantic state."""
        item = self._items.get(item_id)
        if item is None or not self.work_item_is_available(item_id):
            return None
        if viewpoint_xy is not None and not self.viewpoint_is_available(
            item_id, viewpoint_xy, map_epoch=map_epoch,
        ):
            return None
        attempt = {
            "id": self._next_attempt_id,
            "state": ATTEMPT_ACTIVE,
            "viewpoint_xy": self._normalise_xy(viewpoint_xy),
            "map_goal": self._normalise_xy(map_goal),
            "route_kind": None if route_kind is None else str(route_kind),
            "map_epoch": self._normalise_epoch(map_epoch),
            "started_at": float(now),
            "ended_at": None,
            "reason": "route_dispatched",
        }
        self._next_attempt_id += 1
        item["attempts"].append(attempt)
        item["active_attempt_id"] = int(attempt["id"])
        item["last_reason"] = "route_dispatched"
        item["updated_at"] = float(now)
        return int(item["id"])

    def _nearby(self, place_id, xy):
        if place_id is None or xy is None:
            return None
        try:
            x, y = float(xy[0]), float(xy[1])
        except (IndexError, TypeError, ValueError):
            return None
        best = None
        best_distance = float("inf")
        for item in self._items.values():
            if int(item["place_id"]) != int(place_id):
                continue
            if item.get("x") is None or item.get("y") is None:
                continue
            distance = math.hypot(x - item["x"], y - item["y"])
            if distance <= self.merge_radius and distance < best_distance:
                best = item
                best_distance = distance
        return best

    def candidate_is_available(self, place_id, xy):
        """Return whether a nearby WorkItem permits this physical viewpoint."""
        item = self._nearby(place_id, xy)
        return item is None or self.viewpoint_is_available(item["id"], xy)

    def dispatch(self, place_id, xy, now, map_epoch=None):
        """Create or reserve the one work item represented by ``xy``."""
        if place_id is None or xy is None:
            return None
        item = self._nearby(place_id, xy)
        if item is None:
            item = self._new_item(
                place_id, frozenset(), now, xy=xy, reason="discovered",
            )
        return self.dispatch_existing(
            int(item["id"]),
            now,
            viewpoint_xy=xy,
            map_goal=xy,
            route_kind="frontier_endpoint",
            map_epoch=map_epoch,
        )

    def _finish_active_attempt(self, item, state, now, reason):
        attempt = self._active_attempt(item)
        if attempt is None:
            return None
        attempt["state"] = str(state)
        attempt["ended_at"] = float(now)
        attempt["reason"] = str(reason)
        item["active_attempt_id"] = None
        item["last_reason"] = str(reason)
        item["updated_at"] = float(now)
        return attempt

    def resolve(self, item_id, now, reason, attempt_id=None):
        """Resolve work only from observation evidence, never route failure."""
        item = self._items.get(item_id)
        if item is None:
            return None
        active = self._active_attempt(item)
        if active is not None:
            # A terminal without ownership metadata is ambiguous once a
            # replacement Attempt is active.  Leave that Attempt unresolved;
            # only its exact lease may settle the semantic WorkItem.
            if attempt_id is None or int(active["id"]) != int(attempt_id):
                return None
            self._finish_active_attempt(item, ATTEMPT_SUCCEEDED, now, reason)
        elif attempt_id is not None:
            return None
        item["state"] = WORK_RESOLVED
        item["last_reason"] = str(reason)
        item["updated_at"] = float(now)
        return item

    def fail_attempt(self, item_id, now, reason, state=ATTEMPT_FAILED, attempt_id=None):
        """End one bad route while keeping its observation task unresolved."""
        if state not in (ATTEMPT_FAILED, ATTEMPT_BLOCKED):
            raise ValueError("failed or blocked attempt state required")
        item = self._items.get(item_id)
        if item is None or item["state"] != WORK_UNRESOLVED:
            return None, None
        active = self._active_attempt(item)
        if active is None or (
            attempt_id is not None and int(active["id"]) != int(attempt_id)
        ):
            return item, None
        return item, self._finish_active_attempt(item, state, now, reason)

    def settle(self, item_id, state, now, reason):
        """Compatibility facade for old lifecycle callers.

        ``deferred`` used to retire the semantic task.  It now records only a
        failed viewpoint Attempt; callers must use ``resolved`` after real
        observation evidence.
        """
        if state == WORK_RESOLVED:
            return self.resolve(
                item_id,
                now,
                reason,
                attempt_id=self.active_attempt_id(item_id),
            )
        if state == WORK_DEFERRED:
            item, _attempt = self.fail_attempt(item_id, now, reason)
            return item
        raise ValueError("resolved work item or deferred attempt required")

    def unresolved_count(self, place_id):
        """Return work still eligible for this physical place."""
        return sum(
            1
            for item in self._items.values()
            if int(item["place_id"]) == int(place_id)
            and item["state"] == WORK_UNRESOLVED
        )

    def unresolved_observation_count(self, place_id, portal_probe_ledger=None):
        """Count ordinary local WorkItems, excluding doorway-owned items.

        A Portal probe may intentionally keep its associated WorkItem
        unresolved while it moves through ``source_arrived`` and
        ``destination_active`` evidence.  That is an outward edge obligation,
        not unfinished room coverage.  Keeping this distinction in the ledger
        prevents the Place phase from waiting forever for a probe to become a
        Portal certificate.
        """
        try:
            place_id = int(place_id)
        except (TypeError, ValueError):
            return 0
        probe_for_work = (
            None if portal_probe_ledger is None
            else getattr(portal_probe_ledger, "probe_for_work_item", None)
        )
        count = 0
        for item in self._items.values():
            if (
                int(item.get("place_id", -1)) != place_id
                or item.get("state") != WORK_UNRESOLVED
            ):
                continue
            if callable(probe_for_work) and probe_for_work(item.get("id")) is not None:
                continue
            count += 1
        return count

    def snapshot(self):
        """Return a read-only, JSON-safe view for graph-level planning.

        The graph planner must inspect durable WorkItems without reaching into
        this ledger's mutable records.  Copying the small lifecycle fields here
        also makes a planning decision replayable without exposing support
        geometry or allowing a caller to mutate the live attempt state.
        """
        result = []
        for item_id in sorted(self._items):
            item = self._items[item_id]
            attempts = []
            for attempt in item.get("attempts", ()):
                attempts.append(
                    {
                        "id": int(attempt["id"]),
                        "state": str(attempt.get("state", "")),
                        "viewpoint_xy": (
                            None
                            if attempt.get("viewpoint_xy") is None
                            else [
                                float(attempt["viewpoint_xy"][0]),
                                float(attempt["viewpoint_xy"][1]),
                            ]
                        ),
                        "map_goal": (
                            None
                            if attempt.get("map_goal") is None
                            else [
                                float(attempt["map_goal"][0]),
                                float(attempt["map_goal"][1]),
                            ]
                        ),
                        "route_kind": attempt.get("route_kind"),
                        "map_epoch": attempt.get("map_epoch"),
                        "reason": str(attempt.get("reason", "")),
                    }
                )
            route_rejections = []
            for rejection in item.get("route_rejections", ()):
                route_rejections.append(
                    {
                        "viewpoint_xy": (
                            None
                            if rejection.get("viewpoint_xy") is None
                            else [
                                float(rejection["viewpoint_xy"][0]),
                                float(rejection["viewpoint_xy"][1]),
                            ]
                        ),
                        "map_goal": (
                            None
                            if rejection.get("map_goal") is None
                            else [
                                float(rejection["map_goal"][0]),
                                float(rejection["map_goal"][1]),
                            ]
                        ),
                        "map_epoch": rejection.get("map_epoch"),
                        "plan_endpoint": (
                            None
                            if rejection.get("plan_endpoint") is None
                            else [
                                float(rejection["plan_endpoint"][0]),
                                float(rejection["plan_endpoint"][1]),
                            ]
                        ),
                        "reason": str(rejection.get("reason", "")),
                        "recorded_at": float(rejection.get("recorded_at", 0.0)),
                    }
                )
            result.append(
                {
                    "id": int(item["id"]),
                    "place_id": int(item["place_id"]),
                    "state": str(item.get("state", WORK_UNRESOLVED)),
                    "x": item.get("x"),
                    "y": item.get("y"),
                    "anchor_xy": item.get("anchor_xy"),
                    "normal_xy": item.get("normal_xy"),
                    "support_cell_count": len(item.get("support_cells", ())),
                    "active_attempt_id": (
                        None
                        if item.get("active_attempt_id") is None
                        else int(item["active_attempt_id"])
                    ),
                    "updated_at": float(item.get("updated_at", 0.0)),
                    "attempt_count": len(attempts),
                    "attempts": attempts,
                    "route_rejection_count": len(route_rejections),
                    "route_rejections": route_rejections,
                }
            )
        return result


class GlobalFrontierWorkItemLifecycleMixin:
    """Bind pure work-item transitions to one active frontier lease."""

    def work_item_physical_xy(self, message, x, y):
        """Project an endpoint once; map coordinates are only a TF fallback."""
        header = getattr(message, "header", None)
        map_frame = getattr(header, "frame_id", "map") or "map"
        transform = getattr(self, "transform_xy", None)
        if transform is not None:
            physical = transform("odom", map_frame, float(x), float(y))
            if physical is not None:
                return physical
        return float(x), float(y)

    def ensure_current_physical_place(
        self, message, components, known_free, robot_map, now,
    ):
        """Bootstrap the one Place occupied at process start.

        This is the sole non-portal place creation: before the first doorway
        crossing the robot is necessarily somewhere.  Every later physical
        Place is created by the certified portal-arrival transaction.
        """
        current_id = getattr(self, "current_physical_place_id", None)
        if current_id is not None and self.region_memory.by_id(current_id) is not None:
            bind_pending = getattr(self, "_bind_pending_semantic_observations", None)
            if bind_pending is not None:
                bind_pending(int(current_id))
            bind_target = getattr(
                self, "_bind_pending_target_belief_observations", None
            )
            if bind_target is not None:
                bind_target(int(current_id))
            return self.region_memory.by_id(current_id)
        component = self.component_at_map_position(
            message, components, known_free, robot_map[0], robot_map[1],
        )
        if component is None:
            return None
        region, _tier = self.region_memory.activate(
            robot_map[0],
            robot_map[1],
            0.0,
            now,
            component=component,
            physical_xy=(
                None
                if getattr(self, "pose_odom", None) is None
                else (float(self.pose_odom.x), float(self.pose_odom.y))
            ),
        )
        if region is not None:
            self.current_physical_place_id = int(region["id"])
            bind_pending = getattr(self, "_bind_pending_semantic_observations", None)
            if bind_pending is not None:
                bind_pending(int(region["id"]))
            bind_target = getattr(
                self, "_bind_pending_target_belief_observations", None
            )
            if bind_target is not None:
                bind_target(int(region["id"]))
            self.publish_status(
                "physical_place_bootstrapped",
                region_id=int(region["id"]),
                source=[round(float(robot_map[0]), 3), round(float(robot_map[1]), 3)],
            )
        return region

    def dispatch_active_work_item(self, message, selection, region, now):
        """Reserve local work when a route is committed, before it is published.

        The ledger must close immediately so another planning pass cannot pick
        the same WorkItem.  Its *observable* dispatch event is intentionally
        delayed until the first route command has been published: selected
        candidates are not yet executed work, and a ROS observer may attach
        between selection and the first controller command.
        """
        self.active_work_item_id = None
        self.active_work_item_attempt_id = None
        self.active_work_item_place_id = None
        self.active_work_item_goal = None
        self.active_work_item_route_kind = None
        self.active_work_item_dispatch_announced = False
        if not getattr(self, "work_item_memory_enabled", True):
            return None
        if (
            region is None
            or selection.route_kind != "frontier_endpoint"
            or selection.place_hops != 0
        ):
            return None
        viewpoint_xy = self.work_item_physical_xy(
            message, selection.x, selection.y,
        )
        component = getattr(selection, "component", None)
        map_epoch = (
            component.get("epoch")
            if isinstance(component, dict) else getattr(component, "epoch", None)
        )
        item_id = getattr(selection, "work_item_id", None)
        if item_id is not None:
            item_id = self.place_work_items.dispatch_existing(
                item_id,
                now,
                viewpoint_xy=viewpoint_xy,
                map_goal=(selection.x, selection.y),
                route_kind=selection.route_kind,
                map_epoch=map_epoch,
            )
        else:
            # Compatibility fallback for callers that do not yet build a
            # frontier-arc snapshot (for example a narrow test fixture).
            item_id = self.place_work_items.dispatch(
                int(region["id"]),
                viewpoint_xy,
                now,
                map_epoch=map_epoch,
            )
        self.active_work_item_id = item_id
        if item_id is not None:
            self.active_work_item_attempt_id = self.place_work_items.active_attempt_id(
                item_id,
            )
            self.active_work_item_place_id = int(region["id"])
            self.active_work_item_goal = (float(selection.x), float(selection.y))
            self.active_work_item_route_kind = str(selection.route_kind)
        return item_id

    def record_work_item_route_rejection(
        self, message, candidate, now, reason, *, map_epoch=None,
        plan_endpoint=None,
    ):
        """Persist a pre-dispatch Navfn/costmap rejection for one WorkItem."""
        if not getattr(self, "work_item_memory_enabled", True):
            return None
        route = getattr(candidate, "route", candidate)
        try:
            item_id = int(route[12])
            map_goal = (float(route[2]), float(route[3]))
        except (IndexError, TypeError, ValueError):
            return None
        physical = self.work_item_physical_xy(message, map_goal[0], map_goal[1])
        item, rejection = self.place_work_items.record_route_rejection(
            item_id,
            now,
            physical,
            map_goal=map_goal,
            reason=reason,
            map_epoch=map_epoch,
            plan_endpoint=plan_endpoint,
        )
        if item is None or rejection is None:
            return rejection
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "work_item_viewpoint_route_rejected",
                work_item_id=int(item_id),
                place_id=int(item["place_id"]),
                viewpoint_xy=[round(float(value), 3) for value in physical],
                map_goal=[round(float(value), 3) for value in map_goal],
                map_epoch=map_epoch,
                plan_endpoint=(
                    None
                    if plan_endpoint is None
                    else [round(float(value), 3) for value in plan_endpoint]
                ),
                reason=str(reason),
                attempt_state="not_started",
            )
        return rejection

    def reserve_prefetched_work_item(self, message, x, y, region, now):
        """Reserve a promoted same-Place item through the normal capability gate.

        Prefetch promotion is only a route-continuity optimization.  It must
        not create Place or WorkItem state for a geometry-only baseline just
        because the candidate happened to be cached before its predecessor
        completed.
        """
        self.active_work_item_id = None
        self.active_work_item_attempt_id = None
        self.active_work_item_place_id = None
        self.active_work_item_goal = None
        self.active_work_item_route_kind = None
        self.active_work_item_dispatch_announced = False
        if not getattr(self, "work_item_memory_enabled", True) or region is None:
            return None
        ledger = getattr(self, "place_work_items", None)
        if ledger is None:
            return None
        viewpoint_xy = self.work_item_physical_xy(message, x, y)
        prefetched_item_id = getattr(self, "prefetched_work_item_id", None)
        if prefetched_item_id is not None:
            item_id = ledger.dispatch_existing(
                prefetched_item_id,
                now,
                viewpoint_xy=viewpoint_xy,
                map_goal=(x, y),
                route_kind="frontier_endpoint",
            )
        else:
            item_id = ledger.dispatch(
                int(region["id"]),
                viewpoint_xy,
                now,
            )
        self.active_work_item_id = item_id
        if item_id is not None:
            self.active_work_item_attempt_id = ledger.active_attempt_id(item_id)
            self.active_work_item_place_id = int(region["id"])
            self.active_work_item_goal = (float(x), float(y))
            self.active_work_item_route_kind = "frontier_endpoint"
        return item_id

    def announce_active_work_item_dispatch(self):
        """Publish one lifecycle start after the command becomes executable.

        This is deliberately idempotent because a single mission endpoint may
        be represented by several connector commands while online SLAM
        refreshes.  The ledger itself was already reserved at selection time;
        this method only creates the auditable execution interval.
        """
        item_id = getattr(self, "active_work_item_id", None)
        if (
            item_id is None
            or getattr(self, "active_work_item_dispatch_announced", False)
        ):
            return False
        place_id = getattr(self, "active_work_item_place_id", None)
        goal = getattr(self, "active_work_item_goal", None)
        if place_id is None or goal is None:
            return False
        try:
            x, y = float(goal[0]), float(goal[1])
        except (IndexError, TypeError, ValueError):
            return False
        self.publish_work_item_dispatch(
            item_id,
            getattr(self, "active_work_item_attempt_id", None),
            {"id": int(place_id)},
            str(getattr(self, "active_work_item_route_kind", None) or "frontier_endpoint"),
            x,
            y,
        )
        self.active_work_item_dispatch_announced = True
        return True

    def publish_work_item_dispatch(
        self, item_id, attempt_id, region, route_kind, x, y,
    ):
        """Emit the one stable start event used by experiment reduction."""
        self.publish_status(
            "work_item_dispatched",
            work_item_id=int(item_id),
            attempt_id=(None if attempt_id is None else int(attempt_id)),
            place_id=(None if region is None else int(region["id"])),
            route_kind=str(route_kind),
            goal=[round(float(x), 3), round(float(y), 3)],
            work_item_state=WORK_UNRESOLVED,
            attempt_state=ATTEMPT_ACTIVE,
        )

    def settle_active_work_item(self, now, state, reason):
        """Finish an Attempt and resolve WorkItem only from observation proof."""
        item_id = getattr(self, "active_work_item_id", None)
        if item_id is None:
            return None
        # A map observation may resolve the endpoint before the next planner
        # tick publishes a command. Preserve a zero-duration, explicitly
        # paired lifecycle record rather than an unmatched terminal event.
        self.announce_active_work_item_dispatch()
        attempt_id = getattr(self, "active_work_item_attempt_id", None)
        item = None
        if state == WORK_RESOLVED:
            item = self.place_work_items.resolve(
                item_id, now, reason, attempt_id=attempt_id,
            )
            if item is not None:
                self.publish_status(
                    "work_item_settled",
                    work_item_id=int(item_id),
                    attempt_id=(None if attempt_id is None else int(attempt_id)),
                    place_id=int(item["place_id"]),
                    work_item_state=WORK_RESOLVED,
                    attempt_state=ATTEMPT_SUCCEEDED,
                    reason=str(reason),
                )
        elif state == WORK_DEFERRED:
            item, attempt = self.place_work_items.fail_attempt(
                item_id,
                now,
                reason,
                state=ATTEMPT_FAILED,
                attempt_id=attempt_id,
            )
            if item is not None and attempt is not None:
                self.publish_status(
                    "viewpoint_attempt_settled",
                    work_item_id=int(item_id),
                    attempt_id=int(attempt["id"]),
                    place_id=int(item["place_id"]),
                    work_item_state=WORK_UNRESOLVED,
                    attempt_state=str(attempt["state"]),
                    reason=str(reason),
                )
        else:
            raise ValueError("resolved WorkItem or deferred Attempt required")
        # Prevent a later watchdog/terminal callback for the same route from
        # producing a second terminal record for one observation lease.
        self.active_work_item_id = None
        self.active_work_item_attempt_id = None
        self.active_work_item_place_id = None
        self.active_work_item_goal = None
        self.active_work_item_route_kind = None
        self.active_work_item_dispatch_announced = False
        return item
