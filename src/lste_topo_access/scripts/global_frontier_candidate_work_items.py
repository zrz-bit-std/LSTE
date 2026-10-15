"""Rehydrate Place-owned observation WorkItems after a SLAM map change.

The frontier grid is a short-lived projection.  A durable WorkItem keeps its
physical boundary anchor and unknown-side normal, which is enough to compile a
new safe observation viewpoint when the original frontier arc disappears.
"""

import math

import numpy as np

from global_frontier_models import FrontierCandidateRoute


class GlobalFrontierCandidateWorkItemMixin:
    """Build current-map candidates for unresolved local WorkItems."""

    @staticmethod
    def _finite_xy(value):
        if value is None:
            return None
        try:
            point = float(value[0]), float(value[1])
        except (IndexError, TypeError, ValueError):
            return None
        return point if all(math.isfinite(item) for item in point) else None

    @staticmethod
    def _finite_normal(value):
        point = GlobalFrontierCandidateWorkItemMixin._finite_xy(value)
        if point is None:
            return None
        length = math.hypot(point[0], point[1])
        if not math.isfinite(length) or length <= 1e-9:
            return None
        return point[0] / length, point[1] / length

    @staticmethod
    def _viewpoint_ladder(anchor, normal, depth):
        """Return one normal and two lateral views of the same boundary."""
        tangent = -normal[1], normal[0]
        source = (
            anchor[0] - depth * normal[0],
            anchor[1] - depth * normal[1],
        )
        return (
            ("normal", source),
            (
                "lateral_left",
                (source[0] + depth * tangent[0], source[1] + depth * tangent[1]),
            ),
            (
                "lateral_right",
                (source[0] - depth * tangent[0], source[1] - depth * tangent[1]),
            ),
        )

    def _work_item_map_anchor(self, request, item):
        """Project a durable physical anchor into the current map frame."""
        physical_anchor = self._finite_xy(item.get("anchor_xy"))
        map_anchor = self._finite_xy((item.get("x"), item.get("y")))
        anchor = physical_anchor or map_anchor
        if anchor is None:
            return None
        header = getattr(request.message, "header", None)
        map_frame = getattr(header, "frame_id", "map") or "map"
        projector_factory = getattr(self, "planar_xy_projector", None)
        if projector_factory is not None:
            projector = projector_factory(map_frame, "odom")
            if projector is not None:
                projected = self._finite_xy(projector(anchor[0], anchor[1]))
                if projected is not None:
                    return projected
            # A durable ``anchor_xy`` is physical-frame data. Do not silently
            # reinterpret it as map coordinates when the inverse TF is missing.
            return map_anchor
        # Geometry-only fixtures may intentionally use one frame for both map
        # and physical coordinates. Production callers keep the physical frame
        # contract by providing the inverse TF projector above.
        return anchor if request.map_to_physical_xy is None else None

    def pending_local_work_item_candidates(
        self, request, context, frontier, approach_cells,
    ):
        """Return one safe current-map viewpoint for each unresolved item.

        Doorway-owned WorkItems are excluded because their PortalProbe ledger
        is the sole owner of source/destination evidence.  A candidate is still
        validated by the current BFS and costmap; durable geometry only supplies
        the identity and the desired side of the observation.
        """
        ledger = getattr(self, "place_work_items", None)
        place_id = getattr(context, "source_place_id", None)
        del frontier, approach_cells
        if ledger is None or place_id is None:
            return []
        snapshot = getattr(ledger, "snapshot", None)
        records = () if snapshot is None else snapshot()
        work_item_filter = getattr(self, "graph_route_work_item_filter", None)
        selected_work_item_id = getattr(self, "graph_route_work_item_id", None)
        if work_item_filter == "none":
            return []
        if work_item_filter == "selected":
            try:
                selected_work_item_id = int(selected_work_item_id)
            except (TypeError, ValueError):
                return []
            if selected_work_item_id <= 0:
                return []
        probe_ledger = getattr(self, "portal_probe_ledger", None)
        probe_for_work = (
            None if probe_ledger is None
            else getattr(probe_ledger, "probe_for_work_item", None)
        )
        message = request.message
        steps = request.steps
        if steps is None or not hasattr(steps, "shape"):
            return []
        reassociate = getattr(self, "nearest_reachable_cell", None)
        if not callable(reassociate):
            return []
        resolution = max(1e-6, float(message.info.resolution))
        map_epoch = getattr(
            getattr(request, "components", None), "epoch", None
        )
        depth = max(
            float(getattr(self, "frontier_approach_distance", resolution)),
            float(getattr(self, "completed_radius", resolution)) * 0.5,
        )
        report = {
            "place_id": int(place_id),
            "map_epoch": getattr(
                getattr(request, "components", None), "epoch", None,
            ),
            "candidate_count": 0,
            "rejection_reasons": {},
            "items_seen": 0,
            "item_audit": [],
        }
        candidates = []

        def reject(reason):
            report["rejection_reasons"][reason] = (
                report["rejection_reasons"].get(reason, 0) + 1
            )

        for item in records:
            if getattr(self, "planning_should_preempt", lambda: False)():
                return []
            try:
                item_id = int(item.get("id"))
                item_place = int(item.get("place_id"))
            except (TypeError, ValueError):
                continue
            if item_place != int(place_id):
                continue
            if (
                work_item_filter == "selected"
                and item_id != selected_work_item_id
            ):
                continue
            if str(item.get("state", "")).strip().lower() != "unresolved":
                continue
            report["items_seen"] += 1
            audit = {"work_item_id": item_id}
            if item.get("active_attempt_id") is not None:
                audit["reason"] = "attempt_active"
                report["item_audit"].append(audit)
                reject("attempt_active")
                continue
            if callable(probe_for_work) and probe_for_work(item_id) is not None:
                audit["reason"] = "owned_by_portal_probe"
                report["item_audit"].append(audit)
                reject("owned_by_portal_probe")
                continue
            anchor = self._work_item_map_anchor(request, item)
            normal = self._finite_normal(item.get("normal_xy"))
            normal_source = "durable_unknown_side"
            if anchor is None:
                audit["reason"] = "physical_anchor_projection_unavailable"
                report["item_audit"].append(audit)
                reject("physical_anchor_projection_unavailable")
                continue
            if normal is None:
                # A one-cell or symmetric frontier arc can have a valid
                # physical anchor but no unique unknown-side normal.  That is
                # insufficient for a durable *identity* update, but it is
                # still enough for this snapshot to choose the known-side
                # approach from the robot toward the anchor.  Keep this
                # fallback map-local; never persist it as the item's normal.
                robot_anchor = self._finite_xy(
                    getattr(request, "robot_map", None)
                )
                if robot_anchor is None:
                    robot_anchor = self._finite_xy(
                        getattr(request, "route_anchor_xy", None)
                    )
                if robot_anchor is not None:
                    direction = (
                        anchor[0] - robot_anchor[0],
                        anchor[1] - robot_anchor[1],
                    )
                    normal = self._finite_normal(direction)
                    if normal is not None:
                        normal_source = "current_robot_approach"
            if normal is None:
                audit["anchor_map"] = [round(float(value), 3) for value in anchor]
                audit["reason"] = "unknown_side_normal_missing"
                report["item_audit"].append(audit)
                reject("unknown_side_normal_missing")
                continue
            audit["anchor_map"] = [round(float(value), 3) for value in anchor]
            audit["normal"] = [round(float(value), 3) for value in normal]
            audit["normal_source"] = normal_source
            selected = None
            for strategy, desired in self._viewpoint_ladder(anchor, normal, depth):
                view_audit = dict(audit)
                view_audit["strategy"] = strategy
                view_audit["desired_viewpoint"] = [
                    round(float(desired[0]), 3), round(float(desired[1]), 3),
                ]
                cell = reassociate(message, steps, desired[0], desired[1])
                if cell is None or steps[cell] < 0:
                    view_audit["reason"] = "no_reachable_observation_viewpoint"
                    report["item_audit"].append(view_audit)
                    reject("no_reachable_observation_viewpoint")
                    continue
                row, col = cell
                x, y = self.cell_xy(message, row, col)
                costmap_distance = self.candidate_costmap_distance(
                    request.validation, x, y,
                )
                if request.validation is not None and costmap_distance is None:
                    view_audit["cell"] = [int(row), int(col)]
                    view_audit["reason"] = "costmap_route_unreachable"
                    report["item_audit"].append(view_audit)
                    reject("costmap_route_unreachable")
                    continue
                physical = (
                    (x, y)
                    if request.map_to_physical_xy is None
                    else request.map_to_physical_xy(x, y)
                )
                if not ledger.viewpoint_is_available(
                    item_id, physical, map_epoch=map_epoch,
                ):
                    view_audit["cell"] = [int(row), int(col)]
                    route_rejection = getattr(
                        ledger, "route_rejection_for_viewpoint", None
                    )
                    view_audit["reason"] = (
                        "route_rejected_in_map_epoch"
                        if callable(route_rejection)
                        and route_rejection(
                            item_id, physical, map_epoch=map_epoch,
                        ) is not None
                        else "viewpoint_already_attempted"
                    )
                    report["item_audit"].append(view_audit)
                    reject(view_audit["reason"])
                    continue
                path_distance = (
                    float(costmap_distance)
                    if costmap_distance is not None
                    else float(steps[row, col]) * resolution
                )
                retry = bool(
                    ledger.has_failed_viewpoint(item_id, map_epoch=map_epoch)
                )
                selected = (
                    row, col, x, y, costmap_distance, path_distance, physical,
                    strategy, retry,
                )
                break
            if selected is None:
                continue
            row, col, x, y, costmap_distance, path_distance, physical, strategy, retry = selected
            candidate = FrontierCandidateRoute(
                row=int(row),
                col=int(col),
                action_tier="local",
                place_hops=0,
                x=float(x),
                y=float(y),
                path_distance=path_distance,
                score_path_distance=path_distance,
                work_item_id=item_id,
                work_item_match="rehydrated_work_item",
                work_item_support_cells=int(item.get("support_cell_count", 0)),
                work_item_normal_xy=normal,
                viewpoint_retry=retry,
                graph_action="retry_viewpoint" if retry else "observe_local_work",
                graph_action_reason="durable_work_item_rehydrated_%s" % strategy,
            )
            candidates.append((int(row), int(col), candidate))
            report["candidate_count"] += 1
            audit["strategy"] = strategy
            audit["cell"] = [int(row), int(col)]
            audit["viewpoint_map"] = [round(float(x), 3), round(float(y), 3)]
            audit["reason"] = "executable"
            report["item_audit"].append(audit)

        report["rejection_reasons"] = dict(sorted(report["rejection_reasons"].items()))
        if report != getattr(self, "last_local_work_item_rehydration_report", None):
            self.last_local_work_item_rehydration_report = report
            publish = getattr(self, "publish_status", None)
            if callable(publish):
                publish("local_work_item_rehydration", **report)
        return candidates


__all__ = ["GlobalFrontierCandidateWorkItemMixin"]
