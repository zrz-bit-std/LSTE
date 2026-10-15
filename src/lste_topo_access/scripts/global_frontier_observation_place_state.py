"""Persistent place and direct-target claim state derived from one map snapshot."""

import math

import numpy as np

from global_frontier_topology import (
    copy_component_evidence,
    same_topology_component,
)


class GlobalFrontierObservationPlaceStateMixin:

    def covered_place_observation_footprint(
        self, message, known_free, components, route_anchor_xy, source_component,
    ):
        """Rebuild a route barrier for every observed place except the source.

        Component labels are transient SLAM products.  By contrast, a reached
        map-frame viewpoint is a durable fact.  Casting the current known map
        from those viewpoints creates a compact place envelope that survives a
        later split/merge of high-clearance cells. A room becomes unavailable
        to later exploration immediately after its first completed viewpoint;
        waiting until a delayed departure transition marked it dormant left a
        window in which the selector could re-enter it.

        The sole exception is the physical source place. Prefer an exact
        current-map component match to identify it. When SLAM has not exposed
        a core at the robot pose yet, fall back to proximity to a reached
        viewpoint. This exception lets the robot finish local coverage and
        leave its current room without granting access to earlier places.
        """
        footprint = np.zeros_like(known_free, dtype=bool)
        if self.scan_range_max is None or self.scan_range_max <= 0.0:
            return footprint, 0
        source_xy = route_anchor_xy
        source_release_radius = max(
            self.endpoint_terminal_wait_radius, self.waypoint_release_radius
        )
        anchors = []
        for place_id, place_anchors in self.region_memory.covered_viewpoints():
            region = self.region_memory.by_id(place_id)
            place_component = None if region is None else region.get("component")
            source_matches_place = same_topology_component(
                source_component, place_component
            )
            source_is_near_anchor = source_xy is not None and any(
                math.hypot(float(source_xy[0]) - x, float(source_xy[1]) - y)
                <= source_release_radius
                for x, y in place_anchors
            )
            if source_matches_place or source_is_near_anchor:
                continue
            anchors.extend(place_anchors)
        anchored_places = self.observation_anchor_cells(
            message, known_free, anchors,
        )
        if not anchored_places:
            return footprint, 0
        footprint = self.observation_footprint_from_anchors(
            message, known_free, anchored_places, components=components,
        )
        return footprint, len(anchored_places)

    def closed_place_observation_footprint(
        self, message, known_free, components, route_anchor_xy,
    ):
        """Compatibility alias for callers using the former lifecycle name."""
        return self.covered_place_observation_footprint(
            message,
            known_free,
            components,
            route_anchor_xy,
            source_component=None,
        )

    def refresh_region_components(self, message, components, known_free):
        """Align persistent observation regions with the current map epoch."""
        map_frame = message.header.frame_id or "map"
        projector_factory = getattr(self, "planar_xy_projector", None)
        map_from_odom = (
            None
            if projector_factory is None
            else projector_factory(map_frame, "odom")
        )
        return self.region_memory.refresh_components(
            lambda x, y: self.component_at_map_position(
                message, components, known_free, x, y
            ),
            map_from_physical_xy=map_from_odom,
        )

    def rebind_completed_local_egress_place(
        self, message, components, known_free, robot_map, now,
    ):
        """Bind post-egress topology to the place that initiated recovery."""
        lease = self.local_egress_place_lease
        if not lease.rebind_pending:
            return True
        component = self.component_at_map_position(
            message, components, known_free, robot_map[0], robot_map[1],
        )
        if component is None:
            return False
        region = self.region_memory.rebind_after_local_egress(
            lease.source_region_id,
            robot_map[0],
            robot_map[1],
            now,
            component,
        )
        if region is None:
            self.publish_status(
                "local_egress_place_rebind_discarded",
                source_region_id=lease.source_region_id,
                reason="source_place_not_open",
            )
            lease.clear()
            return True
        self.publish_status(
            "local_egress_place_rebound",
            source_region_id=int(region["id"]),
            component={
                "epoch": int(component["epoch"]),
                "label": int(component["label"]),
            },
        )
        lease.clear()
        return True

    def clear_target_region_claim(self, reason):
        """Release the target-room constraint at one explicit lifecycle edge."""
        if not self.target_region_claim_active:
            return
        component = self.target_region_claim_component
        track_id = self.target_region_claim_track_id
        self.target_region_claim_active = False
        self.target_region_claim_anchor_map = None
        self.target_region_claim_component = None
        self.target_region_claim_track_id = ""
        self.target_region_claim_request_id = 0
        self.target_region_claim_waiting_reported = False
        self.target_region_claim_bound_reported = False
        self.target_reinspection_pending = False
        self.publish_status(
            "target_region_claim_released",
            reason=str(reason),
            target_track_id=track_id or None,
            component=(
                None if component is None else {
                    "epoch": int(component["epoch"]),
                    "label": int(component["label"]),
                }
            ),
        )

    def refresh_target_region_claim(
        self, message, components, known_free, robot_map,
    ):
        """Bind a direct-target claim to this map epoch's room component.

        The anchor is fixed when the claim starts, rather than following the
        robot.  Otherwise a failed target route could move the anchor through
        a doorway and silently turn "search this room" into "search whichever
        room the controller happens to enter next".
        """
        if not self.target_region_claim_active:
            return True
        if self.target_region_claim_anchor_map is None:
            self.target_region_claim_anchor_map = (
                float(robot_map[0]), float(robot_map[1]),
            )
        anchor_x, anchor_y = self.target_region_claim_anchor_map
        component = self.component_at_map_position(
            message, components, known_free, anchor_x, anchor_y,
        )
        if component is None:
            if not self.target_region_claim_waiting_reported:
                self.target_region_claim_waiting_reported = True
                self.publish_status(
                    "target_region_claim_waiting_topology",
                    request_id=int(self.target_region_claim_request_id),
                    anchor=[round(anchor_x, 3), round(anchor_y, 3)],
                )
            return False
        previous = self.target_region_claim_component
        self.target_region_claim_component = copy_component_evidence(component)
        # Component labels belong to the current SLAM snapshot.  A changed
        # label is therefore a re-projection of the same physical claim, not
        # a new semantic event.  Emit one acquisition event per claim and keep
        # the transient component only as the current geometric mask.
        if previous is None and not self.target_region_claim_bound_reported:
            self.target_region_claim_bound_reported = True
            self.publish_status(
                "target_region_claim_bound",
                request_id=int(self.target_region_claim_request_id),
                anchor=[round(anchor_x, 3), round(anchor_y, 3)],
                component={
                    "epoch": int(component["epoch"]),
                    "label": int(component["label"]),
                    "cells": int(component["cells"]),
                },
            )
        self.target_region_claim_waiting_reported = False
        return True
