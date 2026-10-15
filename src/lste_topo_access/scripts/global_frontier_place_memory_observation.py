"""Observation and coverage evidence for persistent frontier-place memory."""

import math

from global_frontier_place_memory_physical import append_spaced_anchor
from global_frontier_place_states import (
    PLACE_DORMANT,
    PLACE_OPEN,
    PLACE_SUSPENDED,
)
from global_frontier_topology import copy_component_evidence


class FrontierRegionObservationMixin:
    """Update a region only after an actual controller-reaching observation."""

    def observe(
        self, x, y, information, robot_x, robot_y, now, component=None,
        region_id=None, observation_ready=False,
        observation_session_started=False, physical_robot_xy=None,
    ):
        """Update a place only after its route reached a real viewpoint.

        ``radius`` identifies related frontiers in one structural place. It is
        deliberately larger than endpoint acceptance, so callers must confirm
        that the controller reached an observation viewpoint before this can
        start a place dwell.
        """
        region, association = self._resolve(
            region_id, x, y, component, physical_xy=physical_robot_xy,
        )
        if region is None or region["state"] not in (PLACE_OPEN, PLACE_SUSPENDED):
            return None
        if component is not None:
            region["component"] = copy_component_evidence(component)
            region["last_association"] = association
        if not observation_ready:
            return region
        if math.hypot(float(robot_x) - region["x"], float(robot_y) - region["y"]) > self.radius:
            return region
        if region["entered_at"] is None:
            region["entered_at"] = float(now)
        if region.get("observation_started_at") is None:
            # This place-level no-gain epoch starts only at the first reached
            # viewpoint. A later endpoint action is not a new place visit.
            region["observation_started_at"] = float(now)
            region["last_gain"] = float(now)
            region["last_reason"] = "observation_session_started"
            region["observation_sessions"] = int(
                region.get("observation_sessions", 0)
            ) + 1
        viewpoint = float(robot_x), float(robot_y)
        minimum_anchor_spacing = max(0.25, self.radius / 4.0)
        append_spaced_anchor(
            region.setdefault("viewpoints", []), viewpoint, minimum_anchor_spacing,
        )
        append_spaced_anchor(
            region.setdefault("physical_viewpoints", []),
            physical_robot_xy,
            minimum_anchor_spacing,
        )
        information = float(information)
        # A lower unknown count means observation converted unseen cells to
        # map evidence. A higher count also records a newly visible branch,
        # but does not create a new room session.
        if information <= region["information"] - self.information_delta:
            region["information"] = information
            region["last_gain"] = float(now)
            region["last_reason"] = "unknown_reduced"
        elif information >= region["max_information"] + self.information_delta:
            region["information"] = information
            region["max_information"] = information
            region["last_gain"] = float(now)
            region["last_reason"] = "local_unknown_expanded"
        return region

    def stagnant(
        self, x, y, robot_x, robot_y, now, component=None, region_id=None,
    ):
        """Return the entered region after a real no-information dwell period."""
        region, _association = self._resolve(region_id, x, y, component)
        if (
            region is None
            or region["state"] not in (PLACE_OPEN, PLACE_SUSPENDED)
            or region.get("observation_started_at") is None
        ):
            return None
        if math.hypot(float(robot_x) - region["x"], float(robot_y) - region["y"]) > self.radius:
            return None
        if float(now) - region["last_gain"] < self.stagnation_timeout:
            return None
        return region

    def endpoint_observed(
        self, x, y, now, component=None, region_id=None,
        retain_for_target=False, physical_xy=None,
    ):
        """Add coverage evidence after a reached endpoint was observed.

        A matching controller terminal is an authoritative arrival at one
        viewpoint, not completion of a room. It must establish the first
        observation session itself: waiting for a later planner timer leaves
        a crossed destination room without a durable identity and permits a
        future physical re-entry.
        """
        region, _association = self._resolve(region_id, x, y, component)
        if region is not None:
            region["failures"] = 0
            region["last_selected"] = float(now)
            if region["state"] == PLACE_DORMANT:
                return region
            if region["entered_at"] is None:
                region["entered_at"] = float(now)
                region["physical_entry_count"] = int(
                    region.get("physical_entry_count", 0)
                ) + 1
            if region.get("observation_started_at") is None:
                region["observation_started_at"] = float(now)
                region["last_gain"] = float(now)
                region["observation_sessions"] = int(
                    region.get("observation_sessions", 0)
                ) + 1
            viewpoint = float(x), float(y)
            minimum_anchor_spacing = max(0.25, self.radius / 4.0)
            append_spaced_anchor(
                region.setdefault("viewpoints", []),
                viewpoint,
                minimum_anchor_spacing,
            )
            append_spaced_anchor(
                region.setdefault("physical_viewpoints", []),
                physical_xy,
                minimum_anchor_spacing,
            )
            region["endpoint_observations"] += 1
            if retain_for_target:
                region["last_reason"] = "target_claim_requires_more_observation"
            else:
                region["last_reason"] = "endpoint_coverage_recorded"
        return region
