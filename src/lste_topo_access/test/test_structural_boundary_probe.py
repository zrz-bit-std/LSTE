"""Regression tests for structural boundary evidence.

The key case is intentionally fully known on both sides of the doorway.  A
frontier-only detector must return nothing there, while the structural
evidence compiler must retain one directional source-side probe.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_grid import bfs  # noqa: E402
from global_frontier_portal_probes import (  # noqa: E402
    PortalObservationProbe,
    portal_observation_probe,
)
from global_frontier_candidate_portals import (  # noqa: E402
    GlobalFrontierCandidatePortalMixin,
)
from global_frontier_structural_boundaries import (  # noqa: E402
    structural_boundary_candidates,
)


class _ProbeLedger:
    def __init__(self):
        self.next_id = 1

    def viewpoint_available(self, _probe_id, _viewpoint):
        return True


class _CandidateHarness(GlobalFrontierCandidatePortalMixin):
    frontier_approach_distance = 2.0
    completed_radius = 1.25
    region_topology_clearance = 0.50
    place_furniture_max_span_m = 2.50

    def __init__(self):
        self.portal_probe_ledger = _ProbeLedger()
        self.last_structural_boundary_candidates = 0
        self.last_structural_boundary_rejections = 0
        self.last_structural_boundary_source_place_rejections = 0

    @staticmethod
    def cell_xy(message, row, col):
        origin = message.info.origin.position
        return (
            origin.x + (col + 0.5) * message.info.resolution,
            origin.y + (row + 0.5) * message.info.resolution,
        )

    @staticmethod
    def nearest_safe_approach(
        steps, row, col, max_cells, preferred_steps=None,
        preferred_mask=None, selection_tier="strict_clearance",
    ):
        del preferred_mask, selection_tier
        candidates = np.argwhere(
            (steps >= 0) & (preferred_steps is None or preferred_steps >= 0)
        )
        if candidates.size == 0:
            return None
        distance = (candidates[:, 0] - row) ** 2 + (candidates[:, 1] - col) ** 2
        candidates = candidates[distance <= int(max_cells) ** 2]
        if candidates.size == 0:
            return None
        distance = (candidates[:, 0] - row) ** 2 + (candidates[:, 1] - col) ** 2
        selected = candidates[int(np.argmin(distance))]
        return int(selected[0]), int(selected[1])

    @staticmethod
    def candidate_costmap_distance(_validation, _x, _y):
        return None

    def register_portal_probe(self, _request, probe):
        probe_id = self.portal_probe_ledger.next_id
        self.portal_probe_ledger.next_id += 1
        return PortalObservationProbe(
            probe.opening_cell,
            probe.normal,
            probe_id,
            probe.phase,
            probe.observation_source,
        )


class _CrossPlaceProbeHarness(_CandidateHarness):
    """Force the compiled source viewpoint onto the far-side label."""

    @staticmethod
    def source_portal_probe_viewpoint(_request, _probe, _gate_xy):
        return 20, 32


class StructuralBoundaryProbeTest(unittest.TestCase):
    def doorway_snapshot(self, *, closed=False):
        known_free = np.zeros((41, 61), dtype=bool)
        known_free[5:36, 5:56] = True
        structural = np.zeros_like(known_free)
        structural[5:36, 30] = True
        if not closed:
            structural[18:23, 30] = False
        known_free[structural] = False
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[8:33, 8:29] = 1
        if not closed:
            labels[8:33, 32:53] = 2
        steps = bfs(known_free, (20, 15))
        return known_free, structural, labels, steps

    def test_known_far_side_still_yields_one_outward_boundary(self):
        known_free, structural, labels, steps = self.doorway_snapshot()

        # There is no unknown cell in this snapshot, so the historical
        # frontier-bound probe has no evidence to consume.
        self.assertIsNone(
            portal_observation_probe(
                np.zeros_like(known_free),
                structural,
                (20, 30),
                support_radius=5,
                minimum_wall_span_cells=5,
            )
        )

        candidates = structural_boundary_candidates(
            known_free,
            structural,
            steps,
            labels=labels,
            source_label=1,
            support_radius=5,
            minimum_wall_span_cells=5,
        )

        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].normal, (0, 1))
        self.assertEqual(candidates[0].source_cell[1], 29)
        self.assertEqual(candidates[0].destination_cell[1], 31)

    def test_closed_structural_wall_has_no_boundary(self):
        known_free, structural, labels, steps = self.doorway_snapshot(closed=True)

        self.assertEqual(
            structural_boundary_candidates(
                known_free,
                structural,
                steps,
                labels=labels,
                source_label=1,
                support_radius=5,
                minimum_wall_span_cells=5,
            ),
            (),
        )

    def test_boundary_compiler_never_grants_crossing_permission(self):
        known_free, structural, labels, steps = self.doorway_snapshot()
        candidates = structural_boundary_candidates(
            known_free,
            structural,
            steps,
            labels=labels,
            source_label=1,
            support_radius=5,
            minimum_wall_span_cells=5,
        )

        # The compiler exposes only an opening and direction.  It has no
        # destination Place or Portal ID, so a later probe/admission phase is
        # still required before a physical crossing can be planned.
        self.assertIsNone(getattr(candidates[0], "destination_place_id", None))
        self.assertEqual(candidates[0].source_cell[1] + 2, candidates[0].destination_cell[1])

    def test_candidate_adapter_keeps_boundary_as_a_probe(self):
        known_free, structural, labels, steps = self.doorway_snapshot()
        message = SimpleNamespace(
            info=SimpleNamespace(
                resolution=0.10,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=-3.05, y=-2.05)
                ),
            )
        )
        request = SimpleNamespace(
            unknown=~known_free,
            occupied=~known_free & ~structural,
            components=SimpleNamespace(
                labels=labels,
                structural_occupied=structural,
            ),
            steps=steps,
            preferred_steps=steps,
            preferred_mask=known_free,
            selection_tier="strict_clearance",
            message=message,
            validation=None,
            map_to_physical_xy=None,
        )
        context = SimpleNamespace(
            source_place_observed=True,
            source_place_id=1,
            source_label=1,
        )

        candidates = _CandidateHarness().structural_boundary_probe_candidates(
            request, context, approach_cells=12,
        )

        self.assertEqual(len(candidates), 1)
        candidate = candidates[0][2]
        self.assertEqual(candidate.graph_action, "probe_portal")
        self.assertEqual(
            candidate.graph_action_reason,
            "structural_boundary_evidence",
        )
        self.assertIsNone(candidate.work_item_id)
        self.assertEqual(
            candidate.portal_observation_probe.observation_source,
            "structural_boundary",
        )

    def test_candidate_adapter_rejects_source_viewpoint_in_another_place(self):
        known_free, structural, labels, steps = self.doorway_snapshot()
        message = SimpleNamespace(
            info=SimpleNamespace(
                resolution=0.10,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=-3.05, y=-2.05)
                ),
            )
        )
        request = SimpleNamespace(
            unknown=~known_free,
            occupied=~known_free & ~structural,
            components=SimpleNamespace(
                labels=labels,
                structural_occupied=structural,
            ),
            steps=steps,
            preferred_steps=steps,
            preferred_mask=known_free,
            selection_tier="strict_clearance",
            message=message,
            validation=None,
            map_to_physical_xy=None,
        )
        context = SimpleNamespace(
            source_place_observed=True,
            source_place_id=1,
            source_label=1,
        )

        harness = _CrossPlaceProbeHarness()
        candidates = harness.structural_boundary_probe_candidates(
            request, context, approach_cells=12,
        )

        self.assertEqual(candidates, [])
        self.assertEqual(harness.last_structural_boundary_source_place_rejections, 1)

    def test_boundary_outside_sensor_horizon_is_not_active_probe(self):
        """Global map remnants cannot outrank a doorway in current lidar range."""
        known_free, structural, labels, steps = self.doorway_snapshot()
        message = SimpleNamespace(
            info=SimpleNamespace(
                resolution=0.10,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=-3.05, y=-2.05)
                ),
            )
        )
        request = SimpleNamespace(
            unknown=~known_free,
            occupied=~known_free & ~structural,
            components=SimpleNamespace(
                labels=labels,
                structural_occupied=structural,
            ),
            steps=steps,
            preferred_steps=steps,
            preferred_mask=known_free,
            selection_tier="strict_clearance",
            message=message,
            validation=None,
            map_to_physical_xy=None,
        )
        harness = _CandidateHarness()
        harness.scan_observation_horizon = 1.0

        candidates = harness.structural_boundary_probe_candidates(
            request, SimpleNamespace(
                source_place_observed=True,
                source_place_id=1,
                source_label=1,
            ), approach_cells=12,
        )

        self.assertEqual(candidates, [])
        self.assertEqual(harness.last_structural_boundary_rejections, 1)

    def test_unknown_destination_still_yields_a_source_probe(self):
        known_free, structural, labels, steps = self.doorway_snapshot()
        unknown = np.zeros_like(known_free)
        known_free[18:23, 31] = False
        labels[8:33, 32:53] = 0
        unknown[18:23, 31] = True
        steps[20, 30] = -1

        candidates = structural_boundary_candidates(
            known_free,
            structural,
            steps,
            unknown=unknown,
            labels=labels,
            source_label=1,
            support_radius=5,
            minimum_wall_span_cells=5,
        )

        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].normal, (0, 1))
        self.assertEqual(candidates[0].destination_cell[1], 31)
        self.assertTrue(unknown[candidates[0].destination_cell])

    def test_wide_architectural_opening_uses_window_for_wall_evidence(self):
        """Door width may exceed the robot-clearance throat radius."""
        known_free = np.zeros((41, 81), dtype=bool)
        known_free[5:36, 5:76] = True
        structural = np.zeros_like(known_free)
        # Leave a two-metre opening in the horizontal wall at row 20.
        structural[20, 5:76] = True
        structural[20, 21:41] = False
        known_free[structural] = False
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[8:19, 8:73] = 1
        labels[21:33, 42:73] = 2
        steps = bfs(known_free, (15, 15))

        candidates = structural_boundary_candidates(
            known_free,
            structural,
            steps,
            labels=labels,
            source_label=1,
            support_radius=5,
            minimum_wall_span_cells=5,
            wall_search_radius=30,
        )

        self.assertTrue(candidates)
        self.assertTrue(all(candidate.normal == (1, 0) for candidate in candidates))

    def test_corridor_wall_thickness_is_not_a_portal(self):
        """Parallel corridor walls must not be mistaken for an opening.

        With only the rasterized wall thickness as support, every aisle cell
        is bounded by occupied cells in the tangent direction and the old
        detector can manufacture a side-facing doorway.  Requiring the
        clearance-scale architectural run removes that ambiguity while
        preserving real wall openings.
        """
        known_free = np.ones((40, 60), dtype=bool)
        structural = np.zeros_like(known_free)
        structural[10:13, 2:58] = True
        structural[27:30, 2:58] = True
        known_free[structural] = False
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[13:27, 2:58] = 1
        steps = bfs(known_free, (20, 8))

        loose = structural_boundary_candidates(
            known_free,
            structural,
            steps,
            labels=labels,
            source_label=1,
            support_radius=14,
            minimum_wall_span_cells=2,
            unknown=np.zeros_like(known_free),
            wall_search_radius=20,
        )
        strict = structural_boundary_candidates(
            known_free,
            structural,
            steps,
            labels=labels,
            source_label=1,
            support_radius=14,
            minimum_wall_span_cells=5,
            unknown=np.zeros_like(known_free),
            wall_search_radius=20,
        )

        self.assertTrue(loose)
        self.assertEqual(strict, ())


if __name__ == "__main__":
    unittest.main()
