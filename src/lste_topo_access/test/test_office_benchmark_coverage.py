#!/usr/bin/env python3
"""Regression tests for the fixed office coverage denominator."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from office_benchmark_coverage import evaluate_map_known_fraction, load_manifest_truth


class OfficeBenchmarkCoverageTest(unittest.TestCase):
    def test_enabled_level_selects_only_its_rooms_and_corridor(self):
        manifest = Path(__file__).resolve().parents[3] / "worlds/benchmark/office_building_v1_manifest.yaml"
        truth = load_manifest_truth(manifest, "level_1")
        self.assertEqual(set(truth["regions"]), {"lobby", "main_corridor", "target_room"})
        self.assertGreater(len(truth["points"]), 100)

    def test_known_fraction_uses_fixed_denominator_when_grid_is_cropped(self):
        truth = {
            "definition": "test",
            "level": "unit",
            "sample_resolution_m": 1.0,
            "boundary_clearance_m": 0.0,
            "regions": ("room",),
            "points": (
                type("Point", (), {"x": 0.5, "y": 0.5, "region": "room"})(),
                type("Point", (), {"x": 1.5, "y": 0.5, "region": "room"})(),
                type("Point", (), {"x": 2.5, "y": 0.5, "region": "room"})(),
            ),
        }
        result = evaluate_map_known_fraction(
            truth,
            occupancy_data=[0, -1],
            width=2,
            height=1,
            resolution=1.0,
            map_origin=(0.0, 0.0, 0.0),
            map_from_odom=(0.0, 0.0, 0.0),
        )
        self.assertEqual(result["truth_sample_count"], 3)
        self.assertEqual(result["known_sample_count"], 1)
        self.assertEqual(result["out_of_bounds_sample_count"], 1)
        self.assertAlmostEqual(result["building_truth_fraction"], 1.0 / 3.0, places=6)

    def test_invalid_level_is_rejected(self):
        manifest = Path(__file__).resolve().parents[3] / "worlds/benchmark/office_building_v1_manifest.yaml"
        with self.assertRaisesRegex(ValueError, "unknown benchmark level"):
            load_manifest_truth(manifest, "not_a_level")


if __name__ == "__main__":
    unittest.main()
