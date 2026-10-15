#!/usr/bin/env python3
"""Fast, non-Gazebo checks for the deterministic office benchmark contract."""

from copy import deepcopy
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[3]
BENCHMARK = ROOT / "scripts/tests/office_building"
if str(BENCHMARK) not in sys.path:
    sys.path.insert(0, str(BENCHMARK))

from validate_office_building import validate


MANIFEST = ROOT / "worlds/benchmark/office_building_v1_manifest.yaml"
RUNNER = ROOT / "scripts/tests/office_building/run_office_building.sh"
EXPECTED_HASHES = {
    "level_1": "5c55979e53b2d029939f90d3e1c1fd3324cc02aca6c354e0f5b36d6e2b56da1b",
    "level_2": "6b9e2d314bf21ba568012142590a76252768a6af3d3dcbe0089cdc4dc4089d41",
    "level_3": "74b43250220fdc8107c52d646abfa608fe2c8b24b9c2834b21bf93967e886ce2",
    "level_4": "8b022c48d01a20a9e1551079a145e8395673b41fe15927cbadc7e1e8917c1c07",
}


class OfficeBenchmarkContractTest(unittest.TestCase):
    def test_all_levels_match_their_pinned_world_hash(self):
        for level, expected_hash in EXPECTED_HASHES.items():
            with self.subTest(level=level):
                result, details = validate(MANIFEST, level, runtime=False)
                self.assertEqual(result.errors, [])
                self.assertEqual(details["world_sha256"], expected_hash)

    def test_level_hash_catches_a_modified_world_contract(self):
        manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
        manifest = deepcopy(manifest)
        manifest["levels"]["level_4"]["world_sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.yaml"
            path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
            result, _details = validate(path, "level_4", runtime=False)
        self.assertTrue(
            any("world_sha256 mismatch" in error for error in result.errors),
            result.errors,
        )

    def test_target_entry_is_a_near_target_diagnostic_profile(self):
        manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
        profile = manifest["robot_profiles"]["target_entry"]
        target_xy = manifest["targets"]["primary"]["pose_xy"]
        target_room_id = manifest["targets"]["primary"]["room"]
        target_room = next(
            room for room in manifest["rooms"] if room["id"] == target_room_id
        )
        corridor_door = next(
            door for door in manifest["doors"] if door["id"] == "target_room_corridor"
        )
        x, y, _yaw = [float(value) for value in profile["pose"]]
        room_min_x, room_min_y, room_max_x, room_max_y = [
            float(value) for value in target_room["bounds_m"]
        ]
        door_x, door_y = [float(value) for value in corridor_door["center_m"]]

        self.assertTrue(profile["diagnostic_only"])
        self.assertTrue(profile["bypasses_exploration"])
        self.assertFalse(profile["benchmark_evidence"])
        self.assertIn("target_approach_isolation", profile["use_for"])
        self.assertTrue(room_min_x <= x <= room_max_x)
        self.assertTrue(room_min_y <= y <= room_max_y)
        self.assertLess(math.hypot(x - door_x, y - door_y), 3.0)
        self.assertNotEqual((x, y), tuple(float(value) for value in target_xy))

    def test_target_entry_validation_rejects_target_truth_pose(self):
        manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
        manifest = deepcopy(manifest)
        manifest["robot_profiles"]["target_entry"]["pose"] = [
            *manifest["targets"]["primary"]["pose_xy"],
            0.0,
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.yaml"
            path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
            result, _details = validate(path, "level_4", runtime=False)
        self.assertTrue(
            any("target_entry must not equal primary target truth" in error for error in result.errors),
            result.errors,
        )

    def test_target_entry_validation_requires_the_named_profile(self):
        manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
        manifest = deepcopy(manifest)
        del manifest["robot_profiles"]["target_entry"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.yaml"
            path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
            result, _details = validate(path, "level_4", runtime=False)
        self.assertIn("manifest must declare robot profile target_entry", result.errors)

    def test_launcher_profile_command_uses_office_building_profile(self):
        environment = os.environ.copy()
        environment["LSTE_WS"] = str(ROOT)
        environment["OFFICE_BUILDING_PROFILE"] = "target_entry"
        completed = subprocess.run(
            ["bash", str(RUNNER), "profile"],
            cwd=ROOT,
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(
            completed.stdout.splitlines(),
            ["19.8", "13.8", "1.57079632679"],
        )


if __name__ == "__main__":
    unittest.main()
