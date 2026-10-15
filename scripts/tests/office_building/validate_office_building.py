#!/usr/bin/env python3
"""Validate the office-building benchmark contract before launching it."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except ImportError as exc:  # pragma: no cover - depends on ROS host setup
    raise SystemExit("PyYAML is required: %s" % exc)


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MANIFEST = ROOT / "worlds/benchmark/office_building_v1_manifest.yaml"


class Validation:
    def __init__(self) -> None:
        self.errors = []
        self.warnings = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


def resolve_workspace_path(value: str, root: Path) -> Path:
    path = Path(os.path.expandvars(os.path.expanduser(str(value))))
    return path if path.is_absolute() else root / path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def available_model_dirs() -> list[Path]:
    paths = []
    for value in os.environ.get("GAZEBO_MODEL_PATH", "").split(":"):
        if value:
            paths.append(Path(value))
    paths.extend(
        [
            Path.home() / ".gazebo/models",
            Path("/usr/share/gazebo-11/models"),
        ]
    )
    return paths


def model_exists(model_name: str) -> bool:
    return any((base / model_name / "model.sdf").is_file() for base in available_model_dirs())


def pose_components(node: ET.Element) -> list[float] | None:
    """Return the six SDF pose components, or None for a malformed pose."""
    try:
        values = [float(value) for value in (node.findtext("pose") or "").split()]
    except ValueError:
        return None
    return values if len(values) == 6 else None


def parse_required_level(manifest: dict, level: str, result: Validation) -> None:
    levels = manifest.get("levels", {}) or {}
    if level not in levels:
        result.error("unknown level %r; expected one of %s" % (level, ", ".join(sorted(levels))))


def world_for_level(manifest: dict, level: str, result: Validation) -> str:
    """Resolve the committed world artifact for one deterministic level."""
    levels = manifest.get("levels", {}) or {}
    profile = levels.get(level, {}) or {}
    world_value = str(profile.get("world") or manifest.get("world") or "").strip()
    if not world_value:
        result.error("manifest has no world path for level %r" % level)
    return world_value


def validate(manifest_path: Path, level: str, runtime: bool) -> tuple[Validation, dict]:
    result = Validation()
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        result.error("cannot read manifest %s: %s" % (manifest_path, exc))
        return result, {}

    parse_required_level(manifest, level, result)
    world_value = world_for_level(manifest, level, result)
    if not world_value:
        return result, manifest
    world_path = resolve_workspace_path(world_value, ROOT)
    if not world_path.is_file():
        result.error("world file does not exist: %s" % world_path)
        return result, manifest

    try:
        root = ET.parse(world_path).getroot()
    except (OSError, ET.ParseError) as exc:
        result.error("cannot parse world %s: %s" % (world_path, exc))
        return result, manifest
    world = root.find("world")
    if world is None:
        result.error("world XML has no <world> element")
        return result, manifest
    expected_world_name = str(manifest.get("world_name", "")).strip()
    if expected_world_name and world.get("name") != expected_world_name:
        result.error(
            "world name mismatch: manifest=%r XML=%r"
            % (expected_world_name, world.get("name"))
        )

    model_nodes = list(world.findall("model"))
    include_nodes = list(world.findall("include"))
    names = [node.get("name") for node in model_nodes]
    names.extend(node.findtext("name") for node in include_nodes)
    duplicate_names = sorted({name for name in names if name and names.count(name) > 1})
    if duplicate_names:
        result.error("duplicate world entity names: %s" % ", ".join(duplicate_names))
    if "pro3" in names:
        result.error("benchmark world must not contain Pro3; the launcher spawns it")
    if "ground_plane" not in names:
        result.error("world has no ground_plane model")
    if "office_building_structure" not in names:
        result.error("world has no office_building_structure model")

    structure = next(
        (node for node in model_nodes if node.get("name") == "office_building_structure"),
        None,
    )
    structure_links = {
        str(link.get("name", "")).strip()
        for link in (structure.findall("link") if structure is not None else [])
    }

    # Level-specific entities make the four deterministic difficulty
    # profiles auditable. A required entity may be either an included model
    # or a named structural link (for example, a narrow doorway lintel).
    level_profile = (manifest.get("levels", {}) or {}).get(level, {}) or {}
    world_entities = set(names) | structure_links
    for entity in level_profile.get("required_entities", []) or []:
        entity = str(entity).strip()
        if entity and entity not in world_entities:
            result.error(
                "level %s is missing required entity: %s" % (level, entity)
            )

    include_uris = []
    includes_by_name = {}
    for include in include_nodes:
        include_name = str(include.findtext("name") or "").strip()
        if include_name:
            includes_by_name[include_name] = include
        uri = (include.findtext("uri") or "").strip()
        if not uri.startswith("model://"):
            result.error("include %r does not use model:// URI" % (include.findtext("name")))
            continue
        include_uris.append(uri[len("model://") :])
    for model_name in sorted(set(include_uris)):
        if not model_exists(model_name):
            result.error(
                "required Gazebo model is not installed locally: %s "
                "(searched GAZEBO_MODEL_PATH and ~/.gazebo/models)" % model_name
            )

    required_models = manifest.get("required_models", []) or []
    for model_name in required_models:
        if not model_exists(str(model_name)):
            result.error("manifest required model is unavailable: %s" % model_name)

    building = manifest.get("building", {}) or {}
    bounds = building.get("bounds_m", [])
    target_xy = (
        (manifest.get("targets", {}) or {}).get("primary", {}) or {}
    ).get("pose_xy", [])
    if len(bounds) != 4:
        result.error("building.bounds_m must be [min_x, min_y, max_x, max_y]")
    else:
        min_x, min_y, max_x, max_y = [float(value) for value in bounds]
        target = (manifest.get("targets", {}) or {}).get("primary", {}) or {}
        target_xy = target.get("pose_xy", [])
        if len(target_xy) != 2:
            result.error("targets.primary.pose_xy must contain x and y")
        elif not (min_x <= float(target_xy[0]) <= max_x and min_y <= float(target_xy[1]) <= max_y):
            result.error("primary target lies outside building bounds: %s" % target_xy)

    target = (manifest.get("targets", {}) or {}).get("primary", {}) or {}
    target_model = str(target.get("model", "")).strip()
    target_name = str(target.get("pose_xy", "")).strip()
    if target_model != "cup_yellow":
        result.warning("primary target model is %r, not the default cup_yellow" % target_model)
    if target_model and target_model not in names:
        result.error("primary target model %r is not an entity in the world" % target_model)
    target_room = str(target.get("room", "")).strip()
    room_ids = {str(room.get("id", "")).strip() for room in manifest.get("rooms", []) or []}
    if target_room not in room_ids:
        result.error("primary target room %r is not declared in rooms" % target_room)

    robot_profiles = manifest.get("robot_profiles", {}) or {}
    for profile_name, profile in robot_profiles.items():
        if not isinstance(profile, dict):
            result.error("robot profile %s must be a mapping" % profile_name)
            continue
        pose = profile.get("pose", [])
        if len(pose) != 3:
            result.error("robot profile %s pose must be [x, y, yaw]" % profile_name)
            continue
        try:
            profile_xy = [float(pose[0]), float(pose[1])]
            float(pose[2])
        except (TypeError, ValueError):
            result.error("robot profile %s pose must contain numeric values" % profile_name)
            continue
        if len(bounds) == 4:
            if not (
                float(bounds[0]) <= profile_xy[0] <= float(bounds[2])
                and float(bounds[1]) <= profile_xy[1] <= float(bounds[3])
            ):
                result.error(
                    "robot profile %s lies outside building bounds: %s"
                    % (profile_name, pose)
                )

    target_entry = robot_profiles.get("target_entry")
    if target_entry is None:
        result.error("manifest must declare robot profile target_entry")
    elif isinstance(target_entry, dict):
        if target_entry.get("diagnostic_only") is not True:
            result.error("robot profile target_entry must be diagnostic_only")
        if target_entry.get("bypasses_exploration") is not True:
            result.error("robot profile target_entry must declare bypasses_exploration")
        if target_entry.get("benchmark_evidence") is not False:
            result.error("robot profile target_entry must not produce benchmark evidence")
        if "target_approach_isolation" not in (target_entry.get("use_for") or []):
            result.error(
                "robot profile target_entry must be marked for target_approach_isolation"
            )
        if len(target_xy) == 2:
            try:
                entry_xy = [float(target_entry["pose"][0]), float(target_entry["pose"][1])]
                target_xy_float = [float(target_xy[0]), float(target_xy[1])]
            except (KeyError, TypeError, ValueError, IndexError):
                entry_xy = None
                target_xy_float = None
            if entry_xy is not None and target_xy_float is not None:
                if all(abs(a - b) <= 1e-6 for a, b in zip(entry_xy, target_xy_float)):
                    result.error("robot profile target_entry must not equal primary target truth")
    else:
        result.error("robot profile target_entry must be a mapping")
    door_ids = {str(door.get("id", "")).strip() for door in manifest.get("doors", []) or []}
    for room in manifest.get("rooms", []) or []:
        for door_id in room.get("entry_doors", []) or []:
            if door_id not in door_ids:
                result.error("room %s references unknown door %s" % (room.get("id"), door_id))
    for door in manifest.get("doors", []) or []:
        door_id = str(door.get("id", "")).strip()
        if not door_id:
            result.error("door has no id")
            continue
        if door.get("benchmark_closed"):
            required_link = "south_entry_closed_door"
        else:
            required_link = door_id + "_lintel"
        if required_link not in structure_links:
            result.error(
                "door %s is missing its world marker link %s"
                % (door_id, required_link)
            )

    # Keep furniture semantically grounded.  The supplied monitor model has a
    # visible stand, therefore the conference presentation screen must share
    # a pose with an explicit desk at normal desktop height.
    presentation_desk = includes_by_name.get("conference_presentation_desk")
    presentation_monitor = includes_by_name.get("conference_presentation_monitor")
    if (presentation_desk is None) != (presentation_monitor is None):
        result.error("conference presentation desk and monitor must be included together")
    elif presentation_desk is not None and presentation_monitor is not None:
        desk_pose = pose_components(presentation_desk)
        monitor_pose = pose_components(presentation_monitor)
        if desk_pose is None or monitor_pose is None:
            result.error("conference presentation desk or monitor has an invalid pose")
        elif (
            abs(desk_pose[0] - monitor_pose[0]) > 0.03
            or abs(desk_pose[1] - monitor_pose[1]) > 0.03
            or not (0.70 <= monitor_pose[2] <= 0.75)
        ):
            result.error("conference presentation monitor must rest on its desk at desktop height")

    cup_names = ("cup_blue_target_distractor", "cup_blue_target_distractor_2")
    target_cups = [includes_by_name[name] for name in cup_names if name in includes_by_name]
    if target_cups:
        counter = next(
            (
                link
                for link in (structure.findall("link") if structure is not None else [])
                if link.get("name") == "target_room_refreshment_counter"
            ),
            None,
        )
        if counter is None:
            result.error("target-room cups require target_room_refreshment_counter")
        else:
            counter_pose = pose_components(counter)
            size_text = counter.findtext("visual/geometry/box/size")
            try:
                counter_size = [float(value) for value in (size_text or "").split()]
            except ValueError:
                counter_size = []
            if counter_pose is None or len(counter_size) != 3:
                result.error("target_room_refreshment_counter has invalid geometry or pose")
            else:
                top_z = counter_pose[2] + counter_size[2] / 2.0
                for cup in target_cups:
                    cup_name = str(cup.findtext("name") or "cup")
                    cup_pose = pose_components(cup)
                    if cup_pose is None:
                        result.error("%s has an invalid pose" % cup_name)
                        continue
                    within_counter = (
                        abs(cup_pose[0] - counter_pose[0]) <= counter_size[0] / 2.0
                        and abs(cup_pose[1] - counter_pose[1]) <= counter_size[1] / 2.0
                    )
                    if not within_counter or not (top_z <= cup_pose[2] <= top_z + 0.005):
                        result.error("%s must rest on target_room_refreshment_counter" % cup_name)

    actual_hash = sha256(world_path)
    # Prefer the level-local hash.  A single top-level hash cannot describe
    # four independent deterministic world artifacts and would let a modified
    # Level 4 file pass validation when Level 2 happened to be the default.
    declared_hash = str(
        level_profile.get("world_sha256") or manifest.get("world_sha256", "")
    ).strip()
    if declared_hash and declared_hash != "generated_at_build_time" and declared_hash != actual_hash:
        result.error("world_sha256 mismatch: manifest=%s actual=%s" % (declared_hash, actual_hash))

    if runtime:
        required_topics = ((manifest.get("acceptance", {}) or {}).get("require_topics", []) or [])
        try:
            output = subprocess.check_output(["rostopic", "list"], text=True, stderr=subprocess.STDOUT, timeout=5)
            topics = set(output.splitlines())
            for topic in required_topics:
                if str(topic) not in topics:
                    result.error("required runtime topic is not currently advertised: %s" % topic)
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            result.error("runtime topic check failed: %s" % exc)

    details = {
        "manifest": str(manifest_path),
        "world": str(world_path),
        "world_sha256": actual_hash,
        "world_name": world.get("name"),
        "level": level,
        "world_entities": len(names),
        "model_includes": len(include_nodes),
        "errors": result.errors,
        "warnings": result.warnings,
    }
    return result, details


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--level", default="level_2")
    parser.add_argument("--runtime", action="store_true", help="also require ROS topics to be advertised")
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args()
    manifest_path = args.manifest if args.manifest.is_absolute() else ROOT / args.manifest
    result, details = validate(manifest_path, args.level, args.runtime)
    if args.as_json:
        print(json.dumps(details, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print("office_building validation: %s" % ("PASS" if not result.errors else "FAIL"))
        print("  world: %s" % details.get("world", "unknown"))
        print("  sha256: %s" % details.get("world_sha256", "unknown"))
        print("  level: %s" % args.level)
        for warning in result.warnings:
            print("  WARN: %s" % warning)
        for error in result.errors:
            print("  ERROR: %s" % error)
    return 1 if result.errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
