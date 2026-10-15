#!/usr/bin/env python3
"""Pure parser and validator for the office exploration study matrix."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/lste_topo_access/scripts"
MANIFEST = ROOT / "worlds/benchmark/office_building_v1_manifest.yaml"
DEFAULT_STARTUP_TIMEOUT_SECONDS = 300
DEFAULT_STARTUP_READINESS_TIMEOUT_SECONDS = 45
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_experiment_policy import resolve_exploration_method


@dataclass(frozen=True)
class ExperimentTrial:
    """One independently restarted, comparable benchmark invocation."""

    study_id: str
    phase: str
    method: str
    level: str
    trial_id: int
    profile: str
    controller: str
    startup_timeout_seconds: int
    timeout_seconds: int
    seed: int
    terminal_event: str
    task_json: str
    task_id: str
    target_eval_enabled: bool
    video_required: bool
    video_window: str
    video_fps: int
    # Kept at the end with a default so older programmatic trial constructors
    # remain valid while matrix files can opt into the short readiness gate.
    startup_readiness_timeout_seconds: int = DEFAULT_STARTUP_READINESS_TIMEOUT_SECONDS
    # Diagnostic runners may disable video without changing the declarative
    # matrix. Keep the reason on the immutable trial so every artifact records
    # that this was an explicit local override.
    video_override: str = ""


def load_matrix(path: Path) -> dict:
    """Load the YAML matrix and fail early on malformed study definitions."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict) or int(data.get("schema_version", 0)) != 1:
        raise ValueError("experiment matrix must be schema_version: 1")
    methods = data.get("methods") or []
    if not methods:
        raise ValueError("experiment matrix has no methods")
    if len(set(str(method) for method in methods)) != len(methods):
        raise ValueError("experiment matrix methods must be unique")
    for method in methods:
        resolve_exploration_method(method)
    if not isinstance(data.get("phases"), dict) or not data["phases"]:
        raise ValueError("experiment matrix has no phases")
    return data


def build_trials(matrix: dict, phase: str) -> list[ExperimentTrial]:
    """Expand one phase into a deterministic Cartesian product."""
    try:
        definition = matrix["phases"][phase]
    except KeyError as exc:
        raise ValueError("unknown experiment phase: %s" % phase) from exc
    levels = [str(value) for value in definition.get("levels") or []]
    seeds = [int(value) for value in definition.get("seeds") or []]
    # Keep custom matrices backward compatible while giving navigation a
    # separate bound from the launcher's process-group timeout. Benchmark
    # phases may lower or raise this value when the startup evidence contract
    # calls for a different diagnostic window.
    startup_timeout = int(
        definition.get("startup_timeout_seconds")
        or DEFAULT_STARTUP_TIMEOUT_SECONDS
    )
    startup_readiness_timeout = int(
        definition.get("startup_readiness_timeout_seconds")
        or DEFAULT_STARTUP_READINESS_TIMEOUT_SECONDS
    )
    timeout = int(definition.get("timeout_seconds") or 0)
    terminal_event = str(definition.get("terminal_event") or "").strip()
    task_json = str(definition.get("task_json") or "").strip()
    task_id = str(definition.get("task_id") or "").strip()
    if (
        not levels
        or not seeds
        or startup_timeout <= 0
        or startup_readiness_timeout <= 0
        or timeout <= 0
        or not terminal_event
    ):
        raise ValueError(
            "phase %s needs levels, startup_timeout_seconds, and timeout_seconds"
            % phase
        )
    if any(seed < 0 for seed in seeds) or len(set(seeds)) != len(seeds):
        raise ValueError("phase %s needs distinct non-negative seeds" % phase)
    if terminal_event not in ("frontier_exhausted", "task_done"):
        raise ValueError("phase %s has unsupported terminal_event" % phase)
    if not task_json or not task_id:
        raise ValueError("phase %s needs task_json and task_id" % phase)
    study_id = str(matrix.get("study_id") or "office_study")
    profile = str(matrix.get("profile") or "primary")
    controller = str(matrix.get("controller") or "teb").strip().lower()
    if controller not in ("teb", "teleop", "sappo"):
        raise ValueError("unsupported controller in experiment matrix: %s" % controller)
    if MANIFEST.is_file():
        manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8")) or {}
        declared_levels = set((manifest.get("levels") or {}).keys())
        unknown_levels = sorted(set(levels) - declared_levels)
        if unknown_levels:
            raise ValueError(
                "phase %s has levels absent from benchmark manifest: %s"
                % (phase, ", ".join(unknown_levels))
            )
    task_path = Path(task_json)
    if not task_path.is_absolute():
        task_path = ROOT / task_path
    if not task_path.is_file():
        raise ValueError("phase %s task_json does not exist: %s" % (phase, task_path))
    artifacts = matrix.get("artifacts") or {}
    video_required = bool(artifacts.get("require_video", False))
    video_window = str(artifacts.get("video_window") or "Gazebo").strip()
    video_fps = int(artifacts.get("video_fps") or 20)
    if video_required and (not video_window or video_fps <= 0):
        raise ValueError("video capture needs a non-empty window and positive FPS")
    trials = [
        ExperimentTrial(
            study_id=study_id,
            phase=str(phase),
            method=str(method),
            level=level,
            trial_id=trial_id,
            profile=profile,
            controller=controller,
            startup_timeout_seconds=startup_timeout,
            startup_readiness_timeout_seconds=startup_readiness_timeout,
            timeout_seconds=timeout,
            seed=seed,
            terminal_event=terminal_event,
            task_json=task_json,
            task_id=task_id,
            target_eval_enabled=bool(definition.get("target_eval_enabled", True)),
            video_required=video_required,
            video_window=video_window,
            video_fps=video_fps,
        )
        for method in matrix["methods"]
        for level in levels
        for trial_id, seed in enumerate(seeds, start=1)
    ]
    if len(set(trials)) != len(trials):
        raise ValueError("experiment matrix expands to duplicate trials")
    return trials
