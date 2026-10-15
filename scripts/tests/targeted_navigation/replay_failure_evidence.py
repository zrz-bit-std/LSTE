#!/usr/bin/env python3
"""Replay one captured local navigation failure without starting ROS.

This is the fast inner loop for failure-evidence and classifier changes.  It
replays a small pre-trigger/trigger/post-window fixture through the same pure
classification functions used by ``lste_navigation_metrics``.  It does not
claim to replace a physical Gazebo run: controller, sensor, and costmap
changes still require ``run_t_junction_endpoint.sh`` afterward.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import sys
import time
from typing import Callable, Optional


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_FIXTURE = (
    WORKSPACE_ROOT
    / "scripts"
    / "tests"
    / "targeted_navigation"
    / "fixtures"
    / "t_junction_controller_stall.json"
)
DEFAULT_LOG_ROOT = (
    WORKSPACE_ROOT
    / "runtime"
    / "targeted_navigation"
    / "failure_evidence_replay"
    / "logs"
)
TIMESTAMP_PATTERN = re.compile(r"^\d{8}_\d{6}$")
PROCESS_NAME = "failure_evidence_replay"

SCRIPTS = WORKSPACE_ROOT / "src" / "lste_topo_access" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from navigation_metrics_failure_evidence import (  # noqa: E402
    classify_failure,
    diagnose_failure_sample,
)


class ReplayInvariantError(RuntimeError):
    """Raised when a failure fixture no longer matches its contract."""


class ReplayLogger:
    """Write one timestamped structured log for this replay process."""

    def __init__(self, path: Path, clock: Optional[Callable[[], str]] = None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("w", encoding="utf-8")
        self._clock = clock or _wall_timestamp
        self.events = []

    @staticmethod
    def _field(value):
        return json.dumps(value, ensure_ascii=True, separators=(",", ":"))

    def event(self, level: str, name: str, **fields):
        self.events.append(str(name))
        parts = [
            self._clock(),
            "[%s]" % str(level).upper(),
            "process=%s" % PROCESS_NAME,
            "event=%s" % str(name),
        ]
        for key in sorted(fields):
            parts.append("%s=%s" % (key, self._field(fields[key])))
        self._stream.write(" ".join(parts) + "\n")
        self._stream.flush()

    def close(self):
        self._stream.close()


def _wall_timestamp():
    return datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S.%f%z")


def _new_run_directory(log_root: Path, requested_timestamp=None):
    """Create one collision-free YYYYMMDD_HHMMSS directory."""
    log_root = Path(log_root)
    log_root.mkdir(parents=True, exist_ok=True)
    if requested_timestamp is not None:
        timestamp = str(requested_timestamp)
        if not TIMESTAMP_PATTERN.match(timestamp):
            raise ValueError("timestamp must use YYYYMMDD_HHMMSS")
        directory = log_root / timestamp
        directory.mkdir()
        return timestamp, directory

    while True:
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        directory = log_root / timestamp
        try:
            directory.mkdir()
            return timestamp, directory
        except FileExistsError:
            time.sleep(1.0)


def _pause(step_delay):
    if step_delay:
        time.sleep(float(step_delay))


def _require(condition, message):
    if not condition:
        raise ReplayInvariantError(message)


def load_fixture(path: Path):
    """Load and validate the small replay contract before opening a log."""
    path = Path(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    _require(isinstance(payload, dict), "fixture root must be an object")
    _require(payload.get("schema_version") == 1, "unsupported fixture schema")
    trigger = payload.get("trigger")
    _require(isinstance(trigger, dict), "fixture trigger is missing")
    _require(str(trigger.get("name") or "").strip(), "trigger name is missing")
    samples = payload.get("samples")
    _require(isinstance(samples, list) and samples, "fixture samples are missing")
    for entry in samples:
        _require(isinstance(entry, dict), "fixture sample entry must be an object")
        _require(isinstance(entry.get("sample"), dict), "fixture sample payload is missing")
    expected = payload.get("expected")
    _require(isinstance(expected, dict), "fixture expected contract is missing")
    return payload


def _sample_command(sample):
    command = sample.get("cmd_vel")
    if command is None:
        command = sample.get("cmd")
    if not isinstance(command, (list, tuple)) or len(command) < 2:
        return [0.0, 0.0]
    try:
        return [float(command[0]), float(command[1])]
    except (TypeError, ValueError):
        return [0.0, 0.0]


def _diagnose_sample(trigger, sample, details, recent_events):
    details = dict(details)
    details["trigger"] = trigger
    classification = classify_failure(
        trigger,
        sample,
        details=details,
        recent_events=recent_events,
    )
    diagnosis = diagnose_failure_sample(sample, details=details)
    return classification, diagnosis


def _assert_expected(fixture, trigger_sample, classification, diagnosis):
    expected = fixture["expected"]
    _require(
        classification.get("label") == expected.get("classification"),
        "classification changed: expected %s, got %s"
        % (expected.get("classification"), classification.get("label")),
    )
    _require(
        diagnosis.get("primary_cause") == expected.get("diagnosis_primary_cause"),
        "diagnosis cause changed: expected %s, got %s"
        % (
            expected.get("diagnosis_primary_cause"),
            diagnosis.get("primary_cause"),
        ),
    )
    _require(
        diagnosis.get("layer") == expected.get("diagnosis_layer"),
        "diagnosis layer changed: expected %s, got %s"
        % (expected.get("diagnosis_layer"), diagnosis.get("layer")),
    )
    _require(
        diagnosis.get("route_id") == expected.get("route_id"),
        "route identity changed: expected %s, got %s"
        % (expected.get("route_id"), diagnosis.get("route_id")),
    )
    _require(
        diagnosis.get("route_kind") == expected.get("route_kind"),
        "route kind changed: expected %s, got %s"
        % (expected.get("route_kind"), diagnosis.get("route_kind")),
    )
    scan = trigger_sample.get("scan") or {}
    try:
        forward_clearance = float(scan["forward_min"])
        clearance_limit = float(expected["forward_clearance_min_m"])
    except (KeyError, TypeError, ValueError):
        raise ReplayInvariantError("trigger clearance evidence is missing")
    _require(
        forward_clearance >= clearance_limit,
        "trigger is not a clear-path sample: %.3f < %.3f"
        % (forward_clearance, clearance_limit),
    )
    command = _sample_command(trigger_sample)
    command_zero = abs(command[0]) <= 0.05 and abs(command[1]) <= 0.03
    _require(
        command_zero == bool(expected.get("command_zero_at_trigger")),
        "trigger command zero contract changed",
    )


def replay(fixture, log_path: Path, run_timestamp: str, step_delay=0.0):
    """Run one deterministic evidence replay and return a JSON result."""
    started = time.monotonic()
    trigger = fixture["trigger"]
    trigger_name = str(trigger["name"])
    trigger_source = str(trigger.get("source") or "fixture")
    details = dict(trigger.get("details") or {})
    recent_events = fixture.get("recent_events") or []
    logger = ReplayLogger(log_path)
    phase_results = []

    try:
        logger.event(
            "INFO",
            "run_start",
            scenario_id=fixture.get("scenario_id"),
            run_timestamp=run_timestamp,
            source_failure_id=fixture.get("source_failure_id"),
            trigger=trigger_name,
            trigger_source=trigger_source,
            ros_enabled=False,
            gazebo_enabled=False,
        )
        for entry in fixture["samples"]:
            phase = str(entry.get("phase") or "sample")
            sample = entry["sample"]
            classification, diagnosis = _diagnose_sample(
                trigger_name, sample, details, recent_events
            )
            phase_result = {
                "phase": phase,
                "wall_elapsed_seconds": sample.get("wall_elapsed_seconds"),
                "ros_time": sample.get("ros_time"),
                "classification": classification,
                "diagnosis": diagnosis,
            }
            phase_results.append(phase_result)
            logger.event(
                "INFO",
                "sample_diagnosed",
                phase=phase,
                ros_time=sample.get("ros_time"),
                classification=classification.get("label"),
                confidence=classification.get("confidence"),
                primary_cause=diagnosis.get("primary_cause"),
                layer=diagnosis.get("layer"),
                route_id=diagnosis.get("route_id"),
                route_kind=diagnosis.get("route_kind"),
                pose=(diagnosis.get("spatial") or {}).get("pose"),
                goal=(diagnosis.get("spatial") or {}).get("goal"),
                command_chain=diagnosis.get("command_chain"),
                execution_phase=diagnosis.get("execution_phase"),
            )
            _pause(step_delay)

        trigger_entry = next(
            (item for item in fixture["samples"] if item.get("phase") == "trigger"),
            None,
        )
        _require(trigger_entry is not None, "fixture trigger sample is missing")
        trigger_result = next(
            item for item in phase_results if item.get("phase") == "trigger"
        )
        _assert_expected(
            fixture,
            trigger_entry["sample"],
            trigger_result["classification"],
            trigger_result["diagnosis"],
        )
        logger.event(
            "INFO",
            "failure_snapshot_replayed",
            source_failure_id=fixture.get("source_failure_id"),
            classification=trigger_result["classification"].get("label"),
            diagnosis=trigger_result["diagnosis"].get("primary_cause"),
            route_id=trigger_result["diagnosis"].get("route_id"),
        )
        result = {
            "scenario_id": fixture.get("scenario_id"),
            "run_timestamp": run_timestamp,
            "log_directory": str(log_path.parent),
            "log_file": str(log_path),
            "source_failure_id": fixture.get("source_failure_id"),
            "passed": True,
            "phases": phase_results,
            "trigger_classification": trigger_result["classification"],
            "trigger_diagnosis": trigger_result["diagnosis"],
        }
        logger.event(
            "INFO",
            "run_complete",
            status="passed",
            elapsed_wall_seconds=round(time.monotonic() - started, 6),
            phase_count=len(phase_results),
        )
        return result
    except Exception as error:
        logger.event(
            "ERROR",
            "run_complete",
            status="failed",
            error_type=type(error).__name__,
            error=str(error),
            elapsed_wall_seconds=round(time.monotonic() - started, 6),
        )
        raise
    finally:
        logger.close()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("run",))
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--log-root", type=Path, default=DEFAULT_LOG_ROOT)
    parser.add_argument("--timestamp")
    parser.add_argument("--step-delay", type=float, default=0.0)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.step_delay < 0.0:
        raise SystemExit("step-delay must not be negative")
    fixture = load_fixture(args.fixture)
    run_timestamp, run_directory = _new_run_directory(
        args.log_root, requested_timestamp=args.timestamp
    )
    log_path = run_directory / (run_timestamp + "_failure_evidence_replay.log")
    try:
        result = replay(fixture, log_path, run_timestamp, args.step_delay)
    except Exception as error:
        print("failure_evidence_replay failed: %s" % error, file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
