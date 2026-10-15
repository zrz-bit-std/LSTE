#!/usr/bin/env python3
"""Aggregate timestamped office-study records into CSV and JSON evidence."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

from experiment_aggregation import aggregate_rows, normalized_row
from experiment_protocol import ROOT, build_trials, load_matrix
from summarize_run import read_events


DEFAULT_LOG_ROOT = ROOT / "runtime/office_building_benchmark/logs"
DEFAULT_MATRIX = ROOT / "scripts/tests/office_building/experiment_matrix.yaml"

# These fields define the identity of one formal process-restart trial.  The
# seed is included deliberately: reusing a trial_id with a different seed is
# a different experiment, not a replacement that can silently pass auditing.
TRIAL_KEY_FIELDS = (
    "study_id",
    "phase",
    "method",
    "level",
    "trial_id",
    "seed",
)


def _resolve_path(value):
    path = Path(str(value or ""))
    return path if path.is_absolute() else ROOT / path


def trial_key(value) -> tuple:
    """Return the stable identity used to compare records with the matrix.

    ``value`` may be an ``ExperimentTrial`` or a raw/normalized record.  The
    latter uses ``method_requested`` because normalized rows intentionally
    rename the raw record's ``method`` field.
    """
    fields = {}
    for field in TRIAL_KEY_FIELDS:
        if isinstance(value, dict):
            if field == "method":
                item = value.get("method_requested")
                if item is None:
                    item = value.get("method")
            else:
                item = value.get(field)
        else:
            item = getattr(value, field, None)
        if field in ("trial_id", "seed") and item is not None:
            try:
                item = int(item)
            except (TypeError, ValueError):
                pass
        elif item is not None:
            item = str(item)
        fields[field] = item
    return tuple(fields[field] for field in TRIAL_KEY_FIELDS)


def _key_sort_value(key):
    """Make keys containing missing fields safely sortable."""
    return tuple("" if value is None else str(value) for value in key)


def _key_as_dict(key) -> dict:
    return dict(zip(TRIAL_KEY_FIELDS, key))


def _non_formal_phases(matrix: dict) -> set[str]:
    """Return explicitly diagnostic phases excluded from the denominator."""
    return {
        str(phase) for phase, definition in (matrix.get("phases") or {}).items()
        if isinstance(definition, dict) and definition.get("formal", True) is False
    }


def expected_trial_keys(matrix: dict) -> list[tuple]:
    """Expand formal phases in a matrix into their trial identities."""
    trials = []
    non_formal = _non_formal_phases(matrix)
    for phase in matrix.get("phases", {}):
        if str(phase) in non_formal:
            continue
        trials.extend(build_trials(matrix, phase))
    return [trial_key(trial) for trial in trials]


def compare_trial_keys(records, expected_keys) -> dict:
    """Audit observed records against the formal matrix trial set.

    This is intentionally independent of metric validity.  A trial with a
    failed navigation outcome is still an observed trial; a missing record,
    duplicate identity, or record outside the matrix makes the study
    incomplete and must be reported as such.
    """
    expected_counts = Counter(tuple(key) for key in expected_keys)
    observed_keys = [trial_key(record) for record in records]
    observed_counts = Counter(observed_keys)
    missing_counts = expected_counts - observed_counts
    # Extra copies of a declared key belong to the duplicate category.  Only
    # identities absent from the matrix are "unexpected" trials.
    unexpected_counts = {
        key: count for key, count in observed_counts.items()
        if key not in expected_counts
    }
    duplicate_counts = {
        key: count for key, count in observed_counts.items() if count > 1
    }

    missing = sorted(missing_counts, key=_key_sort_value)
    unexpected = sorted(unexpected_counts, key=_key_sort_value)
    duplicates = sorted(duplicate_counts, key=_key_sort_value)
    complete = not missing and not unexpected and not duplicates
    return {
        "status": "complete" if complete else "incomplete",
        "complete": complete,
        "expected_trial_count": sum(expected_counts.values()),
        "observed_record_count": len(observed_keys),
        "observed_unique_trial_count": len(observed_counts),
        "missing_trial_count": sum(missing_counts.values()),
        "unexpected_trial_count": sum(unexpected_counts.values()),
        "duplicate_trial_count": sum(
            count - 1 for count in duplicate_counts.values()
        ),
        "missing_trial_keys": [_key_as_dict(key) for key in missing],
        "unexpected_trial_keys": [_key_as_dict(key) for key in unexpected],
        "duplicate_trial_keys": [
            {**_key_as_dict(key), "observed_count": duplicate_counts[key]}
            for key in duplicates
        ],
    }


def _metrics_provenance(metrics_path: Path) -> dict:
    """Recover immutable run provenance from the metrics run_start event."""
    if not metrics_path.is_file():
        return {}
    try:
        run_start = next(
            data for event, data in read_events(metrics_path)
            if event == "run_start"
        )
    except StopIteration:
        return {}
    experiment = run_start.get("experiment") or {}
    resolved = run_start.get("resolved_params") or {}
    detector = run_start.get("detector") or {}
    provenance = {
        "pipeline_config": experiment.get("pipeline_config"),
        "pipeline_config_sha256": experiment.get("pipeline_config_sha256"),
        "git_revision": experiment.get("git_revision"),
        "git_dirty": experiment.get("git_dirty"),
        "controller_observed": resolved.get("controller_mode"),
        "detector": detector.get("name"),
        "initial_pose": experiment.get("initial_pose"),
    }
    world = _resolve_path(experiment.get("world"))
    if world.is_file():
        provenance["world_sha256"] = hashlib.sha256(world.read_bytes()).hexdigest()
    return {key: value for key, value in provenance.items() if value is not None}


def load_record(record_path: Path):
    record = json.loads(record_path.read_text(encoding="utf-8"))
    summary_path = _resolve_path(record.get("summary"))
    if not summary_path.is_file():
        # Keep the record visible as an excluded trial instead of aborting the
        # whole study because one process died before writing its summary.
        record["infrastructure_failure"] = "summary_missing"
        summary = {}
    else:
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            record["infrastructure_failure"] = "summary_invalid"
            summary = {}
    metrics_path = _resolve_path(
        record.get("metrics_log") or summary.get("log")
    )
    enriched = dict(record)
    enriched["record_path"] = str(record_path.resolve())
    for key, value in _metrics_provenance(metrics_path).items():
        enriched.setdefault(key, value)
    return normalized_row(enriched, summary)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-root", type=Path, default=DEFAULT_LOG_ROOT)
    parser.add_argument("--matrix", type=Path, default=DEFAULT_MATRIX)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    records = sorted(args.log_root.glob("*/*_experiment_record.json"))
    rows = [load_record(record) for record in records]
    matrix = load_matrix(args.matrix)
    non_formal_phases = _non_formal_phases(matrix)
    formal_rows = [
        row for row in rows
        if str(row.get("phase") or "") not in non_formal_phases
    ]
    completeness = compare_trial_keys(formal_rows, expected_trial_keys(matrix))
    completeness["formal_phases"] = [
        str(phase) for phase in matrix.get("phases", {})
        if str(phase) not in non_formal_phases
    ]
    completeness["non_formal_phases"] = sorted(non_formal_phases)
    completeness["non_formal_record_count"] = len(rows) - len(formal_rows)
    aggregates = aggregate_rows(rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "trial_results.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "method_level_summary.json").write_text(
        json.dumps(aggregates, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "study_completeness.json").write_text(
        json.dumps(completeness, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    fields = sorted({key for row in rows for key in row})
    with (args.output_dir / "trial_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({
        "records": len(rows),
        "study_completeness": completeness,
        "aggregates": aggregates,
    }, ensure_ascii=False, indent=2))
    # Results are still written for diagnosis, but callers must not treat an
    # incomplete matrix as a completed study.
    return 0 if completeness["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
