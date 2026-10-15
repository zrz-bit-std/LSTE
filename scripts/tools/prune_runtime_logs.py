#!/usr/bin/env python3
"""Keep timestamped runtime log directories below a shared byte budget."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import stat
import sys


RUN_DIRECTORY_RE = re.compile(r"^[0-9]{8}_[0-9]{6}$")
MAX_LOG_BYTES = 1_000_000_000


def _walk_regular_bytes(path: Path) -> int:
    """Return regular-file bytes without following directory symlinks."""
    total = 0
    if not path.exists() or path.is_symlink():
        return total
    for directory, directory_names, file_names in os.walk(
        path, topdown=True, followlinks=False
    ):
        directory_path = Path(directory)
        directory_names[:] = [
            name
            for name in directory_names
            if not (directory_path / name).is_symlink()
        ]
        for name in file_names:
            file_path = directory_path / name
            try:
                file_stat = file_path.lstat()
            except OSError:
                continue
            if stat.S_ISREG(file_stat.st_mode):
                total += file_stat.st_size
    return total


def _log_roots(runtime_root: Path) -> list[Path]:
    """Find formal log roots below one runtime tree."""
    roots = []
    if not runtime_root.is_dir():
        return roots
    for directory, directory_names, _ in os.walk(
        runtime_root, topdown=True, followlinks=False
    ):
        directory_path = Path(directory)
        directory_names[:] = [
            name
            for name in directory_names
            if not (directory_path / name).is_symlink()
        ]
        if directory_path.name != "logs":
            continue
        roots.append(directory_path)
        # A formal log root owns its direct timestamp directories. Nested
        # directories are part of a run and must not become separate roots.
        directory_names[:] = []
    return sorted(roots)


def _run_directories(log_root: Path) -> list[Path]:
    try:
        children = list(log_root.iterdir())
    except OSError:
        return []
    return sorted(
        (
            child
            for child in children
            if child.is_dir()
            and not child.is_symlink()
            and RUN_DIRECTORY_RE.fullmatch(child.name)
        ),
        key=lambda path: (path.name, str(path)),
    )


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _protected_by_symlink(runtime_root: Path, candidates: list[Path]) -> set[Path]:
    """Protect a run referenced by a runtime ``current*``-style symlink."""
    protected = set()
    resolved_candidates = {
        candidate: candidate.resolve(strict=False) for candidate in candidates
    }
    for directory, directory_names, file_names in os.walk(
        runtime_root, topdown=True, followlinks=False
    ):
        directory_path = Path(directory)
        entries = [directory_path / name for name in directory_names + file_names]
        for entry in entries:
            # ROS commonly creates an internal ``ros/latest`` link in every
            # run.  It is not an active-run pointer and must not pin old logs.
            if not entry.is_symlink() or not entry.name.startswith("current"):
                continue
            try:
                target = entry.resolve(strict=False)
            except OSError:
                continue
            for candidate, resolved_candidate in resolved_candidates.items():
                if target == resolved_candidate or _is_within(
                    target, resolved_candidate
                ):
                    protected.add(candidate)
    return protected


def prune(runtime_root: Path, max_bytes: int, keep_paths: list[Path]) -> int:
    roots = _log_roots(runtime_root)
    if not roots:
        print("prune_runtime_logs: no log roots found")
        return 0

    candidates = []
    for log_root in roots:
        run_directories = _run_directories(log_root)
        for run_directory in run_directories:
            candidates.append(
                {
                    "path": run_directory,
                    "size": _walk_regular_bytes(run_directory),
                }
            )

    candidate_paths = [item["path"] for item in candidates]
    protected = _protected_by_symlink(runtime_root, candidate_paths)
    for keep_path in keep_paths:
        resolved_keep = keep_path.resolve(strict=False)
        protected.update(
            candidate
            for candidate in candidate_paths
            if resolved_keep == candidate.resolve(strict=False)
            or _is_within(resolved_keep, candidate.resolve(strict=False))
        )

    total_before = sum(_walk_regular_bytes(root) for root in roots)
    total = total_before
    removed = []
    failures = []
    removable = sorted(
        (
            item
            for item in candidates
            if item["path"] not in protected
        ),
        key=lambda item: (item["path"].name, str(item["path"])),
    )
    for item in removable:
        if total <= max_bytes:
            break
        path = item["path"]
        try:
            shutil.rmtree(path)
        except FileNotFoundError:
            total -= item["size"]
            continue
        except OSError as exc:
            failures.append(f"{path}: {exc}")
            continue
        total -= item["size"]
        removed.append(str(path))

    if removed or failures:
        total_after = sum(_walk_regular_bytes(root) for root in roots)
    else:
        total_after = total_before
    print(
        "prune_runtime_logs: total_before=%d total_after=%d "
        "max_bytes=%d removed=%d protected=%d"
        % (total_before, total_after, max_bytes, len(removed), len(protected))
    )
    for path in removed:
        print(f"prune_runtime_logs: removed={path}")
    for failure in failures:
        print(f"prune_runtime_logs: removal_failed={failure}", file=sys.stderr)

    if total_after > max_bytes:
        print(
            "prune_runtime_logs: byte budget remains exceeded; only protected "
            "or non-timestamped log entries remain",
            file=sys.stderr,
        )
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Prune oldest timestamped runtime log directories."
    )
    parser.add_argument(
        "--runtime-root",
        type=Path,
        required=True,
        help="Workspace runtime directory containing one or more logs roots.",
    )
    parser.add_argument(
        "--max-bytes",
        type=int,
        default=MAX_LOG_BYTES,
        help="Maximum aggregate regular-file size (default: 1 GB).",
    )
    parser.add_argument(
        "--keep",
        dest="keep_paths",
        action="append",
        type=Path,
        default=[],
        help="Run directory to preserve; may be repeated.",
    )
    args = parser.parse_args()
    if args.max_bytes <= 0 or args.max_bytes > MAX_LOG_BYTES:
        parser.error(
            "--max-bytes must be between 1 and %d bytes" % MAX_LOG_BYTES
        )
    return prune(
        args.runtime_root.resolve(),
        args.max_bytes,
        [path.resolve() for path in args.keep_paths],
    )


if __name__ == "__main__":
    raise SystemExit(main())
