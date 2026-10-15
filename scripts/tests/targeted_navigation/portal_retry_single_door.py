#!/usr/bin/env python3
"""Replay one source-to-destination portal retry without ROS.

This is a deterministic contract test at the experiment boundary, not a
Gazebo or controller test.  It exercises the smallest useful failure trace:

    source evidence -> route failure -> retry -> destination evidence
    -> crossing accepted

The replay uses the durable directional branch model.  Map frontier
coordinates are deliberately changed between attempts; the physical branch
and WorkItem must remain the same.  Every invocation writes one process log
under ``runtime/targeted_navigation/portal_retry_single_door/logs`` (or the
directory selected by ``--log-root``).
"""

from dataclasses import dataclass
from datetime import datetime
import argparse
import json
from pathlib import Path
import re
import sys
import time
from typing import Callable, Optional, Tuple


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_ROOT = (
    WORKSPACE_ROOT
    / "runtime"
    / "targeted_navigation"
    / "portal_retry_single_door"
    / "logs"
)
TIMESTAMP_PATTERN = re.compile(r"^\d{8}_\d{6}$")
PROCESS_NAME = "portal_retry_single_door"

SCRIPTS = WORKSPACE_ROOT / "src" / "lste_topo_access" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_directional_branch_coverage import (  # noqa: E402
    BRANCH_TRANSIT,
    DirectionalBranchCoverage,
    DirectionalBranchKey,
    EVIDENCE_DESTINATION_VIEW,
    EVIDENCE_SOURCE_VIEW,
)
from global_frontier_portal_transaction import (  # noqa: E402
    PORTAL_TX_PLACE_COMMIT,
    PORTAL_TX_THROAT,
    PortalTransaction,
)


class ReplayInvariantError(RuntimeError):
    """Raised when the replay violates the branch lifecycle contract."""


@dataclass(frozen=True)
class ReplayResult:
    """Machine-readable result of one replay invocation."""

    run_timestamp: str
    log_directory: str
    log_file: str
    branch_id: str
    work_item_id: str
    attempts: int
    events: Tuple[str, ...]
    passed: bool

    def as_dict(self):
        return {
            "run_timestamp": self.run_timestamp,
            "log_directory": self.log_directory,
            "log_file": self.log_file,
            "branch_id": self.branch_id,
            "work_item_id": self.work_item_id,
            "attempts": int(self.attempts),
            "events": list(self.events),
            "passed": bool(self.passed),
        }


class ReplayLogger:
    """Write structured key/value log lines for one simulated process."""

    def __init__(self, path: Path, clock: Optional[Callable[[], str]] = None):
        self.path = Path(path)
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
    """Create a collision-free timestamp directory with no suffixes."""
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
            # The required directory format has one-second precision. Waiting
            # for the next second keeps names deterministic and readable.
            time.sleep(1.0)


def _pause(step_delay):
    if step_delay:
        time.sleep(float(step_delay))


def _require(condition, message):
    if not condition:
        raise ReplayInvariantError(message)


def replay(log_path: Path, run_timestamp: str, step_delay=0.0):
    """Execute the fixed event trace and return its auditable result."""
    source_place_id = 1
    destination_place_id = 2
    key = DirectionalBranchKey(source_place_id, "portal-1")
    ledger = DirectionalBranchCoverage()
    portal_transaction = PortalTransaction()
    logger = ReplayLogger(log_path)

    try:
        logger.event(
            "INFO",
            "run_start",
            scenario_id="portal_retry_single_door",
            run_timestamp=run_timestamp,
            source_place_id=source_place_id,
            destination_place_id=destination_place_id,
            physical_branch_id=key.physical_branch_id,
            branch_id=key.stable_id,
            retry_policy="one_retry_same_branch",
            ros_enabled=False,
            controller_enabled=False,
        )
        _pause(step_delay)

        transaction = portal_transaction.start(
            101,
            portal_id=1,
            source_place_id=source_place_id,
            gate_xy=(2.0, 0.0),
            destination_xy=(3.0, 0.0),
        )
        _require(transaction is not None, "portal transaction did not start")
        source_transaction = portal_transaction.observe_source_side(
            -1.0,
            route_id=101,
            reason="source_side_sensor_evidence",
        )
        _require(
            source_transaction is not None
            and source_transaction.source_side_proven,
            "portal source-side evidence was not retained",
        )

        first = ledger.observe_frontier(
            key,
            (2.0, 0.0),
            direction_xy=(1.0, 0.0),
            map_epoch=1,
            now=1.0,
        )
        source = ledger.record_evidence(
            key,
            EVIDENCE_SOURCE_VIEW,
            source="source_sensor_view",
            event_id="source-view-1",
            now=1.1,
        )
        _require(source.work_item is not None, "source evidence lost its WorkItem")
        work_item_id = source.work_item.id
        logger.event(
            "INFO",
            "source_evidence",
            branch_id=key.stable_id,
            evidence=EVIDENCE_SOURCE_VIEW,
            state=source.state,
            work_item_id=work_item_id,
            frontier_xy=first.frontier_xy,
            map_epoch=first.map_epoch,
            portal_transaction_state=source_transaction.state,
            source_side_proven=source_transaction.source_side_proven,
        )
        _pause(step_delay)

        logger.event(
            "INFO",
            "route_dispatched",
            attempt=1,
            route_id="portal-route-1-attempt-1",
            transaction_route_id=101,
            branch_id=key.stable_id,
            work_item_id=work_item_id,
        )
        _pause(step_delay)
        throat = portal_transaction.gate_reached(
            101, reason="simulated_failure_at_doorway"
        )
        _require(
            throat is not None and throat.state == PORTAL_TX_THROAT,
            "first route did not reach the portal throat",
        )
        failed = ledger.get(key)
        _require(failed is not None, "route failure lost the branch record")
        _require(
            failed.work_item is not None and failed.work_item.id == work_item_id,
            "route failure changed the WorkItem lineage",
        )
        logger.event(
            "WARN",
            "route_failed",
            attempt=1,
            route_id="portal-route-1-attempt-1",
            reason="simulated_route_failure",
            branch_id=key.stable_id,
            work_item_id=work_item_id,
            work_item_preserved=True,
            portal_transaction_state=throat.state,
            source_side_proven=throat.source_side_proven,
        )
        _pause(step_delay)

        retried = portal_transaction.retry("simulated_route_failure")
        _require(
            retried is not None and retried.source_side_proven,
            "portal retry discarded source-side evidence",
        )
        rebound = portal_transaction.bind_route(
            102, reason="portal_retry_after_doorway_failure"
        )
        _require(rebound is not None and rebound.route_id == 102, "portal retry was not rebound")

        second = ledger.observe_frontier(
            key,
            (-3.5, 0.75),
            direction_xy=(0.99, 0.05),
            map_epoch=2,
            now=2.0,
        )
        retry_item = ledger.work_item_for(key)
        _require(retry_item is not None, "retry has no unresolved WorkItem")
        _require(
            retry_item.id == work_item_id,
            "SLAM projection change minted a second WorkItem",
        )
        logger.event(
            "INFO",
            "route_retry",
            attempt=2,
            route_id="portal-route-1-attempt-2",
            branch_id=key.stable_id,
            work_item_id=retry_item.id,
            frontier_xy=second.frontier_xy,
            map_epoch=second.map_epoch,
            projection_changed=True,
            same_work_item=True,
            transaction_route_id=rebound.route_id,
            source_side_proven=rebound.source_side_proven,
            source_signed_distance=rebound.source_signed_distance,
        )
        _pause(step_delay)

        destination_standoff = portal_transaction.destination_standoff(
            102, reason="destination_side_sensor_view"
        )
        _require(
            destination_standoff is not None,
            "portal retry did not reach the destination standoff",
        )
        committed_transaction = portal_transaction.crossing_verified(
            102, reason="crossing_odometry_verified"
        )
        _require(
            committed_transaction is not None,
            "portal crossing evidence was not accepted",
        )
        committed_transaction = portal_transaction.place_commit(
            reason="destination_place_committed",
            route_id=102,
            portal_id=1,
            source_place_id=source_place_id,
        )
        _require(
            committed_transaction is not None
            and committed_transaction.state == PORTAL_TX_PLACE_COMMIT,
            "destination Place commit was not authorized",
        )

        destination = ledger.record_evidence(
            key,
            EVIDENCE_DESTINATION_VIEW,
            source="destination_sensor_view",
            event_id="destination-view-1",
            resolves=True,
            now=2.1,
        )
        _require(destination.work_item is None, "destination evidence left a WorkItem active")
        logger.event(
            "INFO",
            "destination_evidence",
            branch_id=key.stable_id,
            evidence=EVIDENCE_DESTINATION_VIEW,
            state=destination.state,
            work_item_id=work_item_id,
            work_item_closed=True,
            portal_transaction_state=committed_transaction.state,
        )
        _pause(step_delay)

        crossing = ledger.mark_transit(
            key,
            destination_place_id,
            source="crossing_odometry",
            event_id="crossing-1",
            now=2.2,
        )
        _require(crossing.state == BRANCH_TRANSIT, "crossing did not enter transit state")
        _require(crossing.work_item is None, "transit branch recreated a WorkItem")
        _require(crossing.destination_place_id == destination_place_id, "wrong transit destination")
        logger.event(
            "INFO",
            "crossing_accepted",
            branch_id=key.stable_id,
            destination_place_id=destination_place_id,
            state=crossing.state,
            transit=True,
            work_item_id=work_item_id,
            work_item_closed=True,
            portal_transaction_state=committed_transaction.state,
        )
        _pause(step_delay)

        finished_transaction = portal_transaction.finish("replay_finished")
        _require(
            finished_transaction is not None and finished_transaction.state == "idle",
            "portal transaction did not return to idle",
        )

        final = ledger.get(key)
        _require(final is not None and final.state == BRANCH_TRANSIT, "final branch state is not transit")
        _require(ledger.pending_work_items() == (), "completed branch remains pending")
        logger.event(
            "INFO",
            "run_complete",
            status="passed",
            branch_id=key.stable_id,
            attempts=2,
            pending_work_items=0,
        )
        return ReplayResult(
            run_timestamp=run_timestamp,
            log_directory=str(log_path.parent),
            log_file=str(log_path),
            branch_id=key.stable_id,
            work_item_id=work_item_id,
            attempts=2,
            events=tuple(logger.events),
            passed=True,
        )
    except Exception as error:
        logger.event(
            "ERROR",
            "run_complete",
            status="failed",
            error_type=type(error).__name__,
            error=str(error),
        )
        raise
    finally:
        logger.close()


def build_parser():
    parser = argparse.ArgumentParser(
        description="Replay one portal source/fail/retry/destination/crossing trace."
    )
    parser.add_argument(
        "command",
        nargs="?",
        choices=("run",),
        default="run",
        help="replay command (default: run)",
    )
    parser.add_argument(
        "--log-root",
        type=Path,
        default=DEFAULT_LOG_ROOT,
        help="parent directory for timestamped run logs",
    )
    parser.add_argument(
        "--timestamp",
        help="explicit YYYYMMDD_HHMMSS directory name (mainly for tests)",
    )
    parser.add_argument(
        "--step-delay",
        type=float,
        default=0.0,
        help="optional real-time delay in seconds between replay events",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.step_delay < 0.0:
        raise SystemExit("--step-delay must be non-negative")
    try:
        timestamp, run_directory = _new_run_directory(
            args.log_root, requested_timestamp=args.timestamp,
        )
        log_path = run_directory / ("%s_%s.log" % (timestamp, PROCESS_NAME))
        result = replay(log_path, timestamp, step_delay=args.step_delay)
    except Exception as error:
        # Directory creation errors happen before a process log exists, so the
        # CLI reports them on stderr and uses a non-zero status.
        raise SystemExit("portal_retry_single_door failed: %s" % error)
    print(json.dumps(result.as_dict(), ensure_ascii=True, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
