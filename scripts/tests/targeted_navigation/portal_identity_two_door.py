#!/usr/bin/env python3
"""Replay two physical doors from one Place without ROS.

The scenario is deliberately small but exercises the identity boundary that
is easy to lose in an online SLAM loop:

    door A map update -> door B map update -> source evidence for both
    -> A route failure -> A retry -> interleaved map updates
    -> A destination evidence -> A crossing accepted

Each door owns a durable directional branch, an ObservationWorkItem, and a
PortalTransaction.  Frontier coordinates are transient projections only.  The
replay proves that a retry on A cannot modify B, and that a completed branch
can remain a transit edge without producing another WorkItem.
"""

from dataclasses import dataclass
from datetime import datetime
import argparse
import json
from pathlib import Path
import re
import sys
import time
from typing import Callable, Dict, Optional, Tuple


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_ROOT = (
    WORKSPACE_ROOT
    / "runtime"
    / "targeted_navigation"
    / "portal_identity_two_door"
    / "logs"
)
TIMESTAMP_PATTERN = re.compile(r"^\d{8}_\d{6}$")
PROCESS_NAME = "portal_identity_two_door"

SCRIPTS = WORKSPACE_ROOT / "src" / "lste_topo_access" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_directional_branch_coverage import (  # noqa: E402
    BRANCH_OPEN,
    BRANCH_TRANSIT,
    DirectionalBranchCoverage,
    DirectionalBranchKey,
    EVIDENCE_DESTINATION_VIEW,
    EVIDENCE_SOURCE_VIEW,
)
from global_frontier_portal_transaction import (  # noqa: E402
    PORTAL_TX_PLACE_COMMIT,
    PORTAL_TX_SOURCE_PROBE,
    PortalTransaction,
)


class ReplayInvariantError(RuntimeError):
    """Raised when the two-door identity contract is violated."""


@dataclass(frozen=True)
class ReplayResult:
    run_timestamp: str
    log_directory: str
    log_file: str
    branch_ids: Dict[str, str]
    work_item_ids: Dict[str, str]
    transaction_ids: Dict[str, int]
    transaction_portals: Dict[str, int]
    transaction_states: Dict[str, str]
    events: Tuple[str, ...]
    passed: bool

    def as_dict(self):
        return {
            "run_timestamp": self.run_timestamp,
            "log_directory": self.log_directory,
            "log_file": self.log_file,
            "branch_ids": dict(self.branch_ids),
            "work_item_ids": dict(self.work_item_ids),
            "transaction_ids": dict(self.transaction_ids),
            "transaction_portals": dict(self.transaction_portals),
            "transaction_states": dict(self.transaction_states),
            "events": list(self.events),
            "passed": bool(self.passed),
        }


def _wall_timestamp():
    return datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S.%f%z")


class ReplayLogger:
    """Write one structured log for this simulated process."""

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


def _new_run_directory(log_root: Path, requested_timestamp=None):
    """Create one YYYYMMDD_HHMMSS directory for this invocation."""
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


def replay(log_path: Path, run_timestamp: str, step_delay=0.0):
    """Execute the deterministic two-door trace."""
    source_place_id = 1
    destination_place_ids = {"a": 2, "b": 3}
    keys = {
        "a": DirectionalBranchKey(source_place_id, "portal-a"),
        "b": DirectionalBranchKey(source_place_id, "portal-b"),
    }
    ledger = DirectionalBranchCoverage()
    transactions = {"a": PortalTransaction(), "b": PortalTransaction()}
    portal_ids = {"a": 501, "b": 502}
    initial_routes = {"a": 101, "b": 201}
    retry_route_a = 102
    logger = ReplayLogger(log_path)

    try:
        logger.event(
            "INFO",
            "run_start",
            scenario_id="portal_identity_two_door",
            run_timestamp=run_timestamp,
            source_place_id=source_place_id,
            branch_ids={name: key.stable_id for name, key in keys.items()},
            physical_portals=portal_ids,
            ros_enabled=False,
            controller_enabled=False,
        )
        _pause(step_delay)

        # First map epoch: both physical doors are discovered independently.
        projections = {
            "a": ((2.0, 0.0), (1.0, 0.0), 1),
            "b": ((2.0, 4.0), (1.0, 0.0), 1),
        }
        for name in ("a", "b"):
            point, direction, epoch = projections[name]
            snapshot = ledger.observe_frontier(
                keys[name],
                point,
                direction_xy=direction,
                map_epoch=epoch,
                now=1.0,
            )
            _require(snapshot.work_item is not None, "door %s has no WorkItem" % name)
            logger.event(
                "INFO",
                "map_update",
                door=name,
                branch_id=keys[name].stable_id,
                frontier_xy=snapshot.frontier_xy,
                map_epoch=snapshot.map_epoch,
                work_item_id=snapshot.work_item.id,
            )
            _pause(step_delay)

        work_items = {
            name: ledger.work_item_for(keys[name]) for name in ("a", "b")
        }
        _require(work_items["a"] is not None, "door A WorkItem missing")
        _require(work_items["b"] is not None, "door B WorkItem missing")
        _require(
            work_items["a"].id != work_items["b"].id,
            "two physical doors share a WorkItem ID",
        )
        work_item_ids = {
            name: work_items[name].id for name in ("a", "b")
        }

        # Source views are evidence for their own branch only.
        for name in ("a", "b"):
            source = ledger.record_evidence(
                keys[name],
                EVIDENCE_SOURCE_VIEW,
                source="source_view_%s" % name,
                event_id="source-view-%s" % name,
                now=1.1,
            )
            _require(
                source.state == BRANCH_OPEN and source.work_item is not None,
                "source evidence closed or lost door %s WorkItem" % name,
            )
            logger.event(
                "INFO",
                "source_evidence",
                door=name,
                branch_id=keys[name].stable_id,
                evidence=EVIDENCE_SOURCE_VIEW,
                state=source.state,
                work_item_id=source.work_item.id,
            )
            _pause(step_delay)

        # Each transaction starts with a different route and physical portal.
        for name in ("a", "b"):
            transaction = transactions[name]
            started = transaction.start(
                initial_routes[name],
                portal_id=portal_ids[name],
                source_place_id=source_place_id,
                gate_xy=projections[name][0],
                destination_xy=(5.0, projections[name][0][1]),
                source_side_proven=True,
                source_signed_distance=-1.0,
            )
            _require(started is not None, "door %s transaction did not start" % name)
            _require(
                started.portal_id == portal_ids[name]
                and started.source_place_id == source_place_id,
                "door %s transaction identity is wrong" % name,
            )
            logger.event(
                "INFO",
                "transaction_started",
                door=name,
                transaction_id=started.transaction_id,
                route_id=started.route_id,
                portal_id=started.portal_id,
                source_place_id=started.source_place_id,
            )
            _pause(step_delay)

        transaction_ids = {
            name: transactions[name].snapshot().transaction_id for name in ("a", "b")
        }

        # Interleave the second map epoch before A fails.  The coordinates move,
        # but neither transaction identity nor either WorkItem can change.
        for name, point in (("a", (2.6, 0.35)), ("b", (2.4, 4.3))):
            snapshot = ledger.observe_frontier(
                keys[name],
                point,
                direction_xy=(1.0, 0.02),
                map_epoch=2,
                now=2.0,
            )
            _require(
                snapshot.work_item is not None
                and snapshot.work_item.id == work_item_ids[name],
                "map update changed door %s WorkItem" % name,
            )
            logger.event(
                "INFO",
                "map_update",
                door=name,
                branch_id=keys[name].stable_id,
                frontier_xy=snapshot.frontier_xy,
                map_epoch=snapshot.map_epoch,
                work_item_id=snapshot.work_item.id,
            )
            _pause(step_delay)

        # Only A encounters a route failure and is rebound to one fresh route.
        gate = transactions["a"].gate_reached(initial_routes["a"])
        _require(gate is not None, "door A did not reach its gate")
        logger.event(
            "WARN",
            "route_failed",
            door="a",
            transaction_id=gate.transaction_id,
            route_id=initial_routes["a"],
            portal_id=portal_ids["a"],
            reason="simulated_route_failure",
        )
        _pause(step_delay)

        retried = transactions["a"].retry("same_physical_door_retry")
        _require(retried is not None, "door A retry was rejected")
        rebound = transactions["a"].bind_route(retry_route_a)
        _require(rebound is not None, "door A route was not rebound")
        _require(
            rebound.transaction_id == transaction_ids["a"]
            and rebound.portal_id == portal_ids["a"]
            and rebound.source_place_id == source_place_id
            and rebound.retry_count == 1
            and rebound.route_id == retry_route_a,
            "door A retry changed physical transaction identity",
        )
        b_after_retry = transactions["b"].snapshot()
        _require(
            b_after_retry.transaction_id == transaction_ids["b"]
            and b_after_retry.portal_id == portal_ids["b"]
            and b_after_retry.route_id == initial_routes["b"]
            and b_after_retry.retry_count == 0,
            "door A retry modified door B transaction",
        )
        logger.event(
            "INFO",
            "route_retry",
            door="a",
            transaction_id=rebound.transaction_id,
            previous_route_id=initial_routes["a"],
            route_id=rebound.route_id,
            portal_id=rebound.portal_id,
            retry_count=rebound.retry_count,
            branch_id=keys["a"].stable_id,
            work_item_id=work_item_ids["a"],
            same_transaction=True,
        )
        _pause(step_delay)

        # A and B receive another update in the opposite order after retry.
        for name, point in (("b", (3.1, 4.5)), ("a", (3.0, 0.5))):
            snapshot = ledger.observe_frontier(
                keys[name],
                point,
                direction_xy=(0.99, 0.04),
                map_epoch=3,
                now=3.0,
            )
            _require(
                snapshot.work_item is not None
                and snapshot.work_item.id == work_item_ids[name],
                "interleaved update changed door %s WorkItem" % name,
            )
            logger.event(
                "INFO",
                "map_update",
                door=name,
                branch_id=keys[name].stable_id,
                frontier_xy=snapshot.frontier_xy,
                map_epoch=snapshot.map_epoch,
                work_item_id=snapshot.work_item.id,
            )
            _pause(step_delay)

        destination = ledger.complete_branch(
            keys["a"],
            evidence_kind=EVIDENCE_DESTINATION_VIEW,
            source="destination_view_a",
            event_id="destination-view-a",
            now=3.1,
        )
        _require(
            destination.state != BRANCH_OPEN and destination.work_item is None,
            "destination evidence did not close door A WorkItem",
        )
        _require(
            ledger.work_item_for(keys["b"]) is not None
            and ledger.work_item_for(keys["b"]).id == work_item_ids["b"],
            "door A destination evidence changed door B WorkItem",
        )
        logger.event(
            "INFO",
            "destination_evidence",
            door="a",
            branch_id=keys["a"].stable_id,
            evidence=EVIDENCE_DESTINATION_VIEW,
            state=destination.state,
            work_item_id=work_item_ids["a"],
            work_item_closed=True,
        )
        _pause(step_delay)

        # Finish the PortalTransaction for A with identity checks. B is still
        # in its own source probe and must not be committed as a side effect.
        transactions["a"].gate_reached(retry_route_a)
        transactions["a"].destination_standoff(retry_route_a)
        crossed_tx = transactions["a"].crossing_verified(retry_route_a)
        _require(crossed_tx is not None, "door A crossing evidence was rejected")
        committed_tx = transactions["a"].place_commit(
            route_id=retry_route_a,
            portal_id=portal_ids["a"],
            source_place_id=source_place_id,
        )
        _require(
            committed_tx is not None and committed_tx.state == PORTAL_TX_PLACE_COMMIT,
            "door A place commit was rejected",
        )
        crossing = ledger.mark_transit(
            keys["a"],
            destination_place_ids["a"],
            source="crossing_odometry_a",
            event_id="crossing-a",
            now=3.2,
        )
        _require(crossing.state == BRANCH_TRANSIT, "door A is not transit")
        _require(crossing.work_item is None, "transit door A recreated a WorkItem")
        _require(
            transactions["b"].snapshot().state == PORTAL_TX_SOURCE_PROBE,
            "door A crossing changed door B transaction state",
        )
        _require(
            transactions["b"].snapshot().portal_id == portal_ids["b"],
            "door A crossing changed door B portal identity",
        )
        logger.event(
            "INFO",
            "crossing_accepted",
            door="a",
            branch_id=keys["a"].stable_id,
            transaction_id=committed_tx.transaction_id,
            route_id=committed_tx.route_id,
            portal_id=committed_tx.portal_id,
            transaction_state=committed_tx.state,
            destination_place_id=crossing.destination_place_id,
            state=crossing.state,
            transit=True,
            work_item_id=work_item_ids["a"],
            work_item_closed=True,
            door_b_unchanged=True,
        )
        _pause(step_delay)

        # A later SLAM projection for the completed door remains harmless. B's
        # projection can move independently and still owns its original task.
        for name, point in (("a", (30.0, -5.0)), ("b", (-4.0, 18.0))):
            snapshot = ledger.observe_frontier(
                keys[name],
                point,
                direction_xy=(1.0, 0.0),
                map_epoch=4,
                now=4.0,
            )
            if name == "a":
                _require(
                    snapshot.state == BRANCH_TRANSIT
                    and snapshot.work_item is None
                    and ledger.allows_transit(keys[name]),
                    "completed door A became a new observation task",
                )
            else:
                _require(
                    snapshot.state == BRANCH_OPEN
                    and snapshot.work_item is not None
                    and snapshot.work_item.id == work_item_ids[name],
                    "door B lost its independent WorkItem",
                )
            logger.event(
                "INFO",
                "map_update",
                door=name,
                branch_id=keys[name].stable_id,
                frontier_xy=snapshot.frontier_xy,
                map_epoch=snapshot.map_epoch,
                work_item_id=work_item_ids[name],
                work_item_active=snapshot.work_item is not None,
                transit=snapshot.transit,
            )
            _pause(step_delay)

        final_a = ledger.get(keys["a"])
        final_b = ledger.get(keys["b"])
        _require(final_a is not None and final_b is not None, "final branch records missing")
        _require(final_a.state == BRANCH_TRANSIT, "final A state is not transit")
        _require(final_a.work_item is None, "final A WorkItem is active")
        _require(final_b.state == BRANCH_OPEN, "final B state changed unexpectedly")
        _require(
            final_b.work_item is not None and final_b.work_item.id == work_item_ids["b"],
            "final B WorkItem lineage changed",
        )
        logger.event(
            "INFO",
            "run_complete",
            status="passed",
            branch_a_state=final_a.state,
            branch_b_state=final_b.state,
            branch_a_work_item_active=False,
            branch_b_work_item_active=True,
            pending_work_items=1,
        )
        return ReplayResult(
            run_timestamp=run_timestamp,
            log_directory=str(log_path.parent),
            log_file=str(log_path),
            branch_ids={name: keys[name].stable_id for name in ("a", "b")},
            work_item_ids=work_item_ids,
            transaction_ids=transaction_ids,
            transaction_portals=portal_ids,
            transaction_states={
                name: transactions[name].snapshot().state for name in ("a", "b")
            },
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
        description="Replay identity isolation for two physical doors in one Place."
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
            args.log_root,
            requested_timestamp=args.timestamp,
        )
        log_path = run_directory / ("%s_%s.log" % (timestamp, PROCESS_NAME))
        result = replay(log_path, timestamp, step_delay=args.step_delay)
    except Exception as error:
        raise SystemExit("portal_identity_two_door failed: %s" % error)
    print(json.dumps(result.as_dict(), ensure_ascii=True, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
