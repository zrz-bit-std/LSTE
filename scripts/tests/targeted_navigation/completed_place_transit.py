#!/usr/bin/env python3
"""Replay transit through a completed Place in a T-shaped topology.

The replay is deliberately ROS-free.  It feeds durable Place/Portal records
to the real ``GraphRoutePlanner`` and uses ``DirectionalBranchCoverage`` to
keep branch projections separate from branch identities:

    Place 1 --portal 1--> completed Place 2 --portal 2--> unobserved Place 3

The planner returns the complete graph path ``(1, 2)`` from Place 1, but the
runtime-facing action is its first Portal.  After entering Place 2, a new map
projection and replan still treat it as transit-only.  Place 3 is the first
Place that receives an observation action.
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
    / "completed_place_transit"
    / "logs"
)
TIMESTAMP_PATTERN = re.compile(r"^\d{8}_\d{6}$")
PROCESS_NAME = "completed_place_transit"

SCRIPTS = WORKSPACE_ROOT / "src" / "lste_topo_access" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_directional_branch_coverage import (  # noqa: E402
    BRANCH_TRANSIT,
    DirectionalBranchCoverage,
    DirectionalBranchKey,
)
from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_BOOTSTRAP,
    ACTION_CROSS_PORTAL,
    GraphRoutePlanner,
    PLAN_READY,
)


class ReplayInvariantError(RuntimeError):
    """Raised when a planner or identity invariant is violated."""


@dataclass(frozen=True)
class ReplayResult:
    run_timestamp: str
    log_directory: str
    log_file: str
    initial_plan: dict
    transit_plan: dict
    destination_plan: dict
    branch_states: Dict[str, str]
    pending_work_items: int
    events: Tuple[str, ...]
    passed: bool

    def as_dict(self):
        return {
            "run_timestamp": self.run_timestamp,
            "log_directory": self.log_directory,
            "log_file": self.log_file,
            "initial_plan": dict(self.initial_plan),
            "transit_plan": dict(self.transit_plan),
            "destination_plan": dict(self.destination_plan),
            "branch_states": dict(self.branch_states),
            "pending_work_items": int(self.pending_work_items),
            "events": list(self.events),
            "passed": bool(self.passed),
        }


def _wall_timestamp():
    return datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S.%f%z")


class ReplayLogger:
    """Write one structured log for the simulated process."""

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


def _place(place_id, *, state, observed):
    return {
        "id": int(place_id),
        "state": str(state),
        "endpoint_observations": 1 if observed else 0,
    }


def _portal(portal_id, source, destination):
    return {
        "id": int(portal_id),
        "source_place_id": int(source),
        "destination_place_id": int(destination),
        "state": "crossed",
    }


def _work(item_id, place_id):
    return {
        "id": int(item_id),
        "place_id": int(place_id),
        "state": "unresolved",
        "active_attempt_id": None,
    }


def _plan_payload(plan):
    return {
        "status": plan.status,
        "action": plan.action,
        "current_place_id": plan.current_place_id,
        "target_place_id": plan.target_place_id,
        "obligation_kind": plan.obligation_kind,
        "obligation_id": plan.obligation_id,
        "portal_path": list(plan.portal_path),
        "first_portal_id": plan.first_portal_id,
        "reason": plan.reason,
    }


def replay(log_path: Path, run_timestamp: str, step_delay=0.0):
    """Execute the deterministic T-topology replay."""
    places = [
        _place(1, state="open", observed=True),
        # Place 2 is physically known and complete; it remains a transit node.
        _place(2, state="dormant", observed=True),
        # Place 3 has not received an observation yet.
        _place(3, state="open", observed=False),
    ]
    portals = [_portal(1, 1, 2), _portal(2, 2, 3)]
    place3_work = [_work(31, 3)]
    planner = GraphRoutePlanner()
    branch_coverage = DirectionalBranchCoverage()
    branch_a = DirectionalBranchKey(1, "portal:1")
    branch_b = DirectionalBranchKey(2, "portal:2")
    logger = ReplayLogger(log_path)

    try:
        logger.event(
            "INFO",
            "run_start",
            scenario_id="completed_place_transit",
            run_timestamp=run_timestamp,
            topology="place_1->portal_1->place_2->portal_2->place_3",
            source_place_id=1,
            completed_transit_place_id=2,
            unobserved_destination_place_id=3,
            ros_enabled=False,
            controller_enabled=False,
        )
        _pause(step_delay)

        # Durable branch records are created independently. Their map
        # projections are intentionally not used as identity keys.
        for name, key, point in (
            ("a", branch_a, (2.0, 0.0)),
            ("b", branch_b, (2.0, 4.0)),
        ):
            snapshot = branch_coverage.observe_frontier(
                key,
                point,
                direction_xy=(1.0, 0.0),
                map_epoch=1,
                now=1.0,
            )
            logger.event(
                "INFO",
                "map_update",
                branch=name,
                branch_id=key.stable_id,
                frontier_xy=snapshot.frontier_xy,
                map_epoch=snapshot.map_epoch,
            )
            _pause(step_delay)

        # Both physical edges have already been traversed in the durable graph
        # even though Place 3 has not yet been observed by the robot.
        branch_coverage.mark_transit(
            branch_a,
            2,
            source="portal_1_crossing",
            event_id="crossing-portal-1",
            now=1.2,
        )
        branch_coverage.mark_transit(
            branch_b,
            3,
            source="portal_2_crossing",
            event_id="crossing-portal-2",
            now=1.3,
        )
        _require(
            branch_coverage.pending_work_items(source_place_id=2) == (),
            "completed Place 2 has a local branch WorkItem",
        )
        logger.event(
            "INFO",
            "branch_transit_committed",
            branch_a=branch_a.stable_id,
            branch_b=branch_b.stable_id,
            place_2_local_work_items=0,
            branch_a_state=branch_coverage.get(branch_a).state,
            branch_b_state=branch_coverage.get(branch_b).state,
        )
        _pause(step_delay)

        initial = planner.plan(
            1,
            places=places,
            portals=portals,
            work_items=place3_work,
            target_place_id=3,
            branch_coverage=branch_coverage,
        )
        _require(initial.status == PLAN_READY, "initial graph plan is not ready")
        _require(initial.action == ACTION_CROSS_PORTAL, "initial plan is not portal crossing")
        _require(initial.target_place_id == 3, "initial plan does not target Place 3")
        _require(initial.portal_path == (1, 2), "initial plan lost the two-edge path")
        _require(initial.first_portal_id == 1, "initial plan selected the wrong first Portal")
        initial_payload = _plan_payload(initial)
        logger.event(
            "INFO",
            "planner_selected",
            phase="from_place_1",
            **initial_payload,
            complete_place_2_transit_only=True,
        )
        _pause(step_delay)

        # The robot has crossed Portal 1 and is now physically in completed
        # Place 2. A new SLAM projection changes both branch coordinates.
        for name, key, point in (
            ("b", branch_b, (7.5, 4.4)),
            ("a", branch_a, (7.0, 0.2)),
        ):
            snapshot = branch_coverage.observe_frontier(
                key,
                point,
                direction_xy=(0.98, 0.05),
                map_epoch=2,
                now=2.0,
            )
            logger.event(
                "INFO",
                "map_update",
                branch=name,
                branch_id=key.stable_id,
                frontier_xy=snapshot.frontier_xy,
                map_epoch=snapshot.map_epoch,
                place_2_state="dormant",
                place_2_local_work_items=0,
            )
            _pause(step_delay)

        transit = planner.plan(
            2,
            places=places,
            portals=portals,
            work_items=place3_work,
            target_place_id=3,
            branch_coverage=branch_coverage,
        )
        _require(transit.status == PLAN_READY, "Place 2 transit plan is not ready")
        _require(transit.action == ACTION_CROSS_PORTAL, "Place 2 was treated as local exploration")
        _require(transit.target_place_id == 3, "Place 2 transit plan targets the wrong Place")
        _require(transit.portal_path == (2,), "Place 2 transit selected the wrong Portal")
        _require(transit.first_portal_id == 2, "Place 2 transit selected Portal 1 again")
        _require(branch_coverage.get(branch_a).state == BRANCH_TRANSIT, "Portal 1 lost transit state")
        _require(branch_coverage.pending_work_items(2) == (), "Place 2 regenerated local WorkItem")
        transit_payload = _plan_payload(transit)
        logger.event(
            "INFO",
            "planner_replanned",
            phase="from_completed_place_2",
            **transit_payload,
            place_state="dormant",
            place_transit_only=True,
            local_work_items=0,
            portal_1_not_reselected=True,
        )
        _pause(step_delay)

        destination = planner.plan(
            3,
            places=places,
            portals=portals,
            work_items=place3_work,
            target_place_id=3,
            branch_coverage=branch_coverage,
        )
        _require(destination.status == PLAN_READY, "Place 3 observation plan is not ready")
        _require(destination.action == ACTION_BOOTSTRAP, "Place 3 did not receive observation action")
        _require(destination.current_place_id == 3, "Place 3 observation has wrong current Place")
        _require(destination.obligation_id == 31, "Place 3 observation has wrong WorkItem")
        _require(destination.portal_path == (), "Place 3 observation unexpectedly crossed a Portal")
        destination_payload = _plan_payload(destination)
        logger.event(
            "INFO",
            "planner_selected",
            phase="from_unobserved_place_3",
            **destination_payload,
            observation_action=True,
            place_2_reentry=False,
        )
        _pause(step_delay)

        final_a = branch_coverage.get(branch_a)
        final_b = branch_coverage.get(branch_b)
        _require(final_a is not None and final_b is not None, "final branch state missing")
        _require(final_a.state == BRANCH_TRANSIT, "Portal 1 is not final transit")
        _require(final_b.state == BRANCH_TRANSIT, "Portal 2 is not final transit")
        _require(final_a.work_item is None and final_b.work_item is None, "transit branch has WorkItem")
        _require(branch_coverage.pending_work_items(2) == (), "completed Place 2 has pending WorkItem")
        logger.event(
            "INFO",
            "run_complete",
            status="passed",
            place_2_state="dormant",
            place_2_reentry=False,
            place_2_local_work_items=0,
            place_3_observation_action=destination.action,
            branch_a_state=final_a.state,
            branch_b_state=final_b.state,
            pending_work_items=len(branch_coverage.pending_work_items()),
        )
        return ReplayResult(
            run_timestamp=run_timestamp,
            log_directory=str(log_path.parent),
            log_file=str(log_path),
            initial_plan=initial_payload,
            transit_plan=transit_payload,
            destination_plan=destination_payload,
            branch_states={"a": final_a.state, "b": final_b.state},
            pending_work_items=len(branch_coverage.pending_work_items()),
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
        description="Replay transit through a completed Place in a T-shaped graph."
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
        raise SystemExit("completed_place_transit failed: %s" % error)
    print(json.dumps(result.as_dict(), ensure_ascii=True, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
