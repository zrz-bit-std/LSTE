#!/usr/bin/env python3
"""Replay local observation before a source-to-destination Portal crossing.

This small ROS-free experiment uses the production pure-Python ledgers and
``GraphRoutePlanner``.  A local WorkItem must be resolved first; only then can
the same Place select a Portal probe.  After source and destination evidence,
the planner promotes that physical Portal to a crossing action.
"""

from datetime import datetime
import argparse
import json
from pathlib import Path
import re
import sys
import time


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_ROOT = (
    WORKSPACE_ROOT
    / "runtime"
    / "targeted_navigation"
    / "local_work_before_crossing"
    / "logs"
)
TIMESTAMP_PATTERN = re.compile(r"^\d{8}_\d{6}$")
PROCESS_NAME = "local_work_before_crossing"

SCRIPTS = WORKSPACE_ROOT / "src" / "lste_topo_access" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_CROSS_PORTAL,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    GraphRoutePlanner,
    PLAN_READY,
)
from global_frontier_portal_probe_ledger import PortalProbeLedger  # noqa: E402
from global_frontier_portal_transaction import (  # noqa: E402
    PORTAL_TX_PLACE_COMMIT,
    PortalTransaction,
)
from global_frontier_work_items import PlaceWorkItemLedger  # noqa: E402


class ReplayInvariantError(RuntimeError):
    """Raised when the planner lifecycle is not the expected one."""


class ReplayLogger:
    """Write timestamped structured lines for one replay process."""

    def __init__(self, path):
        self.path = Path(path)
        self.stream = self.path.open("w", encoding="utf-8")
        self.events = []

    @staticmethod
    def _value(value):
        return json.dumps(value, ensure_ascii=True, separators=(",", ":"))

    def event(self, level, name, **fields):
        self.events.append(str(name))
        values = [
            datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S.%f%z"),
            "[%s]" % str(level).upper(),
            "process=%s" % PROCESS_NAME,
            "event=%s" % name,
        ]
        values.extend(
            "%s=%s" % (key, self._value(fields[key]))
            for key in sorted(fields)
        )
        self.stream.write(" ".join(values) + "\n")
        self.stream.flush()

    def close(self):
        self.stream.close()


def _new_run_directory(log_root, requested_timestamp=None):
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


def _pause(delay):
    if delay:
        time.sleep(float(delay))


def _require(condition, message):
    if not condition:
        raise ReplayInvariantError(message)


def _place(place_id, state="open", observed=True):
    return {
        "id": int(place_id),
        "state": state,
        "endpoint_observations": 1 if observed else 0,
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
        "portal_probe_phase": plan.portal_probe_phase,
        "reason": plan.reason,
    }


def replay(log_path, run_timestamp, step_delay=0.0):
    """Execute the fixed local-work/probe/crossing event trace."""
    planner = GraphRoutePlanner()
    work_ledger = PlaceWorkItemLedger(merge_radius=1.0)
    probe_ledger = PortalProbeLedger(match_radius=0.3)
    transaction = PortalTransaction()
    places = [_place(1)]
    portals = [{
        "id": 7,
        "source_place_id": 1,
        "destination_place_id": None,
        "state": "certified",
    }]
    logger = ReplayLogger(log_path)

    try:
        logger.event(
            "INFO",
            "run_start",
            scenario_id="local_work_before_crossing",
            run_timestamp=run_timestamp,
            source_place_id=1,
            portal_id=7,
            policy="local_work_then_portal_probe_then_crossing",
            ros_enabled=False,
            controller_enabled=False,
        )
        _pause(step_delay)

        # Seed one unresolved local observation task. Dispatching and then
        # ending its seed attempt leaves the WorkItem pending but executable.
        work_item_id = work_ledger.dispatch(1, (1.0, 1.0), now=1.0)
        _require(work_item_id is not None, "could not seed local WorkItem")
        work_ledger.fail_attempt(
            work_item_id,
            now=1.1,
            reason="seeded_pending_local_work",
        )
        work_items = work_ledger.snapshot()
        local_pending = planner.plan(
            1,
            places=places,
            portals=portals,
            work_items=work_items,
        )
        _require(local_pending.status == PLAN_READY, "local-work plan is not ready")
        _require(
            local_pending.action == ACTION_OBSERVE_LOCAL_WORK,
            "planner crossed Portal while local WorkItem was pending",
        )
        _require(local_pending.obligation_id == work_item_id, "wrong local WorkItem selected")
        logger.event(
            "INFO",
            "local_work_pending_before_crossing",
            planner_action=local_pending.action,
            work_item_id=work_item_id,
            portal_crossing_allowed=False,
            plan_reason=local_pending.reason,
        )
        _pause(step_delay)

        work_ledger.resolve(
            work_item_id,
            now=1.2,
            reason="local_observation_evidence",
        )
        work_items = work_ledger.snapshot()
        _require(
            any(
                item["id"] == work_item_id and item["state"] == "resolved"
                for item in work_items
            ),
            "local observation evidence did not resolve WorkItem",
        )
        logger.event(
            "INFO",
            "local_work_resolved",
            work_item_id=work_item_id,
            evidence="local_observation_evidence",
            portal_crossing_allowed=True,
        )
        _pause(step_delay)

        probe = probe_ledger.observe(
            1,
            (3.0, 0.0),
            (1.0, 0.0),
            map_gate_xy=(3.0, 0.0),
            map_epoch=1,
            now=1.3,
        )
        _require(probe is not None, "could not seed Portal probe")
        probe_id = probe["id"]
        probe_ledger.bind_portal(probe_id, 7)
        probe = probe_ledger.get(probe_id)
        source_plan = planner.plan(
            1,
            places=places,
            portals=portals,
            work_items=work_items,
            probes=[probe],
        )
        _require(source_plan.status == PLAN_READY, "source probe plan is not ready")
        _require(source_plan.action == ACTION_PROBE_PORTAL, "planner did not select Portal probe")
        _require(source_plan.obligation_id == probe_id, "wrong source probe selected")
        _require(source_plan.portal_probe_phase == "source", "wrong source probe phase")
        logger.event(
            "INFO",
            "portal_probe",
            phase="source",
            action=source_plan.action,
            probe_id=probe_id,
            portal_id=7,
            work_item_id=work_item_id,
            plan_reason=source_plan.reason,
        )
        _pause(step_delay)

        probe_ledger.start(probe_id, now=1.4, viewpoint_xy=(2.0, 0.0))
        probe_ledger.finish(
            probe_id,
            "source_arrived",
            now=1.5,
            reason="source_side_evidence",
        )
        probe = probe_ledger.get(probe_id)
        destination_plan = planner.plan(
            1,
            places=places,
            portals=portals,
            work_items=work_items,
            probes=[probe],
        )
        _require(destination_plan.action == ACTION_PROBE_PORTAL, "destination probe was skipped")
        _require(destination_plan.portal_probe_phase == "destination", "wrong destination probe phase")
        logger.event(
            "INFO",
            "portal_probe",
            phase="destination",
            action=destination_plan.action,
            probe_id=probe_id,
            portal_id=7,
            plan_reason=destination_plan.reason,
        )
        _pause(step_delay)

        probe_ledger.start_destination(
            probe_id,
            now=1.6,
            viewpoint_xy=(3.8, 0.0),
        )
        probe_ledger.finish(
            probe_id,
            "observed",
            now=1.7,
            reason="destination_view_evidence",
        )
        probe = probe_ledger.get(probe_id)
        logger.event(
            "INFO",
            "destination_evidence",
            probe_id=probe_id,
            portal_id=7,
            probe_state=probe["state"],
            evidence="destination_view_evidence",
        )
        _pause(step_delay)

        crossing_plan = planner.plan(
            1,
            places=places,
            portals=portals,
            work_items=work_items,
            probes=[probe],
            branch_first=True,
        )
        _require(crossing_plan.status == PLAN_READY, "crossing plan is not ready")
        _require(crossing_plan.action == ACTION_CROSS_PORTAL, "crossing was not promoted")
        _require(crossing_plan.first_portal_id == 7, "wrong Portal crossing selected")
        _require(crossing_plan.obligation_kind == "portal_edge", "wrong crossing obligation")
        crossing_payload = _plan_payload(crossing_plan)
        logger.event(
            "INFO",
            "crossing_selected",
            **crossing_payload,
            source_evidence=True,
            destination_evidence=True,
        )
        _pause(step_delay)

        started = transaction.start(
            77,
            portal_id=7,
            source_place_id=1,
            gate_xy=(3.0, 0.0),
            destination_xy=(5.0, 0.0),
            source_side_proven=True,
            source_signed_distance=-1.0,
        )
        _require(started is not None, "Portal transaction did not start")
        crossed = transaction.crossing_verified(77)
        _require(crossed is not None, "Portal crossing evidence was rejected")
        committed = transaction.place_commit(
            route_id=77,
            portal_id=7,
            source_place_id=1,
        )
        _require(
            committed is not None and committed.state == PORTAL_TX_PLACE_COMMIT,
            "Portal place commit was rejected",
        )
        logger.event(
            "INFO",
            "crossing_accepted",
            portal_id=7,
            source_place_id=1,
            transaction_id=committed.transaction_id,
            route_id=committed.route_id,
            transaction_state=committed.state,
            crossing_evidence=True,
            work_item_id=work_item_id,
            local_work_resolved=True,
        )
        _pause(step_delay)

        _require(work_ledger.snapshot()[0]["state"] == "resolved", "local WorkItem reopened")
        logger.event(
            "INFO",
            "run_complete",
            status="passed",
            required_order=[
                "local_work_pending_before_crossing",
                "portal_probe",
                "destination_evidence",
                "crossing_accepted",
            ],
            pending_local_work=0,
            transaction_state=transaction.snapshot().state,
        )
        return {
            "run_timestamp": run_timestamp,
            "log_directory": str(log_path.parent),
            "log_file": str(log_path),
            "work_item_id": work_item_id,
            "probe_id": probe_id,
            "crossing_plan": crossing_payload,
            "transaction_state": transaction.snapshot().state,
            "events": list(logger.events),
            "passed": True,
        }
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
        description="Replay local WorkItem before Portal probe and crossing."
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
        help="optional real-time delay in seconds between events",
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
        raise SystemExit("local_work_before_crossing failed: %s" % error)
    print(json.dumps(result, ensure_ascii=True, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
