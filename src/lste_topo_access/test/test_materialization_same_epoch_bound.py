"""A materialization retry budget must also bound repeats on one snapshot.

Runs ``20261031_225814`` and ``20261031_235749`` both ended in an explorer
deadlock: the graph selected a ``semantic_room_local_work_only`` plan, the
projection missed the current frontier snapshot, and the selector parked the
cycle in ``GRAPH_ROUTE_FSM_WAITING`` with ``graph_route_materialization_pending``
set.  The wait is documented as "let the next planning wake observe a new
snapshot", but the budget that was supposed to break the loop counted only
*distinct* map epochs (``miss_count`` increments once per new
``(obligation, epoch)`` observation).  A stationary robot keeps publishing the
same epoch, so the counter stayed at 1 for 180 s and the Place never left
``local_work_pending_before_crossing``.

The budget now also counts repeated attempts, so the obligation is retired
after ``_GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES`` projections of the *same*
snapshot instead of waiting for a map update that never arrives.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        loginfo=lambda *_args, **_kwargs: None,
        logwarn=lambda *_args, **_kwargs: None,
    ),
)

from global_frontier_graph_route_adapter import (  # noqa: E402
    GlobalFrontierGraphRouteAdapterMixin,
    _GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES,
)
from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_CROSS_PORTAL,
    GraphRoutePlan,
    PLAN_BLOCKED,
    PLAN_READY,
)


class Fixture(GlobalFrontierGraphRouteAdapterMixin):
    def __init__(self):
        self.events = []
        self.invalidated = []
        self.last_graph_snapshot_epoch = 7

    def publish_status(self, event, **fields):
        self.events.append((event, fields))

    def _transition_graph_route_fsm(self, state, **fields):
        self.last_fsm = (state, fields)

    def _invalidate_graph_route_for_materialization(
        self, plan, epoch, reason, miss_count, epoch_span
    ):
        self.invalidated.append(
            (reason, int(miss_count), int(epoch_span), int(epoch))
        )
        return True


def plan(status=PLAN_READY):
    return GraphRoutePlan(
        status,
        ACTION_CROSS_PORTAL,
        current_place_id=2,
        target_place_id=3,
        obligation_kind="portal_edge",
        obligation_id=11,
        portal_path=(11,),
        first_portal_id=11,
        reason="semantic_room_local_work_only",
    )


class SameEpochMaterializationBudgetTest(unittest.TestCase):
    def test_repeat_misses_on_one_epoch_exhaust_the_budget(self):
        fixture = Fixture()
        route = plan()
        results = [
            fixture._record_graph_route_materialization_miss(route, 7)
            for _ in range(_GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES)
        ]
        self.assertEqual(
            results,
            [False] * (_GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES - 1) + [True],
            results,
        )
        self.assertEqual(
            [reason for reason, *_ in fixture.invalidated],
            ["graph_route_materialization_expired"],
        )

    def test_one_epoch_is_still_one_epoch_for_the_diagnostic(self):
        fixture = Fixture()
        route = plan()
        for _ in range(3):
            fixture._record_graph_route_materialization_miss(route, 7)
        self.assertEqual(fixture.graph_route_materialization_miss_count, 1)
        self.assertEqual(fixture.graph_route_materialization_attempts, 3)

    def test_a_new_epoch_still_counts_as_a_miss(self):
        fixture = Fixture()
        route = plan()
        fixture._record_graph_route_materialization_miss(route, 7)
        fixture._record_graph_route_materialization_miss(route, 8)
        self.assertEqual(fixture.graph_route_materialization_miss_count, 2)
        self.assertEqual(fixture.graph_route_materialization_attempts, 2)

    def test_success_resets_the_attempt_budget(self):
        fixture = Fixture()
        route = plan()
        for _ in range(3):
            fixture._record_graph_route_materialization_miss(route, 7)
        fixture._reset_graph_materialization_tracking()
        self.assertEqual(
            fixture._record_graph_route_materialization_miss(route, 7), False
        )
        self.assertEqual(fixture.graph_route_materialization_attempts, 1)

    def test_blocked_plan_also_consumes_the_budget(self):
        # A blocked plan (e.g. dfs_execution_failed_return_obligation_preserved)
        # cannot materialize either; without a budget it holds the decision
        # forever.
        fixture = Fixture()
        route = plan(PLAN_BLOCKED)
        results = [
            fixture._record_graph_route_materialization_miss(
                route, 7, "graph_route_blocked:dfs_return_obligation",
            )
            for _ in range(_GRAPH_ROUTE_MATERIALIZATION_MAX_MISSES)
        ]
        self.assertEqual(results[-1], True, results)
        self.assertEqual(
            [reason for reason, *_ in fixture.invalidated],
            ["graph_route_materialization_expired"],
        )

    def test_published_miss_carries_the_attempt_count(self):
        fixture = Fixture()
        fixture._record_graph_route_materialization_miss(plan(), 7)
        miss_events = [
            fields for event, fields in fixture.events
            if event == "graph_route_materialization_miss"
        ]
        self.assertEqual(len(miss_events), 1)
        self.assertEqual(miss_events[0]["attempt_count"], 1)
        self.assertEqual(miss_events[0]["miss_count"], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
