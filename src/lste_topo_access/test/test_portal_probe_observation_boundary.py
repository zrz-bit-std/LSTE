"""Regression tests for the Portal-probe observation transaction boundary."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        loginfo=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
    ),
)

from global_frontier_observation_coverage import (  # noqa: E402
    GlobalFrontierObservationCoverageMixin,
)
from global_frontier_candidate_lifecycle import (  # noqa: E402
    GlobalFrontierCandidateLifecycleMixin,
)


class CoverageStub(GlobalFrontierObservationCoverageMixin):
    def __init__(self, active_probe, active_phase="source"):
        self.target_region_claim_active = False
        self.pose_odom = None
        self.active_portal_probe_id = active_probe
        self.portal_work_phase = active_phase
        self.portal_probe_ledger = SimpleNamespace(
            get=lambda _probe_id: (
                None
                if active_probe is None
                else {"active_phase": self.portal_work_phase}
            )
        )
        self.completed_frontiers = []
        self.events = []
        self.region_memory = SimpleNamespace(
            endpoint_observed=lambda *args, **kwargs: {
                "id": 4,
                "visits": 1,
                "endpoint_observations": 1,
                "state": "open",
                "last_reason": "endpoint_observed",
            },
        )

    def settle_active_portal_probe(self, result, now, reason):
        self.events.append(("probe", result, now, reason))
        self.active_portal_probe_id = None

    def settle_active_work_item(self, now, state, reason):
        self.events.append(("work", now, state, reason))

    def frontier_is_completed(self, _x, _y):
        return False

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class CandidateLifecycleStub(GlobalFrontierCandidateLifecycleMixin):
    """Expose the generic-route boundary for a probe-owned WorkItem."""

    def __init__(self, probe):
        self.portal_probe_ledger = SimpleNamespace(
            probe_for_work_item=lambda item_id: (
                probe if int(item_id) == 21 else None
            )
        )

    @staticmethod
    def portal_observation_probe_for_frontier(*_args):
        return None


class TargetObservationLifecycleStub(CandidateLifecycleStub):
    """Expose a probe candidate that target ownership must suppress locally."""

    @staticmethod
    def portal_observation_probe_for_frontier(*_args):
        return {"id": 31, "state": "pending"}


class PortalProbeObservationBoundaryTest(unittest.TestCase):
    def test_source_arrival_does_not_resolve_observation_work(self):
        explorer = CoverageStub(active_probe=9)

        explorer.mark_frontier_observed(1.0, 2.0, now=3.0)

        self.assertEqual(explorer.events[0][:2], ("probe", "source_arrived"))
        self.assertFalse(any(event[0] == "work" for event in explorer.events))

    def test_ordinary_endpoint_still_resolves_observation_work(self):
        explorer = CoverageStub(active_probe=None)

        explorer.mark_frontier_observed(1.0, 2.0, now=3.0)

        self.assertTrue(any(event[0] == "work" for event in explorer.events))

    def test_destination_arrival_uses_observed_terminal(self):
        explorer = CoverageStub(active_probe=9, active_phase="destination")

        explorer.mark_frontier_observed(1.0, 2.0, now=3.0)

        self.assertEqual(explorer.events[0][:2], ("probe", "observed"))
        self.assertEqual(explorer.events[0][3], "destination_viewpoint_observed")
        self.assertFalse(any(event[0] == "work" for event in explorer.events))

    def test_probe_owned_work_item_cannot_become_generic_local_route(self):
        explorer = CandidateLifecycleStub({"id": 9, "state": "source_arrived"})
        context = SimpleNamespace(
            source_place_id=7,
            source_place_observed=True,
            work_item_cells={(3, 4): 21},
            work_item_details={21: {"match": "inherited"}},
        )

        result = explorer.local_observation_work_item(
            SimpleNamespace(), context, 3, 4, 0,
        )

        self.assertEqual(result, (None, None, 0, None))

    def test_target_observation_keeps_frontier_as_local_viewpoint(self):
        explorer = TargetObservationLifecycleStub(None)
        context = SimpleNamespace(
            source_place_id=7,
            source_place_observed=True,
            target_observation_work_pending=True,
            work_item_cells={(3, 4): 21},
            work_item_details={21: {"match": "inherited"}},
        )

        result = explorer.local_observation_work_item(
            SimpleNamespace(), context, 3, 4, 0,
        )

        self.assertEqual(result, (21, "inherited", 0, None))


if __name__ == "__main__":
    unittest.main()
