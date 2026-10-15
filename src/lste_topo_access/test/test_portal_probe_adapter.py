"""Tests for the ROS-boundary adapter around PortalProbeValue selection."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_probe_adapter import (  # noqa: E402
    make_probe_candidate,
    select_probe_route,
    selection_report,
)
from global_frontier_portal_probes import PortalObservationProbe  # noqa: E402
from global_frontier_selection_planner import (  # noqa: E402
    GlobalFrontierSelectionPlannerMixin,
)


def route(probe, *, x, y, path, information, support, retry=False):
    """Build the stable tuple emitted by candidate scoring."""
    values = [
        2,
        3,
        x,
        y,
        path,
        information,
        4.0,
        0.0,
        None,
        0,
        "frontier_endpoint",
        None,
        8,
        "created",
        support,
        probe,
        retry,
        "probe_portal",
        "unknown_side_is_wall_bounded",
    ]
    return tuple(values)


class LedgerStub:
    def __init__(self, unavailable=()):
        self.unavailable = set(unavailable)

    def available(self, probe_id):
        return probe_id not in self.unavailable

    def viewpoint_available(self, _probe_id, _viewpoint):
        return True


class PlannerStub(GlobalFrontierSelectionPlannerMixin):
    def __init__(self):
        self.task_semantic_value_enabled = True
        self.portal_probe_ledger = LedgerStub()
        self.events = []

    def frontier_is_rejected(self, _x, _y, _now):
        return False

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class PortalProbeAdapterTest(unittest.TestCase):
    def setUp(self):
        self.request = SimpleNamespace(
            map_to_physical_xy=lambda x, y: (x + 10.0, y - 4.0),
        )
        self.context = SimpleNamespace(source_place_id=7)

    def test_adapter_preserves_named_probe_identity_and_constraints(self):
        probe = PortalObservationProbe((5, 6), (1, 0), 11)
        candidate = make_probe_candidate(
            route(probe, x=2.0, y=3.0, path=4.0, information=9.0, support=5),
            "revisit",
            "probe",
            self.request,
            self.context,
            ledger=LedgerStub(),
        )

        self.assertEqual(candidate.candidate_id, 11)
        self.assertTrue(candidate.constraints.physical_identity_valid)
        self.assertTrue(candidate.constraints.probe_available)
        self.assertEqual(candidate.value.information_gain, 9.0)
        self.assertEqual(candidate.value.novelty, 5.0)

    def test_rejected_viewpoint_is_not_allowed_to_win_by_information(self):
        good_probe = PortalObservationProbe((1, 1), (1, 0), 3)
        rejected_probe = PortalObservationProbe((2, 2), (1, 0), 4)
        records = (
            (
                route(
                    rejected_probe,
                    x=2.0,
                    y=3.0,
                    path=1.0,
                    information=100.0,
                    support=100,
                ),
                "revisit",
                "probe",
            ),
            (
                route(
                    good_probe,
                    x=6.0,
                    y=3.0,
                    path=2.0,
                    information=1.0,
                    support=1,
                ),
                "revisit",
                "probe",
            ),
        )

        selected, result = select_probe_route(
            records,
            self.request,
            self.context,
            ledger=LedgerStub(),
            route_rejected=lambda x, _y: x < 3.0,
        )

        self.assertEqual(selected[15].probe_id, 3)
        report = selection_report(result)
        self.assertEqual(report["selected_candidate_id"], 3)
        self.assertEqual(report["feasible_count"], 1)
        self.assertEqual(report["rejected"][0]["candidate_id"], 4)
        self.assertIn("route_reachable", report["rejected"][0]["reasons"])

    def test_retry_category_precedes_probe_category(self):
        retry = PortalObservationProbe((1, 1), (1, 0), 5)
        ordinary = PortalObservationProbe((2, 2), (1, 0), 6)
        records = (
            (
                route(
                    ordinary,
                    x=2.0,
                    y=3.0,
                    path=1.0,
                    information=100.0,
                    support=100,
                ),
                "revisit",
                "probe",
            ),
            (
                route(
                    retry,
                    x=3.0,
                    y=3.0,
                    path=100.0,
                    information=1.0,
                    support=1,
                    retry=True,
                ),
                "revisit",
                "probe",
            ),
        )

        selected, result = select_probe_route(
            records, self.request, self.context, ledger=LedgerStub(),
        )

        self.assertEqual(selected[15].probe_id, 5)
        self.assertEqual(result.action_category, "viewpoint_retry")

    def test_unavailable_probe_is_reported_and_not_selected(self):
        probe = PortalObservationProbe((1, 1), (1, 0), 13)
        selected, result = select_probe_route(
            ((
                route(probe, x=2.0, y=3.0, path=1.0, information=2.0, support=1),
                "revisit",
                "probe",
            ),),
            self.request,
            self.context,
            ledger=LedgerStub(unavailable=(13,)),
        )

        self.assertIsNone(selected)
        self.assertEqual(result.rejected[0].reasons, ("probe_available",))

    def test_planner_uses_all_probe_records_before_score_pool_compression(self):
        first = PortalObservationProbe((1, 1), (1, 0), 21)
        second = PortalObservationProbe((2, 2), (1, 0), 22)
        records = (
            (
                route(
                    first,
                    x=2.0,
                    y=3.0,
                    path=3.0,
                    information=3.0,
                    support=1,
                ),
                "revisit",
                "probe",
            ),
            (
                route(
                    second,
                    x=3.0,
                    y=3.0,
                    path=2.0,
                    information=10.0,
                    support=4,
                ),
                "revisit",
                "probe",
            ),
        )
        planner = PlannerStub()
        request = SimpleNamespace(
            map_to_physical_xy=lambda x, y: (x, y),
            allowed_region_tiers=None,
            now=1.0,
        )

        selected = planner._select_portal_probe_candidate(
            request, self.context, records,
        )

        self.assertEqual(selected[15].probe_id, 22)
        self.assertEqual(planner.events[0][0], "portal_probe_value_selected")
        self.assertEqual(
            planner.last_portal_probe_value_selection["candidate_count"], 2,
        )


if __name__ == "__main__":
    unittest.main()
