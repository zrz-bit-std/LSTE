#!/usr/bin/env python3
"""Regression tests for SLAM-merged durable portal egress."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        logwarn=lambda *_args, **_kwargs: None,
        logwarn_throttle=lambda *_args, **_kwargs: None,
    ),
)

from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_portal_egress import GlobalFrontierPortalEgressMixin
from global_frontier_portal_probe_ledger import PortalProbeLedger


class DurableEgressExplorer(GlobalFrontierPortalEgressMixin):
    def __init__(self):
        self.current_physical_place_id = 4
        self.current_task_version = "task-v1"
        self.target_region_claim_active = False
        self.clearance = 0.52
        self.teb_xy_goal_tolerance = 0.50
        self.portal_hypothesis_ledger = PortalHypothesisLedger()
        self.last_portal_hypothesis_id = 0
        self.last_portal_covered_cycle_skips = 0
        self.region = {
            "id": 4,
            "state": "open",
            "entry_portals": [
                {
                    "gate": [2.0, 0.0],
                    "inside": [1.0, 0.0],
                    "physical_gate": [2.0, 0.0],
                    "physical_inside": [1.0, 0.0],
                }
            ],
        }
        self.region_memory = SimpleNamespace(
            by_id=lambda _place_id: self.region,
            portal_exit_from_entry=lambda *_args, **_kwargs: (
                {"id": 1, "state": "dormant"},
                {"gate": [2.0, 0.0], "inside": [1.0, 0.0]},
            ),
        )
        self.events = []

    @staticmethod
    def nearest_reachable_cell(_message, _steps, _x, _y):
        return 0, 3

    @staticmethod
    def cell_xy(_message, row, col):
        return float(col), float(row)

    @staticmethod
    def candidate_costmap_distance(_validation, _x, _y):
        return None

    @staticmethod
    def transform_xy(_target, _source, x, y):
        return float(x), float(y)

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class CoveredTransitEgressExplorer(DurableEgressExplorer):
    """Egress fixture with explicit covered Place graph state."""

    def __init__(self, with_progress):
        super().__init__()
        self.region.update({"state": "dormant", "endpoint_observations": 1})
        destination = {
            "id": 1,
            "state": "dormant",
            "endpoint_observations": 1,
        }
        branch = {
            "id": 3,
            "state": "open",
            "endpoint_observations": 0,
        }
        regions = {
            4: self.region,
            1: destination,
            3: branch,
        }
        self.region_memory = SimpleNamespace(
            by_id=lambda place_id: regions.get(int(place_id)),
            portal_exit_from_entry=lambda *_args, **_kwargs: (
                destination,
                {"gate": [2.0, 0.0], "inside": [1.0, 0.0]},
            ),
        )
        if with_progress:
            edge, _created = self.portal_hypothesis_ledger.certify(
                1, (4.0, 0.0), (4.8, 0.0), now=1.0,
            )
            self.portal_hypothesis_ledger.crossed(edge["id"], now=1.5)
            self.portal_hypothesis_ledger.bind_destination(
                edge["id"], 3, now=2.0,
            )


class DurableProbeExplorer(GlobalFrontierPortalEgressMixin):
    """Fixture for rehydrating a probe after its live frontier disappeared."""

    def __init__(self):
        self.current_physical_place_id = 7
        self.current_task_version = "task-v1"
        self.target_region_claim_active = False
        self.clearance = 0.52
        self.teb_xy_goal_tolerance = 0.50
        self.portal_probe_ledger = PortalProbeLedger(match_radius=0.35)
        self.frontier_approach_distance = 2.0
        self.completed_radius = 0.5
        self.projector_calls = []
        self.events = []

    @staticmethod
    def nearest_reachable_cell(_message, _steps, _x, _y):
        # The current map has no frontier cell for the probe anymore. The
        # route graph still offers one safe viewpoint on the source side.
        return 2, 1

    @staticmethod
    def cell_xy(_message, row, col):
        return float(col), float(row)

    @staticmethod
    def candidate_costmap_distance(_validation, _x, _y):
        return None

    @staticmethod
    def portal_probe_viewpoint_is_local(gate_xy, viewpoint_xy, resolution=0.0):
        return (
            np.hypot(
                float(gate_xy[0]) - float(viewpoint_xy[0]),
                float(gate_xy[1]) - float(viewpoint_xy[1]),
            )
            <= 2.0 + float(resolution)
        )

    def planar_xy_projector(self, target_frame, source_frame):
        self.projector_calls.append((target_frame, source_frame))
        if (target_frame, source_frame) == ("odom", "map"):
            return lambda x, y: (float(x) + 10.0, float(y) - 4.0)
        if (target_frame, source_frame) == ("map", "odom"):
            return lambda x, y: (float(x) - 10.0, float(y) + 4.0)
        raise AssertionError("unexpected projection direction")

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class SourcePhaseFilteredProbeLedger(PortalProbeLedger):
    """Model a map-epoch query that temporarily hides a fresh source probe."""

    def pending_for_source(
        self, _source_place_id, include_source_arrived=True, map_epoch=None,
    ):
        del include_source_arrived, map_epoch
        return []


class LadderDurableProbeExplorer(DurableProbeExplorer):
    """Fixture whose grid exposes distinct source-side ladder cells."""

    def __init__(self):
        super().__init__()
        self.transform_xy = None

    @staticmethod
    def planar_xy_projector(_target_frame, _source_frame):
        return None

    @staticmethod
    def nearest_reachable_cell(message, steps, x, y):
        row = max(0, min(int(steps.shape[0] - 1), int(round(y))))
        col = max(0, min(int(steps.shape[1] - 1), int(round(x))))
        return row, col


class CollapsedDestinationProbeExplorer(LadderDurableProbeExplorer):
    """Fixture where every destination ladder point lands on source cell."""

    @staticmethod
    def nearest_reachable_cell(_message, _steps, _x, _y):
        return 4, 3


class MixedDestinationProbeExplorer(LadderDurableProbeExplorer):
    """Fixture with one collapsed point and one executable lateral point."""

    @staticmethod
    def nearest_reachable_cell(message, steps, x, y):
        # The normal destination point projects onto the historical source
        # cell. A lateral point projects onto a distinct safe cell.
        if float(x) > 4.0:
            return 4, 3
        row = max(0, min(int(steps.shape[0] - 1), int(round(y))))
        col = max(0, min(int(steps.shape[1] - 1), int(round(x))))
        return row, col


def snapshot():
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            resolution=0.1,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
    )
    return SimpleNamespace(
        message=message,
        route_graph=SimpleNamespace(
            route_steps=np.ones((2, 6), dtype=np.int32),
            validation=None,
        ),
        robot_map=(1.0, 0.0),
        now=3.0,
    )


def crossing_snapshot(robot_map=(1.0, 0.0)):
    """Provide a grid whose coordinates cover the durable crossing fixture."""
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
    )
    return SimpleNamespace(
        message=message,
        route_graph=SimpleNamespace(
            route_steps=np.ones((2, 6), dtype=np.int32),
            validation=None,
        ),
        robot_map=robot_map,
        now=3.0,
    )


def destination_probe_snapshot():
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
    )
    return SimpleNamespace(
        message=message,
        route_graph=SimpleNamespace(
            route_steps=np.ones((10, 10), dtype=np.int32),
            validation=None,
        ),
        now=4.0,
    )


def prepare_destination_probe(explorer):
    record = explorer.portal_probe_ledger.observe(
        7,
        (4.0, 4.0),
        (1.0, 0.0),
        map_gate_xy=(4.0, 4.0),
        opening_cell=(4, 4),
        map_epoch=1,
        now=1.0,
    )
    explorer.portal_probe_ledger.start(
        record["id"], now=2.0, viewpoint_xy=(3.0, 4.0),
    )
    explorer.portal_probe_ledger.finish(
        record["id"], "source_arrived", now=3.0,
    )
    explorer.graph_route_probe_id = record["id"]
    return record


class PortalEgressTest(unittest.TestCase):
    def test_unbound_probe_ids_do_not_crash_crossing_materialization(self):
        explorer = DurableEgressExplorer()
        explorer.portal_probe_ledger = PortalProbeLedger()
        record, _created = explorer.portal_hypothesis_ledger.certify(
            4, (2.0, 0.0), (3.0, 0.0), now=1.0,
        )
        explorer.graph_route_portal_id = record["id"]
        explorer.portal_probe_ledger = SimpleNamespace(
            snapshot=lambda: [{"id": 99, "portal_id": None, "state": "pending"}],
        )

        candidate = explorer.select_durable_portal_crossing(crossing_snapshot())

        self.assertIsNone(candidate)
        self.assertEqual(explorer.events, [])

    def test_reverse_egress_reuses_planner_selected_crossed_portal(self):
        explorer = DurableEgressExplorer()
        explorer.graph_route_portal_id = 7
        edge, _created = explorer.portal_hypothesis_ledger.certify(
            1, (2.0, 0.0), (1.0, 0.0), now=1.0,
        )
        # Give the edge the identity selected by the graph planner and bind
        # its destination to the Place occupied by the egress fixture.
        self.assertEqual(edge["id"], 1)
        edge["id"] = 7
        explorer.portal_hypothesis_ledger._records[7] = (
            explorer.portal_hypothesis_ledger._records.pop(1)
        )
        explorer.portal_hypothesis_ledger._next_id = 8
        explorer.portal_hypothesis_ledger.crossed(7, now=1.5)
        explorer.portal_hypothesis_ledger.bind_destination(7, 1, now=2.0)
        explorer.current_physical_place_id = 4

        reused = explorer._remember_durable_egress_hypothesis(
            None,
            (2.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (1.0, 0.0),
            3.0,
            destination_place_id=1,
        )

        self.assertEqual(reused["id"], 7)
        self.assertEqual(reused["state"], "crossed")
        self.assertEqual(explorer.last_portal_hypothesis_id, 7)
        self.assertEqual(len(explorer.portal_hypothesis_ledger.snapshot()), 1)

    def test_reverse_egress_reuses_crossed_portal_at_other_wide_gate_jamb(self):
        explorer = DurableEgressExplorer()
        explorer.graph_route_portal_id = None
        explorer.region_memory.portal_entry_match_radius = 1.25
        edge, _created = explorer.portal_hypothesis_ledger.certify(
            1, (2.0, 0.0), (2.0, 1.0), now=1.0,
        )
        explorer.portal_hypothesis_ledger.crossed(edge["id"], now=1.5)
        explorer.portal_hypothesis_ledger.bind_destination(
            edge["id"], 4, now=1.5,
        )

        reused = explorer._remember_durable_egress_hypothesis(
            None,
            (1.0, 0.0),
            (1.0, -1.0),
            (1.0, 0.0),
            (1.0, -1.0),
            2.0,
            destination_place_id=1,
        )

        self.assertIsNotNone(reused)
        self.assertEqual(reused["id"], edge["id"])
        self.assertEqual(reused["source_place_id"], 1)
        self.assertEqual(len(explorer.portal_hypothesis_ledger.snapshot()), 1)

    def test_known_entry_provides_egress_when_snapshot_has_no_portal_labels(self):
        explorer = DurableEgressExplorer()
        candidate = explorer.select_durable_portal_egress(snapshot())

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate[10], "portal_transition")
        self.assertEqual(candidate[11], (2.0, 0.0))
        self.assertEqual(explorer.last_portal_hypothesis_id, 1)
        self.assertEqual(
            explorer.portal_hypothesis_ledger.get(1)["state"], "selected"
        )
        self.assertEqual(explorer.events[-1][0], "durable_portal_egress_selected")

    def test_target_claim_blocks_durable_egress(self):
        explorer = DurableEgressExplorer()
        explorer.target_region_claim_active = True
        self.assertIsNone(explorer.select_durable_portal_egress(snapshot()))

    def test_covered_to_covered_egress_without_progress_is_rejected(self):
        explorer = CoveredTransitEgressExplorer(with_progress=False)

        self.assertIsNone(explorer.select_durable_portal_egress(snapshot()))
        self.assertEqual(explorer.last_portal_covered_cycle_skips, 1)
        self.assertEqual(
            explorer.events[-1][1]["reason"],
            "covered_to_covered_without_graph_progress",
        )

    def test_covered_to_covered_egress_can_reach_unobserved_branch(self):
        explorer = CoveredTransitEgressExplorer(with_progress=True)

        candidate = explorer.select_durable_portal_egress(snapshot())

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate[10], "portal_transition")
        self.assertEqual(explorer.last_portal_covered_cycle_skips, 0)
        self.assertEqual(explorer.events[-1][0], "durable_portal_egress_selected")

    def test_pending_probe_rehydrates_without_live_frontier(self):
        explorer = DurableProbeExplorer()
        record = explorer.portal_probe_ledger.observe(
            7,
            (13.0, -2.0),
            (1.0, 0.0),
            map_gate_xy=(3.0, 2.0),
            opening_cell=(2, 3),
            map_epoch=4,
            now=1.0,
        )
        explorer.portal_probe_ledger.bind_work_item(record["id"], 21)
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(resolution=0.1),
        )
        snapshot = SimpleNamespace(
            message=message,
            route_graph=SimpleNamespace(
                route_steps=np.ones((4, 8), dtype=np.int32),
                validation=None,
            ),
            now=2.0,
        )

        candidate = explorer.select_durable_portal_probe(snapshot)

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate[15].probe_id, record["id"])
        self.assertEqual(candidate[12], 21)
        self.assertEqual(candidate[17], "probe_portal")
        self.assertIn(("map", "odom"), explorer.projector_calls)
        self.assertIn(("odom", "map"), explorer.projector_calls)
        self.assertEqual(explorer.events[-1][0], "durable_portal_probe_selected")
        self.assertEqual(explorer.events[-1][1]["gate"], [3.0, 2.0])

    def test_preferred_source_probe_survives_epoch_filter(self):
        """A fresh source terminal must hand off to destination evidence."""
        explorer = MixedDestinationProbeExplorer()
        explorer.portal_probe_ledger = SourcePhaseFilteredProbeLedger(
            match_radius=0.35
        )
        record = explorer.portal_probe_ledger.observe(
            7,
            (4.0, 4.0),
            (1.0, 0.0),
            map_gate_xy=(4.0, 4.0),
            opening_cell=(4, 4),
            map_epoch=1,
            now=1.0,
        )
        explorer.portal_probe_ledger.start(
            record["id"], now=2.0, viewpoint_xy=(3.0, 4.0),
        )
        explorer.portal_probe_ledger.finish(
            record["id"], "source_arrived", now=3.0,
        )
        explorer.graph_route_probe_id = record["id"]

        candidate = explorer.select_durable_portal_probe(
            destination_probe_snapshot()
        )

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate[15].probe_id, record["id"])
        self.assertEqual(candidate[15].phase, "destination")

    def test_failed_source_probe_uses_lateral_viewpoint(self):
        explorer = LadderDurableProbeExplorer()
        record = explorer.portal_probe_ledger.observe(
            7,
            (4.0, 4.0),
            (1.0, 0.0),
            map_gate_xy=(4.0, 4.0),
            opening_cell=(4, 4),
            map_epoch=1,
            now=1.0,
        )
        explorer.portal_probe_ledger.start(
            record["id"], now=2.0, viewpoint_xy=(3.0, 4.0),
        )
        explorer.portal_probe_ledger.finish(
            record["id"], "failed", now=3.0, reason="stall",
        )
        explorer.graph_route_probe_id = record["id"]

        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(
                resolution=1.0,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
            ),
        )
        snapshot = SimpleNamespace(
            message=message,
            route_graph=SimpleNamespace(
                route_steps=np.ones((10, 10), dtype=np.int32),
                validation=None,
            ),
            now=4.0,
        )

        candidate = explorer.select_durable_portal_probe(snapshot)

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate[15].probe_id, record["id"])
        self.assertEqual((candidate[2], candidate[3]), (3.0, 5.0))
        self.assertEqual(explorer.events[-1][0], "durable_portal_probe_selected")
        self.assertEqual(
            explorer.events[-1][1]["viewpoint_strategy"],
            "source_lateral_left",
        )

    def test_destination_ladder_projection_collapse_is_rejected_and_reported(self):
        explorer = CollapsedDestinationProbeExplorer()
        record = prepare_destination_probe(explorer)

        candidate = explorer.select_durable_portal_probe(
            destination_probe_snapshot()
        )

        self.assertIsNone(candidate)
        event, report = explorer.events[-1]
        self.assertEqual(event, "durable_portal_probe_unavailable")
        self.assertEqual(report["source_viewpoint_cell"], [4, 3])
        self.assertEqual(report["projected_cells"], [[4, 3]] * 3)
        self.assertTrue(report["projection_collapsed"])
        self.assertEqual(
            report["projection_collapse_reason"],
            "ladder_collapsed_to_source_viewpoint",
        )
        self.assertEqual(report["destination_ladder_candidate_count"], 3)
        self.assertEqual(
            report["rejection_reasons"][
                "projection_collapsed_to_source_viewpoint"
            ],
            3,
        )
        self.assertEqual(len(report["viewpoint_audit"]), 3)
        for audit in report["viewpoint_audit"]:
            self.assertEqual(audit["cell"], [4, 3])
            self.assertEqual(audit["map_viewpoint"], [3.0, 4.0])
            self.assertEqual(audit["physical_viewpoint"], [3.0, 4.0])
            self.assertEqual(
                audit["reason"],
                "projection_collapsed_to_source_viewpoint",
            )
        self.assertEqual(
            explorer.portal_probe_ledger.get(record["id"])["state"],
            "awaiting_projection",
        )

    def test_destination_ladder_keeps_distinct_lateral_viewpoint(self):
        explorer = MixedDestinationProbeExplorer()
        prepare_destination_probe(explorer)

        candidate = explorer.select_durable_portal_probe(
            destination_probe_snapshot()
        )

        self.assertIsNotNone(candidate)
        self.assertEqual((candidate[0], candidate[1]), (5, 3))
        self.assertEqual(
            explorer.events[-1][1]["viewpoint_strategy"],
            "source_lateral_left",
        )
        audit = explorer.events[-1][1]["viewpoint_audit"]
        self.assertEqual(len(audit), 3)
        self.assertEqual(
            audit[0]["reason"],
            "projection_collapsed_to_source_viewpoint",
        )
        self.assertEqual(audit[1]["reason"], "executable")
        self.assertEqual(audit[2]["reason"], "executable")
        self.assertEqual(
            explorer.events[-1][1]["projected_cells"],
            [[4, 3], [5, 3], [3, 3]],
        )
        self.assertFalse(explorer.events[-1][1]["projection_collapsed"])
        self.assertNotEqual(
            [candidate[0], candidate[1]],
            explorer.events[-1][1]["source_viewpoint_cell"],
        )

    def test_unbound_certified_portal_crosses_without_frontier_projection(self):
        explorer = DurableEgressExplorer()
        record, _created = explorer.portal_hypothesis_ledger.certify(
            4, (2.0, 0.0), (3.0, 0.0), now=1.0,
        )
        explorer.graph_route_portal_id = record["id"]
        candidate = explorer.select_durable_portal_crossing(crossing_snapshot())

        self.assertIsNotNone(candidate)
        self.assertEqual(candidate[10], "portal_transition")
        self.assertEqual(candidate[17], "cross_portal")
        self.assertEqual(candidate[19], record["id"])
        self.assertEqual(candidate[11], (2.0, 0.0))
        self.assertFalse(candidate[20])
        self.assertEqual(
            explorer.portal_hypothesis_ledger.get(record["id"])["state"],
            "selected",
        )
        self.assertEqual(explorer.events[-1][0], "durable_portal_crossing_selected")

    def test_destination_side_pose_is_explicitly_preobserved_crossing(self):
        explorer = DurableEgressExplorer()
        record, _created = explorer.portal_hypothesis_ledger.certify(
            4, (2.0, 0.0), (3.0, 0.0), now=1.0,
        )
        explorer.graph_route_portal_id = record["id"]
        pose = crossing_snapshot(robot_map=(4.0, 0.0))

        candidate = explorer.select_durable_portal_crossing(pose)

        self.assertIsNotNone(candidate)
        self.assertTrue(candidate[20])

    def test_destination_probe_reuses_durable_source_side_evidence(self):
        """A destination-side probe may finish the same Portal transaction."""
        explorer = DurableEgressExplorer()
        explorer.portal_probe_ledger = PortalProbeLedger()
        record, _created = explorer.portal_hypothesis_ledger.certify(
            4, (2.0, 0.0), (3.0, 0.0), now=1.0,
        )
        explorer.graph_route_portal_id = record["id"]
        probe = explorer.portal_probe_ledger.observe(
            4,
            (2.0, 0.0),
            (1.0, 0.0),
            map_gate_xy=(2.0, 0.0),
            opening_cell=(0, 2),
            map_epoch=1,
            now=1.0,
        )
        explorer.portal_probe_ledger.bind_portal(probe["id"], record["id"])
        explorer.portal_probe_ledger.start(
            probe["id"], now=2.0, viewpoint_xy=(1.0, 0.0),
        )
        explorer.portal_probe_ledger.finish(
            probe["id"], "source_arrived", now=2.1,
        )
        # The destination view has already been observed; retain the source
        # pose in the probe history while the current robot pose is beyond
        # the gate but not a full controller goal-depth away.
        explorer.portal_probe_ledger._records[probe["id"]]["state"] = "observed"
        explorer.current_physical_place_id = 4
        candidate = explorer.select_durable_portal_crossing(
            crossing_snapshot(robot_map=(2.5, 0.0))
        )

        self.assertIsNotNone(candidate)
        self.assertTrue(explorer.last_portal_source_side_proven)
        self.assertEqual(candidate[17], "cross_portal")


if __name__ == "__main__":
    unittest.main()
