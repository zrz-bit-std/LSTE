"""Regression tests for reusing certified portals after SLAM relabeling."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_candidate_portals import GlobalFrontierCandidatePortalMixin
from global_frontier_portal_belief import PortalHypothesisLedger


class RehydrationExplorer(GlobalFrontierCandidatePortalMixin):
    def __init__(self):
        self.portal_hypothesis_ledger = PortalHypothesisLedger()
        self.current_task_version = "task-v1"
        self.current_physical_place_id = 4
        self.graph_route_portal_id = None
        self.last_portal_unbound_reused = 0
        self.last_portal_unbound_reprojection_skips = 0

    @staticmethod
    def transform_xy(_target, _source, x, y):
        return float(x) + 0.5, float(y) - 0.25

    @staticmethod
    def nearest_reachable_cell(_message, _steps, _x, _y):
        return 2, 4


def request():
    return SimpleNamespace(
        message=SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(
                resolution=1.0,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0)
                ),
            ),
        ),
        steps=np.ones((8, 8), dtype=np.int32),
        now=2.0,
    )


class PortalUnboundRehydrationTest(unittest.TestCase):
    def test_reprojects_unbound_portal_and_preserves_identity(self):
        explorer = RehydrationExplorer()
        record, _created = explorer.portal_hypothesis_ledger.certify(
            4, (2.0, 3.0), (4.0, 3.0), now=1.0,
        )
        sources = {}
        explorer._populate_unbound_portal_sources(
            request(), SimpleNamespace(source_place_id=4), sources,
        )
        self.assertEqual(list(sources), [("hypothesis", record["id"])])
        source = next(iter(sources.values()))
        self.assertEqual(source.hypothesis_id, record["id"])
        self.assertEqual((source.gate_row, source.gate_col), (2, 2))
        self.assertEqual((source.endpoint_row, source.endpoint_col), (2, 4))
        self.assertEqual(explorer.last_portal_unbound_reused, 1)
        self.assertEqual(
            explorer.portal_hypothesis_ledger.get(record["id"])["map_gate"],
            [2.5, 2.75],
        )

    def test_reprojects_graph_selected_crossed_portal_without_labels(self):
        explorer = RehydrationExplorer()
        record, _created = explorer.portal_hypothesis_ledger.certify(
            4, (2.0, 3.0), (4.0, 3.0), now=1.0,
        )
        explorer.portal_hypothesis_ledger.crossed(record["id"], now=2.0)
        explorer.portal_hypothesis_ledger.bind_destination(
            record["id"], 9, now=2.0,
        )
        explorer.graph_route_portal_id = record["id"]
        sources = {}

        explorer._populate_preferred_crossed_portal_source(
            request(), SimpleNamespace(source_place_id=4), sources,
        )

        self.assertEqual(list(sources), [("hypothesis", record["id"])])
        source = next(iter(sources.values()))
        self.assertEqual(source.hypothesis_id, record["id"])
        self.assertEqual((source.gate_row, source.gate_col), (2, 2))
        self.assertEqual((source.endpoint_row, source.endpoint_col), (2, 4))


if __name__ == "__main__":
    unittest.main()
