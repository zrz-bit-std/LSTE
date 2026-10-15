#!/usr/bin/env python3
"""Regression tests for portal direction admission before navigation."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_admission import GlobalFrontierPortalAdmissionMixin
from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_place_memory import FrontierRegionMemory


class DirectionMemory:
    @staticmethod
    def candidate_tier(_x, _y, _information, **_kwargs):
        return "new", None


class DirectionAdmission(GlobalFrontierPortalAdmissionMixin):
    def __init__(self):
        self.region_memory = DirectionMemory()
        self.completed_radius = 1.25
        self.last_portal_source_side_rejections = 0
        self.last_portal_covered_cycle_skips = 0


class CoveredTransitMemory:
    def __init__(self, with_progress):
        self.source = {
            "id": 1,
            "state": "dormant",
            "endpoint_observations": 1,
        }
        self.destination = {
            "id": 2,
            "state": "dormant",
            "endpoint_observations": 1,
        }
        self.branch = {
            "id": 3,
            "state": "open",
            "endpoint_observations": 0,
        }
        self.regions = {1: self.source, 2: self.destination, 3: self.branch}
        self.with_progress = with_progress

    def by_id(self, place_id):
        return self.regions.get(int(place_id))

    def candidate_tier(self, _x, _y, _information, **_kwargs):
        return "dormant", self.destination

    def portal_exit_from_entry(self, _gate, _source, _destination, **_kwargs):
        return self.source, {"gate": [5.0, 0.0], "inside": [4.0, 0.0]}


class CoveredTransitAdmission(DirectionAdmission):
    def __init__(self, with_progress):
        super().__init__()
        self.region_memory = CoveredTransitMemory(with_progress)
        self.portal_hypothesis_ledger = PortalHypothesisLedger()
        if with_progress:
            edge, _created = self.portal_hypothesis_ledger.certify(
                2, (1.0, 0.0), (1.8, 0.0), now=1.0,
            )
            self.portal_hypothesis_ledger.crossed(edge["id"], now=1.5)
            self.portal_hypothesis_ledger.bind_destination(
                edge["id"], 3, now=2.0,
            )


class PortalDirectionAdmissionTest(unittest.TestCase):
    @staticmethod
    def request(source):
        return SimpleNamespace(
            route_anchor_xy=source,
            map_to_physical_xy=lambda x, y: (x, y),
        )

    def test_rejects_a_portal_that_starts_on_its_destination_side(self):
        admission = DirectionAdmission()

        accepted = admission.portal_destination_is_admissible(
            self.request((5.2, 0.0)),
            portal_gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            component={"label": 2},
            information=10.0,
        )

        self.assertFalse(accepted)
        self.assertEqual(admission.last_portal_source_side_rejections, 1)

    def test_accepts_an_uncovered_portal_from_its_source_side(self):
        admission = DirectionAdmission()

        accepted = admission.portal_destination_is_admissible(
            self.request((2.0, 0.0)),
            portal_gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            component={"label": 2},
            information=10.0,
        )

        self.assertTrue(accepted)
        self.assertEqual(admission.last_portal_source_side_rejections, 0)

    def test_rejects_covered_to_covered_transition_without_progress(self):
        admission = CoveredTransitAdmission(with_progress=False)

        accepted = admission.portal_destination_is_admissible(
            self.request((2.0, 0.0)),
            portal_gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            component={"label": 2},
            information=10.0,
        )

        self.assertFalse(accepted)
        self.assertEqual(admission.last_portal_covered_cycle_skips, 1)

    def test_snapshot_unknown_cannot_bypass_durable_progress_gate(self):
        admission = CoveredTransitAdmission(with_progress=False)

        self.assertFalse(
            admission._covered_portal_transit_is_allowed(
                admission.region_memory.source,
                admission.region_memory.destination,
                snapshot_progress=True,
            )
        )

    def test_allows_covered_transit_when_destination_graph_has_unobserved_branch(self):
        admission = CoveredTransitAdmission(with_progress=True)

        accepted = admission.portal_destination_is_admissible(
            self.request((2.0, 0.0)),
            portal_gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            component={"label": 2},
            information=10.0,
        )

        self.assertTrue(accepted)
        self.assertEqual(admission.last_portal_covered_cycle_skips, 0)

    def test_bound_destination_identity_overrides_transient_component(self):
        admission = CoveredTransitAdmission(with_progress=False)
        edge, _created = admission.portal_hypothesis_ledger.certify(
            1, (5.0, 0.0), (7.0, 0.0), now=1.0,
        )
        admission.portal_hypothesis_ledger.bind_destination(
            edge["id"], 2, now=2.0,
        )
        accepted = admission.portal_destination_is_admissible(
            self.request((2.0, 0.0)),
            portal_gate_xy=(5.0, 0.0),
            destination_xy=(7.0, 0.0),
            component={"label": 99},
            information=0.0,
            hypothesis=edge,
        )
        self.assertFalse(accepted)
        self.assertEqual(admission.last_portal_covered_cycle_skips, 1)

    def test_known_door_destination_is_rejected_before_new_place_dispatch(self):
        memory = FrontierRegionMemory(
            radius=2.5,
            information_delta=8.0,
            stagnation_timeout=12.0,
            failure_limit=2,
            limit=16,
        )
        component = {
            "epoch": 1,
            "label": 4,
            "cells": 80,
            "center_x": 19.5,
            "center_y": 6.0,
            "min_x": 17.1,
            "max_x": 21.9,
            "min_y": 0.2,
            "max_y": 9.1,
        }
        region, _tier = memory.enter(
            20.45,
            8.85,
            now=1.0,
            component=component,
            entry_portal=(19.65, 9.25),
            physical_xy=(20.42, 8.82),
            physical_entry_portal=(19.64, 9.42),
        )
        memory.endpoint_observed(
            20.45,
            8.85,
            now=2.0,
            component=component,
            region_id=region["id"],
            physical_xy=(20.42, 8.82),
        )

        admission = DirectionAdmission()
        admission.region_memory = memory
        accepted = admission.portal_destination_is_admissible(
            SimpleNamespace(
                route_anchor_xy=(19.5, 13.0),
                map_to_physical_xy=lambda x, y: (x, y),
            ),
            portal_gate_xy=(19.25, 9.15),
            destination_xy=(18.55, 8.75),
            component=component,
            information=10.0,
        )

        self.assertFalse(accepted)
        self.assertGreater(admission.last_portal_covered_destination_skips, 0)

    def test_snapshot_unknown_boundary_is_direct_transit_evidence(self):
        labels = np.asarray([[1, 1, 0, 2, 2, 2]], dtype=np.int32)
        unknown = np.asarray([[False, False, False, False, False, True]])
        request = SimpleNamespace(
            route_anchor_xy=(2.0, 0.0),
            map_to_physical_xy=lambda x, y: (x, y),
            unknown=unknown,
            components=SimpleNamespace(labels=labels),
        )
        self.assertTrue(
            GlobalFrontierPortalAdmissionMixin._destination_has_live_unknown(
                request, {"label": 2},
            )
        )
        self.assertFalse(
            GlobalFrontierPortalAdmissionMixin._destination_has_live_unknown(
                request, {"label": 1},
            )
        )


if __name__ == "__main__":
    unittest.main()
