#!/usr/bin/env python3
"""Regression tests for controller failures on already-crossed Portals."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

rospy = sys.modules.setdefault("rospy", SimpleNamespace())
if not hasattr(rospy, "Time"):
    rospy.Time = SimpleNamespace(
        now=lambda: SimpleNamespace(to_sec=lambda: 3.0),
    )
if not hasattr(rospy, "logwarn"):
    rospy.logwarn = lambda *_args, **_kwargs: None

from global_frontier_event_callbacks import (  # noqa: E402
    GlobalFrontierEventCallbacksMixin,
)
from global_frontier_portal_belief import PortalHypothesisLedger  # noqa: E402


class BridgeFailureHarness(GlobalFrontierEventCallbacksMixin):
    def __init__(self):
        self.task_done = False
        self.active_frontier = (1, 2, 3.0, 4.0)
        self.active_route_id = 8
        self.active_route_kind = "portal_transition"
        self.active_mission_route_kind = "portal_transition"
        self.last_portal_hypothesis_id = 1
        self.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _created = self.portal_hypothesis_ledger.certify(
            1, (2.0, 0.0), (2.0, 1.0), now=1.0,
        )
        self.portal_hypothesis_ledger.crossed(edge["id"], now=2.0)
        self.portal_hypothesis_ledger.bind_destination(
            edge["id"], 2, now=2.0,
        )
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class PortalExecutionFailureTest(unittest.TestCase):
    def test_aborted_reverse_route_does_not_invalidate_crossed_portal(self):
        harness = BridgeFailureHarness()
        message = SimpleNamespace(
            data=json.dumps({
                "event": "terminal",
                "active_route_id": 8,
                "active_intent_source": "global_slam_frontier",
                "active_route_kind": "portal_transition",
                "active_mission_route_kind": "portal_transition",
                "status": 4,
                "status_text": "ABORTED",
            })
        )

        with patch.object(
            rospy.Time,
            "now",
            return_value=SimpleNamespace(to_sec=lambda: 3.0),
        ):
            harness.on_bridge_status(message)

        record = harness.portal_hypothesis_ledger.get(1)
        self.assertEqual(record["state"], "crossed")
        self.assertEqual(record["destination_place_id"], 2)
        self.assertEqual(record["execution_failure_count"], 1)
        self.assertFalse(any(
            event == "portal_hypothesis_failed"
            for event, _fields in harness.events
        ))
        execution_events = [
            fields for event, fields in harness.events
            if event == "portal_execution_failure_observed"
        ]
        self.assertEqual(len(execution_events), 1)
        self.assertEqual(execution_events[0]["state"], "crossed")
        self.assertEqual(execution_events[0]["portal_id"], 1)


if __name__ == "__main__":
    unittest.main()
