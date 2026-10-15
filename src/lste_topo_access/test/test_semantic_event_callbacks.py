#!/usr/bin/env python3
"""Regression tests for semantic ROS callback lifecycle boundaries."""

from pathlib import Path
import json
import sys
import unittest
from types import SimpleNamespace


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

# The callback module only needs rospy for the asynchronous bridge callbacks;
# this test exercises task/detection input with a lightweight stand-in.
sys.modules.setdefault("rospy", SimpleNamespace())

from global_frontier_event_callbacks import GlobalFrontierEventCallbacksMixin
import global_frontier_event_callbacks


class CallbackHarness(GlobalFrontierEventCallbacksMixin):
    def __init__(self):
        from global_frontier_semantic_belief import SemanticPlaceBelief
        from global_frontier_target_belief import TargetBeliefLedger
        from global_frontier_target_observation_work import TargetObservationWorkLedger

        self.current_task_id = ""
        self.current_task_version = ""
        self.current_mission_id = ""
        self.latest_task = None
        self.semantic_place_belief = SemanticPlaceBelief()
        self.target_belief = TargetBeliefLedger()
        self.target_observation_work = TargetObservationWorkLedger()
        self.pending_semantic_observations = []
        self.pending_target_belief_observations = []
        self.pending_target_track_ids = []
        self._semantic_last_report = {}
        self._target_belief_last_report = {}
        self._target_belief_geometry_reports = set()
        self.current_physical_place_id = 4
        self.region_memory = SimpleNamespace(
            by_id=lambda _place_id: {"id": 4, "endpoint_observations": 1}
        )
        self.place_work_items = SimpleNamespace(unresolved_count=lambda _place_id: 1)
        self.published = []

    def publish_status(self, event, **fields):
        self.published.append((event, fields))


def make_task():
    return SimpleNamespace(
        task_id="yellow_cup",
        target_name="yellow cup",
        target_attributes=["yellow"],
        env_related_structures=["monitor"],
        env_type_prior=["office workspace"],
        obj_key_objects=["monitor"],
        obj_negative_clues=["door"],
        ctx_left="monitor",
        ctx_right="monitor",
        raw_json="",
    )


def make_detection(labels, target_labels=()):
    target = [SimpleNamespace(label=value) for value in target_labels]
    env = [SimpleNamespace(label=value) for value in labels]
    return SimpleNamespace(
        task_id="yellow_cup",
        header=SimpleNamespace(
            stamp=SimpleNamespace(to_sec=lambda: 2.0)
        ),
        env_dets=env,
        target_dets=target,
    )


class SemanticCallbackTest(unittest.TestCase):
    def test_repeated_same_concept_does_not_flood_evidence_events(self):
        harness = CallbackHarness()
        harness.on_task(make_task())
        harness.on_detections(make_detection(["monitor"]))
        first_count = len(harness.published)
        harness.on_detections(make_detection(["monitor"]))
        self.assertEqual(len(harness.published), first_count)
        self.assertEqual(
            harness.semantic_place_belief.evidence(4)["observations"], 2
        )

    def test_target_evidence_is_recorded_as_a_new_semantic_state(self):
        harness = CallbackHarness()
        harness.on_task(make_task())
        harness.on_detections(make_detection([], target_labels=["yellow cup"]))
        self.assertEqual(
            harness.semantic_place_belief.evidence(4)["target_hits"], 1
        )
        self.assertEqual(
            harness.target_belief.evidence(4)["target_hits"], 1
        )
        self.assertEqual(harness.published[-1][0], "semantic_place_evidence")
        self.assertTrue(harness.target_observation_work.has_pending(4))

    def test_target_track_binds_and_explicit_release_settles_place_work(self):
        harness = CallbackHarness()
        harness.on_task(make_task())
        harness.on_detections(make_detection([], target_labels=["yellow cup"]))
        harness.on_goal_arbitration(SimpleNamespace(data=json.dumps({
            "event": "target_track_started",
            "task_version": harness.current_task_version,
            "target_track_id": "yellow_cup:cup:1",
        })))
        self.assertEqual(
            harness.target_observation_work.evidence(4)["track_ids"],
            ["yellow_cup:cup:1"],
        )
        harness.target_observation_work.release(
            harness.current_task_version,
            track_id="yellow_cup:cup:1",
            now=4.0,
            reason="target_track_expired",
        )
        self.assertFalse(harness.target_observation_work.has_pending(4))

    def test_goal_manager_bearing_binds_to_the_current_target_track(self):
        harness = CallbackHarness()
        harness.on_task(make_task())
        global_frontier_event_callbacks.rospy.Time = SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda: 3.0)
        )
        harness.active_work_item_id = 11
        harness.last_portal_hypothesis_id = None
        harness.on_goal_arbitration(SimpleNamespace(data=json.dumps({
            "event": "target_segment_committed",
            "task_version": harness.current_task_version,
            "target_track_id": "yellow_cup:cup:1",
            "target_bearing_odom": [3.0, 4.0],
            "target_observation_origin_odom": [1.0, 2.0],
        })))
        evidence = harness.target_belief.evidence(4)
        self.assertEqual(evidence["state"], "target_confirmed")
        self.assertEqual(evidence["bearing_xy"], (0.6, 0.8))
        self.assertEqual(evidence["origin_xy"], (1.0, 2.0))
        before = len(harness.published)
        harness.on_goal_arbitration(SimpleNamespace(data=json.dumps({
            "event": "target_segment_committed",
            "task_version": "stale-task-version",
            "target_track_id": "old-track",
            "target_bearing_odom": [0.0, 1.0],
        })))
        self.assertEqual(len(harness.published), before)


if __name__ == "__main__":
    unittest.main()
