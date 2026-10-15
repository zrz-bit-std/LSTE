"""Regression contract for detector evidence before target confirmation."""

import ast
from dataclasses import dataclass
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "goal_manager_detection.py"


def load_record_candidate():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    selected = [
        node
        for node in tree.body
        if isinstance(node, (ast.ClassDef, ast.FunctionDef))
        and node.name in (
            "DetectionEvidenceContext",
            "_record_candidate",
            "_detector_source_identity",
            "_update_viewpoint_baseline",
            "_wrap_angle",
        )
    ]
    namespace = {
        "dataclass": dataclass,
        "math": math,
        "rospy": SimpleNamespace(Time=lambda value: value),
        "VIEWPOINT_BASELINE_M": 0.20,
        "VIEWPOINT_BASELINE_YAW_RAD": math.radians(15.0),
    }
    exec(
        compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["_record_candidate"]


class CandidateManager:
    def __init__(self):
        self.target_last_detection_stamp = None
        self.target_observation_epoch = 0
        self.target_blocked = False
        self.target_execution_state = "IDLE"
        self.target_candidate_last_seen = None
        self.target_follow_confirm_window = 20.0
        self.target_candidate_hits = 0
        self.target_candidate_score_sum = 0.0
        self.target_candidate_anchor_cx = None
        self.target_candidate_anchor_cy = None
        self.target_candidate_viewpoint_anchor_odom = None
        self.target_candidate_viewpoint_anchor_yaw = None
        self.target_candidate_viewpoint_translation = 0.0
        self.target_candidate_viewpoint_yaw_delta = 0.0
        self.target_candidate_viewpoint_diverse = False
        self.target_follow_spatial_tolerance = 0.20
        self.target_done_min_score = 0.4
        self.target_candidate_source_stamp = None
        self.target_track_label = ""
        self.target_track_sequence = 0
        self.current_task_id = "yellow_cup"
        self.target_track_id = ""
        self.target_last_seen = None
        self.latest_pose = None
        self.target_candidate_room_claim_requested = False
        self.events = []

    @staticmethod
    def clone_detection(detection):
        return detection

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))

    def request_global_frontier_replan(self, *_args, **_kwargs):
        raise AssertionError("an unconfirmed detector frame must not replan")


class TargetDetectionContractTest(unittest.TestCase):
    def test_one_weak_frame_records_releasable_obligation_without_replanning(self):
        record_candidate = load_record_candidate()
        manager = CandidateManager()
        stamp = SimpleNamespace(secs=10, nsecs=20, to_sec=lambda: 10.00000002)
        message = SimpleNamespace(header=SimpleNamespace(stamp=stamp, seq=1))
        detection = SimpleNamespace(
            score=0.498,
            w=0.021,
            h=0.020,
            cx=0.50,
            cy=0.50,
            label="yellow cup",
        )

        evidence = record_candidate(manager, message, detection, 0.498, 0.021, 12.0)

        self.assertIsNotNone(evidence)
        self.assertEqual(manager.target_candidate_hits, 1)
        # The room observation obligation is registered at track creation, but
        # no navigation replan is requested until the track is confirmed or
        # explicitly lost.
        self.assertTrue(manager.target_candidate_room_claim_requested)
        self.assertEqual([event for event, _fields in manager.events], ["target_track_started"])

    def test_zero_stamped_frames_use_receipt_identity(self):
        record_candidate = load_record_candidate()
        manager = CandidateManager()
        detection = SimpleNamespace(
            score=0.30,
            w=0.02,
            h=0.02,
            cx=0.50,
            cy=0.50,
            label="yellow cup",
        )

        def message():
            stamp = SimpleNamespace(secs=0, nsecs=0, to_sec=lambda: 0.0)
            return SimpleNamespace(
                header=SimpleNamespace(stamp=stamp, seq=0),
            )

        first = record_candidate(manager, message(), detection, 0.30, 0.02, 1.0)
        second = record_candidate(manager, message(), detection, 0.30, 0.02, 2.0)

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertEqual(manager.target_candidate_hits, 2)

    def test_candidate_votes_can_record_a_real_viewpoint_baseline(self):
        record_candidate = load_record_candidate()
        manager = CandidateManager()
        manager.latest_pose = SimpleNamespace(x=1.0, y=2.0, theta=0.0)
        detection = SimpleNamespace(
            score=0.30, w=0.02, h=0.02, cx=0.50, cy=0.50, label="yellow cup",
        )

        def message(seq):
            stamp = SimpleNamespace(
                secs=10 + seq, nsecs=0, to_sec=lambda: float(10 + seq),
            )
            return SimpleNamespace(
                header=SimpleNamespace(stamp=stamp, seq=seq)
            )

        record_candidate(manager, message(1), detection, 0.30, 0.02, 12.0)
        manager.latest_pose = SimpleNamespace(x=1.25, y=2.0, theta=0.0)
        record_candidate(manager, message(2), detection, 0.30, 0.02, 13.0)

        self.assertTrue(manager.target_candidate_viewpoint_diverse)
        self.assertGreaterEqual(manager.target_candidate_viewpoint_translation, 0.20)

    def test_blocked_target_is_not_unlocked_by_detection_callback(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("_clear_blocked_target(manager, score)", source)


if __name__ == "__main__":
    unittest.main()
