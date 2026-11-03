"""Audits must not lose evidence to a corrupt byte or a cut tail message."""
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
AUDITS = ROOT / "scripts/tests/targeted_navigation"
sys.path.insert(0, str(AUDITS))


class MetricsDecodeTest(unittest.TestCase):
    def test_an_invalid_byte_does_not_hide_later_events(self):
        import audit_place_dfs
        payload = json.dumps({"ros_time": 1.0, "frontier_event": "keep"})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "metrics.log"
            path.write_bytes(
                b"2026-11-03T14:16:51 level=INFO event=global_frontier_event "
                b"data=" + payload.encode("utf-8") + b"\n"
                b"2026-11-03T14:16:52 level=INFO \xef broken byte\n"
                b"2026-11-03T14:16:53 level=INFO event=global_frontier_event "
                b"data=" + payload.encode("utf-8") + b"\n"
            )
            events = list(audit_place_dfs.events(path))
        self.assertEqual(len(events), 2)
        self.assertEqual(events[0][1]["frontier_event"], "keep")


    def test_damage_is_bounded_not_unlimited(self):
        import audit_place_dfs
        payload = b'{"ros_time":1.0,"frontier_event"\n'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "metrics.log"
            body = b"".join(
                b"event=global_frontier_event data=" + payload for _ in range(4))
            path.write_bytes(body)
            with self.assertRaises(ValueError):
                list(audit_place_dfs.events(path))


class PlotReaderTest(unittest.TestCase):
    def test_a_corrupt_byte_does_not_break_the_plot_reader(self):
        sys.path.insert(0, str(ROOT / "scripts/tools"))
        import plot_portal_authentication
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "metrics.log"
            path.write_bytes(
                b"2026-11-03T14:16:51 level=INFO event=sample "
                b'data={"pose":[1.0,2.0,0.0],"ros_time":1.0}\n'
                b"2026-11-03T14:16:52 level=INFO event=global_frontier_event "
                b"data={\xef broken\n"
            )
            _events, poses, _tree, _dedup = \
                plot_portal_authentication.read_metrics(path)
        self.assertEqual(len(poses), 1)


class TurnRawTruncationTest(unittest.TestCase):
    def test_one_cut_tail_message_is_counted_not_raised(self):
        status = json.dumps({
            "portal_authentication": {
                "branches": [{
                    "node_id": 1, "branch_id": 2, "initial_yaw": 0.0,
                    "target_yaw": 3.14, "turn_direction": "clockwise",
                    "state": "turning", "events": [
                        {"event": "inside_frontier_observed"},
                    ],
                }],
            },
        })

        class Message:
            def __init__(self, data):
                self.data = data

        class Bag:
            def __init__(self, _path):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *_exc):
                return False

            def read_messages(self, topics=None):
                yield "/rbt_pose", types.SimpleNamespace(theta=0.0), types.SimpleNamespace(to_sec=lambda: 10.0)
                yield "/lste/global_frontier/status", Message(status), types.SimpleNamespace(to_sec=lambda: 11.0)
                yield "/lste/global_frontier/status", Message(status[:-5]), types.SimpleNamespace(to_sec=lambda: 12.0)

        previous = sys.modules.get("rosbag")
        sys.modules["rosbag"] = types.SimpleNamespace(Bag=Bag)
        try:
            import audit_portal_turn_directions
            poses, turns, truncated = audit_portal_turn_directions.read_raw("unused.bag")
        finally:
            if previous is None:
                sys.modules.pop("rosbag", None)
            else:
                sys.modules["rosbag"] = previous
        self.assertEqual(len(poses), 1)
        self.assertIn((1, 2), turns)
        self.assertEqual(truncated, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
