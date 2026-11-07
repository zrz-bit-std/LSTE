"""Campaign contract: perception LOCKED ends an attempt as success."""

import json
from pathlib import Path
import sys
import tempfile
import unittest


TOOLS = Path(__file__).resolve().parents[3] / "scripts" / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import audit_search_attempt as audit
import run_ten_attempts as campaign


class LockedSuccessTest(unittest.TestCase):
    def test_state_two_ends_the_attempt(self):
        records = [
            ("sample", {"pose": [0.0, 0.0]}),
            ("state_change", {"state": 1, "current": "1:Sus-B"}),
            ("state_change", {"state": 2, "current": "2:Locked"}),
        ]
        self.assertEqual(campaign.locked_state(records)["current"], "2:Locked")

    def test_suspicious_state_does_not_end_the_attempt(self):
        records = [
            ("state_change", {"state": 0, "current": "0:-"}),
            ("state_change", {"state": 1, "current": "1:Sus-B"}),
        ]
        self.assertIsNone(campaign.locked_state(records))

    def test_task_done_is_not_locked_state(self):
        self.assertIsNone(campaign.locked_state([("task_done", {"value": True})]))

    def test_non_numeric_state_is_ignored(self):
        records = [
            ("state_change", {"state": None}),
            ("state_change", {"state": "locked"}),
        ]
        self.assertIsNone(campaign.locked_state(records))

    def test_audit_counts_recorded_locked_as_success_evidence(self):
        rows = [
            {"kind": "state", "state": 0},
            {"kind": "state", "state": 2},
        ]
        self.assertTrue(audit.state_rows_show_locked(rows))
        self.assertFalse(audit.state_rows_show_locked([{"kind": "state", "state": 1}]))


class TotalLimitTest(unittest.TestCase):
    def test_zero_limit_means_unlimited(self):
        self.assertFalse(campaign.total_limit_exceeded(10_000_000.0, 0))
        self.assertFalse(campaign.total_limit_exceeded(10_000_000.0, -1))

    def test_positive_limit_still_applies(self):
        self.assertTrue(campaign.total_limit_exceeded(11.0, 10.0))
        self.assertFalse(campaign.total_limit_exceeded(9.0, 10.0))


def _metric_line(ros, pose, event="sample"):
    data = {"ros_time": ros}
    if pose is not None:
        data["pose"] = pose
    return (
        "2026-11-07T01:00:00.000 level=INFO process=test "
        "event=%s data=%s\n" % (event, json.dumps(data))
    )


class MetricsReaderTest(unittest.TestCase):
    def test_partial_trailing_line_is_retried(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            first = _metric_line(1.0, [1.0, 2.0])
            second = _metric_line(2.0, [2.0, 3.0])
            third = _metric_line(3.0, [3.0, 4.0])
            path.write_text(first + second + third[:20])
            records, offset = campaign.read_new_records(path, 0)
            self.assertEqual([data["ros_time"] for _, data in records], [1.0, 2.0])
            self.assertEqual(offset, len(first + second))
            with path.open("ab") as stream:
                stream.write(third[20:].encode("utf-8"))
            records, offset = campaign.read_new_records(path, offset)
            self.assertEqual([data["ros_time"] for _, data in records], [3.0])
            self.assertEqual(offset, path.stat().st_size)

    def test_tail_recovery_reads_the_newest_complete_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text(
                _metric_line(10.0, [1.0, 2.0])
                + _metric_line(20.0, [3.0, 4.0])
                + "2026-11-07T01:00:00.000 level=INFO "
                  "event=sample data={\"pose\":[5.0,"
            )
            recovered = campaign.recover_latest_sample(path)
            self.assertIsNotNone(recovered)
            self.assertEqual(recovered["ros_time"], 20.0)
            self.assertEqual(recovered["pose"], [3.0, 4.0])

    def test_latest_json_line_returns_newest_kind(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pose.jsonl"
            path.write_text(
                json.dumps({"kind": "pose", "x": 1.0, "y": 2.0}) + "\n"
                + json.dumps({"kind": "state", "state": 1}) + "\n"
                + json.dumps({"kind": "pose", "x": 3.0, "y": 4.0}) + "\n"
            )
            pose = campaign.read_latest_json_line(path, "pose")
            state = campaign.read_latest_json_line(path, "state")
            self.assertEqual((pose["x"], pose["y"]), (3.0, 4.0))
            self.assertEqual(state["state"], 1)

    def test_missing_file_is_a_no_op(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.log"
            records, offset = campaign.read_new_records(path, 7)
            self.assertEqual(records, [])
            self.assertEqual(offset, 7)
            self.assertIsNone(campaign.recover_latest_sample(path))


if __name__ == "__main__":
    unittest.main()
