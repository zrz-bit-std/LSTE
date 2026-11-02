"""Candidate doorways are judged over the run, not over its final snapshot.

``confirmed_junction_candidates_observed`` used to scan the *final* tree
snapshot only.  A completed observation period rewrites every branch to
``authenticated``/``authentication_failed``, so the final snapshot can contain
no branch that still looks like an open candidate even though the period ran
correctly.  The check now uses the union of candidates observed in any
snapshot; ``every_stable_candidate_started`` still uses the final snapshot so
nothing else is relaxed.
"""

from pathlib import Path
import json
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts/tests/targeted_navigation"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from check_portal_authentication import (  # noqa: E402
    candidate_branch_facts,
    load_events,
    observed_candidate_branch_ids,
)


def tree(nodes, branches):
    return {
        "nodes": [dict(id=node_id, junction_confirmed=confirmed,
                       physical_xy=xy, branch_ids=[b["id"] for b in branches])
                  for node_id, confirmed, xy in nodes],
        "branches": branches,
    }


def branch(branch_id, node_id, stable=True, passability="open", gate=(1.0, 0.0),
           connection=None, state="pending"):
    return {
        "id": branch_id,
        "node_id": node_id,
        "stable": stable,
        "passability": passability,
        "physical_gate": gate,
        "connection": connection,
        "state": state,
    }


class CandidateFactsTest(unittest.TestCase):
    def test_only_confirmed_open_gated_forward_branches_count(self):
        snapshot = tree(
            [(7, True, (0.0, 0.0)), (8, False, (5.0, 0.0))],
            [
                branch(1, 7),
                branch(2, 7, stable=False),
                branch(3, 7, passability="blocked"),
                branch(4, 7, gate=None),
                branch(5, 7, connection="return"),
                branch(6, 7, state="superseded"),
                branch(9, 8),
            ],
        )
        self.assertEqual(sorted(candidate_branch_facts(snapshot)), [1])

    def test_radius_filter_uses_the_recorded_node_pose(self):
        facts = {1: {"node_id": 7, "node_xy": [0.0, 0.0]},
                 2: {"node_id": 9, "node_xy": [10.0, 0.0]}}
        self.assertEqual(observed_candidate_branch_ids(facts, (0.0, 0.0), 2.0), {1})
        self.assertEqual(
            observed_candidate_branch_ids(facts, None, 2.0), {1, 2}
        )


class HistoryCollectionTest(unittest.TestCase):
    """``load_events`` must keep candidates that a later snapshot lost."""

    def write(self, records):
        handle = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False)
        for payload in records:
            handle.write(
                "2026-11-01T00:00:00 level=INFO process=x "
                "event=global_frontier_event data=%s\n" % json.dumps(payload)
            )
        handle.close()
        return handle.name

    def test_a_candidate_seen_early_survives_a_later_authenticated_snapshot(self):
        early = tree([(7, True, (0.0, 0.0))], [branch(1, 7)])
        late = tree([(7, True, (0.0, 0.0))], [branch(1, 7, state="superseded")])
        path = self.write([
            {"frontier_event": "gp_branch_tree", "tree": early, "ros_time": 10.0},
            {"frontier_event": "gp_branch_tree", "tree": late, "ros_time": 20.0},
        ])
        events = load_events(path)
        record = [data for name, data in events if name == "gp_branch_tree"][0]
        self.assertEqual(sorted(record["candidate_history"]), [1])
        self.assertEqual(candidate_branch_facts(record["tree"]), {})

    def test_the_final_snapshot_is_still_the_single_collapsed_tree(self):
        early = tree([(7, True, (0.0, 0.0))], [branch(1, 7)])
        late = tree([(8, True, (5.0, 0.0))], [branch(2, 8)])
        path = self.write([
            {"frontier_event": "gp_branch_tree", "tree": early, "ros_time": 10.0},
            {"frontier_event": "gp_branch_tree", "tree": late, "ros_time": 20.0},
        ])
        events = load_events(path)
        records = [data for name, data in events if name == "gp_branch_tree"]
        self.assertEqual(len(records), 1)
        self.assertEqual(sorted(candidate_branch_facts(records[0]["tree"])), [2])
        self.assertEqual(sorted(records[0]["candidate_history"]), [1, 2])


if __name__ == "__main__":
    unittest.main(verbosity=2)
