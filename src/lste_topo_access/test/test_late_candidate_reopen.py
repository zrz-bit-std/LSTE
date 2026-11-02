"""A doorway that matures after its junction closed is still an obligation.

Run 20261102_205142 (L attempt 1) ended with 13/15: the only auth failure was
``every_stable_candidate_started``.  Branch 40 on node 11 first appeared at sim
1585.6 as ``pending``/``stable=False``; it became ``stable=True`` while the
robot was elsewhere, by which time node 11's observation period had closed.
Its final snapshot was ``pending, stable, open`` and the planner declared
``all_durable_graph_obligations_settled`` at sim 1800.3 because no probe
existed for it.

Branch stability is time based, so this is a race the runtime must handle:
re-open the junction and route back to it, bounded so a doorway that can never
be entered cannot loop the mission.
"""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

# The module logs through rospy; no node exists in a unit test.
sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        loginfo=lambda *_a, **_k: None,
        logwarn=lambda *_a, **_k: None,
        logwarn_throttle=lambda *_a, **_k: None,
        logerr=lambda *_a, **_k: None,
    ),
)

from gp_branch_frontier import GPBranchFrontierMixin  # noqa: E402


def branch(**overrides):
    value = dict(
        node_id=11,
        stable=True,
        passability="open",
        physical_gate=[1.0, 2.0],
        connection=None,
        state="pending",
        authentication_state="pending",
        first_seen=100.0,
    )
    value.update(overrides)
    return value


class Fixture(GPBranchFrontierMixin):
    def __init__(self, branches=None, nodes=None):
        self.gp_exploration_tree = SimpleNamespace(
            branches=dict(branches or {}), nodes=dict(nodes or {})
        )
        self.late_candidate_reopen_limit = 2
        self.late_candidate_stable_grace = 20.0
        self.pending_graph_resume_place_id = None
        self.junction_portal_authentication = None
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


NODES = {11: dict(id=11, junction_confirmed=True),
         12: dict(id=12, junction_confirmed=True),
         13: dict(id=13, junction_confirmed=True)}


class LateCandidateReopenTest(unittest.TestCase):
    def test_a_matured_doorway_reopens_its_junction(self):
        fixture = Fixture({40: branch()}, NODES)
        self.assertTrue(fixture.reopen_late_stable_candidates(200.0))
        self.assertEqual(fixture.pending_graph_resume_place_id, 11)
        self.assertEqual(
            [event for event, _fields in fixture.events],
            ["late_candidate_junction_reopened"],
        )

    def test_an_immature_branch_is_left_alone(self):
        fixture = Fixture({40: branch(first_seen=195.0)}, NODES)
        self.assertFalse(fixture.reopen_late_stable_candidates(200.0))

    def test_unstable_or_closed_branches_are_ignored(self):
        for overrides in (
            dict(stable=False),
            dict(passability="blocked"),
            dict(physical_gate=None),
            dict(connection="return"),
            dict(state="superseded"),
            dict(state="return"),
        ):
            fixture = Fixture({40: branch(**overrides)}, NODES)
            self.assertFalse(
                fixture.reopen_late_stable_candidates(200.0), overrides)
            self.assertIsNone(fixture.pending_graph_resume_place_id)

    def test_an_already_started_branch_is_not_reopened(self):
        for state in ("started", "completed", "failed", "turning"):
            fixture = Fixture({40: branch(authentication_state=state)}, NODES)
            self.assertFalse(
                fixture.reopen_late_stable_candidates(200.0), state)

    def test_an_unconfirmed_node_is_not_reopened(self):
        fixture = Fixture({40: branch()}, {11: dict(id=11,
                                                    junction_confirmed=False)})
        self.assertFalse(fixture.reopen_late_stable_candidates(200.0))

    def test_the_reopen_is_bounded(self):
        fixture = Fixture(
            {40: branch(node_id=11), 41: branch(node_id=12, first_seen=101.0),
             42: branch(node_id=13, first_seen=102.0)}, NODES)
        self.assertTrue(fixture.reopen_late_stable_candidates(200.0))
        self.assertTrue(fixture.reopen_late_stable_candidates(201.0))
        self.assertFalse(fixture.reopen_late_stable_candidates(202.0))
        self.assertEqual(len(fixture.events), 2)

    def test_the_same_node_is_not_reopened_twice(self):
        fixture = Fixture({40: branch(), 41: branch(first_seen=101.0)}, NODES)
        self.assertTrue(fixture.reopen_late_stable_candidates(200.0))
        fixture.pending_graph_resume_place_id = None
        self.assertFalse(fixture.reopen_late_stable_candidates(201.0))

    def test_a_junction_under_observation_is_skipped(self):
        fixture = Fixture({40: branch()}, NODES)
        fixture.junction_portal_authentication = SimpleNamespace(
            node_id=11, all_finished=lambda: False
        )
        self.assertFalse(fixture.reopen_late_stable_candidates(200.0))


class ConfigContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = (
            SCRIPTS / "global_frontier_config_navigation.py").read_text(
                encoding="utf-8")

    def test_the_reopen_is_bounded_by_parameters(self):
        self.assertIn("late_candidate_reopen_limit", self.config)
        self.assertIn("late_candidate_stable_grace", self.config)


if __name__ == "__main__":
    unittest.main(verbosity=2)
