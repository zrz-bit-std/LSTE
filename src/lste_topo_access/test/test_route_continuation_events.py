"""Route continuation events: a superseded route must close, the adopted one open.

Run 20261031_214617 dispatched route 140 (action_generation 253) and adopted
route 141 inside the same generation.  Only ``persistent_mission_path_adopted``
was published, so:

* route 140 never got a closing record, and
* route 141 never got an opening record.

The native costmap audit reconstructs controller ownership from
dispatch -> release pairs, so route 140 stayed open from sim 969.2 to the end
and its dispatch-time ``transparent`` mode was attributed to the following
650 s: 96 false "mask mode disagrees" errors.

The bridge now emits ``route_superseded`` (old identity) and
``route_continuation_dispatch`` (new identity).  These tests pin the exact
field sets measured from the sealed bag and verify that the audit
reconstruction closes both intervals.
"""

from pathlib import Path
import sys
import unittest


BRIDGE = Path(__file__).resolve().parents[1] / "scripts"
AUDIT = Path("/home/zrz/work/lste_ws/scripts/tests/targeted_navigation")
for path in (BRIDGE, AUDIT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from teb_goal_bridge_mission_runtime import (  # noqa: E402
    TebGoalBridgeMissionRuntimeMixin,
)
import audit_place_native_costmaps as native  # noqa: E402


# Identity measured from the sealed bag (20261031_214617, sim 969.2 / 973.0).
ROUTE_140 = dict(
    route_id=140, transaction_id=140,
    lifecycle_transaction_id=2584895849231313123168608714611613310,
    graph_transaction_id=838, map_epoch=1063, origin_epoch=1063,
    projection_epoch=1063, route_kind="frontier_endpoint",
)
ROUTE_141 = dict(
    route_id=141, transaction_id=141,
    lifecycle_transaction_id=2584895850173504926435501360006946430,
    graph_transaction_id=839, map_epoch=1065, origin_epoch=1065,
    projection_epoch=1065, route_kind="frontier_endpoint",
)
GENERATION = 253


class _Bridge(TebGoalBridgeMissionRuntimeMixin):
    def __init__(self):
        self.action_generation = GENERATION
        self.latest_goal_transaction_id = 141
        self.latest_graph_transaction_id = 839
        self.latest_route_map_epoch = 1065
        self.latest_origin_epoch = 1065
        self.latest_projection_epoch = 1065
        self.latest_route_kind = "frontier_endpoint"
        self.events = []

    def publish_bridge_status(self, event, **fields):
        self.events.append((event, fields))

    def _canonical_lifecycle_transaction_id(self):
        return ROUTE_141["lifecycle_transaction_id"]


class BridgeEventTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        bridge = _Bridge()
        bridge._publish_route_continuation_locked(
            previous_route_id=ROUTE_140["route_id"],
            previous_transaction_id=ROUTE_140["transaction_id"],
            previous_lifecycle_transaction_id=ROUTE_140["lifecycle_transaction_id"],
            previous_graph_transaction_id=ROUTE_140["graph_transaction_id"],
            previous_map_epoch=ROUTE_140["map_epoch"],
            previous_origin_epoch=ROUTE_140["origin_epoch"],
            previous_projection_epoch=ROUTE_140["projection_epoch"],
            previous_route_kind=ROUTE_140["route_kind"],
            new_route_id=ROUTE_141["route_id"],
        )
        cls.events = bridge.events

    def test_emits_exactly_two_events(self):
        self.assertEqual([name for name, _ in self.events],
                         ["route_superseded", "route_continuation_dispatch"])

    def test_superseded_identity_matches_the_sealed_dispatch(self):
        fields = self.events[0][1]
        for key in ("route_id", "transaction_id", "lifecycle_transaction_id",
                    "graph_transaction_id", "map_epoch", "origin_epoch",
                    "projection_epoch", "route_kind"):
            self.assertEqual(fields[key], ROUTE_140[key], key)
        self.assertEqual(fields["action_generation"], GENERATION)
        self.assertEqual(fields["superseded_by_route_id"], 141)

    def test_continuation_dispatch_matches_the_later_ack(self):
        fields = self.events[1][1]
        for key in ("route_id", "transaction_id", "lifecycle_transaction_id",
                    "graph_transaction_id", "map_epoch", "origin_epoch",
                    "projection_epoch", "route_kind"):
            self.assertEqual(fields[key], ROUTE_141[key], key)
        self.assertEqual(fields["action_generation"], GENERATION)
        self.assertTrue(fields["continuation"])
        self.assertEqual(fields["supersedes_route_id"], 140)


def events(pairs):
    return [(float(stamp), dict(payload, event=name)) for stamp, name, payload in pairs]


def sealed_sequence(fixed=True):
    payload_140 = dict(ROUTE_140, action_generation=GENERATION, active=True)
    payload_141 = dict(ROUTE_141, action_generation=GENERATION)
    rows = [
        (969.2, "dispatch", payload_140),
        (973.0, "persistent_mission_path_adopted", payload_141),
        (984.0, "lease_release_ack", payload_141),
    ]
    if fixed:
        rows.insert(1, (973.0, "route_superseded", payload_140))
        rows.insert(2, (973.0, "route_continuation_dispatch", payload_141))
    rows.sort(key=lambda row: row[0])
    return events(rows)


class AuditReconstructionTest(unittest.TestCase):
    def leases(self, fixed):
        return native.controller_lease_intervals(
            sealed_sequence(fixed), lambda route, stamp: "ordinary")

    def test_without_the_new_events_the_old_route_stays_open(self):
        leases = self.leases(fixed=False)
        open_routes = [lease["route_id"] for lease in leases if lease["end"] is None]
        self.assertEqual(open_routes, [140])

    def test_with_the_new_events_every_lease_closes(self):
        leases = self.leases(fixed=True)
        self.assertEqual(len(leases), 2)
        self.assertEqual([lease["route_id"] for lease in leases], [140, 141])
        self.assertTrue(all(lease["end"] is not None for lease in leases))
        self.assertTrue(all(lease["exact_ack"] for lease in leases))

    def test_no_identity_conflict_after_the_fix(self):
        leases = self.leases(fixed=True)
        self.assertFalse(any(lease.get("identity_conflict") for lease in leases))

    def test_conflict_is_detected_without_the_close(self):
        # Two overlapping routes sharing one generation is exactly the state
        # that used to be silent; the reconstruction now flags it.
        payload_140 = dict(ROUTE_140, action_generation=GENERATION, active=True)
        payload_141 = dict(ROUTE_141, action_generation=GENERATION)
        rows = events([
            (969.2, "dispatch", payload_140),
            (973.0, "route_continuation_dispatch", payload_141),
        ])
        leases = native.controller_lease_intervals(rows,
                                                   lambda route, stamp: "ordinary")
        self.assertTrue(any(lease.get("identity_conflict") for lease in leases))


SEALED_BAG = Path("/home/zrz/work/lste_ws/runtime/gp_branch_development/"
                  "place_dfs_suite_20261031_214617/semantic_l_junction_slam.bag")


@unittest.skipUnless(SEALED_BAG.is_file(), "sealed L bag not present")
class SealedBagReplayTest(unittest.TestCase):
    """Inject the two new events into the real event stream and diff the result."""

    @classmethod
    def setUpClass(cls):
        import json
        import rosbag
        rows = []
        with rosbag.Bag(str(SEALED_BAG)) as bag:
            for _, message, stamp in bag.read_messages(
                    topics=["/lste/teb_goal_bridge/status"]):
                if not (960.0 <= stamp.to_sec() <= 990.0):
                    continue
                try:
                    payload = json.loads(message.data)
                except Exception:
                    continue
                rows.append((stamp.to_sec(), payload))
        rows.sort(key=lambda item: item[0])
        cls.events = rows

    def leases(self, rows):
        return native.controller_lease_intervals(
            rows, lambda route, stamp: "transparent")

    def test_real_stream_leaves_the_superseded_route_open(self):
        leases = self.leases(self.events)
        open_routes = [lease["route_id"] for lease in leases if lease["end"] is None]
        self.assertIn(140, open_routes)

    def test_injecting_the_new_events_closes_it(self):
        injected = list(self.events)
        injected.append((973.0, dict(ROUTE_140, event="route_superseded",
                                     action_generation=GENERATION,
                                     superseded_by_route_id=141)))
        injected.append((973.0, dict(ROUTE_141,
                                     event="route_continuation_dispatch",
                                     action_generation=GENERATION,
                                     supersedes_route_id=140)))
        injected.sort(key=lambda item: item[0])
        leases = self.leases(injected)
        self.assertFalse([l for l in leases if l["end"] is None])
        self.assertFalse(any(l.get("identity_conflict") for l in leases))


class MaskModePredicateTest(unittest.TestCase):
    def test_truth_table(self):
        self.assertFalse(native.mask_mode_conflict("ordinary", True))
        self.assertTrue(native.mask_mode_conflict("ordinary", False))
        self.assertTrue(native.mask_mode_conflict("transparent", True))
        self.assertFalse(native.mask_mode_conflict("transparent", False))


if __name__ == "__main__":
    unittest.main(verbosity=2)
