"""A lease left open by the run window is not a lost lease.

Run ``20261101_153446`` (T, 900 s window) dispatched route 102 at sim 638.2 and
the window closed at 639.07.  The audit reported

    raw controller dispatch lacks matching exact lease ACK
    routes=[{route_id: 102, action_generation: 199, start: 638.208}]

which is a truncation artifact: the bridge had no chance to publish the ACK.
The strict identity rule must still catch a lease that is genuinely lost mid
run (run ``20261031_214617``: route 140 dispatched, route 141 adopted in the
same action generation, ACK named 141, 140's lease never closed), so only the
*final* lease-bearing event may be treated as open at the end.
"""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts/tests/targeted_navigation"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from audit_place_native_costmaps import controller_lease_intervals  # noqa: E402


IDENTITY = dict(
    graph_transaction_id=42,
    lifecycle_transaction_id=99,
    transaction_id=17,
    map_epoch=111,
    origin_epoch=111,
    projection_epoch=111,
)


def dispatch(route, generation, stamp):
    return stamp, dict(
        IDENTITY,
        event="dispatch",
        active=True,
        route_id=route,
        active_route_id=route,
        action_generation=generation,
        active_action_generation=generation,
    )


def ack(route, generation, stamp):
    return stamp, dict(
        IDENTITY,
        event="lease_release_ack",
        active=False,
        route_id=route,
        action_generation=generation,
    )


def leases(rows):
    return controller_lease_intervals(rows, lambda _route, _stamp: "ordinary")


class LeaseTruncationTest(unittest.TestCase):
    def test_final_dispatch_open_at_window_end_is_not_an_error(self):
        rows = [
            dispatch(100, 190, 600.0),
            ack(100, 190, 600.5),
            dispatch(102, 199, 638.208),
        ]
        found = leases(rows)
        final = [item for item in found if item["route_id"] == 102][0]
        self.assertFalse(final["exact_ack"])
        self.assertTrue(final.get("open_at_end"))
        # The caller's strict rule, applied to this list, must stay silent.
        self.assertEqual(
            [item for item in found
             if not item["exact_ack"] and not item.get("open_at_end")],
            [],
        )

    def test_an_earlier_unacked_lease_is_still_reported(self):
        # The 20261031_214617 shape: 140 is adopted by 141 and never ACKed.
        rows = [
            dispatch(140, 190, 600.0),
            dispatch(141, 190, 601.0),
            ack(141, 190, 601.5),
        ]
        found = leases(rows)
        lost = [item for item in found
                if not item["exact_ack"] and not item.get("open_at_end")]
        self.assertEqual([item["route_id"] for item in lost], [140])

    def test_open_at_end_requires_the_last_lease_event_to_be_a_dispatch(self):
        rows = [
            dispatch(102, 199, 638.208),
            ack(102, 199, 638.4),
        ]
        found = leases(rows)
        self.assertFalse(found[0].get("open_at_end"))
        self.assertTrue(found[0]["exact_ack"])

    def test_only_the_last_dispatch_is_flagged(self):
        rows = [
            dispatch(101, 195, 620.0),
            dispatch(102, 199, 638.208),
        ]
        found = {item["route_id"]: item for item in leases(rows)}
        self.assertTrue(found[102].get("open_at_end"))
        self.assertFalse(found[101].get("open_at_end"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
