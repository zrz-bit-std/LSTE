"""Regression tests for return_unrecoverable portal-entry cleanup."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from gp_branch_frontier import GPBranchFrontierMixin  # noqa: E402


class _PortalLedger:
    def __init__(self):
        self.record = {
            "id": 1,
            "semantic_entry_work_item_id": 6,
            "gp_authentication_branch_id": 2,
        }
        self.failed = []

    def get(self, portal_id):
        return self.record if int(portal_id) == 1 else None

    def snapshot(self):
        return [self.record]

    def execution_failed(self, portal_id, now, epoch=None):
        self.failed.append((int(portal_id), float(now), epoch))
        return self.record


class _WorkItems:
    def __init__(self):
        self.calls = []

    def abandon_portal_entry(self, portal_id, now, reason,
                             resolution="", item_id=None):
        self.calls.append((
            int(portal_id), float(now), reason, resolution, item_id,
        ))
        return {"id": item_id or 6}


class _Node(GPBranchFrontierMixin):
    def __init__(self):
        self.portal_hypothesis_ledger = _PortalLedger()
        self.place_work_items = _WorkItems()
        self.last_map_epoch = 7
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class PortalReturnCleanupTest(unittest.TestCase):
    def test_canonical_portal_entry_is_retired_and_deferred(self):
        node = _Node()
        auth = SimpleNamespace(branch_id=2)
        self.assertTrue(node.retire_unrecoverable_portal_entry(
            auth, {"canonical_portal_id": 1}, 10.0,
        ))
        self.assertEqual(
            node.place_work_items.calls,
            [(1, 10.0, "return_unrecoverable", "return_unrecoverable", 6)],
        )
        self.assertEqual(
            node.portal_hypothesis_ledger.failed, [(1, 10.0, 7)]
        )
        self.assertEqual(
            node.events[-1][0],
            "portal_entry_return_unrecoverable_retired",
        )

    def test_branch_link_is_used_when_canonical_id_is_missing(self):
        node = _Node()
        auth = SimpleNamespace(branch_id=2)
        self.assertTrue(node.retire_unrecoverable_portal_entry(
            auth, {}, 20.0,
        ))
        self.assertEqual(node.place_work_items.calls[0][0], 1)
        self.assertEqual(node.place_work_items.calls[0][4], 6)

    def test_unlinked_branch_is_a_no_op(self):
        node = _Node()
        node.portal_hypothesis_ledger.record["gp_authentication_branch_id"] = 9
        auth = SimpleNamespace(branch_id=2)
        self.assertFalse(node.retire_unrecoverable_portal_entry(
            auth, {}, 30.0,
        ))
        self.assertEqual(node.place_work_items.calls, [])


if __name__ == "__main__":
    unittest.main()
