"""Moving outside a child must never rename the parent's component as that child."""
from pathlib import Path
from types import SimpleNamespace as S
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from global_frontier_selection_context import GlobalFrontierSelectionContextMixin
from global_frontier_place_dfs import PlaceDfsTraversal


class PlacePartitionAnchorTest(unittest.TestCase):
    def setUp(self):
        self.node = GlobalFrontierSelectionContextMixin()
        self.node.transform_xy = lambda target, source, x, y: (x, y)
        self.node.gp_portal_assistance_enabled = True
        self.node.place_dfs = PlaceDfsTraversal()
        self.node.place_dfs.enter(1, 2, 7)
        self.portal = dict(id=7, source_place_id=1, destination_place_id=2,
                           state='crossed', gp_authentication_state='completed',
                           physical_gate=[10., 10.], physical_destination=[14., 10.],
                           physical_bounds=[[10., 8.], [10., 12.]], opening_width=4.)
        self.records = [self.portal]
        self.node.portal_hypothesis_ledger = S(snapshot=lambda: self.records)
        self.free = np.zeros((20, 30), dtype=bool)
        self.free[8:12, 1:28] = True
        self.message = S(header=S(frame_id='map', stamp=S(to_sec=lambda: 1.)),
                         info=S(resolution=1., origin=S(position=S(x=0., y=0.))))

    def update(self, robot, epoch=1):
        return self.node._update_place_partition(self.message, self.free,
            np.zeros_like(self.free), 2, robot, epoch)

    def test_partition_uses_one_map_stamp_projector_not_latest_tf(self):
        calls = []
        def factory(target, source, stamp=None):
            calls.append((target, source, stamp))
            return lambda x, y: (x, y)
        self.node.planar_xy_projector = factory
        self.node.transform_xy = lambda *args: (_ for _ in ()).throw(
            AssertionError("latest TF must not project a stamped map"))
        self.update((3., 10.))
        self.assertTrue(self.node.current_place_scope_valid)
        self.assertEqual(calls, [("map", "odom", self.message.header.stamp)])
        self.assertEqual(self.node.current_place_scope_seed_xy, [14., 10.])

    def test_unchanged_map_retains_exact_tf_after_buffer_eviction(self):
        calls = []
        def factory(*args, **kwargs):
            calls.append(kwargs["stamp"])
            return (lambda x, y: (x, y)) if len(calls) == 1 else None
        self.node.planar_xy_projector = factory
        self.update((14., 10.))
        self.update((20., 10.), 2)
        self.assertTrue(self.node.current_place_scope_valid)
        self.assertEqual(len(calls), 1)
        self.assertEqual(self.node.current_place_scope_seed_xy, [14., 10.])
        self.message.header.stamp = S(to_sec=lambda: 20.)
        self.update((20., 10.), 3)
        self.assertFalse(self.node.current_place_scope_valid)
        self.assertEqual(len(calls), 2)

    def test_handoff_reprojects_same_raster_without_losing_exact_tf(self):
        from global_frontier_portal_arrival_commit import GlobalFrontierPortalArrivalCommitMixin
        calls=[]
        def factory(*args,**kwargs):
            calls.append(1)
            return (lambda x,y:(x,y)) if len(calls)==1 else None
        self.node.planar_xy_projector=factory
        self.update((14.,10.))
        self.assertTrue(self.node.current_place_scope_valid)
        GlobalFrontierPortalArrivalCommitMixin._invalidate_place_scope_for_handoff(self.node,2)
        self.assertFalse(self.node.current_place_scope_valid)
        self.assertIsNone(self.node._place_partition_key)
        self.update((14.,10.))
        self.assertTrue(self.node.current_place_scope_valid)
        self.assertIsNone(self.node.current_place_scope_invalidated_reason)
        self.assertEqual(len(calls),1)
        self.assertEqual(self.node.current_place_scope_id,2)
        self.assertEqual(self.node.current_place_scope_seed_xy,[14.,10.])

    def test_missing_map_tf_is_retried_until_it_arrives(self):
        calls = []
        def factory(*args, **kwargs):
            calls.append(1)
            return None if len(calls) == 1 else (lambda x, y: (x, y))
        self.node.planar_xy_projector = factory
        self.update((14., 10.))
        self.assertFalse(self.node.current_place_scope_valid)
        self.update((14., 10.), 2)
        self.assertTrue(self.node.current_place_scope_valid)

    def test_missing_exact_map_tf_does_not_fall_back_to_latest_tf(self):
        self.node.planar_xy_projector = lambda *args, **kwargs: None
        self.update((14., 10.))
        self.assertFalse(self.node.current_place_scope_valid)
        self.assertFalse(self.node.current_place_scope_mask.any())

    def test_geometry_projector_passes_requested_stamp_to_tf_buffer(self):
        from global_frontier_geometry import GlobalFrontierGeometryMixin
        calls = []
        transform = S(transform=S(rotation=S(x=0., y=0., z=0., w=1.),
                                  translation=S(x=2., y=3., z=0.)))
        node = GlobalFrontierGeometryMixin()
        node.tf_buffer = S(lookup_transform=lambda *args: calls.append(args) or transform)
        stamp = self.message.header.stamp
        self.assertEqual(tuple(node.planar_xy_projector("map", "odom", stamp)(1., 2.)), (3., 5.))
        self.assertIs(calls[0][2], stamp)

    def test_robot_detour_and_new_map_cannot_move_child_to_parent_side(self):
        self.update((12., 10.))
        first = self.node.current_place_scope_mask.copy()
        self.update((3., 10.), 2)
        np.testing.assert_array_equal(first, self.node.current_place_scope_mask)
        self.assertTrue(self.node.current_place_scope_mask[10, 20])
        self.assertFalse(self.node.current_place_scope_mask[10, 3])
        self.assertEqual(self.node.current_place_scope_seed_xy, [14., 10.])
        self.assertEqual(self.node.current_place_scope_entry_portal_id, 7)

    def test_missing_entry_does_not_fall_back_to_robot_component(self):
        self.records.clear()
        self.update((3., 10.))
        self.assertFalse(self.node.current_place_scope_valid)
        self.assertFalse(self.node.current_place_scope_mask.any())

    def test_missing_gate_projection_cannot_authorize_whole_map(self):
        self.node.transform_xy = lambda t, s, x, y: None if x == 10. else (x, y)
        self.update((3., 10.))
        self.assertFalse(self.node.current_place_scope_valid)

    def test_map_correction_reprojects_same_physical_entrance(self):
        self.node.transform_xy = lambda t, s, x, y: (x + 1., y)
        self.update((3., 10.))
        self.assertEqual(self.node.current_place_scope_seed_xy, [15., 10.])
        self.assertTrue(self.node.current_place_scope_mask[10, 20])
        self.assertFalse(self.node.current_place_scope_mask[10, 3])

    def test_bound_child_portal_leak_clips_to_current_source_place(self):
        # A connected SLAM free-space loop around a finite measured cut must
        # not put the already-entered child Place cells in the parent mask.
        self.free[:] = False
        self.free[2:18, 1:28] = True
        child = dict(
            id=8, source_place_id=2, destination_place_id=3,
            state="crossed", gp_authentication_state="completed",
            physical_gate=[20., 10.], physical_destination=[24., 10.],
            physical_bounds=[[20., 8.], [20., 12.]], opening_width=4.,
        )
        self.records.append(child)
        self.update((15., 10.))
        self.assertTrue(self.node.current_place_scope_valid)
        self.assertNotIn(8, self.node.current_place_scope_leaking_portal_ids)
        self.assertTrue(self.node.current_place_scope_mask[10, 15])
        self.assertFalse(self.node.current_place_scope_mask[10, 25])

    def test_side_clip_removes_fragments_connected_only_through_parent(self):
        self.free[:] = False
        self.free[2:18, 1:28] = True
        self.free[12, 9:28] = False
        self.update((14., 10.))
        self.assertTrue(self.node.current_place_scope_valid)
        self.assertTrue(self.node.current_place_scope_mask[10, 14])
        self.assertFalse(self.node.current_place_scope_mask[15, 14])
        self.assertNotEqual(self.node.current_place_scope_components.labels[15, 14],
                            self.node.current_place_scope_label)

    def test_certified_unbound_child_bounds_source_without_creating_place(self):
        candidate = dict(
            id=8, source_place_id=2, destination_place_id=None,
            state='certified', gp_authentication_state='completed',
            physical_gate=[20.,10.],physical_destination=[24.,10.],
            physical_bounds=[[20.,8.],[20.,12.]],opening_width=4.)
        self.records.append(candidate)
        self.update((14.,10.))
        self.assertTrue(self.node.current_place_scope_valid)
        self.assertEqual(self.node.current_place_scope_portal_ids,[7,8])
        self.assertTrue(self.node.current_place_scope_mask[10,14])
        self.assertFalse(self.node.current_place_scope_mask[10,24])
        self.assertIsNone(candidate['destination_place_id'])
        self.assertEqual(set(self.node.place_dfs.nodes),{1,2})

    def test_uncertified_unbound_proposal_does_not_cut_place(self):
        self.records.append(dict(id=8,source_place_id=2,destination_place_id=None,
            state='certified',gp_authentication_state='pending',
            physical_gate=[20.,10.],physical_destination=[24.,10.],
            physical_bounds=[[20.,8.],[20.,12.]],opening_width=4.))
        self.update((14.,10.))
        self.assertEqual(self.node.current_place_scope_portal_ids,[7])
        self.assertTrue(self.node.current_place_scope_mask[10,24])

    def test_unmapped_entrance_does_not_claim_other_known_component(self):
        self.free[10, 14] = False
        self.update((3., 10.))
        self.assertFalse(self.node.current_place_scope_valid)

if __name__ == '__main__':
    unittest.main()
