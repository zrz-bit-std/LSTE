import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from global_frontier_portal_authentication import (
    DoorwayAuthentication,
    JunctionDoorwayAuthentication,
    resolve_certified_probe_work_items,
)
from gp_exploration_tree import ExplorationTree
from global_frontier_execution_resolution import GlobalFrontierExecutionResolutionMixin


class PortalAuthenticationTest(unittest.TestCase):
    def test_moving_away_or_looping_cannot_renew_source_return_budget(self):
        auth=DoorwayAuthentication(1,1,(0.,0.),(2.,0.));auth.begin((-2.,0.))
        auth.state="returning"
        self.assertFalse(auth.physical_progress_expired((1.,0.),0.,0.,10.))
        self.assertFalse(auth.physical_progress_expired((2.,1.),0.,5.,10.))
        self.assertTrue(auth.physical_progress_expired((1.,0.),0.,10.,10.))
        self.assertFalse(auth.physical_progress_expired((.8,0.),0.,15.,10.))
        self.assertFalse(auth.physical_progress_expired((1.,0.),0.,20.,10.))
        self.assertTrue(auth.physical_progress_expired((1.,0.),0.,25.,10.))

    def test_withdraw_only_unstarted_geometry_and_allow_later_remeasurement(self):
        branch = DoorwayAuthentication(1, 1, (0., 0.), (2., 0.))
        task = JunctionDoorwayAuthentication(1, [branch])
        self.assertTrue(task.withdraw_pending(1, now=2.))
        self.assertNotIn(1, task.branches)
        self.assertEqual(task.phase, "complete")
        self.assertTrue(task.add(DoorwayAuthentication(1, 1, (0., 0.), (2., 0.)), now=3.))
        task.start_next((-2., 0.), now=5.)
        self.assertFalse(task.withdraw_pending(1, now=6.))
        self.assertEqual(task.active_branch_id, 1)
        self.assertEqual(task.branches[1].state, "entering")
        task.branches[1].state = "returning"
        self.assertFalse(task.withdraw_pending(1, now=7.))

    def test_measured_section_controls_entry_and_return_depth(self):
        auth=DoorwayAuthentication(1,1,(0.,0.),(2.,1.),required_depth=1.3,
                                   gate_geometry=(2.,((0.,-1.),(0.,1.))))
        auth.begin((-1.,0.))
        self.assertAlmostEqual(auth.route_goal()[1],0.)
        self.assertFalse(auth.update_entry((1.2,.5)))
        self.assertTrue(auth.update_entry((1.31,.5)))
        self.assertAlmostEqual(auth.signed_depth((1.2,.5)),1.2)

    def test_stalled_return_preserves_source_goal_and_records_failed_approach(self):
        auth = DoorwayAuthentication(41, 8, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "returning"
        auth.last_source_side_return_goal_odom = (-0.8, 0.1)
        auth._progress_phase = "returning"
        auth._progress_pose = (0.2, 0.0)
        auth._progress_at = 1.0

        self.assertTrue(auth.physical_progress_expired(
            (0.2, 0.0), 0.0, 11.0, 10.0,
        ))
        self.assertEqual(
            auth.failed_source_side_return_goals_odom, [],
        )
        self.assertEqual(len(auth.failed_return_approach_obstacles_odom), 1)
        self.assertIn(
            "doorway_return_stall_approach_marked",
            [event["event"] for event in auth.events],
        )

    def test_certified_negative_portal_retires_its_probe_owned_source_work_item(self):
        from global_frontier_portal_probe_ledger import PortalProbeLedger
        from global_frontier_work_items import PlaceWorkItemLedger

        work_items = PlaceWorkItemLedger(merge_radius=0.2)
        item_id = work_items.dispatch(1, (2.0, 3.0), 1.0)
        self.assertIsNotNone(item_id)
        probes = PortalProbeLedger(match_radius=0.3)
        probe = probes.observe(1, (1.0, 1.0), (1.0, 0.0), now=1.0)
        probe_id = probe["id"]
        probes.bind_work_item(probe_id, item_id)
        probes.bind_portal(probe_id, 7)
        probes.finish(probe_id, "certified", now=2.0)

        resolved = resolve_certified_probe_work_items(
            probes, work_items, 7, 3.0, "negative_semantic_room_priority",
        )

        self.assertEqual(resolved, [item_id])
        current = next(row for row in work_items.snapshot() if row["id"] == item_id)
        self.assertEqual(current["state"], "resolved")
        self.assertEqual(probes.get(probe_id)["state"], "certified")
        self.assertEqual(work_items.unresolved_count(1), 0)

    def test_semantic_entry_replaces_probe_work_item_without_being_resolved(self):
        from global_frontier_portal_probe_ledger import PortalProbeLedger
        from global_frontier_work_items import PlaceWorkItemLedger

        work_items = PlaceWorkItemLedger(merge_radius=0.2)
        source_item_id = work_items.dispatch(1, (2.0, 3.0), 1.0)
        probes = PortalProbeLedger(match_radius=0.3)
        probe = probes.observe(1, (1.0, 1.0), (1.0, 0.0), now=1.0)
        probes.bind_work_item(probe["id"], source_item_id)
        probes.bind_portal(probe["id"], 8)
        probes.finish(probe["id"], "certified", now=2.0)
        entry_id = work_items.ensure_portal_entry(8, 1, (4.0, 5.0), 1, 0.9, 2.0)

        resolved = resolve_certified_probe_work_items(
            probes, work_items, 8, 3.0, "portal_entry_work_item_created",
        )

        states = {item["id"]: item["state"] for item in work_items.snapshot()}
        self.assertEqual(resolved, [source_item_id])
        self.assertEqual(states[source_item_id], "resolved")
        self.assertEqual(states[entry_id], "unresolved")

    def test_late_branch_is_queued_without_mutating_active_transaction(self):
        task = JunctionDoorwayAuthentication(7, settle_seconds=0.0)
        first = DoorwayAuthentication(70, 7, (0.0, 0.0), (1.0, 0.0))
        late = DoorwayAuthentication(71, 7, (0.0, 1.0), (1.0, 1.0))
        task.add(first, now=1.0)
        self.assertIs(task.start_next((0.0, 0.0), now=1.0), first)
        # The active arm keeps its lease and phase, but the late stable arm is
        # queued instead of being silently dropped.
        self.assertTrue(task.add(late, now=2.0))
        self.assertEqual(set(task.branches), {70, 71})
        self.assertEqual(task.phase, "authenticating")
        self.assertEqual(task.active_branch_id, 70)
        first.state = "completed"
        self.assertIs(task.finish_active((0.0, 0.0), now=3.0), late)

    def test_completed_gap_is_rejected_when_source_node_is_on_destination_side(self):
        valid = DoorwayAuthentication(20, 2, (0.0, 0.0), (1.0, 0.0))
        valid.begin((-0.8, 0.0))
        valid.state = "completed"
        self.assertTrue(valid.source_side_proven())

        false_portal = DoorwayAuthentication(21, 2, (-2.65, 0.48), (-5.05, 0.15))
        false_portal.begin((-2.95, 0.57))
        false_portal.state = "completed"
        self.assertFalse(false_portal.source_side_proven())
        self.assertTrue(false_portal.reject_completed_portal_candidate(
            "portal_source_side_geometry_unproven",
        ))
        self.assertEqual(false_portal.state, "failed")
        self.assertEqual(false_portal.failure_reason, "portal_source_side_geometry_unproven")

    def test_full_entry_locked_turn_and_return(self):
        branch = DoorwayAuthentication(4, 9, (0, 0), (1, 0), .8)
        self.assertTrue(branch.begin((-.5, 0)))
        self.assertAlmostEqual(branch.route_goal()[0],
                               branch.required_depth + branch.entry_goal_margin)
        self.assertGreater(branch.route_goal()[0], branch.required_depth + .55)
        self.assertFalse(branch.update_entry((.3, 0)))
        self.assertTrue(branch.update_entry((.9, 0)))
        self.assertTrue(branch.observe_inside(2, 5, 0.0, [-1.0]))
        locked = branch.target_yaw
        self.assertEqual(branch.turn_direction, "clockwise")
        self.assertAlmostEqual(branch.target_yaw, -(3.141592653589793 - 0.08))
        self.assertFalse(branch.update_rotation(-.5))
        self.assertFalse(branch.update_rotation(.5))
        self.assertAlmostEqual(branch.target_yaw, locked)
        self.assertFalse(branch.update_rotation(locked))
        self.assertFalse(branch.update_rotation(locked))
        self.assertTrue(branch.update_rotation(locked))
        self.assertEqual(branch.state, "turning")
        self.assertTrue(branch.finish_turn(locked))
        self.assertEqual(branch.route_goal(), (-1.1, 0))
        self.assertTrue(branch.update_return((-.5, 0)))
        self.assertEqual(branch.state, "completed")

    def test_return_goal_clears_gate_when_source_pose_is_near_destination_side(self):
        branch = DoorwayAuthentication(
            9, 4, (-2.68, 0.54), (-5.05, 0.15),
            return_goal_margin=0.60,
        )
        branch.begin((-3.25, 0.57))
        branch.state = "returning"
        goal = branch.route_goal()
        self.assertLess(branch.signed_depth(goal), -0.59)
        self.assertGreater(goal[0], branch.gate_xy[0])

    def test_return_clears_misplaced_gate_then_targets_junction_node(self):
        branch = DoorwayAuthentication(
            4, 2, (-2.76, 0.53), (-5.12, 0.14), return_radius=0.75,
        )
        branch.begin((-3.85, 0.45))
        branch.state = "returning"
        first_goal = branch.route_goal()
        self.assertLess(branch.signed_depth(first_goal), -0.59)
        self.assertFalse(branch.update_return((-2.0, 0.50)))
        self.assertTrue(branch.return_gate_cleared)
        self.assertEqual(branch.route_goal(), branch.source_xy)
        self.assertTrue(branch.update_return(branch.source_xy))
        self.assertEqual(branch.state, "completed")

    def test_return_to_node_after_crossing_gate_when_node_is_source_side(self):
        branch = DoorwayAuthentication(
            12, 5, (-0.14, 0.0), (1.0, 0.0), return_radius=0.75,
        )
        branch.begin((-0.14, 0.0))
        branch.state = "returning"

        # The robot crossed back onto the source side, but has not reached
        # the junction yet. The route must now target the node itself.
        self.assertFalse(branch.update_return((-1.0, 0.0)))
        self.assertTrue(branch.return_gate_cleared)
        self.assertEqual(branch.route_goal(), branch.source_xy)

        self.assertTrue(branch.update_return(branch.source_xy))
        self.assertEqual(branch.state, "completed")
        self.assertEqual(
            branch.events[-1]["event"], "returned_to_junction_node"
        )

    def test_return_tolerates_small_localization_error_at_junction(self):
        branch = DoorwayAuthentication(1, 2, (0, 0), (1, 0), .5)
        branch.begin((-1, 0))
        branch.update_entry((1, 0))
        branch.observe_inside(1, 0, 0.0, [0.0])
        branch.update_rotation(branch.target_yaw)
        branch.update_rotation(branch.target_yaw)
        branch.update_rotation(branch.target_yaw)
        branch.finish_turn(branch.target_yaw)

        # Near the node radius is insufficient if the robot remains beyond
        # the doorway plane; completion requires crossing back to its source.
        self.assertFalse(branch.update_return((0.1, 0.0)))
        self.assertTrue(branch.update_return((-0.35, 0.0)))
        self.assertEqual(branch.state, "completed")

    def test_entry_waits_for_body_and_endpoint_clearance(self):
        branch = DoorwayAuthentication(1, 2, (0, 0), (2, 0), required_depth=0.75)
        branch.begin((-1, 0))
        self.assertFalse(branch.update_entry((0.4, 0)))
        self.assertTrue(branch.update_entry((0.76, 0)))

    def test_successful_return_goal_clears_the_doorway_plane(self):
        branch = DoorwayAuthentication(1, 2, (0, 0), (1, 0))
        branch.begin((-0.5, 0))
        branch.update_entry((0.5, 0))
        branch.observe_inside(1, 0, 0.0, [0.0])
        branch.update_rotation(branch.target_yaw)
        branch.update_rotation(branch.target_yaw)
        branch.update_rotation(branch.target_yaw)
        branch.finish_turn(branch.target_yaw)
        self.assertEqual(branch.route_goal(), (-1.1, 0.0))

    def test_no_frontier_rejects_candidate(self):
        branch = DoorwayAuthentication(1, 2, (0, 0), (1, 0), .5)
        branch.begin((-1, 0))
        branch.update_entry((1, 0))
        self.assertTrue(branch.observe_inside(0, 0, 0.0, []))
        self.assertEqual(branch.state, "returning")
        self.assertEqual(branch.route_goal(), (-1.0, 0.0))
        self.assertTrue(branch.update_return((-1.0, 0.0)))
        self.assertEqual(branch.state, "failed")
        self.assertEqual(branch.failure_reason, "no_frontier_inside_doorway")

    def test_failed_probe_requires_crossing_back_to_source_side_before_terminal(self):
        branch = DoorwayAuthentication(
            9, 4, (0, 0), (1, 0), return_radius=0.75,
        )
        branch.begin((-0.2, 0))
        branch.state = "returning"
        branch.failure_reason = "authentication_physical_stall:inside"
        self.assertFalse(branch.update_return((0.2, 0)))
        self.assertEqual(branch.state, "returning")
        self.assertFalse(branch.return_gate_cleared)
        self.assertTrue(branch.update_return((-0.2, 0)))
        self.assertEqual(branch.state, "failed")
        self.assertTrue(branch.return_gate_cleared)

    def test_bounded_failed_return_retries_block_without_finishing_or_advancing(self):
        first = DoorwayAuthentication(19, 4, (0, 0), (1, 0))
        sibling = DoorwayAuthentication(20, 4, (0, 0), (0, 1))
        task = JunctionDoorwayAuthentication(4, (first, sibling), settle_seconds=0)
        task.start_next((-1, 0), now=0)
        first.state = "returning"
        first.failure_reason = "authentication_physical_stall:turning"
        self.assertFalse(first.physical_progress_expired(
            (0.4, 0), 0.0, 0, 10.0,
        ))
        for now in (10, 20, 30):
            self.assertTrue(first.physical_progress_expired(
                (0.4, 0), 0.0, now, 10.0,
            ))
        self.assertEqual(first.state, "returning")
        self.assertTrue(first.return_recovery_blocked)
        self.assertFalse(task.all_finished())
        self.assertIsNone(task.finish_active((-1, 0), now=31))
        self.assertEqual(task.active_branch_id, first.branch_id)
        self.assertEqual(first.return_retry_count, 3)

    def test_return_controller_failure_keeps_branch_unfinished_for_fallback(self):
        first = DoorwayAuthentication(21, 4, (0, 0), (1, 0))
        first.begin((-1, 0))
        first.state = "returning"
        first.failure_reason = "no_frontier_inside_doorway"
        events = []
        first._event = lambda event, **fields: events.append((event, fields))
        self.assertTrue(first.note_return_route_failure("controller_stall_no_physical_progress"))
        self.assertEqual(first.state, "returning")
        self.assertEqual(getattr(first, "return_retry_count", 0), 0)
        self.assertTrue(first.return_recovery_blocked)
        self.assertFalse(getattr(first, "return_recovery_final_blocked", False))
        self.assertEqual(first.failure_reason, "no_frontier_inside_doorway")
        self.assertEqual(events[-1][0], "doorway_return_controller_failure")
        self.assertFalse(any(name.startswith("returned_to_junction") for name, _ in events))

    def test_observation_period_waits_for_stable_branch_set(self):
        first = DoorwayAuthentication(2, 7, (0, 0), (1, 0), .5)
        task = JunctionDoorwayAuthentication(7, settle_seconds=1.0)
        task.add(first, now=10.0)
        self.assertIsNone(task.start_next((0, 0), now=10.9))
        self.assertEqual(task.phase, "watching")
        self.assertEqual(task.start_next((0, 0), now=11.0).branch_id, 2)

    def test_finished_branch_advances_once_and_preserves_failure(self):
        first = DoorwayAuthentication(2, 7, (0, 0), (1, 0), .5)
        second = DoorwayAuthentication(3, 7, (0, 0), (0, 1), .5)
        task = JunctionDoorwayAuthentication(7, (first, second), settle_seconds=0)
        self.assertEqual(task.start_next((-1, 0), now=1).branch_id, 2)
        first.update_entry((1, 0))
        first.observe_inside(1, 0, 0.0, [0.2])
        first.update_rotation(first.target_yaw)
        first.update_rotation(first.target_yaw)
        first.update_rotation(first.target_yaw)
        first.finish_turn(first.target_yaw)
        first.update_return((-1, 0))
        self.assertEqual(task.finish_active((-1, -1), now=2).branch_id, 3)
        second.update_entry((0, 1))
        second.observe_inside(0, 0, 0.0, [])
        self.assertEqual(second.state, "returning")
        self.assertTrue(second.update_return((-1, -1)))
        self.assertTrue(task.all_finished())
        self.assertEqual(task.snapshot()["branches"][1]["failure_reason"],
                         "no_frontier_inside_doorway")

    def test_new_branch_waits_for_observation_window(self):
        first = DoorwayAuthentication(1, 4, (0, 0), (1, 0), .5)
        task = JunctionDoorwayAuthentication(4, (first,), settle_seconds=1.0)
        self.assertEqual(task.start_next((-1, 0), now=2.0).branch_id, 1)
        first.fail("test_failure")
        second = DoorwayAuthentication(2, 4, (0, 0), (0, 1), .5)
        task.add(second, now=2.5)
        self.assertIsNone(task.finish_active((-1, 0), now=2.6))
        self.assertEqual(task.phase, "watching")
        self.assertEqual(task.start_next((-1, 0), now=3.5).branch_id, 2)

    def test_rejected_door_preserves_tree_lineage(self):
        tree = ExplorationTree()
        tree.nodes = {
            1: {"id": 1, "branch_ids": [5]},
            2: {"id": 2, "parent_branch_id": 5},
        }
        tree.branches = {
            5: {"id": 5, "node_id": 1, "destination_node_id": 2,
                "destination_space_id": 8, "state": "active"},
        }
        tree.spaces = {8: {"id": 8, "parent_branch_id": 5, "node_ids": [2]}}
        tree.reject_branch_authentication(5, "no_frontier_inside_doorway")
        self.assertEqual(tree.branches[5]["state"], "authentication_failed")
        self.assertEqual(tree.nodes[2]["parent_branch_id"], 5)
        self.assertEqual(tree.nodes[2]["rejected_by_branch_id"], 5)
        self.assertEqual(tree.spaces[8]["rejected_by_branch_id"], 5)
        self.assertIn(5, tree.nodes[1]["branch_ids"])


class PendingGeometryFreezeTest(unittest.TestCase):
    def test_waiting_branch_updates_geometry_but_auto_start_freezes_it(self):
        first=DoorwayAuthentication(1,1,(0.,0.),(2.,0.))
        waiting=DoorwayAuthentication(2,1,(0.,0.),(2.,0.))
        task=JunctionDoorwayAuthentication(1,[first,waiting],settle_seconds=0.)
        task.start_next((-1.,0.),now=0.)
        bounds=[[1.,-1.],[1.,1.]]
        self.assertTrue(waiting.refresh_pending_geometry((1.,0.),(3.,0.),2.,bounds))
        bounds[0][0]=99.
        first.state='completed'
        self.assertIs(task.finish_active((-1.,0.),now=1.),waiting)
        self.assertEqual(waiting.gate_xy,(1.,0.))
        self.assertEqual(waiting.gate_geometry,(2.,((1.,-1.),(1.,1.))))
        self.assertFalse(waiting.refresh_pending_geometry((2.,0.),(4.,0.),3.,[[2.,-1.],[2.,1.]]))
        self.assertEqual(waiting.gate_xy,(1.,0.))



class ObservationCompletionRecordTest(unittest.TestCase):
    def test_final_branch_records_complete_without_next_map_callback(self):
        from global_frontier_portal_authentication import GlobalFrontierPortalAuthenticationMixin
        node=GlobalFrontierPortalAuthenticationMixin();node.initialize_portal_authentication()
        records=[];node.publish_status=lambda event,**data:records.append((event,data))
        auth=DoorwayAuthentication(4,2,(0.,0.),(2.,0.))
        task=JunctionDoorwayAuthentication(2,[auth],settle_seconds=0.)
        node.junction_portal_authentication=task
        task.start_next((-2.,0.),now=1.);auth.fail('empty_inside')
        node.finish_portal_authentication_branch(now=2.)
        updates=[d for e,d in records if e=='portal_authentication_observation_period']
        self.assertEqual(len(updates),1)
        self.assertEqual(updates[0]['phase'],'complete');self.assertTrue(updates[0]['all_finished'])
        self.assertEqual(task.active_branch_id,None)


if __name__ == "__main__":
    unittest.main()
