"""Geometry-based regressions for GP proposals, including unframed T exits."""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from gp_branch_discovery import (PolarGaussianProcess, entrance_geometry, angle_delta,
                                 map_ranges, junction_anchor, has_local_fork,
                                 has_confirmed_three_way_fan)


def scene_scan(grid, source, resolution=0.05, count=720):
    angles = np.linspace(-math.pi, math.pi, count, endpoint=False)
    result = []
    for angle in angles:
        distance = 12.0
        for depth in np.arange(resolution, 12, resolution):
            col = int((source[0]+depth*math.cos(angle))/resolution)
            row = int((source[1]+depth*math.sin(angle))/resolution)
            if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
                break
            if grid[row,col] >= 50:
                distance = depth
                break
        result.append(distance)
    return result


def t_map():
    grid=np.full((200,240),100,dtype=np.int8)
    grid[75:125,20:220]=0
    grid[100:190,140:180]=0
    return grid


class GPBranchDiscoveryTest(unittest.TestCase):
    def test_t_has_north_and_east_without_east_door_frame(self):
        grid=t_map(); source=(8.0,5.0)
        candidates=PolarGaussianProcess().propose(scene_scan(grid,source),-math.pi,
                         2*math.pi/720,0.05,12)
        for heading in (0,math.pi/2):
            self.assertTrue(any(abs(angle_delta(x.heading,heading))<0.35
                                for x in candidates),candidates)
        east=entrance_geometry(grid,0.05,(0,0),source,0,3,0.35)
        self.assertEqual(east['passability'],'open')
        self.assertAlmostEqual(east['width'],2.5,delta=0.15)

    def test_oblique_t_view_triggers_slam_anchor_refinement(self):
        grid=t_map(); resolution=0.05; source=(6.2,5.0)
        model=PolarGaussianProcess()
        sensor=scene_scan(grid,source,resolution,720)
        proposals=model.propose(sensor,-math.pi,2*math.pi/720,0.05,12)
        headings=[p.heading for p in proposals]
        self.assertGreaterEqual(len(headings),3,proposals)
        self.assertTrue(has_local_fork(headings),headings)
        anchor=junction_anchor(grid,resolution,(0,0),source,radius=2.0)
        projected=map_ranges(grid,resolution,(0,0),anchor,model.angles,8.0)
        refined=model.propose(projected,-math.pi,2*math.pi/len(projected),
                             0.0,8.0,minimum_width=0.7)
        for heading in (0,math.pi/2,math.pi):
            self.assertTrue(any(abs(angle_delta(p.heading,heading))<0.35
                                for p in refined),refined)

    def test_oblique_three_lobe_fan_is_not_yet_a_confirmed_junction(self):
        self.assertFalse(has_confirmed_three_way_fan([-0.24, 0.50, math.pi]))
        self.assertTrue(has_confirmed_three_way_fan([0.0, math.pi/2, math.pi]))
        self.assertTrue(has_confirmed_three_way_fan([0.7, 0.7+math.pi/2, 0.7+math.pi]))

    def test_confirmed_fan_search_is_independent_of_candidate_order(self):
        from itertools import permutations
        # Captured from L node14 at 3450.966: arms 39/40/41/42. The old greedy
        # dedup selected the wrong representatives and missed valid 39/41/42.
        headings=(0.2814141914, 2.5960562598, -2.8734918216, 1.7038057519)
        for order in permutations(headings):
            self.assertTrue(has_confirmed_three_way_fan(order),order)
        self.assertFalse(has_confirmed_three_way_fan((0.2814, 0.40, 0.51, 0.62)))

    def test_dead_end_corners_are_one_branch(self):
        grid=np.full((200,240),100,dtype=np.int8)
        grid[75:125,20:220]=0
        candidates=PolarGaussianProcess().propose(scene_scan(grid,(3,5)),
                         -math.pi,2*math.pi/720,0.05,12)
        self.assertLessEqual(len(candidates),2,candidates)

    def test_cross_junction_has_four_directions(self):
        grid=t_map();grid[10:100,140:180]=0
        candidates=PolarGaussianProcess().propose(scene_scan(grid,(8,5)),
                         -math.pi,2*math.pi/720,0.05,12)
        for heading in (0,math.pi/2,math.pi,-math.pi/2):
            self.assertTrue(any(abs(angle_delta(x.heading,heading))<0.35
                                for x in candidates),candidates)

    def test_l_junction_retains_turn_without_frame(self):
        grid=t_map();grid[75:125,180:220]=100
        candidates=PolarGaussianProcess().propose(scene_scan(grid,(8,5)),
                         -math.pi,2*math.pi/720,0.05,12)
        for heading in (math.pi/2,math.pi):
            self.assertTrue(any(abs(angle_delta(x.heading,heading))<0.35
                                for x in candidates),candidates)
        self.assertFalse(any(abs(angle_delta(x.heading,0))<0.35 for x in candidates))

    def test_unknown_is_not_passable_or_a_measured_width(self):
        grid=np.full((80,80),-1,dtype=np.int8)
        grid[38:43,10:65]=0
        geometry=entrance_geometry(grid,0.05,(0,0),(1,2),0,2,0.35)
        self.assertEqual(geometry['passability'],'unknown')
        self.assertIsNone(geometry['width'])

    def test_missing_scan_is_not_free_space(self):
        self.assertEqual(PolarGaussianProcess().propose([float('nan')]*720,
                         -math.pi,2*math.pi/720,0.05,12),[])

    def test_slam_throat_uses_the_same_body_width_as_gp_admission(self):
        grid=np.full((80,100),100,dtype=np.int8)
        grid[20:30,5:95]=0
        geometry=entrance_geometry(grid,0.1,(0,0),(1,2.5),0,2,0.52)
        self.assertGreaterEqual(geometry['width'],1.5*0.52)
        self.assertLess(geometry['width'],2*0.52)
        self.assertEqual(geometry['passability'],'blocked')

    def test_narrow_passage_is_blocked(self):
        grid=np.full((80,100),100,dtype=np.int8);grid[37:43,5:95]=0
        geometry=entrance_geometry(grid,0.05,(0,0),(1,2),0,2,0.35)
        self.assertEqual(geometry['passability'],'blocked')

if __name__=='__main__':unittest.main()
