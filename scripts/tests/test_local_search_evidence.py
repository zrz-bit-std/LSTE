"""Check retry timing and that failed trials never acquire invented evidence."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from repeat_local_search import StationaryWatch
from plot_world_run import extract_run, failure_caption


class SearchEvidenceTest(unittest.TestCase):
    def test_stationary_timer_ignores_rotation_and_small_odometry_noise(self):
        watch = StationaryWatch(seconds=30)
        watch.observe([1, 2, 0], 10)
        watch.observe([1.05, 2.01, 2], 20)
        self.assertFalse(watch.stalled(40))
        self.assertTrue(watch.stalled(40.01))
        watch.observe([1.5, 2, 2], 41)
        self.assertFalse(watch.stalled(42))

    def test_failure_without_target_remains_missing(self):
        records = [('run_start', {'ros_time': 10}),
                   ('sample', {'ros_time': 12, 'pose': [1, 2, 0], 'path_length': 2}),
                   ('task_done', {'ros_time': 13, 'value': False}),
                   ('run_stop', {'summary': {'ros_time': 14, 'pose': [2, 2, 0],
                                             'path_length': 3}, 'wall_duration_seconds': 180})]
        data = extract_run(records, {'status': 'failed'})
        self.assertIsNone(data['done'])
        self.assertIsNone(data['first_pose'])
        self.assertIsNone(data['matched'])
        self.assertIsNone(data['evidence'])
        self.assertEqual(data['outcome'], 'failed')
        self.assertEqual(data['duration'], 4)

    def test_success_uses_completion_boundary_not_later_stop(self):
        records = [('run_start', {'ros_time': 10}),
                   ('sample', {'ros_time': 12, 'pose': [1, 2, 0], 'path_length': 2}),
                   ('task_done', {'ros_time': 14, 'value': True, 'completion_evidence': 'test'}),
                   ('task_completed', {'completion_snapshot': {
                       'boundary_ros_time': 14, 'boundary_wall_elapsed_seconds': 8,
                       'metrics': {'pose': [2, 2, 0], 'path_length': 3}}}),
                   ('sample', {'ros_time': 15, 'pose': [9, 2, 0], 'path_length': 10})]
        data = extract_run(records)
        self.assertEqual(data['samples'][-1]['pose'], [2, 2, 0])
        self.assertEqual(data['wall'], 8)
        self.assertEqual(data['evidence'], 'test')
        self.assertEqual(len(data['samples']), 2)

    def test_late_success_does_not_override_runner_timeout(self):
        records = [('run_start', {'ros_time': 10}),
                   ('task_done', {'ros_time': 100, 'value': True})]
        data = extract_run(records, {'status':'failed','reason':'exploration_over_600_wall_seconds'})
        self.assertEqual(data['outcome'],'failed')
        self.assertIsNone(data['done'])
        self.assertTrue(data['recorded_task_done_true'])

    def test_unready_clock_does_not_add_saved_world_timestamp_to_duration(self):
        records = [('run_start', {'ros_time':0}),
                   ('sample', {'ros_time':2100,'pose':[1,2,0],'path_length':0}),
                   ('run_stop',{'summary':{'ros_time':2200,'pose':[3,2,0],'path_length':2}})]
        data=extract_run(records,{'status':'failed'})
        self.assertEqual(data['duration'],100)
        self.assertEqual(data['t0'],2100)
        self.assertNotEqual(data['time_origin'],'run_start')

    def test_failure_caption_includes_reason_location_and_logged_diagnosis(self):
        caption = failure_caption(
            {'reason':'stationary_over_50_wall_seconds','stall_pose':[3.2,-1.4,0.5],
             'stationary_radius_m':0.35},
            {'path_length':12.5}, {'summary':{}},
            {'diagnosis':{'primary_cause':'planner_materialization_stall'}}, 42, 81.5)
        self.assertIn('No translation >50 s', caption)
        self.assertIn('(3.20, -1.40) m', caption)
        self.assertIn('12.5 m traveled', caption)
        self.assertIn('planner materialization stall', caption)


if __name__ == '__main__':
    unittest.main()
