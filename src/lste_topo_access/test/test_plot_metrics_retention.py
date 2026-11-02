"""Plotting keeps measured evidence, not every unrelated planner snapshot."""
from pathlib import Path
import runpy,tempfile,json,unittest
MODULE=Path(__file__).resolve().parents[3]/'scripts/tools/plot_portal_authentication.py'
read_metrics=runpy.run_path(str(MODULE))['read_metrics']
class PlotMetricsRetentionTest(unittest.TestCase):
    def test_projection_preserves_all_pose_dispatch_and_final_tree_evidence(self):
        records=[('sample',dict(pose=[1,2,3],ros_time=1,irrelevant=list(range(1000)))),
                 ('global_frontier_event',dict(frontier_event='work_item_dispatched',work_item_id=2,
                   work_items=[dict(id=1),dict(id=2,attempts=[dict(id=3,viewpoint_xy=[1,2])])],
                   graph_route_plan=dict(work_item_tour=[2,4]),place_dfs=dict(stack=[1,2]))),
                 ('global_frontier_event',dict(frontier_event='gp_branch_tree',tree=dict(nodes=[1]),
                   portal_hypotheses=dict(records=[dict(id=7)]),place_dfs=dict(stack=[1])))]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metrics.log'
            path.write_text(''.join('x event='+e+' data='+json.dumps(d)+'\n' for e,d in records))
            events,poses,tree,dedup=read_metrics(path)
            self.assertEqual(poses,[dict(pose=[1,2,3],ros_time=1)])
            self.assertEqual(events[0][1]['work_items'][0]['id'],2)
            self.assertEqual(events[0][1]['graph_route_plan']['work_item_tour'],[2,4])
            self.assertEqual(events[-1][1]['place_dfs']['stack'],[1])
            self.assertEqual(tree,dict(nodes=[1]))
            self.assertEqual(events[-1][1]['portal_hypotheses']['records'][0]['id'],7)
            path.write_text('x event=sample data={broken}\n')
            with self.assertRaises(ValueError):read_metrics(path)
if __name__=='__main__':unittest.main()
