"""A controller lease wait must not bypass the physical DFS return budget."""
from pathlib import Path
import sys,unittest
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.modules.setdefault('rospy',SimpleNamespace(loginfo_throttle=lambda *_a:None,logwarn=lambda *_a:None,logerr=lambda *_a:None))
from global_frontier_graph_route_adapter import GlobalFrontierGraphRouteAdapterMixin
from global_frontier_planning_runtime import GlobalFrontierPlanningRuntimeMixin
from global_frontier_place_dfs import PlaceDfsTraversal

class Owner(GlobalFrontierGraphRouteAdapterMixin,GlobalFrontierPlanningRuntimeMixin):
    def __init__(self):
        self.place_dfs=PlaceDfsTraversal();self.place_dfs.enter(1,2,10)
        self.place_dfs.ready_to_return(2,projection_complete=True)
        self.current_physical_place_id=2;self.current_place_scope_id=2
        self.current_place_scope_valid=True;self.pose_odom=SimpleNamespace(x=4.,y=0.)
        self.active_route_id=9;self.active_frontier=None;self.prefetched_frontier=None
        self.map_msg=None;self.events=[];self.releases=[];self.preparations=0
    def _prepare_planning_cycle(self):
        self.preparations+=1
        return None,True  # The production controller-release wait exits here.
    def _finish_planning_cycle(self,_token):pass
    def fresh_costmap(self):return None
    def publish_status(self,event,**fields):self.events.append((event,fields))
    def release_active_frontier(self,**fields):self.releases.append(fields)

class PlaceReturnTimerTest(unittest.TestCase):
    def test_waiting_lease_checks_budget_and_stops_once_without_new_plan(self):
        owner=Owner()
        with patch('time.monotonic',return_value=0.):owner.on_timer(None)
        self.assertEqual(owner.preparations,1)
        with patch('time.monotonic',return_value=61.):owner.on_timer(None)
        self.assertEqual(owner.preparations,1)
        self.assertEqual(owner.place_dfs.nodes[2]['state'],'recovery_blocked')
        self.assertEqual(owner.place_dfs.stack,[1,2])
        self.assertEqual(owner.events[0][0],'place_dfs_return_unrecoverable')
        self.assertTrue(owner.events[0][1]['return_obligation_pending'])
        self.assertEqual(len(owner.releases),1)
        with patch('time.monotonic',return_value=120.):owner.on_timer(None)
        self.assertEqual(len(owner.events),1)
        self.assertEqual(len(owner.releases),1)
if __name__=='__main__':unittest.main()
