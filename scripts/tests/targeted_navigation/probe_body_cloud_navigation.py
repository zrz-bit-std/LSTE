#!/usr/bin/env python3
"""Physical low-obstacle navigation regression; world coordinates are audit only."""
import argparse
import json
import math
import threading
import sys
import time
from pathlib import Path
import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src/lste_topo_access/scripts"))
from lifecycle_manager import LifecycleManager


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--timeout',type=float,default=90)
    parser.add_argument('--goal',type=float,nargs=2,default=[2.,0.])
    args=parser.parse_args()
    rospy.init_node('body_cloud_navigation_probe',anonymous=True)
    lock=threading.Lock();poses=[];events=[];maps=[]
    def pose(message):
        with lock:poses.append([rospy.Time.now().to_sec(),message.x,message.y,message.theta])
    def event(message):
        try:record=json.loads(message.data)
        except ValueError:return
        with lock:
            events.append({key:record.get(key) for key in ('event','reason','status','transaction_id','route_id')})
    def costmap(message):
        info=message.info
        col=int(math.floor((0-info.origin.position.x)/info.resolution))
        row=int(math.floor((0-info.origin.position.y)/info.resolution))
        if 0<=row<info.height and 0<=col<info.width:
            with lock:maps.append([rospy.Time.now().to_sec(),int(message.data[row*info.width+col])])
    rospy.Subscriber('/rbt_pose',Pose2D,pose,queue_size=10)
    rospy.Subscriber('/lste/teb_goal_bridge/status',String,event,queue_size=100)
    rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,costmap,queue_size=2)
    publisher=rospy.Publisher('/lste/mission_goal',String,queue_size=1,latch=True)
    rospy.wait_for_message('/map',OccupancyGrid,timeout=45)
    deadline=time.monotonic()+10
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        with lock:ready=bool(poses) and publisher.get_num_connections()>0
        if ready:break
        time.sleep(.1)
    # An observation prerequisite for this physical regression only. The
    # navigation stack still receives only live sensors and the mission goal.
    deadline=time.monotonic()+15
    observed=False
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        with lock:observed=any(sample[1]>=99 for sample in maps)
        if observed:break
        time.sleep(.1)
    if not observed:
        result=dict(passed=False,reason='global_low_obstacle_not_observed',
                    poses=poses,costmap_obstacle_samples=maps)
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(passed=False,reason=result['reason'])))
        raise SystemExit(1)
    payload=dict(event='mission_goal',transaction_id=901,source='target_probe',priority=2,
                 lifecycle_transaction_id=LifecycleManager().begin_transaction(),
                 map_epoch=1,origin_epoch=1,projection_epoch=1,route_id=901,route_kind='target_approach',mission_route_kind='target_approach',
                 target_epoch=1,target_track_id='body-obstacle-navigation-probe',
                 target_viewpoint_candidate_id='body-cloud-goal',target_viewpoint_attempt_id='body-cloud-goal-1',
                 frame_id='odom',goal=list(args.goal),yaw=0.,
                 goal_context=dict(method='navigation_obstacle_regression',task_id='body_cloud_probe',
                                   goal_role='semantic_target_approach'))
    publisher.publish(String(data=json.dumps(payload)))
    deadline=time.monotonic()+args.timeout;reached=False
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        with lock:last=None if not poses else poses[-1]
        if last and math.hypot(last[1]-args.goal[0],last[2]-args.goal[1])<.55:
            reached=True;break
        time.sleep(.1)
    with lock:
        result=dict(passed=reached and observed and bool(poses) and min(math.hypot(p[1],p[2]) for p in poses)>.30,
                    reached=reached,minimum_obstacle_center_distance=min((math.hypot(p[1],p[2]) for p in poses),default=None),
                    poses=poses,bridge_events=events,costmap_obstacle_samples=maps,
                    world='low_body_obstacle_navigation.world',goal=list(args.goal),obstacle_audit_xy=[0,0])
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('poses','bridge_events','costmap_obstacle_samples')}))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
