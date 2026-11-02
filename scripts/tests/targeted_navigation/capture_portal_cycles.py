"""Read-only raw sensing/control capture for every owned Portal cycle."""
import argparse,datetime,fcntl,json,signal,subprocess,time,hashlib
from pathlib import Path
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
p=argparse.ArgumentParser();p.add_argument('--driver',type=int,required=True);p.add_argument('--prefix',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
root=Path(a.output_dir).resolve();root.mkdir(parents=True,exist_ok=True)
lock=(root/('cycle_observer_%s.lock'%a.driver)).open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
proc=Path('/proc/%s/stat'%a.driver)
def token():
 try:return proc.read_text().split(') ',1)[1].split()[19]
 except FileNotFoundError:return None
initial=token();assert initial is not None
rospy.init_node('place_dfs_portal_cycle_raw_capture',anonymous=True)
active=False;last_status=0.;phase=None
def observe(m):
 global active,last_status,phase
 try:d=json.loads(m.data)
 except (ValueError,TypeError):return
 auth=d.get('portal_authentication') or {};bid=auth.get('active_branch_id')
 states=[b.get('state') for b in auth.get('branches',[]) if b.get('branch_id')==bid]
 dfs=any(n.get('state')=='returning' for n in (d.get('place_dfs') or {}).get('nodes',[]))
 active=dfs or any(s in ('entering','inside','turning','returning') for s in states)
 phase=dict(auth_branch_id=bid,auth_states=states,dfs_return=dfs,stack=(d.get('place_dfs') or {}).get('stack'))
 last_status=time.monotonic()
rospy.Subscriber('/lste/global_frontier/status',String,observe,queue_size=1)
topics=['/map','/tf','/tf_static','/rbt_pose','/pro3/wheel_odom','/pro3/os_cloud_node/points','/pro3/rlscan','/gazebo/model_states','/move_base/TebLocalPlannerROS/teb_feedback','/move_base/TebLocalPlannerROS/local_plan','/lste/cmd_vel/teb_planner','/cmd_vel','/lste/global_frontier/status','/lste/teb_turn_supervisor/status','/lste/cmd_vel/teb_contract','/lste/teb_goal_bridge/status','/lste/cmd_vel_mux/status','/move_base/global_costmap/costmap','/move_base/local_costmap/costmap','/move_base/global_costmap/costmap_updates','/move_base/local_costmap/costmap_updates','/lste/place_navigation_mask','/lste/place_navigation_mask_odom']
# Continuous raw mask sidecar also covers ordinary WI between Portal cycles.
# It is observation only, tied to this exact driver start token and singleton.
native_base=root/(a.prefix+'_native_masks_'+str(a.driver))
native_log=Path(str(native_base)+'.log').open('w')
native_command=['rosbag','record','-O',str(native_base)+'.bag',
                '/lste/place_navigation_mask','/lste/place_navigation_mask_odom']
native_recording=subprocess.Popen(native_command,stdout=native_log,stderr=subprocess.STDOUT)
native_contract=Path(str(native_base)+'_contract.json')
native_meta=dict(driver=a.driver,start_ticks=initial,command=native_command,start_sim=rospy.Time.now().to_sec(),observer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
config_path=Path(__file__).resolve().parents[3]/'src/lste_topo_access/config/place_navigation_costmap.yaml'
config_copy=Path(str(native_base)+'_place_navigation_costmap.yaml')
config_copy.write_bytes(config_path.read_bytes())
native_meta.update(config_source=str(config_path),config_copy=str(config_copy),
                   config_sha256=hashlib.sha256(config_copy.read_bytes()).hexdigest(),runtime_mask_topics=None)
native_contract.write_text(json.dumps(native_meta,indent=2))
def observe_physical_mask(message):
 try:
  native_meta['runtime_mask_topics']={
   'local':rospy.get_param('/move_base/local_costmap/place_scope_layer/map_topic'),
   'global':rospy.get_param('/move_base/global_costmap/place_scope_layer/map_topic'),
   'sample_mask_stamp':message.header.stamp.to_sec()}
 except (KeyError,rospy.ROSException):pass
native_mask_subscriber=rospy.Subscriber('/lste/place_navigation_mask_odom',OccupancyGrid,observe_physical_mask,queue_size=1)
try:
 recording=None;quiet=None;contract=None;log=None
 while token()==initial and not rospy.is_shutdown():
  now=time.monotonic()
  if active and recording is None and now-last_status<3:
   stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S');base=root/(a.prefix+'_portal_cycle_'+stamp)
   command=['rosbag','record','--split','--duration=120','-O',str(base)+'.bag',*topics]
   log=Path(str(base)+'.log').open('w');recording=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
   contract=Path(str(base)+'_contract.json');meta=dict(driver=a.driver,start_ticks=initial,start_phase=phase,start_sim=rospy.Time.now().to_sec(),command=command)
   contract.write_text(json.dumps(meta,indent=2));quiet=None
  if recording is not None:
   if active:quiet=None
   elif quiet is None:quiet=now
   if recording.poll() is not None:meta['recorder_exit']=recording.returncode
   elif quiet is not None and now-quiet>=5:
    recording.send_signal(signal.SIGINT);recording.wait(timeout=30);meta['recorder_exit']=recording.returncode
   else:time.sleep(.2);continue
   meta.update(end_phase=phase,end_sim=rospy.Time.now().to_sec());contract.write_text(json.dumps(meta,indent=2));log.close();recording=None
  time.sleep(.2)
 if recording is not None:
  if recording.poll() is None:recording.send_signal(signal.SIGINT);recording.wait(timeout=30)
  meta.update(end_reason='bound_driver_exit',recorder_exit=recording.returncode);contract.write_text(json.dumps(meta,indent=2));log.close()
 print('bound_driver_observer_finished',flush=True)
finally:
 if native_recording.poll() is None:
  native_recording.send_signal(signal.SIGINT);native_recording.wait(timeout=30)
 native_meta.update(end_reason='bound_driver_observer_exit',recorder_exit=native_recording.returncode)
 native_contract.write_text(json.dumps(native_meta,indent=2));native_log.close()
 if native_recording.returncode:raise SystemExit(native_recording.returncode)
