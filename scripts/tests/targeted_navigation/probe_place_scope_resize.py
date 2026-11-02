"""Actual native resize reproduction; isolated ROS, no navigation or Gazebo."""
import os,sys,time,json,signal,subprocess,socket,datetime
from pathlib import Path
plugin=sys.argv[1];port=11744
with socket.socket() as s:s.bind(('127.0.0.1',port))
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11744';os.environ['ROS_IP']='127.0.0.1';os.environ.pop('ROS_HOSTNAME',None)
root=Path('runtime/gp_branch_development/place_dfs_parent_return_fix_20261028_0310')/('global_resize_probe_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));root.mkdir()
children=[];logs=[];result={'plugin':plugin,'passed':False,'root':str(root)}
def launch(cmd,name):
 f=(root/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=f,start_new_session=True);children.append((name,p));return p
try:
 launch(['roscore','-p',str(port)],'master');time.sleep(1.5)
 import rospy,tf2_ros,numpy as np
 from nav_msgs.msg import OccupancyGrid
 from geometry_msgs.msg import TransformStamped
 rospy.init_node('resize_probe',disable_signals=True)
 def grid(n,origin,mask=False):
  m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=rospy.Time.now();m.info.resolution=.1;m.info.width=m.info.height=n;m.info.origin.position.x=m.info.origin.position.y=origin;m.info.origin.orientation.w=1
  a=np.full((n,n),-1 if mask else 0,dtype=np.int8)
  if mask:a[:,:n//2]=100
  else:a[n//2,n//2+10]=100
  m.data=a.ravel().tolist();return m
 tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='probe_base';tr.transform.rotation.w=1.;tr.header.stamp=rospy.Time.now();tf2_ros.StaticTransformBroadcaster().sendTransform(tr)
 obstacle=rospy.Publisher('/resize_probe/map',OccupancyGrid,latch=True,queue_size=1);maskpub=rospy.Publisher('/resize_probe/mask',OccupancyGrid,latch=True,queue_size=1)
 samples=[];rospy.Subscriber('/resize_native/costmap/costmap',OccupancyGrid,lambda m:samples.append(m),queue_size=30)
 cfg={'global_frame':'map','robot_base_frame':'probe_base','rolling_window':False,'width':8.,'height':8.,'resolution':.1,'update_frequency':10.,'publish_frequency':10.,'always_send_full_costmap':True,'robot_radius':.1,'track_unknown_space':False,'plugins':[{'name':'static_layer','type':'costmap_2d::StaticLayer'},{'name':'place_scope_layer','type':plugin}],'static_layer':{'map_topic':'/resize_probe/map','first_map_only':False,'track_unknown_space':False},'place_scope_layer':{'map_topic':'/resize_probe/mask','first_map_only':False,'track_unknown_space':True,'use_maximum':True}}
 rospy.set_param('/resize_native/costmap',cfg);(root/'parameters.json').write_text(json.dumps(cfg,indent=2))
 launch(['rosbag','record','-O',str((root/'raw.bag').resolve()),'/resize_probe/map','/resize_probe/mask','/resize_native/costmap/costmap','/tf_static'],'recorder');time.sleep(.8)
 obstacle.publish(grid(80,-4));mask=grid(80,-4,True);maskpub.publish(mask)
 native=launch([os.environ.get('PLACE_SCOPE_NATIVE_BIN','/opt/ros/noetic/lib/costmap_2d/costmap_2d_node'),'__name:=resize_native'],'native')
 def wait(pred,timeout=10):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   if native.poll() is not None:raise RuntimeError('native exited')
   if samples and pred(samples[-1]):return samples[-1]
   time.sleep(.05)
  raise RuntimeError('native condition timeout')
 def cost(m,x,y):
  c=int((x-m.info.origin.position.x)/.1);r=int((y-m.info.origin.position.y)/.1);return m.data[r*m.info.width+c]
 wait(lambda m:m.info.width==80 and cost(m,-1.05,.05)==100)
 before=len(samples);obstacle.publish(grid(120,-6));wait(lambda m:m.info.width==120);time.sleep(.8)
 resized=[m for m in samples[before:] if m.info.width==120]
 result['resized_samples']=len(resized);result['old_excluded_costs']=[cost(m,-1.05,.05) for m in resized]
 result['resize_preserved']=bool(resized) and all(c==100 for c in result['old_excluded_costs'])
 if plugin=='lste_topo_access::PlaceScopeLayer':
  result['new_area_excluded']=cost(samples[-1],-5.05,.05)
  assert result['new_area_excluded']==100
  invalid=grid(120,-6);invalid.data=[-1]*14400;invalid.header.frame_id='odom';maskpub.publish(invalid);time.sleep(.3)
  result['wrong_frame_retained']=cost(samples[-1],-1.05,.05);assert result['wrong_frame_retained']==100
  obstacle.publish(grid(140,-7));wait(lambda m:m.info.width==140 and cost(m,-1.05,.05)==100);time.sleep(.3)
  replacement=grid(40,-2,True);maskpub.publish(replacement);time.sleep(.4)
  result['mask_does_not_resize_master']=samples[-1].info.width;assert samples[-1].info.width==140
 if os.environ.get('PLACE_SCOPE_PROBE_RESET'):
  from std_srvs.srv import Empty
  rospy.wait_for_service('/resize_native/reset_layers',timeout=5)
  rospy.ServiceProxy('/resize_native/reset_layers',Empty)()
  time.sleep(.5)
  result['reset_layers_excluded']=cost(samples[-1],-1.05,.05)
  assert result['reset_layers_excluded']==100
 # No replacement mask before this check: old immutable physical raster only.
 transparent=grid(120,-6);transparent.data=[-1]*14400;maskpub.publish(transparent)
 cleared=wait(lambda m:cost(m,-1.05,.05)==0 and cost(m,1.05,.05)==100)
 result['transparent_clear']=cost(cleared,-1.05,.05);result['physical_obstacle']=cost(cleared,1.05,.05)
 result['passed']=result['resize_preserved'] and result['transparent_clear']==0 and result['physical_obstacle']==100
except Exception as e:
 import traceback
 result['error']=str(e);result['traceback']=traceback.format_exc()
finally:
 for name,p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
  result.setdefault('child_exit',{})[name]=p.returncode
 for f in logs:f.close()
 (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));sys.exit(0 if result['passed'] else 1)
