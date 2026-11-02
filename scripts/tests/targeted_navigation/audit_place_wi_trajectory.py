#!/usr/bin/env python3
"""Audit measured ordinary WI motion against exact recorded Place SLAM masks."""
import argparse
from bisect import bisect_right
from collections import OrderedDict
import json
import math
from pathlib import Path
import numpy as np
from audit_place_dfs_geometry import component_from_map, bound_portal_identities_valid, events


def ordinary_scope(data):
    context=data.get('goal_context') or {}
    auth=data.get('portal_authentication') or {}
    if (not data.get('active') or context.get('goal_role') not in ('place_observation','place_observation_work')
            or context.get('portal_crossing_certified')
            or data.get('active_execution_route_kind')=='portal_transition'
            or auth.get('active_branch_id') is not None):
        return None
    scope=data.get('place_scope') or {}
    owner=context.get('owner_place_id')
    if owner is None:return None
    return dict(scope=scope,place_id=owner,route_id=data.get('active_execution_route_id',data.get('route_id')),
                valid=bool(scope.get('valid') and scope.get('place_id')==owner
                           and bound_portal_identities_valid(data,scope)))


def interpolate_transform(transforms, times, stamp):
    index=bisect_right(times,stamp)-1
    if index<0:return None
    before=transforms[index]
    if index+1==len(transforms):
        return before[1:] if stamp-before[0]<=.5 else None
    after=transforms[index+1]
    if after[0]-before[0]>1.:return None
    fraction=(stamp-before[0])/max(1e-9,after[0]-before[0])
    angle=math.atan2(math.sin(after[3]-before[3]),math.cos(after[3]-before[3]))
    return (before[1]+fraction*(after[1]-before[1]),
            before[2]+fraction*(after[2]-before[2]),before[3]+fraction*angle)


def audit(metrics, bag_path, sample_period=0.):
    import rosbag
    contexts=[];last_signature=None;dispatch_routes=set();dispatch_places=set()
    for line,data in events(metrics):
        stamp=data.get('ros_time')
        if stamp is None:continue
        if (data.get('frontier_event')=='controller_lease_release_acknowledged'
                and 'active_execution_route_id' not in data):
            # Older logs override route_id with the acknowledged predecessor;
            # the preceding dispatch retains the actual active route identity.
            continue
        context=ordinary_scope(data)
        if context is not None:
            # Epoch counters and frontier totals do not change the raster's
            # geometric component. Cache only the actual reconstruction inputs.
            scope=context['scope']
            context['geometry_key']=json.dumps({key:scope.get(key) for key in
                ('map_stamp','place_id','seed_map_xy','entry_portal_id','projected_portals')},sort_keys=True)
        if data.get('frontier_event')=='work_item_dispatched':
            dispatch_routes.add(data.get('route_id'))
            dispatch_places.add(data.get('place_id'))
        signature=None if context is None else json.dumps(context,sort_keys=True)
        if signature!=last_signature:
            contexts.append((float(stamp),line,context));last_signature=signature
    contexts.sort(key=lambda row:row[0]);context_times=[row[0] for row in contexts]
    wanted={round(float(c['scope']['map_stamp']),6) for _,_,c in contexts
            if c is not None and c['scope'].get('map_stamp') is not None}
    maps={};transforms=[];poses=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,message,received in bag.read_messages(topics=['/map','/tf','/tf_static','/rbt_pose']):
            if topic=='/map':
                stamp=round(message.header.stamp.to_sec(),6)
                if stamp in wanted:maps[stamp]=message
            elif topic in ('/tf','/tf_static'):
                for transform in message.transforms:
                    if (transform.header.frame_id.lstrip('/')!='map'
                            or transform.child_frame_id.lstrip('/')!='odom'):continue
                    t,q=transform.transform.translation,transform.transform.rotation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    transforms.append((transform.header.stamp.to_sec(),t.x,t.y,yaw))
            else:
                poses.append((received.to_sec(),message.x,message.y))
    transforms.sort();tf_times=[t[0] for t in transforms]
    cache=OrderedDict();errors=[];checked=0;routes=set();places=set();distance=0.;previous=None
    last_sample_time=-float('inf');last_sample_scope=None
    def fail(reason,stamp,line,**details):
        if len(errors)<80:errors.append(dict(reason=reason,time=stamp,line=line,**details))
    for stamp,x,y in poses:
        index=bisect_right(context_times,stamp)-1
        if index<0:previous=None;continue
        context_time,line,context=contexts[index]
        if context is None:previous=None;continue
        scope=context['scope'];place=context['place_id'];route=context['route_id']
        sample_scope=(place,route,context['geometry_key'])
        if sample_scope==last_sample_scope and stamp-last_sample_time<sample_period:continue
        last_sample_scope=sample_scope;last_sample_time=stamp
        if not context['valid']:
            fail('active ordinary route lacks valid matching Place scope',stamp,line,place_id=place)
            previous=None;continue
        map_stamp=scope.get('map_stamp');message=maps.get(round(float(map_stamp),6)) if map_stamp is not None else None
        if message is None:
            fail('exact SLAM map absent for ordinary trajectory',stamp,line,map_stamp=map_stamp)
            previous=None;continue
        transform=interpolate_transform(transforms,tf_times,stamp)
        if transform is None:
            fail('map to odom TF absent at measured pose',stamp,line);previous=None;continue
        tx,ty,yaw=transform;c,s=math.cos(yaw),math.sin(yaw)
        xy=(tx+c*x-s*y,ty+s*x+c*y)
        key=context['geometry_key']
        if key not in cache:
            cache[key]=component_from_map(message,scope)
            if len(cache)>8:cache.popitem(last=False)
        labels,region,cell=cache[key]
        # Interpolate measured adjacent samples only; never join across a
        # Portal phase, route identity change, or missing-time interval.
        points=[xy]
        if previous is not None and previous[:2]==(place,route) and previous[4]==key and stamp-previous[2]<=.5:
            start=previous[3];length=math.hypot(xy[0]-start[0],xy[1]-start[1])
            distance+=math.hypot(x-previous[5][0],y-previous[5][1])
            count=max(1,int(math.ceil(length/(message.info.resolution*.5))))
            points=[(start[0]+(xy[0]-start[0])*i/count,start[1]+(xy[1]-start[1])*i/count) for i in range(count+1)]
        bad=[]
        for point in points:
            row,col=cell(point)
            if not (region>0 and 0<=row<labels.shape[0] and 0<=col<labels.shape[1] and labels[row,col]==region):bad.append(point)
        if bad:fail('measured ordinary WI trajectory outside current Place',stamp,line,
                    place_id=place,route_id=route,map_stamp=map_stamp,pose_odom=[x,y],pose_map=list(xy),outside_map=list(bad[0]))
        checked+=1;routes.add(route);places.add(place);previous=(place,route,stamp,xy,key,(x,y))
    missing_places=dispatch_places-places
    if missing_places:fail('no ordinary trajectory samples for dispatched Place',None,None,place_ids=sorted(missing_places))
    missing_routes=dispatch_routes-routes
    if missing_routes:fail('no measured ordinary trajectory for dispatched route',None,None,route_ids=sorted(missing_routes))
    return dict(passed=checked>0 and not errors,checked_pose_samples=checked,
                dispatch_route_count=len(dispatch_routes),checked_dispatch_route_count=len(dispatch_routes & routes),
                unobserved_dispatch_routes=sorted(dispatch_routes-routes),
                measured_distance_m=distance,route_count=len(routes),place_ids=sorted(places),
                errors=errors,metrics=str(metrics),bag=str(bag_path),sample_period_s=sample_period,
                sampling_mode='all_raw_poses' if sample_period==0 else 'sampled_diagnostic',
                contract='ordinary place_observation/place_observation_work; every dispatched route requires measured coverage; active Portal authentication and certified crossings excluded; exact SLAM snapshot, raw map/odom TF and measured poses')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('metrics');parser.add_argument('bag');parser.add_argument('--output')
    parser.add_argument('--sample-period',type=float,default=0.,help='0 checks every raw pose; positive values are explicit sampled diagnostics')
    args=parser.parse_args();result=audit(args.metrics,args.bag,sample_period=max(0.,args.sample_period))
    rendered=json.dumps(result,indent=2);print(rendered)
    if args.output:Path(args.output).write_text(rendered+'\n')
    raise SystemExit(0 if result['passed'] else 1)

if __name__=='__main__':main()
