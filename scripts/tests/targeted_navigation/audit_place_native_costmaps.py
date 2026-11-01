#!/usr/bin/env python3
"""Replay native masters independently; no runtime ready flag is acceptance."""
import argparse,copy,json,math,hashlib
from bisect import bisect_right
from pathlib import Path
import numpy as np
from audit_place_dfs import events
from execution_release_evidence import route_retired,execution_identity,SOURCE_SHA256 as RELEASE_SOURCE_SHA256
AUDIT_SOURCE_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()

MASTERS=('/move_base/global_costmap/costmap','/move_base/local_costmap/costmap')
MASK='/lste/place_navigation_mask'
LOCAL_MASK='/lste/place_navigation_mask_odom'


def authorized_dfs_return_local_egress(d):
    """Recognize local egress only when the canonical entered Place is returning.

    Ordinary WI egress remains bounded. A Place whose DFS node is explicitly
    returning through its recorded entry Portal is an authorized return route,
    so its mask may be transparent until that route's exact lease ACK.
    """
    if d.get('active_execution_route_kind')!='local_egress':return False
    dfs=d.get('place_dfs') or {};stack=dfs.get('stack') or []
    if len(stack)<2:return False
    place_id=stack[-1];nodes={n.get('place_id'):n for n in dfs.get('nodes',[]) if isinstance(n,dict)}
    node=nodes.get(place_id) or {};scope=d.get('place_scope') or {}
    if (node.get('state')!='returning' or scope.get('valid') is not True
            or scope.get('place_id')!=place_id):return False
    portal_id=node.get('portal_id')
    if portal_id is None or scope.get('entry_portal_id')!=portal_id:return False
    edge=next((p for p in scope.get('projected_portals',[])
               if p.get('id')==portal_id),None)
    return bool(edge and edge.get('source_place_id')==node.get('parent_id')
                and edge.get('destination_place_id')==place_id)


def authorized_portal_authentication_route(d):
    """Recognize GP's exclusive active-branch goal, not an arbitrary WI.

    During GP doorway authentication, choose_frontier returns only the active
    branch's observation goal. It keeps the legacy place_observation role for
    bridge compatibility, so the durable identity is the live branch snapshot
    plus a missing durable WI id. An ordinary WI id always wins and remains
    bounded, even if an auth snapshot is concurrently present.
    """
    auth=d.get('portal_authentication') or {}
    branch_id=auth.get('active_branch_id')
    if branch_id is None:return False
    branch=next((item for item in auth.get('branches',[])
                 if item.get('branch_id')==branch_id),None)
    if branch is None or branch.get('state') not in ('entering','inside','turning','returning'):
        return False
    context=d.get('goal_context') or {}
    if context.get('work_item_id') is not None:return False
    if context.get('goal_role') not in ('place_observation','geometry_frontier'):
        return False
    if d.get('active_execution_route_kind') not in ('frontier_endpoint','frontier_turn_connector'):
        return False
    return True


def execution_mode(d,owned_mode=None):
    route,_=execution_identity(d)
    if route_retired(d):return 'idle'
    if (d.get('active') is not True and d.get('pending') is not True
            and route in (None,0) and not d.get('awaiting_controller_lease_release_ack')):
        return 'idle'
    # A paused high-level action still owns the controller until its exact ACK.
    # Preserve the admitted role: a new auth task cannot exempt an old WI lease.
    if owned_mode is not None:return owned_mode
    if authorized_dfs_return_local_egress(d):return 'transparent'
    if authorized_portal_authentication_route(d):return 'transparent'
    if d.get('active_execution_route_kind')=='portal_transition':return 'transparent'
    return 'ordinary'  # Includes ordinary turning connectors and non-DFS local egress.



def controller_lease_intervals(status_events,mode_for):
    """Use the raw goal bridge dispatch→exact ACK as controller ownership.

    Graph route selection can outlive a controller action, or a queued goal can
    be cancelled before a costmap publishes its next full grid. Neither event
    alone creates/extends a controller lease. The bridge's own exact action
    identity is the ownership boundary.
    """
    identity_fields=('route_id','action_generation','graph_transaction_id',
                     'lifecycle_transaction_id','transaction_id','map_epoch',
                     'origin_epoch','projection_epoch')
    ordered=sorted(status_events,key=lambda item:item[0])
    starts=[];seen=set()
    for index,(stamp,data) in enumerate(ordered):
        event=data.get('event')
        # ``route_continuation_dispatch`` opens an interval when a persistent
        # action adopts a new semantic route without a fresh actionlib dispatch
        # (run 20261031_214617: route 140 -> 141 in one action generation).
        if event not in ('dispatch','route_continuation_dispatch'):
            continue
        if event=='dispatch' and data.get('active') is not True:
            continue
        try:
            # A continuation status is published BEFORE the bridge promotes
            # the adopted route, so ``active_route_id`` still names the
            # predecessor there.  Keying that event on ``active_route_id``
            # opened the *previous* route a second time and left the adopted
            # route's real opening at the next successor's status, past its own
            # ACK.  Runs 20261101_160923 closed 35/36 and 40/41 that way: six
            # false "lacks matching exact lease ACK" errors, two identity
            # conflicts, and every mask-mode sample attributed to the wrong
            # owner.
            if event=='route_continuation_dispatch':
                route=int(data.get('route_id',0) or 0)
                generation=int(data.get('action_generation',
                    data.get('active_action_generation',0)) or 0)
            else:
                route=int(data.get('active_route_id',data.get('route_id')) or 0)
                generation=int(data.get('active_action_generation',data.get('action_generation')) or 0)
        except (TypeError,ValueError):
            continue
        if route<=0 or generation<=0 or (route,generation) in seen:
            continue
        seen.add((route,generation))
        end=None
        for ack_stamp,ack in ordered[index+1:]:
            if (ack.get('event') not in ('lease_release_ack','route_superseded')
                    or ack.get('route_id')!=route
                    or ack.get('action_generation')!=generation):
                continue
            if all(data.get(key) is not None and ack.get(key)==data.get(key)
                   for key in identity_fields):
                end=ack_stamp
                break
        starts.append(dict(route_id=route,action_generation=generation,
                           start=stamp,end=end,mode=mode_for(route,stamp),
                           exact_ack=end is not None))
    # A dispatch that is still open when the recording stops is a truncated
    # trial, not a lost lease: the window ended before the bridge could deliver
    # the ACK.  Only the final dispatch can be truncated -- any later lease
    # event proves the run continued, and then the older lease really is lost
    # (run 20261031_214617: route 140 never closed while 141 was adopted).
    lease_events = [
        (stamp, data) for stamp, data in ordered
        if data.get('event') in ('dispatch', 'route_continuation_dispatch',
                                 'lease_release_ack', 'route_superseded')
    ]
    if lease_events:
        last_stamp, last = lease_events[-1]
        if last.get('event') in ('dispatch', 'route_continuation_dispatch'):
            for lease in starts:
                if lease['end'] is None and lease['start'] == last_stamp:
                    lease['open_at_end'] = True
    # Two different routes owning the same transport generation must never
    # overlap. When they do the reconstruction cannot attribute a mask mode to
    # one owner, so report the identity conflict instead of silently letting
    # the older lease own all later time.
    for first in starts:
        for second in starts:
            if first is second: continue
            if first['action_generation']!=second['action_generation']: continue
            if first['route_id']==second['route_id']: continue
            first_end=float('inf') if first['end'] is None else first['end']
            second_end=float('inf') if second['end'] is None else second['end']
            # Half-open intervals: a lease that ends exactly when the next
            # one starts is a clean handoff, not a conflict.
            if first['start']<second_end and second['start']<first_end:
                first['identity_conflict']=True
                second['identity_conflict']=True
    return starts


def mask_mode_conflict(mode,bounded):
    """True when the published mask contradicts the owned execution mode."""
    return (mode=='ordinary' and not bounded) or (mode=='transparent' and bounded)


def controller_mode_at(stamp,leases):
    """Return the admitted role only while a raw controller lease owns time."""
    owners=[lease for lease in leases
            if lease['start']<=stamp and (lease['end'] is None or stamp<=lease['end'])]
    if not owners:return 'idle'
    # Overlapping distinct action identities are not expected; fail closed to
    # ordinary if malformed data creates one.
    modes={lease['mode'] for lease in owners}
    return next(iter(modes)) if len(modes)==1 else 'ordinary'

def apply_update(grid,update):
    if grid is None:raise ValueError('incremental master has no preceding full grid')
    if (update.header.frame_id!=grid.header.frame_id or update.width<=0 or update.height<=0
            or update.x<0 or update.y<0 or update.x+update.width>grid.info.width
            or update.y+update.height>grid.info.height):raise ValueError('invalid native master update extent/frame')
    out=copy.copy(grid);out.header=copy.copy(update.header)
    values=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width).copy()
    values[update.y:update.y+update.height,update.x:update.x+update.width]=np.asarray(update.data).reshape(update.height,update.width)
    out.data=values.ravel();return out


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def exclusion_check(grid,mask,translation=(0.,0.),rotation=0.):
    """Transform actual master cell centers into the original published mask."""
    if grid.header.stamp<mask.header.stamp:raise ValueError('master predates active mask')
    a,b=grid.info,mask.info;rows,cols=np.indices((a.height,a.width))
    px=(cols+.5)*a.resolution;py=(rows+.5)*a.resolution;angle=yaw(a.origin.orientation)
    x=a.origin.position.x+math.cos(angle)*px-math.sin(angle)*py
    y=a.origin.position.y+math.sin(angle)*px+math.cos(angle)*py
    x,y=translation[0]+math.cos(rotation)*x-math.sin(rotation)*y,translation[1]+math.sin(rotation)*x+math.cos(rotation)*y
    x,y=x-b.origin.position.x,y-b.origin.position.y;angle=-yaw(b.origin.orientation)
    c=np.floor((math.cos(angle)*x-math.sin(angle)*y)/b.resolution).astype(int)
    r=np.floor((math.sin(angle)*x+math.cos(angle)*y)/b.resolution).astype(int)
    overlap=(r>=0)&(r<b.height)&(c>=0)&(c<b.width)
    if not np.any(overlap):raise ValueError('master and Place mask have no overlap')
    rr,cc=np.where(overlap);mask_data=np.asarray(mask.data).reshape(b.height,b.width)
    excluded=mask_data[r[rr,cc],c[rr,cc]]==100
    values=np.asarray(grid.data).reshape(a.height,a.width)[rr[excluded],cc[excluded]]
    return dict(excluded_cells=int(values.size),nonlethal_cells=int(np.count_nonzero(values!=100)))


def source_mask_transform(mask, source_frame, buffer):
    """Original mask acquisition TF, independent of costmap publication time."""
    stamp=mask.info.map_load_time
    if stamp.to_sec()<=0 or stamp>mask.header.stamp:
        raise ValueError('missing/future source map epoch in native Place mask')
    if source_frame.lstrip('/')==mask.header.frame_id.lstrip('/'):
        return (0.,0.),0.
    tr=buffer.lookup_transform(mask.header.frame_id,source_frame,stamp).transform
    return (tr.translation.x,tr.translation.y),yaw(tr.rotation)


def paired_physical_mask(global_mask,local_mask):
    if (local_mask.header.frame_id.lstrip('/')!='odom'
            or local_mask.header.stamp!=global_mask.header.stamp
            or local_mask.info.map_load_time!=global_mask.info.map_load_time):
        raise ValueError('native masks do not share one published source-map snapshot')


def audit(metrics,bag_path):
    import rosbag,rospy,tf2_ros
    statuses=[];dispatches=[];owned_modes={};metric_contexts=[];last_metric_time=0.
    for line,d in events(metrics):
        if d.get('ros_time') is None:continue
        t=float(d['ros_time']);last_metric_time=max(last_metric_time,t);identity=execution_identity(d)
        metric_contexts.append((t,d))
        mode=execution_mode(d,owned_modes.get(identity))
        if identity[0] not in (None,0) and mode!='idle':owned_modes.setdefault(identity,mode)
        if not statuses or statuses[-1][1]!=mode:statuses.append((t,mode))
        if d.get('frontier_event')=='work_item_dispatched':
            dispatch_mode=execution_mode(d,owned_modes.get(identity))
            dispatches.append((t,int(d['route_id']),line,dispatch_mode))
    if not dispatches:return dict(passed=False,errors=['no ordinary WI dispatches'])
    statuses.sort(key=lambda item:item[0]);times=[s[0] for s in statuses]
    errors=[];total_errors=0;frames={t:0 for t in MASTERS};route_coverage={r:set() for _,r,_,_ in dispatches};transparent=0
    owned=[(t,statuses[i+1][0] if i+1<len(statuses) else last_metric_time)for i,(t,mode)in enumerate(statuses)if mode!='idle']
    if not owned:return dict(passed=False,errors=['no owned execution timeline for recorded dispatches'])
    required_start=min(t for t,_ in owned);required_end=max(t for _,t in owned)
    def fail(reason,**fields):
        nonlocal total_errors
        total_errors+=1
        if len(errors)<100:errors.append(dict(reason=reason,**fields))
    with rosbag.Bag(str(bag_path)) as bag:
        topics=bag.get_type_and_topic_info().topics
        missing=[t for t in (*MASTERS,MASK) if t not in topics]
        if missing:return dict(passed=False,errors=[dict(reason='missing native raw topics',topics=missing)])
        # Metrics times are rounded to milliseconds. Do not certify a run
        # whose raw bag/physical channels end before its owned execution does.
        epsilon=.001
        channels=('/lste/global_frontier/status','/rbt_pose','/pro3/wheel_odom')
        channel_start={}
        for topic,m,t in bag.read_messages(topics=list(channels)):
            channel_start.setdefault(topic,m.header.stamp.to_sec() if hasattr(m,'header') else t.to_sec())
            if len(channel_start)==len(channels):break
        channel_end={}
        for topic,m,t in bag.read_messages(topics=['/lste/global_frontier/status','/rbt_pose','/pro3/wheel_odom'],
                                           start_time=rospy.Time.from_sec(max(0.,required_end-epsilon))):
            stamp=(m.header.stamp.to_sec() if hasattr(m,'header') else t.to_sec())
            channel_end[topic]=max(channel_end.get(topic,0.),stamp)
        coverage=dict(required_start=required_start,required_end=required_end,
                      bag_start=bag.get_start_time(),bag_end=bag.get_end_time(),
                      starting_channel_stamps=channel_start,ending_channel_stamps=channel_end,rounding_tolerance_s=epsilon)
        coverage['passed']=(coverage['bag_start']<=required_start+epsilon
                            and coverage['bag_end']+epsilon>=required_end
                            and all(channel_start.get(topic,float('inf'))<=required_start+epsilon for topic in channels)
                            and all(channel_end.get(topic,-1.)+epsilon>=required_end
                                for topic in ('/lste/global_frontier/status','/rbt_pose','/pro3/wheel_odom')))
        if not coverage['passed']:fail('raw recording does not cover owned execution endpoints',coverage=coverage)
        buffer=tf2_ros.Buffer(rospy.Duration(max(120.,bag.get_end_time()-bag.get_start_time()+2)),debug=False)
        masks=[];local_masks=[];bridge_events=[];physical_protocol=LOCAL_MASK in topics
        bridge_topic='/lste/teb_goal_bridge/status'
        for topic,m,t in bag.read_messages(topics=['/tf','/tf_static',MASK,LOCAL_MASK,bridge_topic]):
            if topic in (MASK,LOCAL_MASK):
                m.data=np.asarray(m.data,dtype=np.int8)
                (masks if topic==MASK else local_masks).append((m.header.stamp.to_sec(),m,bool(np.any(m.data==100))))
            elif topic==bridge_topic:
                try:bridge_events.append((t.to_sec(),json.loads(m.data)))
                except Exception as error:raise ValueError('invalid raw controller bridge status: '+str(error))
            else:
                for tr in m.transforms:
                    if topic=='/tf_static':buffer.set_transform_static(tr,'raw_native_audit')
                    else:buffer.set_transform(tr,'raw_native_audit')
        masks.sort(key=lambda item:item[0]);mask_times=[item[0] for item in masks]
        local_masks.sort(key=lambda item:item[0]);local_mask_times=[item[0] for item in local_masks]
        if not masks:return dict(passed=False,errors=['no recorded Place masks'])
        controller_leases=[]
        def mode_for_controller_route(route_id,stamp):
            match=None
            for metric_stamp,data in metric_contexts:
                if metric_stamp>stamp:break
                metric_route=int(data.get('active_execution_route_id',data.get('route_id',0)) or 0)
                if metric_route==route_id:match=data
            if match is None:return 'ordinary'
            intent=dict(match);intent['active']=True;intent['pending']=False
            intent['awaiting_controller_lease_release_ack']=False
            intent.pop('released_controller_route',None);intent.pop('last_controller_lease_release_ack',None)
            return execution_mode(intent,owned_modes.get(execution_identity(match)))
        if bridge_topic in topics:
            controller_leases=controller_lease_intervals(bridge_events,mode_for_controller_route)
            missing_ack=[dict(route_id=x['route_id'],action_generation=x['action_generation'],start=x['start'])
                         for x in controller_leases
                         if not x['exact_ack'] and not x.get('open_at_end')]
            if missing_ack:fail('raw controller dispatch lacks matching exact lease ACK',routes=missing_ack[:20])
            controller_dispatches=[(x['start'],x['route_id'],x['action_generation'],x['mode']) for x in controller_leases]
        else:
            controller_dispatches=[]
            if physical_protocol:fail('physical-mask bag lacks raw controller bridge status')
        conflicts=[lease for lease in controller_leases
                   if lease.get('identity_conflict')]
        if conflicts:
            fail('controller lease identity conflict',
                 generations=sorted({lease['action_generation'] for lease in conflicts})[:10],
                 routes=sorted({(lease['route_id'],lease['action_generation'])
                                for lease in conflicts})[:10])
        mask_mode_checks=0
        mask_mode_failures=0
        for stamp,mask,bounded in masks+local_masks:
            if bridge_topic in topics:mode=controller_mode_at(stamp,controller_leases)
            else:
                i=bisect_right(times,stamp)-1
                mode=(controller_mode_at(stamp,controller_leases) if bridge_topic in topics
                      else statuses[i][1] if i>=0 else 'idle')
            if mode=='idle':continue
            mask_mode_checks+=1
            if mask_mode_conflict(mode,bounded):
                mask_mode_failures+=1
                if mask_mode_failures<=5:
                    fail('published mask mode disagrees with owned execution',stamp=stamp,mode=mode,bounded=bounded)
        if mask_mode_failures>5:
            fail('published mask mode disagrees with owned execution (further suppressed)',
                 count=mask_mode_failures-5)
        def check(grid,stamp,topic,route=None,mode='ordinary'):
            i=bisect_right(mask_times,stamp)-1
            if i<0:raise ValueError('no preceding Place mask')
            mask=masks[i][1]
            if grid.header.stamp.to_sec()>stamp:raise ValueError('future master cannot certify an earlier dispatch')
            if mode=='transparent':
                if masks[i][2]:raise ValueError('authorized crossing/auth has bounded map mask')
                if physical_protocol:
                    k=bisect_right(local_mask_times,stamp)-1
                    if k<0 or local_masks[k][2]:raise ValueError('authorized crossing/auth has bounded physical mask')
                    paired_physical_mask(mask,local_masks[k][1])
                route_coverage.setdefault(route,set()).add(topic)
                return
            if mode!='ordinary':raise ValueError('dispatch has no owned ordinary/authorized mode')
            if not masks[i][2]:raise ValueError('ordinary WI has transparent mask')
            target=mask.header.frame_id.lstrip('/');source=grid.header.frame_id.lstrip('/')
            translation=(0.,0.);angle=0.
            local_mask=None
            if physical_protocol:
                k=bisect_right(local_mask_times,stamp)-1
                if k<0:raise ValueError('missing preceding physical native mask')
                local_mask=local_masks[k][1];paired_physical_mask(mask,local_mask)
                # Verify conservative physical rendering from the original raw
                # mask+TF, then independently verify every actual native cell.
                lt,la=source_mask_transform(mask,'odom',buffer)
                rendered=exclusion_check(local_mask,mask,lt,la)
                if rendered['nonlethal_cells']:raise ValueError('physical mask admits source-excluded cells')
                if topic==MASTERS[1]:
                    translation,angle=source_mask_transform(mask,source,buffer)
                    direct=exclusion_check(grid,local_mask)
                    if direct['nonlethal_cells']:raise ValueError('native local master does not enforce physical mask: '+str(direct))
                elif source!=target:raise ValueError('global native mask frame mismatch')
            elif source!=target:
                tr=buffer.lookup_transform(target,source,grid.header.stamp).transform
                translation=(tr.translation.x,tr.translation.y);angle=yaw(tr.rotation)
            result=exclusion_check(grid,mask,translation,angle)
            if result['nonlethal_cells']:raise ValueError('ordinary master has nonlethal excluded cells: '+str(result))
            frames[topic]+=1
            if route is not None:route_coverage[route].add(topic)
        dispatch_checks=[(t,r,line,'graph',mode) for t,r,line,mode in dispatches]
        dispatch_checks.extend((t,r,None,'controller',mode) for t,r,_,mode in controller_dispatches)
        pending=iter(sorted(dispatch_checks,key=lambda item:(item[0],item[1],item[3],item[2] or -1)));next_dispatch=next(pending,None);grids={};startup_updates=0
        def dispatch_until(t):
            nonlocal next_dispatch
            while next_dispatch is not None and next_dispatch[0]<=t:
                stamp,route,line,source,mode=next_dispatch
                for topic in MASTERS:
                    try:
                        if topic not in grids:raise ValueError('no preceding full master at dispatch')
                        check(grids[topic],stamp,topic,route,mode)
                    except Exception as error:
                        fields=dict(route_id=route,line=line,topic=topic,dispatch_source=source)
                        if source=='controller':fields['action_generation']=next(
                            (gen for ts,r,gen,_ in controller_dispatches if ts==stamp and r==route),None)
                        fail(str(error),**fields)
                next_dispatch=next(pending,None)
        for topic,m,t in bag.read_messages(topics=[*MASTERS,*(p+'_updates' for p in MASTERS)]):
            dispatch_until(t.to_sec())
            base=topic[:-8] if topic.endswith('_updates') else topic
            try:
                stamp=m.header.stamp.to_sec();i=bisect_right(times,stamp)-1
                mode=(controller_mode_at(stamp,controller_leases) if bridge_topic in topics
                      else statuses[i][1] if i>=0 else 'idle')
                if topic.endswith('_updates') and base not in grids and mode=='idle':
                    startup_updates+=1
                    continue  # Explicitly outside all execution, never a checked master.
                grids[base]=apply_update(grids.get(base),m) if topic.endswith('_updates') else m
                if mode=='ordinary':check(grids[base],stamp,base)
                elif mode=='transparent':
                    k=bisect_right(mask_times,stamp)-1
                    if k<0 or masks[k][2]:raise ValueError('authorized crossing/auth has bounded mask')
                    if physical_protocol:
                        j=bisect_right(local_mask_times,stamp)-1
                        if j<0 or local_masks[j][2]:raise ValueError('authorized crossing/auth has bounded physical mask')
                        paired_physical_mask(masks[k][1],local_masks[j][1])
                    transparent+=1
            except Exception as error:fail(str(error),topic=base,stamp=m.header.stamp.to_sec())
        dispatch_until(float('inf'))
    uncovered={str(r):[t for t in MASTERS if t not in seen] for r,seen in route_coverage.items() if len(seen)!=2}
    return dict(passed=total_errors==0 and not uncovered and all(frames.values()) and transparent>0,
        errors=errors,error_count=total_errors,execution_time_coverage=coverage,ordinary_master_checks=frames,
        published_mask_mode_checks=mask_mode_checks,transparent_mask_checks=transparent,idle_updates_before_first_full=startup_updates,dispatch_routes=len(route_coverage),controller_dispatches=len(controller_dispatches),controller_owned_intervals=len(controller_leases),uncovered_dispatches=uncovered,
        mask_protocol=('map+same-source-epoch odom physical mask' if physical_protocol else 'legacy single map mask; exact master-stamp TF'),
        contract='Raw goal-bridge dispatch→exact ACK defines controller ownership; all master full+updates in ordinary owned intervals and both masters at graph and controller dispatches are checked. Owned Portal/auth masks stay transparent through exact ACK. Exact original TF at master stamp; no latest TF or skipped errors.',metrics=str(metrics),bag=str(bag_path))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('metrics');p.add_argument('bag');p.add_argument('--output',required=True);a=p.parse_args()
    try:r=audit(a.metrics,a.bag)
    except Exception as e:r=dict(passed=False,errors=[type(e).__name__+': '+str(e)])
    r['audit_source_sha256']={'audit_place_native_costmaps.py':AUDIT_SOURCE_SHA256,'execution_release_evidence.py':RELEASE_SOURCE_SHA256}
    Path(a.output).write_text(json.dumps(r,indent=2)+'\n');raise SystemExit(0 if r['passed'] else 1)
if __name__=='__main__':main()
