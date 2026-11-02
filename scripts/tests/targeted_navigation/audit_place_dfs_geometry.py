#!/usr/bin/env python3
"""Reconstruct WI membership from recorded SLAM, independent of runtime masks."""
import argparse
import json
from pathlib import Path
import numpy as np
from collections import deque
from bisect import bisect_right
import math
from audit_place_dfs import events as metric_events, completed_frontier_accounted



def events(path):
    """Carry only the latest preceding tree across asynchronous status records.

    Tree snapshots are published on updates, while Portal scope records are
    more frequent. Missing tree fields do not erase earlier physical evidence;
    a future snapshot can never certify an earlier record.
    """
    latest_tree = None
    for number, event in metric_events(path):
        if 'tree' in event:
            latest_tree = event['tree']
        elif latest_tree is not None:
            event = dict(event, tree=latest_tree)
        yield number, event

def component_from_map(message, scope):
    """Rebuild certified finite cuts and flood-fill with a standalone flood fill, not planner code."""
    info=message.info
    free=np.asarray(message.data).reshape(info.height,info.width)==0
    resolution=float(info.resolution)
    ox,oy=float(info.origin.position.x),float(info.origin.position.y)
    rows,cols=np.indices(free.shape)
    x,y=ox+(cols+0.5)*resolution,oy+(rows+0.5)*resolution
    for portal in scope['projected_portals']:
        gate=np.asarray(portal['gate'],dtype=float)
        direction=np.asarray(portal['destination'],dtype=float)-gate
        length=np.linalg.norm(direction)
        if length<=1e-6 or float(portal['width'])<=0:continue
        direction/=length
        longitudinal=(x-gate[0])*direction[0]+(y-gate[1])*direction[1]
        transverse=-(x-gate[0])*direction[1]+(y-gate[1])*direction[0]
        bounds=portal.get('bounds')
        if bounds and len(bounds)==2 and all(point is not None for point in bounds):
            start,end=np.asarray(bounds,dtype=float);vector=end-start
            length2=float(np.dot(vector,vector))
            if length2>1e-12:
                fraction=np.minimum(1.,np.maximum(0.,((x-start[0])*vector[0]+(y-start[1])*vector[1])/length2))
                nearest_x,nearest_y=start[0]+fraction*vector[0],start[1]+fraction*vector[1]
                free[np.hypot(x-nearest_x,y-nearest_y)<=resolution]=False
                continue
        free[(np.abs(longitudinal)<=resolution)&(np.abs(transverse)<=float(portal['width'])/2+resolution)]=False
    def cell(point):
        return int(np.floor((point[1]-oy)/resolution)),int(np.floor((point[0]-ox)/resolution))
    seed=cell(scope['seed_map_xy'])
    def flood_from_seed(available):
        labels=np.zeros(available.shape,dtype=np.uint8)
        if not (0<=seed[0]<info.height and 0<=seed[1]<info.width and available[seed]):
            return labels,0
        labels[seed]=1;pending=deque([seed])
        while pending:
            row,col=pending.popleft()
            for r,c in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
                if 0<=r<info.height and 0<=c<info.width and available[r,c] and not labels[r,c]:
                    labels[r,c]=1;pending.append((r,c))
        return labels,1
    labels,seed_label=flood_from_seed(free)
    # A finite section may acquire a SLAM bypass after physical binding.
    # Reconstruct the runtime's bound Place-owned side independently, using
    # the original flood fill and the Portal's source/destination identities.
    leaks,_=portal_side_separation(message,scope,labels,seed_label,cell)
    for portal in scope['projected_portals']:
        source,destination=portal.get('source_place_id'),portal.get('destination_place_id')
        place=scope.get('place_id')
        if (portal['id'] not in leaks or source is None
                or source==destination or place not in (source,destination)):
            continue
        direction=np.asarray(portal['destination'],dtype=float)-np.asarray(portal['gate'],dtype=float)
        direction/=np.linalg.norm(direction)
        depth=(x-portal['gate'][0])*direction[0]+(y-portal['gate'][1])*direction[1]
        sign=-1. if place==source else 1.
        labels[sign*depth < -resolution/2.]=0
    # Clipping may disconnect remote fragments. Independently flood again
    # from the fixed entrance, rather than accepting all surviving pixels.
    if leaks:
        labels,seed_label=flood_from_seed(labels>0)
    return labels,seed_label,cell


def portal_side_separation(message,scope,labels,seed_label,cell):
    """Detect a known free bypass even if the dispatched goal passed its mask."""
    raw=np.asarray(message.data).reshape(message.info.height,message.info.width)
    leaks=[];checked=0
    for portal in scope['projected_portals']:
        gate=np.asarray(portal['gate']);direction=np.asarray(portal['destination'])-gate
        length=np.linalg.norm(direction)
        if length<=1e-6:continue
        source=gate-direction/length*max(.75,2*float(message.info.resolution))
        cells=[cell(source),cell(portal['destination'])]
        if not all(0<=r<raw.shape[0] and 0<=c<raw.shape[1] and raw[r,c]==0 for r,c in cells):continue
        if labels[cells[0]]==seed_label and labels[cells[1]]==seed_label:
            leaks.append(portal['id'])
        elif seed_label in (labels[cells[0]],labels[cells[1]]):checked+=1
    return leaks,checked



def bound_portal_identities_valid(event, scope):
    """A side clip needs a matching physically crossed canonical Portal."""
    records={p['id']:p for p in (event.get('portal_hypotheses') or {}).get('records',[])}
    for portal in scope.get('projected_portals',[]):
        if portal.get('source_place_id') is None:
            return False
        record=records.get(portal['id'],{})
        if portal.get('destination_place_id') is None:
            # An unbound edge may constrain only its proven source. Its far
            # side has no Place identity until a real crossing transaction.
            branch=next((b for b in (event.get('tree') or {}).get('branches',[])
                         if b.get('id')==record.get('gp_authentication_branch_id')), {})
            evidence=branch.get('authentication_evidence') or {}
            names=[e.get('event') for e in evidence.get('events',[])]
            try:
                sequence=[names.index(name) for name in ('doorway_entry_started','vehicle_fully_inside',
                          'inside_frontier_observed','room_facing_turn_complete','returned_to_junction_node')]
                bounds=np.asarray(record.get('physical_bounds'),dtype=float)
                valid=(record.get('state') in ('certified','selected')
                       and record.get('gp_authentication_state')=='completed'
                       and record.get('source_place_id')==portal['source_place_id']==scope.get('place_id')
                       and record.get('destination_place_id') is None
                       and int(record.get('crossing_count',0))==0
                       and branch.get('canonical_portal_id')==portal['id']
                       and branch.get('authentication_state')=='completed'
                       and evidence.get('state')=='completed'
                       and evidence.get('source_side_proven') is True
                       and evidence.get('return_gate_cleared') is True
                       and sequence==sorted(sequence)
                       and bounds.shape==(2,2) and np.all(np.isfinite(bounds))
                       and np.linalg.norm(bounds[1]-bounds[0])>1e-6
                       and float(record.get('opening_width',0))>0)
            except (KeyError,ValueError,TypeError):valid=False
            if not valid:return False
            continue
        if (record.get('source_place_id')!=portal['source_place_id']
                or record.get('destination_place_id')!=portal['destination_place_id']
                or record.get('gp_authentication_state')!='completed'
                or int(record.get('crossing_count',0))<1):
            return False
    return True


def entry_destination_in_component(scope, labels, region, cell):
    """The DFS child's component must contain its actual entrance's inside side."""
    entry_id = scope.get('entry_portal_id')
    if entry_id is None:
        return True
    entry = next((p for p in scope['projected_portals'] if p['id'] == entry_id), None)
    if entry is None or region <= 0:
        return False
    row, col = cell(entry['destination'])
    return bool(0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]
                and labels[row, col] == region)


def completion_frontier_cells(message, scope, labels, region):
    """Independently reconstruct the navigation frontier used by the WI ledger.

    Portal inside-evidence counts still use the raw sensor boundary. Completion
    counts use the existing obstacle-clearance mask, whose physical parameter
    is recorded with the exact map snapshot. No frontier-size cutoff is used.
    """
    raw=np.asarray(message.data).reshape(message.info.height,message.info.width)
    unknown=np.pad(raw<0,1,constant_values=False)
    adjacent=unknown[:-2,1:-1]|unknown[2:,1:-1]|unknown[1:-1,:-2]|unknown[1:-1,2:]
    free=raw==0
    clearance=scope.get('frontier_clearance_m')
    if clearance is not None:
        import math
        clearance=float(clearance)
        if not math.isfinite(clearance) or clearance<0:raise ValueError('invalid recorded frontier clearance')
        radius=max(1,int(math.ceil(clearance/float(message.info.resolution))))
        obstacle=raw>=50
        padded=np.pad(obstacle,radius,constant_values=False)
        unsafe=np.zeros(raw.shape,dtype=bool)
        for dr in range(-radius,radius+1):
            for dc in range(-radius,radius+1):
                if dr*dr+dc*dc<=radius*radius:
                    unsafe|=padded[radius+dr:radius+dr+raw.shape[0],radius+dc:radius+dc+raw.shape[1]]
        free &= ~unsafe
    return {tuple(map(int,cell)) for cell in np.argwhere(free&adjacent&(labels==region))} if region>0 else set()


def physical_handoff_evidence(portal, pose_xy, reverse=False, resolution=.1):
    """Independently check a recorded base pose against the certified section."""
    try:
        required=float(portal['gp_authentication_required_depth'])
        gate=np.asarray(portal['physical_gate'],dtype=float)
        inside=np.asarray(portal['physical_destination'],dtype=float)
        pose=np.asarray(pose_xy,dtype=float)
        if (not math.isfinite(required) or required<=0 or
                any(v.shape!=(2,) or not np.all(np.isfinite(v)) for v in (gate,inside,pose))):
            raise ValueError('invalid crossing data')
        bounds=portal.get('physical_bounds')
        alignment=0.
        if bounds is not None:
            a,b=np.asarray(bounds,dtype=float)
            tangent=b-a;length=float(np.linalg.norm(tangent))
            if not math.isfinite(length) or length<=1e-6:raise ValueError('invalid section')
            normal=np.asarray([-tangent[1],tangent[0]])/length
            if np.dot(inside-gate,normal)<0:normal=-normal
            alignment=abs(float(np.dot(gate-a,normal)))
            depth=float(np.dot(pose-a,normal))
        else:
            normal=inside-gate;length=float(np.linalg.norm(normal))
            if length<=1e-6:raise ValueError('invalid direction')
            depth=float(np.dot(pose-gate,normal/length))
        if reverse:depth=-depth
        tolerance=float(resolution)/2.
        reason=('canonical gate and certified section disagree' if alignment>tolerance+1e-6 else
                'full-body destination depth not reached' if depth+tolerance<required else '')
        return dict(verified=not reason,reason=reason,depth_m=depth,required_depth_m=required,
                    gate_alignment_error_m=alignment,map_tolerance_m=tolerance)
    except (KeyError,TypeError,ValueError,IndexError):
        return dict(verified=False,reason='missing or invalid full-body crossing geometry')


def audit(metrics,bag_path):
    import rosbag
    requested=[];inside_requests=[];completion_requests=[];errors=[];checked=[];inside_checked=[];completion_checked=[];separation_checks=0
    entry_by_place = {}
    handoffs=[];handoff_checks=[]
    for line,event in events(metrics):
        for node in (event.get('place_dfs') or {}).get('nodes', []):
            if node.get('parent_id') is not None and node.get('portal_id') is not None:
                entry_by_place[node['place_id']] = node['portal_id']
        if event.get('place_scope'):
            scope = dict(event['place_scope'])
            # Invalidated scopes are historical diagnostics, never positive
            # membership evidence. Dispatch/completion below require validity.
            if scope.get('valid') and not bound_portal_identities_valid(event,scope):
                errors.append(dict(line=line,reason='projected bound Portal lacks matching physical crossing identity'))
            entry_id = entry_by_place.get(scope.get('place_id'))
            if entry_id is not None:
                if scope.get('entry_portal_id') not in (None, entry_id):
                    errors.append(dict(line=line,reason='scope entry disagrees with DFS edge'))
                scope['entry_portal_id'] = entry_id
            event = dict(event, place_scope=scope)
        if event.get('frontier_event') in ('place_dfs_entered','place_dfs_returned'):
            records={p['id']:p for p in (event.get('portal_hypotheses') or {}).get('records',[])}
            handoffs.append((line,{key:event.get(key) for key in
                ('frontier_event','place_id','portal_id','ros_time','place_scope')},
                records.get(event.get('portal_id'),{})))
        if event.get('frontier_event')=='portal_authentication_inside_evidence':
            scope=event.get('inside_frontier_scope') or {}
            if scope.get('map_stamp') is None or not scope.get('seed_map_xy'):
                errors.append(dict(line=line,reason='missing entered-component frontier evidence'))
            else:inside_requests.append((line,{key:event.get(key) for key in
                ('branch_id','state','left_frontier_cells','right_frontier_cells')},scope))
        if event.get('frontier_event')=='place_dfs_completion_ready':
            scope=event.get('place_scope') or {}
            if not scope.get('valid') or scope.get('map_stamp') is None:
                errors.append(dict(line=line,reason='missing valid completion map snapshot'))
            else:completion_requests.append((line,dict(place_id=event.get('place_id'),
                work_items=[{key:item.get(key) for key in ('id','state','place_id')}
                            for item in event.get('work_items',[])]),scope))
        if event.get('frontier_event')!='work_item_dispatched':continue
        scope=event.get('place_scope') or {}
        membership=event.get('place_membership') or {}
        if not scope.get('valid') or scope.get('map_stamp') is None or not membership.get('viewpoint_map'):
            errors.append(dict(line=line,reason='missing recorded map or viewpoint'));continue
        requested.append((line,{key:event.get(key) for key in
            ('place_id','work_item_id')},scope,membership))
    wanted={round(float(scope['map_stamp']),6) for _,_,scope,_ in requested}
    wanted.update(round(float(scope['map_stamp']),6) for _,_,scope in inside_requests)
    wanted.update(round(float(scope['map_stamp']),6) for _,_,scope in completion_requests)
    wanted.update(round(float((event.get('place_scope') or {})['map_stamp']),6)
                  for _,event,_ in handoffs if (event.get('place_scope') or {}).get('map_stamp') is not None)
    maps={};poses=[]
    with rosbag.Bag(bag_path) as bag:
        for topic,message,received in bag.read_messages(topics=['/map','/rbt_pose']):
            if topic=='/map':
                stamp=round(message.header.stamp.to_sec(),6)
                if stamp in wanted:maps[stamp]=message
            else:poses.append((received.to_sec(),message.x,message.y))
    pose_times=[row[0] for row in poses]
    for line,event,portal in handoffs:
        time=event.get('ros_time')
        index=-1 if time is None else bisect_right(pose_times,float(time))-1
        if index<0 or float(time)-poses[index][0]>.15:
            errors.append(dict(line=line,reason='raw odometry pose absent at physical Place handoff'))
            continue
        scope=event.get('place_scope') or {};map_stamp=scope.get('map_stamp')
        message=maps.get(round(float(map_stamp),6)) if map_stamp is not None else None
        resolution=.1 if message is None else float(message.info.resolution)
        proof=physical_handoff_evidence(portal,poses[index][1:],
              reverse=event['frontier_event']=='place_dfs_returned',resolution=resolution)
        handoff_checks.append(dict(line=line,place_id=event.get('place_id'),portal_id=event.get('portal_id'),
            event=event['frontier_event'],pose_time=poses[index][0],pose_odom=list(poses[index][1:]),**proof))
        if not proof['verified']:
            errors.append(dict(line=line,reason='physical Place handoff: '+proof['reason'],
                               place_id=event.get('place_id'),portal_id=event.get('portal_id')))
    for line,event,scope,membership in requested:
        stamp=round(float(scope['map_stamp']),6);message=maps.get(stamp)
        if message is None:
            errors.append(dict(line=line,reason='exact SLAM snapshot absent from bag',map_stamp=stamp));continue
        labels,seed_label,cell=component_from_map(message,scope)
        if not entry_destination_in_component(scope, labels, seed_label, cell):
            errors.append(dict(line=line,reason='Place component excludes its DFS entrance destination',
                               place_id=scope.get('place_id'),entry_portal_id=scope.get('entry_portal_id')))
        leaks,count=portal_side_separation(message,scope,labels,seed_label,cell)
        separation_checks+=count
        if leaks:errors.append(dict(line=line,reason='certified Portal cut leaks into its other side',portal_ids=leaks))
        row,col=cell(membership['viewpoint_map'])
        inside=bool(seed_label>0 and 0<=row<labels.shape[0] and 0<=col<labels.shape[1]
                    and labels[row,col]==seed_label)
        place=event.get('place_id')
        if not inside or place!=scope.get('place_id'):
            errors.append(dict(line=line,reason='WI is outside reconstructed current Place',
                               place_id=place,map_stamp=stamp,cell=[row,col]))
        checked.append(dict(line=line,place_id=place,work_item_id=event.get('work_item_id'),
                            map_stamp=stamp,inside=inside,cell=[row,col]))
    for line,event,scope in inside_requests:
        stamp=round(float(scope['map_stamp']),6);message=maps.get(stamp)
        if message is None:
            errors.append(dict(line=line,reason='inside frontier SLAM snapshot absent',map_stamp=stamp));continue
        labels,region,_=component_from_map(message,scope)
        raw=np.asarray(message.data).reshape(message.info.height,message.info.width)
        unknown=np.pad(raw<0,1,constant_values=False)
        adjacent=unknown[:-2,1:-1]|unknown[2:,1:-1]|unknown[1:-1,:-2]|unknown[1:-1,2:]
        cells=np.argwhere((raw==0)&adjacent&(labels==region)) if region>0 else np.empty((0,2),dtype=int)
        x=message.info.origin.position.x+(cells[:,1]+.5)*message.info.resolution
        y=message.info.origin.position.y+(cells[:,0]+.5)*message.info.resolution
        heading=float(scope['heading_map']);gate=scope['gate_map_xy']
        lateral=-(x-gate[0])*np.sin(heading)+(y-gate[1])*np.cos(heading)
        left,right=int(np.count_nonzero(lateral>=0)),int(np.count_nonzero(lateral<0))
        if (left!=event.get('left_frontier_cells') or right!=event.get('right_frontier_cells')
                or (event.get('state')=='turning' and left+right==0)):
            errors.append(dict(line=line,reason='inside frontier evidence differs from entered SLAM component',
                               expected_left=left,expected_right=right))
        inside_checked.append(dict(line=line,branch_id=event.get('branch_id'),map_stamp=stamp,left=left,right=right))
    for line,event,scope in completion_requests:
        stamp=round(float(scope['map_stamp']),6);message=maps.get(stamp)
        if message is None:
            errors.append(dict(line=line,reason='completion SLAM snapshot absent'));continue
        labels,region,cell=component_from_map(message,scope)
        if not entry_destination_in_component(scope, labels, region, cell):
            errors.append(dict(line=line,reason='completion component excludes its DFS entrance destination'))
        try:actual=completion_frontier_cells(message,scope,labels,region)
        except (TypeError,ValueError):
            errors.append(dict(line=line,reason='invalid completion frontier clearance'));continue
        proof=scope.get('frontier_work_evidence') or {}
        claimed={tuple(cell[:2]) for cell in proof.get('resolved_frontier_cells',[])}
        if (region<=0 or len(actual)!=scope.get('frontier_count') or actual!=claimed
                or not completed_frontier_accounted(scope,event.get('work_items',[]),event.get('place_id'))):
            errors.append(dict(line=line,reason='completion residual frontiers lack exact SLAM and resolved WI proof',
                               raw_count=len(actual),claimed_count=len(claimed)))
        completion_checked.append(dict(line=line,place_id=event.get('place_id'),map_stamp=stamp,
                                       raw_frontier_count=len(actual),
                                       frontier_clearance_m=scope.get("frontier_clearance_m")))
    return dict(passed=bool(checked) and bool(inside_checked) and separation_checks>0 and not errors,checked=checked,errors=errors,portal_separation_checks=separation_checks,inside_frontier_checks=inside_checked,completion_frontier_checks=completion_checked,
                physical_handoff_checks=handoff_checks,
                independent_component_backend='standalone_four_connected_flood_fill',metrics=str(metrics),bag=str(bag_path))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metrics');parser.add_argument('bag');parser.add_argument('--output')
    args=parser.parse_args();result=audit(args.metrics,args.bag)
    text=json.dumps(result,indent=2);print(text)
    if args.output:Path(args.output).write_text(text+'\n')
    raise SystemExit(0 if result['passed'] else 1)

if __name__=='__main__':main()
