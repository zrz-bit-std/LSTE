"""Resolve an old boundary only when current SLAM proves its local arc observed."""
import math
import numpy as np
from global_frontier_grid import frontier_mask
from global_frontier_geometry import map_snapshot_projector as _map_snapshot_projector


def observed_boundary_evidence(known_free, frontier, place_mask, resolution, origin_xy, points, radius, occupied=None):
    """Known boundary witnesses and no remaining local frontier in their identity discs.

    A farther advancing frontier is a different WI under the ledger's existing
    anchor rule. Unknown space outside occupied walls is not an unobserved arc
    of this Place. Missing witnesses, map coverage or Place ownership fail closed.
    """
    if place_mask is None or not points:return None
    checked=0;witnesses=[];occupied_witnesses=[]
    for index,point in enumerate(points):
        if point is None or not all(math.isfinite(float(v)) for v in point):return None
        col=int(math.floor((point[0]-origin_xy[0])/resolution))
        row=int(math.floor((point[1]-origin_xy[1])/resolution))
        if not (0<=row<known_free.shape[0] and 0<=col<known_free.shape[1]):return None
        if not (known_free[row,col] and place_mask[row,col]):
            # The anchor must remain a known-free witness in this Place. A
            # boundary cell, however, may have been positively observed as a
            # wall. Require direct contact with this exact owned component;
            # unknown cells or walls belonging only to another Place fail.
            if index==0 or occupied is None or not occupied[row,col]:return None
            adjacent=[[r,c] for r,c in ((row-1,col),(row+1,col),(row,col-1),(row,col+1))
                      if 0<=r<known_free.shape[0] and 0<=c<known_free.shape[1]
                      and known_free[r,c] and place_mask[r,c]]
            if not adjacent:return None
            occupied_witnesses.append(dict(cell=[row,col],adjacent_place_free=adjacent))
        cells=int(math.ceil(radius/resolution))
        # No map-edge inference: the complete identity footprint must be recorded.
        if row-cells<0 or col-cells<0 or row+cells>=known_free.shape[0] or col+cells>=known_free.shape[1]:return None
        yy,xx=np.ogrid[-cells:cells+1,-cells:cells+1]
        disc=(xx*xx+yy*yy)*resolution*resolution<=radius*radius
        area=(slice(row-cells,row+cells+1),slice(col-cells,col+cells+1))
        if np.any(frontier[area]&place_mask[area]&disc):return None
        witnesses.append([row,col]);checked+=int(np.count_nonzero(disc))
    reason=('slam_boundary_observed_as_occupied_without_remaining_local_frontier'
            if occupied_witnesses else 'slam_boundary_observed_without_remaining_local_frontier')
    return dict(reason=reason,witness_cells=witnesses,identity_radius_m=float(radius),
                checked_cells=checked,occupied_boundary_witnesses=occupied_witnesses)



def settle_observed_place_work(node,message,known_free,unknown,place_mask,place_id,now):
    """Apply positive map evidence to inactive observation WIs, never Portal entries."""
    ledger=getattr(node,'place_work_items',None)
    project=_map_snapshot_projector(node,message)
    if ledger is None or not callable(project) or place_mask is None:return []
    if not callable(getattr(ledger,'snapshot',None)) or not callable(getattr(ledger,'resolve',None)):return []
    raw_frontier=frontier_mask(known_free,unknown)
    frame=message.header.frame_id or 'map'
    resolution=float(message.info.resolution)
    origin=message.info.origin.position
    radius=max(resolution,float(getattr(ledger,'merge_radius',resolution)))+resolution
    raw=getattr(message,'data',None)
    occupied=(None if raw is None else np.asarray(raw).reshape(known_free.shape)>=50)
    settled=[]
    for item in ledger.snapshot():
        if (item.get('place_id')!=place_id or item.get('state')!='unresolved'
                or item.get('kind')=='portal_entry' or item.get('active_attempt_id') is not None):continue
        physical=[item.get('anchor_xy'),item.get('place_boundary_xy')]
        if any(p is None for p in physical):continue
        points=[project(*p) for p in physical]
        proof=observed_boundary_evidence(known_free,raw_frontier,place_mask,resolution,
                                        (origin.x,origin.y),points,radius,occupied=occupied)
        if proof is None:continue
        result=ledger.resolve(item['id'],now,proof['reason'])
        if result is None:continue
        settled.append(item['id'])
        node.publish_status('work_item_map_observed',work_item_id=item['id'],place_id=int(place_id),
                            work_item_state='resolved',map_stamp=message.header.stamp.to_sec(),
                            evidence=proof,physical_boundary_witnesses=physical)
    return settled


def frontier_work_evidence(frontier, place_mask, cell_to_item, details, map_stamp):
    """Account for every current boundary cell using the existing WI ledger.

    A resolved descendant keeps its WI's completed observation contract. New,
    unresolved and unassigned cells remain pending; no size/noise threshold or
    timeout may silently erase them.
    """
    cells=np.argwhere(frontier & place_mask)
    settled=[];pending=0
    for row,col in cells:
        key=(int(row),int(col));item_id=cell_to_item.get(key)
        detail=details.get(item_id,{})
        if item_id is not None and detail.get('state')=='resolved':
            settled.append([key[0],key[1],int(item_id)])
        else:pending+=1
    return dict(map_stamp=float(map_stamp),raw_count=len(cells),pending_count=pending,
                resolved_count=len(settled),
                resolved_frontier_cells=settled if pending==0 else [])


def observe_empty_place_scope(node, message, known_free, unknown, place_mask, place_id, now):
    """Record a real in-Place viewpoint when raw SLAM has no local frontier.

    Portal cuts can leave all durable frontier WIs on the as-yet-unbound child
    side. They stay unresolved; this proves only the current fragment observed,
    so bootstrap cannot deadlock on an out-of-scope WI before crossing its Portal.
    """
    if (place_mask is None or not np.any(place_mask)
            or not getattr(node, 'current_place_scope_valid', False)
            or getattr(node, 'current_place_scope_id', None) != place_id
            or np.any(frontier_mask(known_free, unknown) & place_mask)):
        return False
    auth = getattr(node, 'active_portal_authentication', lambda: None)()
    task = getattr(node, 'junction_portal_authentication', None)
    if auth is not None or (task is not None and not task.all_finished()):
        return False
    pose = getattr(node, 'pose_odom', None)
    record = getattr(getattr(node, 'region_memory', None), 'endpoint_observed', None)
    if pose is None or not callable(record):
        return False
    frame = message.header.frame_id or 'map'
    project = _map_snapshot_projector(node, message)
    if project is None:
        return False
    xy = project(float(pose.x), float(pose.y))
    if xy is None:
        return False
    col = int(math.floor((xy[0]-message.info.origin.position.x)/message.info.resolution))
    row = int(math.floor((xy[1]-message.info.origin.position.y)/message.info.resolution))
    if not (0 <= row < place_mask.shape[0] and 0 <= col < place_mask.shape[1]
            and known_free[row, col] and place_mask[row, col]):
        return False
    region = record(xy[0], xy[1], now, region_id=place_id,
                    retain_for_target=bool(getattr(node, 'target_region_claim_active', False)),
                    physical_xy=(float(pose.x), float(pose.y)))
    if region is None:
        return False
    node.publish_status('place_observed_from_complete_scope', place_id=int(place_id),
                        map_stamp=message.header.stamp.to_sec(), robot_map_xy=list(xy),
                        raw_frontier_count=0, scope_cell_count=int(np.count_nonzero(place_mask)),
                        reason='current_pose_inside_fully_observed_slam_place')
    return True
