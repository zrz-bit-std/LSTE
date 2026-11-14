"""SLAM confirmation at the handoff from completed nested observation to Portal."""
import math

import numpy as np

from global_frontier_grid import frontier_mask
from global_frontier_geometry import map_snapshot_projector
from global_frontier_place_partition import portal_partition_evidence, branch_opening_width, branch_gate_xy, refine_portal_cut, authentication_gate_geometry, partition_place, point_in_place_scope


def _wall_tangent_cos(free, unknown, resolution, origin, point, section_dir,
                      radius_m=0.55):
    """Return |cos(section, local wall direction)| at one gate endpoint.

    Occupied cells are ``not free and not unknown``.  The wall direction is the
    principal axis of the occupied cells inside ``radius_m``.  ``None`` means
    there is not enough wall evidence to judge.
    """
    try:
        point = (float(point[0]), float(point[1]))
    except (IndexError, TypeError, ValueError):
        return None
    try:
        col = int(math.floor((point[0] - origin[0]) / resolution))
        row = int(math.floor((point[1] - origin[1]) / resolution))
    except (TypeError, ValueError):
        return None
    free = np.asarray(free, dtype=bool)
    if unknown is None:
        unknown = np.zeros_like(free)
    unknown = np.asarray(unknown, dtype=bool)
    occupied = (~free) & (~unknown)
    if not (0 <= row < occupied.shape[0] and 0 <= col < occupied.shape[1]):
        return None
    cells = max(1, int(math.ceil(float(radius_m) / max(1e-6, resolution))))
    r0, r1 = max(0, row - cells), min(occupied.shape[0], row + cells + 1)
    c0, c1 = max(0, col - cells), min(occupied.shape[1], col + cells + 1)
    rows, cols = np.nonzero(occupied[r0:r1, c0:c1])
    if rows.size < 4:
        return None
    xs = origin[0] + (cols + c0 + 0.5) * resolution
    ys = origin[1] + (rows + r0 + 0.5) * resolution
    x = xs - xs.mean()
    y = ys - ys.mean()
    cov = np.array([[float((x * x).mean()), float((x * y).mean())],
                    [float((x * y).mean()), float((y * y).mean())]])
    values, vectors = np.linalg.eigh(cov)
    tangent = vectors[:, int(np.argmax(values))]
    length = math.hypot(float(tangent[0]), float(tangent[1]))
    if length <= 1e-9:
        return None
    cos = abs(float(tangent[0]) * section_dir[0]
              + float(tangent[1]) * section_dir[1]) / length
    return max(0.0, min(1.0, cos))


def portal_cut_suspect(width, left_cos, right_cos, destination_inside):
    """Report-only classifier for a cut across a room's free space."""
    try:
        width = None if width is None else float(width)
    except (TypeError, ValueError):
        width = None
    return bool(
        width is not None and width >= 2.5
        and left_cos is not None and right_cos is not None
        and left_cos < 0.35 and right_cos < 0.35
        and destination_inside is True
    )


def suppress_suspect_portal_cut(evidence, suspect, enabled):
    """Turn a suspect cut into a child-place veto when suppression is enabled.

    The diagnostic itself remains report-only: this helper is the single
    explicit boundary between observation and behavior.  It only rejects the
    child-place decision; the caller's portal geometry and existing places are
    untouched.
    """
    if not enabled or not suspect or not isinstance(evidence, dict):
        return evidence
    result = dict(evidence)
    result["portal_cut_suspect"] = True
    if not result.get("reject"):
        result["reject"] = True
        result["reason"] = "portal_cut_suspect"
    return result


def _publish_portal_cut_diagnostic(node, message, context, auth, evidence,
                                   source, gate, destination, width, bounds,
                                   existing):
    """Publish one report-only audit for an authenticated Portal cut."""
    publish = getattr(node, "publish_status", None)
    if not callable(publish):
        return None
    try:
        tree = getattr(node, "gp_exploration_tree", None)
        branch = (None if tree is None
                  else tree.branches.get(getattr(auth, "branch_id", None)))
        portal_id = None if not isinstance(branch, dict) else branch.get(
            "canonical_portal_id")
        epoch = int(getattr(getattr(context, "components", None), "epoch", 0) or 0)
        key = (int(getattr(auth, "node_id", 0) or 0),
               int(getattr(auth, "branch_id", 0) or 0), epoch)
        seen = getattr(node, "_portal_cut_diagnostic_seen", None)
        if seen is None:
            seen = set()
            node._portal_cut_diagnostic_seen = seen
        if key in seen:
            return None
        seen.add(key)

        resolution = float(message.info.resolution)
        origin = (float(message.info.origin.position.x),
                  float(message.info.origin.position.y))
        free = np.asarray(context.known_free, dtype=bool)
        unknown = getattr(context, "unknown", None)
        if unknown is None:
            unknown = np.zeros_like(free)
        section_deg = None
        left_cos = right_cos = None
        if bounds and len(bounds) == 2 and all(b is not None for b in bounds):
            ax, ay = map(float, bounds[0])
            bx, by = map(float, bounds[1])
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-6:
                section = ((bx - ax) / length, (by - ay) / length)
                section_deg = round(math.degrees(math.atan2(
                    section[1], section[0])), 1)
                left_cos = _wall_tangent_cos(
                    free, unknown, resolution, origin, (ax, ay), section)
                right_cos = _wall_tangent_cos(
                    free, unknown, resolution, origin, (bx, by), section)

        destination_inside = None
        try:
            destination_inside = point_in_place_scope(node, destination)
        except Exception:
            destination_inside = None

        pre_frontiers = post_frontiers = None
        try:
            epoch = int(getattr(
                getattr(context, "components", None), "epoch", 1) or 1)
            pre_mask, _pre_comp, _pre_label, _used = partition_place(
                free, resolution, origin, source, existing, epoch)
            candidate = dict(
                id=0, gate=gate, destination=destination,
                width=float(width), bounds=bounds)
            _post_mask, post_comp, _post_label, _used2 = partition_place(
                free, resolution, origin, source, [*existing, candidate], epoch)
            frontier = frontier_mask(free, unknown)
            pre_frontiers = int(np.count_nonzero(frontier & pre_mask))
            col = int(math.floor((destination[0] - origin[0]) / resolution))
            row = int(math.floor((destination[1] - origin[1]) / resolution))
            labels = getattr(post_comp, "labels", None)
            if (labels is not None
                and 0 <= row < labels.shape[0]
                and 0 <= col < labels.shape[1]):
                label = int(labels[row, col])
                if label > 0:
                    post_frontiers = int(np.count_nonzero(
                        frontier & (labels == label)))
        except Exception:
            pre_frontiers = post_frontiers = None

        suspect = portal_cut_suspect(
            width, left_cos, right_cos, destination_inside)
        publish(
            "portal_cut_suspect_diagnostic",
            node_id=int(getattr(auth, "node_id", 0) or 0),
            branch_id=int(getattr(auth, "branch_id", 0) or 0),
            portal_id=None if portal_id is None else int(portal_id),
            source_place_id=getattr(node, "current_physical_place_id", None),
            width=None if width is None else round(float(width), 3),
            section_deg=section_deg,
            left_wall_cos=None if left_cos is None else round(left_cos, 3),
            right_wall_cos=None if right_cos is None else round(right_cos, 3),
            destination_inside_scope=destination_inside,
            pre_cut_frontiers=pre_frontiers,
            post_cut_frontiers=post_frontiers,
            suspect=bool(suspect),
            gate=[round(float(v), 3) for v in gate],
            destination=[round(float(v), 3) for v in destination],
            evidence_reason=str(evidence.get("reason", "")),
        )
        return suspect
    except Exception:
        return None


def _validation_snapshot_projector(node,message,context,source_xy):
    """Refresh the source partition and use its one exact-map TF for all witnesses."""
    update=getattr(node,'_update_place_partition',None)
    current=getattr(node,'current_physical_place_id',None)
    stamp=getattr(getattr(message,'header',None),'stamp',None)
    if not callable(update) or current is None or source_xy is None or stamp is None:return None
    project=map_snapshot_projector(node,message)
    if project is None:
        acquire=getattr(node,'planar_xy_projector',None)
        if not callable(acquire):return None
        project=acquire(message.header.frame_id or 'map','odom',stamp=stamp)
    if project is None:return None
    source=project(*source_xy)
    if source is None:return None
    update(message,context.known_free,frontier_mask(context.known_free,context.unknown),
           current,source,getattr(context.components,'epoch',1))
    # Never borrow the previous map's cache or switch to latest on TF eviction.
    return map_snapshot_projector(node,message)


def authenticated_place_portal_evidence(node, auth):
    room = getattr(node, 'active_semantic_room', None)
    context = getattr(node, 'portal_authentication_map_context', None)
    message = getattr(node, 'portal_authentication_map_message', None)
    update = getattr(node, '_update_place_partition', None)
    if room is None or context is None or message is None or not callable(update):
        return None
    snapshot=_validation_snapshot_projector(node,message,context,auth.source_xy)
    if snapshot is None:return None
    project = lambda xy: None if xy is None else snapshot(*xy)
    source = project(auth.source_xy)
    inside = project(auth.inside_xy)
    gate, destination = project(auth.gate_xy), project(auth.destination_xy)
    tree = getattr(node, 'gp_exploration_tree', None)
    branch = None if tree is None else tree.branches.get(auth.branch_id)
    width, bounds = authentication_gate_geometry(auth, branch)
    if any(value is None for value in (source,inside,gate,destination,width)):
        return None
    ledger = getattr(node,'portal_hypothesis_ledger',None)
    entry = None if ledger is None else ledger.get(room['portal_id'])
    entry_xy = project(None if entry is None else entry.get('physical_destination'))
    origin = (message.info.origin.position.x,message.info.origin.position.y)
    resolution = float(message.info.resolution)
    epoch = getattr(context.components,'epoch',1)
    existing = getattr(node,'current_place_scope_projected_portals',())
    candidate = refine_portal_cut(dict(id=0,gate=gate,destination=destination,width=float(width),
        bounds=[None if b is None else project(b)
                for b in (bounds or [])]),
        context.known_free,resolution,origin)
    evidence = portal_partition_evidence(context.known_free,resolution,origin,source,inside,
        candidate,existing,entry_xy,epoch)
    if evidence.get('reason') == 'slam_evidence_incomplete' and evidence.get('source_label') == 0:
        # The frozen junction anchor can fall inside a refined gate cut after
        # SLAM correction. Use the actual returned source-side pose only when
        # its physical return and membership in the parent Place are proven.
        import math
        from global_frontier_place_partition import point_in_place_scope
        pose = getattr(node, 'pose_odom', None)
        returned_odom = None if pose is None else (float(pose.x), float(pose.y))
        returned = project(returned_odom)
        if (returned_odom is not None and returned is not None
                and auth.return_gate_cleared
                and math.hypot(returned_odom[0]-auth.source_xy[0],
                               returned_odom[1]-auth.source_xy[1]) <= auth.return_radius
                and auth.signed_depth(returned_odom) < -0.05
                and point_in_place_scope(node, returned) is True):
            evidence = portal_partition_evidence(context.known_free,resolution,origin,returned,inside,
                candidate,existing,entry_xy,epoch)
            evidence.update(source_witness='physical_return_pose',source_map_xy=list(returned),
                            frozen_source_map_xy=list(source))
    if width is not None and gate is not None and destination is not None:
        suspect = _publish_portal_cut_diagnostic(
            node, message, context, auth, evidence, source, gate, destination,
            width, bounds, existing)
        evidence = suppress_suspect_portal_cut(
            evidence, suspect,
            getattr(node, "portal_cut_suspect_suppression_enabled", False))
    return evidence


def duplicate_observation_portal(node, tree, branch, message, context, goal_depth=1.3):
    """Recognize a GP alias only when SLAM puts it behind an existing cut."""
    update=getattr(node,'_update_place_partition',None)
    ledger=getattr(node,'portal_hypothesis_ledger',None)
    current=getattr(node,'current_physical_place_id',None)
    if message is None or context is None or not callable(update) or ledger is None or current is None:
        return None
    source_node=tree.nodes.get(branch.get('node_id'),{})
    source_xy=source_node.get('physical_xy');gate_xy=branch_gate_xy(branch)
    width=branch_opening_width(branch)
    if source_xy is None or gate_xy is None or width is None:return None
    import math
    heading=float(branch.get('heading',0.0))
    destination_xy=(gate_xy[0]+float(goal_depth)*math.cos(heading),gate_xy[1]+float(goal_depth)*math.sin(heading))
    from global_frontier_portal_crossing import portal_section_destination
    destination_xy=portal_section_destination(gate_xy,destination_xy,
        (branch.get('map_geometry') or {}).get('physical_bounds'))
    if destination_xy is None:return None
    project=_validation_snapshot_projector(node,message,context,source_xy)
    if project is None:return None
    source,gate,destination=map(lambda point:project(*point),(source_xy,gate_xy,destination_xy))
    if any(p is None for p in (source,gate,destination)):return None
    epoch=getattr(context.components,'epoch',1)
    result=portal_partition_evidence(context.known_free,float(message.info.resolution),
        (message.info.origin.position.x,message.info.origin.position.y),source,destination,
        refine_portal_cut(dict(id=0,gate=gate,destination=destination,width=float(width),bounds=[
            None if b is None else project(*b)
            for b in ((branch.get('map_geometry') or {}).get('physical_bounds') or [])]),
            context.known_free,float(message.info.resolution),
            (message.info.origin.position.x,message.info.origin.position.y)),
        getattr(node,'current_place_scope_projected_portals',()),epoch=epoch,
        current_place_id=current)
    if result.get('reason') == 'visited_entry_side_of_current_place':
        portal=ledger.get(result['duplicate_portal_id'])
        if (portal is not None and portal.get('destination_place_id') == current
                and portal.get('source_place_id') != current
                and portal.get('state') == 'crossed'
                and int(portal.get('crossing_count', 0)) > 0
                and portal.get('gp_authentication_state') == 'completed'):
            branch['connection'] = 'return'
            branch['return_source_place_id'] = current
            branch['return_destination_place_id'] = portal['source_place_id']
            branch['return_portal_evidence'] = dict(result)
            return portal
        return None
    if result.get('reason')!='duplicate_certified_place_boundary':return None
    portal=ledger.get(result['duplicate_portal_id'])
    if portal is None or portal.get('source_place_id')!=current or portal.get('gp_authentication_state')!='completed':
        return None
    return portal


def authentication_inside_frontiers(node,tree,auth,message,context,raw_frontier):
    """Keep inside evidence on the entered side of certified and candidate cuts."""
    import math
    from global_frontier_place_partition import partition_place
    branch=tree.branches.get(auth.branch_id,{})
    width,bounds=authentication_gate_geometry(auth,branch)
    if width is None or auth.inside_xy is None:return None,None
    snapshot=_validation_snapshot_projector(node,message,context,auth.source_xy)
    if snapshot is None:return None,None
    project=lambda point:None if point is None else snapshot(*point)
    source,inside,gate,destination=map(project,(auth.source_xy,auth.inside_xy,auth.gate_xy,auth.destination_xy))
    if any(point is None for point in (source,inside,gate,destination)):return None,None
    existing=list(getattr(node,'current_place_scope_projected_portals',()) or ())
    candidate=dict(id=-int(auth.branch_id),gate=gate,destination=destination,width=float(width),bounds=[
        None if point is None else project(point)
        for point in (bounds or [])])
    origin=message.info.origin.position
    candidate=refine_portal_cut(candidate,context.known_free,float(message.info.resolution),(origin.x,origin.y))
    cuts=existing+[candidate]
    mask,_,label,_=partition_place(context.known_free,message.info.resolution,(origin.x,origin.y),inside,cuts,
                                  getattr(context.components,'epoch',1))
    if label<=0:return None,None
    evidence=dict(map_stamp=message.header.stamp.to_sec(),seed_map_xy=list(inside),
                  projected_portals=cuts,gate_map_xy=list(gate),
                  heading_map=math.atan2(destination[1]-gate[1],destination[0]-gate[0]),
                  required_depth=float(auth.required_depth),inside_component_label=int(label),
                  frontier_selection='entire_entered_component')
    return raw_frontier & mask,evidence
