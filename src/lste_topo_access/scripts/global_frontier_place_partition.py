"""Online SLAM Place masks cut only at physically certified Portal gates."""
import math
import numpy as np
from global_frontier_topology_components import TopologicalFreeSpaceComponents


def branch_opening_width(branch):
    """Use a measured map opening, or the GP free-sector chord when unmapped."""
    branch=branch or {}
    width=(branch.get('map_geometry') or {}).get('width')
    try:
        width=float(width)
        if math.isfinite(width) and width>0:return width
    except (TypeError,ValueError):pass
    try:
        reach=float(branch['reach']);extent=float(branch['angular_width'])
        if not math.isfinite(reach) or not math.isfinite(extent) or reach<=0 or not 0<extent<=math.pi:
            return None
        return 2.0*reach*math.sin(extent/2.0)
    except (KeyError,TypeError,ValueError):return None


def measured_authentication_gate_geometry(branch):
    """Return only a finite SLAM section; GP sector chords remain candidates."""
    geometry=(branch or {}).get('map_geometry') or {}
    bounds=geometry.get('physical_bounds')
    try:
        width=float(geometry.get('width'))
        points=np.asarray(bounds,dtype=float)
        if (not math.isfinite(width) or width<=0 or points.shape!=(2,2)
                or not np.all(np.isfinite(points))
                or np.linalg.norm(points[1]-points[0])<=1e-6):return None
    except (TypeError,ValueError):return None
    return width,tuple(tuple(float(v) for v in point) for point in points)


def authentication_gate_geometry(auth, branch):
    """Use the entry-frozen section; old standalone callers retain their input."""
    frozen=getattr(auth,'gate_geometry',None)
    if frozen is not None:return frozen
    branch=branch or {}
    return branch_opening_width(branch),(branch.get('map_geometry') or {}).get('physical_bounds')


def branch_gate_xy(branch):
    """Freeze one coherent measured section before authentication starts."""
    branch=branch or {};geometry=branch.get('map_geometry') or {}
    bounds=geometry.get('physical_bounds')
    if bounds and len(bounds)==2 and all(point is not None for point in bounds):
        point=((bounds[0][0]+bounds[1][0])/2.,(bounds[0][1]+bounds[1][1])/2.)
    else:point=geometry.get('physical_position') or branch.get('physical_gate')
    if point is None:return None
    return tuple(map(float,point)) if all(math.isfinite(float(v)) for v in point) else None


def refine_portal_cut(portal, known_free, resolution, origin_xy):
    """Extend measured endpoints locally to the current SLAM non-free boundary.

    A rigid odom projection cannot model local scan-matching wall corrections.
    Endpoint extension and section translation are each bounded by half the
    measured opening width. A translation must preserve the original side
    witnesses. The existing bypass guard still rejects an incomplete cut.
    """
    bounds=portal.get('bounds')
    if not bounds or len(bounds)!=2 or any(p is None for p in bounds):return portal
    points=np.asarray(bounds,dtype=float)
    length=float(np.linalg.norm(points[1]-points[0]))
    if not np.all(np.isfinite(points)) or length<=1e-6:return portal
    limit=max(0.,float(portal['width'])/2.)
    step=float(resolution)/2.
    def attach(section):
        refined=section.copy();extensions=[0.,0.];attached=[]
        for side in (0,1):
            direction=(section[side]-section[1-side])/length
            def is_free(point):
                col=int(math.floor((point[0]-origin_xy[0])/resolution))
                row=int(math.floor((point[1]-origin_xy[1])/resolution))
                return bool(0<=row<known_free.shape[0] and 0<=col<known_free.shape[1]
                            and known_free[row,col])
            start=section[side].copy()
            inward=0.
            # An overlong projected endpoint may already lie inside the wall.
            # Walk back toward the section interior to find known free space,
            # then trace outward to the first non-free cell. An occupied
            # starting cell alone is not evidence that the endpoint is attached.
            if not is_free(start):
                found_free=False
                for distance in np.arange(step,limit+step/2.,step):
                    candidate=section[side]-distance*direction
                    if is_free(candidate):
                        start=candidate;inward=float(distance);found_free=True;break
                if not found_free:
                    attached.append(False);continue
            hit=False
            for distance in np.arange(step,limit+step/2.,step):
                point=start+distance*direction
                if not is_free(point):
                    refined[side]=point
                    extensions[side]=float(distance-inward)
                    hit=True;break
            attached.append(hit)
        return refined,extensions,all(attached)
    refined,extensions,attached=attach(points)
    shift=0.
    if not attached:
        # A frozen section can sit at a junction corner after SLAM correction.
        # Search only a bounded local translation normal to that same section;
        # never move it past either original physical side witness. Portal
        # identity, gate and destination remain unchanged.
        normal=np.array([points[1,1]-points[0,1],points[0,0]-points[1,0]])/length
        gate=np.asarray(portal['gate'],dtype=float)
        destination=np.asarray(portal['destination'],dtype=float)
        direction=destination-gate;distance=float(np.linalg.norm(direction))
        if distance>1e-6:
            source=gate-direction/distance*max(.75,2*resolution)
            for amount in np.arange(step,limit+step/2.,step):
                found=False
                for candidate_shift in (amount,-amount):
                    section=points+candidate_shift*normal
                    center=section.mean(axis=0)
                    if np.dot(source-center,normal)*np.dot(destination-center,normal)>=0:continue
                    candidate,extra,ok=attach(section)
                    if ok:
                        refined,extensions,shift=candidate,extra,float(candidate_shift)
                        found=True;break
                if found:break
    result=dict(portal,bounds=refined.tolist(),measured_bounds=points.tolist(),
                boundary_extension_m=extensions,section_translation_m=shift)
    return result


def partition_place(known_free, resolution, origin_xy, seed_xy, portals, epoch):
    """Return the current seed's free-space component with finite gate cuts.

    Input Portal records are projected map-frame gate/destination/width tuples.
    This read-only mask owns WI membership; navigation keeps its original grid.
    """
    free = np.asarray(known_free, dtype=bool).copy()
    resolution = float(resolution)
    ox, oy = map(float, origin_xy)
    rows, cols = np.indices(free.shape)
    x = ox + (cols + 0.5) * resolution
    y = oy + (rows + 0.5) * resolution
    used = []
    for portal in portals:
        gate, dest, width = portal['gate'], portal['destination'], float(portal['width'])
        dx, dy = float(dest[0])-float(gate[0]), float(dest[1])-float(gate[1])
        norm = math.hypot(dx, dy)
        if norm <= 1e-6 or width <= 0:
            continue
        dx, dy = dx/norm, dy/norm
        along = (x-gate[0])*dx + (y-gate[1])*dy
        across = -(x-gate[0])*dy + (y-gate[1])*dx
        bounds = portal.get('bounds')
        measured=False
        if bounds and len(bounds)==2 and all(point is not None for point in bounds):
            ax,ay=map(float,bounds[0]);bx,by=map(float,bounds[1])
            length2=(bx-ax)**2+(by-ay)**2
            if all(math.isfinite(v) for v in (ax,ay,bx,by)) and length2>1e-12:
                # Use the actual measured segment, including its section
                # position and tilt, rather than an old GP ray's gate plane.
                t=np.clip(((x-ax)*(bx-ax)+(y-ay)*(by-ay))/length2,0.,1.)
                free[(x-(ax+t*(bx-ax)))**2+(y-(ay+t*(by-ay)))**2<=resolution**2]=False
                measured=True
        if not measured:
            free[(np.abs(along)<=resolution)&(np.abs(across)<=width/2+resolution)]=False
        used.append(int(portal['id']))
    components = TopologicalFreeSpaceComponents(free, epoch)
    if seed_xy is None:
        return np.zeros_like(free), components, 0, used
    col = int(math.floor((float(seed_xy[0])-ox)/resolution))
    row = int(math.floor((float(seed_xy[1])-oy)/resolution))
    label = int(components.labels[row,col]) if 0<=row<free.shape[0] and 0<=col<free.shape[1] else 0
    mask = components.labels == label if label > 0 else np.zeros_like(free)
    return mask, components, label, used


def clip_place_mask_to_portal_side(mask, portal, place_id, resolution, origin_xy):
    """Clip a connected SLAM component to a certified Portal's Place-owned side.

    Finite measured cuts can leak when SLAM shows a bypass around the gate.
    Portal source and any bound destination identities define which side belongs
    to the current Place; use that relation only for the leaking certified
    edge, without inferring corridor/room geometry.
    """
    source = portal.get("source_place_id")
    destination = portal.get("destination_place_id")
    try:
        place_id = int(place_id)
        source = int(source)
        destination = None if destination is None else int(destination)
    except (TypeError, ValueError):
        return mask
    if source == place_id:
        keep_sign = -1.0
    elif destination == place_id:
        keep_sign = 1.0
    else:
        return mask
    gate = portal.get("gate")
    dest = portal.get("destination")
    if gate is None or dest is None:
        return mask
    dx = float(dest[0]) - float(gate[0])
    dy = float(dest[1]) - float(gate[1])
    length = math.hypot(dx, dy)
    if not math.isfinite(length) or length <= 1e-6:
        return mask
    dx, dy = dx / length, dy / length
    rows, cols = np.indices(mask.shape)
    x = float(origin_xy[0]) + (cols + 0.5) * float(resolution)
    y = float(origin_xy[1]) + (rows + 0.5) * float(resolution)
    along = (x - float(gate[0])) * dx + (y - float(gate[1])) * dy
    # Retain a half-cell tolerance at the boundary so a cell containing the
    # physical gate remains addressable from either side without admitting the
    # opposite Place's core.
    return np.asarray(mask, dtype=bool) & (keep_sign * along >= -float(resolution) / 2.0)


def leaking_portal_sides(mask, known_free, resolution, origin_xy, portals):
    """A Place mask may not contain both known sides of its certified boundary."""
    leaks=[]
    def inside(point):
        col=int(math.floor((point[0]-origin_xy[0])/resolution))
        row=int(math.floor((point[1]-origin_xy[1])/resolution))
        return bool(0<=row<mask.shape[0] and 0<=col<mask.shape[1]
                    and known_free[row,col] and mask[row,col])
    for portal in portals:
        gate,destination=portal['gate'],portal['destination']
        length=math.hypot(destination[0]-gate[0],destination[1]-gate[1])
        if length<=1e-6:continue
        margin=max(.75,2*resolution)
        source=(gate[0]-(destination[0]-gate[0])*margin/length,
                gate[1]-(destination[1]-gate[1])*margin/length)
        if inside(source) and inside(destination):leaks.append(int(portal['id']))
    return leaks


def portal_partition_evidence(known_free, resolution, origin_xy, source_xy,
                              destination_xy, candidate, existing_portals=(),
                              entry_xy=None, epoch=1, current_place_id=None):
    """Reject a known free-space bypass, duplicate child, or visited return arm.

    Incomplete SLAM evidence is not a mandatory wall-geometry gate: it remains
    inconclusive. Positive rejection requires both physical witnesses in known
    free components. No corridor/room category is inferred here.
    """
    _, base, source_label, _ = partition_place(
        known_free, resolution, origin_xy, source_xy, existing_portals, epoch)
    def label_at(components, point):
        if point is None:
            return 0
        col = int(math.floor((point[0]-origin_xy[0])/resolution))
        row = int(math.floor((point[1]-origin_xy[1])/resolution))
        return int(components.labels[row,col]) if (0<=row<known_free.shape[0] and 0<=col<known_free.shape[1]) else 0
    destination_label = label_at(base, destination_xy)
    if source_label > 0 and destination_label > 0 and destination_label != source_label:
        for portal in existing_portals:
            # Only the bound incoming edge can identify a visited parent arm.
            # Both witnesses must be in distinct known-free components; a
            # predicted heading, nearby gate, or unknown cell is insufficient.
            if (current_place_id is None
                    or portal.get('destination_place_id') != current_place_id
                    or portal.get('source_place_id') in (None, current_place_id)):
                continue
            gate, inside = portal['gate'], portal['destination']
            dx, dy = inside[0]-gate[0], inside[1]-gate[1]
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            margin = max(.75, 2*resolution)
            parent = (gate[0]-dx*margin/length, gate[1]-dy*margin/length)
            if (label_at(base, inside) == source_label
                    and label_at(base, parent) == destination_label):
                return dict(reject=True, reason='visited_entry_side_of_current_place',
                            duplicate_portal_id=portal['id'], source_label=source_label,
                            destination_label=destination_label,
                            entry_label=destination_label,
                            parent_place_id=portal['source_place_id'])
    if source_label > 0 and destination_label > 0 and destination_label != source_label:
        for portal in existing_portals:
            if label_at(base, portal['destination']) == destination_label:
                return dict(reject=True, reason='duplicate_certified_place_boundary',
                            duplicate_portal_id=portal['id'])
    _, cut, source_label, _ = partition_place(
        known_free, resolution, origin_xy, source_xy,
        [*existing_portals, candidate], epoch)
    destination_label = label_at(cut, destination_xy)
    entry_label = label_at(cut, entry_xy)
    result = dict(reject=False, reason='slam_evidence_incomplete',
                  source_label=source_label, destination_label=destination_label,
                  entry_label=entry_label)
    if source_label > 0 and destination_label > 0:
        if source_label == destination_label:
            result.update(reject=True, reason='known_free_bypass_within_current_place')
        elif entry_label > 0 and entry_label == destination_label:
            result.update(reject=True, reason='visited_entry_side_of_current_place')
        else:
            result['reason'] = 'distinct_destination_component'
    return result


def point_in_place_scope(node, point):
    """Return membership, or None when a legacy caller has no Place partition."""
    mask = getattr(node, 'current_place_scope_mask', None)
    message = getattr(node, 'current_place_scope_message', None)
    if mask is None or message is None:
        return None
    if (not getattr(node, 'current_place_scope_valid', False)
            or getattr(node, 'current_place_scope_id', None)
            != getattr(node, 'current_physical_place_id', None)):
        return False
    origin = message.info.origin.position
    col = int(math.floor((point[0] - origin.x) / message.info.resolution))
    row = int(math.floor((point[1] - origin.y) / message.info.resolution))
    return bool(0 <= row < mask.shape[0] and 0 <= col < mask.shape[1] and mask[row, col])
