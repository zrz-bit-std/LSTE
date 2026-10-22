"""Local polar GP proposals, independent of ROS and architectural door frames.

The legacy spherical GP models (angle, elevation) -> occupancy. This horizontal
projection models angle -> measured free range using the same periodic angular
neighbourhood idea. Posterior variance is evidence quality, never free space.
The original sensor ranges remain the safety evidence; GP is only a proposer.
"""
import math
from dataclasses import dataclass
import numpy as np


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


@dataclass(frozen=True)
class DirectionProposal:
    heading: float
    angular_width: float
    reach: float
    variance: float
    support: int


class PolarGaussianProcess:
    def __init__(self, bins=120, length_scale=0.12, noise=0.03):
        self.angles = np.linspace(-math.pi, math.pi, bins, endpoint=False)
        delta = self.angles[:, None] - self.angles[None, :]
        self.kernel = np.exp(-2*np.sin(delta/2)**2 / length_scale**2)
        self.noise = noise

    def propose(self, ranges, angle_min, angle_increment, range_min, range_max,
                minimum_reach=1.2, minimum_width=0.65):
        if angle_increment <= 0 or not math.isfinite(range_max) or range_max <= 0:
            return []
        values = np.asarray(ranges, dtype=float)
        if values.size < 3:
            return []
        raw_angles = angle_min + np.arange(values.size)*angle_increment
        # NaN and missing returns do not provide measured free-space evidence.
        finite = np.where(np.isfinite(values), values, -1.0)
        valid = (finite > range_min) & (finite <= range_max)
        if not valid.any():
            return []
        index = np.floor((raw_angles[valid]+math.pi) / (2*math.pi)
                         * len(self.angles) + 0.5).astype(int) % len(self.angles)
        observed = np.full(len(self.angles), np.nan)
        for i in np.unique(index):
            # Conservative lower quantile suppresses isolated long rays.
            observed[i] = np.quantile(values[valid][index == i], 0.25)
        ids = np.flatnonzero(np.isfinite(observed))
        k = self.kernel[np.ix_(ids, ids)] + np.eye(len(ids))*self.noise
        cross = self.kernel[:, ids]
        # Solve once for both posterior mean and variance, bounded by bins.
        solved = np.linalg.solve(k, np.column_stack((observed[ids], cross.T)))
        mean = cross @ solved[:, 0]
        variance = np.maximum(0, 1-np.sum(cross*solved[:, 1:].T, axis=1))
        # Connected free angular sectors avoid treating the two far corners
        # of a closed corridor end as two independent branches. The depth is
        # derived from the observed lateral scale, not a world-axis template.
        lateral=float(np.quantile(observed[ids], 0.20))
        options=[]
        for scale in (1.5,2.0,2.5):
            candidate_depth=max(minimum_reach,scale*lateral)
            mask=np.isfinite(observed)&(observed>=candidate_depth)&(mean>=candidate_depth)
            candidates=[i for i in range(len(mask)) if mask[i] and not mask[(i-1)%len(mask)]]
            accepted=[]
            for start in candidates:
                size=1; j=(start+1)%len(mask)
                while mask[j] and j!=start:size+=1;j=(j+1)%len(mask)
                extent=size*2*math.pi/len(mask)
                if 2*candidate_depth*math.sin(min(math.pi,extent)/2)>=minimum_width:
                    accepted.append(start)
            options.append((len(accepted),-candidate_depth,mask,accepted))
        _,negative_depth,free,starts=max(options,key=lambda x:x[:2])
        depth=-negative_depth
        if free.all():
            return []
        result=[]
        for start in starts:
            arc=[]; i=start
            while free[i]:
                arc.append(i); i=(i+1)%len(free)
                if i==start: break
            extent=len(arc)*2*math.pi/len(mean)
            width=2*depth*math.sin(min(math.pi,extent)/2)
            if width < minimum_width:
                continue
            weights=np.maximum(0.05,mean[arc]-depth)
            heading=math.atan2(float(np.sum(np.sin(self.angles[arc])*weights)),
                               float(np.sum(np.cos(self.angles[arc])*weights)))
            center=int(np.argmin(np.abs(np.arctan2(np.sin(self.angles-heading),
                                                    np.cos(self.angles-heading)))))
            if not np.isfinite(observed[center]):
                continue
            result.append(DirectionProposal(heading,float(extent),
                          float(min(mean[center],observed[center])),
                          float(variance[center]),len(arc)))
        return result


def entrance_geometry(grid, resolution, origin, source, heading, reach,
                      clearance, maximum_half_width=3.0):
    """Measure a branch cross-section in the unmodified SLAM occupancy grid.

    Unknown side boundaries produce an unknown width, not a fabricated wall.
    The segment to the section must be observed free; motion admission still
    requires the downstream inflated-map BFS and Navfn checks.
    """
    direction = np.array([math.cos(heading), math.sin(heading)])
    tangent = np.array([-direction[1], direction[0]])
    source = np.array(source, dtype=float)
    def value(xy):
        col, row = np.floor((xy-np.asarray(origin))/resolution).astype(int)
        if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]:
            return int(grid[row, col])
        return -1
    sections = []
    maximum = min(float(reach)-clearance, 4.0)
    if maximum < max(0.5, clearance):
        return None
    for distance in np.arange(resolution, maximum+resolution/2, resolution):
        center = source+direction*distance
        if value(center) != 0:
            break
        if distance < max(0.5, clearance):
            continue
        bounds = []
        for sign in (-1, 1):
            bound = None
            for offset in np.arange(resolution, maximum_half_width+resolution/2, resolution):
                point = center+sign*tangent*offset
                cell_value = value(point)
                if cell_value < 0:
                    break
                if cell_value >= 50:
                    bound = (float(offset-resolution/2), point.tolist())
                    break
            bounds.append(bound)
        width = None if any(x is None for x in bounds) else sum(x[0] for x in bounds)
        sections.append(dict(position=center.tolist(), direction=direction.tolist(),
                             width=width, bounds=[None if x is None else x[1] for x in bounds],
                             distance=float(distance),
                             passability='unknown' if width is None else
                             ('open' if width >= max(0.65, 1.5*clearance) else 'blocked')))
    measured = [x for x in sections if x['width'] is not None]
    if not measured:
        return sections[0] if sections else None
    # First minimum cross-section is a doorway throat when one exists and a
    # deterministic corridor-entry section otherwise. Do not require a neck.
    return min(measured, key=lambda x: (round(x['width']/resolution), x['distance']))


def map_ranges(grid, resolution, origin, source, angles, maximum=8.0):
    """Reproject local free rays at a durable node for SLAM association.

    Unknown is represented as NaN. This is map evidence for refining a GP
    observation, not an independent assertion that unobserved space is free.
    """
    depths=np.arange(resolution,maximum+resolution/2,resolution)
    angles=np.asarray(angles)
    x=source[0]+np.cos(angles[:,None])*depths[None,:]
    y=source[1]+np.sin(angles[:,None])*depths[None,:]
    cols=np.floor((x-origin[0])/resolution).astype(int)
    rows=np.floor((y-origin[1])/resolution).astype(int)
    inside=(rows>=0)&(rows<grid.shape[0])&(cols>=0)&(cols<grid.shape[1])
    cells=np.full(rows.shape,-1,dtype=np.int8)
    cells[inside]=grid[rows[inside],cols[inside]]
    result=np.full(len(angles),np.nan)
    for i in range(len(angles)):
        stops=np.flatnonzero(cells[i]!=0)
        if len(stops) and cells[i,stops[0]]>=50:
            result[i]=depths[stops[0]]
        elif not len(stops):
            result[i]=maximum
    return result


def junction_anchor(grid,resolution,origin,source,radius=2.0):
    """Refine a GP-discovered fork to its local maximal-clearance center.

    This is geometric association only: it cannot create a fork without a GP
    proposal. Unknown boundaries limit the estimate like unverified space.
    """
    center_col=int((source[0]-origin[0])/resolution)
    center_row=int((source[1]-origin[1])/resolution)
    span=int(math.ceil((radius+3.0)/resolution))
    r0=max(0,center_row-span);r1=min(grid.shape[0],center_row+span+1)
    c0=max(0,center_col-span);c1=min(grid.shape[1],center_col+span+1)
    crop=grid[r0:r1,c0:c1]
    obstacles=np.argwhere(crop!=0).astype(float)+np.array([r0,c0])
    if not len(obstacles):return tuple(source)
    stride=max(1,int(round(0.15/resolution)))
    choices=[]
    for row in range(max(0,center_row-int(radius/resolution)),
                     min(grid.shape[0],center_row+int(radius/resolution)+1),stride):
        for col in range(max(0,center_col-int(radius/resolution)),
                         min(grid.shape[1],center_col+int(radius/resolution)+1),stride):
            if grid[row,col]!=0:continue
            xy=(origin[0]+(col+.5)*resolution,origin[1]+(row+.5)*resolution)
            gap=math.dist(source,xy)
            if gap>radius:continue
            clearance=float(np.min(np.sum((obstacles-[row,col])**2,axis=1)))
            choices.append((clearance,-gap,xy))
    return max(choices,key=lambda x:x[:2])[2] if choices else tuple(source)
