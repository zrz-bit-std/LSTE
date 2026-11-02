"""Conservative odom raster of a Place domain from one immutable map epoch."""
import math
import numpy as np
from global_frontier_geometry import inverse_planar_projector


def physical_scope_raster(message, allowed, odom_to_map):
    """Return origin/resolution/cells; every allowed target cell is wholly owned.

    This changes only the representation of the same scope. Reprojection may
    exclude extra boundary cells; it never permits a source-excluded location.
    All calculations use the provided captured transform, never another TF.
    """
    info=message.info;resolution=float(info.resolution)
    if not math.isfinite(resolution) or resolution<=0:raise ValueError('invalid mask resolution')
    zero=np.asarray(odom_to_map(0.,0.));px=np.asarray(odom_to_map(1.,0.))-zero;py=np.asarray(odom_to_map(0.,1.))-zero
    matrix=np.column_stack((px,py))
    if not np.all(np.isfinite(matrix)) or not np.allclose(matrix.T@matrix,np.eye(2),atol=1e-6):
        raise ValueError('mask projection must be a captured planar rigid transform')
    inverse=inverse_planar_projector(odom_to_map)
    if inverse is None:raise ValueError('noninvertible mask projection')
    q=info.origin.orientation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    ca,sa=math.cos(angle),math.sin(angle);ox,oy=info.origin.position.x,info.origin.position.y
    corners=[inverse(ox+ca*u-sa*v,oy+sa*u+ca*v) for u,v in
             ((0.,0.),(info.width*resolution,0.),(0.,info.height*resolution),(info.width*resolution,info.height*resolution))]
    origin=np.floor(np.min(corners,axis=0)/resolution)*resolution-resolution
    width,height=(np.ceil((np.max(corners,axis=0)-origin)/resolution).astype(int)+1)
    if width<=0 or height<=0 or width*height>4000000:raise ValueError('invalid projected scope extent')
    data=np.full((height,width),-1,dtype=np.int8)
    if allowed is None:return (float(origin[0]),float(origin[1])),resolution,data
    allowed=np.asarray(allowed,dtype=bool)
    if allowed.shape!=(info.height,info.width):raise ValueError('scope shape disagrees with source map')
    rows,cols=np.indices((height,width));x=origin[0]+(cols+.5)*resolution;y=origin[1]+(rows+.5)*resolution
    minc=np.full(data.shape,np.inf);maxc=-minc.copy();minr=minc.copy();maxr=-minc.copy()
    for dx,dy in ((-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)):
        a=x+dx*resolution;b=y+dy*resolution
        mx=zero[0]+matrix[0,0]*a+matrix[0,1]*b-ox
        my=zero[1]+matrix[1,0]*a+matrix[1,1]*b-oy
        c=(ca*mx+sa*my)/resolution;r=(-sa*mx+ca*my)/resolution
        minc=np.minimum(minc,c);maxc=np.maximum(maxc,c);minr=np.minimum(minr,r);maxr=np.maximum(maxr,r)
    c0=np.floor(minc+1e-8).astype(int);c1=np.floor(maxc-1e-8).astype(int)
    r0=np.floor(minr+1e-8).astype(int);r1=np.floor(maxr-1e-8).astype(int)
    forbidden=(c0<0)|(r0<0)|(c1>=info.width)|(r1>=info.height)
    # A rigidly transformed same-resolution square overlaps at most 3x3 source
    # cells. Its axis-aligned bounding box is conservative around rotated cuts.
    for dr in range(3):
        for dc in range(3):
            rr=r0+dr;cc=c0+dc;used=(rr<=r1)&(cc<=c1)
            valid=used&(rr>=0)&(cc>=0)&(rr<info.height)&(cc<info.width)
            forbidden[valid] |= ~allowed[rr[valid],cc[valid]]
    data[forbidden]=100
    return (float(origin[0]),float(origin[1])),resolution,data
