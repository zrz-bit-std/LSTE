"""Place-domain constraints shared by native Navfn and TEB costmap layers."""
import copy
import hashlib
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


def constraint_cells(mask):
    """Unknown is transparent in the global max layer; only exclusion is lethal."""
    return np.where(np.asarray(mask,dtype=bool),-1,100).astype(np.int8)


def degenerate_scope_raster(local_data):
    """True when a projected scope excludes every cell.

    Such a raster carries no spatial claim a costmap could satisfy: the robot
    itself would sit inside the forbidden set.  Run 20261031_183336 published
    exactly one such frame (sim 1611.889, 100% of a 488x393 odom grid) and the
    native costmap audit rejected it, correctly.
    """
    data = np.asarray(local_data)
    return bool(data.size) and not np.any(data != 100)


def costmap_contains_constraint(message, scope, allowed, project=None):
    """Check actual master-grid costs, not a timer pretending a layer applied."""
    if message is None:return False,0,0
    info=message.info;origin=info.origin.position
    rows,cols=np.indices((info.height,info.width))
    x=origin.x+(cols+.5)*info.resolution;y=origin.y+(rows+.5)*info.resolution
    if project is not None:
        p0=project(0.,0.);px=project(1.,0.);py=project(0.,1.)
        if any(p is None for p in (p0,px,py)):return False,0,0
        x,y=p0[0]+x*(px[0]-p0[0])+y*(py[0]-p0[0]),p0[1]+x*(px[1]-p0[1])+y*(py[1]-p0[1])
    target=scope.info;ox,oy=target.origin.position.x,target.origin.position.y
    c=np.floor((x-ox)/target.resolution).astype(int);r=np.floor((y-oy)/target.resolution).astype(int)
    overlap=(r>=0)&(r<allowed.shape[0])&(c>=0)&(c<allowed.shape[1])
    rr,cc=np.where(overlap)
    excluded=~allowed[r[rr,cc],c[rr,cc]]
    values=np.asarray(message.data).reshape(info.height,info.width)[rr[excluded],cc[excluded]]
    count=int(values.size);missing=int(np.count_nonzero(values!=100))
    return bool(np.any(overlap) and missing==0),count,missing


class PlaceNavigationConstraints:
    """Publish one domain and wait for both real costmaps to contain it."""
    def __init__(self):
        self.publisher=rospy.Publisher('/lste/place_navigation_mask',OccupancyGrid,queue_size=1,latch=True)
        self.local_publisher=rospy.Publisher('/lste/place_navigation_mask_odom',OccupancyGrid,queue_size=1,latch=True)
        self.local_message=None;self.local_allowed=None
        self.local=None;self.message=None;self.allowed=None;self.key=None;self.sequence=0
        self.last_report={};self.bounded=False
        self.subscribers=[
            rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,self.on_local,queue_size=1),
            rospy.Subscriber('/move_base/local_costmap/costmap_updates',OccupancyGridUpdate,self.on_local_update,queue_size=5)]

    def on_local(self,message):self.local=message

    def on_local_update(self,update):
        old=self.local
        if old is None:return
        if (update.x+update.width>old.info.width or update.y+update.height>old.info.height
                or update.header.frame_id!=old.header.frame_id):return
        message=copy.copy(old);message.header=copy.copy(update.header)
        data=np.asarray(old.data,dtype=np.int8).reshape(old.info.height,old.info.width).copy()
        data[update.y:update.y+update.height,update.x:update.x+update.width]=np.asarray(update.data).reshape(update.height,update.width)
        message.data=data.ravel().tolist();self.local=message

    def bootstrap(self,message,node):
        if self.message is None:self.publish(message,None,node)

    def _same_effective_bounded_scope(self,key,physical_origin,resolution,local_data):
        """A newer SLAM stamp alone must not revoke an unchanged active WI lease."""
        if not self.bounded or self.key is None or self.message is None or self.local_message is None:
            return False
        # key[1] is the source map stamp; all other fields describe the global
        # raster and exact map->odom projection used to create both masks.
        if (key[:1]+key[2:]) != (self.key[:1]+self.key[2:]):
            return False
        old=self.local_message;info=old.info;origin=info.origin.position
        if (old.header.frame_id.lstrip("/")!="odom"
                or info.width!=local_data.shape[1] or info.height!=local_data.shape[0]
                or info.resolution!=resolution
                or origin.x!=physical_origin[0] or origin.y!=physical_origin[1]):
            return False
        old_data=np.asarray(old.data,dtype=np.int8).reshape(info.height,info.width)
        return bool(np.array_equal(old_data,local_data))

    def publish(self,message,allowed,node):
        from global_frontier_geometry import map_snapshot_projector
        from global_frontier_place_mask_projection import physical_scope_raster
        info=message.info;origin=info.origin.position
        bounded=allowed is not None
        project=None;basis=()
        if bounded:
            project=map_snapshot_projector(node,message)
            if project is None:return False
            basis=tuple(float(v) for point in ((0.,0.),(1.,0.),(0.,1.)) for v in project(*point))
        data=(constraint_cells(allowed) if bounded else np.full((info.height,info.width),-1,dtype=np.int8))
        key=(message.header.frame_id,message.header.stamp.to_sec(),info.resolution,origin.x,origin.y,info.width,info.height,
             bounded,hashlib.sha256(data.tobytes()).digest(),basis)
        if key==self.key:return True
        if bounded:
            physical_origin,resolution,local_data=physical_scope_raster(message,allowed,project)
        else:
            # Transparency contains no spatial claim and needs no TF. At
            # startup, publishing a neutral cell lets StaticLayer initialize;
            # waiting for TF here could block the planners needed to retry it.
            # Later, clear the whole previous physical canvas, not just 1 cell.
            old=self.local_message
            physical_origin=((0.,0.) if old is None else
                (old.info.origin.position.x,old.info.origin.position.y))
            resolution=float(info.resolution if old is None else old.info.resolution)
            shape=(1,1) if old is None else (old.info.height,old.info.width)
            local_data=np.full(shape,-1,dtype=np.int8)
        if bounded and degenerate_scope_raster(local_data):
            # Keep the previous valid scope for this epoch rather than publish
            # an unenforceable all-excluded mask.
            rospy.logwarn_throttle(
                5.0,
                "Place scope raster excludes every cell; keeping the previous "
                "valid scope (stamp=%.3f)", message.header.stamp.to_sec())
            return False
        unchanged_scope=self._same_effective_bounded_scope(key,physical_origin,resolution,local_data)
        if bounded and self.bounded and self.key is not None and not unchanged_scope:
            if getattr(node,'active_frontier',None) is not None and getattr(node,'active_route_id',0):
                from clock_provider import now_for
                suspend=getattr(node,'suspend_active_work_item',None)
                if callable(suspend):suspend(now_for(node),'place_scope_revision')
                node.release_active_frontier(discard_prefetch=True,queue_successor=False,
                    await_controller_release_ack=True,
                    preserve_place_departure=bool(getattr(getattr(node,'place_departure',None),'active',False)))
                self.last_report.update(ready=False,reason='scope_revision_controller_release_pending')
                return False
            if (getattr(node,'awaiting_controller_lease_release_ack',False)
                    or getattr(node,'pending_controller_lease_releases',None)):
                self.last_report.update(ready=False,reason='scope_revision_controller_release_pending')
                return False
        out=OccupancyGrid();out.header=copy.copy(message.header);out.info=copy.deepcopy(info)
        self.sequence+=1;out.header.seq=self.sequence;out.header.stamp=rospy.Time.now()
        # Record the original raster epoch separately from publication time.
        out.info.map_load_time=message.header.stamp
        out.data=data.ravel().tolist()
        local_out=OccupancyGrid();local_out.header=copy.copy(out.header);local_out.header.frame_id='odom'
        local_out.info=copy.deepcopy(out.info);local_out.info.resolution=resolution
        local_out.info.height,local_out.info.width=local_data.shape
        local_out.info.origin.position.x,local_out.info.origin.position.y=physical_origin
        local_out.info.origin.orientation.x=local_out.info.origin.orientation.y=local_out.info.origin.orientation.z=0.
        local_out.info.origin.orientation.w=1.;local_out.data=local_data.ravel().tolist()
        self.key=key;self.message=out;self.allowed=None if allowed is None else np.asarray(allowed,dtype=bool).copy()
        self.local_message=local_out;self.local_allowed=None if allowed is None else local_data!=100
        self.bounded=bounded
        self.publisher.publish(out);self.local_publisher.publish(local_out)
        self.last_report=dict(sequence=self.sequence,bounded=bounded,ready=False,
                              excluded_cells=int(np.count_nonzero(data==100)),map_stamp=message.header.stamp.to_sec(),
                              local_frame='odom',local_excluded_cells=int(np.count_nonzero(local_data==100)))
        return True

    def robot_in_scope(self,node,message,allowed):
        pose=getattr(node,'pose_odom',None)
        if pose is None:return False
        from global_frontier_geometry import map_snapshot_projector
        project=map_snapshot_projector(node,message)
        point=None if project is None else project(pose.x,pose.y)
        if point is None:return False
        origin=message.info.origin.position;resolution=message.info.resolution
        row=int(np.floor((point[1]-origin.y)/resolution));col=int(np.floor((point[0]-origin.x)/resolution))
        return bool(0<=row<allowed.shape[0] and 0<=col<allowed.shape[1] and allowed[row,col])

    def invalidate(self,node,message):
        if getattr(node,'active_route_kind','')!='portal_transition' and getattr(node,'active_route_id',0):
            from clock_provider import now_for
            suspend=getattr(node,'suspend_active_work_item',None)
            if callable(suspend):suspend(now_for(node),'place_navigation_scope_invalid')
            node.release_active_frontier(discard_prefetch=True,queue_successor=False,
                await_controller_release_ack=True,
                preserve_place_departure=bool(getattr(getattr(node,'place_departure',None),'active',False)))
        if (getattr(node,'awaiting_controller_lease_release_ack',False)
                or getattr(node,'pending_controller_lease_releases',None)):
            self.last_report.update(ready=False,reason='scope_revision_controller_release_pending')
            return
        self.publish(message,np.zeros((message.info.height,message.info.width),dtype=bool),node)
        self.last_report.update(ready=False,robot_inside=False,reason='place_navigation_scope_invalid')

    def update_scope(self,node):
        if self.bounded and not self.set_mode(node,True):
            if self.last_report.get('reason')=='scope_revision_controller_release_pending':return
            message=getattr(node,'current_place_scope_message',None)
            if message is not None:self.invalidate(node,message)

    def set_mode(self,node,bounded):
        auth=getattr(node,'active_portal_authentication',lambda:None)()
        if auth is not None:bounded=False
        if bounded != self.bounded and (
                getattr(node,'awaiting_controller_lease_release_ack',False)
                or getattr(node,'pending_controller_lease_releases',None)):
            self.last_report.update(ready=False,reason='controller_release_pending')
            return False
        if bounded:
            message=getattr(node,'current_place_scope_message',None)
            allowed=getattr(node,'current_place_scope_mask',None)
            if (message is None or allowed is None or not getattr(node,'current_place_scope_valid',False)
                    or getattr(node,'current_place_scope_id',None)!=getattr(node,'current_physical_place_id',None)):
                self.last_report.update(ready=False,robot_inside=False,reason='place_scope_pending')
                return False
            if not self.robot_in_scope(node,message,allowed):
                self.last_report.update(ready=False,robot_inside=False,reason='robot_not_inside_place_scope')
                return False
        else:message=getattr(node,'map_msg',None);allowed=None
        if message is None:return False
        if not self.publish(message,allowed,node):
            self.last_report.update(ready=False,reason='place_snapshot_transform_pending')
            return False
        if bounded:self.last_report.update(robot_inside=True)
        return True

    def for_plan(self,node,plan):
        if plan.action=='cross_portal':self.set_mode(node,False)
        elif plan.action in ('bootstrap_observation','observe_local_work','retry_viewpoint','reinspect_target'):
            self.set_mode(node,True)

    def prepare_candidate(self,node,candidate):
        plan=getattr(node,'last_graph_route_plan',None)
        bounded=(getattr(candidate,'route_kind','')!='portal_transition'
                 and getattr(plan,'action','')!='cross_portal')
        return self.set_mode(node,bounded) and self.ready(node)

    def ready(self,node):
        if self.message is None or self.local_message is None:return False
        grids=(getattr(node,'costmap_msg',None),self.local)
        scopes=(self.message,self.local_message)
        if any(m is None or m.header.stamp<scope.header.stamp for m,scope in zip(grids,scopes)):return False
        results=[]
        if self.bounded:
            for message,scope,allowed in zip(grids,scopes,(self.allowed,self.local_allowed)):
                # Each native layer now receives its own frame. No Time(0)
                # re-projection may move the Place boundary during readiness.
                if message.header.frame_id.lstrip('/')!=scope.header.frame_id.lstrip('/'):return False
                results.append(costmap_contains_constraint(message,scope,allowed))
            ready=all(result[0] for result in results)
        else:ready=True
        self.last_report.update(ready=ready,costmaps=[dict(excluded=n,unmarked=miss) for _,n,miss in results],
                                place_id=getattr(node,'current_physical_place_id',None))
        return ready
