"""GP branch discovery and tree adapter for the existing Navfn/TEB executor."""
import math
import numpy as np
import rospy
from gp_branch_discovery import PolarGaussianProcess, entrance_geometry, angle_delta, map_ranges, junction_anchor, has_local_fork, has_confirmed_three_way_fan
from gp_exploration_tree import ExplorationTree
from gp_observation_tree import JunctionObservationTree
from global_frontier_portal_authentication import DoorwayAuthentication
from global_frontier_execution_resolution import portal_authentication_retry_budget
from global_frontier_place_partition import branch_opening_width, branch_gate_xy


def advance_goal_along_portal_axis(point_xy, gate_xy, destination_xy, distance):
    dx = float(destination_xy[0]) - float(gate_xy[0])
    dy = float(destination_xy[1]) - float(gate_xy[1])
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return float(point_xy[0]), float(point_xy[1])
    return (float(point_xy[0]) + float(distance) * dx / length,
            float(point_xy[1]) + float(distance) * dy / length)


def reproject_locked_entry_goal(transform_xy, target_frame, goal_odom):
    """Project a stable odom-frame entry waypoint into the latest map frame."""
    if goal_odom is None:
        return None
    return transform_xy(target_frame, "odom", float(goal_odom[0]), float(goal_odom[1]))


def select_reachable_approach_goal(points, gate_xy, destination_xy, current_xy, minimum_advance=0.2, max_lateral=0.4):
    dx = float(destination_xy[0]) - float(gate_xy[0])
    dy = float(destination_xy[1]) - float(gate_xy[1])
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    ux, uy = dx / length, dy / length
    candidates = []
    for index, point in enumerate(points):
        depth = (float(point[0]) - float(gate_xy[0])) * ux + (float(point[1]) - float(gate_xy[1])) * uy
        advance = (float(point[0]) - float(current_xy[0])) * ux + (float(point[1]) - float(current_xy[1])) * uy
        if -0.25 <= depth <= length + 0.5 and advance >= float(minimum_advance):
            lateral = abs((float(point[0]) - float(gate_xy[0])) * -uy + (float(point[1]) - float(gate_xy[1])) * ux)
            candidates.append((depth, lateral, index))
    if not candidates:
        return None
    centered = [item for item in candidates if item[1] <= float(max_lateral)]
    # An off-axis room elsewhere in the map is not an approach to this gate.
    return None if not centered else max(centered, key=lambda item: (item[0], -item[1]))[2]


def select_reachable_source_side_return_goal(
        points, path_costs, gate_xy, destination_xy, source_xy,
        resolution=0.05, minimum_depth=0.10, current_xy=None,
        excluded_goals=(), minimum_goal_separation=None,
        blocked_approach_points=(), blocked_approach_radius=0.35,
        maximum_source_distance=None, gate_cleared=False,
        return_radius=0.75, goal_xy_tolerance=0.0):
    """Choose a reachable source-side cell, or a junction cell after gate return."""
    points = np.asarray(points, dtype=float)
    costs = np.asarray(path_costs, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        return None
    if costs.shape != (len(points),):
        return None
    gx, gy = float(gate_xy[0]), float(gate_xy[1])
    dx = float(destination_xy[0]) - gx
    dy = float(destination_xy[1]) - gy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    ux, uy = dx / length, dy / length
    depth = (points[:, 0] - gx) * ux + (points[:, 1] - gy) * uy
    margin = max(float(minimum_depth), float(resolution))
    reachable = np.isfinite(costs) & (costs >= 0.0)
    if gate_cleared:
        # The controller may stop anywhere within its endpoint tolerance.
        # Keep that whole disk inside the authentication return radius;
        # otherwise a reached fallback can strand the robot outside it.
        endpoint_radius = (float(return_radius) - max(0.0, float(goal_xy_tolerance))
                           - (0.5 * float(resolution) if goal_xy_tolerance else 0.0))
        if endpoint_radius <= 0.0:
            return None
        reachable &= np.sum((points - np.asarray(source_xy, dtype=float)) ** 2,
                            axis=1) <= endpoint_radius ** 2
    else:
        reachable &= depth <= -margin
    valid = np.flatnonzero(reachable)
    if len(valid) == 0:
        return None
    source = np.asarray(source_xy, dtype=float)
    euclidean = np.sum((points[valid] - source) ** 2, axis=1)
    if maximum_source_distance is not None:
        maximum_sq = max(0.0, float(maximum_source_distance)) ** 2
        near_source = euclidean <= maximum_sq
        valid = valid[near_source]
        euclidean = euclidean[near_source]
        if len(valid) == 0:
            return None
    if current_xy is not None and not gate_cleared:
        current = np.asarray(current_xy, dtype=float)
        if current.shape != (2,) or not np.all(np.isfinite(current)):
            return None
        current_depth = float(
            (current[0] - gx) * ux + (current[1] - gy) * uy
        )
        # Require progress through the Portal plane toward its source side.
        # Euclidean distance to the junction can increase during a valid
        # obstacle-avoidance step along that axis, so it must not suppress
        # every reachable return waypoint.
        # TEB accepts XY goals within about 0.5 m. Requiring only a half-cell
        # of Portal-axis progress can repeatedly select a fallback inside that
        # tolerance; the controller reports it reached without moving, while
        # the authentication is still outside the junction return radius.
        minimum_advance = max(0.65, 4.0 * float(resolution))
        advancing = depth[valid] <= current_depth - minimum_advance
        valid = valid[advancing]
        euclidean = euclidean[advancing]
        if len(valid) == 0:
            return None
    excluded = np.asarray(tuple(excluded_goals), dtype=float)
    if excluded.size:
        if excluded.ndim != 2 or excluded.shape[1] != 2:
            return None
        separation = (max(0.65, 4.0 * float(resolution))
                      if minimum_goal_separation is None
                      else max(0.0, float(minimum_goal_separation)))
        keep = np.ones(len(valid), dtype=bool)
        for point in excluded:
            keep &= np.sum((points[valid] - point) ** 2, axis=1) >= separation ** 2
        valid = valid[keep]
        euclidean = euclidean[keep]
        if len(valid) == 0:
            return None
    blocked = np.asarray(tuple(blocked_approach_points), dtype=float)
    if blocked.size:
        if blocked.ndim != 2 or blocked.shape[1] != 2:
            return None
        # These are failed controller approach samples, not map obstacles.
        # Navfn has already produced a collision-aware reachable set and can
        # route around them; reject only a goal at the failed sample itself.
        endpoints = points[valid]
        keep = np.ones(len(valid), dtype=bool)
        radius_sq = max(0.0, float(blocked_approach_radius)) ** 2
        for obstacle in blocked:
            keep &= np.sum((endpoints - obstacle) ** 2, axis=1) >= radius_sq
        valid = valid[keep]
        euclidean = euclidean[keep]
        if len(valid) == 0:
            return None
    # Choose the shortest reachable route toward the source side; use
    # junction proximity only as a tie-breaker so obstacle detours remain valid.
    score = (costs[valid] * float(resolution)
             + 0.10 * np.sqrt(euclidean))
    return int(valid[int(np.argmin(score))])



def select_reachable_return_sidestep_goal(
        points, path_costs, current_xy, gate_xy, destination_xy,
        resolution=0.05, excluded_goals=(), blocked_approach_points=(),
        blocked_approach_radius=0.35, minimum_lateral=0.55,
        minimum_range=0.55, maximum_range=1.25, source_xy=None, gate_cleared=False):
    """Choose a short reachable lateral step without advancing deeper through a Portal."""
    points = np.asarray(points, dtype=float)
    costs = np.asarray(path_costs, dtype=float)
    current = np.asarray(current_xy, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) == 0
            or costs.shape != (len(points),) or current.shape != (2,)
            or not np.all(np.isfinite(current))):
        return None
    gx, gy = float(gate_xy[0]), float(gate_xy[1])
    dx, dy = float(destination_xy[0])-gx, float(destination_xy[1])-gy
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    ux, uy = dx/length, dy/length
    delta = points-current
    distance = np.linalg.norm(delta, axis=1)
    depth = (points[:,0]-gx)*ux+(points[:,1]-gy)*uy
    current_depth = (current[0]-gx)*ux+(current[1]-gy)*uy
    lateral_delta = delta[:,0]*(-uy)+delta[:,1]*ux
    valid = (np.isfinite(costs) & (costs >= 0.0)
             & (distance >= max(float(minimum_range), 4.0*float(resolution)))
             & (distance <= float(maximum_range))
             & (np.abs(lateral_delta) >= float(minimum_lateral))
             & (depth <= current_depth + max(0.05, float(resolution)))
             & (depth >= current_depth - 1.50))
    if gate_cleared and source_xy is not None:
        source = np.asarray(source_xy, dtype=float)
        if source.shape != (2,) or not np.all(np.isfinite(source)):
            return None
        source_distance = np.linalg.norm(points-source, axis=1)
        # Once across the gate, the junction can lie toward that gate again.
        # Portal-axis retreat would walk away forever after overshooting it.
        valid = (np.isfinite(costs) & (costs >= 0.)
                 & (distance >= max(.65, 4.*float(resolution)))
                 & (distance <= float(maximum_range))
                 & (depth <= -max(.05, float(resolution)/2.))
                 & (source_distance <= np.linalg.norm(current-source)-.65))
    indices = np.flatnonzero(valid)
    if not len(indices):
        return None
    excluded = np.asarray(tuple(excluded_goals), dtype=float)
    if excluded.size:
        if excluded.ndim != 2 or excluded.shape[1] != 2:
            return None
        separation = max(0.65, 4.0*float(resolution))
        keep = np.ones(len(indices), dtype=bool)
        for point in excluded:
            keep &= np.linalg.norm(points[indices]-point, axis=1) >= separation
        indices = indices[keep]
        if not len(indices):
            return None
    blocked = np.asarray(tuple(blocked_approach_points), dtype=float)
    if blocked.size:
        if blocked.ndim != 2 or blocked.shape[1] != 2:
            return None
        vectors = points[indices]-current
        lengths_sq = np.sum(vectors*vectors, axis=1)
        keep = np.ones(len(indices), dtype=bool)
        radius_sq = max(0.0, float(blocked_approach_radius))**2
        for obstacle in blocked:
            t = np.zeros(len(indices), dtype=float)
            nonzero = lengths_sq > 1e-12
            t[nonzero] = np.clip(
                np.sum((obstacle-current)*vectors[nonzero], axis=1)
                / lengths_sq[nonzero], 0.0, 1.0)
            closest = current+vectors*t[:,None]
            keep &= np.sum((closest-obstacle)**2, axis=1) >= radius_sq
        indices = indices[keep]
        if not len(indices):
            return None
    score = (costs[indices]*float(resolution)
             + 0.20*np.maximum(0.0, distance[indices]-0.85)
             + 0.08/np.maximum(0.1, np.abs(lateral_delta[indices])))
    return int(indices[int(np.argmin(score))])


def select_reachable_interior_goal(points, gate_xy, destination_xy, target_xy, minimum_depth, max_lateral=1.0):
    dx = float(destination_xy[0]) - float(gate_xy[0])
    dy = float(destination_xy[1]) - float(gate_xy[1])
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    ux, uy = dx / length, dy / length
    candidates = []
    for index, point in enumerate(points):
        depth = (float(point[0]) - float(gate_xy[0])) * ux + (float(point[1]) - float(gate_xy[1])) * uy
        lateral = abs((float(point[0])-float(gate_xy[0])) * -uy
                      + (float(point[1])-float(gate_xy[1])) * ux)
        if float(minimum_depth) <= depth <= max(length, float(minimum_depth)) + 0.5 and lateral <= float(max_lateral):
            distance = (float(point[0]) - float(target_xy[0])) ** 2 + (float(point[1]) - float(target_xy[1])) ** 2
            candidates.append((distance, index))
    return None if not candidates else min(candidates)[1]


# Three withdrawals of an unchanged measured section is a decision, not a
# retry.  Run 20261103_191849 withdrew branch 11 58 times in one run.
PASSABLE_SECTION_WITHDRAWAL_LIMIT = 3


def withdrawn_candidate_retryable(branch, tolerance=0.05):
    """A candidate withdrawn for an unpassable section retries only on new geometry.

    Run 20261103_191849 branch 11 flipped passability open/blocked every 2.5 s
    while its physical_gate stayed within 0.02 m: the observation task re-added
    it 2 s later, withdrew it again, reset the 30 s watch and kept the route
    preempted for 180 s until the watchdog killed the run.
    """
    previous = branch.get("authentication_withdrawn_gate")
    current = branch.get("physical_gate")
    if not previous or not current:
        return True
    try:
        return math.dist([float(v) for v in previous],
                         [float(v) for v in current]) > float(tolerance)
    except (TypeError, ValueError):
        return True


def withdrawal_is_terminal(count, limit=3):
    """Repeated withdrawal of an unchanged section is a decision, not a retry."""
    try:
        return int(count) >= max(1, int(limit))
    except (TypeError, ValueError):
        return False


class GPBranchFrontierMixin:
    def portal_authentication_node_to_pin(self):
        active_auth = getattr(self, 'active_portal_authentication', lambda: None)()
        if active_auth is not None:
            return int(active_auth.node_id)
        task = getattr(self, 'junction_portal_authentication', None)
        if (task is not None
                and task.phase in ("watching", "authenticating")
                and not task.all_finished()):
            return int(task.node_id)
        return None

    def tree_observations_for_authentication(self, tree, observations):
        task = getattr(self, "junction_portal_authentication", None)
        if (task is not None and task.active_branch_id is not None
                and int(task.node_id) == int(tree.current_node_id)
                and task.phase in ("watching", "authenticating")):
            # The robot is traversing one already-recorded doorway. New scan
            # lobes from the room interior must not become sibling arms of
            # the source junction while this transaction owns navigation.
            return ()
        return observations

    def update_gp_branch_tree(self, message, map_context, robot_map, robot_yaw, now):
        if not getattr(self, 'gp_portal_assistance_enabled', False):
            return
        scan=getattr(self,'gp_branch_scan',None)
        if scan is None:
            return
        if not hasattr(self,'gp_exploration_tree'):
            self.gp_exploration_tree=ExplorationTree()
            self.gp_observation_tree=JunctionObservationTree()
            self.gp_direction_model=PolarGaussianProcess()
        tree=self.gp_exploration_tree
        active_auth = getattr(self, 'active_portal_authentication', lambda: None)()
        pinned_node_id = self.portal_authentication_node_to_pin()
        if pinned_node_id in tree.nodes:
            # The whole observe/authenticate transaction owns its source node,
            # including the quiet-settle window before the first branch starts.
            # Otherwise normal GP association can advance the cursor to the
            # next node during that window and strand every pending branch.
            tree.current_node_id = pinned_node_id
        stamp=scan.header.stamp.to_sec()
        if stamp==tree.last_stamp:
            return
        # Use the sensor acquisition transform, including its real mounting
        # offset; repeated current-pose projection would move old openings.
        try:
            odom_tf=self.tf_buffer.lookup_transform('odom',scan.header.frame_id,
                          scan.header.stamp,rospy.Duration(0.05)).transform
        except Exception as exc:
            rospy.logwarn_throttle(5,'GP branch acquisition transform unavailable: %s',str(exc))
            return
        def yaw(transform):
            q=transform.rotation
            return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        # The scan stays at its acquisition-time odom pose, while every
        # raster query and measured doorway use this map's immutable epoch.
        from global_frontier_geometry import map_snapshot_projector, inverse_planar_projector
        frame = message.header.frame_id or 'map'
        project = map_snapshot_projector(self, message)
        if project is None:
            project = self.planar_xy_projector(frame, 'odom', stamp=message.header.stamp)
        if project is None:
            return
        unproject = inverse_planar_projector(project)
        if unproject is None:
            return
        def snapshot_xy(target, source_frame, x, y):
            if target == frame and source_frame == 'odom':
                return project(x, y)
            if target == 'odom' and source_frame == frame:
                return unproject(x, y)
            raise ValueError('unexpected GP raster projection frames')
        physical=(odom_tf.translation.x,odom_tf.translation.y)
        source=project(*physical)
        odom_yaw=yaw(odom_tf)
        forward=project(physical[0]+math.cos(odom_yaw), physical[1]+math.sin(odom_yaw))
        map_yaw=math.atan2(forward[1]-source[1], forward[0]-source[0])
        snapshot_rotation=map_yaw-odom_yaw
        grid=np.asarray(message.data,dtype=np.int8).reshape(message.info.height,message.info.width)
        origin=(message.info.origin.position.x,message.info.origin.position.y)
        resolution=message.info.resolution
        proposals=self.gp_direction_model.propose(scan.ranges,scan.angle_min,
            scan.angle_increment,scan.range_min,scan.range_max,
            minimum_width=2*self.clearance)
        # Once a junction is associated, refine directions from its fixed
        # anchor in the accumulated map. A robot turning inside the junction
        # must not mint a new branch for the same corridor at each viewpoint.
        raw_proposals=tuple(proposals)
        sensor_map_yaw=map_yaw
        self.gp_last_dedup_audit = {
            "raw_gp_proposals": len(raw_proposals),
            "slam_refined_proposals": 0,
            "slam_supported_rejections": 0,
            "slam_direction_deduplications": 0,
            "sensor_only_candidates_retained": 0,
            "source": "online_slam_known_free_ray_and_junction_anchor",
        }
        def retain_sensor_proposals(refined, anchor):
            # SLAM may not yet cover the full sensor range. Refinement must
            # not erase a real GP proposal simply because map rays are unknown.
            from dataclasses import replace
            merged=list(refined)
            rejected = 0
            deduplicated = 0
            retained = 0
            for item in raw_proposals:
                heading=sensor_map_yaw+item.heading
                end=(source[0]+item.reach*math.cos(heading),
                     source[1]+item.reach*math.sin(heading))
                projected=math.atan2(end[1]-anchor[1],end[0]-anchor[0])
                unknown_support=False
                for distance in np.arange(resolution,math.dist(end,anchor),resolution):
                    point=(anchor[0]+distance*math.cos(projected),
                           anchor[1]+distance*math.sin(projected))
                    cell=self.xy_to_grid_cell(message,*point)
                    if cell is None or grid[cell]<0:
                        unknown_support=True
                        break
                    if grid[cell]>=50:
                        break
                # When the map already explains the entire ray, its stable
                # node projection owns deduplication. Only missing map support
                # can justify adding an unmatched live-sensor direction.
                if not unknown_support:
                    rejected += 1
                    continue
                if not any(abs(angle_delta(projected,p.heading))<=max(.4,item.angular_width/2)
                           for p in merged):
                    merged.append(replace(item,heading=projected,reach=math.dist(end,anchor)))
                    retained += 1
                else:
                    deduplicated += 1
            self.gp_last_dedup_audit = {
                "raw_gp_proposals": len(raw_proposals),
                "slam_refined_proposals": len(refined),
                "slam_supported_rejections": rejected,
                "slam_direction_deduplications": deduplicated,
                "sensor_only_candidates_retained": retained,
                "merged_candidate_count": len(merged),
                "source": "online_slam_known_free_ray_and_junction_anchor",
            }
            return merged
        observation_source=source
        observation_physical=physical
        if tree.current_node_id is not None:
            node=tree.nodes[tree.current_node_id]
            pinned_source = pinned_node_id == node['id']
            if ((node['id']==1 or node.get('junction_confirmed') or pinned_source) and
                (math.dist(physical,node['physical_xy'])<=tree.node_radius or active_auth is not None or pinned_source)):
                anchor=snapshot_xy(message.header.frame_id or 'map','odom',*node['physical_xy'])
                if anchor is not None:
                    rays=map_ranges(grid,resolution,origin,anchor,self.gp_direction_model.angles)
                    refined=self.gp_direction_model.propose(rays,-math.pi,
                        2*math.pi/len(rays),0.,8.,minimum_width=2*self.clearance)
                    refined=retain_sensor_proposals(refined,anchor)
                    if refined:
                        proposals=refined
                        map_yaw=0.0
                        # Map directions transform back to the physical frame.
                        odom_yaw=-snapshot_rotation
                        observation_source=anchor
                        observation_physical=tuple(node['physical_xy'])
        # Before a new fork's identity is frozen, move its sensor-view
        # hypothesis to the local map's actual junction center. This avoids
        # recording the robot's distant viewing position as the junction.
        headings=[p.heading for p in proposals]
        # Use the GP candidate fan to decide whether to re-project at a map
        # anchor. Durable branch creation below keeps the stricter tree
        # stability and association thresholds.
        local_fork=has_local_fork(headings)
        current_node=None if tree.current_node_id is None else tree.nodes[tree.current_node_id]
        if active_auth is None and pinned_node_id is None and local_fork and (current_node is None or not current_node.get('junction_confirmed') or
                          math.dist(physical,current_node['physical_xy'])>tree.node_radius):
            anchor=junction_anchor(grid,resolution,origin,source)
            physical_anchor=snapshot_xy('odom',message.header.frame_id or 'map',*anchor)
            if physical_anchor is not None:
                rays=map_ranges(grid,resolution,origin,anchor,self.gp_direction_model.angles)
                refined=self.gp_direction_model.propose(rays,-math.pi,2*math.pi/len(rays),0.,8.,minimum_width=2*self.clearance)
                refined=retain_sensor_proposals(refined,anchor)
                if refined:
                    proposals=refined
                    map_yaw=0.
                    odom_yaw=-snapshot_rotation
                    observation_source=anchor
                    observation_physical=physical_anchor
        observations=[]
        for proposal in proposals:
            geometry=entrance_geometry(grid,resolution,origin,observation_source,
                       map_yaw+proposal.heading,proposal.reach,self.clearance)
            gate=None
            if geometry is not None:
                from global_frontier_portal_certification_wall import structural_wall_opening_for_normal
                normal=(int(round(math.sin(map_yaw+proposal.heading))),
                        int(round(math.cos(map_yaw+proposal.heading))))
                cell=self.xy_to_grid_cell(message,*geometry['position'])
                support=getattr(map_context.components,'structural_occupied',None)
                confirmed=(cell is not None and support is not None and
                    abs(normal[0])+abs(normal[1])==1 and
                    structural_wall_opening_for_normal(support,cell,normal,
                        max(1,int(math.ceil((geometry.get('width') or 2.)/resolution))),
                        max(2,int(math.ceil(self.region_topology_clearance/resolution)))))
                geometry['wall_confirmed']=bool(confirmed)
                geometry['kind']='doorway' if confirmed else 'branch'
                gate=snapshot_xy('odom',message.header.frame_id or 'map',*geometry['position'])
                geometry['physical_position'] = gate
                geometry['physical_bounds'] = [
                    None if bound is None else snapshot_xy(
                        'odom', message.header.frame_id or 'map', *bound)
                    for bound in geometry.get('bounds', [])]
            observations.append(dict(heading=odom_yaw+proposal.heading,
                variance=proposal.variance,reach=proposal.reach,
                angular_width=proposal.angular_width,geometry=geometry,
                physical_gate=None if gate is None else list(gate)))
        def line_free(a,b):
            # Reproject durable odom anchors through this raster's frozen TF.
            first=snapshot_xy(message.header.frame_id or 'map','odom',*a)
            last=snapshot_xy(message.header.frame_id or 'map','odom',*b)
            if first is None or last is None:
                return False
            for fraction in np.linspace(0,1,max(2,int(math.dist(first,last)/resolution)+1)):
                x=first[0]+fraction*(last[0]-first[0]);y=first[1]+fraction*(last[1]-first[1])
                cell=self.xy_to_grid_cell(message,x,y)
                if cell is None or grid[cell]!=0:
                    return False
            return True
        def same_structural_component(first, second):
            labels = getattr(map_context.components, 'labels', None)
            if not isinstance(labels, np.ndarray) or labels.ndim != 2:
                return False

            def nearby_labels(point):
                mapped = snapshot_xy(
                    message.header.frame_id or 'map', 'odom',
                    float(point[0]), float(point[1]),
                )
                if mapped is None:
                    return set()
                cell = self.xy_to_grid_cell(message, *mapped)
                if cell is None:
                    return set()
                row, col = int(cell[0]), int(cell[1])
                radius = max(1, int(math.ceil(0.25 / resolution)))
                r0, r1 = max(0, row-radius), min(labels.shape[0], row+radius+1)
                c0, c1 = max(0, col-radius), min(labels.shape[1], col+radius+1)
                window = labels[r0:r1, c0:c1]
                return {int(value) for value in np.unique(window) if int(value) > 0}

            return bool(nearby_labels(first) & nearby_labels(second))

        self.gp_latest_observations = observations
        tree_observations = self.tree_observations_for_authentication(
            tree, observations,
        )
        tree.observe(
            observation_physical,observation_source,tree_observations,
            now,stamp,map_context.components.epoch,line_free,
            same_component=same_structural_component, pinned_node_id=pinned_node_id,
        )
        self._sync_portal_authentication(tree, message, map_context, now)
        self._update_portal_authentication_from_map(
            tree, message, map_context, proposals, map_yaw, now)
        active=tree.active_branch_id
        if active is not None and tree.branches[active]['state']=='active':
            tree.arrived(active,(self.pose_odom.x,self.pose_odom.y),self.clearance)
        if active is not None:
            branch=tree.branches[active]
            if branch['state']=='entered' and tree.current_node_id==branch['node_id']:
                # A measured wall ahead plus only reverse GP directions is
                # evidence of an exhausted branch, not a new junction behind.
                real_headings=[yaw(odom_tf)+p.heading for p in
                    self.gp_direction_model.propose(scan.ranges,scan.angle_min,
                    scan.angle_increment,scan.range_min,scan.range_max,
                    minimum_width=2*self.clearance)]
                if real_headings and all(abs(angle_delta(h,branch['heading']))>math.pi/2
                                         for h in real_headings):
                    branch['state']='explored'
                    space=tree.spaces.get(branch.get('destination_space_id'))
                    if space is not None:space['state']='explored'
        self.publish_status('gp_branch_tree',tree=tree.snapshot(),
            gp_proposals=len(proposals),sensor_stamp=stamp,
            geometry_source='pointcloud_scan_and_online_slam',
            dedup_audit=dict(getattr(self, 'gp_last_dedup_audit', {})),
            observation_tree=(
                None if not hasattr(self, "gp_observation_tree")
                else self.gp_observation_tree.snapshot()
            ),
            portal_authentication=(
                None if getattr(self, "junction_portal_authentication", None) is None
                else self.junction_portal_authentication.snapshot()
            ))

    def _expire_stranded_portal_authentication(self, tree, message, now):
        """Conclude a pinned observation period that can never start a branch.

        ``expire_unstartable`` is the bounded conclusion for a junction whose
        remaining branches cannot be entered.  It used to be reached only from
        the in-scope discovery path, so a task pinned by
        ``portal_authentication_node_to_pin`` survived every scope check and
        held ``tree.current_node_id`` indefinitely.
        """
        task = getattr(self, "junction_portal_authentication", None)
        if task is None or task.active_branch_id is not None:
            return []
        if task.all_finished():
            return []
        get_param = getattr(
            rospy, "get_param", lambda _name, default=None: default,
        )
        timeout = float(get_param(
            "/lste_global_frontier/portal_observation_start_timeout", 30.0))
        expired = task.expire_unstartable(now, timeout)
        if not expired:
            return []
        if tree is not None:
            for expired_id in expired:
                tree_branch = tree.branches.get(expired_id)
                if tree_branch is not None:
                    tree_branch["authentication_state"] = "failed"
                    tree_branch["authentication_failure_reason"] = (
                        "observation_geometry_unavailable")
        self.publish_status(
            "portal_authentication_observation_expired",
            node_id=int(task.node_id), branch_ids=list(expired),
            timeout=timeout, phase=task.phase,
            reason="pinned_observation_unstartable",
        )
        return expired

    def _sync_portal_authentication(self, tree, message, map_context, now):
        """Maintain the stable branch set for a node's observation period."""
        # A certified crossing already owns the physical edge. A second GP
        # node for the same junction must not abort it at the gate. Discovery
        # continues; new observation tasks wait for commit into the next Place.
        transaction = getattr(self, "portal_transaction", None)
        if transaction is not None and transaction.active:
            return
        # A pinned task whose branches can never start must not hold the tree
        # cursor, and therefore the whole graph, forever.  The scope guards
        # below deliberately skip discovery for a node the active Place no
        # longer owns; before this check they also skipped the bounded expiry,
        # so a task pinned in ``watching`` blocked every dispatch for the rest
        # of the run (sealed 20261101_002732: 180 s with two pending branches
        # and ``active_branch_id is None``).
        self._expire_stranded_portal_authentication(tree, message, now)
        ledger = getattr(self, "place_work_items", None)
        closed = getattr(ledger, "place_is_closed", None)
        current = getattr(self, "current_physical_place_id", None)
        if current is not None and callable(closed) and closed(current):
            # Local completion already committed the original Portal return.
            # Discovery may continue, but never restart observation in this
            # completed space while its exit route is being materialized.
            return
        node_id = tree.current_node_id
        node = tree.nodes.get(node_id) if node_id is not None else None
        active_room = getattr(self, "active_semantic_room", None)
        current_place_id = getattr(self, "current_physical_place_id", None)
        if current_place_id is None:
            current_place_id = getattr(self, "active_frontier_region_id", None)
        # A nested junction is eligible inside the active Place. The parent
        # junction remains outside this Place's certified SLAM gate cuts.
        if active_room is not None:
            if node_id == active_room.get("node_id"):
                return
            xy = None if node is None else node.get("physical_xy")
            if message is None or xy is None:
                return
            # A watching task can pause ordinary planning, which normally
            # refreshes this cache. Rebuild its current-map source scope here
            # before testing ownership; requiring an already-fresh cache
            # would leave both planning and the pending task waiting forever.
            from global_frontier_place_portal_validation import _validation_snapshot_projector
            project = _validation_snapshot_projector(self, message, map_context, xy)
            mask = getattr(self, "current_place_scope_mask", None)
            if (project is None or mask is None
                    or getattr(self, "current_place_scope_id", None) != current_place_id):
                return
            if xy is not None:
                mapped = project(*xy)
                if mapped is None:
                    return
                col = int(math.floor((mapped[0]-message.info.origin.position.x)/message.info.resolution))
                row = int(math.floor((mapped[1]-message.info.origin.position.y)/message.info.resolution))
                if not (0 <= row < mask.shape[0] and 0 <= col < mask.shape[1] and mask[row,col]):
                    return
        if node is None:
            return
        existing = getattr(self, "portal_authentication_by_node", {}).get(int(node_id))
        if existing is not None:
            for branch_id in tuple(existing.branches):
                branch = tree.branches.get(branch_id) or {}
                if (branch.get("passability") == "blocked"
                        and existing.withdraw_pending(branch_id, now=now)):
                    count = int(branch.get("passable_section_withdrawals", 0) or 0) + 1
                    branch["passable_section_withdrawals"] = count
                    gate = branch.get("physical_gate")
                    branch["authentication_withdrawn_gate"] = (
                        None if not gate else [float(v) for v in gate])
                    if withdrawal_is_terminal(count, PASSABLE_SECTION_WITHDRAWAL_LIMIT):
                        # The same measured section keeps flipping because it
                        # sits on the passability threshold; its gate does not
                        # move, so there is no new evidence to observe.  Conclude
                        # the candidate instead of re-adding it forever.
                        branch["state"] = "authentication_failed"
                        branch["authentication_state"] = "failed"
                        branch["authentication_failure_reason"] = (
                            "measured_section_not_passable")
                        self.publish_status(
                            "portal_authentication_pending_withdrawn",
                            node_id=int(node_id), branch_id=int(branch_id),
                            reason="measured_section_not_passable",
                            withdrawals=count, terminal=True)
                    else:
                        branch["authentication_state"] = (
                            "waiting_passable_section")
                        self.publish_status(
                            "portal_authentication_pending_withdrawn",
                            node_id=int(node_id), branch_id=int(branch_id),
                            reason="measured_section_not_passable",
                            withdrawals=count)
        if not node.get("junction_confirmed"):
            # GP can see the three-way candidate fan slightly before its
            # angular clusters satisfy the stricter durable-node rule (for
            # example, a T side-room opening viewed obliquely from its stem).
            # On non-root nodes, a stable three-branch fan is enough to start
            # the reversible observe/authenticate cycle. Physical full-body
            # crossing plus inside frontier still decides whether each Portal
            # is real, so this does not register a Place by geometry alone.
            stable_branches = [
                tree.branches[branch_id]
                for branch_id in node.get("branch_ids", ())
                if (tree.branches.get(branch_id) is not None
                    and tree.branches[branch_id].get("state") not in (
                        "superseded", "authentication_failed", "authenticated")
                    and tree.branches[branch_id].get("stable")
                    and tree.branches[branch_id].get("passability") in ("open", "unknown")
                    and tree.branches[branch_id].get("connection") != "return")
            ]
            stable_fan = len(stable_branches)
            stable_fan_confirmed = has_confirmed_three_way_fan(
                branch.get("heading") for branch in stable_branches
            )
            if (int(node_id) == 1 or not node.get("junction")
                    or stable_fan < 3 or not stable_fan_confirmed):
                return
        active_task = getattr(self, "junction_portal_authentication", None)
        if (active_task is not None and active_task.active_branch_id is not None
                and int(active_task.node_id) != int(node_id)):
            return
        task = getattr(self, "portal_authentication_by_node", {}).get(int(node_id))
        if task is None:
            from global_frontier_portal_authentication import JunctionDoorwayAuthentication
            task = JunctionDoorwayAuthentication(node_id, settle_seconds=1.0)
            self.portal_authentication_by_node[int(node_id)] = task
        self.junction_portal_authentication = task
        # Different GP node IDs can describe the same physical junction.
        # Reuse only a certified Portal proven by the current SLAM gate cuts;
        # a common heading alone never merges neighboring doorways.
        # TEB's XY goal tolerance can be wider than body-clearance depth.
        # Aim one tolerance plus a margin beyond the robot radius so the
        # controller cannot stop while the base still straddles the gate.
        # Keep evidence tied to actual full-body clearance.
        get_param = getattr(
            rospy, "get_param", lambda _name, default=None: default,
        )
        body_radius = float(get_param(
            "/move_base/global_costmap/robot_radius", 0.30))
        terminal_radius = float(getattr(
            self, "endpoint_terminal_wait_radius", 0.45))
        # Do not begin the observation turn at the bare body-clearance
        # boundary: endpoint tolerance could leave too little known-free
        # space to materialize the later Portal-entry WorkItem.
        required_depth = max(0.35, body_radius + terminal_radius)
        goal_depth = required_depth + terminal_radius + 0.10
        entry_margin = max(0.55, terminal_radius + 0.10)
        from global_frontier_place_portal_validation import duplicate_observation_portal
        for branch_id in node.get("branch_ids", ()):
            branch = tree.branches.get(branch_id)
            if (branch is None or branch.get("state") in (
                    "superseded", "authenticated", "authentication_failed")
                    or (branch.get("connection") == "return"
                        and branch.get("authentication_state") == "deduplicated")):
                continue
            duplicate = duplicate_observation_portal(self, tree, branch, message, map_context, goal_depth=goal_depth)
            if duplicate is not None:
                branch["state"] = ("return" if branch.get("connection") == "return"
                                   else "superseded")
                branch["canonical_portal_id"] = int(duplicate["id"])
                branch["authentication_state"] = "deduplicated"
                self.publish_status("gp_portal_branch_deduplicated", node_id=int(node_id),
                    branch_id=int(branch_id), portal_id=int(duplicate["id"]),
                    canonical_branch_id=duplicate.get("gp_authentication_branch_id"),
                    evidence=("known_parent_component_across_bound_entry" if branch.get("connection") == "return"
                              else "same_destination_component_beyond_certified_slam_gate"))
        observation_tree = getattr(self, "gp_observation_tree", None)
        if observation_tree is not None:
            incoming = tree.branches.get(node.get("parent_branch_id"))
            reference = 0.0 if incoming is None else float(incoming.get("heading", 0.0)) + math.pi
            for branch_id in node.get("branch_ids", ()):
                branch_record = tree.branches.get(branch_id)
                if branch_record is None or branch_record.get("connection") == "return":
                    continue
                if branch_record.get("state") in ("superseded", "authentication_failed"):
                    room = observation_tree.prune_branch(
                        node_id, branch_id,
                        invalidated=branch_record.get("state") == "authentication_failed",
                    )
                    if room is not None:
                        branch_record["room_observation"] = {
                            key: value for key, value in room.items() if key != "frames"
                        }
                    continue
                relative = angle_delta(float(branch_record.get("heading", 0.0)), reference)
                room = observation_tree.begin(node_id, branch_id, relative)
                branch_record["room_observation"] = {key: value for key, value in room.items() if key != "frames"}
        branches = []
        for branch_id in node.get("branch_ids", ()):
            branch = tree.branches.get(branch_id)
            if (branch is None or branch.get("state") in (
                    "superseded", "authentication_failed", "authenticated")
                    or branch.get("connection") == "return"):
                continue
            if (not branch.get("stable")
                    or branch.get("passability") not in ("open", "unknown")):
                continue
            if (branch.get("authentication_state") == "waiting_passable_section"
                    and not withdrawn_candidate_retryable(branch)):
                continue
            from global_frontier_place_partition import measured_authentication_gate_geometry
            measured_geometry = measured_authentication_gate_geometry(branch)
            if measured_geometry is None:
                if branch.get("authentication_geometry_wait") != "measured_slam_section":
                    branch["authentication_geometry_wait"] = "measured_slam_section"
                    self.publish_status("portal_authentication_waiting_measured_section",
                        node_id=int(node_id), branch_id=int(branch_id))
                continue
            gate = branch_gate_xy(branch)
            if gate is None or branch.get("authentication_state") in (
                    "completed", "failed"):
                continue
            heading = float(branch.get("heading", 0.0))
            destination = (float(gate[0]) + goal_depth * math.cos(heading),
                           float(gate[1]) + goal_depth * math.sin(heading))
            auth = DoorwayAuthentication(
                branch_id, node_id, gate, destination,
                required_depth=required_depth,
                entry_goal_margin=entry_margin,
                gate_geometry=measured_geometry,
            )
            source_depth = auth.signed_depth(node.get("physical_xy"))
            if source_depth is None or not math.isfinite(source_depth) or source_depth >= -0.05:
                # A shifted section behind its own source cannot define an
                # entering/returning experiment. Keep discovery for remeasurement;
                # only an unstarted task may be withdrawn, never an active return.
                task.withdraw_pending(branch_id, now=now)
                if branch.get("authentication_geometry_wait") != "source_side_section":
                    branch["authentication_geometry_wait"] = "source_side_section"
                    self.publish_status("portal_authentication_waiting_source_side_geometry",
                        node_id=int(node_id), branch_id=int(branch_id),
                        source_xy=node.get("physical_xy"), gate_xy=list(auth.gate_xy),
                        destination_xy=list(auth.destination_xy), source_depth=source_depth)
                continue
            branch.pop("authentication_geometry_wait", None)
            pending = task.branches.get(int(branch_id))
            if pending is not None and pending.refresh_pending_geometry(
                    auth.gate_xy, auth.destination_xy, *auth.gate_geometry):
                branch["physical_gate"] = list(pending.gate_xy)
            if task.add(auth, now=now):
                branch["physical_gate"] = list(gate)
                branch["authentication_state"] = "pending"
                branch["authentication_failure_reason"] = None
                branches.append(auth)
        # A junction whose branches can never be entered must not hold the
        # ordinary route retired forever.  Bound the "watching" wait, fail the
        # unstartable branches, and let the graph dispatch again.
        observation_timeout = float(get_param(
            "/lste_global_frontier/portal_observation_start_timeout", 30.0))
        expired_branches = task.expire_unstartable(now, observation_timeout)
        if expired_branches:
            for expired_id in expired_branches:
                tree_branch = tree.branches.get(expired_id)
                if tree_branch is not None:
                    tree_branch["authentication_state"] = "failed"
                    tree_branch["authentication_failure_reason"] = (
                        "observation_geometry_unavailable")
            self.publish_status(
                "portal_authentication_observation_expired",
                node_id=int(node_id), branch_ids=expired_branches,
                timeout=observation_timeout, phase=task.phase,
            )
        # GP owns the whole observation/authentication phase. Retire any
        # ordinary local/probe route before dispatching its first branch goal;
        # otherwise a durable probe lease can steal a return-to-junction leg.
        if task.branches and task.phase in ("watching", "authenticating"):
            constraints = getattr(self, "place_navigation_constraints", None)
            auth_route_active = str(getattr(
                self, "active_portal_authentication_phase", "") or ""
            ) in ("entering", "turning", "returning")
            if not auth_route_active:
                pause_branch_id = task.active_branch_id
                if pause_branch_id is None:
                    pause_branch_id = min(task.branches)
                frozen_work_item = getattr(self, "active_work_item_id", None) is not None
                if frozen_work_item:
                    self.suspend_active_work_item(
                        now, "portal_junction_observation_period",
                    )
                settle_probe = getattr(self, "settle_active_portal_probe", None)
                if (getattr(self, "active_portal_probe_id", None) is not None
                        and callable(settle_probe)):
                    settle_probe(
                        "deferred", now, "portal_junction_observation_period",
                    )
                if getattr(self, "active_frontier", None) is not None:
                    self.release_active_frontier(
                        discard_prefetch=True,
                        queue_successor=False,
                        await_controller_release_ack=True,
                    )
                clear_lease = getattr(self, "_clear_graph_route_plan_lease", None)
                if callable(clear_lease):
                    clear_lease("portal_junction_observation_period")
                self.last_graph_route_plan = None
                self.last_graph_route_plan_signature = None
                self.graph_route_action_transaction = None
                for field in ("graph_route_portal_id", "graph_route_probe_id",
                              "graph_route_work_item_id", "graph_route_work_item_filter"):
                    if hasattr(self, field):
                        setattr(self, field, None)
                self.publish_status(
                    "portal_authentication_preempted_graph_route",
                    node_id=int(node_id), branch_id=int(pause_branch_id),
                    frozen_work_item=bool(frozen_work_item),
                    frozen_portal_probe=True,
                    resume_after="all_portals_returned_to_junction",
                )
            # The old ordinary lease still owns Navfn/TEB until its exact ACK.
            # Keep the bounded mask and do not start the physical auth clock yet.
            if (getattr(self, "awaiting_controller_lease_release_ack", False)
                    or getattr(self, "pending_controller_lease_releases", None)):
                if not getattr(task, "controller_release_wait_reported", False):
                    task.controller_release_wait_reported = True
                    self.publish_status("portal_authentication_waiting_controller_release",
                        node_id=int(node_id), active_branch_id=task.active_branch_id)
                return
            task.controller_release_wait_reported = False
            if constraints is not None:
                constraints.set_mode(self, False)
        # start_next enforces a quiet observation window after the last new
        # stable branch. T and cross junctions therefore wait for all arms.
        if task.active_branch_id is None:
            source_xy = node.get("physical_xy")
            next_branch = task.start_next(source_xy, now=now)
            if next_branch is not None:
                tree_branch = tree.branches.get(next_branch.branch_id)
                if tree_branch is not None:
                    tree_branch["authentication_state"] = "entering"
                self.publish_status(
                    "portal_authentication_branch_started",
                    node_id=int(node_id), branch_id=int(next_branch.branch_id),
                    state=next_branch.state, route_goal=next_branch.route_goal(),
                )
        report_signature = (
            task.phase, tuple(sorted(task.branches)), task.active_branch_id,
            task.all_finished(),
        )
        if getattr(task, "last_report_signature", None) != report_signature:
            task.last_report_signature = report_signature
            self.publish_status(
                "portal_authentication_observation_period",
                node_id=int(node_id), phase=task.phase,
                branch_ids=sorted(task.branches),
                active_branch_id=task.active_branch_id,
                all_finished=task.all_finished(),
            )

    def complete_portal_turn_from_supervisor(self, payload):
        """Accept TEB's completed atomic turn only for this auth route."""
        auth = self.active_portal_authentication()
        if auth is None or auth.state != "turning":
            return False
        if str(payload.get("active_route_kind", payload.get("route_kind", ""))) != "frontier_turn_connector":
            return False
        try:
            event_route_id = int(payload.get("route_id", 0) or 0)
            active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        except (TypeError, ValueError):
            return False
        if event_route_id and active_route_id and event_route_id != active_route_id:
            return False
        # Pose callbacks are coalesced through the lifecycle queue. The
        # matching supervisor event contains the odom pose at completion;
        # reading the queue's older/newer cache here can lose a valid turn.
        event_pose = payload.get("pose")
        pose = getattr(self, "pose_odom", None)
        try:
            final_yaw = (float(event_pose[2]) if event_pose is not None
                         else float(pose.theta))
        except (TypeError, ValueError, IndexError, AttributeError):
            return False
        if not math.isfinite(final_yaw):
            return False
        if not auth.update_rotation(final_yaw, stable_samples=1):
            return False
        if not auth.finish_turn(final_yaw):
            return False
        self.active_portal_authentication_phase = "returning"
        self.publish_status(
            "portal_authentication_turn_complete",
            node_id=int(auth.node_id), branch_id=int(auth.branch_id),
            turn_direction=auth.turn_direction,
            target_yaw=auth.target_yaw, final_yaw=final_yaw,
            supervisor_acknowledged=True,
        )
        self.release_active_frontier(
            discard_prefetch=True,
            preserve_place_departure=bool(
                getattr(getattr(self, "place_departure", None), "active", False)
            ),
        )
        return True

    def sync_portal_authentication_phase(self, auth):
        """Keep the route resolver phase aligned with the auth state machine."""
        state = str(getattr(auth, "state", "") or "").strip().lower()
        phase = state if state in ("entering", "turning", "returning") else ""
        self.active_portal_authentication_phase = phase
        return phase

    def portal_authentication_physical_budget_due(self, now):
        """Let expensive proposal loops yield without mutating authentication."""
        auth = self.active_portal_authentication()
        pose = getattr(self, "pose_odom", None)
        due = getattr(auth, "physical_progress_due", None)
        return bool(pose is not None and callable(due) and due(
            (pose.x, pose.y), pose.theta, now,
            portal_authentication_retry_budget(getattr(self, "active_timeout", 20.0))))

    def reopen_late_stable_candidates(self, now):
        """Route back to a junction whose doorway matured after it closed.

        Branch stability is time based (``observations >= 2`` and
        ``now - first_seen >= stable_seconds``).  Run 20261102_205142 finished
        with branch 40 on node 11 still ``pending`` although it was stable and
        open: it matured after that junction's observation period had closed, so
        no probe existed and the completion gate saw no outstanding obligation.
        The audit is right to require it -- the doorway was never attempted.

        Re-open such a junction (bounded) by asking the graph planner to resume
        it; the normal arrival path then starts the observation period again.
        """
        tree = getattr(self, "gp_exploration_tree", None)
        if tree is None:
            return False
        limit = int(getattr(self, "late_candidate_reopen_limit", 2) or 0)
        if limit <= 0:
            return False
        reopened = getattr(self, "_late_candidate_reopened", None)
        if reopened is None:
            reopened = set()
            self._late_candidate_reopened = reopened
        if len(reopened) >= limit:
            return False
        task = getattr(self, "junction_portal_authentication", None)
        observing = 0
        if task is not None and not task.all_finished():
            observing = int(getattr(task, "node_id", 0) or 0)
        grace = float(getattr(self, "late_candidate_stable_grace", 20.0) or 0.0)
        for branch_id, branch in sorted(tree.branches.items()):
            if not isinstance(branch, dict):
                continue
            if not branch.get("stable") or branch.get("passability") != "open":
                continue
            if branch.get("physical_gate") is None:
                continue
            if branch.get("connection") == "return":
                continue
            if branch.get("state") in ("return", "superseded"):
                continue
            if str(branch.get("authentication_state") or "") != "pending":
                continue
            try:
                node_id = int(branch.get("node_id", 0) or 0)
            except (TypeError, ValueError):
                continue
            node = tree.nodes.get(node_id)
            if node is None or not node.get("junction_confirmed"):
                continue
            if node_id in reopened or node_id == observing:
                continue
            first_seen = float(branch.get("first_seen", now) or now)
            if now - first_seen < grace:
                continue
            reopened.add(node_id)
            self.pending_graph_resume_place_id = node_id
            self.publish_status(
                "late_candidate_junction_reopened",
                node_id=node_id,
                branch_id=int(branch_id),
                stable_seconds=round(float(now) - first_seen, 1),
                reopen_count=len(reopened),
            )
            rospy.logwarn(
                "Reopening junction %d: doorway branch %d stayed stable and "
                "pending after its observation period closed",
                node_id, int(branch_id),
            )
            return True
        return False

    def check_portal_authentication_physical_progress(self, now):
        """Check the same physical budget even when no map plan is rebuilt."""
        # A pinned observation period with no active branch must be concluded
        # here, not only from the discovery callback: once the active Place
        # stops owning that node the callback returns early, and the pinned
        # task then holds tree.current_node_id forever (run 20261101_232628
        # stalled 180 s in exactly this state).
        if self.reopen_late_stable_candidates(now):
            return True
        task = getattr(self, "junction_portal_authentication", None)
        if (task is not None and task.active_branch_id is None
                and not task.all_finished()):
            expired = self._expire_stranded_portal_authentication(
                getattr(self, "gp_exploration_tree", None),
                getattr(self, "map_msg", None), now)
            if expired:
                return True
        auth = self.active_portal_authentication()
        pose = getattr(self, "pose_odom", None)
        if auth is None or pose is None:
            return False
        self.sync_portal_authentication_phase(auth)
        if auth.state == "returning" and getattr(auth, "return_recovery_final_blocked", False):
            # A final recovery failure is an explicit failed execution, never
            # a settled doorway or permission to dispatch the next branch.
            # Let a real last-moment return win before reporting it.
            auth.update_return((pose.x, pose.y))
            if auth.state == "returning":
                if not getattr(auth, "return_unrecoverable_reported", False):
                    auth.return_unrecoverable_reported = True
                    self.publish_status(
                        "portal_authentication_return_unrecoverable",
                        node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                        execution_failed=True, return_obligation_pending=True,
                        reason="source_side_recovery_budget_exhausted",
                        snapshot=auth.snapshot(),
                    )
                    # Close the branch as failed and advance the observation
                    # period.  The obligation stays explicit (the ledger entry
                    # and the tree branch both keep ``return_obligation_pending``),
                    # but it no longer freezes a mission that has other
                    # doorways to authenticate.
                    fail = getattr(auth, "fail", None)
                    if callable(fail):
                        fail("return_unrecoverable")
                    tree = getattr(self, "gp_exploration_tree", None)
                    if tree is not None:
                        tree_branch = tree.branches.get(int(auth.branch_id))
                        if tree_branch is not None:
                            tree_branch["authentication_state"] = "failed"
                            tree_branch["authentication_failure_reason"] = (
                                "return_unrecoverable")
                            tree_branch["return_obligation_pending"] = True
                    self.release_active_frontier(
                        discard_prefetch=True,
                        preserve_place_departure=bool(getattr(
                            getattr(self, "place_departure", None), "active", False)),
                    )
                    finish = getattr(
                        self, "finish_portal_authentication_branch", None)
                    if callable(finish):
                        finish(now=now)
                return True
        if auth.physical_progress_expired(
                (pose.x, pose.y), pose.theta, now,
                portal_authentication_retry_budget(getattr(self, "active_timeout", 20.0))):
            phase = auth.state
            reason = "authentication_physical_stall:%s" % phase
            if phase == "returning":
                # Keep the branch active until update_return proves that the
                # robot crossed the gate and returned to the source node. The
                # authentication object rearms its bounded progress timer.
                pass
            else:
                auth.request_failure_return(reason, pose_xy=(pose.x, pose.y))
            self.sync_portal_authentication_phase(auth)
            self.publish_status(
                "portal_authentication_physical_progress_expired",
                node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                phase=phase, reason=reason, state=auth.state,
                snapshot=auth.snapshot(),
            )
            self.release_active_frontier(
                discard_prefetch=True, queue_successor=False,
                await_controller_release_ack=True,
                preserve_place_departure=bool(getattr(
                    getattr(self, "place_departure", None), "active", False)),
            )
            return True
        return False

    def _update_portal_authentication_from_map(
        self, tree, message, map_context, frontier_proposals, map_yaw, now,
    ):
        """Use live SLAM frontier cells and odometry to advance one probe."""
        self.portal_authentication_map_context = map_context
        self.portal_authentication_map_message = message
        auth = self.active_portal_authentication()
        if auth is None or getattr(self, "pose_odom", None) is None:
            return
        self.sync_portal_authentication_phase(auth)
        pose = self.pose_odom
        if self.check_portal_authentication_physical_progress(now):
            return
        if auth.state in ("entering", "inside"):
            if auth.state == "entering":
                auth.update_entry((pose.x, pose.y))
            if auth.state == "inside":
                from global_frontier_grid import frontier_mask
                grid_frontiers = frontier_mask(
                    map_context.known_free, map_context.unknown,
                )
                from global_frontier_place_portal_validation import authentication_inside_frontiers
                grid_frontiers, inside_scope = authentication_inside_frontiers(
                    self, tree, auth, message, map_context, grid_frontiers)
                if grid_frontiers is None:
                    # Missing SLAM/TF is not negative doorway evidence. Keep
                    # the recorded full-body witness and retry on the next map.
                    return
                cells = np.argwhere(grid_frontiers)
                frame = message.header.frame_id or "map"
                from global_frontier_geometry import map_snapshot_projector, inverse_planar_projector
                project = map_snapshot_projector(self, message)
                unproject = None if project is None else inverse_planar_projector(project)
                if unproject is None:
                    return
                gate_map = inside_scope["gate_map_xy"]
                heading_map = inside_scope["heading_map"]
                if cells.size == 0:
                    left = right = 0
                    headings = []
                else:
                    resolution = float(message.info.resolution)
                    origin = message.info.origin.position
                    xs = float(origin.x) + (cells[:, 1] + 0.5) * resolution
                    ys = float(origin.y) + (cells[:, 0] + 0.5) * resolution
                    dx, dy = math.cos(heading_map), math.sin(heading_map)
                    rx, ry = xs - gate_map[0], ys - gate_map[1]
                    lateral = -rx * dy + ry * dx
                    # The cut component already owns these frontiers. Include
                    # its whole connected Place, including long/bent branches.
                    left = int(np.count_nonzero(lateral >= 0.0))
                    right = int(np.count_nonzero(lateral < 0.0))
                    side = (lateral >= 0.0 if left >= right else lateral < 0.0)
                    if np.any(side):
                        mean_x = float(np.mean(xs[side]))
                        mean_y = float(np.mean(ys[side]))
                        mean_odom = unproject(mean_x, mean_y)
                        headings = ([] if mean_odom is None else [math.atan2(
                            mean_odom[1] - float(pose.y),
                            mean_odom[0] - float(pose.x),
                        )])
                    else:
                        headings = []
                auth.observe_inside(left, right, float(pose.theta), headings)
                self.sync_portal_authentication_phase(auth)
                self.publish_status(
                    "portal_authentication_inside_evidence",
                    node_id=auth.node_id, branch_id=auth.branch_id,
                    state=auth.state, left_frontier_cells=left,
                    right_frontier_cells=right,
                    turn_direction=auth.turn_direction,
                    target_yaw=auth.target_yaw,
                    failure_reason=auth.failure_reason,
                    inside_frontier_scope=inside_scope,
                )
                if (auth.state == "turning"
                        or (auth.state == "returning" and auth.failure_reason)):
                    # Full-body entry finishes the entry lease. A turn must
                    # get its own typed action after that exact release ACK,
                    # rather than mutating the old entry endpoint in place.
                    # Empty pockets likewise return without chasing entry.
                    self.release_active_frontier(
                        discard_prefetch=True, queue_successor=False,
                        await_controller_release_ack=True,
                        preserve_place_departure=bool(getattr(
                            getattr(self, "place_departure", None), "active", False)),
                    )
        elif auth.state == "turning":
            auth.update_rotation(float(pose.theta), stable_samples=3)
            if auth.rotation_ready and auth.finish_turn(float(pose.theta)):
                self.active_portal_authentication_phase = "returning"
                self.publish_status(
                    "portal_authentication_turn_complete",
                    node_id=auth.node_id, branch_id=auth.branch_id,
                    turn_direction=auth.turn_direction,
                    target_yaw=auth.target_yaw, final_yaw=float(pose.theta),
                )
                self.release_active_frontier(
                    discard_prefetch=True,
                    preserve_place_departure=bool(
                        getattr(getattr(self, "place_departure", None),
                                "active", False)
                    ),
                )
        elif auth.state == "returning":
            auth.update_return((pose.x, pose.y))
        if auth.state in ("completed", "failed") and not getattr(
                auth, "terminal_reported", False):
            auth.terminal_reported = True
            self.publish_status(
                "portal_authentication_terminal_evidence",
                node_id=auth.node_id, branch_id=auth.branch_id,
                state=auth.state, failure_reason=auth.failure_reason,
                snapshot=auth.snapshot(),
            )
            # This callback owns the physical return-radius evidence. Commit
            # the ledger here so a wider auth radius is not held hostage by
            # the narrower generic waypoint terminal contract.
            # Physical return completes this lease at the authentication
            # radius. Do not leave TEB chasing the narrower endpoint/yaw
            # after the ledger has advanced or entered its waiting period.
            self.active_portal_authentication_phase = ""
            self.release_active_frontier(
                discard_prefetch=True, queue_successor=False,
                await_controller_release_ack=True,
                preserve_place_departure=bool(
                    getattr(getattr(self, "place_departure", None), "active", False)),
            )
            if auth.state == "completed":
                self._register_authenticated_gp_portal(auth, now)
            self.finish_portal_authentication_branch(now=now)

    def _register_authenticated_gp_portal(self, auth, now):
        """Hand a returned, observed doorway to the canonical Portal ledger."""
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        source_place_id = getattr(self, "current_physical_place_id", None)
        if ledger is None or source_place_id is None:
            self.publish_status(
                "gp_portal_handoff_deferred", branch_id=int(auth.branch_id),
                reason="canonical_source_place_unavailable",
            )
            return None
        tree = getattr(self, "gp_exploration_tree", None)
        branch = None if tree is None else tree.branches.get(auth.branch_id)
        from global_frontier_place_partition import authentication_gate_geometry
        width, bounds = authentication_gate_geometry(auth, branch)
        if width is None:
            auth.reject_completed_portal_candidate("opening_width_evidence_unavailable")
            self.publish_status("portal_candidate_rejected", node_id=int(auth.node_id),
                branch_id=int(auth.branch_id), reason="opening_width_evidence_unavailable")
            return None
        gate = tuple(float(v) for v in auth.gate_xy)
        inside = getattr(auth, "inside_xy", None)
        if inside is None:
            auth.reject_completed_portal_candidate("inside_witness_unavailable")
            self.publish_status("portal_candidate_rejected", node_id=int(auth.node_id),
                branch_id=int(auth.branch_id), reason="inside_witness_unavailable")
            return None
        # The GP ray endpoint can lie beyond a wall. The canonical Place seed
        # must be the same full-body pose that supplied the SLAM inside proof.
        destination = tuple(float(v) for v in inside)
        from global_frontier_place_portal_validation import authenticated_place_portal_evidence
        spatial = authenticated_place_portal_evidence(self, auth)
        if spatial is not None:
            self.publish_status("portal_place_spatial_evidence", node_id=int(auth.node_id),
                branch_id=int(auth.branch_id), source_place_id=int(source_place_id), evidence=spatial)
            if not spatial.get("reject") and spatial.get("reason") != "distinct_destination_component":
                spatial = dict(spatial, reject=True, reason="slam_place_boundary_unproven")
            if spatial.get("reject"):
                auth.reject_completed_portal_candidate(spatial["reason"])
                self.publish_status("portal_candidate_rejected", node_id=int(auth.node_id),
                    branch_id=int(auth.branch_id), reason=spatial["reason"], spatial_evidence=spatial)
                return None
        elif getattr(self, "active_semantic_room", None) is not None:
            auth.reject_completed_portal_candidate("slam_place_boundary_unproven")
            self.publish_status("portal_candidate_rejected", node_id=int(auth.node_id),
                branch_id=int(auth.branch_id), reason="slam_place_boundary_unproven")
            return None
        result, created = ledger.certify(
            int(source_place_id), gate, destination, now=float(now),
            task_version=str(getattr(self, "current_task_version", "") or ""),
        )
        if result is None:
            return None
        if created:
            result["gp_predicted_destination"] = list(auth.destination_xy)
            result["gp_inside_witness"] = list(destination)
            result["gp_authentication_required_depth"] = float(auth.required_depth)
            result["opening_width"] = width
            result["physical_bounds"] = None if bounds is None else [list(point) for point in bounds]
        probe_ledger = getattr(self, "portal_probe_ledger", None)
        if probe_ledger is not None:
            direction = (
                destination[0] - gate[0],
                destination[1] - gate[1],
            )
            matched_probe = probe_ledger.certify_matching(
                int(source_place_id), gate, direction, int(result["id"]),
                now=float(now),
            )
            if matched_probe is not None:
                self.publish_status(
                    "gp_portal_probe_satisfied",
                    branch_id=int(auth.branch_id),
                    portal_id=int(result["id"]),
                    probe_id=int(matched_probe["id"]),
                    state=str(matched_probe["state"]),
                )
        self.publish_status(
            "gp_portal_handed_to_place_graph", portal_id=int(result["id"]),
            source_place_id=int(source_place_id), branch_id=int(auth.branch_id),
            created=bool(created), gate=list(gate), destination=list(destination),
            evidence="gp_slam_dedup+inside_frontier+return_to_junction",
        )
        if branch is not None:
            branch["canonical_portal_id"] = int(result["id"])
            tree = getattr(self, "gp_exploration_tree", None)
            if tree is not None:
                duplicate = next((other for other in tree.branches.values()
                                  if int(other.get("id", -1)) != int(auth.branch_id)
                                  and int(other.get("node_id", -1)) == int(auth.node_id)
                                  and other.get("canonical_portal_id") == int(result["id"])
                                  and other.get("state") != "superseded"), None)
                if duplicate is not None:
                    branch["state"] = "superseded"
                    branch["duplicate_of_branch_id"] = int(duplicate["id"])
                    self.publish_status(
                        "gp_portal_branch_deduplicated",
                        portal_id=int(result["id"]), node_id=int(auth.node_id),
                        branch_id=int(auth.branch_id),
                        canonical_branch_id=int(duplicate["id"]),
                        evidence="canonical_portal_ledger_same_source_place_and_gate",
                    )
        return result

    def _portal_authentication_route_candidate(self, message, steps, auth):
        """Compile one authentication phase through the existing Navfn route."""
        return_recovery_fallback = bool(
            auth.state == "returning"
            and (getattr(auth, "return_recovery_blocked", False)
                 or int(getattr(auth, "return_retry_count", 0)) > 0)
        )
        if return_recovery_fallback and not getattr(
                auth, "return_recovery_blocked_reported", False):
            auth.return_recovery_blocked_reported = True
            self.publish_status(
                "portal_authentication_failure_return_recovery_blocked",
                node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                retry_count=int(getattr(auth, "return_retry_count", 0)),
                source_xy=auth.source_xy,
            )
        goal_odom = auth.route_goal()
        if goal_odom is None:
            return None
        frame = message.header.frame_id or "map"
        goal_map = self.transform_xy(frame, "odom", *goal_odom)
        if goal_map is None:
            return None
        cell = self.xy_to_grid_cell(message, *goal_map)
        if ((cell is None or steps[cell] < 0) and auth.state == "returning"
                and not auth.failure_reason and not auth.return_gate_cleared
                and not auth.source_side_proven()):
            # The estimated plane is beyond the source node and its clearing
            # waypoint is unreachable. A nearest-cell substitute may never
            # cross that plane: retire this unproven candidate and return to
            # the actual junction instead of redispatching the substitute.
            pose = getattr(self, "pose_odom", None)
            pose_xy = None if pose is None else (float(pose.x), float(pose.y))
            if not auth.request_failure_return(
                    "no_reachable_source_side_return_goal", pose_xy=pose_xy):
                self.finish_portal_authentication_branch(now=rospy.get_time())
                return None
            self.publish_status("portal_authentication_failure_return",
                node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                reason=auth.failure_reason, rejected_goal_odom=list(goal_odom))
            goal_odom = auth.route_goal()
            goal_map = self.transform_xy(frame, "odom", *goal_odom)
            if goal_map is None:
                return None
            cell = self.xy_to_grid_cell(message, *goal_map)
        if return_recovery_fallback or cell is None or steps[cell] < 0:
            # After repeated no-progress returns, keep the authentication
            # obligation active but force a newly selected Navfn-reachable
            # source-side waypoint. Do not leave a returning branch with no
            # executable route merely because its original point is reachable.
            # A reachable route cell must also become the controller goal.
            # Keeping the original inaccessible destination here repeatedly
            # launches an endpoint the global planner cannot reach.
            fallback_goal = None
            if auth.state == "entering":
                gate_map = self.transform_xy(frame, "odom", *auth.gate_xy)
                destination_map = self.transform_xy(frame, "odom", *auth.destination_xy)
                reachable = np.argwhere(steps >= 0)
                points = np.asarray([
                    self.cell_xy(message, int(row), int(col))
                    for row, col in reachable
                ], dtype=float)
                minimum_depth = float(auth.required_depth) + max(
                    0.1, float(auth.entry_goal_margin) - 0.05,
                )
                if gate_map is not None and destination_map is not None:
                    locked_approach_odom = getattr(auth, "entry_approach_goal_odom", None)
                    locked_approach = reproject_locked_entry_goal(
                        self.transform_xy, frame, locked_approach_odom,
                    )
                    locked_cell = (None if locked_approach is None else
                                   self.xy_to_grid_cell(message, *locked_approach))
                    if locked_cell is not None and steps[locked_cell] >= 0:
                        cell = locked_cell
                        fallback_goal = tuple(float(v) for v in locked_approach)
                        self.active_portal_authentication_entry_approach = True
                    else:
                        tree = getattr(self, "gp_exploration_tree", None)
                        branch = {} if tree is None else tree.branches.get(auth.branch_id, {})
                        half_width = max(0.4, float(branch_opening_width(branch) or 1.0) / 2.0)
                        nearest_inside = select_reachable_interior_goal(
                            points, gate_map, destination_map, goal_map, minimum_depth,
                            max_lateral=half_width,
                        )
                        if nearest_inside is not None:
                            cell = int(reachable[nearest_inside, 0]), int(reachable[nearest_inside, 1])
                            fallback_goal = points[nearest_inside]
                            auth.entry_approach_goal_odom = None
                if fallback_goal is None and gate_map is not None and destination_map is not None:
                    pose = getattr(self, "pose_odom", None)
                    current = (goal_map if pose is None else self.transform_xy(
                        frame, "odom", float(pose.x), float(pose.y)))
                    if current is None:
                        return None
                    approach = select_reachable_approach_goal(
                        points, gate_map, destination_map, current,
                    )
                    if approach is not None:
                        anchor_cell = int(reachable[approach, 0]), int(reachable[approach, 1])
                        anchor_goal = points[approach]
                        extended_goal = advance_goal_along_portal_axis(
                            anchor_goal, gate_map, destination_map,
                            max(0.45, float(getattr(
                                self, "endpoint_terminal_wait_radius", 0.45,
                            ))),
                        )
                        extended_cell = self.xy_to_grid_cell(message, *extended_goal)
                        if extended_cell is not None and steps[extended_cell] >= 0:
                            cell = extended_cell
                            fallback_goal = extended_goal
                        else:
                            # Use the reachable map cell to anchor Navfn, while
                            # asking the local point-cloud planner for one small
                            # collision-checked step into the already-open portal.
                            cell = anchor_cell
                            fallback_goal = advance_goal_along_portal_axis(
                                anchor_goal, gate_map, destination_map, 0.25,
                            )
                        goal_odom_locked = self.transform_xy("odom", frame, *fallback_goal)
                        if goal_odom_locked is not None:
                            auth.entry_approach_goal_odom = tuple(float(v) for v in goal_odom_locked)
                        self.active_portal_authentication_entry_approach = True
                        self.publish_status(
                            "portal_authentication_entry_approach_selected",
                            node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                            goal=[round(float(v), 3) for v in fallback_goal],
                        )
                if fallback_goal is None:
                    stamp = getattr(message.header, "stamp", None)
                    auth._event("doorway_interior_plan_unavailable",
                        minimum_depth=float(minimum_depth),
                        feasible_interior_goal_count=0,
                        map_stamp=None if stamp is None else stamp.to_sec(),
                        map_epoch=getattr(self, "last_map_epoch", None),
                        navigation_clearance=getattr(self, "clearance", None))
                    self.publish_status(
                        "portal_authentication_no_reachable_interior_goal",
                        node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                        minimum_depth=round(minimum_depth, 3),
                    )
                    pose = getattr(self, "pose_odom", None)
                    pose_xy = None if pose is None else (float(pose.x), float(pose.y))
                    if not auth.request_failure_return(
                            "no_reachable_interior_entry_goal", pose_xy=pose_xy):
                        self.finish_portal_authentication_branch(now=rospy.get_time())
                        return None
                    goal_odom = auth.route_goal()
                    goal_map = self.transform_xy(frame, "odom", *goal_odom)
                    self.active_portal_authentication_phase = "returning"
                    if goal_map is None:
                        return None
            if fallback_goal is None:
                reachable = np.argwhere(steps >= 0)
                if reachable.size == 0:
                    return None
                points = np.asarray([
                    self.cell_xy(message, int(row), int(col))
                    for row, col in reachable
                ], dtype=float)
                if auth.state == "returning":
                    gate_map = self.transform_xy(frame, "odom", *auth.gate_xy)
                    destination_map = self.transform_xy(
                        frame, "odom", *auth.destination_xy,
                    )
                    source_map = self.transform_xy(frame, "odom", *auth.source_xy)
                    if gate_map is None or destination_map is None or source_map is None:
                        return None
                    resolution = float(message.info.resolution)
                    path_costs = np.asarray([
                        float(steps[int(row), int(col)])
                        for row, col in reachable
                    ], dtype=float)
                    pose = getattr(self, "pose_odom", None)
                    current_map = None if pose is None else self.transform_xy(
                        frame, "odom", float(pose.x), float(pose.y),
                    )
                    failed_return_goals = []
                    for failed_goal_odom in getattr(
                            auth, "failed_source_side_return_goals_odom", ()):
                        failed_goal_map = self.transform_xy(
                            frame, "odom", *failed_goal_odom,
                        )
                        if failed_goal_map is not None:
                            failed_return_goals.append(failed_goal_map)
                    blocked_approaches = []
                    for obstacle_odom in getattr(
                            auth, "failed_return_approach_obstacles_odom", ()):
                        obstacle_map = self.transform_xy(
                            frame, "odom", *obstacle_odom,
                        )
                        if obstacle_map is not None:
                            blocked_approaches.append(obstacle_map)
                    maximum_source_distance = 3.0
                    if current_map is not None:
                        maximum_source_distance = max(
                            maximum_source_distance,
                            math.hypot(
                                current_map[0] - source_map[0],
                                current_map[1] - source_map[1],
                            ) + 1.0,
                        )
                    selected = select_reachable_source_side_return_goal(
                        points, path_costs, gate_map, destination_map,
                        source_map, resolution=resolution,
                        minimum_depth=max(0.55, 4.0 * resolution),
                        current_xy=current_map,
                        excluded_goals=failed_return_goals,
                        blocked_approach_points=blocked_approaches,
                        maximum_source_distance=maximum_source_distance,
                        gate_cleared=bool(getattr(auth, "return_gate_cleared", False)),
                        return_radius=float(getattr(auth, "return_radius", 0.75)),
                        goal_xy_tolerance=float(getattr(self, "teb_xy_goal_tolerance", 0.5)),
                    )
                    return_goal_kind = "source_side"
                    if selected is None:
                        selected = select_reachable_return_sidestep_goal(
                            points, path_costs, current_map, gate_map,
                            destination_map, resolution=resolution,
                            excluded_goals=failed_return_goals,
                            blocked_approach_points=blocked_approaches,
                            source_xy=source_map,
                            gate_cleared=bool(getattr(auth, "return_gate_cleared", False)),
                        )
                        return_goal_kind = "local_sidestep"
                        if selected is None:
                            report_key = (
                                int(getattr(auth, "return_retry_count", 0)),
                                len(failed_return_goals),
                                len(blocked_approaches),
                            )
                            if getattr(auth, "last_no_return_goal_report_key", None) != report_key:
                                auth.last_no_return_goal_report_key = report_key
                                dx = destination_map[0]-gate_map[0]
                                dy = destination_map[1]-gate_map[1]
                                norm = max(1e-9, math.hypot(dx, dy))
                                current_depth = (
                                    (current_map[0]-gate_map[0])*dx
                                    + (current_map[1]-gate_map[1])*dy
                                ) / norm
                                self.publish_status(
                                    "portal_authentication_no_reachable_source_side_return_goal",
                                    node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                                    source_xy=list(auth.source_xy),
                                    gate_xy=list(auth.gate_xy),
                                    destination_xy=list(auth.destination_xy),
                                    navfn_reachable_count=int(len(points)),
                                    current_depth=round(float(current_depth), 3),
                                    failed_goal_count=len(failed_return_goals),
                                    avoided_approach_count=len(blocked_approaches),
                                    return_retry_count=int(getattr(auth, "return_retry_count", 0)),
                                    recovery_blocked=bool(getattr(
                                        auth, "return_recovery_final_blocked", False)),
                                )
                            return None
                    cell = int(reachable[selected, 0]), int(reachable[selected, 1])
                    fallback_goal = points[selected]
                    fallback_goal_odom = self.transform_xy(
                        "odom", frame, *fallback_goal,
                    )
                    if fallback_goal_odom is not None:
                        auth.last_source_side_return_goal_odom = tuple(
                            float(value) for value in fallback_goal_odom
                        )
                    dx = destination_map[0]-gate_map[0]
                    dy = destination_map[1]-gate_map[1]
                    source_depth = float(
                        (fallback_goal[0]-gate_map[0])*dx
                        + (fallback_goal[1]-gate_map[1])*dy
                    ) / max(1e-9, math.hypot(dx, dy))
                    self.publish_status(
                        "portal_authentication_source_side_return_goal_selected"
                        if return_goal_kind == "source_side"
                        else "portal_authentication_return_sidestep_selected",
                        node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                        goal_map=[round(float(v), 3) for v in fallback_goal],
                        avoided_approach_count=len(blocked_approaches),
                        source_depth=round(source_depth, 3),
                        return_retry_count=int(getattr(auth, "return_retry_count", 0)),
                    )
                else:
                    nearest = int(np.argmin(np.sum(
                        (points - np.asarray(goal_map)) ** 2, axis=1,
                    )))
                    cell = int(reachable[nearest, 0]), int(reachable[nearest, 1])
                    fallback_goal = points[nearest]
            goal_map = (float(fallback_goal[0]), float(fallback_goal[1]))
            fallback_goal_odom = self.transform_xy("odom", frame, *goal_map)
            if fallback_goal_odom is not None:
                goal_odom = (float(fallback_goal_odom[0]), float(fallback_goal_odom[1]))
        distance = max(0.0, float(steps[cell])) * float(message.info.resolution)
        route_kind = (
            "frontier_turn_connector" if auth.state == "turning"
            else "frontier_endpoint"
        )
        route = [cell[0], cell[1], goal_map[0], goal_map[1], distance,
                 0.0, 0.0, 0.0, None, 0, route_kind]
        route.extend([None] * (21 - len(route)))
        route.append(int(auth.branch_id))
        route.append(str(auth.state))
        signature = (auth.state, tuple(round(float(value), 3) for value in goal_odom),
                     None if auth.target_yaw is None else round(float(auth.target_yaw), 3))
        if getattr(auth, "last_route_signature", None) != signature:
            auth.last_route_signature = signature
            self.publish_status(
                "portal_authentication_route_selected",
                node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                phase=auth.state, goal_odom=list(goal_odom),
                goal_map=list(goal_map), target_yaw=auth.target_yaw,
                turn_direction=auth.turn_direction,
            )
        return tuple(route)

    def choose_gp_branch_frontier(self, message, steps, now, robot_map, semantic_hint, excluded):
        """Route only the active Portal observation phase through Navfn."""
        auth = getattr(self, "active_portal_authentication", lambda: None)()
        if auth is None:
            return None
        if (getattr(auth, "state", "") == "returning"
                and getattr(auth, "return_recovery_final_blocked", False)):
            return None
        if getattr(self, "awaiting_controller_lease_release_ack", False):
            # A phase can release its predecessor during this same planning
            # pass, after the usual ingress gate. Do not mint an undispatched
            # successor lease while the exact predecessor ACK is still pending.
            return None
        return self._portal_authentication_route_candidate(message, steps, auth)
