"""GP branch discovery and tree adapter for the existing Navfn/TEB executor."""
import math
import numpy as np
import rospy
from gp_branch_discovery import PolarGaussianProcess, entrance_geometry, angle_delta, map_ranges, junction_anchor
from gp_exploration_tree import ExplorationTree
from global_frontier_portal_authentication import DoorwayAuthentication


class GPBranchFrontierMixin:
    def update_gp_branch_tree(self, message, map_context, robot_map, robot_yaw, now):
        if getattr(self,'exploration_method','')!='gp_branch_tree':
            return
        scan=getattr(self,'gp_branch_scan',None)
        if scan is None:
            return
        if not hasattr(self,'gp_exploration_tree'):
            self.gp_exploration_tree=ExplorationTree()
            self.gp_direction_model=PolarGaussianProcess()
        tree=self.gp_exploration_tree
        active_auth = getattr(self, 'active_portal_authentication', lambda: None)()
        if active_auth is not None and active_auth.node_id in tree.nodes:
            # Observation owns its source node until the robot returns, even
            # when a wider entrance lies beyond the usual association radius.
            tree.current_node_id = active_auth.node_id
        stamp=scan.header.stamp.to_sec()
        if stamp==tree.last_stamp:
            return
        # Use the sensor acquisition transform, including its real mounting
        # offset; repeated current-pose projection would move old openings.
        try:
            map_tf=self.tf_buffer.lookup_transform(message.header.frame_id or 'map',
                          scan.header.frame_id,scan.header.stamp,rospy.Duration(0.05)).transform
            odom_tf=self.tf_buffer.lookup_transform('odom',scan.header.frame_id,
                          scan.header.stamp,rospy.Duration(0.05)).transform
        except Exception as exc:
            rospy.logwarn_throttle(5,'GP branch acquisition transform unavailable: %s',str(exc))
            return
        def yaw(transform):
            q=transform.rotation
            return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        map_yaw=yaw(map_tf);odom_yaw=yaw(odom_tf)
        source=(map_tf.translation.x,map_tf.translation.y)
        physical=(odom_tf.translation.x,odom_tf.translation.y)
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
            if ((node['id']==1 or node.get('junction_confirmed')) and
                (math.dist(physical,node['physical_xy'])<=tree.node_radius or active_auth is not None)):
                anchor=self.transform_xy(message.header.frame_id or 'map','odom',*node['physical_xy'])
                if anchor is not None:
                    rays=map_ranges(grid,resolution,origin,anchor,self.gp_direction_model.angles)
                    refined=self.gp_direction_model.propose(rays,-math.pi,
                        2*math.pi/len(rays),0.,8.,minimum_width=2*self.clearance)
                    refined=retain_sensor_proposals(refined,anchor)
                    if refined:
                        proposals=refined
                        map_yaw=0.0
                        # Map directions transform back to the physical frame.
                        odom_yaw=yaw(odom_tf)-yaw(map_tf)
                        observation_source=anchor
                        observation_physical=tuple(node['physical_xy'])
        # Before a new fork's identity is frozen, move its sensor-view
        # hypothesis to the local map's actual junction center. This avoids
        # recording the robot's distant viewing position as the junction.
        headings=[p.heading for p in proposals]
        separated=all(abs(angle_delta(a,b))>=0.9 for i,a in enumerate(headings) for b in headings[i+1:])
        local_fork=(len(headings)>=3 and separated) or (len(headings)==2 and 1.1<=abs(angle_delta(*headings))<=2.1)
        current_node=None if tree.current_node_id is None else tree.nodes[tree.current_node_id]
        if active_auth is None and local_fork and (current_node is None or not current_node.get('junction_confirmed') or
                          math.dist(physical,current_node['physical_xy'])>tree.node_radius):
            anchor=junction_anchor(grid,resolution,origin,source)
            physical_anchor=self.transform_xy('odom',message.header.frame_id or 'map',*anchor)
            if physical_anchor is not None:
                rays=map_ranges(grid,resolution,origin,anchor,self.gp_direction_model.angles)
                refined=self.gp_direction_model.propose(rays,-math.pi,2*math.pi/len(rays),0.,8.,minimum_width=2*self.clearance)
                refined=retain_sensor_proposals(refined,anchor)
                if refined:
                    proposals=refined
                    map_yaw=0.
                    odom_yaw=yaw(odom_tf)-yaw(map_tf)
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
                gate=self.transform_xy('odom',message.header.frame_id or 'map',*geometry['position'])
            observations.append(dict(heading=odom_yaw+proposal.heading,
                variance=proposal.variance,reach=proposal.reach,
                angular_width=proposal.angular_width,geometry=geometry,
                physical_gate=None if gate is None else list(gate)))
        def line_free(a,b):
            # Reproject durable odom anchors through this map's current TF.
            first=self.transform_xy(message.header.frame_id or 'map','odom',*a)
            last=self.transform_xy(message.header.frame_id or 'map','odom',*b)
            if first is None or last is None:
                return False
            for fraction in np.linspace(0,1,max(2,int(math.dist(first,last)/resolution)+1)):
                x=first[0]+fraction*(last[0]-first[0]);y=first[1]+fraction*(last[1]-first[1])
                cell=self.xy_to_grid_cell(message,x,y)
                if cell is None or grid[cell]!=0:
                    return False
            return True
        self.gp_latest_observations = observations
        tree.observe(observation_physical,observation_source,observations,now,stamp,map_context.components.epoch,line_free)
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
            dedup_audit=dict(getattr(self, 'gp_last_dedup_audit', {})))

    def _sync_portal_authentication(self, tree, message, map_context, now):
        """Maintain the stable branch set for a node's observation period."""
        node_id = tree.current_node_id
        node = tree.nodes.get(node_id) if node_id is not None else None
        if node is None or not node.get("junction_confirmed"):
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
        branches = []
        for branch_id in node.get("branch_ids", ()):
            branch = tree.branches.get(branch_id)
            if (branch is None or branch.get("state") in (
                    "superseded", "authentication_failed", "authenticated")
                    or branch.get("connection") == "return"):
                continue
            if not branch.get("stable") or branch.get("passability") != "open":
                continue
            gate = branch.get("physical_gate")
            if gate is None or branch.get("authentication_state") in (
                    "completed", "failed"):
                continue
            heading = float(branch.get("heading", 0.0))
            # The full base footprint must clear the gate plane. The
            # navigation launch uses a 0.30 m circular footprint; add only
            # 0.10 m beyond its rear edge so the goal remains reachable in a
            # short doorway while still proving the whole body crossed.
            body_radius = float(rospy.get_param(
                "/move_base/global_costmap/robot_radius", 0.30))
            depth = max(0.35, body_radius + 0.10)
            destination = (float(gate[0]) + depth * math.cos(heading),
                           float(gate[1]) + depth * math.sin(heading))
            terminal_radius = float(getattr(
                self, "endpoint_terminal_wait_radius", 0.45))
            entry_margin = max(0.55, terminal_radius + 0.10)
            auth = DoorwayAuthentication(
                branch_id, node_id, gate, destination, required_depth=depth,
                entry_goal_margin=entry_margin,
            )
            if task.add(auth, now=now):
                branch["authentication_state"] = "pending"
                branch["authentication_failure_reason"] = None
                branches.append(auth)
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

    def _update_portal_authentication_from_map(
        self, tree, message, map_context, frontier_proposals, map_yaw, now,
    ):
        """Use live SLAM frontier cells and odometry to advance one probe."""
        auth = self.active_portal_authentication()
        if auth is None or getattr(self, "pose_odom", None) is None:
            return
        pose = self.pose_odom
        if auth.state == "entering":
            auth.update_entry((pose.x, pose.y))
            if auth.state == "inside":
                from global_frontier_grid import frontier_mask
                grid_frontiers = frontier_mask(
                    map_context.known_free, map_context.unknown,
                )
                cells = np.argwhere(grid_frontiers)
                frame = message.header.frame_id or "map"
                gate_map = self.transform_xy(frame, "odom", *auth.gate_xy)
                tree_branch = tree.branches.get(auth.branch_id, {})
                heading_odom = float(tree_branch.get("heading", 0.0))
                if gate_map is None:
                    # A transient TF gap is not negative doorway evidence.
                    return
                if cells.size == 0:
                    left = right = 0
                    headings = []
                else:
                    resolution = float(message.info.resolution)
                    origin = message.info.origin.position
                    xs = float(origin.x) + (cells[:, 1] + 0.5) * resolution
                    ys = float(origin.y) + (cells[:, 0] + 0.5) * resolution
                    gate_odom = auth.gate_xy
                    forward_map = self.transform_xy(
                        frame, "odom",
                        gate_odom[0] + math.cos(heading_odom),
                        gate_odom[1] + math.sin(heading_odom),
                    )
                    if forward_map is None:
                        return
                    heading_map = math.atan2(
                        forward_map[1] - gate_map[1],
                        forward_map[0] - gate_map[0],
                    )
                    dx, dy = math.cos(heading_map), math.sin(heading_map)
                    rx, ry = xs - gate_map[0], ys - gate_map[1]
                    depth = rx * dx + ry * dy
                    lateral = -rx * dy + ry * dx
                    room = (depth >= auth.required_depth * 0.5) & (depth <= 8.0)
                    left = int(np.count_nonzero(room & (lateral >= 0.0)))
                    right = int(np.count_nonzero(room & (lateral < 0.0)))
                    side = (room & (lateral >= 0.0) if left >= right
                            else room & (lateral < 0.0))
                    if np.any(side):
                        mean_x = float(np.mean(xs[side]))
                        mean_y = float(np.mean(ys[side]))
                        mean_odom = self.transform_xy(frame, "odom", mean_x, mean_y)
                        headings = ([] if mean_odom is None else [math.atan2(
                            mean_odom[1] - float(pose.y),
                            mean_odom[0] - float(pose.x),
                        )])
                    else:
                        headings = []
                auth.observe_inside(left, right, float(pose.theta), headings)
                self.publish_status(
                    "portal_authentication_inside_evidence",
                    node_id=auth.node_id, branch_id=auth.branch_id,
                    state=auth.state, left_frontier_cells=left,
                    right_frontier_cells=right,
                    turn_direction=auth.turn_direction,
                    target_yaw=auth.target_yaw,
                    failure_reason=auth.failure_reason,
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
            self.finish_portal_authentication_branch(now=now)

    def observe_gp_branch_semantics(self, message):
        tree=getattr(self,'gp_exploration_tree',None)
        info=getattr(self,'gp_branch_camera_info',None)
        intent=getattr(getattr(self,'semantic_place_belief',None),'intent',None)
        if tree is None or info is None or intent is None or not intent.active:
            return
        from global_frontier_semantic_belief import matched_terms
        if not info.K or info.K[0]<=0 or info.width<=0:
            return
        try:
            tf=self.tf_buffer.lookup_transform('odom',info.header.frame_id,
                  message.header.stamp,rospy.Duration(0.05)).transform
        except Exception:
            return
        q=tf.rotation
        rotation=np.array([
            [1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.z*q.w),2*(q.x*q.z+q.y*q.w)],
            [2*(q.x*q.y+q.z*q.w),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.x*q.w)],
            [2*(q.x*q.z-q.y*q.w),2*(q.y*q.z+q.x*q.w),1-2*(q.x*q.x+q.y*q.y)]])
        for detection in list(message.target_dets)+list(message.env_dets):
            cx=float(detection.cx)
            if not math.isfinite(cx) or not 0<=cx<=1:
                continue
            vector=rotation @ np.array([(cx*info.width-info.K[2])/info.K[0],0.,1.])
            if math.hypot(vector[0],vector[1])<1e-6:
                continue
            bearing=math.atan2(vector[1],vector[0])
            options=[b for b in tree.branches.values()
                if b['node_id']==tree.current_node_id and b['state']!='superseded'
                and abs(angle_delta(b['heading'],bearing))<=0.55]
            if not options:
                continue
            branch=min(options,key=lambda b:abs(angle_delta(b['heading'],bearing)))
            if branch.get('semantic_task_version')!=intent.task_version:
                branch['semantic_evidence']={}
                branch['semantic_task_version']=intent.task_version
            label=str(detection.label)
            confidence=float(detection.score)
            if not math.isfinite(confidence) or not 0<=confidence<=1:
                continue
            kind=('target' if matched_terms(label,intent.target_terms) else
                  'context' if matched_terms(label,intent.context_terms) else
                  'negative' if matched_terms(label,intent.negative_terms) else None)
            if kind is not None:
                evidence=branch['semantic_evidence']
                evidence[kind]=max(evidence.get(kind,0.),confidence)
                branch['semantic_score']=evidence.get('target',0.)+evidence.get('context',0.)-evidence.get('negative',0.)

    def commit_gp_branch_transaction(self, branch_id, route_id):
        from global_frontier_graph_route_planner import GraphRoutePlan
        from global_frontier_graph_route_transaction import GraphRouteActionTransaction
        epoch=max(1,int(getattr(self,'topology_component_epoch',1)))
        sequence=int(getattr(self,'graph_route_action_transaction_sequence',0))+1
        self.graph_route_action_transaction_sequence=sequence
        branch=self.gp_exploration_tree.branches[branch_id]
        plan=GraphRoutePlan(status='ready',action='observe_local_work',
             map_epoch=epoch,current_place_id=branch['node_id'],
             obligation_kind='branch',obligation_id=branch_id,
             reason='gp_branch_semantic_selection')
        self.last_graph_route_plan=plan
        self.graph_route_action_transaction=GraphRouteActionTransaction(
             transaction_id=sequence,plan=plan,map_epoch=epoch,
             origin_epoch=epoch,projection_epoch=epoch,phase='committed')
        self.gp_exploration_tree.dispatch(branch_id,route_id,
                    (self.pose_odom.x,self.pose_odom.y))

    def _portal_authentication_route_candidate(self, message, steps, auth):
        """Compile one authentication phase through the existing Navfn route."""
        goal_odom = auth.route_goal()
        if goal_odom is None:
            return None
        frame = message.header.frame_id or "map"
        goal_map = self.transform_xy(frame, "odom", *goal_odom)
        if goal_map is None:
            return None
        cell = self.xy_to_grid_cell(message, *goal_map)
        if cell is None or steps[cell] < 0:
            branch = self.gp_exploration_tree.branches.get(auth.branch_id, {})
            gate_map = self.transform_xy(frame, "odom", *auth.gate_xy)
            if auth.state == "entering" and gate_map is not None:
                cell = self.xy_to_grid_cell(message, *gate_map)
            if cell is None or steps[cell] < 0:
                reachable = np.argwhere(steps >= 0)
                if reachable.size == 0:
                    return None
                points = np.asarray([
                    self.cell_xy(message, int(row), int(col))
                    for row, col in reachable
                ], dtype=float)
                nearest = int(np.argmin(np.sum((points - np.asarray(goal_map)) ** 2, axis=1)))
                cell = int(reachable[nearest, 0]), int(reachable[nearest, 1])
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
        tree=getattr(self,'gp_exploration_tree',None)
        if tree is None:
            return None
        task = getattr(self, "junction_portal_authentication", None)
        auth = getattr(self, "active_portal_authentication", lambda: None)()
        if auth is not None:
            return self._portal_authentication_route_candidate(message, steps, auth)
        if task is not None and int(task.node_id) == int(tree.current_node_id or -1):
            # A confirmed junction owns a complete serial observation pass.
            # Wait here both while the branch set settles and after every
            # stable branch is terminal; semantic exploration decides what
            # happens after all doorways have been observed.
            return None
        scores={};endpoints={};tie_break={}
        active=tree.continuation_branch()
        continuation=None
        if active is not None:
            frame=message.header.frame_id or 'map'
            start=(self.pose_odom.x,self.pose_odom.y)
            for depth in (2.0,1.5,1.0):
                physical_goal=(start[0]+depth*math.cos(active['heading']),
                               start[1]+depth*math.sin(active['heading']))
                goal=self.transform_xy(frame,'odom',*physical_goal)
                if goal is None:continue
                cell=self.xy_to_grid_cell(message,*goal)
                if cell is not None and steps[cell]>=0 and not self.frontier_is_rejected(*goal,now):
                    continuation=(active,cell,goal)
                    break
        frame=message.header.frame_id or 'map'
        for branch in tree.branches.values():
            gate=branch['physical_gate']
            if gate is None or branch['passability']!='open':
                continue
            gate_map=self.transform_xy(frame,'odom',*gate)
            gate_cell=None if gate_map is None else self.xy_to_grid_cell(message,*gate_map)
            if gate_cell is None or steps[gate_cell]<0:
                continue
            # A target beyond the gate must still be in the current safe BFS.
            depth=self.clearance+getattr(self,'teb_xy_goal_tolerance',0.35)+message.info.resolution
            goal_physical=(gate[0]+depth*math.cos(branch['heading']),
                           gate[1]+depth*math.sin(branch['heading']))
            goal=self.transform_xy(frame,'odom',*goal_physical)
            if goal is None:
                continue
            cell=self.xy_to_grid_cell(message,*goal)
            if cell is None:
                cell=gate_cell
            route_cell=cell
            if steps[cell] < 0:
                # Unknown space beyond a confirmed open gate is a valid frontier;
                # route to the last known-safe gate cell and let navigation
                # establish the destination evidence by crossing it.
                route_cell=gate_cell
            if excluded and any(math.dist(goal,p)<self.completed_radius for p in excluded):
                continue
            if self.frontier_is_rejected(*goal,now):
                continue
            score=0.0
            if branch.get('semantic_task_version')==getattr(self,'current_task_version',''):
                score=float(branch.get('semantic_score') or 0.)
            tie_break[branch['id']]=(-math.dist(goal,semantic_hint) if semantic_hint is not None
                                    else -float(steps[route_cell]))
            scores[branch['id']]=score
            endpoints[branch['id']]=(route_cell,goal)
        if continuation is not None:
            branch,cell,goal=continuation
            scores[branch['id']]=float(branch.get('semantic_score') or 0.)
            endpoints[branch['id']]=(cell,goal)
        else:
            branch=tree.choose(scores,endpoints,tie_break)
        if branch is None:
            return None
        cell,goal=endpoints[branch['id']]
        self.last_graph_selected_candidate=None
        self.publish_status('gp_branch_selected',branch_id=branch['id'],node_id=branch['node_id'],
             semantic_score=scores[branch['id']],goal=list(goal),geometry=branch['map_geometry'])
        # The existing endpoint lifecycle owns navigation; physical branch
        # entry is separately verified from odometry by the tree adapter.
        route=[cell[0],cell[1],goal[0],goal[1],float(steps[cell])*message.info.resolution,
               0.,0.,scores[branch['id']],None,0,'frontier_endpoint']
        route.extend([None]*(21-len(route)))
        route.append(branch['id'])
        return tuple(route)
