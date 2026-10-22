"""Persistent node/branch exploration identity; no ROS or motion ownership.

A branch can exist before its destination is known. Geometric map labels are
snapshot evidence, not room IDs. Extra connections reuse an existing node.
"""
import math
from gp_branch_discovery import angle_delta


def distance(a,b):
    return math.hypot(a[0]-b[0],a[1]-b[1])


class ExplorationTree:
    def __init__(self, node_radius=2.0, heading_tolerance=0.65, stable_seconds=0.5):
        self.node_radius=node_radius
        self.heading_tolerance=heading_tolerance
        self.stable_seconds=stable_seconds
        self.nodes={}
        self.branches={}
        self.current_node_id=None
        self.active_branch_id=None
        self.last_stamp=None
        self.spaces={}

    def _new_node(self, physical_xy, map_xy, now):
        node_id=len(self.nodes)+1
        parent=self.active_branch_id
        if parent is not None:
            edge=self.branches[parent]
            gate=edge.get('physical_gate')
            if (edge['state']=='superseded' or gate is None or
                (physical_xy[0]-gate[0])*math.cos(edge['heading'])+
                (physical_xy[1]-gate[1])*math.sin(edge['heading'])<0):
                parent=None
        self.nodes[node_id]=dict(id=node_id,physical_xy=list(physical_xy),
             map_xy=list(map_xy),parent_branch_id=parent,branch_ids=[],
             first_seen=now,last_seen=now)
        if parent is None and node_id==1:
            self.spaces[1]=dict(id=1,parent_branch_id=None,entry_xy=list(physical_xy),
                                node_ids=[node_id],state='entered')
            self.nodes[node_id]['space_id']=1
        if parent is not None:
            branch=self.branches[parent]
            branch['destination_node_id']=node_id
            branch.setdefault('destination_verified',False)
            if branch.get('destination_space_id') is not None:
                self.nodes[node_id]['space_id']=branch['destination_space_id']
                self.spaces[branch['destination_space_id']]['node_ids'].append(node_id)
        self.current_node_id=node_id
        return self.nodes[node_id]

    def _matching_node(self, physical_xy, headings, line_is_free):
        matches=[]
        for node in self.nodes.values():
            gap=distance(physical_xy,node['physical_xy'])
            if gap>self.node_radius:
                continue
            if not line_is_free(node['physical_xy'],physical_xy):
                continue
            prior=[self.branches[i]['heading'] for i in node['branch_ids']]
            overlap=sum(any(abs(angle_delta(h,p))<=self.heading_tolerance for p in prior)
                        for h in headings)
            if prior and overlap<min(2,len(headings),len(prior)):
                continue
            matches.append((gap,node['id'],node))
        return min(matches,key=lambda x:x[:2])[2] if matches else None

    def observe(self, physical_xy, map_xy, proposals, now, stamp, map_epoch,
                line_is_free=lambda a,b:False):
        # A repeated timer execution of the same sensor message is not a new
        # observation and cannot satisfy temporal stability.
        if stamp==self.last_stamp:
            return self.current_node_id
        self.last_stamp=stamp
        # Sensor/map fusion can provide the same arm more than once. Its
        # duplicate must neither create a fork nor prevent fork confirmation.
        headings=[p['heading'] for p in proposals]
        distinct_headings=[]
        for heading in headings:
            if not any(abs(angle_delta(heading,prior))<=self.heading_tolerance
                       for prior in distinct_headings):
                distinct_headings.append(heading)
        if not headings:
            return self.current_node_id
        incoming=self.branches.get(self.active_branch_id)
        if (incoming is not None and incoming['state'] in ('active','entered') and
            all(abs(angle_delta(h,incoming['heading']))>math.pi/2 for h in headings)):
            # Two views back into the same previous junction at a dead end
            # are not a new child junction on the travelled branch.
            return self.current_node_id
        is_junction=len(headings)>=3
        is_turn=(len(headings)==2 and
                 1.1<=abs(angle_delta(*headings))<=2.3)
        if not self.nodes:
            node=self._new_node(physical_xy,map_xy,now)
        elif is_junction or is_turn:
            node=self._matching_node(physical_xy,headings,line_is_free)
            if node is None:
                current=self.nodes[self.current_node_id]
                if (distance(physical_xy,current['physical_xy'])<self.node_radius and
                    line_is_free(current['physical_xy'],physical_xy)):
                    node=current
                else:
                    node=self._new_node(physical_xy,map_xy,now)
            gate=None if incoming is None else incoming.get('physical_gate')
            forward=(gate is not None and
                (node['physical_xy'][0]-gate[0])*math.cos(incoming['heading'])+
                (node['physical_xy'][1]-gate[1])*math.sin(incoming['heading'])>=0)
            if incoming is not None and incoming['node_id']!=node['id'] and forward:
                incoming['destination_node_id']=node['id']
                if node.get('space_id') is not None:
                    incoming['destination_space_id']=node['space_id']
            self.current_node_id=node['id']
        else:
            # Corridor observations refine existing directions without
            # creating an endless chain of nodes at every robot pose.
            node=self.nodes[self.current_node_id]
            if distance(physical_xy,node['physical_xy'])>self.node_radius:
                return self.current_node_id
        node['map_xy']=list(map_xy) if distance(physical_xy,node['physical_xy'])<0.1 else node['map_xy']
        node['last_seen']=now
        node['junction']=bool(node.get('junction',False) or is_junction or is_turn)
        separated = all(abs(angle_delta(a,b))>=0.9
                        for i,a in enumerate(distinct_headings) for b in distinct_headings[i+1:])
        confirmed = (len(distinct_headings)>=3 and separated) or (len(distinct_headings)==2 and
                       1.1<=abs(angle_delta(*distinct_headings))<=2.1)
        if confirmed and not node.get('junction_confirmed',False):
            # The first distant view can merge east+north into one diagonal
            # lobe. Keep that evidence as history, then refine the node at the
            # actual separated view rather than freezing its early viewpoint.
            for old_id in node['branch_ids']:
                old=self.branches[old_id]
                old['state']='superseded'
            node['physical_xy']=list(physical_xy)
            node['map_xy']=list(map_xy)
            node['junction_confirmed']=True
        for proposal in proposals:
            heading=float(proposal['heading'])
            geometry=proposal.get('geometry')
            gate=proposal.get('physical_gate')
            # Direction and a local gate association are both required.
            matches=[self.branches[i] for i in node['branch_ids']
                     if self.branches[i]['state']!='superseded' and abs(angle_delta(self.branches[i]['heading'],heading))<=self.heading_tolerance
                     and (gate is None or self.branches[i]['physical_gate'] is None or
                          distance(gate,self.branches[i]['physical_gate'])<=2*self.node_radius)]
            if matches:
                branch=min(matches,key=lambda b:abs(angle_delta(b['heading'],heading)))
            else:
                branch_id=len(self.branches)+1
                branch=dict(id=branch_id,node_id=node['id'],heading=heading,
                    physical_gate=gate,map_geometry=None,state='pending',
                    destination_node_id=None,first_seen=now,last_seen=now,
                    observations=0,stable=False,semantic_score=None,
                    attempts=0,failures=0,last_failure=None)
                parent=self.branches.get(node.get('parent_branch_id'))
                if parent is not None and abs(angle_delta(heading,parent['heading']+math.pi))<self.heading_tolerance:
                    parent_node=self.nodes[parent['node_id']]
                    branch['destination_node_id']=parent_node['id']
                    branch['destination_space_id']=parent_node.get('space_id')
                    branch['connection']='return'
                    branch['state']='return'
                self.branches[branch_id]=branch
                node['branch_ids'].append(branch_id)
            if (branch['state'] in ('pending','retry') and
                    branch.get('authentication_state') not in ('pending','entering','turning','returning')):
                branch['heading']=heading
                if gate is not None:
                    branch['physical_gate']=gate
            branch['last_seen']=now
            branch['observations']+=1
            branch['stable']=branch['observations']>=2 and now-branch['first_seen']>=self.stable_seconds
            branch['map_epoch']=map_epoch
            branch['gp_variance']=proposal.get('variance')
            branch['reach']=proposal.get('reach')
            branch['angular_width']=proposal.get('angular_width')
            if geometry is not None:
                branch['map_geometry']=geometry
                branch['passability']=geometry['passability']
                if branch['physical_gate'] is None:
                    branch['physical_gate']=gate
            else:
                branch.setdefault('passability','unknown')
        return node['id']

    def continuation_branch(self):
        """Return an entered branch at the current node even after dispatch moved on."""
        active=self.branches.get(self.active_branch_id)
        if (active is not None and active['state']=='entered' and
                active['node_id']==self.current_node_id):
            return active
        return next((branch for branch in self.branches.values()
                     if branch['state']=='entered' and
                     branch['node_id']==self.current_node_id),None)

    def choose(self, scores, available_ids, tie_break=None):
        """Semantic score is primary among safe stable unresolved branches."""
        available=set(available_ids)
        candidates=[]
        for branch in self.branches.values():
            if branch['id'] not in available or not branch['stable']:
                continue
            if branch['state'] not in ('pending','retry') or branch['passability']!='open':
                continue
            score=scores.get(branch['id'])
            if score is None or not math.isfinite(score):
                continue
            branch['semantic_score']=float(score)
            candidates.append(branch)
        return max(candidates,key=lambda b:(b['semantic_score'],
                   (tie_break or {}).get(b['id'],0.),-b['id'])) if candidates else None

    def dispatch(self, branch_id, route_id, start_xy=None):
        branch=self.branches[branch_id]
        if self.active_branch_id!=branch_id:
            branch['attempts']+=1
        self.active_branch_id=branch_id
        previous=self.active_branch_id
        for other in self.branches.values():
            if other['id']!=branch_id and other['state']=='active':
                other['state']='retry'
        branch['state']='active'
        branch['route_id']=route_id
        if start_xy is not None and branch['physical_gate'] is not None:
            gate=branch['physical_gate']
            signed=((start_xy[0]-gate[0])*math.cos(branch['heading'])+
                    (start_xy[1]-gate[1])*math.sin(branch['heading']))
            branch['entry_source_proven']=bool(branch.get('entry_source_proven') or signed<0)
            branch['last_physical_pose']=list(start_xy)

    def arrived(self, branch_id, physical_xy, required_depth=0.35):
        branch=self.branches[branch_id]
        gate=branch['physical_gate']
        if gate is None:
            return False
        depth=((physical_xy[0]-gate[0])*math.cos(branch['heading'])+
               (physical_xy[1]-gate[1])*math.sin(branch['heading']))
        previous=branch.get('last_physical_pose')
        if previous is not None:
            prior_depth=((previous[0]-gate[0])*math.cos(branch['heading'])+
                         (previous[1]-gate[1])*math.sin(branch['heading']))
            if prior_depth<0<=depth:
                fraction=-prior_depth/max(1e-9,depth-prior_depth)
                crossing=(previous[0]+fraction*(physical_xy[0]-previous[0]),
                          previous[1]+fraction*(physical_xy[1]-previous[1]))
                lateral=abs(-(crossing[0]-gate[0])*math.sin(branch['heading'])+
                             (crossing[1]-gate[1])*math.cos(branch['heading']))
                width=(branch.get('map_geometry') or {}).get('width')
                if width is not None and lateral<=max(0.,width/2-required_depth):
                    branch['gate_crossed']=True
                    branch['crossing_xy']=list(crossing)
        branch['last_physical_pose']=list(physical_xy)
        if (depth<required_depth or not branch.get('entry_source_proven') or
            not branch.get('gate_crossed')):
            return False
        branch['state']='entered'
        branch['arrival_xy']=list(physical_xy)
        branch['destination_verified']=True
        known_node=self.nodes.get(branch.get('destination_node_id'))
        if known_node is not None and known_node.get('space_id') is not None:
            branch['destination_space_id']=known_node['space_id']
        if branch.get('destination_space_id') is None:
            space_id=len(self.spaces)+1
            self.spaces[space_id]=dict(id=space_id,parent_branch_id=branch_id,
                entry_xy=list(physical_xy),node_ids=([branch['destination_node_id']]
                    if branch['destination_node_id'] is not None else []),state='entered')
            branch['destination_space_id']=space_id
        destination=self.nodes.get(branch.get('destination_node_id'))
        if destination is not None:
            destination['space_id']=branch['destination_space_id']
        return True

    def fail(self, branch_id, reason):
        branch=self.branches[branch_id]
        branch['failures']+=1
        branch['last_failure']=str(reason)
        branch['state']='retry'
        if self.active_branch_id==branch_id:
            self.active_branch_id=None

    def reject_branch_authentication(self, branch_id, reason):
        """Retire a false doorway while preserving auditable parent/child links."""
        branch = self.branches.get(int(branch_id))
        if branch is None:
            return None
        branch["state"] = "authentication_failed"
        branch["authentication_state"] = "failed"
        branch["authentication_failure_reason"] = str(reason or "no_frontier_inside_doorway")
        branch["last_failure"] = branch["authentication_failure_reason"]
        branch["failures"] = int(branch.get("failures", 0)) + 1
        space = self.spaces.get(branch.get("destination_space_id"))
        if space is not None:
            space["state"] = "rejected_doorway_destination"
            space["rejected_by_branch_id"] = int(branch_id)
        child = self.nodes.get(branch.get("destination_node_id"))
        if child is not None:
            child["state"] = "rejected_doorway_destination"
            child["rejected_by_branch_id"] = int(branch_id)
        # Keep IDs in both directions for audit and stable SLAM de-duplication.
        return branch

    def snapshot(self):
        import copy
        return copy.deepcopy(dict(spaces=list(self.spaces.values()),nodes=list(self.nodes.values()),
             branches=list(self.branches.values()),current_node_id=self.current_node_id,
             active_branch_id=self.active_branch_id))
