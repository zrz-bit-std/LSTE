"""Depth-first Place traversal; physical Portal commits own every stack change."""


class PlaceDfsTraversal:
    def __init__(self):
        self.nodes = {}
        self.stack = []

    def bootstrap(self, place_id):
        place_id = int(place_id)
        if not self.stack:
            self.stack.append(place_id)
            self.nodes.setdefault(place_id, dict(place_id=place_id, parent_id=None,
                portal_id=None, children=[], state="exploring"))
        return self.stack[-1] == place_id

    def enter(self, source_id, destination_id, portal_id):
        source_id, destination_id, portal_id = map(int, (source_id, destination_id, portal_id))
        if not self.bootstrap(source_id) or destination_id in self.stack:
            return False
        prior = self.nodes.get(destination_id)
        if prior is not None and (prior["parent_id"] != source_id or prior["portal_id"] != portal_id):
            return False
        if prior is not None and prior["state"] == "completed":
            return False
        self.nodes.setdefault(destination_id, dict(place_id=destination_id,
            parent_id=source_id, portal_id=portal_id, children=[], state="exploring"))
        if destination_id not in self.nodes[source_id]["children"]:
            self.nodes[source_id]["children"].append(destination_id)
        self.nodes[source_id]["state"] = "suspended"
        self.stack.append(destination_id)
        return True

    def update_projection_recovery(self, place_id, valid, now, pose_xy=None,
                                   invalid_budget=30., return_budget=60.,
                                   route_id=0):
        """Bound invalid-map waits; retreat preserves all unfinished obligations."""
        place_id=int(place_id)
        if len(self.stack)<2 or self.stack[-1]!=place_id:
            return None
        node=self.nodes[place_id]
        if node['state']=='returning':
            # The budget must measure the *return drive*, not the time spent
            # waiting for one.  A new route id means the return was just
            # dispatched, so restart the clock: run 20261102_174505 spent 60 s
            # in 'returning' while it closed a portal transaction, dispatched
            # the return at sim 1657.5, and was declared unrecoverable 0.1 s
            # later.
            if route_id and int(node.get('return_route_id', 0) or 0) != int(route_id):
                node['return_route_id']=int(route_id)
                node['return_progress_at']=float(now)
                node['return_progress_xy']=(
                    None if pose_xy is None else list(pose_xy))
            # This physical return obligation survives graph/map transactions.
            since=node.setdefault('return_progress_at',float(now))
            prior=node.get('return_progress_xy')
            if pose_xy is not None and (prior is None or
                    (pose_xy[0]-prior[0])**2+(pose_xy[1]-prior[1])**2>=.1**2):
                node['return_progress_xy']=list(pose_xy)
                node['return_progress_at']=float(now)
                since=float(now)
            if float(now)-since>=float(return_budget):
                node.update(state='recovery_blocked',
                            failure_reason='dfs_return_no_physical_progress')
                return 'return_blocked'
            return None
        if node['state']=='recovery_returning':
            prior=node.get('recovery_progress_xy')
            if pose_xy is not None and (prior is None or
                    (pose_xy[0]-prior[0])**2+(pose_xy[1]-prior[1])**2>=.1**2):
                node['recovery_progress_xy']=list(pose_xy)
                node['recovery_progress_at']=float(now)
            if float(now)-node['recovery_progress_at']>=float(return_budget):
                node['state']='recovery_blocked'
                return 'blocked'
            return None
        if node['state']!='exploring':
            return None
        if valid:
            node.pop('projection_invalid_since',None)
            return None
        since=node.setdefault('projection_invalid_since',float(now))
        if float(now)-since<float(invalid_budget):
            return None
        node.update(state='recovery_returning',failure_reason='place_projection_invalid_budget_exhausted',
                    recovery_progress_at=float(now),recovery_progress_xy=None if pose_xy is None else list(pose_xy))
        return 'started'

    def ready_to_return(self, place_id, unresolved_work=(), pending_portals=(),
                        frontier_count=0, projection_complete=False):
        place_id = int(place_id)
        if not self.stack or self.stack[-1] != place_id or len(self.stack) < 2:
            return False
        node = self.nodes[place_id]
        if (node['state'] in ('recovery_returning','recovery_blocked','failed')
                or unresolved_work or pending_portals or int(frontier_count) > 0
                or not projection_complete
                or any(self.nodes[child]["state"] != "completed" for child in node["children"])):
            return False
        node["state"] = "returning"
        return True

    def return_to_parent(self, source_id, destination_id, portal_id):
        source_id, destination_id, portal_id = map(int, (source_id, destination_id, portal_id))
        if len(self.stack) < 2 or self.stack[-2:] != [destination_id, source_id]:
            return False
        node = self.nodes[source_id]
        if node["state"] not in ("returning", "recovery_returning") or node["portal_id"] != portal_id:
            return False
        node["state"] = "failed" if node["state"] == "recovery_returning" else "completed"
        self.stack.pop()
        self.nodes[destination_id]["state"] = "exploring"
        return True

    def snapshot(self):
        return {"stack": list(self.stack), "nodes": [dict(node, children=list(node["children"]))
                for _, node in sorted(self.nodes.items())]}
