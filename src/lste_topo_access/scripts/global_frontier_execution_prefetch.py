"""Successor prefetching for a healthy active frontier route."""

import math

import rospy


class GlobalFrontierExecutionPrefetchMixin:
    """Prepare a continuous next action without replacing the active lease."""

    def _transition_topology_cache_key(self, message, x, y, frontier_free):
        """Return the durable boundary for one endpoint-rooted transition BFS."""
        info = message.info
        origin = info.origin.position
        return (
            int(getattr(self, "active_route_id", 0) or 0),
            round(float(x), 3),
            round(float(y), 3),
            tuple(int(value) for value in frontier_free.shape),
            round(float(info.resolution), 6),
            round(float(origin.x), 3),
            round(float(origin.y), 3),
        )

    def _endpoint_transition_topology(
        self, message, frontier_free, row, col, x, y,
    ):
        """Build/reuse the slow endpoint-rooted topology advisory.

        The returned BFS is used only to score the next frontier's entry
        tangent.  Candidate reachability is still checked against the current
        snapshot and Navfn, so retaining this advisory across non-structural
        SLAM updates cannot authorize a stale command.
        """
        key = self._transition_topology_cache_key(
            message, x, y, frontier_free,
        )
        cached = getattr(self, "active_transition_topology_cache", None)
        if isinstance(cached, dict) and cached.get("key") == key:
            return cached.get("steps"), cached.get("seed"), True
        transition_seed = self.nearest_seed(
            frontier_free,
            row,
            col,
            max(1, int(0.8 / message.info.resolution)),
        )
        if transition_seed is None:
            return None, None, False
        transition_steps = self.bfs(
            frontier_free,
            transition_seed,
            getattr(self, "planning_should_preempt", None),
        )
        if transition_steps is None:
            return None, transition_seed, False
        self.active_transition_topology_cache = {
            "key": key,
            "seed": transition_seed,
            "steps": transition_steps,
        }
        event_graph = getattr(self, "event_graph", None)
        structural_revision = getattr(event_graph, "structural_revision", None)
        self.publish_status(
            "transition_topology_cached",
            route_id=int(getattr(self, "active_route_id", 0) or 0),
            endpoint=[round(float(x), 3), round(float(y), 3)],
            grid_shape=list(key[3]),
            map_resolution=key[4],
            map_origin=[key[5], key[6]],
            structural_revision=structural_revision,
        )
        rospy.loginfo(
            "Global frontier cached endpoint-rooted transition topology "
            "route_id=%d endpoint=(%.2f,%.2f) structural_revision=%s",
            int(getattr(self, "active_route_id", 0) or 0),
            x,
            y,
            "unknown" if structural_revision is None else str(structural_revision),
        )
        return transition_steps, transition_seed, False

    def prefetch_active_route_successor(
        self, message, strict_steps, strict_free, frontier, frontier_free,
        unknown, occupied, components, validation, route_steps, seed,
        robot_map, robot_yaw_map, now, row, col, x, y, distance,
    ):
        """Cache one executable successor without replacing the active lease."""
        if getattr(self, "planning_should_preempt", lambda: False)():
            return
        if distance > self.prefetch_distance:
            return

        # Select a real transition ``robot -> active endpoint -> successor``.
        # Rooting BFS at the active endpoint gives the selector the successor's
        # entry tangent. The result is cached per active route lease;
        # Navfn validation below remains rooted at the live robot pose.
        transition_steps, transition_seed, cache_hit = (
            self._endpoint_transition_topology(
                message, frontier_free, row, col, x, y,
            )
            if getattr(self, "persistent_execution", False)
            else (None, None, False)
        )
        if getattr(self, "planning_should_preempt", lambda: False)():
            return
        _, active_terminal_heading = self.route_headings(
            message,
            route_steps,
            seed,
            (row, col),
            robot_map,
        )
        if transition_steps is not None and active_terminal_heading is not None:
            prefetch_steps = transition_steps
            prefetch_seed = transition_seed
            prefetch_heading = active_terminal_heading
            prefetch_anchor = (x, y)
            transition_basis = (
                "active_endpoint_bfs_tangent_cached"
                if cache_hit else "active_endpoint_bfs_tangent"
            )
            score_from_transition = True
        else:
            # A SLAM update can temporarily reassociate the endpoint into an
            # occupied cell. Keep a valid robot-rooted fallback rather than
            # stopping map coverage; the status records the degraded route.
            prefetch_steps = route_steps
            prefetch_seed = seed
            prefetch_heading = robot_yaw_map
            prefetch_anchor = robot_map
            transition_basis = "robot_bfs_tangent_fallback"
            score_from_transition = False
            rospy.logwarn_throttle(
                3.0,
                "Global frontier successor transition topology unavailable; "
                "using robot-rooted fallback route_id=%d",
                self.active_route_id,
            )

        envelopes = self.successor_transition_envelopes(score_from_transition)
        attempted_limits = set()
        for heading_limit, preference in envelopes:
            if getattr(self, "planning_should_preempt", lambda: False)():
                return
            limit_key = round(float(heading_limit), 6)
            if limit_key in attempted_limits:
                continue
            attempted_limits.add(limit_key)
            selected = self.prefetch_next_frontier(
                message,
                prefetch_steps,
                frontier,
                unknown,
                occupied,
                now,
                (x, y),
                robot_map,
                validation=validation,
                heading_reference=prefetch_heading,
                route_seed=prefetch_seed,
                preferred_steps=strict_steps,
                preferred_mask=strict_free,
                route_anchor_xy=prefetch_anchor,
                transition_basis=transition_basis,
                score_path_from_steps=score_from_transition,
                max_heading_delta=heading_limit,
                allow_heading_fallback=False,
                transition_preference=preference,
                components=components,
            )
            if selected or self.frontier_validation_pending:
                return

    def successor_transition_envelopes(self, score_from_transition):
        """Return heading envelopes in route-continuity preference order."""
        if score_from_transition:
            return [
                (self.successor_smooth_heading_limit, "smooth_preferred"),
                (self.successor_curve_heading_limit, "curve_preferred"),
                (
                    self.heading_hard_limit,
                    "terminal_reorientation_preferred",
                ),
                (math.pi, "terminal_reorientation_unbounded"),
            ]
        return [
            (self.heading_hard_limit, "robot_fallback_preferred"),
            (math.pi, "robot_fallback_unbounded"),
        ]

    def try_early_prefetch_promotion(
        self, message, route_steps, seed, robot_map, now, validation,
        active_component, row, col, x, y, distance,
    ):
        """Keep endpoint observation atomic before selecting a successor.

        A prefix-continuation proof describes geometry, but not observation
        ownership: the scan at the pending endpoint can reveal that the whole
        branch belongs to the room just observed.  Replacing the active action
        early therefore bypasses the only reliable place-lifecycle edge.
        Successor selection is deliberately deferred to the endpoint terminal,
        where it runs against one fresh SLAM snapshot.
        """
        return None

    def active_cell_from_promoted_frontier(
        self, validation, route_steps, resolution, promoted,
    ):
        """Build the selector's tuple for a verified streamed successor."""
        row, col, x, y = promoted
        distance = self.candidate_costmap_distance(validation, x, y)
        if distance is None:
            distance = route_steps[row, col] * resolution
        return row, col, x, y, distance, 0.0, 0.0, 0.0
