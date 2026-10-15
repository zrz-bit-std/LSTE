# Place Phase and Portal Transaction Architecture

Date: 2026-09-06

Status: executable graph-state slice with branch-first suspension; no complete
benchmark conclusion yet.

## Research question

Greedy frontier exploration can enter a newly discovered room and immediately
select a doorway leading back to a previously seen room.  A second failure mode
is that a route failure leaves a prefetched successor alive; the next planning
cycle then promotes a goal whose route proof and WorkItem ownership came from a
stale map snapshot.  Both behaviors create repeated rooms, long dwell time and
oscillating graph state. They are representation/lifecycle errors, not missing
TEB gains.

The current research slice asks:

> Can a small, typed Place phase plus a non-preemptible Portal transaction
> prevent illegal graph actions while leaving Navfn/TEB and all controller
> parameters unchanged?

## External architectural references

- **STGPlanner** ([arXiv:2412.13664](https://arxiv.org/abs/2412.13664),
  [code](https://github.com/Haochen-Niu/STGPlanner)) keeps a current topological
  branch and switches only after the branch is complete.  We retain the
  branch-first principle, but represent a branch with the existing physical
  Place/Portal identities instead of copying its implementation.
- **GRID-FAST** ([arXiv:2406.11635](https://arxiv.org/abs/2406.11635),
  [code](https://github.com/LTU-RAI/GRID-FAST)) separates regions, openings and
  unexplored pathways.  This supports treating an opening as a Portal
  candidate rather than as an ordinary frontier endpoint.
- **Passage-aware structural mapping**
  ([arXiv:2604.24707](https://arxiv.org/abs/2604.24707),
  [code](https://github.com/snt-arg/visual_sgraphs/tree/doorway_integration))
  requires geometric opening evidence and signed traversal evidence before a
  passage is accepted.  LSTE uses the same boundary in 2D odometry: a route
  terminal is not by itself a crossing proof.
- **R2F/RayFronts** ([R2F](https://arxiv.org/abs/2603.08475),
  [RayFronts](https://arxiv.org/abs/2504.06994)) bind unknown-space evidence to
  a direction.  LSTE's directional WorkItem/Portal probe ledger is the small
  ROS-1-compatible version of that idea.

These references motivate the architecture but are not runtime dependencies.

## Durable state model

```text
Physical Place
  |-- ordinary ObservationWorkItem (local room evidence)
  |-- Portal probe (outward doorway evidence)
  +-- PortalTransaction (one directed physical edge execution)
```

The occupancy grid and structural component labels are snapshot-local.  A
Place, WorkItem and Portal keep their physical identity across SLAM updates.

### Place phase

`global_frontier_place_progress.py` derives a phase from facts only:

```text
no Place                  -> bootstrap
Place not yet observed    -> observe
ordinary WorkItem pending -> observe
ordinary WorkItems clear  -> exit
dormant Place             -> transit-only
```

The full branch-first method adds a fifth state transition for a source Place:

```text
observed Place + novel certified Portal + residual WorkItems
    -> suspended
    -> open (only when that Place is deliberately selected for local work)
```

`suspended` is different from `dormant`. A dormant Place is complete and may
only be used as transit; a suspended Place has already supplied valid local
evidence, but its remaining WorkItems are still resumable. This lets the
planner pursue a newly exposed room without deleting the unfinished branch or
mistaking a return to that Place for a new room. A branch-first crossing is
therefore admitted by the graph fact "certified edge leads to an unobserved
Place", not by a dwell timeout or a larger frontier score.

A source-side Portal probe is deliberately not counted as ordinary local room
work.  It remains a durable outward obligation and can be selected when no
certified exit exists, but it does not reopen a completed observation phase.
An unknown cell in the current SLAM mask is never a progress certificate for a
covered-room re-entry.

### Portal transaction

`global_frontier_portal_transaction.py` makes the directed edge lifecycle
explicit:

```text
source_probe
  -> throat
  -> destination_standoff
  -> crossing_verified
  -> place_commit
  -> idle
```

The only alternate path is an explicit `retry` from the same physical Portal
after a source-side egress, or an explicit `aborted` terminal when no legal
recovery exists. A normal frontier or semantic replan cannot preempt an active
transaction. Replans are queued and replayed after the transaction commits.

## Runtime rules implemented in this slice

1. A certified Portal cannot be selected while the source Place has not yet
   supplied its first observation.
2. A certified Portal cannot bypass an unresolved ordinary WorkItem when it is
   covered transit. The full method may suspend those WorkItems for a novel
   edge, and the departure transaction also protects them on recovery or
   covered-transit exits; the WorkItems remain owned by the source Place and
   can be resumed later.
3. A WorkItem attached to a Portal probe is accounted for by the probe ledger,
   not by the local observation counter.
4. Covered-to-covered transit is admitted from durable graph progress only.
   Raw `unknown` adjacency is not enough.
5. A failed route invalidates its prefetched successor before another selector
   pass. Prefetch is an optimization, not a second source of route ownership.
6. A source-arrived probe can request at most one destination viewpoint per
   SLAM evidence epoch. A new map epoch is the explicit evidence that reopens
   the attempt; elapsed time alone cannot create an infinite retry loop.
7. A Portal probe endpoint must stay within the existing doorway observation
   neighborhood. If no safe local endpoint exists, the probe remains pending
   rather than becoming a far-away frontier goal.
8. Every durable entry Portal stores `source_place_id` as well as its
   destination Place. Reverse egress therefore follows the directed graph
   edge instead of returning a self-loop on the destination Place.
9. A failed physical edge becomes negative Portal evidence. One explicit
   same-edge retry may use the existing recovery transaction; after that, a
   failed edge cannot be silently selected again.
10. Portal route phases are logged in every status payload, including the
    transaction id, Portal id, source Place and current Place phase.
11. A departure with unresolved local WorkItems commits the source Place as
    `suspended`, while a completed Place with no local work uses `dormant`;
    these states are never conflated.

No TEB speed, obstacle weight, SLAM parameter, detector threshold, timeout or
new numeric score coefficient was changed for these rules.

## Evidence from Level 4 development runs

Run `20260906_073446` showed the former failure pattern:

- Place 2 was entered and immediately issued a reverse Portal action before a
  local observation;
- covered transit used current `unknown` adjacency as progress;
- a Portal edge deadline then left the bridge at an active/geometry mismatch.

The clean run `20260906_080512` after the first phase gate recorded:

- `total_room_reentries = 0` during the captured interval;
- `move_base_aborts = 0`, `collision_events = 0`;
- the route sequence remained in one Place until local WorkItems were handled;
- Portal probes were represented as `source_arrived` evidence rather than
  silently creating new Places.

This is development evidence, not a benchmark claim.  The run did not reach
the semantic target and later exposed a separate local-route stall, which must
be solved and measured before claiming success.

## Regression tests

The pure contracts are covered without ROS/Gazebo:

```bash
python3 -m pytest -q src/lste_topo_access/test
# 416 passed (2026-09-06)
catkin_make --pkg lste_topo_access
```

Important tests include:

- unobserved Place cannot issue an immediate Portal crossing;
- ordinary local WorkItem completion is independent of target claims;
- Portal-owned WorkItems are excluded from local observation counts;
- unknown cells alone do not justify covered transit;
- active Portal transactions reject ordinary preemption and preserve identity
  across retries;
- failed routes clear prefetched successors.

## Next architectural increment

The next step is active target-observation work and a replay reducer for the
semantic evidence path. The branch-first policy itself is already an explicit
experiment arm, keyed by the durable Place/Portal graph rather than a
floating-point frontier score. Compare it with:

1. ordinary frontier exploration;
2. distance-deduplicated frontier;
3. Place/Portal/WorkItem without branch ordering;
4. Place/Portal/WorkItem with strict local-work ordering;
5. Place/Portal/WorkItem with branch-first suspension and Pareto probe value.

All arms must use the same world, controller, SLAM settings and task seed. The
comparison will report room re-entries, maximum effective dwell per Place,
duplicate WorkItem dispatches, coverage, target success, path length, time,
collisions/failures and motion smoothness.
