# Event-Driven Evidence Graph

Date: 2026-09-06  
Status: implemented as a ROS-free projection plus an execution wake boundary;
Level 4 completion is not claimed.

## Research motivation

The former control loop allowed several short-lived observations to compete for
the same navigation decision:

```text
detector frame -> target coordinate -> action replacement
map frontier   -> new coordinate  -> action replacement
route failure  -> recovery point   -> action replacement
```

That representation loses the reason a decision was made. A missed detector
frame can look like a new object, and a structural Portal candidate can look
like an executable route. More thresholds do not repair this loss of meaning.

The new architecture separates two rates:

```text
fast reactive layer: SLAM, costmap, Navfn, TEB
    execute the currently committed route and publish status

slow deliberative layer: Place/Portal/WorkItem/Object evidence graph
    wakes only on a new durable fact or an action terminal
```

The slow layer is not an additional controller. It is a replayable projection
of the existing lifecycle events, so the same decision can be inspected from a
timestamped log after a run.

## Relation to recent work

The design is informed by the following recent open research reports:

- **AECNav**, [arXiv:2608.10817](https://arxiv.org/abs/2608.10817), August 2026:
  evidence-gated perception, consolidated positive/negative evidence, and
  active information acquisition.
- **Hierarchical Fast-Slow ReAct Agent**,
  [arXiv:2608.09816](https://arxiv.org/abs/2608.09816), August 2026:
  a reactive value-map layer continues operating while a slower deliberator is
  woken by structural events and retrieves persistent pose-tagged memory.
- **STEGNav**, [arXiv:2608.28279](https://arxiv.org/abs/2608.28279), August 2026:
  spatio-temporal event graphs combine target candidates, frontiers and
  verified navigation outcomes.
- **ObsGraph**, [arXiv:2606.24068](https://arxiv.org/abs/2606.24068), June 2026:
  room, view and object observations form an explicit graph whose next action
  is chosen from an evidence gap.

These papers motivate the separation, but LSTE keeps its own boundary:
semantic evidence may order an already legal graph action; only physical
Portal evidence can create a Place, and only Navfn/TEB can execute motion.

## Implementation

`src/lste_topo_access/scripts/global_frontier_event_graph.py` defines
`EvidenceEventGraph`. It accepts the status event name and fields, then stores:

- a monotonically increasing event sequence;
- a structural revision number;
- active and completed route identities;
- compact Place, Portal, WorkItem and target states;
- a bounded normalized event history;
- a one-shot `wake_pending` decision for the slow layer.

The graph class is ROS-free. `GlobalFrontierReportingMixin.publish_status()`
feeds each status event into it and includes both `graph_event` and
`event_graph` in the same JSON status message. Existing logs therefore contain
the graph sequence without a second runtime file or a second source of truth.

Latched or repeated status messages are de-duplicated by durable identity
(`route_id`, `place_id`, `work_item_id`, `portal_id`, request id and lifecycle
state). Coordinates are intentionally absent from the identity key. A new
SLAM projection cannot create a new deliberation event merely because a point
moved in `map`.

The graph is an audit/scheduling layer, not a hidden policy. It does not change
frontier scores, TEB gains, detector confidence, speed, timers or SLAM
parameters. The existing graph executive remains responsible for hard action
legality and Navfn/TEB remains responsible for reachability and safety.

## Execution Wake Boundary

The event graph records durable meaning, but recording alone does not change
the old timer behavior. The main `place_portal_workitem` and
`place_portal_workitem_strict` arms now also construct
`DecisionWakeScheduler` from
`src/lste_topo_access/scripts/global_frontier_decision_wake.py`. It is a small
ROS-free queue between the graph and the existing planning timer:

```text
map/costmap/semantic fact/route terminal
                 |
                 v
       DecisionWakeScheduler
                 |
       one graph decision claim
                 |
             Navfn/TEB
```

Repeated map publications are fingerprinted without their ROS sequence or
timestamp. A changed map, a changed full costmap, a newly available pose, a
navigation-readiness transition, semantic evidence, or a route terminal adds
one coalesced reason. A live route blocks claims, so a new semantic/map event
cannot preempt the controller. Releasing or terminating the route exposes the
queued wake; `task_done` halts it until the mission is explicitly resumed.

The scheduler has no planning period, distance, confidence, score, or retry
parameter. It does not choose a candidate and does not issue a velocity
command. The fast execution layer still runs at the existing ROS rates, while
the slow graph layer is no longer rebuilt merely because the timer ticked.
`frontier`, `frontier_distance_dedup`, `place_portal`, and
`place_portal_workitem_legacy_rank` retain their timer-driven behavior as
control arms. The scheduler's effect is therefore measurable as an explicit
architectural ablation, not a hidden baseline change.

## Fast-Slow Boundary Repair

Persistent execution also calls a short-horizon prefetch while the current
route is still active.  That prefetch now passes `graph_route_planning=False`:
it may propose a geometry-only successor, but it cannot update the slow graph
plan, its transaction, or its durable action identity.  The graph planner is
invoked only after a terminal, when the next action can be selected from a
complete reconciled snapshot.  This prevents an in-flight Portal route from
being overwritten by a transient local frontier observation.

## Related structural fixes

This increment also closes three identity leaks exposed by code audit:

1. Durable Portal probe recovery now uses one captured `map <-> odom` projection
   in the correct direction. A missing projection cannot silently leave the
   graph blocked after a SLAM update.
2. Recovery distinguishes `portal_candidates_seen` from
   `portal_executable_candidates`. A candidate rejected by costmap, Navfn,
   direction, Place ownership or Portal admission cannot request graph transit.
3. A locked target track uses semantic identity (`target_track_identity.py`)
   rather than the current frame's maximum score. Synonyms such as `mug` and
   `cup` are compatible; a conflicting color or object class is not. Legacy
   all-zero detector headers receive a callback receipt identity so they do not
   collapse into one frame.

Target promotion also now treats image-center displacement as diagnostic rather
than identity evidence. A small object naturally moves in the image as the
base changes pose; promotion is based on compatible semantics, distinct
physical viewpoints, and a forward intersection of source-stamped rays. This
is the lightweight 2-D counterpart of AECNav's evidence consolidation and
avoids another pixel-distance tuning loop.

These are representation fixes, not new tuning knobs.

## Typed Evidence Boundary

The latest development run exposed a subtle identity leak: a durable Portal
hypothesis (`portal.id`) was sometimes sent to the source-side probe adapter,
which expects a PortalProbe identity (`probe.id`).  These are different graph
objects and must not share an integer namespace in the action contract:

```text
PortalHypothesis (physical gate fact, possibly unbound)
        |
        +-- bound by portal_id
        v
PortalProbe (source-side information obligation)
        |
        +-- reprojected into the current map snapshot
        v
Navfn/TEB route candidate
```

`global_frontier_graph_route_planner.py` now performs this conversion before
returning a ready action.  A bound probe remains eligible even when its
frontier cell is absent from the current SLAM snapshot; geometry visibility is
left to the rehydration adapter.  `global_frontier_candidate_lifecycle.py`
also prevents a probe-owned WorkItem from leaking back into the ordinary local
WorkItem pool.  This is a type/ownership invariant, not a score or threshold.

When a durable probe cannot be materialized in the current map, the ledger
uses the explicit `awaiting_projection` state in
`global_frontier_portal_probe_ledger.py`:

```text
pending/source_arrived -> awaiting_projection -> (new physical observation)
                                             -> pending/source_arrived
```

It is distinct from `failed` (route outcome) and `rejected` (negative physical
evidence).  The planner therefore does not replay one impossible viewpoint on
every timer tick, and a later observation of the same physical gate restores
the original phase without minting a new probe.

The outgoing Portal transaction uses the same ownership rule.  In
`global_frontier_observation_departure.py`, a structural departure is prepared
against `current_physical_place_id`; the transient SLAM component is retained
only as geometric evidence.  This prevents a temporary component merge from
assigning a new Portal route to an old source Place.

The Level 4 development run after these changes produced the following
causal sequence in
`runtime/office_building_benchmark/logs/20260906_194322/`:

```text
Place 1 --Portal 1--> Place 2 --Portal 2--> Place 3
```

The run is evidence that the transaction can advance through multiple Places,
not a completion claim.  The benchmark still has to measure repeated entries,
WorkItem redispatches, target success, coverage, path length, failures and
smoothness across repeated trials.

## Research Search Update

The current architecture is consistent with recent open work found during the
September 2026 literature scan:

- **SCOUT**, [arXiv:2606.06721](https://arxiv.org/abs/2606.06721), uses
  uncertainty-guided semantic coverage to avoid repeatedly observing saturated
  regions.  LSTE's durable WorkItem/Place completion is the 2-D, ROS-compatible
  analogue, while Portal certification remains an LSTE-specific physical gate.
- **SENSEI**, [arXiv:2503.01584](https://arxiv.org/abs/2503.01584), and
  **SeGuE**, [arXiv:2504.03629](https://arxiv.org/abs/2504.03629), separate
  semantic exploration decisions from metric execution.  LSTE keeps Navfn/TEB
  as the metric executor and makes the semantic decision a typed graph action.
- **Open-vocabulary semantic exploration**,
  [arXiv:2509.19851](https://arxiv.org/abs/2509.19851), reinforces the need to
  preserve object evidence across views and map updates; LSTE stores that
  evidence by task version and physical Place rather than by image coordinates.

These references motivate the representation boundary only.  No external
model is treated as a runtime dependency, and no claim is made that LSTE
matches the reported benchmark numbers.

## Durable Route-Lease Reconciliation

The runtime also makes route preemption an explicit transaction. A replan is
not allowed to clear the active route fields while leaving a durable
WorkItem/Portal Attempt active. `DurableActionLease` snapshots the route and
its identities, then `decide_replan_lease` applies one structural rule:

```text
active route with no physical crossing -> preempt, settle Attempts, then replan
physical Portal crossing in progress   -> defer until arrival commit
no durable route                        -> replan normally
```

This closes a failure mode that is easy to miss in a fast/slow system: the
semantic layer can legitimately request a new viewpoint while the previous
geometry action is still executing, but the graph must receive an explicit
negative route outcome before it can issue another Attempt. The lease boundary
is ROS-free in `global_frontier_durable_action_lease.py` and is applied by
`global_frontier_durable_lease_lifecycle.py`. It is therefore replayable and
does not depend on a timeout, distance threshold, or controller tuning.

`PLAN_BLOCKED` is treated as the corresponding planning barrier. A blocked
graph plan stops candidate materialization, and identical blocked signatures
are reported once until a new structural fact or terminal changes the plan.
Reprojected Portal probes retain the explicit `probe_portal` action even when
the current SLAM snapshot temporarily lacks a complete Place label.

The semantic policy has the same boundary.  Generic office labels are a
one-pass context hint; after the first evidence-bearing observation of a Place,
`monitor/chair/desk` evidence cannot keep the robot there indefinitely.  A
task-specific target observation remains a durable exception and is still
owned by the target WorkItem gate.  This prevents common office context from
starving expansion toward a target room.

## Verification boundary

The following tests cover the new contracts without ROS or Gazebo:

```bash
python3 -m pytest -q \
  src/lste_topo_access/test/test_global_frontier_event_graph.py \
  src/lste_topo_access/test/test_target_track_identity.py \
  src/lste_topo_access/test/test_target_detection_contract.py \
  src/lste_topo_access/test/test_portal_egress.py \
  src/lste_topo_access/test/test_completion_gate_integration.py
```

Unit tests establish identity and transition invariants only. They do not show
that the robot completes Level 4. The required next evidence is a clean run on
the same seeded world, followed by paired baseline/ablation trials and the
metrics in `20260827_office_building_benchmark_plan.md`.
