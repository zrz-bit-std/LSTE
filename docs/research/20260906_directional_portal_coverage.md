# Directional Portal Coverage

## Research Problem

Raw frontier exploration treats every newly visible frontier cell as a new
task. Online SLAM can move the cell, split one room into several components, or
temporarily hide it. The resulting planner repeatedly sends the robot through
the same doorway and recreates local observation work that has already been
completed.

The architectural question is therefore not "which frontier has the highest
weight?" It is: "which persistent physical branch still lacks the evidence
required by the mission?"

## Method

`DirectionalBranchCoverage` is a ROS-free, event-driven ledger for that
question. A branch is identified by:

```text
(source_place_id, physical_branch_id)
```

`physical_branch_id` must come from a durable source such as `portal_id`; it is
never derived from the current frontier coordinate. The latest frontier
coordinate and map epoch are projections only.

Each branch has one stable Observation WorkItem lineage and an explicit state:

```text
open -> completed -> transit
                 \\-> open (only after contradictory evidence)
```

The state transitions are caused by typed evidence events, not by elapsed time,
distance thresholds, score weights, or detector confidence thresholds:

- `observe_frontier`: update the transient projection and reuse the same branch
  and WorkItem identity;
- `complete_branch`: record that the branch observation obligation is resolved;
- `mark_transit`: record a physically verified crossing and allow graph routing
  through the edge without creating an observation task;
- `record_evidence(..., contradictory=True)`: explicitly reopen the same
  WorkItem lineage when later evidence invalidates the earlier conclusion.

Ordinary map updates and ordinary positive evidence cannot reopen a completed
branch. Replayed event IDs are idempotent, which makes the ledger suitable for
event-log replay and benchmark evaluation.

## Why This Is Different From Frontier De-duplication

Distance de-duplication stores a geometric suppression radius. It cannot tell
whether two nearby points are the same doorway, whether a completed room is
being revisited for transit, or whether a changed SLAM projection is a new
physical branch. Directional Portal Coverage stores those semantics directly.

The existing Place/Portal/WorkItem graph remains the owner of physical place
and portal identity. The new ledger is a coverage view over that graph. The
intended production key is:

```text
DirectionalBranchKey(source_place_id=current_place_id,
                     physical_branch_id="portal:<portal_id>")
```

This keeps the ROS boundary thin: the node supplies durable IDs and evidence
events, while the pure module decides whether an observation obligation still
exists.

## Relationship To Recent Work

The design takes the persistent-direction idea from RayFronts, R2F, and
DRIVE-Nav, and combines it with the region/opening/pathway separation used by
GRID-FAST and the semantic-region completion policy in SRAAE. Its project-
specific contribution is the Evidence Contract: a directional branch is not
considered traversable until the corresponding Portal evidence and physical
crossing transaction are complete.

## Planned Integration

Integration should be incremental and evidence-gated:

1. Reconcile every certified Portal into one directional branch key.
2. Emit `observe_frontier` only as a projection update during snapshot
   reconciliation.
3. Let the graph planner query `pending_work_items()` when choosing an
   observation action.
4. Let the Portal crossing transaction emit `mark_transit` only after its
   source-side, destination-side, and odometry crossing facts are committed.
5. Keep completed branches available to graph search as transit edges, but
   exclude them from local observation candidate generation.
6. Use contradictory evidence as the sole reopen path and report its cause in
   the experiment log.

No new ROS topic or controller parameter is required for the pure model.

## Test Contract

The focused tests in
`src/lste_topo_access/test/test_directional_branch_coverage.py` verify:

- arbitrary SLAM projection changes preserve branch and WorkItem identity;
- completed branches do not recreate WorkItems;
- transit edges remain routeable without local observation work;
- only explicit contradictory evidence reopens a branch;
- replayed events are idempotent; and
- distinct physical branch IDs remain distinct even at the same coordinate.

The benchmark should compare ordinary frontier, distance-deduplicated frontier,
Place/Portal/WorkItem, and Directional Portal Coverage using repeated doorway
entry, repeated WorkItem dispatch, completed-place reopen count, coverage,
semantic-goal success, path length, completion time, collision/failure rate,
and command smoothness.
