# Durable Portal Action Compiler

Date: 2026-09-06  
Status: implemented as a narrow architecture slice; Level 4 completion is not
claimed.

## Research question

How can an explorer keep moving toward a physically certified opening when
SLAM has removed the frontier cell that originally exposed that opening?

The failure is not a missing TEB parameter. It is a type mismatch between a
long-lived graph obligation and a short-lived geometric candidate:

```text
Portal evidence -> durable graph action -> (missing frontier projection) -> no route
```

This is the same boundary highlighted by recent topology- and evidence-driven
exploration work, but the implementation here is specific to LSTE's 2-D ROS 1
stack.

## External design evidence

- **STGPlanner**, [arXiv:2412.13664](https://arxiv.org/abs/2412.13664), uses a
  sparse topological structure to keep exploration intent stable while local
  geometry changes.
- **GRID-FAST**, [repository](https://github.com/LTU-RAI/GRID-FAST), separates
  regions, openings, pathways and dead ends instead of treating every frontier
  cell as the same object.
- **Robotic Exploration through Semantic Topometric Mapping**,
  [arXiv:2406.18381](https://arxiv.org/abs/2406.18381), represents
  intersections and pathways as persistent exploration structure.
- **SCOUT**, [arXiv:2606.06721](https://arxiv.org/abs/2606.06721), and
  **AECNav**, [arXiv:2608.10817](https://arxiv.org/abs/2608.10817), use fused
  evidence and active observation rather than reacting to one transient
  perception frame.
- **RayFronts**, [repository](https://github.com/RayFronts/RayFronts), and
  **R2F**, [repository](https://github.com/Lab-RoCoCo-Sapienza/r2f), preserve
  directional unknown-space hypotheses rather than raw frontier coordinates.

The common architectural lesson is to make the persistent object and its
execution contract explicit. LSTE retains Navfn and TEB as metric executors;
the graph layer does not issue velocity commands.

## LSTE method

`PortalHypothesisLedger` stores the physical gate and destination-side point in
the odometry frame. `GraphRoutePlanner` can therefore select a certified,
unbound Portal as a `cross_portal` action even though no destination Place has
been created yet. `GlobalFrontierPortalEgressMixin.select_durable_portal_crossing`
then compiles that action into a normal route tuple:

```text
durable Portal ID
    -> current map projection (one TF snapshot)
    -> source-side proof
    -> destination-side crossing goal
    -> reachable grid cell + costmap/Navfn validation
    -> graph transaction commit
    -> TEB route execution
```

The route carries `portal_transition`, `cross_portal`, and the same Portal ID
through activation. A new Place is still created only after physical crossing
and fresh destination structural evidence. A stale SLAM frontier cannot mint a
second WorkItem or a second Portal.

There is one additional explicit state. A destination-view action is allowed
to look through a door, so its safe endpoint can occasionally carry the base
past the gate before the graph promotes the Portal. If the current odometry is
already beyond the existing crossing depth, the compiler emits
`portal_crossing_preobserved`; the terminal verifier closes the same edge using
that fact. A pose that is merely near or slightly beyond the gate is still
rejected until a source-side crossing is proven.

The pure conversion lives in
`src/lste_topo_access/scripts/global_frontier_durable_portal_route.py`. It
prefers a fresh physical TF projection and falls back to the last coherent map
projection during a short TF outage. It has no ROS, score, timer, or tunable
policy.

## Why this is architectural

The old adapter had two choices when a graph-selected Portal was not present in
the current frontier: wait forever or allow an unrelated frontier to replace
the action. The new adapter has a third, typed choice: compile the persistent
physical edge directly. This removes the accidental dependency

```text
graph identity == current frontier cell
```

without weakening collision checks or Portal evidence requirements.

## Falsifiable experiment

The next Level 4 runs must compare the same graph method with and without this
compiler. Required logs and metrics include:

- `durable_portal_crossing_selected` and
  `durable_portal_crossing_unavailable` counts;
- graph materialization waits after a destination-view promotion;
- Portal crossing success and covered-place re-entry;
- repeated WorkItem dispatches, completion time, path length, collision/failure
  rate, and motion smoothness;
- exact route/action IDs in the timestamped event log.

The compiler fixes one observed deadlock, but it is not evidence that the full
semantic target task is solved. Complete Level 4, baseline, and ablation runs
remain required before claiming superiority.
