# Targeted Navigation Experiments

## Purpose

Full Level 4 runs are useful for final evidence, but they are too slow for
diagnosing one navigation failure.  The development loop therefore starts with
the smallest reproducible counterexample and grows the environment only after
the relevant invariant is verified.  A run should normally finish in seconds
or a few tens of seconds.

The first failure class is a doorway route that reaches the source-side throat,
fails, and is retried after SLAM moves the frontier projection.  The physical
Portal and its Observation WorkItem must survive that retry unchanged.

## Experiment Ladder

| Stage | Scene | Main invariant | Evidence before promotion |
| --- | --- | --- | --- |
| A | One Place, one doorway | Retry keeps one Portal transaction and one WorkItem lineage | ROS-free replay passes; no duplicate identity |
| B | Two Places, two doorways | Portal identities do not cross or get swapped | Each route is bound to its selected Portal and source Place |
| C | T-junction with several Places | A completed Place is transit only | Planner never creates a normal observation WorkItem for a completed Place |
| D | Small office area | Place/Portal/WorkItem graph handles local work and outward branches together | Repeated map updates do not re-dispatch old routes |
| E | Level 4 office benchmark | End-to-end research comparison | Automated metrics, logs, and repeatable video/results |

Promotion is evidence-gated.  A later stage must not be used to explain a
failure that has not been reproduced at the earlier stage.

## Stage A: Pure Replay

Run without ROS, Gazebo, detectors, MiniCPM, or a controller:

```bash
scripts/tests/targeted_navigation/portal_retry_single_door.py run
```

The replay uses the production `DirectionalBranchCoverage` and
`PortalTransaction` modules.  Its event trace is:

```text
run_start
source_evidence
route_dispatched
route_failed
route_retry
destination_evidence
crossing_accepted
run_complete
```

The frontier projection changes between attempts, while the durable branch and
WorkItem IDs remain constant.  `PortalTransaction` also verifies that the new
route ID inherits source-side proof and that destination Place commit is only
accepted after crossing evidence.

Each invocation creates exactly one process log at:

```text
runtime/targeted_navigation/portal_retry_single_door/logs/YYYYMMDD_HHMMSS/
└── YYYYMMDD_HHMMSS_portal_retry_single_door.log
```

For deterministic replay tests, pass `--timestamp YYYYMMDD_HHMMSS` and a
temporary `--log-root`.  The CLI rejects a colliding timestamp directory so a
run cannot silently overwrite evidence.

## Stage A Acceptance Criteria

- The process exits successfully and reports `"passed": true`.
- There are exactly two route attempts and one stable branch identity.
- The retry reports `projection_changed=true` and `same_work_item=true`.
- Source-side proof remains present after retry and route rebinding.
- Destination evidence closes the WorkItem; crossing changes the branch to
  `transit` without creating another WorkItem.
- The log contains all eight events in order and no non-log artifact in its
  timestamped directory.

## Stage B: Two-Door Identity Replay

Run the next topology-only counterexample after Stage A:

```bash
scripts/tests/targeted_navigation/portal_identity_two_door.py run
```

This replay keeps two physical doorways in the same source Place, interleaves
their map projections, fails and retries only doorway A, and then crosses A.
It verifies that a retry cannot alter doorway B's transaction, route, Portal,
or WorkItem.  A later map update for completed A is also checked to remain
transit-only.

Its log is written to:

```text
runtime/targeted_navigation/portal_identity_two_door/logs/YYYYMMDD_HHMMSS/
└── YYYYMMDD_HHMMSS_portal_identity_two_door.log
```

Stage B is accepted only when the two branch IDs and WorkItem IDs remain
distinct, A reaches `transit`, B remains `open`, and the transaction state for
A is `place_commit` while B is still `source_probe`.

## Stage C: Completed-Place Transit Replay

The T-topology replay exercises the graph planner itself:

```bash
scripts/tests/targeted_navigation/completed_place_transit.py run
```

The durable graph is `Place 1 -> Portal 1 -> Place 2 -> Portal 2 -> Place 3`.
Place 2 is already observed and is allowed as a transit node only.  The real
`GraphRoutePlanner` must select the two-edge path from Place 1, then select only
Portal 2 after the robot enters Place 2.  Place 3, and not Place 2, receives
the next `bootstrap_observation` WorkItem action.

The replay also moves both directional branch projections through later map
epochs.  It is accepted only when Place 2 never gets a local WorkItem or a
second re-entry action and both completed branches remain `transit`.

## Fast Regression Commands

```bash
python3 -m unittest \
  src/lste_topo_access/test/test_portal_retry_single_door_replay.py \
  src/lste_topo_access/test/test_portal_identity_two_door_replay.py \
  src/lste_topo_access/test/test_completed_place_transit_replay.py \
  src/lste_topo_access/test/test_graph_route_transaction.py \
  src/lste_topo_access/test/test_graph_route_gate.py

python3 -m unittest discover -s src/lste_topo_access/test -p 'test_*.py'
python3 -m unittest discover -s src/lste_core/test -p 'test_*.py'
```

The first command is the tight development loop.  The two discovery commands
are the regression gate before moving from one stage to the next.

## What This Does Not Prove

The pure replay does not prove Navfn reachability, TEB control quality, sensor
timing, or Gazebo physics.  Those belong to the next minimal simulation stage.
The replay is deliberately limited to the durable identity and evidence
contract, so a passing result is necessary but not sufficient for the Level 4
goal.

## 20260908 State-Ownership Checkpoint

The next focused slice converted four previously implicit state assumptions
into production-code regression cases:

1. Only a Portal whose lifecycle state is `crossed` contributes an edge to the
   durable Place graph. A destination binding by itself is still a hypothesis
   and cannot justify covered-to-covered transit.
2. WorkItem reconciliation uses the physical boundary anchor and its
   unknown-side normal when both are available. A disjoint support after SLAM
   relabelling can therefore inherit one WorkItem; a nearby compatible opening
   with a different anchor receives a different identity. A resolved support
   remains allowed to absorb an overlapping descendant even when its frontier
   anchor has advanced.
3. A semantic WorkItem terminal must carry the currently active Attempt ID.
   A no-ID or stale terminal cannot resolve a replacement Attempt. The legacy
   `settle()` facade obtains the active ID internally so existing lifecycle
   callers retain the same explicit ownership semantics.
4. A target-observation transaction owns its current Place. Its local safe
   viewpoint may come from a frontier candidate, but that candidate is not
   upgraded to a Portal probe until target evidence has settled. This prevents
   a target reinspection from becoming an unexecutable doorway action.

The wide-door fallback also accepts an optional signed candidate direction.
When a caller has directional evidence, only the reverse half-plane of an
already crossed gate is admitted; the compatibility path without direction
remains available to older geometry-only callers.

The focused and full tests pass:

```text
python3 -m pytest -q src/lste_topo_access/test
822 passed
```

The four ROS-free replay commands in this document also pass in one short
batch. This is an identity/state result, not a Gazebo result. Historical Level
4 logs still contain long graph-materialization waits in the target Place, so
the Level 4 exploration and semantic-task objective remains open until a new
run demonstrates executable target/local recovery and is verified by the
benchmark scripts.

## Current Architecture Checkpoint

The targeted loop now has two explicit recovery boundaries:

- A destination-side probe that ends in a controller `failed`/`blocked` result
  is released to another physical viewpoint in the same map epoch. An
  inconclusive destination observation still waits for a later map epoch, so
  the timer cannot replay one viewpoint indefinitely.
- An unresolved local WorkItem can be rehydrated from its physical anchor and
  unknown-side normal when its transient frontier arc disappears. In the full
  graph-first method, a current-place obligation that cannot yet be
  materialized is a hard `hold`; the planner cannot satisfy a remote task by
  re-entering an already observed Place.
- A controller failure on an already-crossed Portal is execution evidence,
  not negative physical evidence. It increments
  `execution_failure_count` while preserving `state=crossed` and the original
  destination; only an un-crossed Portal may enter the durable `failed` state.

The pure regression suite covers these contracts and currently passes 722
`lste_topo_access` tests plus 8 `lste_core` tests (one optional detector test
is skipped when Torch is unavailable). A 45-second headless single-door smoke
run reached the second Place,
completed local observation viewpoints, and ended at the explicit frontier
exhaustion gate. It still exposed a legitimate return to the source Place for
remaining source-side work, so this checkpoint does not claim that all room
revisits are eliminated. It does not establish Level 4 completion or controller
smoothness.

## Wide-Doorway Self-Loop Checkpoint

The compact T-junction world at
`worlds/targeted_navigation/t_junction_small.world` isolates a failure that
the one-door replay cannot represent: the architectural opening is wider than
one grid cell, so SLAM may report its two jambs as different map candidates and
may flip the instantaneous opening normal. Before this checkpoint, that
combination produced additional Portal records whose source and destination
were both Place 2.

The fix keeps two layers of evidence separate:

1. `PortalHypothesisLedger.crossed_gate_for_destination()` compares a new
   physical gate to the durable crossed gate plane. It allows a derived
   tangent span for a wide opening while keeping the crossing-direction offset
   tight, so a nearby independent doorway is not accepted just because it is
   on the same wall.
2. Probe registration and Portal certification fail closed when the current
   Place is already the destination of that crossed gate. The final destination
   binding also rejects `source_place_id == destination_place_id` as an explicit
   graph invariant.

The focused tests cover both the geometry and the ROS boundary:

```bash
python3 -m unittest \
  src/lste_topo_access/test/test_portal_belief.py \
  src/lste_topo_access/test/test_portal_probe_ledger.py
```

The first headless integration run was captured at
`runtime/targeted_navigation/t_junction_autonomous/logs/20260907_055542/`.
Its status stream contained 325 snapshots, one
`same_wide_gate_as_crossed_portal_is_transit` suppression, one physical
Portal crossing, and no record satisfying
`source_place_id == destination_place_id`. A follow-up run after wiring the
same gate-band query into reverse egress was captured at
`runtime/targeted_navigation/t_junction_autonomous/logs/20260907_060805/`.
It contained 460 snapshots, two gate-band probe suppressions, one durable
reverse-egress reuse, and three crossing events involving two Portal IDs (one
forward crossing and one forward/reverse pair), while still producing zero
self-loops. These are targeted invariant results, not a claim that the
T-junction or Level 4 mission is solved: both runs retained unresolved
source-side obligations and were stopped after the relevant crossing window to
keep the experiment short.

## 20260907 Evidence-Scoped Viewpoint Checkpoint

The next targeted slice addresses a distinct failure from the wide-door
identity bug: after a source-side probe arrived, all three destination ladder
viewpoints could be projected by the current SLAM/costmap snapshot onto the
same source-side cell.  Treating those cells as independent attempts both
polluted the rejection count and could leave the graph waiting for a
destination observation that had never been executable.

`PortalProbeLedger` now records each completed viewpoint with its physical
coordinate, observation phase (`source` or `destination`), map epoch, and
outcome.  Deduplication is scoped to the phase and epoch, so source evidence
cannot suppress the first destination-side evidence.  A later map epoch can
reopen an inconclusive destination obligation without minting another Portal,
Place, or WorkItem.  The materializer also records the complete mapping
`desired point -> projected cell -> physical viewpoint -> rejection reason`.
When the full ladder collapses to the source cell it emits
`projection_collapsed=true`, rejects the action as
`projection_collapsed_to_source_viewpoint`, and parks the durable probe as
`awaiting_projection`.

The structural-boundary compiler now uses the existing clearance-derived wall
support for source probes.  A two-cell raster wall thickness is not sufficient
architectural evidence; this prevents parallel corridor walls from producing
false Portal hypotheses while retaining the same wall evidence used by Portal
certification.  A focused regression fixture demonstrates that a corridor
without a doorway yields no structural probe.

The ROS-free evidence gate currently passes:

```text
python3 -m unittest discover -s src/lste_topo_access/test -p 'test_*.py'
Ran 728 tests in 0.800s
OK
```

The short Gazebo T-junction smoke run on 2026-09-07 reached three physical
Place identities in approximately 68 seconds.  The first new Place was
committed at about 35 seconds and the third at about 68 seconds.  It produced
no self-loop, no `source_place_id == destination_place_id` Portal record, and
no projection-collapse redispatch.  Several `frontier_route_unavailable`
messages were immediately followed by a graph-route selection in the same
decision cycle; this is a route-snapshot handoff, not yet evidence that the
full benchmark is stall-free.  The run was stopped after the third Place
entry, so it is targeted invariant evidence only and not a Level 4 completion
claim.

The same smoke trace also exposed a lifecycle distinction in the ROS adapter:
after a successful terminal, the successor planner can briefly lack a
materialized graph edge while `active_frontier` is already cleared.  That is a
planning snapshot gap, not an instruction to cancel a live controller lease.
The adapter now suppresses the stale `frontier_route_unavailable` handoff in
that state; the durable candidate-unavailable event remains available for
replanning and audit.  The focused regression is
`test_post_terminal_unavailable_plan_does_not_release_old_controller_lease`.

## Subsecond Failure-Evidence Replay

The physical endpoint runner still has to restart Gazebo when sensor, costmap,
or TEB behavior is under test.  That is the outer validation loop.  Changes to
the evidence classifier and the route/command diagnosis should not require
that restart, however.  The fixed fixture
`scripts/tests/targeted_navigation/fixtures/t_junction_controller_stall.json`
captures the smallest useful local trace: pre-trigger motion, a
`route_invalidated_stall` trigger, and the post-window state.  Replay it with:

```bash
python3 scripts/tests/targeted_navigation/replay_failure_evidence.py run \
  --log-root /tmp/lste_failure_replay
```

The command does not import ROS or start Gazebo.  It runs the same pure
`classify_failure()` and `diagnose_failure_sample()` functions used by
`lste_navigation_metrics`, then checks that the result remains
`controller_stall`, `layer=global_frontier`, `route_id=9`, and
`route_kind=frontier_endpoint`.  It writes one timestamped
`*_failure_evidence_replay.log`; the default invocation completes in well
under one second.  Pass `--timestamp YYYYMMDD_HHMMSS` to make a replay
addressable; an existing timestamp is rejected rather than overwritten.

This replay proves the causal evidence contract and protects against
classifier regressions.  It does not prove that the live controller will
reach the endpoint.  After a replay passes, use the physical inner-loop
runner for that question:

```bash
scripts/tests/targeted_navigation/run_t_junction_endpoint.sh --max-seconds 30
```

## Minimal Gazebo Baseline

A minimal two-room, one-door Gazebo world is now available at
`worlds/targeted_navigation/portal_retry_single_door.world`. It has no embedded
robot; spawn the robot from a launch file so the pose is explicit and
reproducible. The structural test checks the six wall segments and the central
one-metre opening. A short `gzserver` smoke test confirms that the world parses
and exits cleanly.

This composition was the first targeted integration baseline. It keeps the
minimum SLAM/Navfn/TEB nodes needed for a doorway route and omits detectors and
MiniCPM; the same event assertions and timestamped log contract are reused by
the T-junction experiment above. The full office benchmark remains outside
this diagnosis loop.

The development-only composition is
`portal_retry_single_door_nav.launch`. It starts Gazebo, Pro3, the point-cloud
to LaserScan adapter, online SLAM, global frontier, Navfn/TEB, and the command
velocity mux, but none of the detector or MiniCPM nodes. Validate its XML
without starting anything with:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch --dump-params \
  scripts/tests/targeted_navigation/portal_retry_single_door_nav.launch
```

For a manual controller baseline, leave that launch running headless and use a
second shell to publish one atomic mission-goal JSON message:

```bash
rostopic pub -1 /lste/mission_goal std_msgs/String \
  "data: '{\"event\":\"mission_goal\",\"transaction_id\":1,\"source\":\"targeted_single_door\",\"priority\":0,\"route_kind\":\"targeted_door\",\"mission_route_kind\":\"targeted_door\",\"route_id\":1,\"frame_id\":\"map\",\"goal\":[2.0,0.0],\"yaw\":0.0}'"
```

A short headless smoke run showed all required nodes registered and a direct
goal through the one-metre doorway moved the robot about 1.92 m and returned
MoveBase status `SUCCEEDED` within the bounded run. This is a controller and
geometry baseline, not yet proof of autonomous Portal discovery. The first
recorded run is under
`runtime/targeted_navigation/single_door_nav/logs/20260907_005130/`; its
`20260907_005130_motion_monitor.log` records `succeeded=True` and
`odom_x_delta=1.937`. The current composition was then rerun under
`runtime/targeted_navigation/single_door_nav/logs/20260907_010222/`; it records
`succeeded=True`, `odom_x_delta=2.627`, Place bootstrap, and a
`local_egress_recovery` frontier decision.

## Next Implementation Step

The next short experiment should focus on route ownership after a successful
crossing.  A reverse transit through a completed Place is legal only when it
is the first graph hop needed by a still-pending WorkItem or Portal.  Capture a
fresh status stream through the first reverse route and assert that one
crossing commits the robot to the source Place before any second transit route
is selected.  Do not tune a timeout or speed parameter to hide a duplicate
route; the route lease and Place identity must explain every crossing.

## 20260907 Navfn Viewpoint-Admissibility Checkpoint

The Level 2 office smoke exposed a different boundary failure after the
Place/Portal transaction was already correct.  A durable local WorkItem was
rehydrated successfully, but Navfn returned a plan ending 0.20 m away from the
requested standoff.  The old selector treated that as an empty candidate and
repeated the same request until its validation budget was exhausted.

The WorkItem ledger now keeps `route_rejections` separately from controller
Attempts.  A pre-dispatch Navfn/costmap rejection records the physical
viewpoint, map goal, map epoch, actual Navfn endpoint, and reason, but does not
invent a dispatched or failed Attempt.  The next projection of that same
WorkItem rejects only the known-bad standoff and compiles the next safe point
from the existing normal/lateral viewpoint ladder.  The graph plan remains
owned by the same WorkItem, and the transaction records the explicit
`observe_local_work -> retry_viewpoint` refinement.
These route rejections are scoped to the current topology/map epoch; an older
costmap decision remains auditable but cannot permanently poison a later SLAM
snapshot. Executed failed Attempts remain physical-history facts and keep their
existing long-term viewpoint exclusion.

Focused regression coverage includes:

```bash
python3 -m unittest \
  src/lste_topo_access.test.test_place_work_items \
  src/lste_topo_access.test.test_work_item_rehydration
```

The first Level 2 targeted run is recorded at
`runtime/office_building_benchmark/logs/20260907_075050/`.  It produced one
`work_item_viewpoint_route_rejected` event with
`reason=navfn_endpoint_offset`, followed 0.574 seconds later by a committed
same-WorkItem `lateral_left` route, and no
`graph_route_materialization_wait` event.  A repeat run with another seed is
at `runtime/office_building_benchmark/logs/20260907_075419/`; it crossed into a
new Place without a materialization-wait storm, although that run did not
encounter an endpoint-offset rejection before it was intentionally stopped.
A third short progression run is at
`runtime/office_building_benchmark/logs/20260907_080210/`; it completed four
local WorkItem attempts and selected a certified `cross_portal` route to the
next Place at `ros_time=93.82`, again with zero
`graph_route_materialization_wait` events before the run was stopped.
After scoping route rejections to the current map epoch, a follow-up Level 2
run is at `runtime/office_building_benchmark/logs/20260907_084314/`; it crossed
into Place 2, continued to a second local WorkItem, and recorded zero
materialization waits or route-unavailable events in its bounded window.

The full ROS-free regression gate after this change is:

```text
lste_topo_access: 740 tests passed
lste_core: 8 tests passed, 1 optional detector test skipped without Torch
```

This checkpoint demonstrates route-admission recovery for one Level 2 failure
class.  It does not establish Level 4 completion, target success, collision
robustness, or smoothness across the formal benchmark matrix.

## 20260907 Persistent Egress Terminal Checkpoint

The first Level 3 short run exposed a separate persistent-execution hole.  A
failed local WorkItem route reached a physically safe egress anchor, but the
bridge accepted persistent endpoint reports only for `frontier_endpoint` and
`portal_transition`.  `PersistentTebLocalPlanner` therefore kept reporting
the reached endpoint while the bridge never emitted the matching terminal;
the recovery route eventually aged into `local_egress_stall`.

`local_egress` is now a first-class route kind at the same endpoint identity
boundary.  The bridge still checks source/active goal identity, route ID,
frame, and endpoint displacement before publishing the terminal.  The global
terminal recorder already owns the corresponding Place-lease completion, so a
successful egress now closes the recovery route and requests a fresh graph
snapshot without a controller restart.

The bridge regression covers the accepted local-egress report:

```bash
python3 -m unittest \
  src/lste_topo_access.test.test_teb_persistent_portal_terminal
```

The Level 3 reproduction run is at
`runtime/office_building_benchmark/logs/20260907_080911/`; before the bridge
change it showed a 34-second egress hold followed by
`local_egress_stall`.  A patched Level 3 rerun is at
`runtime/office_building_benchmark/logs/20260907_082015/`; it produced
multiple persistent endpoint terminals and two Place entries without a
materialization-wait storm.  Its route did not enter the egress branch before
the bounded stop, so the pure bridge regression remains the direct evidence
for that route-kind fix.

## 20260907 Level 4 Post-Fix Diagnostic

After the two focused fixes, a bounded Level 4 topology-only diagnostic was
recorded at
`runtime/office_building_benchmark/logs/20260907_082603/`.  It ran for
`158.7s`, entered multiple physical Places, completed three local WorkItems,
and reached a third Place before the intentional stop.  The offline summary
reported:

```text
collision_events=0
failure_events=0
graph_route_materialization_wait=0
frontier_route_unavailable=0
duplicate_dispatch_count=0
room_reentries=0
```

The run ended with `task_done=false` and an active WorkItem at the stop
boundary.  It is post-fix diagnostic evidence only, does not count as a
completed Level 4 trial, and does not justify a comparison against the formal
benchmark matrix.

## 20260907 Active-Parallax Target Checkpoint

The target-room isolation run exposed a perception/action deadlock: WeDetect
returned the same target from a stationary pose, while target confirmation
required physical viewpoint diversity.  The new `target_parallax` action
compiles one short side-step, validates it with Navfn, and only then lets the
target track own a pursuit segment.  It is bounded to two deterministic side
choices and is separate from the target approach transaction.

Evidence from
`runtime/office_building_benchmark/logs/20260907_102435/`:

- side-step selected at `27.710s`, target confirmed at `28.297s`;
- three target approach segments committed at `28.512s`, `31.111s`, and
  `33.322s`;
- task completion at `68.289s`, with `move_base_aborts=0` and zero collision
  events;
- target geometry evaluation matched the Gazebo target in 388/394 exposed
  detector frames.

This is a fast architecture check, not a benchmark result: frontier
exploration was disabled and the verifier correctly rejected it for visiting
only one room.  A first attempt to use the raw triangulated point as a metric
completion/standoff signal was also rejected: a low-residual estimate can be
systematically biased by camera calibration and produced an early false
`task_done`.  Production keeps that estimate diagnostic-only until an
independent calibration contract is implemented.

## 20260907 Level 2 Bounded Exploration Check

The bounded production composition at
`runtime/office_building_benchmark/logs/20260907_104231/` ran the normal
frontier method for approximately 156 simulated seconds before the targeted
window was stopped.  It had no target-parallax event because the robot had not
yet reached the target room, and the navigation stream recorded zero
`move_base_aborts` and zero recovery events.  This is a useful compatibility
check for ordinary exploration after the target-state changes, but it is not a
Level 2 completion trial and is intentionally excluded from benchmark
aggregates.

## 20260907 Portal-Crossing Priority Checkpoint

The Level 1 primary bounded run at
runtime/office_building_benchmark/logs/20260907_120420/ exposed a planner
ordering bug. A Portal with completed source and destination evidence was
available at the same time as a new structural-boundary probe. The planner
selected the new probe first, so one physical corridor Place accumulated route
dispatches without crossing the certified edge.

The graph planner now orders these actions as:

target observation -> destination probe -> certified Portal crossing
-> new structural probe -> local WorkItem

The focused regression
test_certified_portal_crossing_precedes_new_structural_probe covers the
previously missing coexistence case.

The post-change run produced:

- portal_place_entered: two physical Place transitions;
- Place sequence: main_corridor -> conference -> main_corridor -> lobby;
- physical room reentries: 0;
- portal hypothesis crossings: 2;
- graph invariant violations: 0;
- collisions: 0;
- route-unavailable episodes: 0.

The run was intentionally bounded before target completion. Motion remains an
open issue: maximum stop duration was about 21.6s, with persistent TEB turn
transitions still requiring a separate controller-level experiment. The
route_dispatches, physical_entry_count, and observation_sessions fields now
distinguish route selection from physical Place evidence in the durable memory.

## 20260907 Target-Entry Evidence-Gated Revalidation Checkpoint

The focused Level 4 run at
`runtime/office_building_benchmark/logs/20260907_221802/` used the
`target_entry` diagnostic profile. It started at the target-room approach pose,
so it did not pay the full exploration cost. The run validates the new target
evidence boundary, but it is not a completion trial.

The weak-track policy rejected the first two short-lived candidates. The third
track was promoted only after nine observations, six rays, a real translation
baseline, and the estimator-backed geometry contract:

```text
first target track:       19.327s
confirmed target track:   77.355s
target viewpoint segments: 4
portfolio revalidation:   87.318s
move_base aborts:         0
move_base recoveries:     0
collision events:         0
task_done:                false
```

Exhausting the four known viewpoints did not publish `task_done`; it emitted
`target_viewpoint_portfolio_requires_revalidation`, preserved the target-room
obligation, and requested a fresh local observation transaction. The target
WorkItem was still unresolved when the bounded run stopped. This is the
intended result: finite viewpoint exhaustion is not proof that the semantic
object was reached. The run still accumulated approximately 67 seconds of
zero-velocity time, so controller smoothness and target completion remain open
experiments.

The same checkpoint also records a geometry consistency correction: target ray
origins now come from the camera TF translation at the detector image stamp,
matching the source-stamped bearing. Legacy fixtures without that TF use the
latest-pose fallback and label the degraded source in the arbitration event.

## 20260908 Local Place-Closure Checkpoint

The bounded Level 1 run at
`runtime/office_building_benchmark/logs/20260908_004657/` was stopped after
approximately sixteen wall-clock minutes. It did not deadlock on a target
lease: `target_route_failures=0`, the longest `frontier_route_unavailable`
episode was `5.097s`, and no route-stagnation event was observed. Collision
truth and MoveBase abort counts were both zero.

The run did expose one real graph-policy defect. The route sequence was:

```text
Place 8 -> Place 9 -> Place 10 -> Portal 11 -> covered Place 8
```

Place 8 already owned WorkItem 142 with a durable `normal_xy` and physical
anchor, but `branch_first` crossed into the next Place before that local work
was resolved. The return was correctly authenticated as
`portal_place_covered_arrival` and did not duplicate the WorkItem; it was still
an avoidable physical room re-entry because the local viewpoint was already
rehydratable.

The graph planner now treats a rehydratable local WorkItem as a Place-closure
barrier. `branch_first` may still bypass a candidate with no durable viewpoint
geometry, preserving the non-deadlocking fallback for temporarily unmaterial-
izable evidence. The focused planner tests cover both branches, including the
case where the WorkItem is hidden from the current frontier projection.

The benchmark summary now scopes room truth to the level recorded in the
run-start world, preventing Level 1 from being classified with disabled Level 2
room rectangles. The current runtime result remains diagnostic rather than a
completion claim: it visited only two enabled semantic rooms, recorded one
target-room re-entry before the policy change, and did not finish the task.
