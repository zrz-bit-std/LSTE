# Branch-First Evidence-Gated Exploration

Date: 2026-09-06

Status: executable research slice; no benchmark conclusion yet.

## Problem

The original Place/Portal/WorkItem policy treated every unresolved local
frontier as a prerequisite for leaving a Place. In an online SLAM map this is
not a finite obligation: a scan can expose more frontier arcs while the robot
is still observing the current room. The result is a structurally correct but
operationally wrong policy: the robot spends the whole episode clearing local
frontier descendants and never follows an already certified doorway to the
next region.

The opposite failure is also unsafe. Leaving a Place before any observation
evidence can make the next room immediately select its reverse door, and a
late terminal can commit a destination using the wrong route identity.

## Method

The full `place_portal_workitem` method now declares one additional capability:
`branch_first`. It changes the graph action policy, not a score or controller
parameter:

```text
new Place
  -> one physical observation is required
  -> certified Portal branch may execute
  -> unresolved local WorkItems remain owned by the source Place
  -> later transit may return through the directed graph edge
```

The rule is deliberately asymmetric:

- a Place with no observation cannot cross any Portal;
- a certified Portal to an **unobserved** Place may outrank remaining local
  WorkItems after the first observation in the full method;
- an uncertified unknown boundary is still only a `probe_portal` action;
- a certified edge into a covered Place remains blocked while local work is
  pending, so branch-first cannot become a room-reentry shortcut;
- any physical departure that still owns ordinary local WorkItems is committed
  as `suspended`, so recovery and covered-transit routes cannot silently erase
  unfinished work;
- a failed viewpoint creates another Attempt for the same WorkItem;
- a dormant Place can be transit, but cannot reopen local work.

The baseline methods retain their previous strict local-work gate. This makes
the capability an explicit architectural arm rather than a hidden behavior
change shared by every experiment.

## Why this is architectural

The old question was “which frontier has the largest score?” The new question
is “which graph action is legal in the current Place phase?” Numeric scoring is
only a tie-break inside that action class. A transient SLAM frontier therefore
cannot monopolize the mission, and a small parameter change cannot silently
turn a local endpoint into a room transition.

This follows the branch-first idea used by STGPlanner and the explicit
region/opening/pathway separation in GRID-FAST, while retaining LSTE's own
physical Portal certificate and durable WorkItem ownership. The implementation
is ROS-free at the policy boundary in
`global_frontier_graph_executive.py`; the ROS node only passes the declared
method capability into candidate routing and portal selection.

## Transaction integrity

Portal arrival is an asynchronous event. Before `region_memory.enter()` writes
anything, the arrival must match the active transaction's route, Portal and
source Place identities. A stale or mismatched arrival is rejected and logged;
it cannot create a phantom Place. The pure transaction contract keeps old
fixture calls compatible, while production arrival code supplies the complete
identity available in the event.

## Falsifiable predictions

With controller, SLAM, detector, world and seed held fixed, branch-first should:

1. reduce time spent in the initial Place before the first new Place commit;
2. increase the number of certified Portal crossings before frontier exhaustion;
3. keep resolved-WorkItem redispatches and phantom Place commits at zero;
4. preserve the no-reentry invariant even when local WorkItems remain pending.

These are hypotheses, not results. A single development run is not evidence
of superiority.

## Required ablations and metrics

At minimum compare:

1. ordinary frontier;
2. distance-deduplicated frontier;
3. Place/Portal/WorkItem with the strict local-work gate;
4. Place/Portal/WorkItem with `branch_first`.

The strict arm is registered as `place_portal_workitem_strict`. It shares the
full method's perception, Place, Portal and WorkItem capabilities and differs
only in the discrete branch transition rule.

Use paired seeds and the same Level 4 layout. Report repeated physical Place
entries, effective time per Place, resolved WorkItem redispatches, Portal probe
yield, certified crossings, phantom Place rate, stale transaction commit rate,
coverage, target success, path length, completion time, collisions/failures and
motion smoothness. Keep `branch_first` in the method provenance so aggregators
cannot merge the two policies.

## Current verification

Pure policy and transaction tests cover:

- strict baselines still block crossing with unresolved local work;
- branch-first permits a certified Portal only after first observation;
- an unobserved destination cannot immediately chain another Portal;
- stale route identity cannot commit a transaction;
- matching route/Portal/source identity commits exactly once.

Full system behavior still requires clean Level 4 runs and automated replay of
the timestamped event logs. No completion or performance claim is made here.
