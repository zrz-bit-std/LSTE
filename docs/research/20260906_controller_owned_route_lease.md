# Controller-Owned Route Lease

Date: 2026-09-06  
Status: implemented as an execution-policy slice; no benchmark conclusion.

## Problem

The Place/Portal/WorkItem graph can prevent a duplicate room decision, but a
second policy can still undo that work: the global frontier watchdog used
elapsed time and map-derived progress to cancel a live TEB action. In the
latest Level 4 run, the navigation log contains 21 `frontier_route_invalidated`
stalls and 21 new move_base dispatches. These are not proof that the selected
physical branch was impossible. They are evidence that route failure ownership
was split between the graph layer and the local controller.

## Architectural decision

The main persistent execution path now has one explicit route lease:

```text
graph action commit -> one move_base/TEB lease
                         |
                         +-- SLAM/map change: update evidence, retain lease
                         +-- no motion: report route_stagnant_observed
                         +-- TEB/move_base recovery terminal: release lease
                         +-- task/graph transaction terminal: release lease
```

The global layer still computes physical progress and reports stagnation. It
does not convert a timer expiry into a failure while persistent execution is
active. TEB and move_base already own collision checking, recovery behaviors,
and the authoritative action terminal, so the global explorer waits for that
fact instead of guessing from a second clock.

The pure contract is in
`src/lste_topo_access/scripts/global_frontier_route_lease.py`:

- persistent mode: `recovery_pending` is the only controller failure release;
  stall, post-turn stall, portal deadline, and active timeout are diagnostic;
- legacy mode: the historical global-watchdog release behavior remains;
- no new score, confidence, distance, speed, or timeout parameter is added.

This is a lease-ownership change, not a larger watchdog timeout. It follows
the fast/slow separation used by recent active-evidence navigation work such as
AECNav ([arXiv:2608.10817](https://arxiv.org/abs/2608.10817)) and the
structural action commitment used by topological exploration work such as
STGPlanner ([arXiv:2412.13664](https://arxiv.org/abs/2412.13664)). LSTE keeps
its own boundary: the Place/Portal graph chooses a legal information action;
Navfn/TEB owns metric execution and failure.

The bridge keeps a failed route identity in a separate controller lease record
while clearing action-scoped feedback and TEB health. This prevents an
`ABORTED` callback from erasing the only evidence that a subsequent timer is
trying to resend the same route. The lease is cleared only by a new route ID,
an explicit higher-priority cancellation, or a successful terminal.

## Runtime consequences

`route_stagnant_observed` is emitted once per route identity with the active
route, authority, elapsed evidence and last physical-progress signal. A
temporary map/costmap disconnection is held under the same lease in persistent
mode. The old endpoint mode remains available for a controlled comparison.

This should reduce false route cancellation, zero-velocity gaps and repeated
selection from a stale local snapshot. It cannot by itself prove that TEB can
solve every local trap; that requires the controller's own terminal/recovery
result and paired benchmark runs.

## Verification

```bash
python3 -m pytest -q src/lste_topo_access/test/test_route_lease_authority.py
python3 -m pytest -q src/lste_topo_access/test
```

The required system comparison keeps world, seed, detector, SLAM, Navfn and
TEB fixed. It must compare lease releases, unexpected preemptions, stagnation
events, repeated Place entries, WorkItem redispatches, coverage, target
success, path length, completion time, failures/collisions and motion
smoothness. The current development log remains evidence for the problem
statement only; this increment has not been benchmarked yet.
