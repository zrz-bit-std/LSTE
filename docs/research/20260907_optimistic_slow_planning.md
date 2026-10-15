# Optimistic Slow Planning Contract

Date: 2026-09-07  
Status: implemented as a runtime architecture slice; no benchmark claim.

## Motivation

The Place/Portal/WorkItem policy already separates durable evidence from a
short-lived occupancy-grid snapshot.  The execution path still had one
architectural leak: the timer callback held `planning_lock` while it built a
map snapshot, ran BFS, enumerated frontier candidates, and waited for Navfn
validation.  The Level 4 runtime showed selection phases as long as 15.855 s.
An execution terminal arriving during that phase could be queued, but the
successor route still had to wait for the entire selector to release the lock.

This is a scheduling problem, not a TEB gain problem.  Making a timeout or a
score coefficient smaller would only hide it in one scene.

## Architecture

The planner now uses an optimistic two-speed boundary:

```text
fast ROS ingress / TEB execution
        | terminal invalidates proposal
        v
immutable planning token -> slow map/graph selection -> proposal commit
                                                |
                                                +-- current route lease: publish
                                                +-- stale lease: discard
```

`global_frontier_planning_contract.py` defines four ROS-free records:

- `PlanningToken` identifies one slow snapshot generation and the route lease
  that existed when it began.
- `PlanningProposal` carries the selected candidate and action mode without
  changing the candidate tuple contract.
- `PlanningProposalGate` serializes generation invalidation and the short
  activation callback.
- `ProposalDecision` makes stale and accepted outcomes explicit in tests.

The global-frontier timer acquires `planning_cycle_lock` only to prevent two
timer callbacks from running at once.  It uses the older `planning_lock` for
short ingress and cleanup boundaries, then releases it for the expensive
snapshot and candidate-selection phases.  The final route activation and
command publication run through `commit_if_current`.  The execution-terminal
callback invalidates the gate before enqueueing its message; it never waits
for candidate enumeration.

This is an optimistic commit protocol, not a second controller.  Navfn still
proves route reachability, TEB still owns velocity, and the Place/Portal graph
still owns semantic action legality.  A map update may continue to be used for
the next snapshot; a terminal cannot be overwritten by a result computed for
the old route lease.

## Why this is an architectural change

The old synchronization model coupled the deliberative layer and the action
terminal through one long critical section:

```text
map snapshot + graph choice + route publication
                         all under planning_lock
```

The new model has a clear ownership boundary:

```text
slow layer proposes -> gate validates route identity -> fast layer executes
```

The candidate selector may still be expensive, but it no longer blocks the
controller's event ingress or lets an obsolete result commit.  This gives the
benchmark a measurable architectural quantity: planning-cycle lock skips,
proposal generations, terminal invalidations, and stale-proposal discards.

## Regression evidence

`test_planning_contract.py` proves that:

1. an execution terminal invalidates an old snapshot;
2. a changed route lease rejects a candidate even when its geometry is valid;
3. concurrent invalidation and commit are linearized, so the old route is
   either committed before the terminal or discarded before publication.

The change does not alter TEB speed, obstacle weights, frontier radii, detector
thresholds, SLAM settings, or experiment method names.  A full Level 4 run
and the declared frontier/Place-Portal ablations are still required before
claiming lower completion time or smoother motion.

## Next measurement

The next clean runtime should compare the pre-change baseline log with the
new architecture using the same world and seed.  In addition to the existing
navigation metrics, aggregate:

- `planning_contract.proposal_generation`;
- `planning_contract.cycle_skipped`;
- terminal invalidations and stale-proposal discard events;
- terminal-to-route-command and terminal-to-path-adoption latency.

The expected falsifiable result is a reduction in terminal-to-successor delay
without changing the controller or graph policy.  If selection remains the
dominant CPU cost, the next architectural increment should be an incremental
candidate compiler, not another runtime timeout.
