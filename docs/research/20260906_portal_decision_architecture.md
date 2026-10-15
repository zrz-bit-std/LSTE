# Task-Conditioned Evidence-Gated Portal Decisions

## Problem

The old exploration policy allowed a single scalar frontier score to decide
which doorway to cross.  That score mixed information, distance, structure and
heading.  A small change in one coefficient could therefore send the robot
through a covered room, revisit a failed doorway, or ignore a task-relevant
branch.  These are graph-state errors, not controller-tuning errors.

The current method keeps three different questions separate:

1. **Is the action legal?**  Place ownership, directed Portal certification,
   Navfn reachability and the Portal/WorkItem ledgers answer this question.
2. **What information action is it?**  A target-relevant Portal, a new Place,
   an information-bearing boundary and covered transit have different semantic
   meanings.
3. **Which legal action is preferable?**  Candidates in one semantic class are
   Pareto-pruned; a stable identity tie-break selects one representative.

Navfn and TEB remain execution modules.  They do not change the Place graph or
invent a new semantic action after a route has been committed.

## Policy

The full `place_portal_workitem` and its strict local-work ablation use the
following class order:

```text
target_relevant > new_place > information_gain > covered_transit
```

Within one class, the immutable evidence vector is:

```text
task_relevance       maximize
novelty              maximize
information_gain     maximize
clearance             maximize
path_cost             minimize
risk                  minimize
```

Candidate `B` dominates candidate `A` when it is no worse on every dimension
and strictly better on at least one.  Dominated candidates are removed.  An
incomparable trade-off is retained on the Pareto front and resolved by
`(portal_id, endpoint_x, endpoint_y)`, never by a hidden weighted sum.

The implementation is ROS-free in
`src/lste_topo_access/scripts/global_frontier_portal_decision.py`.  The ROS
adapter in `global_frontier_portal_selection.py` supplies facts already proven
by the existing admission stages.  A semantic hint is used only as a discrete
forward predicate; it is not converted into a distance reward.

## Relation to Recent Work

The design is intentionally a small, testable synthesis rather than a claim to
reimplement another paper:

- [SAP-Nav](https://arxiv.org/abs/2608.12707) separates room semantics from
  active viewpoint verification.  LSTE applies the same separation to a
  source-side Portal probe and its crossing transaction.
- [AECNav](https://arxiv.org/abs/2608.10817) consolidates positive, confuser and
  miss evidence at the cluster level.  LSTE retains task evidence by durable
  Place and WorkItem identity, while keeping detector frames short-lived.
- [Concept-Guided Exploration](https://arxiv.org/abs/2608.23650) uses room and
  door concept agents with explicit action preconditions.  LSTE represents the
  same boundary as `Place -> DirectedPortal -> Place` and enforces it with the
  graph executive.
- [R2F](https://arxiv.org/abs/2603.08475) treats frontiers as directional
  semantic hypotheses.  LSTE's Portal gate, unknown-side normal and directed
  crossing evidence are the 2-D, ROS-compatible form of that idea.
- [SCOUT](https://arxiv.org/abs/2606.06721) avoids repeatedly observing an
  object after information gain has saturated.  LSTE records probe/work-item
  completion and does not recreate a resolved descendant after SLAM relabels
  the grid.

These references motivate the architecture; they are not used as unverified
runtime dependencies.  The repository must still report results only from
reproducible trials.

## Invariants

- A Portal candidate enters the decision layer only after physical admission.
- A covered transit edge cannot outrank a legal route to a new Place.
- A failed Portal remains negative evidence and cannot be silently recreated.
- A SLAM snapshot can move a route endpoint but cannot change a Place or
  WorkItem identity.
- A decision is replayable from its structured `portal_decision` event.

## Verification

`test_portal_decision.py` covers category precedence, dominance, incomparable
trade-offs, deterministic tie-breaking and malformed values.  The benchmark
reducer additionally accepts both real event orders:

```text
place_commit -> arrival -> finished
place_commit -> finished -> arrival
```

The latter is required because the map callback and transaction callback are
independent ROS event streams.  A delayed arrival is accepted only when its
transaction history proves the same route reached `place_commit`; an unrelated
arrival remains a `phantom_place` violation.

The policy is an architectural experiment arm, not a completion claim.  It
must be compared against the declared frontier and Place/Portal ablations in
the paired benchmark before any performance conclusion is made.

## Target Approach Transaction

The same separation is used for the semantic target itself.  A confirmed
visual track starts `TargetApproachTransaction` in
`goal_manager_target_approach_transaction.py`.  A TEB terminal moves it to
`reobserving`; a fresh target frame moves it back to `approaching`.  The
transaction ends only on explicit `task_done`, target-evidence expiry, or a
route failure.  `target_cache_max_advances` may still describe a short-horizon
prefetch optimization, but it is no longer allowed to end the mission or
release a confirmed target to frontier exploration.

This makes the previous failure observable: a target can be detected in a
small box, several validated approach viewpoints can be reached, and the
system will hold/reobserve rather than silently switching to a frontier just
because an integer advance budget was exhausted.

The mission-level target ledger has matching terminal states: `completed` is
written by `task_done`, while `released` is written only by an explicit
track-loss/room-claim release carrying the target identity. Empty detector
frames by themselves leave the work `unresolved`.
