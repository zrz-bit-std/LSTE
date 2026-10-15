# Graph-First Architecture Innovation

Date: 2026-09-06

Status: branch-first implementation slice; no benchmark conclusion yet.

## Research decision

The navigation problem is not primarily a missing TEB parameter. The current
frontier pipeline lets a temporary SLAM component label influence three
different decisions at once:

1. whether a boundary is worth observing;
2. whether it belongs to the current physical place;
3. whether the robot is allowed to cross into another place.

Those decisions have different evidence requirements. The proposed architecture
therefore makes the action type a discrete graph decision before any numeric
frontier score is applied.

```text
online map snapshot -> legal graph action -> viewpoint tie-break -> Navfn/TEB
                            |
                            +-- bootstrap observation
                            +-- observe unresolved WorkItem
                            +-- retry the same WorkItem from another viewpoint
                            +-- probe a wall-bounded unknown opening
                            +-- cross a certified Portal
                            +-- hold for new topology evidence
```

The score still ranks viewpoints *inside* one legal action class. It cannot
turn an unowned local candidate into an exploration task or turn a route over
an uncertified boundary into a place transition. The full method now adds a
discrete branch-first capability: after the first observation, a certified edge
to an unobserved Place can suspend residual local WorkItems and proceed; a
covered transit edge remains strict.

The third implementation slice adds `PortalProbeValue` at the source-side
probe boundary.  The planner now retains every legal probe route before the
legacy score pools compress candidates.  A pure selector applies hard facts
first (`Navfn`/costmap reachability, current Place ownership, physical probe
identity, and unused viewpoint), then chooses an explicit action category and
keeps only Pareto-undominated values for information support, task direction,
novelty, path cost, clearance, and risk.  There is no weighted sum and no new
runtime parameter.  The selected candidate and rejected-candidate reasons are
published as `portal_probe_value_selected` and attached to `route_selected`.

This is intentionally limited to the full `place_portal_workitem` experiment
contract.  The ordinary frontier, distance-deduplicated frontier, and
`place_portal` arms still use their original score-pool behavior, so the value
layer can be evaluated as a named architectural ablation rather than silently
changing a baseline.

## The project-specific method

The durable state is a small, typed graph:

```text
Physical Place
  |-- Observation WorkItem (unresolved/resolved)
  |-- Portal hypothesis (certified/selected/crossed/failed)
  +-- Viewpoint Attempts (active/succeeded/failed)
```

The key invariant is monotonic ownership:

- an observed Place can dispatch local exploration only through an unresolved
  WorkItem owned by that Place;
- a failed viewpoint creates a new Attempt for the same WorkItem, not a new
  WorkItem and not a new Place;
- a wall-bounded unknown opening creates a `probe_portal` action, which gathers
  evidence but does not create a destination Place;
- a new Place can be created only after a certified Portal crossing has a
  physical arrival commit;
- a suspended Place remains available for graph transit and can reopen its
  retained WorkItems only when a local action is selected;
- a dormant Place remains available as graph transit, but cannot reopen local
  WorkItems.

Portal arrival association now runs before transient component matching. It
uses the durable gate and the inward doorway normal inferred from the
destination structural bounds, so a lateral SLAM/standoff change does not
create a second Place while the opposite side of the same gate remains a
different graph state.

The ROS-free policy lives in
`src/lste_topo_access/scripts/global_frontier_graph_executive.py`. It is
deliberately not a score function and has no distance, confidence, timer, or
controller parameter. The frontier selector records its decision as
`graph_action` and `graph_action_reason` in the route event, making rejected
ownership visible in experiment logs.

The second slice adds a separate source-side Portal probe ledger in
`global_frontier_portal_probe_ledger.py`. A probe is keyed by
`source_place_id + physical_gate_xy + unknown_side_normal`, not by a transient
SLAM cell. Its lifecycle is:

```text
pending -> active -> observed
                 \-> pending        (route failure; another viewpoint allowed)
                 \-> rejected       (explicit negative evidence)
```

The probe ledger is intentionally independent from the generic
`ObservationWorkItem` ledger. A completed WorkItem viewpoint cannot erase a
still-unresolved doorway hypothesis, and a successful probe settles its bound
WorkItem instead of dispatching that WorkItem a second time. If the current
frontier arc disappears after SLAM updates, `pending_portal_probe_candidates`
reprojects the physical gate and rehydrates a safe viewpoint from the current
frontier. Both ordinary candidates and explicit Portal transitions pass
through the same graph executive; a certified edge is held while the current
snapshot still exposes executable local work.

An unsuccessful source-side view is not forced back into the source phase. Its
physical record can be rehydrated as `destination_active` from a different
safe viewpoint, while the same gate/normal identity and WorkItem remain in
place. A destination-view failure therefore preserves the `source_arrived`
obligation and records the attempted viewpoint; it cannot mint a duplicate
door or silently downgrade the evidence to a generic frontier.

Reaching a source-side probe viewpoint is recorded as
`source_viewpoint_arrived`, not `observed`.  Only a later directed Portal
certificate may resolve the associated WorkItem.  This prevents a geometric
endpoint event from removing the only durable obligation for a doorway whose
unknown side has not actually been inspected.

This is a more meaningful architectural contribution than adding another
penalty weight: it changes the state/action representation and makes illegal
state transitions unrepresentable at the candidate boundary.

## External research that informed the design

The following papers were checked through the arXiv API on 2026-09-06. They
are references for individual ideas, not code dependencies or claims that LSTE
reproduces their results.

| Work | Date | Relevant idea | Boundary retained in LSTE |
| --- | --- | --- | --- |
| [SAP-Nav](https://arxiv.org/abs/2608.12707) | 2026-08 | Queryable spatial-semantic representation and active viewpoint verification | semantic evidence may request a viewpoint; Navfn/TEB still owns motion |
| [AECNav](https://arxiv.org/abs/2608.10817) | 2026-08 | Evidence-gated perception, cluster-level evidence consolidation, active evidence acquisition | detector frames update belief; they do not directly preempt a route |
| [RTNav](https://arxiv.org/abs/2608.26496) | 2026-08 | Treat inference latency and asynchronous stepping as first-class system state | future perception integration should remain asynchronous |
| [SSTG-Nav](https://arxiv.org/abs/2608.00527) | 2026-08 | Reusable metric-semantic topology and source-aware recovery standoffs | physical Place/Portal identities survive map updates within one run |
| [SCOUT](https://arxiv.org/abs/2606.06721) | 2026-06 | Uncertainty-guided traversal coupled to online semantic graph construction | uncertainty can rank legal WorkItems later, not bypass graph legality |
| [R2F](https://arxiv.org/abs/2603.08475) | 2026-03 | Direction-conditioned ray frontiers and LLM-free semantic search | WorkItem unknown-side normals are the lightweight ROS-1-compatible analogue |
| [OSG Navigator](https://arxiv.org/abs/2508.04678) | 2025-08 | Hierarchical open-world scene graph separating places and connectors | Portal is the explicit connector; transient labels are not places |
| [UniGoal](https://arxiv.org/abs/2503.10630) | 2025-03 | Graph matching, phase transitions, correction and blacklist mechanisms | finite-state action transitions and explicit recovery are kept auditable |
| [VLFM](https://arxiv.org/abs/2312.03275) | 2023-12 | Semantic valuation of frontier observations | used as future value-layer inspiration, not as a replacement for geometry |

The strongest recent convergence is architectural: semantic perception is
valuable when it is accumulated and used to choose an information-bearing
viewpoint, while a geometric planner remains responsible for reachability and
safety. LSTE's distinctive boundary is stricter: a semantic belief may order
only actions already legal in the Place-Portal graph, and only physical portal
evidence can create a new Place.

## What is deliberately not being tuned

This increment does not change TEB speed, obstacle weights, frontier clearance,
stall timers, detector confidence, or SLAM parameters. Those values remain
common execution settings for baseline comparison. A future value model must be
added as an explicit experiment arm rather than silently replacing this
finite-state policy with another collection of thresholds.

## Verification plan

The first regression suite covers the policy without ROS:

```bash
python3 -m pytest -q src/lste_topo_access/test/test_global_frontier_graph_executive.py
python3 -m pytest -q src/lste_topo_access/test
```

The next system experiment must compare the existing baseline methods and the
full method on the same seeded office-building levels. It must report graph
action counts, repeated physical Place entries, resolved WorkItem redispatches,
coverage, target success, path length, completion time, failure/collision rate,
and motion smoothness. Unit tests prove the transition contract only; they do
not prove task completion.

The clean Level 4 development runs used to validate this slice are retained
under `runtime/office_building_benchmark/logs/` and must be treated as
development evidence, not benchmark aggregates. In the latest run, the log
contained paired `portal_probe_started`/`portal_probe_settled` events, including
rehydrated probes after route failures, zero collision truth events, and no
frontier timer exceptions. It still ended without task completion, so no claim
of navigation success is made.

The post-integration run `20260906_064243` produced 11 value-selection events
over 44 candidate routes (40 hard-feasible, 14 Pareto-front members), 7 probe
route activations, and 6 `source_arrived` settlements. It recorded zero
collision events and zero resolved-WorkItem redispatches, but the robot did not
finish the semantic target task and encountered a separate Portal crossing
stall. These numbers validate the transaction and logging boundaries only;
they are not a method comparison.

## Next research increments

1. Compare `place_portal_workitem_strict` against branch-first on paired Level 4
   trials; do not merge the two method contracts in one aggregate.
2. Extend the current probe value adapter with an explicit information-gain
   estimator from ray visibility, while retaining the hard-constraint and
   Pareto boundary.
3. Add a persistent directional evidence record to each WorkItem, inspired by
   ray-frontier methods, and use it to rank probes without changing legality.
4. Add an active-view verification action for ambiguous target/context evidence,
   borrowing SAP-Nav/AECNav's evidence-consolidation boundary.
5. Add controlled ablations: no WorkItem lineage, no Portal certification, and
   no directional evidence. Keep all controller and map settings fixed.
6. Run the complete benchmark before making any claim that the method is better
   or paper-ready.

## 20260906 Target-Obligation Increment

The previous target path still had one architectural leak: a target-bearing
Place could be treated like an ordinary branch-first candidate, and
`target_cache_max_advances` could end a confirmed approach.  The new
`TargetObservationWorkLedger` and `TargetApproachTransaction` close that leak.

- target evidence creates one Place-owned mission WorkItem;
- branch-first cannot suspend that WorkItem while it is unresolved;
- `approaching -> reobserving -> approaching` is driven by target evidence and
  controller terminals, not an advance counter;
- `task_done` completes the WorkItem, while an explicit, track-identified loss
  releases it;
- detector gaps and frontier exhaustion are not completion evidence.

The corresponding regression tests are
`test_target_observation_work.py`, `test_target_approach_transaction.py`, and
`test_global_frontier_graph_executive.py`.  This is still an architectural
slice: the clean Level 4 runs so far contain no complete semantic task trial.

## 20260906 Event-Driven Evidence Graph Slice

The runtime now projects lifecycle status into a bounded
`EvidenceEventGraph`. This is the first Fast-Slow slice: reactive status stays
high-rate, while the slow graph receives one wake event for each new durable
route, Place, Portal, WorkItem, or target fact. It is replayable from the
existing JSON status stream and does not introduce a second motion policy. See
`20260906_event_driven_evidence_graph.md` for the contract and tests.

## 20260906 GraphRoutePlanner Increment

The previously identified missing graph-level route interface is now present
in `global_frontier_graph_route_planner.py`. It searches only crossed durable
Portal edges, returns the complete shortest Place path to the next unresolved
obligation, and exposes only `first_portal_id` to the existing geometry
adapter. `global_frontier_graph_route_adapter.py` preserves the ROS/Navfn/TEB
boundary and records `graph_route_plan_selected` plus
`graph_route_edge_materialized` events. WorkItem state is exposed through a
read-only ledger snapshot. The focused tests are
`test_graph_route_planner.py` and `test_graph_route_adapter.py`.

This increment does not change TEB, Navfn, detector, SLAM, speed, or frontier
thresholds. It is an executable architecture hypothesis; clean Level 4
repetitions and baseline/ablation comparisons are still required before any
performance claim.

## 20260906 Graph-Action Ownership Repair

The first Level 4 trace exposed a consistency error in the original
integration.  `GraphRoutePlanner` was called before the current frontier
snapshot reconciled its WorkItems and Portal probes.  The planner could select
WorkItem 38 while the same snapshot created/re-hydrated candidates 52 and 53;
the selector then executed one of those local candidates.  The durable graph
was being logged, but it did not own the action.

The repair is a two-stage contract:

```text
current map snapshot
    -> reconcile Place / PortalProbe / WorkItem identities
    -> GraphRoutePlan over the reconciled ledger
    -> exact identity gate
    -> endpoint projection + Navfn + TEB
```

`global_frontier_graph_route_gate.py` is ROS-free and has no score or timer.
For a `ready` plan it admits only:

- the selected `work_item_id` for `observe_local_work`;
- the selected `probe_id` for `probe_portal`;
- an explicit `portal_transition` for `cross_portal` (the existing Portal
  selector still verifies the selected durable Portal ID and physical gate).

If the identity is absent from the current temporary map, the runtime emits
`graph_route_candidate_unavailable` or `graph_route_materialization_wait` and
waits for a coherent projection.  It does not silently choose a nearby
frontier, a cached prefetch, or another WorkItem.  The full graph method now
executes this graph-owned path before terminal prefetch; baseline methods keep
their declared historical ordering, so the change remains a measurable method
boundary rather than an invisible baseline modification.

This is the architectural response to the repeated-room failure mode: a
temporary SLAM boundary can disappear, but it cannot replace a durable
obligation with a new one.  The focused regression cases are in
`test_graph_route_adapter.py`, including local WorkItem, PortalProbe, and
cross-place route identity checks.  The latest source-tree run passes 568
topology/runtime tests; this validates the contract only.  A clean Level 4
trial and paired baseline/ablation measurements are still required.

## 20260906 Durable Versus Executable Work

The first hard gate then revealed a second representation error.  A WorkItem
can remain unresolved after its temporary frontier arc is no longer visible,
so treating every unresolved record as immediately executable freezes the
robot at an obsolete boundary.  The planner now carries two facts separately:

```text
durable obligation: must eventually be resolved
current projection: rehydrated by this map snapshot and executable now
```

`visible_work_item_ids` is derived from the current physical support
reconciliation; it is not a new threshold.  The graph planner filters only the
current Place's local action set by this projection, while retaining unresolved
WorkItems in the durable ledger and in remote graph obligations.  A dedicated
`graph_obligation_candidates` channel preserves Navfn/costmap-valid viewpoints
that are closer than the ordinary frontier minimum movement distance.  The
minimum distance therefore remains a baseline tie-break, not a hidden way to
discard a named physical observation obligation.

This is a direct implementation of the recent active-exploration pattern seen
in ObsGraph and SCOUT: representation retrieval determines which evidence gap
is actionable, and geometric planning only materializes that selected view.
The LSTE-specific contribution is the stricter Place/Portal ownership rule:
retrieval can reopen an existing WorkItem, but it cannot mint a new Place or
cross an uncertified Portal.  The current Level 4 run is being used as a
counterexample-driven validation; no success or efficiency claim is made
until paired seeded trials are complete.

## 20260906 Two-Phase Graph-Action Transaction

The exact-identity gate exposed a second design question: a durable planner can
name the first pending WorkItem before the current SLAM snapshot has
rehydrated that WorkItem's frontier arc. Treating that as an unconditional
failure is too strict; allowing another candidate without recording the change
is too weak. `global_frontier_graph_route_transaction.py` makes this boundary
explicit:

```text
prepared GraphRoutePlan
        |
        v
materialized candidate -- hard evidence/identity check --> rejected
        |
        v
committed GraphRoutePlan + route_selected
```

The transaction requires an unresolved local candidate to carry the same
`obligation_id` as the prepared plan; a different WorkItem is a substitution,
not a refinement, and is rejected. It accepts `retry_viewpoint` only as a
refinement of the same observation action. A local observation can be promoted
to `probe_portal` or `cross_portal` only when the
candidate carries the corresponding Portal/probe facts; crossing also
requires the named branch-first capability, a portal-transition route, an
explicit gate, and a durable Portal identity. A prepared Portal action can
never be replaced by a local route or by another Portal identity.

The adapter publishes `graph_route_plan_reconciled` for an allowed refinement,
`graph_route_action_committed` for the final action, and
`graph_route_plan_mismatch` for a rejected substitution. `route_selected`
therefore carries the committed plan rather than the earlier tentative plan.
The benchmark reducer reports both explicit transaction events and any
remaining `route_plan_action_mismatch_count`; a zero mismatch count is a
necessary evidence condition for comparing exploration methods, not a claim
of task completion.

This is a representation change rather than a parameter adjustment. The
transaction is ROS-free and has no speed, clearance, timeout, score, detector,
or controller setting. Focused tests are in
`test_graph_route_transaction.py` and the summary regression is in
`test_benchmark_summary.py`. The current Level 4 process predates this code
and must be restarted for runtime evidence; its existing log is not silently
rewritten.

After a terminal snapshot has prepared an action, the adapter reuses that
immutable plan while materializing its durable probe or reverse egress. A new
plan is allowed only after a subsequent map/evidence snapshot, so an in-flight
transaction cannot be changed by a second planner call.

## 20260906 Projection Parking and Two-Phase Planning

The next trace exposed two forms of avoidable churn rather than a missing
controller parameter.

First, a `PortalProbe` whose physical gate could not be projected into the
current map was correctly parked as `awaiting_projection`, but its parent
unbound Portal was still returned as a fresh `probe_portal` action.  The graph
therefore replayed the same doorway until SLAM exposed a compatible cell.  The
planner now follows the typed binding all the way through: an awaiting probe
remains an unresolved completion obligation, but is excluded from the
executable unbound-Portal set.  If another doorway is available it can proceed;
if none is available the result is an explicit `PLAN_BLOCKED` hold.  A new
physical observation calls the existing ledger `observe()` path, restores the
probe's prior phase, and makes it eligible again.  No timeout, retry counter or
distance threshold was added.

Second, the selector used to create a committed graph transaction before it
had collected the current snapshot's WorkItem and probe projections, then
replace that plan a few lines later.  The adapter now exposes a read-only
`stage_only` phase.  It installs the durable identity preference needed while
collecting candidates, but leaves the previous plan and transaction untouched.
Only the reconciled second phase creates `GraphRouteActionTransaction` and
publishes `graph_route_plan_selected`.  This is a transaction-boundary change:
it removes provisional route IDs and slow-layer wake events instead of hiding
them with log throttling.

The causal boundary is recorded by
`src/lste_topo_access/scripts/global_frontier_transition.py`.  Portal and Place
events carry a JSON `transition` envelope with `from_place_id`, `to_place_id`,
`portal_id`, `transaction_id`, `route_id`, `phase`, and evidence.  The helper
does not infer a transaction from a stale embedded report when that report
belongs to another Portal.  Replay can therefore distinguish a valid
source-to-destination commit from a late failure without consulting the
mutable current Place.  This follows the event-envelope idea in recent
spatio-temporal navigation systems while keeping the runtime ROS-1 compatible.

The new pure tests cover parked-probe graph behavior, read-only graph staging,
causal arrival/departure envelopes, and cross-Portal stale-report isolation.
The source-tree suite now passes 601 tests.  This remains a structural
verification result; a clean Level 4 run and paired baseline/ablation trials
are still required before claiming better navigation.

## Research Search Update (2026-09-06)

An additional arXiv search was performed instead of relying only on earlier
references.  The most relevant architectural directions are:

| Work | Main idea | What LSTE can borrow without adding a tuning stack |
| --- | --- | --- |
| [RTNav](https://arxiv.org/abs/2608.26496) | Treat inference latency and asynchronous environment stepping as part of navigation | Keep detector/semantic updates asynchronous and prevent them from preempting an owned graph action |
| [CORE Planner](https://arxiv.org/abs/2606.29222) | Sparse visibility graph plus contextual memory for unknown environments | Use sparse Place/Portal identities as the long-term memory; keep metric execution in Navfn/TEB |
| [FPAS](https://arxiv.org/abs/2606.22838) | Adaptive frontier sampling that is sparse in open space and denser in narrow passages | Future candidate generation can be topology-aware; the current action legality remains parameter-free |
| [UNSEEN](https://arxiv.org/abs/2606.20755) | Propagate localization/mapping uncertainty into receding-horizon planning | Add uncertainty as an evidence state or admission fact, not as another weighted frontier score |
| [ViTL](https://arxiv.org/abs/2606.30696) | Compile task temporal logic to an automaton and use directional frontier scores | Represent multi-step semantic tasks as Place-owned WorkItems/DFA states while preserving Portal certification |
| [CDIS](https://arxiv.org/abs/2607.17778) | Close the loop between 2-D track identity and 3-D spatial merging | Extend target evidence with cross-view physical identity, rather than trusting per-frame detector maxima |

These results reinforce the same research boundary: stable memory and
asynchronous evidence should drive discrete information actions, while a
metric controller handles the short horizon.  LSTE's proposed contribution is
the explicit physical Portal certificate and monotonic Place ownership that
make repeated-room behavior an invariant instead of a tuned penalty.
