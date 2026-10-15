# Task-Conditioned Place/Portal Belief

Date: 2026-09-06

## Research problem

The former explorer optimized a local frontier score. That is sufficient for
mapping nearby free space, but it does not express the mission question:

> Which physical place should be observed next, and which doorway is the next
> information-bearing action?

As a result, a robot can spend a long time generating nearby frontier points in
one office, while an unobserved room containing the target remains outside the
search policy. Replacing one numeric weight with another would not change this
failure mode.

## Architectural boundary

The implementation adds a task-conditioned semantic belief beside the existing
topological memory:

```text
/lste/task + /lste/detections
            |
            v
  SemanticPlaceBelief(task_version, physical_place_id)
            |
            v
  finite-state Place action policy
            |
            v
  existing Place/Portal/WorkItem selector -> Navfn -> TEB
```

The semantic module does not create places, certify doors, publish poses, or
change TEB/Navfn parameters. Physical legality remains owned by the existing
portal admission and route validation stages.

`global_frontier_portal_belief.py` adds the missing intermediate identity for a
doorway. A certified edge now has an in-process lifecycle
`certified -> selected -> crossed/failed`, keyed by source Place and physical
gate/destination coordinates. A new SLAM snapshot can update the map geometry
and still reuse that physical hypothesis. A failed action is recorded rather
than deleting the hypothesis, so later policy work can distinguish a known
hard doorway from a never-tested one.

`global_frontier_portal_egress.py` closes the complementary failure case: when
SLAM merges a room and its corridor so no fresh label transition is visible,
the remembered entry edge can produce one reverse egress action. It is admitted
through the existing directional portal check and Navfn validation, then the
normal departure transaction closes the source Place. The fallback cannot
reopen local WorkItems or invent a new Place.

## Belief state

`global_frontier_semantic_belief.py` is ROS-independent and stores evidence by
the durable `Place` identity, never by a transient SLAM component label. A task
is identified by both `task_id` and the deterministic `task_version` already
used by Goal Manager.

The task-specific target stream now has a second, stricter ledger in
`global_frontier_target_belief.py`. Generic labels such as `monitor` are useful
context, but a target observation is kept under
`(task_version, place_id, work_item_id, portal_id, target_track_id)`. A committed
visual segment also records a normalized bearing and the odom pose at which the
ray was observed. If the detector loses the object for a few frames, this
record remains; changing the task version clears only this task belief and does
not erase the physical Place/Portal/WorkItem graph.

The task message is normalized into three concept sets:

- `target_terms`: the target name and attributes;
- `context_terms`: related structures, key objects, environment priors, and
  the left/right context objects;
- `negative_terms`: clues that should not be treated as positive context.

Small domain aliases (`mug`/`cup`, `computer monitor`/`monitor`,
`table`/`desk`) make the policy independent of the detector backend. Target
evidence is accepted only from `target_dets`; an environment detection such as
`other cups` cannot claim the target.

## Finite-state policy

For an observed physical Place, the policy returns one of four states:

1. `bootstrap_observation`: no task or no completed viewpoint exists yet;
2. `target_place`: target evidence belongs to this Place;
3. `context_place`: task-related context evidence belongs to this Place and
   local `ObservationWorkItem`s remain unresolved;
4. `expand_unobserved_portal`: the Place has no target evidence, or its local
   WorkItems are complete, so a certified Portal to an unobserved Place is
   preferred before another unrelated local frontier.

This is lexicographic architecture, not a new reward function. There is no
semantic distance weight, confidence threshold, timer, or map-coordinate prior
in the new policy. If no valid certified portal exists, the old safe frontier
selection remains the fallback.

The policy is enabled only by the full
`place_portal_workitem` experiment contract. The existing `frontier`,
`frontier_distance_dedup`, and `place_portal` methods remain unchanged for
baseline and ablation comparisons.

## Evidence and lifecycle

The global frontier node subscribes to `/lste/task` and `/lste/detections`.
Detection labels are associated with the current physical Place. Frames that
arrive before the initial Place bootstrap are held briefly and attached after
the first map-derived Place is created. A task update resets the semantic
ledger, so evidence from a previous task version cannot leak into a new run.

Every newly meaningful semantic update is emitted as a
`semantic_place_evidence` status event. Route selection emits
`semantic_topology_expansion_selected` when the finite-state expansion branch
actually chooses a Portal. The normal status payload also includes the current
semantic state, making this decision auditable in the timestamped benchmark
logs.

`target_belief_updated` and `target_belief_geometry_bound` are emitted at the
same ROS boundary. Goal Manager remains the sole owner of executable goals; it
publishes `target_segment_committed` with `target_bearing_odom` and
`target_observation_origin_odom`, while the explorer only records that evidence
against the current physical Place. This keeps perception, topology, and
control as separate transactions.

Goal Manager also keeps the source-stamped target rays for the active visual
track. A track can acquire navigation ownership only after the newest ray has
two forward intersections with earlier rays; parallel or backward rays remain
candidate evidence. This is a geometric consistency contract, not a higher
detector-score threshold, and makes false positives observable in the same
arbitration log.

Covered Places are not globally deleted from the graph. A covered label remains
eligible as transit only when the current occupancy snapshot shows an unknown
boundary on that vertex; the Portal ledger then requires that the destination
vertex or an attached unbound edge can still lead to unresolved work. This
distinguishes a necessary return to a corridor from a useless covered-room
cycle without relying on a timeout or a distance-based exception.

Portal failures are also durable negative evidence for that physical gate. A
fresh SLAM snapshot cannot silently reopen a failed edge; only the explicit
one-edge recovery transaction may retry it, and a materially different physical
hypothesis gets a new identity. This prevents failure handling from becoming an
unbounded retry loop.

`summarize_run.py` reduces these events into
`semantic_place_evidence_events`, `semantic_topology_expansion_events`, and
`portal_hypothesis_event_counts`, so the architecture can be compared without
parsing raw log text by hand.

## Relation to established research

The design is informed by recent open research rather than an end-to-end
replacement of the existing controller:

- [Clio](https://arxiv.org/abs/2404.13696) keeps only task-relevant semantic
  entities and relations through an information bottleneck. Its task-conditioned
  representation motivates filtering evidence by the active mission.
- [OneMap](https://arxiv.org/abs/2409.11764) accumulates probabilistic semantic
  evidence over time instead of replacing a place label with the newest frame.
  Our current ledger is the small ROS-1-compatible first step toward that
  persistent belief.
- [OSG Navigator](https://arxiv.org/abs/2508.04678) separates places from
  connectors. This maps directly to the `Place -> DirectedPortal` boundary.
- [VLFM](https://arxiv.org/abs/2312.03275) demonstrates semantic valuation of
  frontier actions, but still treats frontier points as the dominant state.
  LSTE keeps the useful valuation idea while making the durable state a place
  and a portal.
- [SG-Nav](https://arxiv.org/abs/2410.08189) and [UniGoal](https://arxiv.org/abs/2503.10630)
  reinforce graph matching, explicit phase transitions, and re-observation
  when visual evidence conflicts.
- [RayFronts](https://arxiv.org/abs/2504.06994) and the newer [R2F](https://arxiv.org/abs/2603.08475)
  motivate treating a boundary as a persistent directional hypothesis rather
  than a new point after every SLAM update. LSTE's existing WorkItem lineage
  is the compatible ROS-1 form of that idea.

These works share a separation used in active and semantic exploration:

- frontier exploration supplies geometric information boundaries;
- active semantic exploration keeps a belief over where mission-relevant
  observations are likely;
- topological navigation treats doorways as graph edges and rooms as durable
  nodes rather than repeatedly optimizing raw map points.

The project-specific contribution is the strict interface between these ideas:
semantic belief may order already legal graph actions, but it cannot bypass
directed portal certification or turn a temporary map component into a Place.
That boundary makes the method testable against ordinary frontier and
distance-deduplicated baselines.

## Current implementation slice

The current slice deliberately uses qualitative evidence states rather than a
learned or hand-tuned utility function:

```text
unobserved -> context_supported -> target_supported -> target_confirmed
                         \\-> contradicted
```

The transition is monotonic within one task epoch; a missing frame does not
delete a state. Candidate ranking still uses the existing deterministic
Navfn-valid route score as a tie-break inside a legal action class. A future
log-odds or information-gain variant should be evaluated as an explicit
ablation, not silently introduced as another parameter.

The follow-up architecture slice makes source-side doorway observation a
separate persistent obligation. `PortalProbeLedger` stores one directional
probe per physical gate, survives map-cell motion, rehydrates a current safe
viewpoint when the frontier arc moves, and settles its associated WorkItem only
after directed Portal evidence certifies the unknown side. A probe route failure releases an Attempt
without creating a new Place or WorkItem. This keeps semantic evidence, Portal
legality, and controller execution as three separate transactions.

When source-side evidence is reached but the destination core is not yet
visible, the same physical probe is rehydrated as a `destination_active`
Attempt from another safe viewpoint. The failed viewpoint is recorded in the
probe lineage; the durable Portal/WorkItem identity is retained. This gives the
active-perception phase a real state transition instead of repeatedly minting
frontiers at the same doorway.

The next slice adds a ROS-free `PortalProbeValue` contract and a small adapter
at the planner boundary.  All legal probe candidates are retained before the
legacy score buckets discard alternatives.  Hard constraints are evaluated
first, then candidates are ordered by explicit action category and compared by
Pareto dominance over independent evidence dimensions.  A high information
count cannot compensate for an invalid route, a stale Place owner, or a reused
viewpoint.  The full method emits the selected category, Pareto-front size,
and rejection reasons; baseline methods do not use this layer.

## Regression evidence

The policy has ROS-independent tests in
`src/lste_topo_access/test/test_semantic_belief.py` and a selector-level test
that verifies a certified Portal is chosen before unrelated local work only
when the full method contract enables the semantic layer. Run:

```bash
python3 -m pytest -q src/lste_topo_access/test
```

Do not claim Level 4 task completion from these unit tests. The required next
evidence is a clean Level 4 run with the resolved startup configuration, a
`semantic_topology_expansion_selected` event, and a baseline/ablation summary
covering room re-entry, WorkItem duplication, coverage, target success, path
length, completion time, failures, and motion smoothness.

## Scope of this slice

This change is an architectural increment, not the finished paper method. It
implements task-conditioned Place evidence, finite-state graph actions,
persistent Portal hypotheses, and durable source-side Portal probes. The next
research increment should add
log-odds target belief and direction-conditioned WorkItems so an unknown Portal
can be ranked by expected task-information gain. Those additions must preserve
the same physical admission boundary and be evaluated against the existing
benchmark baselines before any completion claim.
