# Portal Evidence Graph Architecture

Date: 2026-09-06

Status: implemented architecture slice; no navigation-success claim.

## Research question

Can an unknown-office explorer avoid repeated room entry and repeated doorway
goals by making the next information obligation a durable graph action, rather
than selecting a new map frontier on every SLAM update?

The question is deliberately architectural. TEB, Navfn, detector thresholds,
velocity limits, and SLAM settings are held constant while the representation
and action contract are compared.

## The proposed method

LSTE keeps three durable object types:

```text
Place      physical observation area; owns local work
Portal     directed physical doorway hypothesis/edge
WorkItem   one unresolved observation obligation owned by a Place
```

A PortalProbe is an information action attached to a physical Portal opening.
It is not a Place and it is not a generic frontier point. Its lifecycle is:

```text
pending
  -> source_active -> source_arrived
  -> destination_active -> observed/certified
  -> rejected
```

Route failure releases an Attempt but does not create another WorkItem or
Place. A temporary map projection failure becomes `awaiting_projection`; a
later physical observation rehydrates the same identity. A Place is created
only after a physically verified Portal crossing and destination arrival.

The new execution contract is explicit:

```text
source_view -> destination_view -> portal_crossing -> Place commit
```

The map endpoint remains a short-lived `frontier_endpoint` for Navfn/TEB. The
mission metadata carries `portal_probe`, `portal_probe_id`, and
`portal_probe_phase` so a map-frame geometry update cannot erase the evidence
phase. Materialization rejects a candidate whose probe identity or phase does
not match the prepared graph action.

## Why this is different from score tuning

An ordinary frontier selector answers one numerical question: which reachable
point looks best now? That is insufficient when the same physical doorway is
represented by different SLAM cells across time. The graph executive first
answers a discrete question:

```text
Is the legal action bootstrap, local work, retry, source probe,
destination probe, portal crossing, transit, or hold?
```

Only after that answer does the geometric layer choose a current safe cell.
Therefore:

- coordinates may change without changing action identity;
- local WorkItems cannot preempt destination evidence;
- a covered Place remains transit-capable but cannot reopen local work;
- a failed viewpoint creates an Attempt for the same obligation;
- an uncertified opening cannot create a destination Place.

The candidate Pareto selector is only a tie-break within the legal action
class. It is not a replacement policy and adds no runtime coefficient.

## Fast/slow boundary

The fast layer is reactive:

```text
SLAM -> costmap -> Navfn -> TEB -> route status
```

The slow layer is event-driven and identity-based:

```text
route/place/portal/work-item fact -> evidence contract -> graph action
```

`EvidenceEventGraph` and `EvidenceContractSnapshot` make this boundary
replayable from the status stream. A route terminal is not automatically an
observation fact; the corresponding evidence event must be emitted. This
prevents a controller stop, a missing detector frame, or a transient frontier
from silently completing a graph obligation.

## Recent work consulted

The following public papers were checked through the arXiv/Semantic Scholar
APIs on 2026-09-06. They motivate design choices; LSTE does not claim to
reproduce their results or import their code.

| Work | Date | Relevant idea | LSTE boundary |
| --- | --- | --- | --- |
| [Concept-Guided Exploration: Building Persistent, Actionable Scene Graphs](https://arxiv.org/abs/2608.23650) | 2026-08 | Asynchronous room/door concept agents, incremental validation, hierarchical constraints | Keep the room/door graph explicit, but use a ROS-1-compatible single executive and physical Portal certificates |
| [RGB-only Active 3D Scene Graph Generation for Indoor Mobile Robots](https://arxiv.org/abs/2605.18197) | 2026-05 | Active viewpoints selected from a partial semantic/geometric graph | Treat visual evidence as a reason to request a viewpoint, never as direct motion authority |
| [Active Semantic Perception](https://arxiv.org/abs/2510.05430) | 2025-10 | Scene-graph hypotheses and information gain for choosing views | Retain information-bearing views, but do not hallucinate an unobserved room or doorway |
| [Where Did I Leave My Glasses?](https://arxiv.org/abs/2509.19851) | 2025-09 | Persistent object identity and active map maintenance under semi-static changes | Bind task evidence to mission version and Place, not image pixels or transient map cells |
| [SCOUT](https://arxiv.org/abs/2606.06721) | 2026-06 | Uncertainty-guided semantic coverage and exploration | Use uncertainty/value only after graph legality; it cannot bypass Portal/Place ownership |
| [UniGoal](https://arxiv.org/abs/2503.10630) | 2025-03 | Graph matching, explicit phases, correction and blacklist recovery | Preserve finite action phases and explicit recovery, while keeping recovery tied to a physical edge |

The retained contribution is the stricter contract between these ideas:
semantic or geometric uncertainty can select an information-bearing viewpoint,
but only physical evidence can promote a Portal and create a new Place.

## Implementation map

- `global_frontier_evidence_contract.py`: typed requirements, facts, actions,
  reducer, and compatibility vocabulary.
- `global_frontier_graph_route_planner.py`: deterministic Place-graph BFS and
  phase-aware action plan.
- `global_frontier_graph_route_transaction.py`: prepared-intent to
  materialized-candidate admission; rejects identity/phase substitution.
- `global_frontier_portal_probe_ledger.py`: physical probe identity and
  source/destination Attempt lifecycle.
- `global_frontier_portal_probe_lifecycle.py`: ROS route boundary and explicit
  `portal_destination_view_observed` event.
- `goal_context.py`: route identity that survives map-coordinate updates.
- `global_frontier_event_graph.py`: bounded, replayable fast/slow event
  projection.

## Falsifiable predictions

With the same world seeds and controller settings, the full method should
reduce these failure modes relative to ordinary frontier and distance-dedup
baselines:

1. repeated entries into an already completed Place;
2. redispatches of one completed WorkItem;
3. source-side probes that are mistaken for destination evidence;
4. route identity changes caused only by SLAM endpoint movement.

The method may still fail to cross a doorway when the current costmap cannot
materialize a safe destination-side viewpoint. That is a reachability failure,
not evidence that a new room exists. Such cases must be reported as blocked
Portal obligations and included in the limitation analysis.

## Required evaluation

Run paired seeds for:

- ordinary frontier;
- distance-deduplicated frontier;
- Place/Portal/WorkItem without branch-first;
- Place/Portal/WorkItem with branch-first;
- the full phase-aware evidence contract;
- ablations removing WorkItem lineage, Portal certification, or phase identity.

Report repeated Place entries, effective exploration time per Place, repeated
WorkItem dispatches, coverage, semantic-goal success, path length, completion
time, collision/failure rate, route mismatch count, Portal crossing rate, and
motion smoothness. Every trial must retain its timestamped process logs,
resolved configuration, Git revision, seed, and replayable video/result.

The current Level4 logs are development evidence only. Until the paired suite
and ablations are complete, this work must not be described as a completed
navigation result or as superior to a baseline.
