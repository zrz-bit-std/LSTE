# Portal Probe Value and Pareto Selection

Date: 2026-09-06

## Purpose

The existing explorer correctly separates a source-side Portal probe from an
ordinary frontier route, but its remaining selection path still uses a scalar
frontier score. That score is useful as a legacy geometric tie-break, yet it
cannot express the architectural question for a probe:

> Which physically legal doorway observation gives the most useful next
> evidence without hiding a safety or execution failure inside a weight?

This increment defines the value boundary without changing a ROS node. The
module is `src/lste_topo_access/scripts/global_frontier_portal_probe_value.py`.

## Selection pipeline

```text
current snapshot candidates
          |
          v
hard physical constraints
          |
          v
explicit action-category order
          |
          v
Pareto front inside the selected category
          |
          v
stable identity tie-break -> one candidate
```

The stages are intentionally not interchangeable:

1. A candidate that cannot be routed, is not collision-free, belongs to a
   different physical Place, has no valid physical probe identity, is no
   longer available in the probe ledger, or repeats a failed viewpoint is
   rejected before value comparison.
2. The first category with any feasible candidate wins. The default order is
   the existing action-tier contract: `viewpoint_retry`,
   `target_direction`, `adjacent`, `probe`, `local`, `unconstrained`.
   Experiments may pass an explicit sequence or mapping, so the policy is
   visible in configuration rather than hidden in a score coefficient.
3. Only candidates in that category are compared. A candidate from a lower
   category cannot defeat a higher-priority recovery or information action
   merely by having a better geometric value.
4. A candidate dominates another only when it is no worse on every declared
   objective and strictly better on at least one. Incomparable candidates are
   both retained on the Pareto front. A stable ID/viewpoint key chooses one
   representative only after that front has been computed; the tie-break is
   not an objective and is not a weighted utility.

## Immutable contracts

`PortalProbeConstraints` carries only facts owned by existing boundaries:

| Constraint | Evidence owner | Meaning |
| --- | --- | --- |
| `route_reachable` | Navfn/route validation | The current endpoint has a valid route. |
| `collision_free` | costmap/safety validation | The endpoint is safe to dispatch. |
| `source_place_current` | Place memory | The probe belongs to the active physical Place. |
| `physical_identity_valid` | PortalProbeLedger | The doorway identity is not a transient SLAM cell. |
| `probe_available` | PortalProbeLedger | The obligation is pending and not already active/terminal. |
| `viewpoint_available` | WorkItem/probe lineage | This physical viewpoint has not already failed. |

`PortalProbeValue` has six independently reported dimensions:

- maximize `information_gain`;
- maximize `task_relevance`;
- maximize `novelty`;
- maximize `clearance`;
- minimize `path_cost`;
- minimize `risk`.

No dimension is normalized against the others and no coefficient is exposed.
This is deliberate: a change in units or detector calibration must not
silently change the semantic action class. If a future study changes the
objective set, it should be a named ablation with its own result table.

## Why this is an architectural change

The former pattern lets a sufficiently high information or structure score
compete with route legality. The new contract makes the order of reasoning
explicit:

```text
physical legality -> action meaning -> multi-objective preference
```

This preserves the current Place/Portal/WorkItem ownership boundary. Semantic
or information evidence can influence `PortalProbeValue`, but it cannot create
a Place, bypass directed Portal certification, or revive a failed probe. The
module is pure and therefore suitable for deterministic replay and ablation
tests without Gazebo or ROS time.

## Current scope and integration boundary

The selector is integrated at the full-method source-side probe boundary.
`global_frontier_portal_probe_adapter.py` converts every probe route retained
before score-pool compression into `PortalProbeCandidate` values, and the
planner publishes `portal_probe_value_selected` with the selected category,
Pareto-front size, and hard-constraint rejection counts. A Navfn-rejected
viewpoint is excluded from the next pass while the same physical probe remains
eligible through a different viewpoint. A source-arrived probe can enter a
distinct `destination_active` Attempt from a new viewpoint; it keeps its
identity until directed Portal evidence is certified. The selector does not
replace the certified Portal transition selector: probing and crossing remain
separate actions. Navfn, costmap, the PortalProbeLedger, and TEB remain the
owners of execution facts.

Keeping the adapter separate makes baseline comparisons possible:

- existing scalar frontier ranking;
- hard constraints plus category order;
- hard constraints plus category order plus Pareto probe value.

The tests in `src/lste_topo_access/test/test_portal_probe_value.py` cover hard
constraint precedence, category precedence, dominated/incomparable values,
non-finite rejection, deterministic tie-breaking, and explicit experiment
orders. `test_portal_probe_adapter.py` covers the planner boundary and the
Navfn rejection handoff. They are not evidence of complete Level 4
navigation; that still requires repeated clean benchmark runs with timestamped
logs and truth metrics.
