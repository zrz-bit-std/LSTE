# Target Transaction Ownership Guard

Date: 2026-09-07

## Problem

The persistent TEB execution path has one streaming MoveBase lease. A visual
target or an information-gathering parallax action can therefore be active
while Global Frontier continues publishing newer `priority=0` route commands.
The old bridge accepted those commands unconditionally and adopted the
frontier path in place. The controller then moved away from the target action,
while GoalManager still believed that the target observation was active.

The same race existed one layer earlier: an unconfirmed candidate could expire
while its parallax or room-reacquisition route was still executing. GoalManager
then published a frontier goal before the target route terminal, so the next
terminal was classified against the wrong source.

## Invariants

1. A target or target-information route owns the persistent controller until
   its matching terminal boundary.
2. Candidate evidence timeout cannot clear an active parallax or reacquisition
   route.
3. A lower-priority frontier transaction may be admitted only after the target
   route terminal, and it must not mutate the target transaction metadata while
   waiting.
4. A terminal for an unconfirmed observation route releases the GoalManager
   target owner before a new frontier transaction is published.
5. Transport action identity and semantic mission transaction identity are
   logged separately. A persistent stream may retain one action generation
   while adopting several mission transactions.

## Implementation

- `goal_manager_target_follow.py` keeps candidate memory while either
  `target_parallax_goal` or `target_reacquire_goal` is active.
- `goal_manager_teb_callbacks.py` treats terminals from
  `target_candidate_room_search`, `target_reacquisition_sweep`,
  `target_viewpoint_retry`, and `target_candidate_pending` as observation
  terminals. It clears the temporary route and returns ownership to the
  frontier planner without erasing the Place-level target obligation.
- `teb_goal_bridge_mission_input.py` rejects a frontier command while a target
  owner is active. The rejection is transaction-aware and emits
  `mission_goal_ignored` with `reason=higher_priority_target_active`.
- `teb_goal_bridge_persistent_target_approach.py` records the exact target
  transaction terminal. The next frontier command consumes that boundary.
- `teb_goal_bridge_action_client.py` stores the semantic transaction in the
  immutable action contract. `active_goal_transaction_id` is updated on
  persistent path adoption, while `action_generation` remains transport
  identity.

## Regression Evidence

ROS-free tests cover:

- candidate timeout during active parallax;
- candidate timeout during active reacquisition;
- target observation terminal releasing the temporary owner;
- a frontier transaction arriving during active target parallax;
- semantic transaction identity in the bridge status event.

The full suites pass with `808` topology/access tests and `8` core tests (one
pre-existing skip). The targeted physical run is:

```
runtime/office_building_benchmark/logs/20260907_234915
```

On `level_4` with the `target_entry` diagnostic profile:

- target parallax selected at about `18.76 s`;
- frontier transaction rejected at `20.96 s` with
  `higher_priority_target_active`;
- target follow confirmed at `30.58 s`;
- task completed at `31.16 s`;
- no frontier adoption occurred during the active target transaction.

This is a short target-isolation proof, not a claim that full Level 4
exploration is solved. The next experiment should replay the same owner
ordering on a small deterministic target-room world before comparing complete
office-building baselines.
