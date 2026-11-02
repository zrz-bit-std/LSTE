# Semantic junction Gazebo tests

Run these tests on the lab Ubuntu host from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
bash scripts/tests/targeted_navigation/run_semantic_junction_suite.sh
```

The default suite runs `semantic_t_junction`, `semantic_l_junction`, `semantic_cross_junction`, and `semantic_t_junction_rotated`. Each run starts Gazebo, online SLAM, GP branch discovery, WeDetect semantics, Portal observation/authentication, and Place-scoped WorkItem exploration. It writes tree, authentication, semantic-lifecycle audits and a Portal route PNG under `runtime/gp_branch_development/semantic_junction_suite_<timestamp>/`.

To run a single scene and adjust its wall-clock limit:

```bash
SEMANTIC_CASES=semantic_t_junction_rotated GP_SUITE_SECONDS=1500 \
  bash scripts/tests/targeted_navigation/run_semantic_junction_suite.sh
```

The rotated semantic T uses a focused T layout so the robot reaches the target junction before entering a semantic room. Regenerate the rotated scene with:

```bash
python3 scripts/tests/targeted_navigation/generate_focused_semantic_t.py
python3 scripts/tests/targeted_navigation/generate_rotated_semantic_world.py \
  worlds/targeted_navigation/semantic_t_junction_focused.world \
  worlds/targeted_navigation/semantic_t_junction_focused_rotated.world
```

The four-scene results and current audit JSON/PNG artifacts are collected in:

`runtime/gp_branch_development/semantic_portal_goal_final_20261024/`

Run the focused code regression suite with the commands in that directory’s `README.md`. For reproducible background and object placement, see each world’s matching JSON manifest and `*_geometry_result.json` under `worlds/targeted_navigation/`.

## Place-scoped TSP integration

Normal exploration remains WorkItem-driven. GP + online SLAM temporarily owns
Portal observation/authentication; this freezes ordinary WI dispatch. Once a
certified entrance WI crosses its Portal, the existing crossing transaction
creates the destination Place. Only that Place's admissible WIs enter the open
TSP optimizer. Room semantic priority, branch isolation, room closure, return to
the junction, and resumption of the suspended parent WI keep their existing
local behavior.

The pose-cost optimizer is copied from lumole's
`20260805_exploration_navigation_dev` commit
`8125577a62a0e9123fe4041329212bc5bbd74f03`. It includes translation and heading
cost, 2-opt segment reversal, and Or-opt relocation. The local adapter keeps
robot pose, WI anchors, WI normals, and semantic room scopes in their original
physical odom frame before sorting. Rigid SLAM reprojection cannot change room
membership; map projection remains at route materialization.

Acceptance requires the scene's actual arm count: **T = 3, L = 2, cross = 4,
rotated T = 3**. The TSP audit checks actual `work_item_dispatched` events as well
as planned tours; a sibling WI cannot pass by merely recording the intended
head in the plan. The semantic lifecycle audit also compares actual room-entry
order with semantic priority and rejects repeated entry.

To audit or plot an existing run (no Gazebo or ROS master needed):

```bash
RUN=runtime/targeted_navigation/t_junction_small/logs/<timestamp>
METRICS="$RUN/$(basename "$RUN")_navigation_metrics.log"
python3 scripts/tests/targeted_navigation/audit_place_tsp.py "$METRICS" \
  --output "$RUN/place_tsp_audit.json"
python3 scripts/tests/targeted_navigation/audit_semantic_room_lifecycle.py \
  "$METRICS" --center 0 0 --expected-rooms 2 \
  --output "$RUN/semantic_lifecycle_audit.json"
python3 scripts/tools/plot_place_tsp.py "$METRICS" \
  --world worlds/targeted_navigation/semantic_l_junction.world \
  --center 0 0 --expected-rooms 2 --output "$RUN/place_tsp.png"
```

`plot_place_tsp.py` preserves retries and writes a JSON data/provenance file
beside the PNG. Grey lines are recorded odometry; colored overlays are recorded
room exploration intervals. Numbered squares are dispatched viewpoints, not
hypothetical TSP travel segments. Gaps above two simulation seconds are not
connected. Missing geometry models are listed in the JSON.

For fresh Gazebo runs, use one ROS master per scene, avoid simultaneous Gazebo
instances, and keep running shell files unchanged until their process exits.
The runner fails if a roslaunch child crashes; `roslaunch` remaining alive is
insufficient. Python native-crash tracebacks are enabled for diagnosis. Such a
run must be repeated with healthy perception/planning processes; it is not a
passing scenario even if some earlier audits were green.

### 本轮 Place TSP 的专项完成条件

`run_semantic_junction_suite.sh` 会为每个场景传入实际期望方向。测试运行器
每 30 秒调用 `check_semantic_junction_goal.py`，原有树结构、门洞认证、房间
生命周期、实际 TSP 派发四类审计全部通过后正常停止，记录
`semantic_junction_goal_complete`，并保存 `<timestamp>_semantic_goal.json`。
该结果表示本轮路口任务通过，不表示整张 world 的普通探索已结束。
独立 `run_t_junction.sh` 未设置 `T_JUNCTION_GOAL_HEADINGS` 时保持原运行方式。
必要子进程意外退出和最大运行时窗仍会检查；专项审计失败不会被记为成功。

场景套件的任一启动、专项审计或绘图检查失败时，套件返回非零；详细状态始终写入 `summary.log`。


Install either supported low-memory checkpoint on bit1010 with:
WDETECT_MODEL_SIZE=tiny bash scripts/models/setup_wedetect.sh
(or use base). General tasks select the detector with DETECTOR=wedetect-tiny,
wedetect-base, or wedetect-large before starting the pipeline.

WeDetect model size for semantic targeted-navigation runs is selected per run with
T_JUNCTION_WEDETECT_MODEL_SIZE=tiny, base, or large (defaults to base for the
8 GiB RTX 2070). Small-object tiled fallback is independently controlled with
T_JUNCTION_WEDETECT_TILE_SEARCH_ENABLED=true|false and defaults to false
to avoid its additional GPU-memory peak. The standard pipeline can still select
DETECTOR: wedetect-base or DETECTOR: wedetect-large in its task config.

For controller A/B diagnostics only, set T_JUNCTION_TEB_FORWARD_ONLY=false.
The installed TEB does not support min_vel_x; setting that ROS parameter has no effect.

For the known-free Navfn diagnostic, use `T_JUNCTION_NAVFN_ALLOW_UNKNOWN=false`.
This is a supported Navfn setting; it does not alone enforce Portal/Place boundaries.
The default remains true until the full scene evidence supports a production change.

### Physical observation turn evidence (2026-10-29)

A `room_facing_turn_complete` event alone does not establish the selected
clockwise/counterclockwise sweep. The suite's `turn_raw` gate independently
checks the sealed main bag's `/rbt_pose` against every reported completed
observation turn, including the original physical initial yaw, continuous
samples, signed swept angle and frozen target yaw. Bounded failed turns remain
subject to the full authentication audit; this check never settles a WI.

For suites launched from an older immutable shell snapshot, run it manually:

```bash
python3 scripts/tests/targeted_navigation/audit_portal_turn_directions.py \
  /absolute/path/to/sealed_slam.bag --output /absolute/path/to/turn_raw_audit.json
```

The runtime carries the frozen physical yaw and sweep in the existing JSON
route/intent contract, bound to route, lifecycle and graph transaction. The
supervisor executes it only for the matching frontier action in odom; a declared
but invalid contract refuses generic shortest-angle fallback. No ROS message
schema or collision/velocity budget is changed. Check the new process's source
manifest for `portal_authentication_turn_contract.py` and its producer/consumer
modules; editing source does not prove a running process loaded it.

### Same-source-epoch native Place masks (2026-10-29)

The global layer consumes `/lste/place_navigation_mask` in map. The rolling
local layer consumes `/lste/place_navigation_mask_odom` in odom. Both are built
from the same source map and immutable exact-stamp transform;
`info.map_load_time` records that source epoch, while the header records
publication time. The odom raster is conservative: a target cell is traversable
only if its entire source-cell bounding box belongs to the Place. Physical
obstacle costs are never reduced by this rendering.

A bounded mask revision releases the old ordinary route and waits for its exact
controller ACK before publishing either new mask. New ordinary dispatch still
waits for both actual master grids to contain the exclusion. The raw native
auditor independently checks both input frames, their paired source epoch,
original raw TF and every actual master cell. Future TF is not substituted.
Legacy single-mask recordings keep the older strict replay path and are not
reclassified as successes.

New suites record both mask topics in the main bag. The Portal observer also
records a continuous, driver/start-token-bound mask sidecar, including ordinary
WI intervals, and saves its source configuration and runtime topic parameters.
For an older immutable suite snapshot which did not record the new topic,
a successfully sealed sidecar can be joined without modifying either input:

```bash
python3 scripts/tests/targeted_navigation/merge_place_mask_sidecar.py \
  /absolute/main_slam.bag /absolute/native_masks_DRIVER.bag \
  /absolute/native_masks_DRIVER_contract.json /absolute/new_joined.bag
```

Only original odom-mask payloads and timestamps are added; the provenance file
hashes all input/output bags. An absent, failed or unsealed historical sidecar
cannot be reconstructed or treated as positive evidence.
