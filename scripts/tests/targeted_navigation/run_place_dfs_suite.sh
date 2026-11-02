#!/usr/bin/env bash
set -o pipefail
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
# Gazebo camera sensors need a rendering context even when the GUI is hidden.
if [[ -z "${DISPLAY:-}" ]] && command -v xdpyinfo >/dev/null 2>&1 \
    && xdpyinfo -display :0 >/dev/null 2>&1; then
  export DISPLAY=:0
fi
cd "$WS"
export GP_SUITE_EXECUTED_SCRIPT="$(readlink -f "${BASH_SOURCE[0]}")"
export OPENBLAS_NUM_THREADS=1
export LSTE_BODY_CLOUD_OBSTACLES="${LSTE_BODY_CLOUD_OBSTACLES:-true}"
# ROS setup supplies a default 11311 URI even when no URI was requested.
# Use a dedicated suite master, with an explicit suite-specific override.
export ROS_MASTER_URI="${PLACE_DFS_MASTER_URI:-http://127.0.0.1:11673}" ROS_HOSTNAME=127.0.0.1
BAG_PID=
DRIVER_PID=
trap '[[ -z "$BAG_PID" ]] || kill -INT "$BAG_PID" 2>/dev/null; [[ -z "$DRIVER_PID" ]] || kill -TERM "$DRIVER_PID" 2>/dev/null' INT TERM
export GAZEBO_MODEL_PATH="$WS/worlds/targeted_navigation/models:$HOME/.gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export T_JUNCTION_EXPLORATION_METHOD=place_portal_workitem
export T_JUNCTION_STOP_ON_FAILURE=false
export T_JUNCTION_SEMANTIC_PERCEPTION="${T_JUNCTION_SEMANTIC_PERCEPTION:-true}"
export T_JUNCTION_TARGET_DONE_POLICY=graph_exploration
RESULT_ROOT="$WS/runtime/gp_branch_development/place_dfs_suite_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULT_ROOT"
echo "$RESULT_ROOT"
SUITE_STATUS=0
for CASE in ${SEMANTIC_CASES:-semantic_t_junction semantic_l_junction semantic_cross_junction semantic_t_junction_rotated}; do
  export T_JUNCTION_WORLD="$WS/worlds/targeted_navigation/$CASE.world"
  export T_JUNCTION_X=-3.5 T_JUNCTION_Y=0 T_JUNCTION_YAW=0
  HEADINGS=(0 1.57079632679 3.14159265359)
  CENTER=(0 0)
  HEADING_TOLERANCE=0.45
  EXPECTED_ROOMS=${#HEADINGS[@]}
  DFS_DEPTH=2
  [[ "$CASE" != semantic_nested_place_dfs ]] || DFS_DEPTH=3
  export T_JUNCTION_GOAL_DFS_MIN_DEPTH="$DFS_DEPTH"
  if [[ "$CASE" == semantic_t_junction_rotated ]]; then
    CENTER=(0 0)
    export T_JUNCTION_X=-2.474873734 T_JUNCTION_Y=-2.474873734 T_JUNCTION_YAW=0.78539816339
    HEADINGS=(0.78539816339 2.35619449019 3.92699081699)
    EXPECTED_ROOMS=3
  elif [[ "$CASE" == t_junction_rotated ]]; then
    CENTER=(1.3 1.6)
    export T_JUNCTION_WORLD="$WS/worlds/targeted_navigation/t_junction_rotated.world"
    export T_JUNCTION_X=-2.474873734 T_JUNCTION_Y=-2.474873734 T_JUNCTION_YAW=0.78539816339
    HEADINGS=(0.78539816339 2.35619449019 3.92699081699)
  elif [[ "$CASE" == semantic_l_junction ]]; then
    HEADINGS=(1.57079632679 3.14159265359)
  elif [[ "$CASE" == semantic_cross_junction ]]; then
    HEADINGS=(0 1.57079632679 3.14159265359 -1.57079632679)
    HEADING_TOLERANCE=0.70
  fi
  # Each physical outgoing arm must complete its own room lifecycle.
  EXPECTED_ROOMS=${#HEADINGS[@]}
  export T_JUNCTION_GOAL_HEADINGS="${HEADINGS[*]}"
  export T_JUNCTION_GOAL_CENTER="${CENTER[*]}"
  export T_JUNCTION_GOAL_HEADING_TOLERANCE="$HEADING_TOLERANCE"
  # Freeze exact dirty-worktree sources and the actual world for this run.
  python3 - "$WS" "$T_JUNCTION_WORLD" "$RESULT_ROOT/${CASE}_source_manifest.json" <<'PY_MANIFEST'
from pathlib import Path
import hashlib,json,subprocess,sys,os
root,world,out=map(Path,sys.argv[1:])
files=list((root/'src/lste_topo_access/scripts').glob('*.py'))
files+=list((root/'src/lste_topo_access/config').glob('*.yaml'))
files+=list((root/'src/lste_topo_access/launch').glob('*.launch'))
files+=list((root/'src/lste_topo_access/test').glob('test_*.py'))
files+=list((root/'src/lste_topo_access/src').glob('*.cpp'))
files+=list((root/'src/lste_topo_access').glob('*.xml'))
files+=[root/'src/lste_topo_access/CMakeLists.txt',root/'devel/lib/liblste_place_scope_layer.so']
files+=list((root/'scripts/tests/targeted_navigation').glob('*.py'))
files+=list((root/'scripts/tests/targeted_navigation').glob('*.launch'))
files+=[root/'scripts/tests/targeted_navigation/run_t_junction.sh',root/'scripts/tests/targeted_navigation/run_place_dfs_suite.sh',root/'scripts/tests/targeted_navigation/gdb_mux_capture.sh']
executed=Path(os.environ['GP_SUITE_EXECUTED_SCRIPT'])
result=dict(executed_suite_script=str(executed),executed_suite_sha256=hashlib.sha256(executed.read_bytes()).hexdigest(),
    world=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
    git_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=str(root),text=True).strip(),
    files={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)})
out.write_text(json.dumps(result,indent=2)+'\n')
PY_MANIFEST
  RUNNER_LOG="$RESULT_ROOT/${CASE}_runner.log"
  LSTE_WS="$WS" bash scripts/tests/targeted_navigation/run_t_junction.sh --persistent \
      --max-seconds "${GP_SUITE_SECONDS:-3600}" --startup-seconds 90 --no-stop-on-failure >"$RUNNER_LOG" 2>&1 &
  DRIVER_PID=$!
  BAG_PID=
  CYCLE_PID=
  for ATTEMPT in $(seq 1 120); do
    kill -0 "$DRIVER_PID" 2>/dev/null || break
    if rosparam get /run_id >/dev/null 2>&1; then
      # The bag must cover every execution endpoint.  Some channels (notably
      # /pro3/wheel_odom and the frontier status) appear a few seconds after
      # the master: publishing started before them produced bags whose first
      # 3.5 s has no wheel odometry, and the native replay then fails with
      # "raw recording does not cover owned execution endpoints" even though
      # the costmap itself was clean (run 20261102_174505).
      channel_ready() {
        rostopic info "$1" 2>/dev/null \
          | sed -n '/Publishers:/,$p' | grep -q '^ *\*'
      }
      for _ready_wait in $(seq 1 120); do
        ready=1
        for _topic in /map /tf /rbt_pose /pro3/wheel_odom /pro3/rlscan \
                      /lste/global_frontier/status; do
          if ! channel_ready "$_topic"; then ready=0; break; fi
        done
        [[ "$ready" == 1 ]] && break
        sleep 0.5
      done
      RECORD_TOPICS=(/map /tf /tf_static /rbt_pose /pro3/wheel_odom /pro3/rlscan
        /lste/global_frontier/status /lste/teb_turn_supervisor/status
        /lste/cmd_vel/teb_contract /lste/teb_goal_bridge/status
        /lste/cmd_vel_mux/status
        /lste/place_navigation_mask /lste/place_navigation_mask_odom
        /move_base/global_costmap/costmap /move_base/global_costmap/costmap_updates
        /move_base/local_costmap/costmap /move_base/local_costmap/costmap_updates
        /move_base/TebLocalPlannerROS/teb_feedback /move_base/TebLocalPlannerROS/local_plan
        /move_base/TebLocalPlannerROS/parameter_updates
        /move_base/TebLocalPlannerROS/parameter_descriptions
        /move_base/NavfnROS/plan /move_base/current_goal
        /move_base/goal /move_base/status /move_base/result /move_base/recovery_status
        /lste/cmd_vel/teb_planner /cmd_vel)
      rosbag record -O "$RESULT_ROOT/${CASE}_slam.bag" "${RECORD_TOPICS[@]}" \
        >"$RESULT_ROOT/${CASE}_recorder.log" 2>&1 &
      BAG_PID=$!
      if [[ "${GP_CAPTURE_PORTAL_CYCLES:-0}" == 1 ]]; then
        CYCLE_COMMAND=(python3 scripts/tests/targeted_navigation/capture_portal_cycles.py
          --driver "$DRIVER_PID" --prefix "$CASE" --output-dir "$RESULT_ROOT/raw_cycles")
        "${CYCLE_COMMAND[@]}" >"$RESULT_ROOT/${CASE}_cycle_observer.log" 2>&1 &
        CYCLE_PID=$!
      fi
      break
    fi
    sleep 1
  done
  wait "$DRIVER_PID"
  LAUNCH_STATUS=$?
  CYCLE_STATUS=0
  if [[ -n "$CYCLE_PID" ]]; then
    wait "$CYCLE_PID"
    CYCLE_STATUS=$?
    if [[ "$CYCLE_STATUS" != 0 ]]; then SUITE_STATUS=1; fi
    CYCLE_PID=
  fi
  DRIVER_PID=
  if [[ -n "$BAG_PID" ]]; then
    kill -INT "$BAG_PID" 2>/dev/null
    wait "$BAG_PID"
    BAG_PID=
  fi
  RECORDING_COMMAND=(python3 scripts/tests/targeted_navigation/check_place_dfs_recording.py
    "$RESULT_ROOT/${CASE}_slam.bag" --output "$RESULT_ROOT/${CASE}_recording_evidence.json")
  "${RECORDING_COMMAND[@]}" >"$RESULT_ROOT/${CASE}_recording_evidence.log" 2>&1
  RECORDING_STATUS=$?
  if ((RECORDING_STATUS)); then SUITE_STATUS=1; fi
  RUN_DIR=$(sed -n 's/^run_directory=//p' "$RUNNER_LOG" | tail -1)
  if [[ -z "$RUN_DIR" ]]; then
    printf '%s launch_failed %s\n' "$CASE" "$LAUNCH_STATUS" >>"$RESULT_ROOT/summary.log"
    SUITE_STATUS=1
    continue
  fi
  STAMP=$(basename "$RUN_DIR")
  METRICS="$RUN_DIR/${STAMP}_navigation_metrics.log"
  python3 scripts/tests/targeted_navigation/check_gp_tree.py \
      "$METRICS" --headings "${HEADINGS[@]}" --center "${CENTER[@]}" \
      --heading-tolerance "$HEADING_TOLERANCE" --output "$RESULT_ROOT/${CASE}_tree_result.json" >"$RESULT_ROOT/${CASE}_tree_check.log" 2>&1
  TREE_STATUS=$?
  python3 scripts/tests/targeted_navigation/check_portal_authentication.py \
      "$METRICS" --junction-center "${CENTER[@]}" --junction-radius 2.0 \
      --output "$RESULT_ROOT/${CASE}_authentication_result.json" \
      >"$RESULT_ROOT/${CASE}_authentication_check.log" 2>&1
  AUTH_STATUS=$?
  ALL_AUTH_COMMAND=(python3 scripts/tests/targeted_navigation/check_portal_authentication.py
      "$METRICS" --all-started-nodes
      --output "$RESULT_ROOT/${CASE}_all_authentication_result.json")
  "${ALL_AUTH_COMMAND[@]}" >"$RESULT_ROOT/${CASE}_all_authentication_check.log" 2>&1
  ALL_AUTH_STATUS=$?
  TURN_RAW_COMMAND=(python3 scripts/tests/targeted_navigation/audit_portal_turn_directions.py
      "$RESULT_ROOT/${CASE}_slam.bag" --output "$RESULT_ROOT/${CASE}_turn_raw_audit.json")
  "${TURN_RAW_COMMAND[@]}" >"$RESULT_ROOT/${CASE}_turn_raw_check.log" 2>&1
  TURN_RAW_STATUS=$?
  python3 scripts/tests/targeted_navigation/audit_semantic_room_lifecycle.py \
      "$METRICS" --center "${CENTER[@]}" --expected-rooms "$EXPECTED_ROOMS" \
      --output "$RESULT_ROOT/${CASE}_lifecycle_audit.json" \
      >"$RESULT_ROOT/${CASE}_lifecycle_check.log" 2>&1
  LIFECYCLE_STATUS=$?
  python3 scripts/tests/targeted_navigation/audit_place_tsp.py \
      "$METRICS" --output "$RESULT_ROOT/${CASE}_tsp_audit.json" \
      >"$RESULT_ROOT/${CASE}_tsp_check.log" 2>&1
  TSP_STATUS=$?
  # These four layouts may have only one child layer. A separate nested
  # topology must exercise depth >=3; no synthetic child counts as evidence.
  python3 scripts/tests/targeted_navigation/audit_place_dfs.py "$METRICS" \
      --min-stack-depth "$DFS_DEPTH" --output "$RESULT_ROOT/${CASE}_dfs_audit.json" \
      >"$RESULT_ROOT/${CASE}_dfs_check.log" 2>&1
  DFS_STATUS=$?
  python3 scripts/tests/targeted_navigation/audit_place_dfs_geometry.py "$METRICS" \
      "$RESULT_ROOT/${CASE}_slam.bag" --output "$RESULT_ROOT/${CASE}_geometry_audit.json" \
      >"$RESULT_ROOT/${CASE}_geometry_check.log" 2>&1
  GEOMETRY_STATUS=$?
  python3 scripts/tests/targeted_navigation/audit_place_wi_trajectory.py "$METRICS" \
      "$RESULT_ROOT/${CASE}_slam.bag" --output "$RESULT_ROOT/${CASE}_wi_trajectory_audit.json" \
      >"$RESULT_ROOT/${CASE}_wi_trajectory_check.log" 2>&1
  TRAJECTORY_STATUS=$?
  NATIVE_AUDIT_COMMAND=(python3 scripts/tests/targeted_navigation/audit_place_native_costmaps.py
    "$METRICS" "$RESULT_ROOT/${CASE}_slam.bag" --output "$RESULT_ROOT/${CASE}_native_costmap_audit.json")
  "${NATIVE_AUDIT_COMMAND[@]}" >"$RESULT_ROOT/${CASE}_native_costmap_check.log" 2>&1
  NATIVE_STATUS=$?
  python3 scripts/tools/plot_place_tsp.py "$METRICS" --world "$T_JUNCTION_WORLD" \
      --center "${CENTER[@]}" --expected-rooms "$EXPECTED_ROOMS" \
      --output "$RESULT_ROOT/${CASE}_tsp.png" \
      >"$RESULT_ROOT/${CASE}_tsp_visualization.log" 2>&1
  TSP_VIS_STATUS=$?
  python3 scripts/tools/plot_place_dfs.py "$METRICS" --world "$T_JUNCTION_WORLD" \
      --min-stack-depth "$DFS_DEPTH" --output "$RESULT_ROOT/${CASE}_place_dfs.png" \
      >"$RESULT_ROOT/${CASE}_dfs_visualization.log" 2>&1
  DFS_VIS_STATUS=$?
  PLOT_ARGS=(python3 scripts/tools/plot_portal_authentication.py "$METRICS"
      --world "$T_JUNCTION_WORLD" --route-zoom
      --junction-center "${CENTER[@]}" --junction-radius 2.0
      --output "$RESULT_ROOT/${CASE}_portal_authentication.png")
  "${PLOT_ARGS[@]}" >"$RESULT_ROOT/${CASE}_visualization.log" 2>&1
  VIS_STATUS=$?
  printf '%s launch=%s recording=%s tree=%s auth=%s auth_all=%s turn_raw=%s lifecycle=%s tsp=%s dfs=%s geometry=%s trajectory=%s native=%s tsp_visual=%s dfs_visual=%s visual=%s run=%s\n' \
      "$CASE" "$LAUNCH_STATUS" "$RECORDING_STATUS" "$TREE_STATUS" "$AUTH_STATUS" "$ALL_AUTH_STATUS" "$TURN_RAW_STATUS" "$LIFECYCLE_STATUS" "$TSP_STATUS" "$DFS_STATUS" "$GEOMETRY_STATUS" "$TRAJECTORY_STATUS" "$NATIVE_STATUS" "$TSP_VIS_STATUS" "$DFS_VIS_STATUS" "$VIS_STATUS" "$RUN_DIR" \
      >>"$RESULT_ROOT/summary.log"
  if ((LAUNCH_STATUS || RECORDING_STATUS || TREE_STATUS || AUTH_STATUS || ALL_AUTH_STATUS || TURN_RAW_STATUS || LIFECYCLE_STATUS || TSP_STATUS || DFS_STATUS || GEOMETRY_STATUS || TRAJECTORY_STATUS || NATIVE_STATUS || TSP_VIS_STATUS || DFS_VIS_STATUS || VIS_STATUS)); then
    SUITE_STATUS=1
  fi
done
exit "$SUITE_STATUS"
