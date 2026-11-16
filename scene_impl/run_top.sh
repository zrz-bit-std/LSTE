#!/bin/bash
# one top-down frame from a world: run_top.sh <world> <name> <x> <y> <z>
set +u
W=${1:?}; N=${2:?}; X=${3:?}; Y=${4:?}; Z=${5:?}
D=/home/zrz/lste_scene_impl
mkdir -p "$D/logs" "$D/render" "$D/shots/$W"
export ROS_MASTER_URI=http://127.0.0.1:11399
export GAZEBO_MASTER_URI=http://127.0.0.1:11398
export GAZEBO_MODEL_PATH=/home/zrz/.gazebo/models
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/noetic/setup.bash
set -u

pkill -f "lste_scene_impl/render/${W}_top" 2>/dev/null
pkill -f "lste_scene_impl/capture_cams.py" 2>/dev/null
pkill -f "Xvfb :77" 2>/dev/null
pkill -f "roscore -p 11399" 2>/dev/null
pkill -f "rosmaster --core -p 11399" 2>/dev/null
rm -f /tmp/.X77-lock
sleep 2

SRC=${WORLD_SRC:-/home/zrz/work/lste_ws/worlds/topo3.0_catch_mode/session_test/$W.world}
python3 "$D/make_top_world.py" "$SRC" "$N" "$X" "$Y" "$Z" \
  "$D/render/${W}_top.world"

Xvfb :77 -screen 0 1600x1200x24 >"$D/logs/xvfb_top.log" 2>&1 &
sleep 3
export DISPLAY=:77
xdpyinfo >/dev/null 2>&1 || { echo "display FAILED"; exit 1; }

roscore -p 11399 >"$D/logs/roscore_top.log" 2>&1 &
sleep 7

gzserver --verbose -s /opt/ros/noetic/lib/libgazebo_ros_paths_plugin.so \
  -s /opt/ros/noetic/lib/libgazebo_ros_api_plugin.so "$D/render/${W}_top.world" \
  >"$D/logs/gz_top_$W.log" 2>&1 &
sleep 35

PROBE_OUT="$D/shots/$W" CAMS="$N" EXPECT=1 \
  timeout 240 python3 "$D/capture_cams.py" >"$D/logs/cap_top_$W.log" 2>&1
echo "capture_rc=$?"
ls -la "$D/shots/$W/$N.png"

pkill -f "lste_scene_impl/render/${W}_top" 2>/dev/null
pkill -f "roscore -p 11399" 2>/dev/null
pkill -f "rosmaster --core -p 11399" 2>/dev/null
pkill -f "Xvfb :77" 2>/dev/null
rm -f /tmp/.X77-lock
sleep 1
echo done
