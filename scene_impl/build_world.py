#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Emit the two Gazebo worlds on bit1010 (door width is the only geometry knob)."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_design as bd  # noqa: E402

DOOR_W = float(os.environ.get("DOOR_W", "2.30"))
OUT = os.environ.get("WORLD_OUT", "/home/zrz/work/lste_ws/worlds/topo3.0_catch_mode/session_test")
WALL_H = 2.5

HEADER = """<?xml version='1.0'?>
<sdf version='1.7'>
  <world name='lab_world'>
    <gravity>0 0 -9.8</gravity>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>0</shadows>
    </scene>
    <light type='directional' name='sun'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 12 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include><uri>model://ground_plane</uri><name>ground_plane</name></include>
"""

FOOTER = """  </world>
</sdf>
"""


def with_door_width(scene, w):
    """Return a copy of the scene whose wall openings all use width w."""
    out = dict(scene)
    walls = []
    for wid, kind, at, a, b, _old in scene["walls"]:
        cuts = []
        for did, room, axis, dat, dc, dw, s in scene["doors"]:
            if axis == kind and abs(dat - at) < 1e-6:
                cuts.append((dc - w / 2.0, dc + w / 2.0))
        walls.append((wid, kind, at, a, b, sorted(cuts)))
    out["walls"] = walls
    return out


def wall_model(name, scene):
    lines = [f"    <model name='{name}'>", "      <static>1</static>"]
    for i, b in enumerate(bd.wall_boxes(scene)):
        x0, y0, x1, y1 = bd.aabb(b["cx"], b["cy"], b["w"], b["d"], b["yaw"])
        dx, dy = x1 - x0, y1 - y0
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        lines += [
            f"      <link name='wall_{i:02d}'>",
            f"        <pose>{cx:.4f} {cy:.4f} {WALL_H/2:.3f} 0 -0 0</pose>",
            "        <collision name='collision'>",
            f"          <geometry><box><size>{dx:.4f} {dy:.4f} {WALL_H:.2f}</size></box></geometry>",
            "        </collision>",
            "        <visual name='visual'>",
            f"          <geometry><box><size>{dx:.4f} {dy:.4f} {WALL_H:.2f}</size></box></geometry>",
            "          <material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
            "<name>Gazebo/Grey</name></script></material>",
            "        </visual>",
            "      </link>",
        ]
    lines.append("    </model>")
    return "\n".join(lines)


def object_includes(scene):
    out = []
    for oid, model, x, y, yaw, z, role, note in scene["objects"]:
        out += [
            "    <include>",
            f"      <uri>model://{model}</uri>",
            f"      <name>{oid}</name>",
            f"      <pose>{x:.4f} {y:.4f} {z:.4f} 0 -0 {math.radians(yaw):.6f}</pose>",
            "      <static>1</static>",
            "    </include>",
        ]
    return "\n".join(out)


def gui(scene):
    x0, y0, x1, y1 = scene["bbox"]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    d = max(x1 - x0, y1 - y0) * 0.95 + 4
    return (f"    <gui fullscreen='0'>\n      <camera name='user_camera'>\n"
            f"        <pose>{cx:.2f} {cy:.2f} {d:.2f} 0 1.35 0</pose>\n"
            "        <view_controller>orbit</view_controller>\n"
            "        <projection_type>perspective</projection_type>\n"
            "      </camera>\n    </gui>\n")


def build(name, scene):
    sc = with_door_width(scene, DOOR_W)
    body = [HEADER, wall_model(name, sc), object_includes(sc), gui(sc), FOOTER]
    path = os.path.join(OUT, name + ".world")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(body))
    return path


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    print("door width =", DOOR_W)
    for name, scene in bd.SCENES.items():
        p = build(name, scene)
        print("wrote", p, os.path.getsize(p), "bytes")
