#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Emit a near-orthographic top-down render world.

The camera is placed very high with a narrow FOV so wall footprints project
almost straight down.  Wall material is replaced by a dark material *only in
this render copy* to make the wall tops readable against the light floor.
"""
import math
import os
import re
import sys

world, name, x, y, z = (sys.argv[1], sys.argv[2], float(sys.argv[3]),
                        float(sys.argv[4]), float(sys.argv[5]))
out = sys.argv[6]

COVER_W = float(os.environ.get("TOP_COVER_W", "28.0"))  # metres covered at floor level
fov = 2.0 * math.atan(COVER_W / (2.0 * z))

CAM = f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 1.5708 1.5708</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <camera>
            <horizontal_fov>{fov:.6f}</horizontal_fov>
            <image><width>1600</width><height>1200</height><format>R8G8B8</format></image>
            <clip><near>1.0</near><far>500.0</far></clip>
          </camera>
          <plugin name="p" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <cameraName>{name}</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>{name}</frameName>
          </plugin>
        </sensor>
      </link>
    </model>
"""

OLD_WALL = ("<material><script><uri>file://media/materials/scripts/gazebo.material"
            "</uri><name>Gazebo/Grey</name></script></material>")
NEW_WALL = ("<material><ambient>0.02 0.02 0.02 1</ambient>"
            "<diffuse>0.12 0.12 0.12 1</diffuse>"
            "<specular>0.05 0.05 0.05 1</specular></material>")

src = open(world, encoding="utf-8").read()
n = src.count(OLD_WALL)
src = src.replace(OLD_WALL, NEW_WALL)
# Saved buildings (env04) wrap the same Gazebo/Grey script over several
# lines; recolour those wall materials too.
PATTERN = re.compile(
    r"<material>\s*<script>\s*"
    r"<uri>file://media/materials/scripts/gazebo\.material</uri>\s*"
    r"<name>Gazebo/Grey</name>\s*</script>.*?</material>",
    re.S,
)
src, n2 = PATTERN.subn(NEW_WALL, src)
n += n2
src = src.replace("  </world>", CAM + "  </world>")
open(out, "w", encoding="utf-8").write(src)
open(out + ".cams", "w", encoding="utf-8").write(name)
print("wrote", out, "camera", name, (x, y, z),
      "fov_deg=%.2f" % math.degrees(fov), "walls_recoloured=%d" % n)
