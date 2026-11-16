#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Append render cameras to a copy of a world (the shipped world stays clean)."""
import os
import sys

CAMS = {
    "env05_gate_ring": [
        ("cam_a1",     5.40,   2.80, 0.60, -1.5708, "A1 门口"),
        ("cam_a1in",   8.50,  -0.50, 0.60, -2.8500, "A1 室内（东侧）"),
        ("cam_a2",    14.60,   2.80, 0.60, -1.5708, "A2 门口"),
        ("cam_a2in",  17.20,   0.40, 0.60, -2.3562, "A2 室内侧视（桌+椅+显示器）"),
        ("cam_a3",     5.40, -10.80, 0.60,  1.5708, "A3 仓库门口"),
        ("cam_a3in",   3.00,  -8.80, 0.60,  0.8960, "A3 仓库内部（西南角→梯子）"),
        ("cam_a3target", 6.00, -7.00, 0.60,  2.0344, "A3 目标观察圈 → 梯子"),
        ("cam_a4",    14.60, -10.80, 0.60,  1.5708, "A4 门口"),
        ("cam_a4in",  14.60,  -6.20, 0.60, -1.5708, "A4 室内"),
    ],

    "env06_gate_nested": [
        ("cam_b1",     4.00,  -4.10, 0.60,  1.5708, "B1 门口 0.88"),
        ("cam_b1in",   2.20,  -0.20, 0.60,  1.5708, "B1 会议室西北角桌/椅"),
        ("cam_b2",    14.50,  -4.10, 0.60,  1.5708, "B2 门口 0.50"),
        ("cam_b2in",  15.90,   1.60, 0.60,  1.5708, "B2 台面：咖啡机/黄杯/蓝杯"),
        ("cam_b2tr",  17.60,  -0.20, 0.60, -1.5708, "B2 垃圾桶"),
        ("cam_b3",    20.60,   1.00, 0.60,  0.2000, "B3 室内入口视角（含桌面正向物品）"),
        ("cam_b3ring", 20.80, 0.90, 0.60,  -0.1630, "B3 观察圈 → 桌面绿植/桌椅（室内近距）"),
        ("cam_b5",     9.50,  -4.55, 0.60, -1.5708, "B5 机房（负分货物）"),
    ],

    "env04_no_pro3": [
        ("cam_p1",      3.40,   0.00, 0.60,  0.0,    "P1 西侧房间（spawn）"),
        ("cam_p1in",    7.50,   0.00, 0.60,  3.1416, "P1 室内（向西）"),
        ("cam_p2",      7.00,   4.50, 0.60,  3.1416, "P2 北侧房间（file boxes）"),
        ("cam_p5door",  9.00,  -6.40, 0.60,  0.0,    "P5/P6 西侧 0.9 m 门"),
        ("cam_p5in",   12.50,  -8.00, 0.60,  3.1416, "P5/P6 室内"),
        ("cam_east",   16.00,  -3.50, 0.60,  0.0,    "P4 东侧目标房"),
        ("cam_target", 19.00,  -3.56, 0.60,  0.0,    "目标观察 — yellow cup"),
        ("cam_eastdoor", 17.00, -11.00, 0.60, 1.5708, "目标房南入口 — 向北"),
        ("cam_south",   7.00, -11.00, 0.60,  0.0,    "南侧走廊 — 向东"),
    ],
}

TPL = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <camera>
            <horizontal_fov>1.5708</horizontal_fov>
            <image><width>640</width><height>360</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>20.0</far></clip>
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


def main():
    world, name = sys.argv[1], sys.argv[2]
    out = sys.argv[3]
    src = open(world, encoding="utf-8").read()
    extra = "".join(TPL.format(name=n, x=x, y=y, z=z, yaw=yaw)
                    for n, x, y, z, yaw, _note in CAMS[name])
    src = src.replace("  </world>", extra + "  </world>")
    open(out, "w", encoding="utf-8").write(src)
    open(out + ".cams", "w", encoding="utf-8").write(",".join(c[0] for c in CAMS[name]))
    print("wrote", out, "with", len(CAMS[name]), "cameras")
    for c in CAMS[name]:
        print("   ", c[0], c[5])


if __name__ == "__main__":
    main()
