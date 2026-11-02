#!/usr/bin/env python3
"""Rotate a semantic junction Gazebo world around the world origin."""
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--yaw", type=float, default=math.pi / 4)
    args = parser.parse_args()
    tree = ET.parse(str(args.source))
    sdf = tree.getroot()
    world = sdf.find("world")
    if world is None:
        raise SystemExit("input has no <world>")
    theta = args.yaw
    c, s = math.cos(theta), math.sin(theta)
    for model in world.findall("model"):
        name = model.get("name", "")
        if name == "ground_plane":
            continue
        pose = model.find("pose")
        if pose is None:
            pose = ET.SubElement(model, "pose")
            values = [0.0] * 6
        else:
            values = [float(value) for value in pose.text.split()]
            values += [0.0] * (6 - len(values))
        if name == "semantic_room_structure":
            x, y, yaw = 0.0, 0.0, theta
        else:
            x, y = values[0], values[1]
            x, y = c * x - s * y, s * x + c * y
            yaw = values[5] + theta
        values[0], values[1], values[5] = x, y, yaw
        pose.text = " ".join("%.12g" % value for value in values)
    world.set("name", "semantic_t_junction_rotated")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="utf-8", xml_declaration=True)
    print("generated", args.output, "models", len(world.findall("model")), "yaw", theta)

if __name__ == "__main__":
    main()
