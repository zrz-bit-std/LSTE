#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""env04 top view for the scene-impl shots (updated world geometry).

Reads the saved world's box collisions and draws the wall footprints, so the
p5/p6 west wall thickening (0.9 m, two faces + end caps) is directly visible.
"""
import math
import os
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

WS = "/home/zrz/work/lste_ws"
WORLD = os.path.join(
    WS, "worlds/topo3.0_catch_mode/session_test/env04_no_pro3.world")
OUT_DIR = "/home/zrz/lste_scene_impl/shots/env04"
TAG = "env04_no_pro3"


def pose_values(node):
    if node is None:
        return [0.0] * 6
    return [float(v) for v in node.text.split()]


def rotate(yaw, x, y):
    return (math.cos(yaw) * x - math.sin(yaw) * y,
            math.sin(yaw) * x + math.cos(yaw) * y)


def boxes():
    root = ET.parse(WORLD).getroot()
    world = root.find("world")
    result = []
    for model in world.findall("model"):
        name = model.get("name")
        if name != "env04":
            continue
        mx, my, _mz, _r, _p, myaw = pose_values(model.find("pose"))
        for link in model.findall("link"):
            lx, ly, _lz, _lr, _lp, lyaw = pose_values(link.find("pose"))
            for collision in link.findall("collision"):
                box = collision.find("geometry/box")
                if box is None:
                    continue
                size = [float(v) for v in box.find("size").text.split()]
                cpose = collision.find("pose")
                cx, cy = (pose_values(cpose)[:2]
                          if cpose is not None else (0.0, 0.0))
                wx, wy = rotate(lyaw, cx, cy)
                wx, wy = wx + lx, wy + ly
                wx, wy = rotate(myaw, wx, wy)
                wx, wy = wx + mx, wy + my
                yaw = lyaw + myaw
                hx = 0.5 * (abs(math.cos(yaw)) * size[0]
                            + abs(math.sin(yaw)) * size[1])
                hy = 0.5 * (abs(math.sin(yaw)) * size[0]
                            + abs(math.cos(yaw)) * size[1])
                result.append((wx - hx, wy - hy, 2 * hx, 2 * hy,
                               link.get("name"), size[2]))
    return result


def draw(ax, rows, focus=False, highlight=True):
    for x0, y0, w, h, name, height in rows:
        if height < 1.0:  # skip low furniture/floor detail
            continue
        ax.add_patch(Rectangle((x0, y0), w, h, facecolor="#2b3445",
                               edgecolor="#2b3445", linewidth=0.6, zorder=2))
    if highlight:
        # highlight the thickened p5/p6 west wall
        ax.add_patch(Rectangle((10.25, -7.62), 1.22, 2.28, fill=False,
                               edgecolor="#d62728", linewidth=2.0, zorder=5))
    if not focus:
        ax.annotate("p5/p6 west wall\n0.9 m thick (was 0.15 m)",
                    xy=(11.36, -6.45), xytext=(13.4, -5.6),
                    fontsize=9, color="#d62728",
                    arrowprops=dict(arrowstyle="->", color="#d62728",
                                    lw=1.2))
        ax.plot(3.4, 0.0, marker="*", markersize=12, color="#2ca02c")
        ax.annotate("spawn (3.4, 0.0)", (3.4, 0.0), xytext=(2.2, -1.0),
                    fontsize=8, color="#2ca02c")
        ax.plot(20.61, -3.56, marker="*", markersize=14, color="#d62728")
        ax.annotate("target yellow cup (20.61, -3.56)", (20.61, -3.56),
                    xytext=(17.2, -2.0), fontsize=8, color="#d62728")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#d9dde3", linewidth=0.6, zorder=0)
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World y (m)")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = boxes()
    fig, ax = plt.subplots(figsize=(18, 15))
    draw(ax, rows)
    ax.set_xlim(-1.5, 23.0)
    ax.set_ylim(-14.0, 7.5)
    ax.set_title(
        "%s top view (updated world, SDF collision footprints)\n"
        "p5/p6 west partition rebuilt like the P2 doorway: two 0.15 m faces "
        "0.9 m apart + 1.05 m end caps; both corner gaps closed at x=11.365"
        % TAG, fontsize=11)
    fig.tight_layout()
    top = os.path.join(OUT_DIR, "env04_top.png")
    fig.savefig(top, dpi=100)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 12))
    draw(ax, rows, focus=True)
    ax.set_xlim(8.0, 14.5)
    ax.set_ylim(-10.5, -3.8)
    ax.set_title("p5/p6 west wall zoom (0.9 m thick, P2 style)", fontsize=11)
    fig.tight_layout()
    zoom = os.path.join(OUT_DIR, "env04_wall_zoom.png")
    fig.savefig(zoom, dpi=100)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    for ax, yc, title in (
        (axes[0], -2.02, "north corner: Wall_19 -> x=11.365\n"
                         "meets Wall_24_e (y=-1.945)"),
        (axes[1], -10.12, "south corner: Wall_33 -> x=11.365\n"
                          "meets Wall_32_e (y=-10.195)"),
    ):
        draw(ax, rows, focus=True, highlight=False)
        ax.set_xlim(9.8, 12.0)
        ax.set_ylim(yc - 1.5, yc + 1.5)
        ax.set_title(title, fontsize=10)
    fig.suptitle("env04 corner gaps closed (world x=11.365)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    corners = os.path.join(OUT_DIR, "env04_corner_zoom.png")
    fig.savefig(corners, dpi=100)
    plt.close(fig)
    print("wrote", top)
    print("wrote", zoom)
    print("wrote", corners)


if __name__ == "__main__":
    main()
