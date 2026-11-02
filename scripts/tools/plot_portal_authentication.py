#!/usr/bin/env python3
"""Plot a GP doorway-authentication run over the test-world geometry."""
import argparse
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "tests" / "targeted_navigation"))
sys.path.insert(0, str(ROOT / "scripts" / "tools"))
from check_portal_authentication import audit
from plot_world_run import read_scene

COLORS = {"completed": "#16836c", "failed": "#bd5144", "entering": "#2674b8",
          "turning": "#c08a16", "returning": "#8a5aa8", "pending": "#94a3b8",
          "active": "#2674b8", "explored": "#64748b", "authenticated": "#16836c"}

def read_metrics(path):
    events=[]; poses=[]; latest_tree=None; dedup=[]; latest_state={}
    with Path(path).open(encoding="utf-8",errors="strict") as stream:
        for lineno,line in enumerate(stream,1):
            if not line.strip():continue
            if " data=" not in line or not line.endswith("\n"):
                raise ValueError("incomplete metrics record at line %d" % lineno)
            event=line.split(" event=",1)[1].split(" data=",1)[0] if " event=" in line else ""
            data=json.loads(line.split(" data=",1)[1])
            if not isinstance(data,dict):raise ValueError("non-object metrics record")
            if event == "sample" and data.get("pose"):
                poses.append({"pose":data["pose"],"ros_time":data.get("ros_time")})
            if event == "global_frontier_event":
                name=data.get("frontier_event","")
                keys=("ros_time","branch_id","node_id","phase","goal_odom","route_id",
                      "work_item_id","attempt_id","place_id","parent_place_id","portal_id")
                row={k:data[k] for k in keys if k in data}
                if name == "work_item_dispatched":
                    row["graph_route_plan"]=data.get("graph_route_plan")
                    row["work_items"]=[w for w in data.get("work_items",[])
                                       if w.get("id")==data.get("work_item_id")]
                if name in ("semantic_room_exploration_started","semantic_room_closed"):
                    row["active_semantic_room"]=data.get("active_semantic_room")
                if name in ("place_dfs_entered","place_dfs_returned","place_dfs_completion_ready"):
                    row["place_dfs"]=data.get("place_dfs")
                events.append((name,row))
                for key in ("place_dfs","portal_hypotheses"):
                    if data.get(key) is not None:latest_state[key]=data[key]
                if name == "gp_branch_tree":
                    latest_tree=data.get("tree") or latest_tree
                    if data.get("dedup_audit"):
                        dedup.append((data.get("ros_time",0),data["dedup_audit"]))
    if events:events[-1][1].update(latest_state)
    return events, poses, latest_tree, dedup

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("metrics_log", type=Path)
    ap.add_argument("--world", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--model-root", type=Path, action="append")
    ap.add_argument("--junction-center", nargs=2, type=float,
                    help="Audit candidate completion only for this junction center")
    ap.add_argument("--junction-radius", type=float, default=2.0)
    ap.add_argument("--route-zoom", action="store_true", help="Focus on measured motion; use scene PNG for the full floorplan")
    args=ap.parse_args()
    events,poses,tree,dedup=read_metrics(args.metrics_log)
    result=audit(args.metrics_log, junction_center=args.junction_center,
                  junction_radius=args.junction_radius)
    target_nodes = None
    if args.junction_center is not None and tree:
        target_nodes = {
            int(node["id"]) for node in tree.get("nodes", [])
            if node.get("junction_confirmed") and node.get("physical_xy")
            and math.dist(node["physical_xy"], args.junction_center) <= args.junction_radius
        }
        target_branches = {int(branch["id"]) for branch in tree.get("branches", [])
                           if int(branch.get("node_id", -1)) in target_nodes}
        tree = dict(tree, nodes=[node for node in tree.get("nodes", [])
                                 if int(node["id"]) in target_nodes],
                    branches=[branch for branch in tree.get("branches", [])
                              if int(branch["id"]) in target_branches])
        events = [(name, data) for name, data in events
                  if not name.startswith("portal_authentication")
                  or int(data.get("node_id", -1)) in target_nodes]
    roots=args.model_root or [Path.home()/".gazebo/models", ROOT/"worlds"]
    try: shapes,_,missing=read_scene(args.world,roots)
    except Exception: shapes,missing=[],["world_geometry_parse_failed"]
    fig,(ax,side)=plt.subplots(1,2,figsize=(16,9),gridspec_kw={"width_ratios":[1.65,1]})
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#f8fafc")
    for shape in shapes:
        if shape.get("kind") != "wall": continue
        ax.add_patch(Polygon(shape["xy"],closed=True,facecolor="#263746",
                             edgecolor="#263746",linewidth=.5,zorder=1))
    xy=np.asarray([p["pose"][:2] for p in poses],dtype=float).reshape(-1,2)
    if len(xy)>1:
        ax.plot(xy[:,0],xy[:,1],color="#b8c5d1",lw=2.6,zorder=2)
        ax.plot(xy[:,0],xy[:,1],color="#287aa5",lw=1.2,zorder=3)
        ax.scatter(*xy[0],s=55,marker="o",c="#234b76",edgecolor="white",zorder=7)
        ax.scatter(*xy[-1],s=58,marker="s",c="#d68b47",edgecolor="white",zorder=7)
    nodes={}; branches=[]
    if tree:
        nodes={int(n["id"]):n for n in tree.get("nodes",[])}
        branches=tree.get("branches",[])
        for node in nodes.values():
            pt=node.get("physical_xy")
            if pt is None:continue
            ax.scatter(*pt,s=105,c="#172b40" if node.get("junction") else "#64748b",
                       edgecolor="white",linewidth=1.2,zorder=8)
            ax.text(pt[0],pt[1]+.18,"J%d"%node["id"] if node.get("junction") else "N%d"%node["id"],
                    ha="center",fontsize=8,weight="bold",zorder=9)
        for branch in branches:
            node=nodes.get(int(branch.get("node_id",-1)))
            gate=branch.get("physical_gate")
            if node is None or gate is None:continue
            source=node.get("physical_xy")
            if source is None:continue
            state=branch.get("authentication_state") or branch.get("state","pending")
            color=COLORS.get(state,"#64748b")
            ax.annotate("",xy=gate,xytext=source,arrowprops=dict(arrowstyle="-|>",lw=2,color=color),zorder=5)
            ax.scatter(*gate,s=35,c=color,marker="D",edgecolor="white",linewidth=.5,zorder=6)
            ax.text(gate[0],gate[1]+.16,"B%d %s"%(branch["id"],state),fontsize=7,
                    color=color,ha="center",bbox=dict(fc="white",ec="none",alpha=.78,pad=1),zorder=9)
    auth_rows={}
    for name,data in events:
        if name=="portal_authentication_branch_started":
            auth_rows[int(data["branch_id"])]=[]
        if name.startswith("portal_authentication") and data.get("branch_id") is not None:
            auth_rows.setdefault(int(data["branch_id"]),[]).append((name,data))
            goal=data.get("goal_odom")
            phase=data.get("phase")
            if goal and phase in ("entering","turning","returning"):
                marker={"entering":"^","turning":"*","returning":"v"}[phase]
                ax.scatter(*goal,s=85 if phase=="turning" else 44,marker=marker,
                           c=COLORS.get(phase,"#333"),edgecolor="white",zorder=10)
    ax.set_aspect("equal",adjustable="datalim");ax.grid(color="#e4eaf0",lw=.5)
    ax.set_xlabel("World x (m)");ax.set_ylabel("World y (m)")
    ax.set_title("GP junction tree, doorway candidates and measured route",loc="left",weight="bold")
    handles=[Line2D([],[],color="#287aa5",lw=2,label="Online SLAM odometry"),
             Line2D([],[],marker="D",color="none",markerfacecolor="#2674b8",label="GP doorway candidate"),
             Line2D([],[],marker="^",color="none",markerfacecolor="#2674b8",label="Enter goal"),
             Line2D([],[],marker="*",color="none",markerfacecolor="#c08a16",label="Locked turn goal"),
             Line2D([],[],marker="v",color="none",markerfacecolor="#8a5aa8",label="Return-to-node goal")]
    ax.legend(handles=handles,loc="best",framealpha=.93,fontsize=8)
    if args.route_zoom and len(xy):
        ax.set_xlim(float(xy[:,0].min())-2, float(xy[:,0].max())+2)
        ax.set_ylim(float(xy[:,1].min())-2, float(xy[:,1].max())+2)
        ax.set_aspect("equal", adjustable="box")
    side.axis("off")
    verdict="PASS" if result["passed"] else "IN PROGRESS / FAIL"
    fig.suptitle("Doorway observation audit  |  %s  |  %s"%(args.world.stem,verdict),
                 x=.05,y=.97,ha="left",fontsize=19,weight="bold",color="#172b40")
    side.text(0,1.0,"Branch-by-branch evidence",fontsize=12,weight="bold",va="top")
    y=.94
    for branch_id in sorted(auth_rows):
        trace=auth_rows[branch_id]
        finish=next((d for n,d in trace if n=="portal_authentication_branch_finished"),{})
        inside=next((d for n,d in trace if n=="portal_authentication_inside_evidence"),{})
        final=finish.get("state","running")
        color=COLORS.get(final,"#2674b8")
        side.text(0,y,"Branch %d  ·  %s"%(branch_id,final.upper()),color=color,weight="bold",fontsize=10,va="top")
        y-=.032
        if inside:
            side.text(.035,y,"Inside frontier cells  left %s  /  right %s"%(inside.get("left_frontier_cells"),inside.get("right_frontier_cells")),fontsize=8.5,va="top")
            y-=.027
            target_yaw = inside.get("target_yaw")
            turn_text = ("Turn not performed (no inside frontier)"
                         if target_yaw is None else
                         "Turn  %s  ·  target yaw %.1f°" %
                         (inside.get("turn_direction"), math.degrees(float(target_yaw))))
            side.text(.035,y,turn_text,fontsize=8.5,va="top")
            y-=.027
        if finish.get("failure_reason"):
            side.text(.035,y,"Failure  "+str(finish["failure_reason"]),fontsize=8,color="#a33d35",va="top",wrap=True)
            y-=.035
        milestones=[]
        evmap={"portal_authentication_branch_started":"watch","portal_authentication_inside_evidence":"inside/frontier",
               "portal_authentication_route_selected":"route stage","portal_authentication_failure_returning":"failure return",
               "portal_authentication_branch_finished":"settled"}
        for name,data in trace:
            if name in evmap:
                milestones.append("%s @ %.1fs"%(evmap[name],data.get("ros_time",0)))
        side.text(.035,y,"  →  ".join(milestones[-5:]),fontsize=7.3,color="#526273",va="top",wrap=True)
        y-=.047
    side.text(0,y-.01,"Online SLAM / GP dedup audit (run maxima)",fontsize=11,weight="bold",va="top")
    y-=.06
    dedup_latest=result.get("online_slam_dedup_audit") or {}
    for label,key in (("Raw GP proposals","raw_gp_proposals"),
                      ("SLAM-refined directions","slam_refined_proposals"),
                      ("Suppressed by mapped free-space","slam_supported_rejections"),
                      ("Direction overlap removed","slam_direction_deduplications"),
                      ("Sensor-only candidates retained","sensor_only_candidates_retained")):
        side.text(.02,y,"%-35s %s"%(label,str(dedup_latest.get(key,"not recorded"))),fontsize=8.2,va="top",family="monospace")
        y-=.028
    side.text(.02,y-.005,"Frontier point coordinates are not logged; bars above report\nthe actual left/right SLAM frontier-cell counts used for turn choice.",
              fontsize=7.4,color="#64748b",va="top")
    checks=result["checks"]
    failed=[k.replace("_"," ") for k,v in checks.items() if not v]
    status="Audited junction checks pass; route shows full recorded run" if not failed else "Pending checks: "+", ".join(failed[:3])
    fig.text(.05,.035,status,fontsize=9,color="#16836c" if not failed else "#9b5c1b")
    fig.text(.96,.035,"Source: recorded world model + odometry + GP/auth events; no frontier points synthesized.",
             fontsize=7.2,color="#64748b",ha="right")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=220,bbox_inches="tight",facecolor="white")
    plt.close(fig)
    print(json.dumps({"output":str(args.output.resolve()),"passed":result["passed"],
                      "checks":result["checks"],"missing_world_meshes":missing,
                      "dedup_snapshots":len(dedup)},indent=2))

if __name__=="__main__":main()
