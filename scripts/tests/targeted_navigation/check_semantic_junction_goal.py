#!/usr/bin/env python3
"""Require root-junction acceptance and complete physical Place DFS."""
import argparse
import os
import json
from pathlib import Path
from check_gp_tree import check
from check_portal_authentication import audit as authentication_audit
from audit_semantic_room_lifecycle import audit as lifecycle_audit
from audit_place_tsp import audit as tsp_audit
from audit_place_dfs import audit as dfs_audit
from execution_release_evidence import route_retired


def completion_candidate(path):
    """Cheap negative gate only; success still needs every complete audit.

    Read the latest complete status from a bounded tail. An unavailable or
    malformed status falls back to the original full checks, never an earlier
    positive status or a successful result.
    """
    with Path(path).open('rb') as stream:
        stream.seek(0, 2)
        offset=max(0, stream.tell()-4*1024*1024)
        stream.seek(offset)
        if offset:stream.readline()
        lines=stream.read().splitlines()
    for line in reversed(lines):
        marker=b'event=global_frontier_event data='
        if marker not in line:continue
        try:data=json.loads(line.split(marker,1)[1])
        except (ValueError,UnicodeError):return None
        if not isinstance(data,dict) or 'graph_route_plan' not in data or 'place_dfs' not in data:
            return None
        plan=data.get('graph_route_plan') or {}
        stack=(data.get('place_dfs') or {}).get('stack') or []
        route=data.get('active_execution_route_id',data.get('route_id',0))
        retired=route_retired(data)
        return (plan.get('status')=='complete' and len(stack)==1
                and plan.get('current_place_id')==stack[0]
                and (route in (None,0) or retired)
                and not data.get('awaiting_controller_lease_release_ack',False))
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics")
    parser.add_argument("--headings", nargs="+", type=float, required=True)
    parser.add_argument("--center", nargs=2, type=float, default=[0, 0])
    parser.add_argument("--heading-tolerance", type=float, default=0.45)
    parser.add_argument("--output", required=True)
    parser.add_argument("--dfs-min-depth", type=int, default=int(os.environ.get("T_JUNCTION_GOAL_DFS_MIN_DEPTH", "2")),
                        help="Use 3 for a genuine nested-junction world; flat cases still require all entered subtrees to return")
    args = parser.parse_args()
    if completion_candidate(args.metrics) is False:
        result=dict(passed=False,metrics=args.metrics,precondition='root_work_pending',audits={})
        output=Path(args.output);temporary=output.with_suffix('.tmp')
        temporary.write_text(json.dumps(result,indent=2)+'\n');temporary.replace(output)
        raise SystemExit(1)
    results = {
        "tree": check(args.metrics, args.headings, args.center, 2.0, args.heading_tolerance),
        "authentication": authentication_audit(args.metrics, args.center, 2.0),
        "all_authentication": authentication_audit(args.metrics, all_started_nodes=True),
        "lifecycle": lifecycle_audit(args.metrics, args.center, 2.0, len(args.headings)),
        "tsp": tsp_audit(args.metrics),
        "dfs": dfs_audit(args.metrics, args.dfs_min_depth),
    }
    result = {"passed": all(item["passed"] for item in results.values()),
              "metrics": args.metrics, "audits": results}
    output = Path(args.output)
    temporary = output.with_suffix(".tmp")
    temporary.write_text(json.dumps(result, indent=2) + "\n")
    temporary.replace(output)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
