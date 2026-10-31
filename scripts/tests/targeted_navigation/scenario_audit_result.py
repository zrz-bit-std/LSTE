"""Evaluate one scenario's audit summary line.

A run passes only when the summary carries every expected status key and every
one of them is 0. Two failure modes that used to slip through:

* ``semantic_l_junction launch_failed 143`` -- a killed run has no key=value
  tokens, so a naive parser produced an empty dict and reported PASS;
* a truncated summary missing some audits.

Usage: scenario_audit_result.py <summary.log> <case>
"""

import sys


EXPECTED = (
    "launch", "recording", "tree", "auth", "auth_all", "turn_raw",
    "lifecycle", "tsp", "dfs", "geometry", "trajectory", "native",
    "tsp_visual", "dfs_visual", "visual",
)


def parse(summary_path, case):
    try:
        with open(summary_path, errors="replace") as handle:
            lines = handle.readlines()
    except OSError:
        return None, "summary_unreadable"
    for line in lines:
        fields = line.split()
        if not fields or fields[0] != case:
            continue
        values = {}
        flags = []
        for token in fields[1:]:
            if "=" in token:
                key, value = token.split("=", 1)
                values[key] = value
            else:
                flags.append(token)
        return values, flags
    return None, "no_summary_line"


def evaluate(summary_path, case):
    values, flags = parse(summary_path, case)
    if isinstance(flags, str):
        return "FAIL " + flags
    if "launch_failed" in flags:
        return "FAIL launch_failed"
    if "launch_failed" in values:
        return "FAIL launch_failed_%s" % values.get("launch_failed", "?")
    missing = [key for key in EXPECTED if key not in values]
    if missing:
        return "FAIL missing_status_fields:" + ",".join(missing)
    failing = [key for key in EXPECTED if values[key] != "0"]
    if failing:
        return "FAIL " + ",".join(failing)
    return "PASS"


def main(argv):
    if len(argv) != 3:
        print("usage: scenario_audit_result.py <summary.log> <case>")
        return 2
    result = evaluate(argv[1], argv[2])
    print(result)
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
