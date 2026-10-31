"""Classify why one navigation sample produced a zero velocity command.

A zero ``cmd_vel`` has three structurally different owners in this stack:

* upstream -- ``PersistentTebLocalPlanner`` could not obtain a command from
  TEB and published ``PlannerCommandContract`` state ZERO.  The plugin reports
  ``planner_no_command`` exactly when ``teb_.computeVelocityCommands()``
  returned ``false``, i.e. TEB's own feasibility check rejected every
  optimised trajectory.
* handoff -- the planner did produce a command, but the turn supervisor or the
  goal bridge held it at a contract boundary.
* downstream -- a command reached the mux and was filtered (forward-only
  governor, angular deadband) or blocked.

The distinction matters because each owner needs a different fix, and because
downstream "explanations" of a zero command are usually *consequences*: a mux
that reports ``filter_reason=angular_deadband`` is describing an input that was
already zero.  The checks below therefore run upstream first and only attribute
a downstream cause when a non-zero command actually reached that stage.

The module is a pure function over the recorded sample schema used by the
failure-evidence recorder, so the same code can label a live sample, a sealed
failure artifact, or a synthetic fixture in a unit test.
"""

import json
import math
import os

# --- verdicts -------------------------------------------------------------

PLANNER_INFEASIBLE_START_POSE = "planner_infeasible_start_pose"
PLANNER_INFEASIBLE_GEOMETRY = "planner_infeasible_geometry"
PLANNER_ZERO_COMMAND = "planner_zero_command"
BRIDGE_HANDOFF_BOUNDARY = "bridge_handoff_boundary"
MUX_BLOCKED = "mux_blocked"
FORWARD_ONLY_REVERSE_CLAMP = "forward_only_reverse_clamp"
ANGULAR_DEADBAND_ONLY = "angular_deadband_only"
NO_FRESH_FEEDBACK = "no_fresh_feedback"
UNKNOWN = "unknown"

ALL_CAUSES = (
    PLANNER_INFEASIBLE_START_POSE,
    PLANNER_INFEASIBLE_GEOMETRY,
    PLANNER_ZERO_COMMAND,
    BRIDGE_HANDOFF_BOUNDARY,
    MUX_BLOCKED,
    FORWARD_ONLY_REVERSE_CLAMP,
    ANGULAR_DEADBAND_ONLY,
    NO_FRESH_FEEDBACK,
    UNKNOWN,
)

# Safety envelope.  These are the values the navigation launch file installs;
# they are defaults, never an override of the live configuration.
DEFAULT_FOOTPRINT_RADIUS = 0.30
DEFAULT_MIN_OBSTACLE_DIST = 0.22

# Same thresholds the persistent plugin uses to decide ACTIVE vs ZERO.
LINEAR_EPS = 0.001
ANGULAR_EPS = 0.01
ANGULAR_DEADBAND = 0.02

ZERO_BLOCK_REASONS = (None, "", "none")
PLANNER_NO_COMMAND_REASONS = ("planner_no_command",)


def required_clearance(
    footprint_radius=DEFAULT_FOOTPRINT_RADIUS,
    min_obstacle_dist=DEFAULT_MIN_OBSTACLE_DIST,
):
    """Clearance a TEB trajectory must keep from every lethal cell.

    The costmap already marks cells inside the robot's inscribed footprint, so
    the remaining reservation is TEB's physical ``min_obstacle_dist`` on top of
    the footprint radius.
    """
    return float(footprint_radius) + float(min_obstacle_dist)


def _pair(value):
    """Normalise a command into ``(linear_x, angular_z)``."""
    if value is None:
        return (0.0, 0.0)
    if isinstance(value, dict):
        return (
            float(value.get("linear_x", value.get("linear", 0.0)) or 0.0),
            float(value.get("angular_z", value.get("angular", 0.0)) or 0.0),
        )
    if isinstance(value, (list, tuple)):
        values = list(value) + [0.0, 0.0]
        return (float(values[0] or 0.0), float(values[1] or 0.0))
    return (0.0, 0.0)


def command_is_zero(command):
    linear, angular = _pair(command)
    return abs(linear) <= LINEAR_EPS and abs(angular) <= ANGULAR_EPS


def _flatten(values):
    flat = []
    for item in values or ():
        if isinstance(item, (list, tuple)):
            flat.extend(float(x) for x in item)
        else:
            flat.append(float(item))
    return flat


def _grid(values):
    values = values or ()
    if values and isinstance(values[0], (list, tuple)):
        return [[float(x) for x in row] for row in values]
    flat = _flatten(values)
    size = int(round(math.sqrt(len(flat)))) if flat else 0
    if size <= 0 or size * size != len(flat):
        return []
    return [flat[row * size:(row + 1) * size] for row in range(size)]


def clearance_profile(window, resolution=None):
    """Distance from the robot cell to the nearest lethal/inscribed cell.

    ``window`` is the recorder's ``local_costmap_window`` dict (``values`` plus
    ``resolution``) or a bare 2-D grid.  The robot occupies the centre cell.
    The returned distance is measured cell-centre to cell-centre, so it can
    differ from the true obstacle surface by up to half a cell diagonal.
    """
    if isinstance(window, dict):
        grid = _grid(window.get("values"))
        if resolution is None:
            resolution = window.get("resolution")
        center = window.get("center_cell")
    else:
        grid = _grid(window)
        center = None
    if not grid:
        return None

    if resolution is None:
        resolution = 0.1
    resolution = float(resolution or 0.1)

    rows = len(grid)
    cols = len(grid[0])
    center_row = rows // 2
    center_col = cols // 2
    if isinstance(center, (list, tuple)) and len(center) == 2:
        # center_cell is in costmap cell coordinates; the window is centred on
        # it, so the window-relative centre stays the geometric centre.
        center_row = rows // 2
        center_col = cols // 2

    lethal = None
    lethal_cell = None
    inscribed = None
    for row in range(rows):
        for col in range(cols):
            value = grid[row][col]
            distance = math.hypot(row - center_row, col - center_col) * resolution
            if value >= 100.0:
                if lethal is None or distance < lethal:
                    lethal = distance
                    lethal_cell = (row, col)
            elif value >= 99.0:
                if inscribed is None or distance < inscribed:
                    inscribed = distance
    return {
        "lethal_m": lethal,
        "lethal_cell": lethal_cell,
        "inscribed_m": inscribed,
        "resolution": resolution,
        "center_value": grid[center_row][center_col],
        "lethal_cells": sum(1 for row in grid for value in row if value >= 100.0),
    }


def _planner_raw_command(sample):
    """The command TEB itself last produced, before any supervisor gating."""
    for key in ("teb_planner_cmd", "teb_cmd"):
        if key in sample:
            return _pair(sample.get(key))
    supervisor = sample.get("teb_turn_supervisor") or {}
    return _pair(supervisor.get("planner_command"))


def classify(
    sample,
    footprint_radius=DEFAULT_FOOTPRINT_RADIUS,
    min_obstacle_dist=DEFAULT_MIN_OBSTACLE_DIST,
    window_key="local_costmap_window",
):
    """Return the causal verdict for one navigation sample."""
    sample = sample or {}
    contract = sample.get("planner_command_contract") or {}
    mux = sample.get("cmd_vel_mux") or {}
    supervisor = sample.get("teb_turn_supervisor") or {}
    feedback = sample.get("teb_feedback") or {}

    state = contract.get("state")
    reason = str(contract.get("reason") or "")
    planner = _planner_raw_command(sample)
    supervisor_in = _pair(supervisor.get("planner_command"))
    supervisor_out = _pair(supervisor.get("output_command"))
    mux_in = _pair(mux.get("input"))
    mux_out = _pair(mux.get("output"))

    profile = clearance_profile(sample.get(window_key))
    required = required_clearance(footprint_radius, min_obstacle_dist)
    clearance = profile["lethal_m"] if profile else None

    evidence = {
        "contract_state": state,
        "contract_reason": reason,
        "planner_command": list(planner),
        "supervisor_output": list(supervisor_out),
        "mux_input": list(mux_in),
        "mux_output": list(mux_out),
        "mux_block_reason": mux.get("block_reason"),
        "mux_filter_reason": mux.get("filter_reason"),
        "mux_governor_reason": mux.get("governor_reason"),
        "required_clearance_m": round(required, 3),
        "lethal_clearance_m": None if clearance is None else round(clearance, 3),
        "teb_feedback_valid": sample.get("teb_feedback_valid", feedback.get("valid")),
        "teb_feedback_invalid_reason": (
            sample.get("teb_feedback_invalid_reason") or feedback.get("invalid_reason")
        ),
        "teb_feedback_age_seconds": sample.get("teb_feedback_age_seconds"),
    }

    def verdict(cause, explanation):
        return {"cause": cause, "explanation": explanation, "evidence": evidence}

    # 1. A mux that actively blocks is genuinely downstream and final.
    if mux.get("block_reason") not in ZERO_BLOCK_REASONS:
        return verdict(
            MUX_BLOCKED,
            "cmd_vel_mux blocked the command before publication "
            "(block_reason=%r)" % (mux.get("block_reason"),),
        )

    planner_is_zero = command_is_zero(planner)

    # 2a. Newer plugins name the start-pose deadlock directly.  Trust the
    #     contract reason when present; it is measured against the same live
    #     costmap the feasibility check uses.  ``planner_clearance_stall`` is
    #     the same deadlock seen through a rotation-only TEB output instead of
    #     a hard feasibility failure.
    if reason in ("planner_start_pose_infeasible", "planner_clearance_stall"):
        return verdict(
            PLANNER_INFEASIBLE_START_POSE,
            "the planner reported %r: its own cell violates the %.3f m "
            "feasibility clearance, so every candidate trajectory starts "
            "inside the forbidden band" % (reason, required),
        )

    # 2b. The planner produced nothing while the robot's own pose already
    #     violates the clearance the feasibility check requires.  No trajectory
    #     can be feasible from here: every candidate starts at this pose.
    if planner_is_zero and clearance is not None and clearance < required:
        return verdict(
            PLANNER_INFEASIBLE_START_POSE,
            "TEB produced no command and the robot's own cell is %.3f m from "
            "the nearest lethal cell, below the %.3f m required by footprint "
            "%.2f m + min_obstacle_dist %.2f m; every trajectory starts inside "
            "the forbidden band" % (clearance, required,
                                    footprint_radius, min_obstacle_dist),
        )

    # 3. TEB reported no command from a pose that satisfies the clearance, so
    #    the failure is inside trajectory generation, not at the start pose.
    if planner_is_zero and reason in PLANNER_NO_COMMAND_REASONS:
        return verdict(
            PLANNER_INFEASIBLE_GEOMETRY,
            "TEB produced no command (contract reason=%r) while the start pose "
            "has adequate clearance (%.3f m >= %.3f m); the optimiser rejected "
            "every candidate trajectory for another geometric reason"
            % (reason, clearance if clearance is not None else float("nan"), required),
        )

    # 4. The planner did produce a command but it never left the supervisor.
    if not planner_is_zero and command_is_zero(supervisor_out):
        return verdict(
            BRIDGE_HANDOFF_BOUNDARY,
            "the planner produced %r but the turn supervisor published zero "
            "(decision=%r); the boundary is in the planner->bridge handoff, "
            "not in trajectory generation"
            % (list(planner), supervisor.get("output_decision")),
        )

    if not command_is_zero(supervisor_out) and command_is_zero(mux_in):
        return verdict(
            BRIDGE_HANDOFF_BOUNDARY,
            "the supervisor published %r but the mux received zero; the "
            "command was dropped between the supervisor and the mux"
            % (list(supervisor_out),),
        )

    # 5. A non-zero command reached the mux and was attenuated there.
    if not command_is_zero(mux_in) and command_is_zero(mux_out):
        governor = str(mux.get("governor_reason") or "")
        filt = str(mux.get("filter_reason") or "")
        if mux_in[0] < 0.0 and ("forward" in governor or "forward" in filt):
            return verdict(
                FORWARD_ONLY_REVERSE_CLAMP,
                "the mux received a non-forward command %r and the forward-only "
                "governor removed it (governor_reason=%r)"
                % (list(mux_in), governor),
            )
        if "deadband" in filt and abs(mux_in[1]) < max(ANGULAR_DEADBAND, ANGULAR_EPS):
            return verdict(
                ANGULAR_DEADBAND_ONLY,
                "the mux received %r, below the angular deadband %r"
                % (list(mux_in), filt),
            )
        return verdict(
            MUX_BLOCKED,
            "the mux received %r and published zero (filter=%r governor=%r)"
            % (list(mux_in), filt, governor),
        )

    # 6. TEB succeeded but its own output was below the ACTIVE thresholds.
    if state == 0 and reason == "planner_command":
        return verdict(
            PLANNER_ZERO_COMMAND,
            "TEB returned success with a sub-threshold command; the contract was "
            "published as ZERO",
        )

    # 7. Nothing produced a command and the feedback path is the only thing
    #    that is demonstrably broken.
    if (
        command_is_zero(planner)
        and command_is_zero(supervisor_out)
        and command_is_zero(mux_in)
        and evidence["teb_feedback_valid"] is False
    ):
        return verdict(
            NO_FRESH_FEEDBACK,
            "no command at any stage and TEB feedback is invalid (%r); the "
            "feedback path is the only failing stage recorded"
            % (evidence["teb_feedback_invalid_reason"],),
        )

    return verdict(UNKNOWN, "recorded channels do not distinguish a single causal boundary")


def classify_failure_file(path, **kwargs):
    """Classify the trigger sample of one failure-evidence artifact."""
    with open(path, "r") as handle:
        artifact = json.load(handle)
    sample = artifact.get("trigger_sample") or artifact
    result = classify(sample, **kwargs)
    result["artifact"] = os.path.basename(path)
    result["failure_id"] = artifact.get("failure_id")
    result["ros_time"] = sample.get("ros_time")
    return result


def _main(argv):
    paths = []
    for arg in argv:
        if os.path.isdir(arg):
            paths.extend(
                os.path.join(arg, name)
                for name in sorted(os.listdir(arg))
                if name.endswith(".json")
                and "_failure_" in name
                and name.rsplit("_failure_", 1)[-1][:4].isdigit()
            )
        else:
            paths.append(arg)
    print("artifact\tcause\tros_time\tlethal_m\trequired_m\tcontract\tmux_governor")
    for path in paths:
        try:
            result = classify_failure_file(path)
        except Exception as exc:  # pragma: no cover - CLI convenience
            print("%s\tERROR\t%s" % (os.path.basename(path), exc))
            continue
        evidence = result["evidence"]
        print("%s\t%s\t%s\t%s\t%s\t%s\t%s" % (
            result.get("artifact"),
            result["cause"],
            result.get("ros_time"),
            evidence["lethal_clearance_m"],
            evidence["required_clearance_m"],
            evidence["contract_reason"],
            evidence["mux_governor_reason"],
        ))


if __name__ == "__main__":
    import sys

    _main(sys.argv[1:])
