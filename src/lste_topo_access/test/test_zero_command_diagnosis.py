"""Unit tests for the zero-command causal classifier.

The point of these tests is to make the question "why is the command zero?"
answerable per hypothesis instead of per run:

* local costmap / obstacle constraint  -> PLANNER_INFEASIBLE_START_POSE
* trajectory generation failure (free start) -> PLANNER_INFEASIBLE_GEOMETRY
* planner <-> bridge handoff           -> BRIDGE_HANDOFF_BOUNDARY
* mux policy (forward-only / deadband) -> FORWARD_ONLY_REVERSE_CLAMP / ANGULAR_DEADBAND_ONLY
* feedback plumbing                    -> NO_FRESH_FEEDBACK

The last test replays every sealed failure artifact of run ``20261030_033332``
and is skipped when that evidence directory is not present.
"""

import json
import os
from collections import Counter
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import zero_command_diagnosis as zcd  # noqa: E402


SEALED_RUN = Path(os.environ.get(
    "LSTE_ZERO_COMMAND_EVIDENCE_DIR",
    "/home/zrz/work/lste_ws/runtime/targeted_navigation/"
    "t_junction_small/logs/20261030_033332",
))


def wall_window(distance_m, resolution=0.1):
    """15x15-style costmap window with a lethal wall this far from the robot."""
    cells = int(round(float(distance_m) / resolution))
    size = 2 * cells + 3
    center = size // 2
    grid = [[0 for _ in range(size)] for _ in range(size)]
    nearest_lethal_row = center - cells
    for row in range(nearest_lethal_row + 1):
        for col in range(size):
            grid[row][col] = 100
    return {
        "values": grid,
        "resolution": resolution,
        "center_cell": [size, size],
        "center_world": [0.0, 0.0],
    }


def free_window(resolution=0.1, size=15):
    """Window with no obstacle cell anywhere in the local planning horizon."""
    return {
        "values": [[0 for _ in range(size)] for _ in range(size)],
        "resolution": resolution,
        "center_cell": [size, size],
        "center_world": [0.0, 0.0],
    }


def sample(
    contract_state=0,
    contract_reason="planner_no_command",
    planner=(0.0, 0.0),
    supervisor_in=(0.0, 0.0),
    supervisor_out=(0.0, 0.0),
    supervisor_decision="planner_contract_boundary",
    mux_in=(0.0, 0.0),
    mux_out=(0.0, 0.0),
    mux_block_reason="none",
    mux_filter_reason="none",
    mux_governor_reason="none",
    window=None,
    feedback_valid=False,
    invalid_reason="planner_contract_planner_no_command",
    feedback_age=120.0,
):
    return {
        "ros_time": 1000.0,
        "planner_command_contract": {
            "state": contract_state,
            "reason": contract_reason,
        },
        "teb_planner_cmd": list(planner),
        "cmd_vel_mux": {
            "block_reason": mux_block_reason,
            "filter_reason": mux_filter_reason,
            "governor_reason": mux_governor_reason,
            "input": {"linear_x": mux_in[0], "angular_z": mux_in[1]},
            "output": {"linear_x": mux_out[0], "angular_z": mux_out[1]},
        },
        "teb_turn_supervisor": {
            "planner_command": list(supervisor_in),
            "output_command": list(supervisor_out),
            "output_decision": supervisor_decision,
        },
        "local_costmap_window": window if window is not None else free_window(),
        "teb_feedback": {
            "valid": feedback_valid,
            "invalid_reason": invalid_reason,
        },
        "teb_feedback_valid": feedback_valid,
        "teb_feedback_invalid_reason": invalid_reason,
        "teb_feedback_age_seconds": feedback_age,
    }


class ClearanceProfileTest(unittest.TestCase):
    def test_required_clearance_matches_launch_envelope(self):
        # footprint 0.30 m circular + min_obstacle_dist 0.22 m
        self.assertAlmostEqual(zcd.required_clearance(), 0.52)

    def test_profile_reads_nearest_lethal_cell(self):
        window = wall_window(0.4)
        profile = zcd.clearance_profile(window)
        self.assertAlmostEqual(profile["lethal_m"], 0.4)
        size = len(window["values"])
        self.assertEqual(profile["lethal_cell"], (size // 2 - 4, size // 2))

    def test_profile_accepts_bare_grid(self):
        profile = zcd.clearance_profile(wall_window(0.3)["values"], resolution=0.1)
        self.assertAlmostEqual(profile["lethal_m"], 0.3)

    def test_profile_reports_no_lethal_cell(self):
        profile = zcd.clearance_profile(free_window())
        self.assertIsNone(profile["lethal_m"])


class ZeroCommandCauseTest(unittest.TestCase):
    def test_start_pose_inside_clearance_band_is_infeasible(self):
        result = zcd.classify(sample(window=wall_window(0.4)))
        self.assertEqual(result["cause"], zcd.PLANNER_INFEASIBLE_START_POSE)
        self.assertLess(result["evidence"]["lethal_clearance_m"],
                        result["evidence"]["required_clearance_m"])

    def test_marginal_start_pose_of_0_32m_is_infeasible(self):
        # the value measured in the sealed L run
        result = zcd.classify(sample(window=wall_window(0.32)))
        self.assertEqual(result["cause"], zcd.PLANNER_INFEASIBLE_START_POSE)

    def test_free_start_pose_blames_trajectory_generation(self):
        result = zcd.classify(sample(window=free_window()))
        self.assertEqual(result["cause"], zcd.PLANNER_INFEASIBLE_GEOMETRY)
        self.assertEqual(result["evidence"]["contract_reason"], "planner_no_command")

    def test_new_contract_reason_is_trusted_over_geometry(self):
        # The plugin now measures the start pose against the live costmap and
        # names the deadlock itself; the classifier must honour that even when
        # the recorded window happens to look free (e.g. a stale snapshot).
        result = zcd.classify(sample(
            contract_reason="planner_start_pose_infeasible",
            window=free_window(),
        ))
        self.assertEqual(result["cause"], zcd.PLANNER_INFEASIBLE_START_POSE)

    def test_rotation_only_clearance_stall_maps_to_the_same_deadlock(self):
        # Same physical state reached through a rotation-only TEB output: the
        # robot spins in place because translation is infeasible, so this must
        # not be reported as an unknown or a turn-supervisor fault.
        result = zcd.classify(sample(
            contract_reason="planner_clearance_stall",
            window=free_window(),
        ))
        self.assertEqual(result["cause"], zcd.PLANNER_INFEASIBLE_START_POSE)

    def test_terminal_invalidation_at_wedged_pose_still_blames_start_pose(self):
        # `move_base_failed_terminal` is the downstream reaction to the same
        # physical state; the classifier must not mistake it for a new cause.
        result = zcd.classify(sample(
            contract_state=2,
            contract_reason="move_base_failed_terminal",
            window=wall_window(0.41),
        ))
        self.assertEqual(result["cause"], zcd.PLANNER_INFEASIBLE_START_POSE)

    def test_planner_command_held_by_supervisor_is_a_handoff_boundary(self):
        result = zcd.classify(sample(
            contract_state=1,
            contract_reason="planner_command",
            planner=(0.18, 0.0),
            supervisor_in=(0.18, 0.0),
            supervisor_out=(0.0, 0.0),
            supervisor_decision="planner_contract_boundary",
            window=free_window(),
            feedback_valid=True,
            invalid_reason="",
        ))
        self.assertEqual(result["cause"], zcd.BRIDGE_HANDOFF_BOUNDARY)

    def test_command_dropped_between_supervisor_and_mux_is_a_handoff_boundary(self):
        result = zcd.classify(sample(
            contract_state=1,
            contract_reason="planner_command",
            planner=(0.18, 0.0),
            supervisor_out=(0.18, 0.0),
            mux_in=(0.0, 0.0),
            window=free_window(),
            feedback_valid=True,
            invalid_reason="",
        ))
        self.assertEqual(result["cause"], zcd.BRIDGE_HANDOFF_BOUNDARY)

    def test_mux_block_reason_is_reported_first(self):
        result = zcd.classify(sample(
            contract_state=1,
            contract_reason="planner_command",
            planner=(0.18, 0.0),
            supervisor_out=(0.18, 0.0),
            mux_in=(0.18, 0.0),
            mux_block_reason="missing_transaction_identity",
        ))
        self.assertEqual(result["cause"], zcd.MUX_BLOCKED)
        self.assertEqual(result["evidence"]["mux_block_reason"],
                         "missing_transaction_identity")

    def test_reverse_command_is_forward_only_clamp(self):
        result = zcd.classify(sample(
            contract_state=1,
            contract_reason="planner_command",
            planner=(-0.10, 0.0),
            supervisor_out=(-0.10, 0.0),
            mux_in=(-0.10, 0.0),
            mux_governor_reason="not_forward",
            window=free_window(),
            feedback_valid=True,
            invalid_reason="",
        ))
        self.assertEqual(result["cause"], zcd.FORWARD_ONLY_REVERSE_CLAMP)

    def test_angular_deadband_needs_a_non_zero_mux_input(self):
        clamped = zcd.classify(sample(
            contract_state=1,
            contract_reason="planner_command",
            planner=(0.0, 0.015),
            supervisor_out=(0.0, 0.015),
            mux_in=(0.0, 0.015),
            mux_filter_reason="angular_deadband",
            window=free_window(),
            feedback_valid=True,
            invalid_reason="",
        ))
        self.assertEqual(clamped["cause"], zcd.ANGULAR_DEADBAND_ONLY)

    def test_deadband_on_an_already_zero_input_is_not_a_cause(self):
        # Sealed-run shape: mux input is already zero, so "angular_deadband"
        # is a symptom of the upstream failure, never its explanation.
        result = zcd.classify(sample(
            mux_in=(0.0, 0.0),
            mux_filter_reason="angular_deadband",
            mux_governor_reason="not_forward",
            window=wall_window(0.4),
        ))
        self.assertEqual(result["cause"], zcd.PLANNER_INFEASIBLE_START_POSE)

    def test_subthreshold_teb_output_is_planner_zero_command(self):
        result = zcd.classify(sample(
            contract_state=0,
            contract_reason="planner_command",
            planner=(0.0005, 0.0005),
            window=free_window(),
            feedback_valid=True,
            invalid_reason="",
        ))
        self.assertEqual(result["cause"], zcd.PLANNER_ZERO_COMMAND)

    def test_stale_feedback_alone_is_feedback_plumbing(self):
        result = zcd.classify(sample(
            contract_state=1,
            contract_reason="planner_command",
            window=free_window(),
            feedback_valid=False,
            invalid_reason="age",
            feedback_age=12.0,
        ))
        self.assertEqual(result["cause"], zcd.NO_FRESH_FEEDBACK)

    def test_every_verdict_is_in_the_declared_vocabulary(self):
        cases = [
            sample(window=wall_window(0.4)),
            sample(window=free_window()),
            sample(mux_block_reason="disabled"),
        ]
        for case in cases:
            self.assertIn(zcd.classify(case)["cause"], zcd.ALL_CAUSES)


@unittest.skipUnless(SEALED_RUN.is_dir(), "sealed run evidence not present")
class SealedRunReplayTest(unittest.TestCase):
    """Replay regression over the real L run that motivated the classifier."""

    @classmethod
    def setUpClass(cls):
        cls.paths = sorted(SEALED_RUN.glob("*_failure_[0-9][0-9][0-9][0-9].json"))
        cls.results = [zcd.classify_failure_file(str(path)) for path in cls.paths]

    def test_all_110_failure_artifacts_are_replayed(self):
        self.assertEqual(len(self.paths), 110)

    def test_every_failure_is_a_start_pose_clearance_deadlock(self):
        counts = Counter(result["cause"] for result in self.results)
        self.assertEqual(counts, Counter({zcd.PLANNER_INFEASIBLE_START_POSE: 110}))

    def test_every_failure_measures_below_the_required_clearance(self):
        for result in self.results:
            evidence = result["evidence"]
            self.assertLess(evidence["lethal_clearance_m"],
                            evidence["required_clearance_m"],
                            result["failure_id"])
            self.assertEqual(evidence["required_clearance_m"], 0.52)

    def test_planner_never_saw_a_nonzero_command_at_trigger(self):
        for result in self.results:
            self.assertEqual(result["evidence"]["planner_command"], [0.0, 0.0],
                             result["failure_id"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
