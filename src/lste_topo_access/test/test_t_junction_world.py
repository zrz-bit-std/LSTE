"""Structural contract for the fast physical T-junction benchmark."""

from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET


WORLD = Path(__file__).resolve().parents[3] / (
    "worlds/targeted_navigation/t_junction_small.world"
)


class TJunctionWorldTest(unittest.TestCase):
    def test_world_has_one_open_branch_and_no_embedded_robot(self):
        root = ET.parse(str(WORLD)).getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.attrib["name"], "lste_t_junction_small")

        models = {model.attrib["name"]: model for model in world.findall("model")}
        self.assertEqual(set(models), {"ground_plane", "t_junction_structure"})
        structure = models["t_junction_structure"]
        self.assertEqual(structure.findtext("static"), "true")

        collisions = {
            collision.attrib["name"]: collision
            for collision in structure.findall("./link/collision")
        }
        self.assertEqual(
            set(collisions),
            {
                "south_wall",
                "north_wall_west",
                "north_wall_east",
                "west_end_wall",
                "east_end_wall",
                "branch_west_wall",
                "branch_east_wall",
                "branch_end_wall",
            },
        )

        # The north-wall segments cover x=[-5,1] and x=[3,5].  Their two-metre
        # gap leaves exactly x=[1,3] open for the north branch.
        west_pose = [float(v) for v in collisions["north_wall_west"].findtext("pose").split()]
        west_size = [
            float(v)
            for v in collisions["north_wall_west"].find("./geometry/box/size").text.split()
        ]
        east_pose = [float(v) for v in collisions["north_wall_east"].findtext("pose").split()]
        east_size = [
            float(v)
            for v in collisions["north_wall_east"].find("./geometry/box/size").text.split()
        ]
        self.assertTrue(math.isclose(west_pose[1], 1.25))
        self.assertTrue(math.isclose(east_pose[1], 1.25))
        self.assertEqual(west_size, [6.0, 0.15, 2.5])
        self.assertEqual(east_size, [2.0, 0.15, 2.5])
        self.assertTrue(math.isclose(west_pose[0] + west_size[0] / 2.0, 1.0))
        self.assertTrue(math.isclose(east_pose[0] - east_size[0] / 2.0, 3.0))

        for name, expected_x in (("branch_west_wall", 1.0), ("branch_east_wall", 3.0)):
            pose = [float(v) for v in collisions[name].findtext("pose").split()]
            size = [
                float(v)
                for v in collisions[name].find("./geometry/box/size").text.split()
            ]
            self.assertTrue(math.isclose(pose[0], expected_x))
            self.assertEqual(size, [0.15, 5.75, 2.5])

    def test_launch_defaults_match_world_start(self):
        launch = Path(__file__).resolve().parents[3] / (
            "scripts/tests/targeted_navigation/t_junction_nav.launch"
        )
        text = launch.read_text(encoding="utf-8")
        self.assertIn("t_junction_small.world", text)
        self.assertIn('default="-2.5"', text)
        self.assertIn('default="0.0"', text)
        self.assertIn('global_frontier_enabled" default="true"', text)
        self.assertIn('start_goal_manager" default="true"', text)
        self.assertIn('start_frontier" default="true"', text)
        self.assertIn('persistent_execution" default="false"', text)
        self.assertIn('teb_forward_only" default="true"', text)

    def test_runner_exposes_persistent_diagnostic_switch(self):
        runner = Path(__file__).resolve().parents[3] / (
            "scripts/tests/targeted_navigation/run_t_junction.sh"
        )
        text = runner.read_text(encoding="utf-8")
        self.assertIn("--persistent", text)
        self.assertIn('"persistent_execution:=$PERSISTENT_EXECUTION"', text)
        self.assertIn(
            'BASE_GLOBAL_PLANNER="lste_topo_access/StreamingNavfnPlanner"',
            text,
        )
        self.assertIn(
            'BASE_LOCAL_PLANNER="lste_topo_access/PersistentTebLocalPlanner"',
            text,
        )
        self.assertIn('PLANNER_FREQUENCY="${T_JUNCTION_PLANNER_FREQUENCY:-20.0}"', text)
        self.assertIn('"planner_frequency:=$PLANNER_FREQUENCY"', text)

    def test_short_runners_make_cleanup_idempotent_and_record_final_outcome(self):
        root = Path(__file__).resolve().parents[3]
        for name in (
            "run_t_junction.sh",
            "run_t_junction_endpoint.sh",
        ):
            text = (root / "scripts/tests/targeted_navigation" / name).read_text(
                encoding="utf-8"
            )
            self.assertIn("CLEANUP_DONE=false", text)
            self.assertIn('if [[ "$CLEANUP_DONE" == "true" ]]', text)
            self.assertIn("log_lifecycle INFO run_outcome", text)

    def test_persistent_runner_wakes_streaming_navfn_without_changing_endpoint_default(self):
        launch = Path(__file__).resolve().parents[3] / (
            "scripts/tests/targeted_navigation/portal_retry_single_door_nav.launch"
        )
        text = launch.read_text(encoding="utf-8")
        self.assertIn('arg name="planner_frequency" default="0.0"', text)
        self.assertIn('value="$(arg planner_frequency)"', text)
        self.assertIn('arg name="move_base_respawn" default="false"', text)

    def test_target_failure_probe_is_bounded_and_writes_a_process_result(self):
        root = Path(__file__).resolve().parents[3]
        probe = root / "scripts/tests/targeted_navigation/publish_target_mission.py"
        runner = root / "scripts/tests/targeted_navigation/run_t_junction.sh"
        probe_text = probe.read_text(encoding="utf-8")
        runner_text = runner.read_text(encoding="utf-8")
        self.assertIn('"target_plan_failed"', probe_text)
        self.assertIn('"target_controller_lease_released"', probe_text)
        self.assertIn('"frontier_takeover"', probe_text)
        self.assertIn("_target_probe.log", probe_text)
        self.assertIn("_target_probe_result.json", probe_text)
        self.assertIn("--target-transaction-id", runner_text)
        self.assertIn(
            'TARGET_PROBE_TRANSACTION_ID="${T_JUNCTION_TARGET_PROBE_TRANSACTION_ID:-900}"',
            runner_text,
        )

    def test_endpoint_diagnostic_omits_frontier_but_keeps_slam_and_metrics(self):
        launch = Path(__file__).resolve().parents[3] / (
            "scripts/tests/targeted_navigation/t_junction_endpoint_nav.launch"
        )
        text = launch.read_text(encoding="utf-8")
        self.assertIn('start_frontier" value="false"', text)
        self.assertIn('online_slam_enabled" type="bool" value="true"', text)
        self.assertIn('global_frontier_enabled" type="bool" value="false"', text)
        self.assertIn('start_metrics" default="false"', text)

    def test_endpoint_runner_can_close_immediately_after_success(self):
        runner = Path(__file__).resolve().parents[3] / (
            "scripts/tests/targeted_navigation/run_t_junction_endpoint.sh"
        )
        text = runner.read_text(encoding="utf-8")
        self.assertIn(
            'STOP_ON_SUCCESS="${T_JUNCTION_ENDPOINT_STOP_ON_SUCCESS:-true}"',
            text,
        )
        self.assertIn("--no-stop-on-success", text)
        self.assertIn('"status_name":"SUCCEEDED"', text)
        self.assertIn('STOP_REASON="goal_succeeded"', text)

    def test_optional_metrics_entrypoint_keeps_run_identity_as_strings(self):
        launch = Path(__file__).resolve().parents[3] / (
            "scripts/tests/targeted_navigation/t_junction_nav.launch"
        )
        text = launch.read_text(encoding="utf-8")
        self.assertIn('start_metrics" default="false"', text)
        self.assertIn(
            '<param name="run_directory" type="str" value="$(arg metrics_run_directory)"/>',
            text,
        )
        self.assertIn(
            '<param name="run_timestamp" type="str" value="$(arg metrics_run_timestamp)"/>',
            text,
        )
        self.assertIn(
            '<param name="failure_evidence_post_window" type="double" value="$(arg metrics_failure_post_window)"/>',
            text,
        )


if __name__ == "__main__":
    unittest.main()
