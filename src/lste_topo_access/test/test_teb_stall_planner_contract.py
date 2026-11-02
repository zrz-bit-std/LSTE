"""Guard the TEB topology recovery fix and keep collision limits unchanged."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "teb_navigation.launch"


class TebStallPlannerContractTest(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(str(LAUNCH)).getroot()

    def arg(self, name):
        for item in self.root.findall("arg"):
            if item.get("name") == name:
                return item.get("default")
        self.fail("missing launch arg: " + name)

    def param(self, name):
        for item in self.root.iter("param"):
            if item.get("name") == name:
                return item.get("value")
        self.fail("missing launch param: " + name)

    def test_multiple_topologies_are_enabled_inside_existing_move_base_action(self):
        self.assertEqual(self.arg("teb_homotopy_class_planning"), "true")
        self.assertEqual(self.param("TebLocalPlannerROS/enable_homotopy_class_planning"),
                         "$(arg teb_homotopy_class_planning)")
        self.assertEqual(self.param("TebLocalPlannerROS/max_number_classes"), "5")
        self.assertEqual(self.param("TebLocalPlannerROS/enable_multithreading"), "true")
        self.assertEqual(self.param("TebLocalPlannerROS/viapoints_all_candidates"), "false")

    def test_recovery_does_not_weaken_collision_or_reverse_limits(self):
        self.assertEqual(self.param("TebLocalPlannerROS/footprint_model/radius"), "0.30")
        self.assertEqual(self.param("TebLocalPlannerROS/min_obstacle_dist"), "0.22")
        self.assertEqual(self.param("TebLocalPlannerROS/inflation_dist"), "0.55")
        self.assertEqual(self.param("TebLocalPlannerROS/max_vel_x_backwards"), "0.01")
        self.assertEqual(self.param("TebLocalPlannerROS/feasibility_check_no_poses"), "3")

    def test_alternative_topology_does_not_enable_goal_replacement(self):
        self.assertEqual(self.arg("allow_in_place_replacement"), "false")
        self.assertEqual(self.arg("allow_route_continuation_replacement"), "false")


if __name__ == "__main__":
    unittest.main()
