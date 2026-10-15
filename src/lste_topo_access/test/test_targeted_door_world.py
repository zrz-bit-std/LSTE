"""Structural regression checks for the minimal targeted Gazebo world."""

from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET


WORLD = Path(__file__).resolve().parents[3] / (
    "worlds/targeted_navigation/portal_retry_single_door.world"
)


class TargetedDoorWorldTest(unittest.TestCase):
    def test_world_has_one_explicit_doorway_and_no_embedded_robot(self):
        root = ET.parse(str(WORLD)).getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.attrib["name"], "lste_portal_retry_single_door")

        models = {model.attrib["name"]: model for model in world.findall("model")}
        self.assertEqual(set(models), {"ground_plane", "single_door_structure"})
        structure = models["single_door_structure"]
        self.assertEqual(structure.findtext("static"), "true")

        collisions = {
            collision.attrib["name"]: collision
            for collision in structure.findall("./link/collision")
        }
        self.assertEqual(
            set(collisions),
            {
                "north_wall",
                "south_wall",
                "west_wall",
                "east_wall",
                "divider_lower",
                "divider_upper",
            },
        )

        # The two divider segments occupy y=[-2,-0.5] and y=[0.5,2],
        # leaving exactly one 1 m central opening for the robot footprint.
        for name, expected_y in (("divider_lower", -1.25), ("divider_upper", 1.25)):
            pose = [float(value) for value in collisions[name].findtext("pose").split()]
            size = [
                float(value)
                for value in collisions[name].find("./geometry/box/size").text.split()
            ]
            self.assertTrue(math.isclose(pose[0], 0.0))
            self.assertTrue(math.isclose(pose[1], expected_y))
            self.assertEqual(size, [0.15, 1.5, 2.5])


if __name__ == "__main__":
    unittest.main()
