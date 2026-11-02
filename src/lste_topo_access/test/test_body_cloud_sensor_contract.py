#!/usr/bin/env python3
"""Prevent frame-height errors and uncleared low obstacles in the cloud stack."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

WS = Path(__file__).resolve().parents[3]


class BodyCloudSensorContractTest(unittest.TestCase):
    def test_ray_origin_matches_its_published_lidar_frame(self):
        root = ET.parse(WS / 'src/tools/ouster_description/urdf/OS1-64.urdf.xacro').getroot()
        macro = next(node for node in root if node.tag.endswith('}macro') and node.get('name') == 'OS1-64')
        joint = next(j for j in macro.findall('joint')
                     if j.find('child').get('link') == '${lidar_link}')
        frame_xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
        sensor = macro.find("gazebo[@reference='${name}']/sensor")
        self.assertEqual(sensor.findtext('plugin/frameName'), '${lidar_link}')
        ray_xyz = [float(v) for v in sensor.findtext('pose').split()[:3]]
        for actual, expected in zip(ray_xyz, frame_xyz):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_both_costmaps_mark_low_objects_and_clear_floor_rays(self):
        config = yaml.safe_load((WS / 'src/lste_topo_access/config/body_cloud_costmap.yaml').read_text())
        for name in ('global_costmap', 'local_costmap'):
            layer = config[name]['obstacle_layer']
            sources = [layer.get(s, {}) for s in layer['observation_sources'].split()]
            low_markers = [s for s in sources if s.get('data_type') == 'PointCloud2'
                           and s.get('marking') and 0 < s.get('min_obstacle_height', 0) < .1]
            ground_clearers = [s for s in sources if s.get('data_type') == 'PointCloud2'
                               and s.get('clearing') and not s.get('marking')
                               and s.get('min_obstacle_height', 1) <= 0]
            self.assertTrue(low_markers, name)
            # The live Pro3 Kinect body spans 0.534..0.634 m. Those
            # collisions are below the 2-D laser plane and need 3-D marking.
            self.assertTrue(any(s.get('max_obstacle_height',0) >= .634
                                for s in low_markers), name)
            self.assertTrue(ground_clearers, name)


if __name__ == '__main__':
    unittest.main()
