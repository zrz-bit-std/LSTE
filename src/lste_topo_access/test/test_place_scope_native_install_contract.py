"""Packaging/frame contract; behavior is additionally exercised by real C++ probe."""
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[1]
class NativeScopeInstallContract(unittest.TestCase):
 def test_global_persistent_local_physical(self):
  c=yaml.safe_load((ROOT/'config/place_navigation_costmap.yaml').read_text())
  g={p['name']:p['type'] for p in c['global_costmap']['plugins']}
  l={p['name']:p['type'] for p in c['local_costmap']['plugins']}
  self.assertEqual(g['place_scope_layer'],'lste_topo_access::PlaceScopeLayer')
  self.assertEqual(l['place_scope_layer'],'costmap_2d::StaticLayer')
  self.assertEqual(c['local_costmap']['place_scope_layer']['map_topic'],'/lste/place_navigation_mask_odom')
  self.assertEqual(c['global_costmap']['obstacle_layer']['combination_method'],1)
 def test_export_and_install(self):
  p=ET.parse(ROOT/'place_scope_plugins.xml').getroot()
  self.assertEqual(p.attrib['path'],'lib/liblste_place_scope_layer')
  self.assertEqual(p.find('class').attrib['base_class_type'],'costmap_2d::Layer')
  exports=ET.parse(ROOT/'package.xml').getroot().findall('./export/costmap_2d')
  self.assertIn('${prefix}/place_scope_plugins.xml',[e.attrib['plugin'] for e in exports])
  c=(ROOT/'CMakeLists.txt').read_text()
  self.assertIn('install(TARGETS lste_place_scope_layer',c)
  self.assertIn('install(FILES place_scope_plugins.xml',c)
if __name__=='__main__':unittest.main()
