"""Same-epoch native mask coordinates may restrict, never expand, a Place."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_place_mask_projection import physical_scope_raster
from global_frontier_geometry import inverse_planar_projector
class PhysicalMaskTest(unittest.TestCase):
 def fixture(self):
  m=S(info=S(width=8,height=7,resolution=.1,origin=S(position=S(x=0.,y=0.),orientation=S(w=1.,x=0.,y=0.,z=0.))))
  allowed=np.ones((7,8),dtype=bool);allowed[:,4:]=False;return m,allowed
 def test_identity_preserves_interiors_and_excludes_outside_source(self):
  m,a=self.fixture();origin,res,data=physical_scope_raster(m,a,lambda x,y:(x,y))
  self.assertEqual(origin,(-.1,-.1));self.assertTrue(np.all(data[1:8,1:9]==np.where(a,-1,100)));self.assertTrue(np.all(data[0]==100))
 def test_rotated_translated_mask_does_not_admit_any_excluded_source_point(self):
  m,a=self.fixture();theta=.37;c,s=math.cos(theta),math.sin(theta)
  project=lambda x,y:(.3+c*x-s*y,-.2+s*x+c*y);inverse=inverse_planar_projector(project)
  origin,res,data=physical_scope_raster(m,a,project)
  for y in np.arange(.005,.7,.01):
   for x in np.arange(.405,.8,.01):
    px,py=inverse(x,y);r=int(math.floor((py-origin[1])/res));col=int(math.floor((px-origin[0])/res))
    self.assertEqual(data[r,col],100)
 def test_all_transparent_authorization_stays_transparent(self):
  m,a=self.fixture();_,_,data=physical_scope_raster(m,None,lambda x,y:(x+.2,y-.1));self.assertFalse(np.any(data==100))
 def test_wrong_shape_or_nonrigid_transform_refused(self):
  m,a=self.fixture()
  with self.assertRaises(ValueError):physical_scope_raster(m,a,lambda x,y:(2*x,y))
  with self.assertRaises(ValueError):physical_scope_raster(m,a[:2],lambda x,y:(x,y))
class PhysicalMaskInstallTest(unittest.TestCase):
 def test_all_project_siblings_reachable_from_gp_and_constraints_are_installed(self):
  import ast
  root=Path(__file__).resolve().parents[1]/'scripts';cm=(root.parent/'CMakeLists.txt').read_text()
  todo=['gp_branch_frontier','global_frontier_place_navigation'];seen=set()
  while todo:
   name=todo.pop()
   if name in seen:continue
   seen.add(name);p=root/(name+'.py')
   self.assertIn('scripts/'+p.name,cm)
   for n in ast.walk(ast.parse(p.read_text())):
    modules=[n.module] if isinstance(n,ast.ImportFrom) else [a.name for a in n.names] if isinstance(n,ast.Import) else []
    todo.extend(m for m in modules if m and (root/(m+'.py')).exists())
if __name__=='__main__':unittest.main()
