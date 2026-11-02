"""A locally displaced certified section must keep its physical side identity."""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_place_partition import refine_portal_cut,partition_place,leaking_portal_sides

class PortalCutRefinementTest(unittest.TestCase):
    def setUp(self):
        self.free=np.zeros((100,100),dtype=bool)
        self.free[35:65,5:95]=True
        self.free[50:95,35:65]=True
        self.portal=dict(id=3,gate=[6.3,5.],destination=[8.5,5.],width=3.,
                         bounds=[[6.3,3.5],[6.3,6.5]])
    def partition(self,p):
        return partition_place(self.free,.1,(0.,0.),self.portal['destination'],[p],1)[0]
    def test_junction_corner_correction_separates_original_sides(self):
        self.assertEqual(leaking_portal_sides(self.partition(self.portal),self.free,.1,(0,0),[self.portal]),[3])
        result=refine_portal_cut(self.portal,self.free,.1,(0,0))
        mask=self.partition(result)
        self.assertEqual(leaking_portal_sides(mask,self.free,.1,(0,0),[result]),[])
        self.assertFalse(mask[50,55]);self.assertTrue(mask[50,85])
        self.assertEqual(result['gate'],self.portal['gate'])
        self.assertEqual(result['destination'],self.portal['destination'])
        self.assertLessEqual(abs(result['section_translation_m']),1.5)
    def test_unbounded_open_area_stays_rejected(self):
        self.free[:]=True
        result=refine_portal_cut(self.portal,self.free,.1,(0,0))
        self.assertEqual(result['section_translation_m'],0.)
        self.assertEqual(leaking_portal_sides(self.partition(result),self.free,.1,(0,0),[result]),[3])
    def test_anchored_section_is_not_moved(self):
        self.portal['bounds']=[[7.,3.5],[7.,6.5]]
        result=refine_portal_cut(self.portal,self.free,.1,(0,0))
        self.assertEqual(result['section_translation_m'],0.)
    def test_overextended_endpoints_are_trimmed_back_to_near_wall(self):
        free=np.zeros((100,100),dtype=bool)
        # A mapped vertical passage is bounded by walls near x=+-1.1.
        free[:,39:62]=True
        portal=dict(id=4,gate=[0.,1.5],destination=[0.,3.5],width=2.8,
                    bounds=[[-1.42,1.5],[1.47,1.5]])
        refined=refine_portal_cut(portal,free,.1,(-5.,-1.))
        self.assertLess(refined['bounds'][0][0],-1.0)
        self.assertGreater(refined['bounds'][0][0],-1.3)
        self.assertGreater(refined['bounds'][1][0],1.0)
        self.assertLess(refined['bounds'][1][0],1.3)
        mask,_,label,_=partition_place(free,.1,(-5.,-1.),portal['destination'],[refined],1)
        self.assertGreater(label,0)
        self.assertEqual(leaking_portal_sides(mask,free,.1,(-5.,-1.),[refined]),[])
        self.assertEqual(refined['gate'],portal['gate'])
        self.assertEqual(refined['destination'],portal['destination'])

    def test_correction_cannot_pass_destination_witness(self):
        self.portal['destination']=[6.4,5.]
        result=refine_portal_cut(self.portal,self.free,.1,(0,0))
        self.assertEqual(result['section_translation_m'],0.)

if __name__=='__main__':unittest.main()
