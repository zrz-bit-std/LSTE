"""Linear string-event aggregation retains the exact old report values."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tools'))
from analyze_navigation_metrics import bridge_event_counts
class AnalysisCountsTest(unittest.TestCase):
 def old(self,records):
  return {str(r.get('bridge_event','unknown')):sum(x.get('bridge_event','unknown')==r.get('bridge_event','unknown') for x in records) for r in records}
 def test_all_string_events_equal_original(self):
  records=[{'bridge_event':name} for name in ('status','dispatch','status','terminal','status')]+[{}]
  self.assertEqual(bridge_event_counts(records),self.old(records))
 def test_legacy_nonstring_and_string_key_collisions_keep_old_behavior(self):
  records=[{'bridge_event':name} for name in (None,1,True,'1','None',[],{},[],{})]
  self.assertEqual(bridge_event_counts(records),self.old(records))
 def test_empty_and_large_regular_stream(self):
  self.assertEqual(bridge_event_counts([]),{})
  result=bridge_event_counts([{'bridge_event':'feedback'}]*50000+[{'bridge_event':'terminal'}]*7)
  self.assertEqual(result,{'feedback':50000,'terminal':7})
if __name__=='__main__':unittest.main()
