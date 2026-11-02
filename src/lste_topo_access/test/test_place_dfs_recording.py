"""Missing native raw evidence cannot pass the formal suite prerequisite."""
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from check_place_dfs_recording import audit_counts,REQUIRED_TOPICS
class RecordingTest(unittest.TestCase):
    def full(self):return {topic:2 for topic in REQUIRED_TOPICS}
    def test_every_required_topic_is_mandatory(self):
        for topic in REQUIRED_TOPICS:
            counts=self.full();counts[topic]=0
            result=audit_counts(counts)
            self.assertFalse(result['passed']);self.assertIn(topic,result['missing_topics'])
    def test_repeated_full_grids_do_not_require_incremental_publication(self):
        self.assertTrue(audit_counts(self.full())['passed'])
    def test_incremental_stream_requires_its_initial_full_grid(self):
        counts=self.full();counts['/move_base/local_costmap/costmap']=1
        self.assertFalse(audit_counts(counts)['passed'])
        counts['/move_base/local_costmap/costmap_updates']=5
        self.assertTrue(audit_counts(counts)['passed'])
        counts['/move_base/local_costmap/costmap']=0
        self.assertFalse(audit_counts(counts)['passed'])
if __name__=='__main__':unittest.main()
