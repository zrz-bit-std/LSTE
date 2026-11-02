"""The scene runner may not finish when only the old four acceptance gates pass."""
from pathlib import Path
import sys,tempfile,json,unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
import check_semantic_junction_goal as goal


class SemanticDfsGateTest(unittest.TestCase):
    def test_incomplete_dfs_prevents_success_even_if_all_old_checks_pass(self):
        for passed in (False,True):
            with tempfile.TemporaryDirectory() as directory:
                output=Path(directory)/'result.json'
                argv=['check_semantic_junction_goal.py','metrics.log','--headings','0','1.57','3.14','--output',str(output)]
                with patch.object(sys,'argv',argv),patch.object(goal,'check',return_value={'passed':True}), \
                     patch.object(goal,'authentication_audit',return_value={'passed':True}), \
                     patch.object(goal,'lifecycle_audit',return_value={'passed':True}), \
                     patch.object(goal,'tsp_audit',return_value={'passed':True}), \
                     patch.object(goal,'dfs_audit',return_value={'passed':passed}) as dfs:
                    with self.assertRaises(SystemExit) as error:goal.main()
                self.assertEqual(error.exception.code,0 if passed else 1)
                self.assertEqual(json.loads(output.read_text())['passed'],passed)
                dfs.assert_called_once_with('metrics.log',2)

if __name__=='__main__':unittest.main()
