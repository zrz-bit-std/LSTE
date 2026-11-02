"""Preserve original input bytes/times and reject unsealed or wrong sidecars."""
import sys,json,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
import rosbag,genpy
from std_msgs.msg import String
from merge_place_mask_sidecar import merge,LOCAL_MASK
class SidecarMergeTest(unittest.TestCase):
 def test_raw_input_times_preserved_without_duplicate_global_topic(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);a=root/'main.bag';b=root/'sidecar.bag';out=root/'joined.bag';c=root/'contract.json'
   with rosbag.Bag(str(a),'w') as bag:bag.write('/original',String('original'),genpy.Time.from_sec(1.))
   with rosbag.Bag(str(b),'w') as bag:
    bag.write('/ignored_global',String('duplicate'),genpy.Time.from_sec(.5));bag.write(LOCAL_MASK,String('physical'),genpy.Time.from_sec(2.))
   c.write_text(json.dumps(dict(command=['rosbag','record','-O',str(b)],driver=123,start_ticks='456',recorder_exit=0)))
   r=merge(a,b,c,out);self.assertEqual(r['message_counts'],{'/original':1,LOCAL_MASK:1})
   with rosbag.Bag(str(out)) as bag:
    records=list(bag.read_messages());self.assertEqual([(m.topic,m.message.data,m.timestamp.to_sec()) for m in records],[('/original','original',1.),(LOCAL_MASK,'physical',2.)])
   with self.assertRaises(ValueError):merge(a,b,c,out)
   c.write_text(json.dumps(dict(command=['rosbag','record','-O',str(b)],driver=123,start_ticks='456',recorder_exit=1)))
   with self.assertRaises(ValueError):merge(a,b,c,root/'bad.bag')
if __name__=='__main__':unittest.main()
