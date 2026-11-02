import concurrent.futures,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_metrics_line_stream import VerifiedAsciiAppendLog
class LineStreamTest(unittest.TestCase):
 def test_threads_and_append_restart(self):
  with tempfile.TemporaryDirectory() as t:
   p=Path(t)/'metrics.log';s=VerifiedAsciiAppendLog(p)
   def write(i):s.write(json.dumps({'i':i,'payload':'x'*20000},sort_keys=True)+'\n')
   with concurrent.futures.ThreadPoolExecutor(8) as pool:list(pool.map(write,range(240)))
   s.close();s=VerifiedAsciiAppendLog(p);s.write('{"i":240}\n');s.flush();s.close()
   self.assertEqual(sorted(json.loads(x)['i'] for x in p.read_text().splitlines()),list(range(241)))
 def test_non_ascii_and_incomplete_rejected(self):
  with tempfile.TemporaryDirectory() as t:
   s=VerifiedAsciiAppendLog(Path(t)/'metrics.log')
   for x in ['é\n','{}','{}\n{}\n']:
    with self.assertRaises((UnicodeError,ValueError)):s.write(x)
   s.close();self.assertEqual((Path(t)/'metrics.log').read_bytes(),b'')
 def test_partial_kernel_write(self):
  import navigation_metrics_line_stream as module
  original=module.os.write
  with tempfile.TemporaryDirectory() as t:
   p=Path(t)/'metrics.log';s=VerifiedAsciiAppendLog(p)
   with patch.object(module.os,'write',side_effect=lambda fd,data:original(fd,data[:3])):s.write('{"ok":true}\n')
   s.close();self.assertEqual(p.read_bytes(),b'{"ok":true}\n')
 def test_readback_fault_is_not_silently_accepted(self):
  with tempfile.TemporaryDirectory() as t:
   s=VerifiedAsciiAppendLog(Path(t)/'metrics.log')
   with patch('navigation_metrics_line_stream.os.pread',return_value=b'broken'):
    with self.assertRaisesRegex(OSError,'readback mismatch'):s.write('{}\n')
   s.close()
if __name__=='__main__':unittest.main()
