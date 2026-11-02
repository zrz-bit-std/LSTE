#!/usr/bin/env python3
"""The metrics TextIOWrapper must have one writer across ROS callbacks."""
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_metrics_logging import NavigationMetricsLoggingMixin
class OverlapDetectingStream:
    def __init__(self):self.busy=False;self.overlaps=0;self.lines=[]
    def write(self,line):
        if self.busy:self.overlaps+=1
        self.busy=True
        time.sleep(.0002)
        self.lines.append(line)
        self.busy=False
class LoggingConcurrencyTest(unittest.TestCase):
    def test_callbacks_serialize_whole_records(self):
        o=SimpleNamespace(log_write_lock=threading.RLock(),stream=OverlapDetectingStream(),process_name='test')
        def writer(i):
            for j in range(30):NavigationMetricsLoggingMixin._write(o,'INFO','sample',callback=i,index=j,ros_time=1.)
        threads=[threading.Thread(target=writer,args=(i,)) for i in range(8)]
        with patch("navigation_metrics_logging.rospy.Time.now",return_value=SimpleNamespace(to_sec=lambda:1.)):
            for t in threads:t.start()
            for t in threads:t.join()
        self.assertEqual(o.stream.overlaps,0)
        records=[json.loads(l.split(' data=',1)[1]) for l in o.stream.lines]
        self.assertEqual(len({(d['callback'],d['index']) for d in records}),240)
if __name__=='__main__':unittest.main()
