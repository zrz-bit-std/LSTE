"""Independent all-cell replay of the isolated 80->120 native resize probe."""
import argparse,json
from pathlib import Path
import numpy as np

def replay(root):
 import rosbag
 mask=None;checks=[]
 with rosbag.Bag(str(root/'raw.bag')) as bag:
  for topic,m,t in bag.read_messages():
   if topic=='/resize_probe/mask' and mask is None:
    if m.info.width!=80 or m.header.frame_id!='map':raise ValueError('missing original 80-cell physical mask')
    mask=m
   if topic!='/resize_native/costmap/costmap' or mask is None or m.info.width!=120:continue
   rows,cols=np.indices((m.info.height,m.info.width))
   x=m.info.origin.position.x+(cols+.5)*m.info.resolution;y=m.info.origin.position.y+(rows+.5)*m.info.resolution
   c=np.floor((x-mask.info.origin.position.x)/mask.info.resolution).astype(int);r=np.floor((y-mask.info.origin.position.y)/mask.info.resolution).astype(int)
   overlap=(r>=0)&(c>=0)&(r<mask.info.height)&(c<mask.info.width)
   selected=np.zeros(overlap.shape,bool);rr,cc=np.where(overlap)
   data=np.asarray(mask.data).reshape(mask.info.height,mask.info.width)
   selected[rr,cc]=data[r[rr,cc],c[rr,cc]]==100
   master=np.asarray(m.data).reshape(m.info.height,m.info.width)
   checks.append(dict(stamp=t.to_sec(),original_excluded_cells=int(selected.sum()),missing=int(np.count_nonzero(master[selected]!=100))))
 result=dict(passed=bool(checks) and all(c['original_excluded_cells']==3200 and c['missing']==0 for c in checks),checks=checks,
  scope='Every original excluded cell in every recorded expanded120 master including first. No ready flag or skipped initial resize frame. Isolated plugin only, not full Gazebo.')
 (root/'independent_resize_raw_replay.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args();r=replay(a.root);print(json.dumps(r));raise SystemExit(0 if r['passed'] else 1)
