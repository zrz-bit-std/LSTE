#!/usr/bin/env python3
"""Join a sealed immutable suite bag with its bound observer's new mask topic."""
import argparse,hashlib,heapq,json
from pathlib import Path
LOCAL_MASK='/lste/place_navigation_mask_odom'


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def merge(main,sidecar,contract,output):
    import rosbag
    main,sidecar,contract,output=map(Path,(main,sidecar,contract,output))
    if output.exists() or output.resolve() in (main.resolve(),sidecar.resolve()):raise ValueError('output must be new; originals are immutable')
    if any(Path(str(p)+'.active').exists() or p.suffix!='.bag' for p in (main,sidecar)):raise ValueError('inputs must be sealed bags')
    c=json.loads(contract.read_text());cmd=c['command']
    if c.get('recorder_exit')!=0 or Path(cmd[cmd.index('-O')+1]).resolve()!=sidecar.resolve():
        raise ValueError('sidecar must match the successfully sealed bound-driver contract')
    if not c.get('driver') or not c.get('start_ticks'):raise ValueError('missing bound driver identity')
    counts={};total=0
    with rosbag.Bag(str(main)) as a,rosbag.Bag(str(sidecar)) as b:
        if LOCAL_MASK in a.get_type_and_topic_info().topics:raise ValueError('main bag already records physical mask; no merge needed')
        if LOCAL_MASK not in b.get_type_and_topic_info().topics:raise ValueError('sidecar has no physical mask evidence')
        streams=(a.read_messages(raw=True),b.read_messages(topics=[LOCAL_MASK],raw=True))
        with rosbag.Bag(str(output),'w') as out:
            for topic,message,t in heapq.merge(*streams,key=lambda row:row.timestamp.to_nsec()):
                out.write(topic,message,t,raw=True);counts[topic]=counts.get(topic,0)+1;total+=1
    result=dict(main=str(main),main_sha256=sha(main),sidecar=str(sidecar),sidecar_sha256=sha(sidecar),
        bound_contract=str(contract),driver=c['driver'],start_ticks=c['start_ticks'],output=str(output),
        output_sha256=sha(output),message_counts=counts,total=total,
        contract='Only original local-mask records added, with original ROS payloads and bag timestamps. Inputs untouched; no fabricated historical records.')
    Path(str(output)+'.provenance.json').write_text(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('main','sidecar','contract','output'):p.add_argument(name,type=Path)
    a=p.parse_args();r=merge(a.main,a.sidecar,a.contract,a.output);print(json.dumps({k:v for k,v in r.items() if k!='message_counts'},indent=2))
if __name__=='__main__':main()
