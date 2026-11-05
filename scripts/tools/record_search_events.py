#!/usr/bin/env python3
"""Audit-side recorder for /lste/state and /lste/task_done.

Independent of the metrics node: writes one JSON object per line so the
offline audit can verify that the perception state machine reached LOCKED
around task completion.  Waits for roscore before subscribing.
"""
import argparse
import json
import time

import rospy
from std_msgs.msg import Bool

from lste_msgs.msg import LsteState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    deadline = time.time() + 600.0
    while True:
        try:
            rospy.init_node("lste_state_audit_recorder", anonymous=True,
                            disable_signals=True)
            break
        except Exception:
            if time.time() > deadline:
                raise
            time.sleep(1.0)

    stream = open(args.out, "a", buffering=1)

    def write(payload):
        stream.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def on_state(msg):
        write({"kind": "state", "wall": time.time(),
               "ros": msg.header.stamp.to_sec(), "task_id": msg.task_id,
               "state": int(msg.state), "subtype": msg.subtype})

    def on_done(msg):
        write({"kind": "task_done", "wall": time.time(),
               "ros": rospy.Time.now().to_sec(), "value": bool(msg.data)})

    rospy.Subscriber("/lste/state", LsteState, on_state, queue_size=50)
    rospy.Subscriber("/lste/task_done", Bool, on_done, queue_size=10)
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
