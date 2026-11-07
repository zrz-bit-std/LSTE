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
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from lste_msgs.msg import LsteState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--pose-out", default="")
    args = parser.parse_args()
    pose_out = args.pose_out or (args.out + ".pose")

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
    pose_stream = open(pose_out, "a", buffering=1)

    def write(payload):
        stream.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def write_pose(payload):
        pose_stream.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def on_state(msg):
        write({"kind": "state", "wall": time.time(),
               "ros": msg.header.stamp.to_sec(), "task_id": msg.task_id,
               "state": int(msg.state), "subtype": msg.subtype})

    def on_done(msg):
        write({"kind": "task_done", "wall": time.time(),
               "ros": rospy.Time.now().to_sec(), "value": bool(msg.data)})

    def on_pose(msg):
        write_pose({"kind": "pose", "wall": time.time(),
                    "ros": rospy.Time.now().to_sec(),
                    "x": float(msg.x), "y": float(msg.y),
                    "theta": float(msg.theta)})

    rospy.Subscriber("/lste/state", LsteState, on_state, queue_size=50)
    rospy.Subscriber("/lste/task_done", Bool, on_done, queue_size=10)
    rospy.Subscriber("/rbt_pose", Pose2D, on_pose, queue_size=50)
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
