#!/usr/bin/env python3
"""Capture one frame from every /cam_* topic that appears, then exit."""
import os
import sys

import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image

OUT = os.environ["PROBE_OUT"]
EXPECT = int(os.environ.get("EXPECT", "6"))

os.makedirs(OUT, exist_ok=True)
bridge = cv_bridge.CvBridge()
saved = {}
subs = []


def cb(name, msg):
    if name in saved:
        return
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imwrite(os.path.join(OUT, name + ".png"), img)
    saved[name] = 1
    rospy.loginfo("saved %s %s", name, img.shape)


def main():
    rospy.init_node("render_capture", anonymous=True)
    for c in sorted(os.environ["CAMS"].split(",")):
        subs.append(rospy.Subscriber("/%s/image_raw" % c, Image, lambda m, n=c: cb(n, m), queue_size=1))
    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and len(saved) < EXPECT:
        rate.sleep()
    print("CAPTURED", len(saved), sorted(saved))
    return 0 if len(saved) == EXPECT else 1


if __name__ == "__main__":
    sys.exit(main())
