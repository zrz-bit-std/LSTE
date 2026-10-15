#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Header
from lste_msgs.msg import LsteTask, LsteDetection, LsteDetections

class DetFakeNode:
    def __init__(self):
        rospy.init_node("lste_det_fake_node")

        self.task = None
        self.sub_task = rospy.Subscriber("/lste/task", LsteTask, self.on_task, queue_size=1)
        self.pub = rospy.Publisher("/lste/detections", LsteDetections, queue_size=5)

        self.rate_hz = rospy.get_param("~rate_hz", 2.0)
        self.frame_id = rospy.get_param("~frame_id", "camera_color_optical_frame")

        rospy.loginfo("lste_det_fake_node started, waiting for /lste/task ...")

    def on_task(self, msg):
        self.task = msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        t = 0
        while not rospy.is_shutdown():
            if self.task is None:
                rate.sleep()
                continue

            out = LsteDetections()
            out.header = Header()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.frame_id
            out.task_id = self.task.task_id

            # Fake target detection (bbox slowly moving)
            det = LsteDetection()
            det.label = self.task.target_name
            det.score = 0.6 + 0.3 * ((t % 20) / 20.0)   # 0.6~0.885
            det.cx = 0.5 + 0.1 * (0.5 - ((t % 10) / 10.0))
            det.cy = 0.5
            det.w = 0.2
            det.h = 0.3
            out.target_dets = [det]

            # Fake env detections
            env1 = LsteDetection()
            env1.label = "door"
            env1.score = 0.5
            env1.cx, env1.cy, env1.w, env1.h = 0.2, 0.5, 0.2, 0.5

            env2 = LsteDetection()
            env2.label = "exit sign"
            env2.score = 0.4
            env2.cx, env2.cy, env2.w, env2.h = 0.8, 0.2, 0.1, 0.1

            out.env_dets = [env1, env2]

            out.prompt_a = "FAKE_PROMPT_A"
            out.prompt_b_terms = ["door", "wall", "exit sign"]

            self.pub.publish(out)
            t += 1
            rate.sleep()

if __name__ == "__main__":
    DetFakeNode().spin()
