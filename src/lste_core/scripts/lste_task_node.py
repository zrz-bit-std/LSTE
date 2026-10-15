#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os

import rospy
from lste_msgs.msg import LsteTask


def build_msg_from_params():
    msg = LsteTask()
    msg.task_id = rospy.get_param("~task_id", "task_0001")
    msg.target_name = rospy.get_param("~target_name", "fire extinguisher")
    msg.target_attributes = rospy.get_param(
        "~target_attributes", ["red", "cylinder-shaped"]
    )
    msg.env_related_structures = rospy.get_param(
        "~env_related_structures", ["door", "wall", "exit sign"]
    )
    msg.env_type_prior = rospy.get_param("~env_type_prior", ["corridor"])
    msg.obj_key_objects = rospy.get_param("~obj_key_objects", ["door"])
    msg.obj_negative_clues = rospy.get_param(
        "~obj_negative_clues", ["kitchen", "bedroom", "bathroom"]
    )
    msg.ctx_left = rospy.get_param("~ctx_left", "exit sign")
    msg.ctx_right = rospy.get_param("~ctx_right", "door")
    msg.raw_json = rospy.get_param("~raw_json", "")
    return msg


def build_msg_from_json(json_path):
    msg = LsteTask()
    msg.task_id = rospy.get_param("~task_id", "task_0001")

    if not os.path.isfile(json_path):
        rospy.logwarn("JSON file '%s' not found. Falling back to params.", json_path)
        return build_msg_from_params()

    try:
        with open(json_path, "r", encoding="utf-8") as f:
            raw = f.read()
        msg.raw_json = raw  # 保留原始 JSON 文本，格式和内容完全不改

        data = json.loads(raw)
        # 既兼容 {\"task_parsed\": {...}}，也兼容直接就是 task_parsed 的情况
        task_parsed = data.get("task_parsed", data)

        target = task_parsed.get("target", {})
        msg.target_name = target.get("name", "")
        attrs = target.get("attributes", [])
        if isinstance(attrs, list):
            msg.target_attributes = [str(a) for a in attrs]

        env = task_parsed.get("env", {})
        related_structures = env.get("related_structures", [])
        if isinstance(related_structures, list):
            msg.env_related_structures = [str(x) for x in related_structures]
        env_type_prior = env.get("env_type_prior", [])
        if isinstance(env_type_prior, list):
            msg.env_type_prior = [str(x) for x in env_type_prior]

        obj_related = task_parsed.get("obj_related", {})
        key_objects = obj_related.get("key_objects", [])
        if isinstance(key_objects, list):
            msg.obj_key_objects = [str(x) for x in key_objects]
        negative_clues = obj_related.get("negative_clues", [])
        if isinstance(negative_clues, list):
            msg.obj_negative_clues = [str(x) for x in negative_clues]

        target_ctx = task_parsed.get("target_ctx", {})
        msg.ctx_left = str(target_ctx.get("left", ""))
        msg.ctx_right = str(target_ctx.get("right", ""))

        rospy.loginfo("Loaded task from JSON: %s", json_path)
        return msg
    except Exception as e:
        rospy.logerr("Failed to parse JSON '%s': %s", json_path, e)
        return build_msg_from_params()


def main():
    rospy.init_node("lste_task_node")

    pub = rospy.Publisher("/lste/task", LsteTask, queue_size=1, latch=True)

    json_path = rospy.get_param("~json_path", "")
    if json_path:
        msg = build_msg_from_json(json_path)
    else:
        msg = build_msg_from_params()

    rospy.loginfo("lste_task_node started, publishing /lste/task (latched).")
    pub.publish(msg)
    rospy.loginfo("Published /lste/task (latched). Waiting for task completion...")
    rospy.spin()


if __name__ == "__main__":
    main()
