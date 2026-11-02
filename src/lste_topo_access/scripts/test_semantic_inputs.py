#!/usr/bin/env python3
"""Stable task/prompt input for the real-WeDetect semantic Gazebo tests."""
import rospy
from lste_msgs.msg import LstePrompts, LsteState, LsteTask


def main():
    rospy.init_node("portal_semantic_test_inputs")
    task_pub = rospy.Publisher("/lste/task", LsteTask, queue_size=1, latch=True)
    prompts_pub = rospy.Publisher("/lste/prompts", LstePrompts, queue_size=1, latch=True)
    state_pub = rospy.Publisher("/lste/state", LsteState, queue_size=1, latch=True)

    task = LsteTask()
    task.task_id = "portal_semantic_test"
    task.target_name = ""
    task.env_type_prior = ["office", "living room", "bedroom"]
    task.env_related_structures = ["room", "furniture"]
    task.obj_key_objects = ["chair", "desk", "table", "cup", "sofa", "bed", "plant"]
    task.raw_json = '{"target":{"name":"","attributes":[]},"env":{"env_type_prior":["office","living room","bedroom"],"related_structures":["room","furniture"]},"obj_related":{"key_objects":["chair","desk","table","cup","sofa","bed","plant"]}}'

    prompts = LstePrompts()
    prompts.task_id = task.task_id
    prompts.prompt_a = "room"
    prompts.prompt_b_terms = ["chair", "desk", "table", "cup", "sofa", "bed", "plant", "cabinet", "bookshelf", "lamp"]
    prompts.raw_llm_output = "fixed semantic-junction test prompts"

    state = LsteState()
    state.task_id = task.task_id
    state.state = 0
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        prompts_pub.publish(prompts)
        task_pub.publish(task)
        state.header.stamp = rospy.Time.now()
        state_pub.publish(state)
        rate.sleep()


if __name__ == "__main__":
    main()
