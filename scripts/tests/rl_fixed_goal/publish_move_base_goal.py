#!/usr/bin/env python3
"""Publish the fixed test target to move_base after SLAM is available."""

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("rl_fixed_goal_move_base_goal")
    goal_x = rospy.get_param("~goal_x")
    goal_y = rospy.get_param("~goal_y")
    publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
    rospy.sleep(6.0)  # Let gmapping receive initial scans and publish /map.
    message = PoseStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "map"
    message.pose.position.x = goal_x
    message.pose.position.y = goal_y
    message.pose.orientation.w = 1.0
    publisher.publish(message)
    rospy.loginfo("Published fixed move_base goal: (%.2f, %.2f)", goal_x, goal_y)
    rospy.spin()


if __name__ == "__main__":
    main()
