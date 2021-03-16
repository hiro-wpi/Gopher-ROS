#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped

import time


class MoveRobot():

    def __init__(self):
        # variables
        
        # instantiate the node
        rospy.init_node('haptic_teleop_control')

        # subscribe to 


        # publish to the move_base
        self.move_base_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

    def start(self):
        time.sleep(20)

        first_goal = PoseStamped()
        first_goal.header.seq = 1
        first_goal.header.stamp = rospy.Time.now()
        first_goal.header.frame_id = "map"
        first_goal.pose.position.x = 6.0
        first_goal.pose.position.y = -8.0
        first_goal.pose.position.z = 0.0
        first_goal.pose.orientation.x = 0.0
        first_goal.pose.orientation.y = 0.0
        first_goal.pose.orientation.z = 0.8509035
        first_goal.pose.orientation.w = 0.525322
        self.move_base_pub.publish(first_goal)

        time.sleep(90)

        second_goal = PoseStamped()
        second_goal.header.seq = 2
        second_goal.header.stamp = rospy.Time.now()
        second_goal.header.frame_id = "map"
        second_goal.pose.position.x = -2.0
        second_goal.pose.position.y = 4.0
        second_goal.pose.position.z = 0.0
        second_goal.pose.orientation.x = 0.0
        second_goal.pose.orientation.y = 0.0
        second_goal.pose.orientation.z = -0.8509035
        second_goal.pose.orientation.w = 0.525322
        self.move_base_pub.publish(second_goal)


if __name__ == "__main__":
    move_robot = MoveRobot()
    move_robot.start()

    # rospy.spin()
    while not rospy.is_shutdown():
        time.sleep(1)
