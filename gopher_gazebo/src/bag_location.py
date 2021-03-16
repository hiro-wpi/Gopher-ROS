#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import time


class BagPose():

    def __init__(self):
        # variables
        
        # instantiate the node
        rospy.init_node('bag_pose_node')

        # subscribe to 
        # self.odom_sub = rospy.Subscriber('base_controller/odom', Odometry, self.getPose_callback)

        # publish to the move_base
        self.robot_pose_pub = rospy.Publisher('robot_pose', Pose, queue_size=1)

    def send_robot_odometry(self):
        robot_pose = Pose()

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            robot_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            robot_state_res = robot_state_service("gopher_1", "")
            robot_pose = robot_state_res.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        self.robot_pose_pub.publish(robot_pose)


if __name__ == "__main__":
    bag_pose = BagPose()

    # rospy.spin()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        bag_pose.send_robot_odometry()
        r.sleep()
