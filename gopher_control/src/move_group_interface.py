#!/usr/bin/env python

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String, Empty, Float64, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, Twist
from moveit_commander.conversions import pose_to_list


class MoveGroupPythonInteface(object):
    """MoveGroupPythonInteface"""
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface')

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). 
        ## This interface can be used to plan and execute motions:
        group_left_arm = "left_arm"
        group_right_arm = "right_arm"
        group_left_gripper = "left_gripper"
        group_right_gripper = "right_gripper"
        self.move_group_left_arm = moveit_commander.MoveGroupCommander(group_left_arm)
        self.move_group_right_arm = moveit_commander.MoveGroupCommander(group_right_arm)
        self.move_group_left_gripper = moveit_commander.MoveGroupCommander(group_left_gripper)
        self.move_group_right_gripper = moveit_commander.MoveGroupCommander(group_right_gripper)

        # Subscribe robot arm joint command
        self.left_joint_subscriber = rospy.Subscriber("set_left_joint", Float64MultiArray, self.set_left_joint_callback)
        self.right_joint_subscriber = rospy.Subscriber("set_right_joint", Float64MultiArray, self.set_right_joint_callback)
        # Subscribe robot end effector command
        self.left_eef_subscriber = rospy.Subscriber("set_left_eef", Float64MultiArray, self.set_left_eef_callback)
        self.right_eef_subscriber = rospy.Subscriber("set_right_eef", Float64MultiArray, self.set_right_eef_callback)
        # Subscribe robot arm pose command
        self.left_pose_subscriber = rospy.Subscriber("set_left_pose", Pose, self.set_left_pose_callback)
        self.right_pose_subscriber = rospy.Subscriber("set_right_pose", Pose, self.set_right_pose_callback)
        # Subscribe robot arm end effector velocity command
        self.left_vel_subscriber = rospy.Subscriber("set_left_vel", Twist, self.set_left_vel_callback)
        self.right_vel_subscriber = rospy.Subscriber("set_right_vel", Twist, self.set_right_vel_callback)

        self.home_robot()

    # Callback functions
    def set_left_joint_callback(self, data):
        self.set_joint(self.move_group_left_arm, data.data)

    def set_right_joint_callback(self, data):
        self.set_joint(self.move_group_right_arm, data.data)    

    def set_left_eef_callback(self, data):
        grip = data.data[0] * 0.799 # 0->45.8 degrees
        gripper_joints = [grip, -grip, grip, grip, grip, -grip]
        self.set_joint(self.move_group_left_gripper, gripper_joints)

    def set_right_eef_callback(self, data):
        grip = data.data[0] * 0.799 # 0->45.7 degrees
        gripper_joints = [grip, -grip, grip, grip, grip, -grip]
        self.set_joint(self.move_group_right_gripper, gripper_joints)

    def set_left_pose_callback(self, data):
        self.set_pose(self.move_group_left_arm, data)

    def set_right_pose_callback(self, data):
        self.set_pose(self.move_group_right_arm, data)

    def set_left_vel_callback(self, data):
        self.set_cartesian_vel(self.move_group_left_arm, data)

    def set_right_vel_callback(self, data):
        self.set_cartesian_vel(self.move_group_right_arm, data)

    # Set joint / gripper values
    def set_joint(self, move_group, joint_values, trials=1, wait=False):
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        for index in range(len(joint_goal)):
            joint_goal[index] = joint_values[index]

        trial = 0
        while True:
            # For testing:
            current_joints = move_group.get_current_joint_values()
            if all_close(joint_goal, current_joints, 0.05, angular=True):
                break
            
            # Check trial times
            trial += 1
            if trial > trials:
                rospy.loginfo("Moving arm to goal failed")
                break

            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            move_group.go(joint_goal, wait=wait)
        
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

    # Set joint pose
    def set_pose(self, move_group, pose_goal, trials=1, wait=False):
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the end-effector:
        trial = 0
        while True:
            # For testing:
            current_pose = move_group.get_current_pose().pose
            if all_close(pose_goal, current_pose, 0.05):
                break
            
            # Check trial times
            trial += 1
            if trial > trials:
                rospy.loginfo("Moving arm to pose failed")
                break

            move_group.set_pose_target(pose_goal)
            ## Now, we call the planner to compute the plan and execute it.
            plan = move_group.go(wait=wait)
        
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

    # Control end effector with velocity command
    def set_cartesian_vel(self, move_group, vel_command, scale=0.05, trials=1, wait=False):
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through.
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x += scale * vel_command.linear.y
        wpose.position.y += - scale * vel_command.linear.x
        wpose.position.z += scale * vel_command.linear.z
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        trial = 0
        while True:
            # Check trial times
            trial += 1
            if trial > trials:
                rospy.loginfo("Moving arm to pose failed")
                break

            move_group.execute(plan, wait=wait)

    # Default robot pose
    def home_robot(self):
        left_joint_goal = [-2.2, 1.57, 1.57, 1.57, 0.0, 0.0, 1.57]
        right_joint_goal = [-2.7, 1.5, 0.0, 1.43, 0.07, 1.71, 3.14]

        # Move the robot arm to home position # 10 trials
        self.set_joint(self.move_group_left_arm, left_joint_goal, trials=10, wait=True)
        self.set_joint(self.move_group_right_arm, right_joint_goal, trials=10, wait=True)

        print(self.move_group_right_arm.get_current_pose().pose)


def all_close(goal, actual, tolerance, angular=False):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if angular:
                if (abs(actual[index] - goal[index]) > tolerance) and \
                (abs(abs(actual[index]-goal[index])-2*pi) > tolerance) :
                    return False
            else:
                if (abs(actual[index] - goal[index]) > tolerance):
                    return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance, angular)

    elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance, angular)

    return True


def main():
    try:
        # Home robot arm
        interface = MoveGroupPythonInteface()
        
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
