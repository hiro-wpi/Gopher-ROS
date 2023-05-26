#!/usr/bin/env python
import sys
import moveit_commander
import rospy

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from gopher_moveit.srv import PlanTrajectory, PlanTrajectoryResponse

import geometry_msgs.msg


def plan_trajectory(req, group_name):
    """ Plan a trajectory for a given group """
    # Initialize request
    move_group = moveit_commander.MoveGroupCommander(group_name)
    start_joint_angles = req.joints_input
    target_pose = req.target_pose
    
    # Viz target pose
    '''
    scene = moveit_commander.PlanningSceneInterface()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "gopher/base_link"
    box_pose.pose = req.target_pose
    box_name = "target"
    scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.1))
    '''

    # Set the current joint angles
    current_joint_state = JointState()
    joint_names = move_group.get_active_joints()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    # Set the target pose
    move_group.set_pose_target(target_pose)

    # Joint space
    # Plan the trajectory
    plan = move_group.plan()

    # If no trajectory is valid, return empty response
    response = PlanTrajectoryResponse()
    if not plan[0]:
        return response
    # If trajectory is valid, set plan to response
    response.trajectory = plan[1]
    
    # Cartesian
    '''
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        [target_pose], 0.01, 0.0  # waypoints to follow  # eef_step
    )  # target_posejump_threshold
    
    # Set plan to response
    response = PlanTrajectoryResponse()
    response.trajectory = plan
    '''
    
    # debug
    # rospy.loginfo(target_pose)
    # rospy.loginfo(plan[1]);
    
    # Clear
    move_group.clear_pose_targets()

    return response


def plan_left_trajectory(req):
    return plan_trajectory(req, "left_arm")


def plan_right_trajectory(req):
    return plan_trajectory(req, "right_arm")


def moveit_server():
    # Init node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_trajectory_server')

    # Start service
    service_left = rospy.Service(
        'left_arm/plan_trajectory', PlanTrajectory, plan_left_trajectory
    )
    service_right = rospy.Service(
        'right_arm/plan_trajectory', PlanTrajectory, plan_right_trajectory
    )

    # Spin
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
