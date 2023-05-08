#! /usr/bin/env python


import rospy
import roslib
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse

# TODO Change the srv msg for GetPlan to custom srv
#   We just need to send in the goal, not the start location or the tolerance

def movebase_client(goalRequested):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    # fill in our goal msg

    goal.target_pose.header =   goalRequested.header
    goal.target_pose.pose =     goalRequested.pose
    
    # goal.target_pose.header.frame_id = 'map'
    # goal.target_pose.header.stamp = rospy.Time.now()

    # # Move forward 2 meter
    # goal.target_pose.pose.position.x = 6.0
    # goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    # client.cancel_goal()
    
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not avaiable!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
        # client.get_state()

def send_goal(req):
    
    goal = req.goal

    result = movebase_client(goal)
    if result:
        rospy.loginfo("Goal Execution Done")
    return GetPlanResponse()

if __name__ == "__main__":
    
    try:
        rospy.init_node("move_base_send_goal_node")
        rospy.loginfo("Initcialized: move_base_send_goal_node")
        s = rospy.Service("move_base/send_goal", GetPlan, send_goal)
        # result = movebase_client()
        # if result:
        #     rospy.loginfo("Goal Execution Done")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



    